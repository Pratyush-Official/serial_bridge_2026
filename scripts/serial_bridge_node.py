import rclpy
import struct
import csv
import os
import time
from rclpy.node import Node
from serial_bridge_2026.serial_comms import serial_comms
from std_msgs.msg import Float32MultiArray, UInt16MultiArray, Int32MultiArray, Byte, Float32
from geometry_msgs.msg import Twist
from serial_bridge_2026.crc8 import crc8
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# rx_port = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_54A7002607-if00'
# tx_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
serial_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'

START_BYTE = 0xA5

bluepill_tx_size = 1 + 21 + 1   
# nucleo_rx_size = 1 + 4  + 1   

bluepill_rx_size = 1 + 29 + 1

# --- Hardcoded test parameters (only used when HARDCODED_MODE = True) ---
HARDCODED_MODE  = False   # set False to revert to topic-based control
HARDCODED_VX    =  0.1
HARDCODED_VY    =  0.1
HARDCODED_OMEGA =  0.0
FLIP_DELAY_S    =  10.0   # seconds before vx sign flips
# -----------------------------------------------------------------------

class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__('serial_bridge_node')

        self.get_logger().info("SerialBridgeNode initialized")
       
        # self.serial_nucleo = serial_comms(
        #     serial_port=tx_port,
        #     serial_baudrate=115200,
        #     tx_data_size=nucleo_tx_size,
        #     rx_data_size=nucleo_rx_size,
        #     hash_func_="CRC",
        # )

        self.serial_bluepill = serial_comms(
            serial_port=serial_port,
            serial_baudrate=115200,
            tx_data_size=bluepill_tx_size,  
            rx_data_size=bluepill_rx_size,
            hash_func_="CRC",
        )

        qos_profile_cmd = QoSProfile(depth=1)
        qos_profile_cmd.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile_cmd.durability = QoSDurabilityPolicy.VOLATILE

        # qos_profile_sensor = QoSProfile(depth=10)
        # qos_profile_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT
        # qos_profile_sensor.durability = QoSDurabilityPolicy.VOLATILE

        # qos_profile_sensor_imu = QoSProfile(depth=1)
        # qos_profile_sensor_imu.reliability = QoSReliabilityPolicy.BEST_EFFORT
        # qos_profile_sensor_imu.durability = QoSDurabilityPolicy.VOLATILE
        
        qos_profile = QoSProfile(depth=10)

        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.yaw_pub        = self.create_publisher(Float32, '/imu_data_received', qos_profile)
        self.dead_pub       = self.create_publisher(Int32MultiArray,   '/odom_data_received', qos_profile)
        self.sick_pub       = self.create_publisher(Float32MultiArray,  '/sick_data', qos_profile)
        self.prox_front_pub = self.create_publisher(Byte,              '/prox_front_received', qos_profile)
        self.prox_rear_pub  = self.create_publisher(Byte,              '/prox_rear_received', qos_profile)
        self.yaw_setpoint_sub = self.create_subscription(Float32MultiArray, '/setpoint_yaw', self.yaw_setpoint_callback, qos_profile_cmd)

        self.twist_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            qos_profile_cmd
        )

        self.twist_msg = None
        self.setpoint_msg = Float32MultiArray()
        self.setpoint_msg.data = [0.0, 0.0]
        self.setpoint_msg.data[1] = 0x42

        # --- CSV logger setup ---
        log_dir = os.path.expanduser('~/tmp1')
        os.makedirs(log_dir, exist_ok=True)
        csv_path = os.path.join(log_dir, f'serial_bridge_odom_{int(time.time() * 1000)}.csv')
        self._csv_file = open(csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(['timestamp_ms', 'dw1', 'dw2'])
        self._csv_file.flush()
        self.get_logger().info(f'[SerialBridge] Odom CSV logging to: {csv_path}')
        # --- end CSV logger setup ---

        self._start_time = time.time()   # used only in HARDCODED_MODE
        self._flipped     = False         # tracks whether flip has been logged

        self.timer = self.create_timer(0.001, self.read_write_serial_data)

    def twist_callback(self, msg):
        self.twist_msg = msg

    def write_bluepill(self):
        if HARDCODED_MODE:
            # --- hardcoded velocity block ---------------------------------
            elapsed = time.time() - self._start_time
            vx      = HARDCODED_VX if elapsed < FLIP_DELAY_S else -HARDCODED_VX
            vy      = HARDCODED_VY
            omega   = HARDCODED_OMEGA

            if elapsed >= FLIP_DELAY_S and not self._flipped:
                self.get_logger().info(f'[SerialBridge] Flipping vx sign after {elapsed:.1f}s')
                self._flipped = True
            # --------------------------------------------------------------
        else:
            # --- topic-based block (original) -----------------------------
            if self.twist_msg is None:
                return
            vx    = self.twist_msg.linear.x
            vy    = self.twist_msg.linear.y
            omega = self.twist_msg.angular.z
            # --------------------------------------------------------------

        payload = b"".join([
            struct.pack("f", vx),
            struct.pack("f", vy),
            struct.pack("f", omega),
            struct.pack("f", self.setpoint_msg.data[0]),       
            struct.pack("B", int(self.setpoint_msg.data[1]) & 0xFF)
        ])

        crc = self.serial_bluepill.calc_crc(payload)
        frame = b"".join([
            struct.pack("B", START_BYTE),
            payload,
            struct.pack("B", crc),
        ])
        self.serial_bluepill.write_data(frame)
        # print(frame)
        # print(f"sent: vx={vx:.3f}, vy={vy:.3f}, omega={omega:.3f}")

    # def read_nucleo(self):
    #     _data = self.serial_nucleo.read_data()
    #     if _data is None:
            
    #         return

    #     payload = _data[:-1]  
    #     if len(payload) != 4:
    #         print(f"Nucleo bad packet: expected 4 bytes, got {len(payload)}")
    #         return

    #     yaw, = struct.unpack("f", payload)
    #     print(f"yaw: {yaw:.3f}")

    #     yaw_msg = Float32()
    #     yaw_msg.data = yaw
    #     self.yaw_pub.publish(yaw_msg)

    def read_bluepill(self):
        _data = self.serial_bluepill.read_data()
        if _data is None:
            # print("No data received from Bluepill")
            return
        payload = _data[:-1]  

        if len(payload) != 29:
            print(f"Bluepill bad packet: expected 21 bytes, got {len(payload)}")
            return

        dw1, dw2, s1, s2, s3, s4, prox, yaw, pitch, roll = struct.unpack("<iiHHHHBfff", payload)

        front_prox1 = prox & 0b00000001
        front_prox2 = prox & 0b00000010
        rear_prox1  = prox & 0b00000100
        rear_prox2  = prox & 0b00001000
        front_prox = 1 if (front_prox1 or front_prox2) else 0
        rear_prox  = 1 if (rear_prox1  or rear_prox2)  else 0

        print(f"received: dw1={dw1}, dw2={dw2}, s1={s1}, s2={s2}, s3={s3}, s4={s4}, front={front_prox}, rear={rear_prox}, yaw={yaw:.3f}, raw_prox:{prox}")

        received_dead = Int32MultiArray()
        received_dead.data = [dw1, dw2]

        received_sick = UInt16MultiArray()
        received_sick.data = [s1, s2, s3, s4]
        self.process_sick(received_sick.data)

        received_front_prox = Byte()
        received_front_prox.data = bytes([front_prox & 0xFF])
        received_rear_prox = Byte()
        received_rear_prox.data = bytes([rear_prox & 0xFF])

        yaw_msg = Float32()
        yaw_msg.data = yaw

        self.dead_pub.publish(received_dead)
        self.prox_front_pub.publish(received_front_prox)
        self.prox_rear_pub.publish(received_rear_prox)
        self.yaw_pub.publish(yaw_msg)

        # --- CSV log raw odom ---
        now_ms = int(time.time() * 1000)
        self._csv_writer.writerow([now_ms, dw1, dw2])
        self._csv_file.flush()
        # --- end CSV log ---

    # def read_bluepill(self):
    #     _data = self.serial_bluepill.read_data()
    #     if _data is None:
    #         return

    #     payload = _data[:-1]  
    #     # print(f"raw payload: {payload.hex()}")
    #     if len(payload) != 17:
    #         print(f"Bluepill bad packet: expected 17 bytes, got {len(payload)}")
    #         return

    #     dw1, dw2, s1, s2, s3, s4, prox = struct.unpack("iiHHHHB", payload)

    #     front_prox1 = prox & 0b00000001
    #     front_prox2 = prox & 0b00000010
    #     rear_prox1  = prox & 0b00000100
    #     rear_prox2  = prox & 0b00001000

    #     front_prox = 1 if (front_prox1 and front_prox2) else 0
    #     rear_prox  = 1 if (rear_prox1  or rear_prox2)  else 0

    #     print(f"received: dw1={dw1}, dw2={dw2}, s1={s1}, s2={s2}, s3={s3}, s4={s4}, front={front_prox}, rear={rear_prox}")

    #     received_dead = Int32MultiArray()
    #     received_dead.data = [dw1, dw2]

    #     received_sick = UInt16MultiArray() #Its uint16 right now but needs to be changed to float later during publish
    #     received_sick.data = [s1, s2, s3, s4]
    #     self.process_sick(received_sick.data)

    #     # print(f"Sick data: {s1}, {s2}, {s3}, {s4}")

    #     received_front_prox = Byte()
    #     received_front_prox.data = bytes([front_prox & 0xFF])

    #     received_rear_prox = Byte()
    #     received_rear_prox.data = bytes([rear_prox & 0xFF])

    #     self.dead_pub.publish(received_dead)
    #     # self.sick_pub.publish(received_sick) # Sick data is published in function
    #     self.prox_front_pub.publish(received_front_prox)
    #     self.prox_rear_pub.publish(received_rear_prox)

    def read_write_serial_data(self):
        self.write_bluepill()
        # self.read_nucleo()
        self.read_bluepill()

    def yaw_setpoint_callback(self, msg: Float32MultiArray):
        self.setpoint_msg = msg

    def process_sick(self, data):
        sick_msg = Float32MultiArray()

        data_tuned_0 =  0.99 * data[0] 
        data_tuned_1 =  0.99 * data[1] 
        data_tuned_2 =  0.99 * data[2] 
        data_tuned_3 =  0.99 * data[3] 
        sick_msg.data = [data_tuned_0, data_tuned_1, data_tuned_2, data_tuned_3]

        self.sick_pub.publish(sick_msg)
        # print(f"SICK data published: {sick_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # node.serial_nucleo.close()
        node.serial_bluepill.close()
        node._csv_file.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
