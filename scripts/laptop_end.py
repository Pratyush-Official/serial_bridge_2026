import math
import struct
import time
from math import cos, sin

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Float64MultiArray, UInt8MultiArray
#from ros2_bno_msg.msg import BnoData
from serial_bridge_2026.crc8 import crc8
from serial_bridge_2026.serial_comms import serial_comms
from geometry_msgs.msg import TransformStamped
import tf2_ros

class LaptopEnd(Node):


    qos_profile = QoSProfile(depth=10)
    qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

    def __init__(self):#constructor
        super().__init__("LaptopEnd_node")
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.imu_subscriber = self.create_subscription(Imu, "imu_data", self.interpret_imu, qos_profile=self.qos_profile)
        self.imu_subscriber = self.create_subscription(Imu, "imu_data", self.publish_tf, qos_profile=self.qos_profile)
    
    def interpret_imu(self, msg: Imu):
        q = [
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        ]
        roll, pitch, yaw = self.quaternion_to_rollpitchyaw(q)
        
        self.get_logger().info(
            f"Roll: {roll}, Pitch: {pitch}, Yaw: {math.degrees(yaw):.2f}"
        )

    def quaternion_to_rollpitchyaw(self, q):
        w, x, y, z = q

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)   # clamp
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
  
    def publish_tf(self, msg: Imu):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'      # Parent frame
        t.child_frame_id = 'imu_link'        # Child frame

        q = [
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        ]

        qw, qx, qy, qz = q

        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        t.transform.translation.x = 0.0  # Optional: set if IMU has offset
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        self.tf_broadcaster.sendTransform(t)


    
def main(args=None):
    rclpy.init()
    node = LaptopEnd()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()
    exit()


if __name__ == "__main__":
    main()