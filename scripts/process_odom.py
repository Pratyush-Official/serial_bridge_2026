# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32MultiArray, Float32MultiArray
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# import math

# WHEEL_DIAMETER_M = 0.0574
# COUNTS_PER_REV   = 4000
# DIST_PER_COUNT   = (math.pi * WHEEL_DIAMETER_M) / COUNTS_PER_REV


# class ProcessOdom(Node):
#     def __init__(self):
#         super().__init__('process_odom')

#         qos_profile_sensor = QoSProfile(depth=10)
#         qos_profile_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT
#         qos_profile_sensor.durability = QoSDurabilityPolicy.VOLATILE

#         qos_profile = QoSProfile(depth=10)


#         self._pub = self.create_publisher(Float32MultiArray, '/odom_dist_m', qos_profile)
#         self._sub = self.create_subscription(
#             Int32MultiArray,
#             '/odom_data_received',
#             self.odom_callback,
#             qos_profile_sensor,
#         )

#         self.get_logger().info(
#             f'Ready. dist_per_count={DIST_PER_COUNT * 1000:.6f} mm/count'
#         )

#     def odom_callback(self, msg: Int32MultiArray):
#         if len(msg.data) < 2:
#             self.get_logger().warn(f'Expected 2 values, got {len(msg.data)}')
#             return

        
#         dist_x = float(-msg.data[0]) * DIST_PER_COUNT   # dw2 → X meters
#         dist_y = float(-msg.data[1]) * DIST_PER_COUNT   # dw1 → Y meters

#         print(msg.data[0], msg.data[1])
#         out = Float32MultiArray()
#         out.data = [dist_x, dist_y]
#         self._pub.publish(out)


# def main(args=None):
#     rclpy.init(args=args)
#     node = ProcessOdom()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import math

WHEEL_DIAMETER_M = 0.0574
COUNTS_PER_REV   = 4000
DIST_PER_COUNT   = (math.pi * WHEEL_DIAMETER_M) / COUNTS_PER_REV


class ProcessOdom(Node):
    def __init__(self):
        super().__init__('process_odom')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        qos_profile_sensor = QoSProfile(depth=10)
        qos_profile_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile_sensor.durability = QoSDurabilityPolicy.VOLATILE
        qos_profile = QoSProfile(depth=10)

        # --- existing publishers ---
        self._pub = self.create_publisher(Float32MultiArray, '/odom_dist_m', qos_profile_sensor)

        # --- new velocity publisher ---
        self._vel_pub = self.create_publisher(Float32MultiArray, '/robot_vel', qos_profile_sensor)

        # --- existing subscription (unchanged) ---
        self._sub = self.create_subscription(
            Int32MultiArray,
            '/odom_data_received',
            self.odom_callback,
            qos_profile_sensor,
        )

        # --- velocity estimation state ---
        self._prev_count_x = None   # will be set on first callback
        self._prev_count_y = None
        self._cur_count_x  = 0
        self._cur_count_y  = 0

        # --- 20ms timer ONLY for velocity publishing, nothing else touched ---
        self._vel_timer = self.create_timer(0.02, self._publish_velocity)

        self.get_logger().info(
            f'Ready. dist_per_count={DIST_PER_COUNT * 1000:.6f} mm/count'
        )

    # ---- existing callback, unchanged except storing raw counts ----
    def odom_callback(self, msg: Int32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn(f'Expected 2 values, got {len(msg.data)}')
            return

        dist_x = float(-msg.data[0]) * DIST_PER_COUNT
        dist_y = float( msg.data[1]) * DIST_PER_COUNT

        print(msg.data[0], msg.data[1])

        out = Float32MultiArray()
        out.data = [dist_x, dist_y]
        self._pub.publish(out)

        # store latest raw counts for velocity timer
        # note: dist_x uses -dw2, dist_y uses +dw1, mirror that sign here
        self._cur_count_x = -msg.data[0]
        self._cur_count_y =  msg.data[1]

    # ---- new: called every 20 ms only ----
    def _publish_velocity(self):
        # wait until we have at least one odom reading
        if self._prev_count_x is None:
            self._prev_count_x = self._cur_count_x
            self._prev_count_y = self._cur_count_y
            return

        dt = 0.02   # seconds

        delta_x = self._cur_count_x - self._prev_count_x
        delta_y = self._cur_count_y - self._prev_count_y

        vel_x = (delta_x * DIST_PER_COUNT) / dt   # m/s
        vel_y = (delta_y * DIST_PER_COUNT) / dt   # m/s

        self._prev_count_x = self._cur_count_x
        self._prev_count_y = self._cur_count_y

        vel_msg = Float32MultiArray()
        vel_msg.data = [vel_x, vel_y]
        self._vel_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ProcessOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()