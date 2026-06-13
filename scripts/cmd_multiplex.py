import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

TIMECOMPARE = 1.0e9  # 1 second in nanoseconds


class CmdVelMultiplex(Node):

    def __init__(self):
        super().__init__('cmd_vel_multiplex')

        # Commands
        self.move_cmd = None
        self.rotate_cmd = None
        self.vel_cmd = None
        self.brake_cmd = None
        self.sick_cmd = None

        # Timestamps
        self.move_stamp = 0
        self.rotate_stamp = 0
        self.vel_stamp = 0
        self.brake_stamp = 0
        self.sick_stamp = 0

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # Publisher
        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile
        )

        # Subscribers
        self.create_subscription(
            Twist,
            '/move_server/cmd_vel',
            self.move_callback,
            qos_profile
        )

        self.create_subscription(
            Twist,
            '/rotate_server/cmd_vel',
            self.rotate_callback,
            qos_profile
        )

        self.create_subscription(
            Twist,
            '/vel_pub/cmd_vel',
            self.vel_callback,
            qos_profile
        )

        self.create_subscription(
            Twist,
            '/brake/cmd_vel',
            self.brake_callback,
            qos_profile
        )

        self.create_subscription(
            Twist,
            '/sick/cmd_vel',
            self.sick_callback,
            qos_profile
        )

        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('cmd_vel_mux started')

    # ---------------- Callbacks ----------------

    def move_callback(self, msg):
        self.move_cmd = msg
        self.move_stamp = self.get_clock().now().nanoseconds

    def rotate_callback(self, msg):
        self.rotate_cmd = msg
        self.rotate_stamp = self.get_clock().now().nanoseconds

    def vel_callback(self, msg):
        self.vel_cmd = msg
        self.vel_stamp = self.get_clock().now().nanoseconds

    def brake_callback(self, msg):
        self.brake_cmd = msg
        self.brake_stamp = self.get_clock().now().nanoseconds

    def sick_callback(self, msg):
        self.sick_cmd = msg
        self.sick_stamp = self.get_clock().now().nanoseconds

    # ---------------- Timer ----------------

    def timer_callback(self):

        now = self.get_clock().now().nanoseconds

        active_cmds = []

        if self.move_cmd is not None and (now - self.move_stamp) < TIMECOMPARE:
            active_cmds.append(("move", self.move_stamp))

        if self.rotate_cmd is not None and (now - self.rotate_stamp) < TIMECOMPARE:
            active_cmds.append(("rotate", self.rotate_stamp))

        if self.vel_cmd is not None and (now - self.vel_stamp) < TIMECOMPARE:
            active_cmds.append(("vel", self.vel_stamp))

        if self.brake_cmd is not None and (now - self.brake_stamp) < TIMECOMPARE:
            active_cmds.append(("brake", self.brake_stamp))

        if self.sick_cmd is not None and (now - self.sick_stamp) < TIMECOMPARE:
            active_cmds.append(("sick", self.sick_stamp))

        msg = Twist()

        if active_cmds:

            # Select most recent active command
            latest_name, _ = max(active_cmds, key=lambda x: x[1])

            if latest_name == "move":
                msg.linear.x = self.move_cmd.linear.x
                msg.linear.y = self.move_cmd.linear.y
                self.get_logger().debug("MOVE active")

            elif latest_name == "rotate":
                msg.angular.z = self.rotate_cmd.angular.z
                self.get_logger().debug("ROTATE active")

            elif latest_name == "vel":
                msg.linear.x = self.vel_cmd.linear.x
                msg.linear.y = self.vel_cmd.linear.y
                self.get_logger().debug("VEL active")

            elif latest_name == "brake":
                msg.linear.x = self.brake_cmd.linear.x
                msg.linear.y = self.brake_cmd.linear.y
                msg.angular.z = self.brake_cmd.angular.z
                self.get_logger().debug("BRAKE active")

            elif latest_name == "sick":
                msg.linear.x = self.sick_cmd.linear.x
                msg.linear.y = self.sick_cmd.linear.y
                msg.angular.z = self.sick_cmd.angular.z
                self.get_logger().debug("SICK active")

        # Otherwise publish zeros (stop)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = CmdVelMultiplex()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()