import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


TIMECOMPARE = 1.0e9  # 1.0 second in nanoseconds


class CmdVelMultiplex(Node):
    def __init__(self):
        super().__init__('cmd_vel_multiplex')

        self.move_cmd   = None
        self.rotate_cmd = None
        self.vel_cmd = None
        self.sick_cmd = None

        self.move_stamp   = 0
        self.rotate_stamp = 0
        self.vel_stamp = 0
        self.sick_stamp = 0

        qos_profile = QoSProfile(depth=10)

        qos_profile_cmd = QoSProfile(depth=1)
        qos_profile_cmd.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile_cmd.durability = QoSDurabilityPolicy.VOLATILE

        self.pub = self.create_publisher(Twist, '/cmd_vel',qos_profile_cmd)

        self.create_subscription(Twist, '/move_server/cmd_vel',   self.move_callback, qos_profile_cmd)
        self.create_subscription(Twist, '/rotate_server/cmd_vel', self.rotate_callback, qos_profile_cmd)
        self.create_subscription(Twist, '/vel_pub/cmd_vel',   self.vel_callback, 10)
        self.create_subscription(Twist, '/sick_server/cmd_vel',   self.sick_callback, qos_profile_cmd)

        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('cmd_vel_mux started')

    def move_callback(self, msg):
        self.move_cmd   = msg
        self.move_stamp = self.get_clock().now().nanoseconds

    def rotate_callback(self, msg):
        self.rotate_cmd   = msg
        self.rotate_stamp = self.get_clock().now().nanoseconds

    def vel_callback(self, msg):
        
        self.vel_cmd   = msg
        # print(self.vel_cmd)
        # print("he, im in callback")
        self.vel_stamp = self.get_clock().now().nanoseconds
    
    def sick_callback(self, msg):
        self.sick_cmd = msg
        self.sick_stamp = self.get_clock().now().nanoseconds


    def timer_callback(self):
        now = self.get_clock().now().nanoseconds

        move_active   = (self.move_cmd   is not None and
                         (now - self.move_stamp)   < TIMECOMPARE)
        rotate_active = (self.rotate_cmd is not None and
                         (now - self.rotate_stamp) < TIMECOMPARE)

        vel_active   = (self.vel_cmd   is not None and
                         (now - self.vel_stamp)   < TIMECOMPARE)
        
        sick_active = (self.sick_cmd   is not None and
                         (now - self.vel_stamp)   < TIMECOMPARE)
        vx    = 0.0
        vy    = 0.0
        omega = 0.0

        if move_active and rotate_active and vel_active and sick_active:
            # Most-recent command wins
            if self.move_stamp >= self.rotate_stamp and  self.move_stamp >= self.vel_stamp and self.move_stamp >= self.sick_stamp:
                vx = self.move_cmd.linear.x
                vy = self.move_cmd.linear.y

            elif self.sick_stamp >= self.move_stamp and self.sick_stamp >= self.rotate_stamp and  self.sick_stamp >= self.vel_stamp:
                vx = self.move_cmd.linear.x
                vy = self.move_cmd.linear.y

            elif self.vel_stamp >= self.rotate_stamp and  self.move_stamp <= self.vel_stamp and self.vel_stamp >=self.sick_stamp:
                vx = self.vel_cmd.linear.x
                vy = self.vel_cmd.linear.y
            else:
                omega = self.rotate_cmd.angular.z

        elif move_active:
            vx = self.move_cmd.linear.x
            vy = self.move_cmd.linear.y

        elif sick_active:
            vx = self.move_cmd.linear.x
            vy = self.move_cmd.linear.y

        elif rotate_active:
            omega = self.rotate_cmd.angular.z
        
        elif vel_active:
            vx = self.vel_cmd.linear.x
            vy = self.vel_cmd.linear.y
        
        else :
            print(self.vel_cmd   is not None)
            print(self.vel_cmd)
        # If neither is active we publish zeros — that is correct stop behaviour.

        msg = Twist()
        msg.linear.x  = vx
        msg.linear.y  = vy
        msg.angular.z = omega
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