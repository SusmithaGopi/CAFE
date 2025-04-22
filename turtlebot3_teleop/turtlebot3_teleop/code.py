import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtleBotMover(Node):
    def __init__(self):
        super().__init__('bot_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.move_forward)

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.1  # Move forward
        msg.angular.z = 0.0  # No rotation
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: Linear X: %0.2f' % msg.linear.x)


def main(args=None):
    rclpy.init(args=args)
    bot_controller = TurtleBotMover()
    rclpy.spin(bot_controller)

    bot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

