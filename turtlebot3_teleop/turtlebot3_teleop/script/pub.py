import rclpy
from std_msgs.msg import String
from rclpy.node import Node

class CancelTaskPublisher(Node):
    def __init__(self):
        super().__init__('cancel_task_publisher')
        self.publisher = self.create_publisher(String, '/cancel_task', 10)
        
        # Create a cancel message
        self.cancel_msg = String()
        self.cancel_msg.data = 'cancel'

        # Publish the cancel message once
        self.publish_cancel_task()

    def publish_cancel_task(self):
        self.get_logger().info('Publishing cancel task message...')
        self.publisher.publish(self.cancel_msg)
        self.get_logger().info(f'Published: {self.cancel_msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = CancelTaskPublisher()
    rclpy.spin_once(node)  # Spin once to ensure the message gets published
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

