import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ConfirmationTester(Node):
    def __init__(self):
        super().__init__('confirmation_tester')
        self.publisher = self.create_publisher(String, '/confirmation', 10)
        self.get_logger().info("✅ ConfirmationTester ready. Type 'k', 't1', 't2', 't3', or 'c' (for cancel).")
        self.run_input_loop()

    def run_input_loop(self):
        while rclpy.ok():
            user_input = input("👉 Enter confirmation (k/t1/t2/t3/c): ").strip().lower()
            data_map = {
                'k': 'kitchen',
                't1': 'table1',
                't2': 'table2',
                't3': 'table3',
                'c': 'cancel'
            }
            msg = String()
            msg.data = data_map.get(user_input, user_input)

            self.publisher.publish(msg)
            self.get_logger().info(f"📤 Published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ConfirmationTester()
    try:
        rclpy.spin(node)  # Keep the node alive in case needed
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
