import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class ButlerAllInOne(Node):
    def __init__(self):
        super().__init__('butler_all_in_one')

        # Publisher to simulate driving
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for FSM loop
        self.timer = self.create_timer(1.0, self.task_loop)

        # Internal state
        self.order_queue = []
        self.current_table = None
        self.state = 'IDLE'
        self.step = 0

    def get_user_input(self):
        table = input("Enter table (table1/table2/table3): ").strip().lower()
        if table in ['table1', 'table2', 'table3']:
            self.order_queue.append(table)
            self.get_logger().info(f"📝 Order received for {table}")
        else:
            self.get_logger().warn(f"⚠️ Invalid input: {table}")

    def publish_drive_command(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    def task_loop(self):
        if self.state == 'IDLE':
            if not self.order_queue:
                self.get_user_input()
                return
            else:
                self.current_table = self.order_queue.pop(0)
                self.get_logger().info(f"🍽 Starting task for {self.current_table}")
                self.state = 'GO_TO_KITCHEN'
                self.step = 0

        elif self.state == 'GO_TO_KITCHEN':
            self.publish_drive_command(linear=0.1)
            self.get_logger().info("🔪 Moving to kitchen...")
            self.step += 1
            if self.step > 5:
                self.state = 'GO_TO_TABLE'
                self.step = 0

        elif self.state == 'GO_TO_TABLE':
            self.publish_drive_command(linear=0.1)
            self.get_logger().info(f"🚶 Delivering to {self.current_table}...")
            self.step += 1
            if self.step > 5:
                self.state = 'RETURN_HOME'
                self.step = 0

        elif self.state == 'RETURN_HOME':
            self.publish_drive_command(linear=-0.1)
            self.get_logger().info("🏠 Returning to home position...")
            self.step += 1
            if self.step > 5:
                self.get_logger().info("✅ Task completed.")
                self.state = 'IDLE'
                self.step = 0


def main(args=None):
    rclpy.init(args=args)
    node = ButlerAllInOne()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

