import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class ButlerAllInOne(Node):
    def __init__(self):
        super().__init__('butler_all_in_one')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(String, 'confirmation', self.confirmation_callback, 10)
        self.timer = self.create_timer(1.0, self.task_loop)

        self.order_queue = []
        self.task_queue = []
        self.current_table = None
        self.state = 'IDLE'
        self.step = 0
        self.timeout_steps = 10
        self.confirmation = ''

    def confirmation_callback(self, msg):
        incoming = msg.data.strip().lower()
        self.get_logger().info(f"📩 Confirmation received: {incoming}")
        shortcut_map = {
            'k': 'kitchen',
            't1': 'table1',
            't2': 'table2',
            't3': 'table3'
        }
        self.confirmation = shortcut_map.get(incoming, incoming)

    def get_user_input(self):
        table_input = input("Enter table (t1/t2/t3): ").strip().lower().split()
        shortcut_map = {
            't1': 'table1',
            't2': 'table2',
            't3': 'table3'
        }
        for table in table_input:
            full_name = shortcut_map.get(table, table)
            if full_name in ['table1', 'table2', 'table3']:
                self.order_queue.append(full_name)
                self.get_logger().info(f"📝 Order received for {full_name}")
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
                if self.order_queue:
                    self.task_queue = self.order_queue.copy()
                    self.state = 'GO_TO_KITCHEN'
                    self.step = 0
                return

        elif self.state == 'GO_TO_KITCHEN':
            self.publish_drive_command(linear=0.1)
            self.get_logger().info("🔪 Moving to kitchen...")
            self.step += 1
            if self.step > 5:
                self.publish_drive_command(0.0)
                self.state = 'WAIT_KITCHEN_CONFIRM'
                self.confirmation = ''
                self.step = 0

        elif self.state == 'WAIT_KITCHEN_CONFIRM':
            self.get_logger().info("📦 Waiting for kitchen confirmation...")
            if self.confirmation == 'kitchen':
                self.get_logger().info("✅ Kitchen confirmed.")
                self.state = 'NEXT_TABLE'
                self.step = 0
            elif self.step > self.timeout_steps:
                self.get_logger().warn("⏰ Kitchen confirmation timeout. Returning home.")
                self.state = 'RETURN_HOME'
                self.step = 0
            else:
                self.step += 1

        elif self.state == 'NEXT_TABLE':
            if self.task_queue:
                self.current_table = self.task_queue.pop(0)
                self.get_logger().info(f"🍽 Delivering to {self.current_table}...")
                self.state = 'GO_TO_TABLE'
                self.step = 0
            else:
                self.state = 'GO_TO_KITCHEN_AGAIN'
                self.step = 0

        elif self.state == 'GO_TO_TABLE':
            self.publish_drive_command(linear=0.1)
            self.step += 1
            if self.step > 5:
                self.publish_drive_command(0.0)
                self.state = 'WAIT_TABLE_CONFIRM'
                self.confirmation = ''
                self.step = 0

        elif self.state == 'WAIT_TABLE_CONFIRM':
            self.get_logger().info(f"🍽 Waiting for confirmation at {self.current_table}...")
            if self.confirmation == self.current_table:
                self.get_logger().info(f"✅ {self.current_table} confirmed.")
                self.state = 'NEXT_TABLE'
                self.step = 0
            elif self.confirmation == 'cancel':
                self.get_logger().warn(f"❌ Skipping {self.current_table} as canceled.")
                self.state = 'NEXT_TABLE'
                self.step = 0
            elif self.step > self.timeout_steps:
                self.get_logger().warn(f"⏰ No confirmation at {self.current_table}. Skipping.")
                self.state = 'NEXT_TABLE'
                self.step = 0
            else:
                self.step += 1

        elif self.state == 'GO_TO_KITCHEN_AGAIN':
            self.publish_drive_command(linear=0.1)
            self.get_logger().info("🔁 Returning to kitchen after deliveries...")
            self.step += 1
            if self.step > 5:
                self.publish_drive_command(0.0)
                self.state = 'RETURN_HOME'
                self.step = 0

        elif self.state == 'RETURN_HOME':
            self.publish_drive_command(linear=-0.1)
            self.get_logger().info("🏠 Returning to home...")
            self.step += 1
            if self.step > 5:
                self.publish_drive_command(0.0)
                self.get_logger().info("✅ All tasks completed.")
                self.state = 'IDLE'
                self.step = 0
                self.order_queue.clear()
                self.task_queue.clear()
                self.confirmation = ''
                self.current_table = None

def main(args=None):
    rclpy.init(args=args)
    node = ButlerAllInOne()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
