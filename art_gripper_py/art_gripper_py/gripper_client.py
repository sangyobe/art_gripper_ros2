import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from art_gripper_interfaces.msg import GripperControl, GripperStatus

class GripperClient(Node):
    def __init__(self):
        super().__init__('gripper_client_py')
        self.publisher = self.create_publisher(String, 'greeting', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0
        
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello Python: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'pub: {msg.data}')
        self.count += 1
        
def main(args=None):
    rclpy.init(args=args)
    node = GripperClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    