import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from art_gripper_interfaces.msg import GripperControl, GripperStatus
from art_gripper_interfaces.srv import MotorOn

class GripperClient(Node):
    def __init__(self):
        super().__init__('gripper_client_py')
        self.publisher = self.create_publisher(String, 'greeting', 10)
        self.motor_on_client = self.create_client(MotorOn, 'MotorOn')
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello Python: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'pub: {msg.data}')
        self.count += 1

        # if self.count % 2 == 0:
        #     self.call_moount % 2 == 0:
        #     self.call_motor_on(True)
        # else:
        #     self.tor_on(True)
        # else:
        #     self.call_motor_on(False)

    def call_motor_on(self, on):
        while not self.motor_on_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = MotorOn.Request()
        request.on = on
        self.future = self.motor_on_client.call_async(request)
        self.future.add_done_callback(self.motor_on_callback)

    def motor_on_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'service call result: {response.result}')
        except Exception as e:
            self.get_logger().error(f'service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GripperClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    