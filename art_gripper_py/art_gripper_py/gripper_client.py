import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from art_gripper_interfaces.msg import GripperControl, GripperStatus, GripperInfo
from art_gripper_interfaces.srv import MotorOn, ResetAbsEncoder, SetTargetWidth, SetTargetPose, SetTargetCurrent, SetTarget, ResetFrictionModel, GetGripperInfo

class GripperClient(Node):
    def __init__(self):
        super().__init__('gripper_client_py')
        self.gripper_control_publisher = self.create_publisher(GripperControl, 'GripperControl', 10)
        self.motor_on_client = self.create_client(MotorOn, 'MotorOn')
        self.reset_abs_encoder_client = self.create_client(ResetAbsEncoder, 'ResetAbsEncoder')
        self.set_target_width_client = self.create_client(SetTargetWidth, 'SetTargetWidth')
        self.set_target_pose_client = self.create_client(SetTargetPose, 'SetTargetPose')
        self.set_target_current_client = self.create_client(SetTargetCurrent, 'SetTargetCurrent')
        self.set_target_client = self.create_client(SetTarget, 'SetTarget')
        self.reset_friction_model_client = self.create_client(ResetFrictionModel, 'ResetFrictionModel')
        self.get_gripper_info_client = self.create_client(GetGripperInfo, 'GetGripperInfo')

        # for periodic action
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.working_count = 0

    def timer_callback(self):
        if self.working_count % 2 == 0:
            self.call_motor_on(True)
        else:
            self.call_motor_on(False)

        if self.working_count % 5 == 0:
            self.call_reset_abs_encoder()
            self.call_set_target_width(50, 100, 50, 80)
            self.call_set_target_pose(90, 120)
            self.call_set_target_current([100, 200, 300, 400])
            self.call_set_target(25, 45, 75, 90, 20, 60)
            self.call_reset_friction_model()
            self.call_get_gripper_info()

        self.publish_gripper_control()

        self.working_count += 1

    def publish_gripper_control(self):
        msg = GripperControl()
        msg.control_word = 1
        msg.finger_target_width = 50
        msg.finger_target_pose = 90
        msg.finger_width_speed = 100
        msg.finger_pose_speed = 150
        msg.gripping_force = 30
        msg.contact_detection_sesitivity = 80
        msg.target_current = [100, 200, 300, 400]
        self.gripper_control_publisher.publish(msg)
        self.get_logger().info(f'Publishing GripperControl message.')

    def call_motor_on(self, on):
        while not self.motor_on_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('MotorOn service not available, waiting again...')

        request = MotorOn.Request()
        request.on = on
        self.future = self.motor_on_client.call_async(request)
        self.future.add_done_callback(self.motor_on_callback)

    def motor_on_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'MotorOn service call result: {response.result}')
        except Exception as e:
            self.get_logger().error(f'MotorOn service call failed: {e}')

    def call_reset_abs_encoder(self):
        while not self.reset_abs_encoder_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ResetAbsEncoder service not available, waiting again...')

        request = ResetAbsEncoder.Request()
        self.future = self.reset_abs_encoder_client.call_async(request)
        self.future.add_done_callback(self.reset_abs_encoder_callback)

    def reset_abs_encoder_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'ResetAbsEncoder service call result: {response.result}')
        except Exception as e:
            self.get_logger().error(f'ResetAbsEncoder service call failed: {e}')

    def call_set_target_width(self, finger_target_width, finger_width_speed, gripping_force, contact_detection_sesitivity):
        while not self.set_target_width_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetTargetWidth service not available, waiting again...')

        request = SetTargetWidth.Request()
        request.finger_target_width = finger_target_width
        request.finger_width_speed = finger_width_speed
        request.gripping_force = gripping_force
        request.contact_detection_sesitivity = contact_detection_sesitivity
        self.future = self.set_target_width_client.call_async(request)
        self.future.add_done_callback(self.set_target_width_callback)

    def set_target_width_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'SetTargetWidth service call result: {response.result}')
        except Exception as e:
            self.get_logger().error(f'SetTargetWidth service call failed: {e}')

    def call_set_target_pose(self, finger_target_pose, finger_pose_speed):
        while not self.set_target_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetTargetPose service not available, waiting again...')

        request = SetTargetPose.Request()
        request.finger_target_pose = finger_target_pose
        request.finger_pose_speed = finger_pose_speed
        self.future = self.set_target_pose_client.call_async(request)
        self.future.add_done_callback(self.set_target_pose_callback)

    def set_target_pose_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'SetTargetPose service call result: {response.result}')
        except Exception as e:
            self.get_logger().error(f'SetTargetPose service call failed: {e}')

    def call_set_target_current(self, target_current):
        while not self.set_target_current_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetTargetCurrent service not available, waiting again...')

        request = SetTargetCurrent.Request()
        request.target_current = target_current
        self.future = self.set_target_current_client.call_async(request)
        self.future.add_done_callback(self.set_target_current_callback)

    def set_target_current_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'SetTargetCurrent service call result: {response.result}')
        except Exception as e:
            self.get_logger().error(f'SetTargetCurrent service call failed: {e}')

    def call_set_target(self, finger_target_width, finger_target_pose, finger_width_speed, finger_pose_speed, gripping_force, contact_detection_sesitivity):
        while not self.set_target_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetTarget service not available, waiting again...')

        request = SetTarget.Request()
        request.finger_target_width = finger_target_width
        request.finger_target_pose = finger_target_pose
        request.finger_width_speed = finger_width_speed
        request.finger_pose_speed = finger_pose_speed
        request.gripping_force = gripping_force
        request.contact_detection_sesitivity = contact_detection_sesitivity
        self.future = self.set_target_client.call_async(request)
        self.future.add_done_callback(self.set_target_callback)

    def set_target_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'SetTarget service call result: {response.result}')
        except Exception as e:
            self.get_logger().error(f'SetTarget service call failed: {e}')

    def call_reset_friction_model(self):
        while not self.reset_friction_model_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ResetFrictionModel service not available, waiting again...')

        request = ResetFrictionModel.Request()
        self.future = self.reset_friction_model_client.call_async(request)
        self.future.add_done_callback(self.reset_friction_model_callback)

    def reset_friction_model_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'ResetFrictionModel service call result: {response.result}')
        except Exception as e:
            self.get_logger().error(f'ResetFrictionModel service call failed: {e}')

    def call_get_gripper_info(self):
        while not self.get_gripper_info_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GetGripperInfo service not available, waiting again...')

        request = GetGripperInfo.Request()
        self.future = self.get_gripper_info_client.call_async(request)
        self.future.add_done_callback(self.get_gripper_info_callback)

    def get_gripper_info_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'GetGripperInfo service call result: Name={response.info.name}, Version={response.info.version}, Description={response.info.description}')
        except Exception as e:
            self.get_logger().error(f'GetGripperInfo service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GripperClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    