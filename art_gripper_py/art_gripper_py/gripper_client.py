# MIT License
#
# Copyright (c) 2025 Hyundai Motor Company
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Author: Sangyup Yi (sean.yi@hyundai.com)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from art_gripper_interfaces.msg import GripperControl, GripperStatus, GripperInfo
from art_gripper_interfaces.srv import (
    MotorOn, ResetAbsEncoder, SetTargetFingerWidth, SetTargetFingerPose, SetTargetCurrent, SetTarget, ResetFrictionModel, GetGripperInfo,
    SetContactSensitivity, SetGrippingForce, SetTargetFingerPoseSpeed, SetTargetFingerPoseWithSpeed, SetTargetFingerWidthSpeed, SetTargetFingerWidthWithSpeed
)

class GripperClient(Node):
    def __init__(self):
        super().__init__('gripper_client_py')
        self.gripper_control_publisher = self.create_publisher(GripperControl, 'gripper_control', 10)
        self.motor_on_client = self.create_client(MotorOn, 'motor_on')
        self.reset_abs_encoder_client = self.create_client(ResetAbsEncoder, 'reset_abs_encoder')
        self.set_target_finger_width_client = self.create_client(SetTargetFingerWidth, 'set_target_finger_width')
        self.set_target_finger_pose_client = self.create_client(SetTargetFingerPose, 'set_target_finger_pose')
        self.set_target_current_client = self.create_client(SetTargetCurrent, 'set_target_current')
        self.set_target_client = self.create_client(SetTarget, 'set_target')
        self.reset_friction_model_client = self.create_client(ResetFrictionModel, 'reset_friction_model')
        self.get_gripper_info_client = self.create_client(GetGripperInfo, 'get_gripper_info')
        self.set_contact_sensitivity_client = self.create_client(SetContactSensitivity, 'set_contact_sensitivity')
        self.set_gripping_force_client = self.create_client(SetGrippingForce, 'set_gripping_force')
        self.set_target_finger_pose_speed_client = self.create_client(SetTargetFingerPoseSpeed, 'set_target_finger_pose_speed')
        self.set_target_finger_pose_with_speed_client = self.create_client(SetTargetFingerPoseWithSpeed, 'set_target_finger_pose_with_speed')
        self.set_target_finger_width_speed_client = self.create_client(SetTargetFingerWidthSpeed, 'set_target_finger_width_speed')
        self.set_target_finger_width_with_speed_client = self.create_client(SetTargetFingerWidthWithSpeed, 'set_target_finger_width_with_speed')

        # Get gripper info
        self.call_get_gripper_info()

        # for periodic action
        timer_period = 3.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.working_count = 0

        # List of sequential calls for periodic execution
        self.sequential_calls = [
            lambda: self.call_motor_on(True),
            self.call_reset_abs_encoder,
            self.call_reset_friction_model,
            lambda: self.call_set_target_finger_pose_with_speed(180, 120),
            lambda: self.call_set_target_finger_width_with_speed(0, 150),
            lambda: self.call_set_target_finger_width_with_speed(100, 150),
            lambda: self.call_set_target_finger_pose_with_speed(90, 120),
            lambda: self.call_set_target_finger_pose_with_speed(0, 120),
            lambda: self.call_set_target_finger_width_with_speed(0, 150),
            lambda: self.call_set_target_finger_width_with_speed(100, 150),
            lambda: self.call_set_target_finger_pose_with_speed(90, 120),
            # lambda: self.call_set_contact_sensitivity(80),
            # lambda: self.call_set_gripping_force(30),
            lambda: self.call_motor_on(False),
        ]
        self.sequential_call_index = 0

    def timer_callback(self):
        # Call one function from the sequential_calls list per timer tick
        if self.sequential_calls: # Ensure the list is not empty
            self.sequential_calls[self.sequential_call_index]()
            self.sequential_call_index = (self.sequential_call_index + 1) % len(self.sequential_calls)

        # self.publish_gripper_control()

        self.working_count += 1

    def publish_gripper_control(self):
        msg = GripperControl()
        msg.gripper_control = 1
        msg.finger_width = 50
        msg.finger_pose = 90
        msg.finger_width_speed = 100
        msg.finger_pose_speed = 150
        msg.gripping_force = 30
        msg.contact_sensitivity = 80
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

    def call_set_target_finger_width(self, finger_width):
        while not self.set_target_finger_width_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetTargetFingerWidth service not available, waiting again...')

        request = SetTargetFingerWidth.Request()
        request.finger_width = finger_width
        self.future = self.set_target_finger_width_client.call_async(request)
        self.future.add_done_callback(self.set_target_finger_width_callback)

    def set_target_finger_width_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'SetTargetFingerWidth service call result: {response.result}')
        except Exception as e:
            self.get_logger().error(f'SetTargetFingerWidth service call failed: {e}')

    def call_set_target_finger_pose(self, finger_pose):
        while not self.set_target_finger_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetTargetFingerPose service not available, waiting again...')

        request = SetTargetFingerPose.Request()
        request.finger_pose = finger_pose
        self.future = self.set_target_finger_pose_client.call_async(request)
        self.future.add_done_callback(self.set_target_finger_pose_callback)

    def set_target_finger_pose_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'SetTargetFingerPose service call result: {response.result}')
        except Exception as e:
            self.get_logger().error(f'SetTargetFingerPose service call failed: {e}')

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

    def call_set_target(self, finger_width, finger_pose, finger_width_speed, finger_pose_speed, gripping_force, contact_sensitivity):
        while not self.set_target_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetTarget service not available, waiting again...')

        request = SetTarget.Request()
        request.finger_width = finger_width
        request.finger_pose = finger_pose
        request.finger_width_speed = finger_width_speed
        request.finger_pose_speed = finger_pose_speed
        request.gripping_force = gripping_force
        request.contact_sensitivity = contact_sensitivity
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

    def call_set_contact_sensitivity(self, contact_sensitivity):
        while not self.set_contact_sensitivity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetContactSensitivity service not available, waiting again...')

        request = SetContactSensitivity.Request()
        request.contact_sensitivity = contact_sensitivity
        self.future = self.set_contact_sensitivity_client.call_async(request)
        self.future.add_done_callback(self.set_contact_sensitivity_callback)

    def set_contact_sensitivity_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'SetContactSensitivity service call result: {response.result}')
        except Exception as e:
            self.get_logger().error(f'SetContactSensitivity service call failed: {e}')

    def call_set_gripping_force(self, force):
        while not self.set_gripping_force_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetGrippingForce service not available, waiting again...')

        request = SetGrippingForce.Request()
        request.gripping_force = force
        self.future = self.set_gripping_force_client.call_async(request)
        self.future.add_done_callback(self.set_gripping_force_callback)

    def set_gripping_force_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'SetGrippingForce service call result: {response.result}')
        except Exception as e:
            self.get_logger().error(f'SetGrippingForce service call failed: {e}')

    def call_set_target_finger_pose_speed(self, speed):
        while not self.set_target_finger_pose_speed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetTargetFingerPoseSpeed service not available, waiting again...')

        request = SetTargetFingerPoseSpeed.Request()
        request.finger_pose_speed = speed
        self.future = self.set_target_finger_pose_speed_client.call_async(request)
        self.future.add_done_callback(self.set_target_finger_pose_speed_callback)

    def set_target_finger_pose_speed_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'SetTargetFingerPoseSpeed service call result: {response.result}')
        except Exception as e:
            self.get_logger().error(f'SetTargetFingerPoseSpeed service call failed: {e}')

    def call_set_target_finger_pose_with_speed(self, pose, speed):
        while not self.set_target_finger_pose_with_speed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetTargetFingerPoseWithSpeed service not available, waiting again...')

        request = SetTargetFingerPoseWithSpeed.Request()
        request.finger_pose = pose
        request.finger_pose_speed = speed
        self.future = self.set_target_finger_pose_with_speed_client.call_async(request)
        self.future.add_done_callback(self.set_target_finger_pose_with_speed_callback)

    def set_target_finger_pose_with_speed_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'SetTargetFingerPoseWithSpeed service call result: {response.result}')
        except Exception as e:
            self.get_logger().error(f'SetTargetFingerPoseWithSpeed service call failed: {e}')

    def call_set_target_finger_width_speed(self, speed):
        while not self.set_target_finger_width_speed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetTargetFingerWidthSpeed service not available, waiting again...')

        request = SetTargetFingerWidthSpeed.Request()
        request.finger_width_speed = speed
        self.future = self.set_target_finger_width_speed_client.call_async(request)
        self.future.add_done_callback(self.set_target_finger_width_speed_callback)

    def set_target_finger_width_speed_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'SetTargetFingerWidthSpeed service call result: {response.result}')
        except Exception as e:
            self.get_logger().error(f'SetTargetFingerWidthSpeed service call failed: {e}')

    def call_set_target_finger_width_with_speed(self, width, speed):
        while not self.set_target_finger_width_with_speed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetTargetFingerWidthWithSpeed service not available, waiting again...')

        request = SetTargetFingerWidthWithSpeed.Request()
        request.finger_width = width
        request.finger_width_speed = speed
        self.future = self.set_target_finger_width_with_speed_client.call_async(request)
        self.future.add_done_callback(self.set_target_finger_width_with_speed_callback)

    def set_target_finger_width_with_speed_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'SetTargetFingerWidthWithSpeed service call result: {response.result}')
        except Exception as e:
            self.get_logger().error(f'SetTargetFingerWidthWithSpeed service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GripperClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    