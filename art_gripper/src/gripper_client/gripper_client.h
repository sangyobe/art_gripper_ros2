/*
MIT License

Copyright (c) 2025 Hyundai Motor Company

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Author: Sangyup Yi (sean.yi@hyundai.com)
*/
#ifndef __GRIPPER_CLIENT_H__
#define __GRIPPER_CLIENT_H__

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "art_gripper_interfaces/msg/gripper_control.hpp"
#include "art_gripper_interfaces/msg/gripper_status.hpp"
#include "art_gripper_interfaces/msg/gripper_info.hpp"
#include "art_gripper_interfaces/srv/motor_on.hpp"
#include "art_gripper_interfaces/srv/reset_abs_encoder.hpp"
#include "art_gripper_interfaces/srv/set_target_finger_width.hpp"
#include "art_gripper_interfaces/srv/set_target_finger_pose.hpp"
#include "art_gripper_interfaces/srv/set_target_current.hpp"
#include "art_gripper_interfaces/srv/set_target.hpp"
#include "art_gripper_interfaces/srv/reset_friction_model.hpp"
#include "art_gripper_interfaces/srv/get_gripper_info.hpp"
#include "art_gripper_interfaces/srv/set_contact_sensitivity.hpp"
#include "art_gripper_interfaces/srv/set_gripping_force.hpp"
#include "art_gripper_interfaces/srv/set_target_finger_pose_speed.hpp"
#include "art_gripper_interfaces/srv/set_target_finger_pose_with_speed.hpp"
#include "art_gripper_interfaces/srv/set_target_finger_width_speed.hpp"
#include "art_gripper_interfaces/srv/set_target_finger_width_with_speed.hpp"

class GripperClient : public rclcpp::Node
{
public:
    GripperClient();
    void CallMotorOn(const bool on);
    void CallResetAbsEncoder();
    void CallSetTargetFingerWidth(uint8_t finger_width);
    void CallSetTargetFingerPose(uint8_t finger_pose);
    void CallSetTargetCurrent(int16_t target_current[4]);
    void CallSetTarget(uint8_t finger_width, uint8_t finger_pose, uint8_t finger_width_speed, uint8_t finger_pose_speed, uint8_t gripping_force, uint8_t contact_sensitivity);
    void CallResetFrictionModel();
    void PublishGripperControl();
    void CallGetGripperInfo();
    void CallSetContactSensitivity(uint8_t contact_sensitivity);
    void CallSetGrippingForce(uint8_t force);
    void CallSetTargetFingerPoseSpeed(uint8_t speed);
    void CallSetTargetFingerPoseWithSpeed(uint8_t pose, uint8_t speed);
    void CallSetTargetFingerWidthSpeed(uint8_t speed);
    void CallSetTargetFingerWidthWithSpeed(uint8_t width, uint8_t speed);

private:
    void OnTimer();
    void OnMotorOnResponse(rclcpp::Client<art_gripper_interfaces::srv::MotorOn>::SharedFuture future);
    void OnResetAbsEncoderResponse(rclcpp::Client<art_gripper_interfaces::srv::ResetAbsEncoder>::SharedFuture future);
    void OnSetTargetFingerWidthResponse(rclcpp::Client<art_gripper_interfaces::srv::SetTargetFingerWidth>::SharedFuture future);
    void OnSetTargetFingerPoseResponse(rclcpp::Client<art_gripper_interfaces::srv::SetTargetFingerPose>::SharedFuture future);
    void OnSetTargetCurrentResponse(rclcpp::Client<art_gripper_interfaces::srv::SetTargetCurrent>::SharedFuture future);
    void OnSetTargetResponse(rclcpp::Client<art_gripper_interfaces::srv::SetTarget>::SharedFuture future);
    void OnResetFrictionModelResponse(rclcpp::Client<art_gripper_interfaces::srv::ResetFrictionModel>::SharedFuture future);
    void OnGetGripperInfoResponse(rclcpp::Client<art_gripper_interfaces::srv::GetGripperInfo>::SharedFuture future);
    void OnSetContactSensitivityResponse(rclcpp::Client<art_gripper_interfaces::srv::SetContactSensitivity>::SharedFuture future);
    void OnSetGrippingForceResponse(rclcpp::Client<art_gripper_interfaces::srv::SetGrippingForce>::SharedFuture future);
    void OnSetTargetFingerPoseSpeedResponse(rclcpp::Client<art_gripper_interfaces::srv::SetTargetFingerPoseSpeed>::SharedFuture future);
    void OnSetTargetFingerPoseWithSpeedResponse(rclcpp::Client<art_gripper_interfaces::srv::SetTargetFingerPoseWithSpeed>::SharedFuture future);
    void OnSetTargetFingerWidthSpeedResponse(rclcpp::Client<art_gripper_interfaces::srv::SetTargetFingerWidthSpeed>::SharedFuture future);
    void OnSetTargetFingerWidthWithSpeedResponse(rclcpp::Client<art_gripper_interfaces::srv::SetTargetFingerWidthWithSpeed>::SharedFuture future);

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<art_gripper_interfaces::msg::GripperControl>::SharedPtr _gripper_control_publisher;
    rclcpp::Client<art_gripper_interfaces::srv::MotorOn>::SharedPtr _motor_on_client;
    rclcpp::Client<art_gripper_interfaces::srv::ResetAbsEncoder>::SharedPtr _reset_abs_encoder_client;
    rclcpp::Client<art_gripper_interfaces::srv::SetTargetFingerWidth>::SharedPtr _set_target_finger_width_client;
    rclcpp::Client<art_gripper_interfaces::srv::SetTargetFingerPose>::SharedPtr _set_target_finger_pose_client;
    rclcpp::Client<art_gripper_interfaces::srv::SetTargetCurrent>::SharedPtr _set_target_current_client;
    rclcpp::Client<art_gripper_interfaces::srv::SetTarget>::SharedPtr _set_target_client;
    rclcpp::Client<art_gripper_interfaces::srv::ResetFrictionModel>::SharedPtr _reset_friction_model_client;
    rclcpp::Client<art_gripper_interfaces::srv::GetGripperInfo>::SharedPtr _gripper_info_client;
    rclcpp::Client<art_gripper_interfaces::srv::SetContactSensitivity>::SharedPtr _set_contact_sensitivity_client;
    rclcpp::Client<art_gripper_interfaces::srv::SetGrippingForce>::SharedPtr _set_gripping_force_client;
    rclcpp::Client<art_gripper_interfaces::srv::SetTargetFingerPoseSpeed>::SharedPtr _set_target_finger_pose_speed_client;
    rclcpp::Client<art_gripper_interfaces::srv::SetTargetFingerPoseWithSpeed>::SharedPtr _set_target_finger_pose_with_speed_client;
    rclcpp::Client<art_gripper_interfaces::srv::SetTargetFingerWidthSpeed>::SharedPtr _set_target_finger_width_speed_client;
    rclcpp::Client<art_gripper_interfaces::srv::SetTargetFingerWidthWithSpeed>::SharedPtr _set_target_finger_width_with_speed_client;
    size_t _working_count;
    std::vector<std::function<void()>> _sequential_calls;
    size_t _sequential_call_index;

};

#endif // __GRIPPER_CLIENT_H__