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
#ifndef __GRIPPER_ECAT_H__
#define __GRIPPER_ECAT_H__

#include <memory>
#include <thread>
#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include "art_gripper_interfaces/msg/gripper_info.hpp"
#include "art_gripper_interfaces/msg/gripper_control.hpp"
#include "art_gripper_interfaces/msg/gripper_status.hpp"
#include "art_gripper_interfaces/msg/ethercat_state.hpp"
#include "art_gripper_interfaces/srv/get_gripper_info.hpp"
#include "art_gripper_interfaces/srv/motor_on.hpp"
#include "art_gripper_interfaces/srv/reset_abs_encoder.hpp"
#include "art_gripper_interfaces/srv/set_target_finger_width.hpp"
#include "art_gripper_interfaces/srv/set_target_finger_pose.hpp"
#include "art_gripper_interfaces/srv/set_target_current.hpp"
#include "art_gripper_interfaces/srv/set_target.hpp"
#include "art_gripper_interfaces/srv/reset_friction_model.hpp"
#include "art_gripper_interfaces/srv/set_contact_sensitivity.hpp"
#include "art_gripper_interfaces/srv/set_gripping_force.hpp"
#include "art_gripper_interfaces/srv/set_target_finger_pose_speed.hpp"
#include "art_gripper_interfaces/srv/set_target_finger_pose_with_speed.hpp"
#include "art_gripper_interfaces/srv/set_target_finger_width_speed.hpp"
#include "art_gripper_interfaces/srv/set_target_finger_width_with_speed.hpp"
#include "sysData.h"

class GripperEcat : public rclcpp::Node
{
public:
    GripperEcat(std::shared_ptr<SysData> sys_data);
    ~GripperEcat();

private:
    void OnGripperControl(const art_gripper_interfaces::msg::GripperControl::SharedPtr msg);
    void OnMotorOn(
        const std::shared_ptr<art_gripper_interfaces::srv::MotorOn::Request> request,
        std::shared_ptr<art_gripper_interfaces::srv::MotorOn::Response> response);
    void OnResetAbsEncoder(
        const std::shared_ptr<art_gripper_interfaces::srv::ResetAbsEncoder::Request> request,
        std::shared_ptr<art_gripper_interfaces::srv::ResetAbsEncoder::Response> response);
    void OnSetTargetFingerWidth(
        const std::shared_ptr<art_gripper_interfaces::srv::SetTargetFingerWidth::Request> request,
        std::shared_ptr<art_gripper_interfaces::srv::SetTargetFingerWidth::Response> response);
    void OnSetTargetFingerPose(
        const std::shared_ptr<art_gripper_interfaces::srv::SetTargetFingerPose::Request> request,
        std::shared_ptr<art_gripper_interfaces::srv::SetTargetFingerPose::Response> response);
    void OnSetTargetCurrent(
        const std::shared_ptr<art_gripper_interfaces::srv::SetTargetCurrent::Request> request,
        std::shared_ptr<art_gripper_interfaces::srv::SetTargetCurrent::Response> response);
    void OnSetTarget(
        const std::shared_ptr<art_gripper_interfaces::srv::SetTarget::Request> request,
        std::shared_ptr<art_gripper_interfaces::srv::SetTarget::Response> response);
    void OnResetFrictionModel(
        const std::shared_ptr<art_gripper_interfaces::srv::ResetFrictionModel::Request> request,
        std::shared_ptr<art_gripper_interfaces::srv::ResetFrictionModel::Response> response);
    void OnGetGripperInfo(
        const std::shared_ptr<art_gripper_interfaces::srv::GetGripperInfo::Request> request,
        std::shared_ptr<art_gripper_interfaces::srv::GetGripperInfo::Response> response);
    void OnSetContactSensitivity(
        const std::shared_ptr<art_gripper_interfaces::srv::SetContactSensitivity::Request> request,
        std::shared_ptr<art_gripper_interfaces::srv::SetContactSensitivity::Response> response);
    void OnSetGrippingForce(
        const std::shared_ptr<art_gripper_interfaces::srv::SetGrippingForce::Request> request,
        std::shared_ptr<art_gripper_interfaces::srv::SetGrippingForce::Response> response);
    void OnSetTargetFingerPoseSpeed(
        const std::shared_ptr<art_gripper_interfaces::srv::SetTargetFingerPoseSpeed::Request> request,
        std::shared_ptr<art_gripper_interfaces::srv::SetTargetFingerPoseSpeed::Response> response);
    void OnSetTargetFingerPoseWithSpeed(
        const std::shared_ptr<art_gripper_interfaces::srv::SetTargetFingerPoseWithSpeed::Request> request,
        std::shared_ptr<art_gripper_interfaces::srv::SetTargetFingerPoseWithSpeed::Response> response);
    void OnSetTargetFingerWidthSpeed(
        const std::shared_ptr<art_gripper_interfaces::srv::SetTargetFingerWidthSpeed::Request> request,
        std::shared_ptr<art_gripper_interfaces::srv::SetTargetFingerWidthSpeed::Response> response);
    void OnSetTargetFingerWidthWithSpeed(
        const std::shared_ptr<art_gripper_interfaces::srv::SetTargetFingerWidthWithSpeed::Request> request,
        std::shared_ptr<art_gripper_interfaces::srv::SetTargetFingerWidthWithSpeed::Response> response);

    void GripperStatusPublishCallback();
    void EthercatStatePublishCallback();

    rclcpp::Subscription<art_gripper_interfaces::msg::GripperControl>::SharedPtr _gripper_control_subscriber;
    rclcpp::Service<art_gripper_interfaces::srv::MotorOn>::SharedPtr _motor_on_server;
    rclcpp::Service<art_gripper_interfaces::srv::ResetAbsEncoder>::SharedPtr _reset_abs_encoder_server;
    rclcpp::Service<art_gripper_interfaces::srv::SetTargetFingerWidth>::SharedPtr _set_target_width_server;
    rclcpp::Service<art_gripper_interfaces::srv::SetTargetFingerPose>::SharedPtr _set_target_pose_server;
    rclcpp::Service<art_gripper_interfaces::srv::SetTargetCurrent>::SharedPtr _set_target_current_server;
    rclcpp::Service<art_gripper_interfaces::srv::SetTarget>::SharedPtr _set_target_server;
    rclcpp::Service<art_gripper_interfaces::srv::ResetFrictionModel>::SharedPtr _reset_friction_model_server;
    rclcpp::Service<art_gripper_interfaces::srv::GetGripperInfo>::SharedPtr _gripper_info_server;
    rclcpp::Service<art_gripper_interfaces::srv::SetContactSensitivity>::SharedPtr _set_contact_sensitivity_server;
    rclcpp::Service<art_gripper_interfaces::srv::SetGrippingForce>::SharedPtr _set_gripping_force_server;
    rclcpp::Service<art_gripper_interfaces::srv::SetTargetFingerPoseSpeed>::SharedPtr _set_target_finger_pose_speed_server;
    rclcpp::Service<art_gripper_interfaces::srv::SetTargetFingerPoseWithSpeed>::SharedPtr _set_target_finger_pose_with_speed_server;
    rclcpp::Service<art_gripper_interfaces::srv::SetTargetFingerWidthSpeed>::SharedPtr _set_target_finger_width_speed_server;
    rclcpp::Service<art_gripper_interfaces::srv::SetTargetFingerWidthWithSpeed>::SharedPtr _set_target_finger_width_with_speed_server;
    rclcpp::Publisher<art_gripper_interfaces::msg::GripperStatus>::SharedPtr _status_publisher;
    rclcpp::Publisher<art_gripper_interfaces::msg::EthercatState>::SharedPtr _ethercat_state_publisher;
    rclcpp::TimerBase::SharedPtr _gripper_status_publish_timer;
    rclcpp::TimerBase::SharedPtr _ethercat_state_publish_timer;

    std::shared_ptr<SysData> _sys_data;
    std::shared_ptr<RobotData> _robot_data;
};

#endif // __GRIPPER_ECAT_H__