#ifndef __GRIPPER_ECAT_H__
#define __GRIPPER_ECAT_H__

#include <memory>
#include <thread>
#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include "art_gripper_interfaces/msg/gripper_info.hpp"
#include "art_gripper_interfaces/msg/gripper_control.hpp"
#include "art_gripper_interfaces/msg/gripper_status.hpp"
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
#include "robotData.h"

class GripperEcat : public rclcpp::Node
{
public:
    GripperEcat(std::shared_ptr<RobotData> robot_data);
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

    void StatusPublishThread();

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

    std::shared_ptr<RobotData> _robot_data;
    std::thread _status_thread;
    std::atomic<bool> _is_running;
};

#endif // __GRIPPER_ECAT_H__