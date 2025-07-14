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
#include "art_gripper_interfaces/srv/set_target_width.hpp"
#include "art_gripper_interfaces/srv/set_target_pose.hpp"
#include "art_gripper_interfaces/srv/set_target_current.hpp"
#include "art_gripper_interfaces/srv/set_target.hpp"
#include "art_gripper_interfaces/srv/reset_friction_model.hpp"
#include "robot_data.h"

class GripperEcat : public rclcpp::Node
{
public:
    GripperEcat();
    ~GripperEcat();

private:
    void OnGripperControl(const art_gripper_interfaces::msg::GripperControl::SharedPtr msg);
    void OnMotorOn(
        const std::shared_ptr<art_gripper_interfaces::srv::MotorOn::Request> request,
        std::shared_ptr<art_gripper_interfaces::srv::MotorOn::Response> response);
    void OnResetAbsEncoder(
        const std::shared_ptr<art_gripper_interfaces::srv::ResetAbsEncoder::Request> request,
        std::shared_ptr<art_gripper_interfaces::srv::ResetAbsEncoder::Response> response);
    void OnSetTargetWidth(
        const std::shared_ptr<art_gripper_interfaces::srv::SetTargetWidth::Request> request,
        std::shared_ptr<art_gripper_interfaces::srv::SetTargetWidth::Response> response);
    void OnSetTargetPose(
        const std::shared_ptr<art_gripper_interfaces::srv::SetTargetPose::Request> request,
        std::shared_ptr<art_gripper_interfaces::srv::SetTargetPose::Response> response);
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
    void StatusPublishThread();

    rclcpp::Subscription<art_gripper_interfaces::msg::GripperControl>::SharedPtr _gripper_control_subscriber;
    rclcpp::Service<art_gripper_interfaces::srv::MotorOn>::SharedPtr _motor_on_server;
    rclcpp::Service<art_gripper_interfaces::srv::ResetAbsEncoder>::SharedPtr _reset_abs_encoder_server;
    rclcpp::Service<art_gripper_interfaces::srv::SetTargetWidth>::SharedPtr _set_target_width_server;
    rclcpp::Service<art_gripper_interfaces::srv::SetTargetPose>::SharedPtr _set_target_pose_server;
    rclcpp::Service<art_gripper_interfaces::srv::SetTargetCurrent>::SharedPtr _set_target_current_server;
    rclcpp::Service<art_gripper_interfaces::srv::SetTarget>::SharedPtr _set_target_server;
    rclcpp::Service<art_gripper_interfaces::srv::ResetFrictionModel>::SharedPtr _reset_friction_model_server;
    rclcpp::Service<art_gripper_interfaces::srv::GetGripperInfo>::SharedPtr _gripper_info_server;
    rclcpp::Publisher<art_gripper_interfaces::msg::GripperStatus>::SharedPtr _status_publisher;

    std::shared_ptr<RobotData> _robot_data;
    std::thread _status_thread;
    std::atomic<bool> _is_running;
};

#endif // __GRIPPER_ECAT_H__