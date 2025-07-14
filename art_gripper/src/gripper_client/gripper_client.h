#ifndef __GRIPPER_CLIENT_H__
#define __GRIPPER_CLIENT_H__

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "art_gripper_interfaces/msg/gripper_control.hpp"
#include "art_gripper_interfaces/msg/gripper_status.hpp"
#include "art_gripper_interfaces/msg/gripper_info.hpp"
#include "art_gripper_interfaces/srv/motor_on.hpp"
#include "art_gripper_interfaces/srv/reset_abs_encoder.hpp"
#include "art_gripper_interfaces/srv/set_target_width.hpp"
#include "art_gripper_interfaces/srv/set_target_pose.hpp"
#include "art_gripper_interfaces/srv/set_target_current.hpp"
#include "art_gripper_interfaces/srv/set_target.hpp"
#include "art_gripper_interfaces/srv/reset_friction_model.hpp"
#include "art_gripper_interfaces/srv/get_gripper_info.hpp"

class GripperClient : public rclcpp::Node
{
public:
    GripperClient();
    void CallMotorOn(const bool on);
    void CallResetAbsEncoder();
    void CallSetTargetWidth(uint8_t finger_target_width, uint8_t finger_width_speed, uint8_t gripping_force, uint8_t contact_detection_sesitivity);
    void CallSetTargetPose(uint8_t finger_target_pose, uint8_t finger_pose_speed);
    void CallSetTargetCurrent(int16_t target_current[4]);
    void CallSetTarget(uint8_t finger_target_width, uint8_t finger_target_pose, uint8_t finger_width_speed, uint8_t finger_pose_speed, uint8_t gripping_force, uint8_t contact_detection_sesitivity);
    void CallResetFrictionModel();
    void PublishGripperControl();
    void CallGetGripperInfo();

private:
    void OnTimer();
    void OnMotorOnResponse(rclcpp::Client<art_gripper_interfaces::srv::MotorOn>::SharedFuture future);
    void OnResetAbsEncoderResponse(rclcpp::Client<art_gripper_interfaces::srv::ResetAbsEncoder>::SharedFuture future);
    void OnSetTargetWidthResponse(rclcpp::Client<art_gripper_interfaces::srv::SetTargetWidth>::SharedFuture future);
    void OnSetTargetPoseResponse(rclcpp::Client<art_gripper_interfaces::srv::SetTargetPose>::SharedFuture future);
    void OnSetTargetCurrentResponse(rclcpp::Client<art_gripper_interfaces::srv::SetTargetCurrent>::SharedFuture future);
    void OnSetTargetResponse(rclcpp::Client<art_gripper_interfaces::srv::SetTarget>::SharedFuture future);
    void OnResetFrictionModelResponse(rclcpp::Client<art_gripper_interfaces::srv::ResetFrictionModel>::SharedFuture future);
    void OnGetGripperInfoResponse(rclcpp::Client<art_gripper_interfaces::srv::GetGripperInfo>::SharedFuture future);

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<art_gripper_interfaces::msg::GripperControl>::SharedPtr _gripper_control_publisher;
    rclcpp::Client<art_gripper_interfaces::srv::MotorOn>::SharedPtr _motor_on_client;
    rclcpp::Client<art_gripper_interfaces::srv::ResetAbsEncoder>::SharedPtr _reset_abs_encoder_client;
    rclcpp::Client<art_gripper_interfaces::srv::SetTargetWidth>::SharedPtr _set_target_width_client;
    rclcpp::Client<art_gripper_interfaces::srv::SetTargetPose>::SharedPtr _set_target_pose_client;
    rclcpp::Client<art_gripper_interfaces::srv::SetTargetCurrent>::SharedPtr _set_target_current_client;
    rclcpp::Client<art_gripper_interfaces::srv::SetTarget>::SharedPtr _set_target_client;
    rclcpp::Client<art_gripper_interfaces::srv::ResetFrictionModel>::SharedPtr _reset_friction_model_client;
    rclcpp::Client<art_gripper_interfaces::srv::GetGripperInfo>::SharedPtr _gripper_info_client;
    size_t _working_count;

};

#endif // __GRIPPER_CLIENT_H__