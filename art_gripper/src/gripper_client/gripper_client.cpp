#include "gripper_client.h"
#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

GripperClient::GripperClient()
: Node("gripper_client"), _count(0)
{
    _publisher = this->create_publisher<std_msgs::msg::String>("greeting", 10);
    _timer = this->create_wall_timer(1000ms, std::bind(&GripperClient::OnTimer, this));
    _motor_on_client = this->create_client<art_gripper_interfaces::srv::MotorOn>("MotorOn");
    _reset_abs_encoder_client = this->create_client<art_gripper_interfaces::srv::ResetAbsEncoder>("ResetAbsEncoder");
    _target_width_client = this->create_client<art_gripper_interfaces::srv::TargetWidth>("TargetWidth");
    _target_pose_client = this->create_client<art_gripper_interfaces::srv::TargetPose>("TargetPose");
    _set_target_current_client = this->create_client<art_gripper_interfaces::srv::SetTargetCurrent>("SetTargetCurrent");
    _set_target_client = this->create_client<art_gripper_interfaces::srv::SetTarget>("SetTarget");
    _reset_friction_model_client = this->create_client<art_gripper_interfaces::srv::ResetFrictionModel>("ResetFrictionModel");
}

void GripperClient::OnTimer()
{
    auto msg = std_msgs::msg::String();
    msg.data = "Hello " + std::to_string(_count++);
    RCLCPP_INFO(this->get_logger(), "pub: %s", msg.data.c_str());
    _publisher->publish(msg);

    if (_count % 2 == 0) {
        this->CallMotorOn(true);
    } else {
        this->CallMotorOn(false);
    }

    if (_count % 5 == 0) {
        this->CallResetAbsEncoder();
        this->CallTargetWidth(50, 100, 50, 80);
        this->CallTargetPose(90, 120);
        int16_t current_array[4] = {100, 200, 300, 400};
        this->CallSetTargetCurrent(current_array);
        this->CallSetTarget(25, 45, 75, 90, 20, 60);
        this->CallResetFrictionModel();
    }
}

void GripperClient::CallMotorOn(const bool on)
{
    RCLCPP_INFO(this->get_logger(), "CallMotorOn(%s)...", (on ? "True" : "False"));
    if (!_motor_on_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        return;
    }

    auto request = std::make_shared<art_gripper_interfaces::srv::MotorOn::Request>();
    request->on = on;

    _motor_on_client->async_send_request(
        request, std::bind(&GripperClient::OnMotorOnResponse, this, _1));
}

void GripperClient::CallResetAbsEncoder()
{
    RCLCPP_INFO(this->get_logger(), "CallResetAbsEncoder()...");
    if (!_reset_abs_encoder_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        return;
    }

    auto request = std::make_shared<art_gripper_interfaces::srv::ResetAbsEncoder::Request>();

    _reset_abs_encoder_client->async_send_request(
        request, std::bind(&GripperClient::OnResetAbsEncoderResponse, this, _1));
}

void GripperClient::OnMotorOnResponse(rclcpp::Client<art_gripper_interfaces::srv::MotorOn>::SharedFuture future)
{
    try {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "service call result: %d", result->result);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service MotorOn: %s", e.what());
    }
}

void GripperClient::OnResetAbsEncoderResponse(rclcpp::Client<art_gripper_interfaces::srv::ResetAbsEncoder>::SharedFuture future)
{
    try {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "service call result: %d", result->result);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service ResetAbsEncoder: %s", e.what());
    }
}

void GripperClient::CallTargetWidth(uint8_t finger_target_width, uint8_t finger_width_speed, uint8_t gripping_force, uint8_t contact_detection_sesitivity)
{
    RCLCPP_INFO(this->get_logger(), "CallTargetWidth()...");
    if (!_target_width_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        return;
    }

    auto request = std::make_shared<art_gripper_interfaces::srv::TargetWidth::Request>();
    request->finger_target_width = finger_target_width;
    request->finger_width_speed = finger_width_speed;
    request->gripping_force = gripping_force;
    request->contact_detection_sesitivity = contact_detection_sesitivity;

    _target_width_client->async_send_request(
        request, std::bind(&GripperClient::OnTargetWidthResponse, this, _1));
}

void GripperClient::OnTargetWidthResponse(rclcpp::Client<art_gripper_interfaces::srv::TargetWidth>::SharedFuture future)
{
    try {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "service call result: %d", result->result);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service TargetWidth: %s", e.what());
    }
}

void GripperClient::CallTargetPose(uint8_t finger_target_pose, uint8_t finger_pose_speed)
{
    RCLCPP_INFO(this->get_logger(), "CallTargetPose()...");
    if (!_target_pose_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        return;
    }

    auto request = std::make_shared<art_gripper_interfaces::srv::TargetPose::Request>();
    request->finger_target_pose = finger_target_pose;
    request->finger_pose_speed = finger_pose_speed;

    _target_pose_client->async_send_request(
        request, std::bind(&GripperClient::OnTargetPoseResponse, this, _1));
}

void GripperClient::OnTargetPoseResponse(rclcpp::Client<art_gripper_interfaces::srv::TargetPose>::SharedFuture future)
{
    try {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "service call result: %d", result->result);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service TargetPose: %s", e.what());
    }
}

void GripperClient::CallSetTargetCurrent(int16_t target_current[4])
{
    RCLCPP_INFO(this->get_logger(), "CallSetTargetCurrent()...");
    if (!_set_target_current_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        return;
    }

    auto request = std::make_shared<art_gripper_interfaces::srv::SetTargetCurrent::Request>();
    for (int i = 0; i < 4; ++i) {
        request->target_current[i] = target_current[i];
    }

    _set_target_current_client->async_send_request(
        request, std::bind(&GripperClient::OnSetTargetCurrentResponse, this, _1));
}

void GripperClient::OnSetTargetCurrentResponse(rclcpp::Client<art_gripper_interfaces::srv::SetTargetCurrent>::SharedFuture future)
{
    try {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "service call result: %d", result->result);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service SetTargetCurrent: %s", e.what());
    }
}

void GripperClient::CallSetTarget(uint8_t finger_target_width, uint8_t finger_target_pose, uint8_t finger_width_speed, uint8_t finger_pose_speed, uint8_t gripping_force, uint8_t contact_detection_sesitivity)
{
    RCLCPP_INFO(this->get_logger(), "CallSetTarget()...");
    if (!_set_target_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        return;
    }

    auto request = std::make_shared<art_gripper_interfaces::srv::SetTarget::Request>();
    request->finger_target_width = finger_target_width;
    request->finger_target_pose = finger_target_pose;
    request->finger_width_speed = finger_width_speed;
    request->finger_pose_speed = finger_pose_speed;
    request->gripping_force = gripping_force;
    request->contact_detection_sesitivity = contact_detection_sesitivity;

    _set_target_client->async_send_request(
        request, std::bind(&GripperClient::OnSetTargetResponse, this, _1));
}

void GripperClient::OnSetTargetResponse(rclcpp::Client<art_gripper_interfaces::srv::SetTarget>::SharedFuture future)
{
    try {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "service call result: %d", result->result);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service SetTarget: %s", e.what());
    }
}

void GripperClient::CallResetFrictionModel()
{
    RCLCPP_INFO(this->get_logger(), "CallResetFrictionModel()...");
    if (!_reset_friction_model_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        return;
    }

    auto request = std::make_shared<art_gripper_interfaces::srv::ResetFrictionModel::Request>();

    _reset_friction_model_client->async_send_request(
        request, std::bind(&GripperClient::OnResetFrictionModelResponse, this, _1));
}

void GripperClient::OnResetFrictionModelResponse(rclcpp::Client<art_gripper_interfaces::srv::ResetFrictionModel>::SharedFuture future)
{
    try {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "service call result: %d", result->result);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service ResetFrictionModel: %s", e.what());
    }
}