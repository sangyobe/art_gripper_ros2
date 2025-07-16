#include "gripper_client.h"
#include <chrono>
#include <functional>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

GripperClient::GripperClient()
: Node("gripper_client"), _working_count(0), _sequential_call_index(0)
{
    _gripper_control_publisher = this->create_publisher<art_gripper_interfaces::msg::GripperControl>("gripper_control", 10);
    _timer = this->create_wall_timer(3000ms, std::bind(&GripperClient::OnTimer, this));
    _motor_on_client = this->create_client<art_gripper_interfaces::srv::MotorOn>("motor_on");
    _reset_abs_encoder_client = this->create_client<art_gripper_interfaces::srv::ResetAbsEncoder>("reset_abs_encoder");
    _set_target_finger_width_client = this->create_client<art_gripper_interfaces::srv::SetTargetFingerWidth>("set_target_finger_width");
    _set_target_finger_pose_client = this->create_client<art_gripper_interfaces::srv::SetTargetFingerPose>("set_target_finger_pose");
    _set_target_current_client = this->create_client<art_gripper_interfaces::srv::SetTargetCurrent>("set_target_current");
    _set_target_client = this->create_client<art_gripper_interfaces::srv::SetTarget>("set_target");
    _reset_friction_model_client = this->create_client<art_gripper_interfaces::srv::ResetFrictionModel>("reset_friction_model");
    _gripper_info_client = this->create_client<art_gripper_interfaces::srv::GetGripperInfo>("get_gripper_info");
    _set_contact_sensitivity_client = this->create_client<art_gripper_interfaces::srv::SetContactSensitivity>("set_contact_sensitivity");
    _set_gripping_force_client = this->create_client<art_gripper_interfaces::srv::SetGrippingForce>("set_gripping_force");
    _set_target_finger_pose_speed_client = this->create_client<art_gripper_interfaces::srv::SetTargetFingerPoseSpeed>("set_target_finger_pose_speed");
    _set_target_finger_pose_with_speed_client = this->create_client<art_gripper_interfaces::srv::SetTargetFingerPoseWithSpeed>("set_target_finger_pose_with_speed");
    _set_target_finger_width_speed_client = this->create_client<art_gripper_interfaces::srv::SetTargetFingerWidthSpeed>("set_target_finger_width_speed");
    _set_target_finger_width_with_speed_client = this->create_client<art_gripper_interfaces::srv::SetTargetFingerWidthWithSpeed>("set_target_finger_width_with_speed");

    // Get gripper info
    this->CallGetGripperInfo(); 

    // Motor On
    this->CallMotorOn(true);

    // Populate sequential calls
    // _sequential_calls.push_back(std::bind(&GripperClient::CallMotorOn, this, true));
    // _sequential_calls.push_back(std::bind(&GripperClient::CallResetAbsEncoder, this));
    // _sequential_calls.push_back(std::bind(&GripperClient::CallResetFrictionModel, this));
    _sequential_calls.push_back(std::bind(&GripperClient::CallSetTargetFingerPoseWithSpeed, this, 180, 120));
    _sequential_calls.push_back(std::bind(&GripperClient::CallSetTargetFingerWidthWithSpeed, this, 0, 150));
    _sequential_calls.push_back(std::bind(&GripperClient::CallSetTargetFingerWidthWithSpeed, this, 100, 150));
    _sequential_calls.push_back(std::bind(&GripperClient::CallSetTargetFingerPoseWithSpeed, this, 90, 120));
    _sequential_calls.push_back(std::bind(&GripperClient::CallSetTargetFingerPoseWithSpeed, this, 0, 120));
    _sequential_calls.push_back(std::bind(&GripperClient::CallSetTargetFingerWidthWithSpeed, this, 0, 150));
    _sequential_calls.push_back(std::bind(&GripperClient::CallSetTargetFingerWidthWithSpeed, this, 100, 150));
    _sequential_calls.push_back(std::bind(&GripperClient::CallSetTargetFingerPoseWithSpeed, this, 90, 120));
    // _sequential_calls.push_back(std::bind(&GripperClient::CallSetContactSensitivity, this, 80));
    // _sequential_calls.push_back(std::bind(&GripperClient::CallSetGrippingForce, this, 30));
    // _sequential_calls.push_back(std::bind(&GripperClient::CallMotorOn, this, false));
    
    // _sequential_calls.push_back(std::bind(&GripperClient::CallSetTargetFingerPose, this, 90));
    // _sequential_calls.push_back(std::bind(&GripperClient::CallSetTargetFingerPoseSpeed, this, 180));
    // _sequential_calls.push_back(std::bind(&GripperClient::CallSetTargetFingerWidth, this, 50));
    // _sequential_calls.push_back(std::bind(&GripperClient::CallSetTargetFingerWidthSpeed, this, 150));
    // _sequential_calls.push_back(std::bind(&GripperClient::CallSetTarget, this, 25, 45, 75, 90, 20, 60));

    // _sequential_calls.push_back([this]() {
    //     int16_t current_array[4] = {100, 200, 300, 400};
    //     this->CallSetTargetCurrent(current_array);
    // });
}

void GripperClient::OnTimer()
{
    // Call one function from the sequential_calls list per timer tick
    if (!_sequential_calls.empty()) {
        _sequential_calls[_sequential_call_index]();
        _sequential_call_index = (_sequential_call_index + 1) % _sequential_calls.size();
    }

    // this->PublishGripperControl();

    _working_count++;
}

void GripperClient::PublishGripperControl()
{
    auto msg = art_gripper_interfaces::msg::GripperControl();
    msg.gripper_control = 1;
    msg.finger_width = 50;
    msg.finger_pose = 90;
    msg.finger_width_speed = 100;
    msg.finger_pose_speed = 150;
    msg.gripping_force = 30;
    msg.contact_sensitivity = 80;
    msg.target_current = {100, 200, 300, 400};
    _gripper_control_publisher->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing GripperControl message.");
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

void GripperClient::CallSetTargetFingerWidth(uint8_t finger_width)
{
    RCLCPP_INFO(this->get_logger(), "CallSetTargetFingerWidth()...");
    if (!_set_target_finger_width_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        return;
    }

    auto request = std::make_shared<art_gripper_interfaces::srv::SetTargetFingerWidth::Request>();
    request->finger_width = finger_width;

    _set_target_finger_width_client->async_send_request(
        request, std::bind(&GripperClient::OnSetTargetFingerWidthResponse, this, _1));
}

void GripperClient::OnSetTargetFingerWidthResponse(rclcpp::Client<art_gripper_interfaces::srv::SetTargetFingerWidth>::SharedFuture future)
{
    try {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "service call result: %d", result->result);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service SetTargetFingerWidth: %s", e.what());
    }
}

void GripperClient::CallSetTargetFingerPose(uint8_t finger_pose)
{
    RCLCPP_INFO(this->get_logger(), "CallSetTargetFingerPose()...");
    if (!_set_target_finger_pose_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        return;
    }

    auto request = std::make_shared<art_gripper_interfaces::srv::SetTargetFingerPose::Request>();
    request->finger_pose = finger_pose;

    _set_target_finger_pose_client->async_send_request(
        request, std::bind(&GripperClient::OnSetTargetFingerPoseResponse, this, _1));
}

void GripperClient::OnSetTargetFingerPoseResponse(rclcpp::Client<art_gripper_interfaces::srv::SetTargetFingerPose>::SharedFuture future)
{
    try {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "service call result: %d", result->result);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service SetTargetFingerPose: %s", e.what());
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

void GripperClient::CallSetTarget(uint8_t finger_width, uint8_t finger_pose, uint8_t finger_width_speed, uint8_t finger_pose_speed, uint8_t gripping_force, uint8_t contact_sensitivity)
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
    request->finger_width = finger_width;
    request->finger_pose = finger_pose;
    request->finger_width_speed = finger_width_speed;
    request->finger_pose_speed = finger_pose_speed;
    request->gripping_force = gripping_force;
    request->contact_sensitivity = contact_sensitivity;

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

void GripperClient::CallGetGripperInfo()
{
    RCLCPP_INFO(this->get_logger(), "CallGetGripperInfo()...");
    if (!_gripper_info_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        return;
    }

    auto request = std::make_shared<art_gripper_interfaces::srv::GetGripperInfo::Request>();

    _gripper_info_client->async_send_request(
        request, std::bind(&GripperClient::OnGetGripperInfoResponse, this, _1));
}

void GripperClient::OnGetGripperInfoResponse(rclcpp::Client<art_gripper_interfaces::srv::GetGripperInfo>::SharedFuture future)
{
    try {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "GetGripperInfo service call result: Name=%s, Version=%s, Description=%s", result->info.name.c_str(), result->info.version.c_str(), result->info.description.c_str());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service GetGripperInfo: %s", e.what());
    }
}

void GripperClient::CallSetContactSensitivity(uint8_t contact_sensitivity)
{
    RCLCPP_INFO(this->get_logger(), "CallSetContactSensitivity()...");
    if (!_set_contact_sensitivity_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        return;
    }

    auto request = std::make_shared<art_gripper_interfaces::srv::SetContactSensitivity::Request>();
    request->contact_sensitivity = contact_sensitivity;

    _set_contact_sensitivity_client->async_send_request(
        request, std::bind(&GripperClient::OnSetContactSensitivityResponse, this, _1));
}

void GripperClient::OnSetContactSensitivityResponse(rclcpp::Client<art_gripper_interfaces::srv::SetContactSensitivity>::SharedFuture future)
{
    try {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "service call result: %d", result->result);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service SetContactSensitivity: %s", e.what());
    }
}

void GripperClient::CallSetGrippingForce(uint8_t force)
{
    RCLCPP_INFO(this->get_logger(), "CallSetGrippingForce()...");
    if (!_set_gripping_force_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        return;
    }

    auto request = std::make_shared<art_gripper_interfaces::srv::SetGrippingForce::Request>();
    request->gripping_force = force;

    _set_gripping_force_client->async_send_request(
        request, std::bind(&GripperClient::OnSetGrippingForceResponse, this, _1));
}

void GripperClient::OnSetGrippingForceResponse(rclcpp::Client<art_gripper_interfaces::srv::SetGrippingForce>::SharedFuture future)
{
    try {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "service call result: %d", result->result);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service SetGrippingForce: %s", e.what());
    }
}

void GripperClient::CallSetTargetFingerPoseSpeed(uint8_t speed)
{
    RCLCPP_INFO(this->get_logger(), "CallSetTargetFingerPoseSpeed()...");
    if (!_set_target_finger_pose_speed_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        return;
    }

    auto request = std::make_shared<art_gripper_interfaces::srv::SetTargetFingerPoseSpeed::Request>();
    request->finger_pose_speed = speed;

    _set_target_finger_pose_speed_client->async_send_request(
        request, std::bind(&GripperClient::OnSetTargetFingerPoseSpeedResponse, this, _1));
}

void GripperClient::OnSetTargetFingerPoseSpeedResponse(rclcpp::Client<art_gripper_interfaces::srv::SetTargetFingerPoseSpeed>::SharedFuture future)
{
    try {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "service call result: %d", result->result);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service SetTargetFingerPoseSpeed: %s", e.what());
    }
}

void GripperClient::CallSetTargetFingerPoseWithSpeed(uint8_t pose, uint8_t speed)
{
    RCLCPP_INFO(this->get_logger(), "CallSetTargetFingerPoseWithSpeed()...");
    if (!_set_target_finger_pose_with_speed_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        return;
    }

    auto request = std::make_shared<art_gripper_interfaces::srv::SetTargetFingerPoseWithSpeed::Request>();
    request->finger_pose = pose;
    request->finger_pose_speed = speed;

    _set_target_finger_pose_with_speed_client->async_send_request(
        request, std::bind(&GripperClient::OnSetTargetFingerPoseWithSpeedResponse, this, _1));
}

void GripperClient::OnSetTargetFingerPoseWithSpeedResponse(rclcpp::Client<art_gripper_interfaces::srv::SetTargetFingerPoseWithSpeed>::SharedFuture future)
{
    try {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "service call result: %d", result->result);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service SetTargetFingerPoseWithSpeed: %s", e.what());
    }
}

void GripperClient::CallSetTargetFingerWidthSpeed(uint8_t speed)
{
    RCLCPP_INFO(this->get_logger(), "CallSetTargetFingerWidthSpeed()...");
    if (!_set_target_finger_width_speed_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        return;
    }

    auto request = std::make_shared<art_gripper_interfaces::srv::SetTargetFingerWidthSpeed::Request>();
    request->finger_width_speed = speed;

    _set_target_finger_width_speed_client->async_send_request(
        request, std::bind(&GripperClient::OnSetTargetFingerWidthSpeedResponse, this, _1));
}

void GripperClient::OnSetTargetFingerWidthSpeedResponse(rclcpp::Client<art_gripper_interfaces::srv::SetTargetFingerWidthSpeed>::SharedFuture future)
{
    try {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "service call result: %d", result->result);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service SetTargetFingerWidthSpeed: %s", e.what());
    }
}

void GripperClient::CallSetTargetFingerWidthWithSpeed(uint8_t width, uint8_t speed)
{
    RCLCPP_INFO(this->get_logger(), "CallSetTargetFingerWidthWithSpeed()...");
    if (!_set_target_finger_width_with_speed_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        return;
    }

    auto request = std::make_shared<art_gripper_interfaces::srv::SetTargetFingerWidthWithSpeed::Request>();
    request->finger_width = width;
    request->finger_width_speed = speed;

    _set_target_finger_width_with_speed_client->async_send_request(
        request, std::bind(&GripperClient::OnSetTargetFingerWidthWithSpeedResponse, this, _1));
}

void GripperClient::OnSetTargetFingerWidthWithSpeedResponse(rclcpp::Client<art_gripper_interfaces::srv::SetTargetFingerWidthWithSpeed>::SharedFuture future)
{
    try {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "service call result: %d", result->result);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service SetTargetFingerWidthWithSpeed: %s", e.what());
    }
}