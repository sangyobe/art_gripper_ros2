#include "gripper_ecat.h"
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

// Global instance of RobotData
std::shared_ptr<RobotData> g_robot_data = std::make_shared<RobotData>();

GripperEcat::GripperEcat()
: Node("gripper_ecat_driver"), _is_running(true)
{
    _robot_data = g_robot_data;

    // Declare and get parameter for publish rate
    this->declare_parameter<int>("status_publish_rate_hz", 100);
    int publish_rate_hz = this->get_parameter("status_publish_rate_hz").as_int();

    _gripper_control_subscriber = this->create_subscription<art_gripper_interfaces::msg::GripperControl>(
        "GripperControl", 10, std::bind(&GripperEcat::OnGripperControl, this, _1));

    _motor_on_server = this->create_service<art_gripper_interfaces::srv::MotorOn>(
        "MotorOn", std::bind(&GripperEcat::OnMotorOn, this, _1, _2));

    _reset_abs_encoder_server = this->create_service<art_gripper_interfaces::srv::ResetAbsEncoder>(
        "ResetAbsEncoder", std::bind(&GripperEcat::OnResetAbsEncoder, this, _1, _2));

    _set_target_width_server = this->create_service<art_gripper_interfaces::srv::SetTargetWidth>(
        "SetTargetWidth", std::bind(&GripperEcat::OnSetTargetWidth, this, _1, _2));

    _set_target_pose_server = this->create_service<art_gripper_interfaces::srv::SetTargetPose>(
        "SetTargetPose", std::bind(&GripperEcat::OnSetTargetPose, this, _1, _2));

    _set_target_current_server = this->create_service<art_gripper_interfaces::srv::SetTargetCurrent>(
        "SetTargetCurrent", std::bind(&GripperEcat::OnSetTargetCurrent, this, _1, _2));

    _set_target_server = this->create_service<art_gripper_interfaces::srv::SetTarget>(
        "SetTarget", std::bind(&GripperEcat::OnSetTarget, this, _1, _2));

    _reset_friction_model_server = this->create_service<art_gripper_interfaces::srv::ResetFrictionModel>(
        "ResetFrictionModel", std::bind(&GripperEcat::OnResetFrictionModel, this, _1, _2));

    _gripper_info_server = this->create_service<art_gripper_interfaces::srv::GetGripperInfo>(
        "GetGripperInfo", std::bind(&GripperEcat::OnGetGripperInfo, this, _1, _2));

    _status_publisher = this->create_publisher<art_gripper_interfaces::msg::GripperStatus>("GripperStatus", 10);

    _status_thread = std::thread(&GripperEcat::StatusPublishThread, this);
    RCLCPP_INFO(this->get_logger(), "GripperEcat node has been initialized with a status publish rate of %d Hz.", publish_rate_hz);
}

GripperEcat::~GripperEcat()
{
    _is_running = false;
    if (_status_thread.joinable()) {
        _status_thread.join();
    }
}

void GripperEcat::OnGripperControl(const art_gripper_interfaces::msg::GripperControl::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(_robot_data->mtx);
    RCLCPP_INFO(this->get_logger(), "Received GripperControl: control_word=%d, finger_target_width=%d", msg->control_word, msg->finger_target_width);
    _robot_data->control.control_word = msg->control_word;
    _robot_data->control.finger_target_width = msg->finger_target_width;
    _robot_data->control.finger_target_pose = msg->finger_target_pose;
    _robot_data->control.finger_width_speed = msg->finger_width_speed;
    _robot_data->control.finger_pose_speed = msg->finger_pose_speed;
    _robot_data->control.gripping_force = msg->gripping_force;
    _robot_data->control.contact_detection_sesitivity = msg->contact_detection_sesitivity;
    for (int i = 0; i < 4; ++i) {
        _robot_data->control.target_current[i] = msg->target_current[i];
    }
}

void GripperEcat::OnMotorOn(
    const std::shared_ptr<art_gripper_interfaces::srv::MotorOn::Request> request,
    std::shared_ptr<art_gripper_interfaces::srv::MotorOn::Response> response)
{
    std::lock_guard<std::mutex> lock(_robot_data->mtx);
    if (request->on) {
        RCLCPP_INFO(this->get_logger(), "motor on");
        // Example of modifying shared data
        _robot_data->status.gripper_status = 1; // In-motion for example
    } else {
        RCLCPP_INFO(this->get_logger(), "motor off");
        _robot_data->status.gripper_status = 0; // Ready for example
    }
    response->result = 0;
}

void GripperEcat::OnResetAbsEncoder(
    [[maybe_unused]] const std::shared_ptr<art_gripper_interfaces::srv::ResetAbsEncoder::Request> request,
    std::shared_ptr<art_gripper_interfaces::srv::ResetAbsEncoder::Response> response)
{
    std::lock_guard<std::mutex> lock(_robot_data->mtx);
    RCLCPP_INFO(this->get_logger(), "reset abs encoder");
    // Example of modifying shared data
    _robot_data->status.gripper_status = 2; // In-motion for example
    response->result = 0;
}

void GripperEcat::OnSetTargetWidth(
    const std::shared_ptr<art_gripper_interfaces::srv::SetTargetWidth::Request> request,
    std::shared_ptr<art_gripper_interfaces::srv::SetTargetWidth::Response> response)
{
    std::lock_guard<std::mutex> lock(_robot_data->mtx);
    RCLCPP_INFO(this->get_logger(), "SetTargetWidth: %d, %d, %d, %d", request->finger_target_width, request->finger_width_speed, request->gripping_force, request->contact_detection_sesitivity);
    response->result = 0;
}

void GripperEcat::OnSetTargetPose(
    const std::shared_ptr<art_gripper_interfaces::srv::SetTargetPose::Request> request,
    std::shared_ptr<art_gripper_interfaces::srv::SetTargetPose::Response> response)
{
    std::lock_guard<std::mutex> lock(_robot_data->mtx);
    RCLCPP_INFO(this->get_logger(), "SetTargetPose: %d, %d", request->finger_target_pose, request->finger_pose_speed);
    response->result = 0;
}

void GripperEcat::OnSetTargetCurrent(
    const std::shared_ptr<art_gripper_interfaces::srv::SetTargetCurrent::Request> request,
    std::shared_ptr<art_gripper_interfaces::srv::SetTargetCurrent::Response> response)
{
    std::lock_guard<std::mutex> lock(_robot_data->mtx);
    RCLCPP_INFO(this->get_logger(), "SetTargetCurrent: %d, %d, %d, %d", request->target_current[0], request->target_current[1], request->target_current[2], request->target_current[3]);
    response->result = 0;
}

void GripperEcat::OnSetTarget(
    const std::shared_ptr<art_gripper_interfaces::srv::SetTarget::Request> request,
    std::shared_ptr<art_gripper_interfaces::srv::SetTarget::Response> response)
{
    std::lock_guard<std::mutex> lock(_robot_data->mtx);
    RCLCPP_INFO(this->get_logger(), "SetTarget: %d, %d, %d, %d, %d, %d", request->finger_target_width, request->finger_target_pose, request->finger_width_speed, request->finger_pose_speed, request->gripping_force, request->contact_detection_sesitivity);
    response->result = 0;
}

void GripperEcat::OnResetFrictionModel(
    [[maybe_unused]] const std::shared_ptr<art_gripper_interfaces::srv::ResetFrictionModel::Request> request,
    std::shared_ptr<art_gripper_interfaces::srv::ResetFrictionModel::Response> response)
{
    std::lock_guard<std::mutex> lock(_robot_data->mtx);
    RCLCPP_INFO(this->get_logger(), "ResetFrictionModel");
    response->result = 0;
}

void GripperEcat::OnGetGripperInfo(
    [[maybe_unused]] const std::shared_ptr<art_gripper_interfaces::srv::GetGripperInfo::Request> request,
    std::shared_ptr<art_gripper_interfaces::srv::GetGripperInfo::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "GetGripperInfo service called.");
    response->info.name = "ART Gripper";
    response->info.version = "1.0.0";
    response->info.description = "HMC ART Gripper control system.";
}

void GripperEcat::StatusPublishThread()
{
    int publish_rate_hz = this->get_parameter("status_publish_rate_hz").as_int();
    rclcpp::Rate rate(publish_rate_hz);

    while (rclcpp::ok() && _is_running)
    {
        auto msg = art_gripper_interfaces::msg::GripperStatus();
        {
            std::lock_guard<std::mutex> lock(_robot_data->mtx);
            // Copy data from the custom GripperStatus to the ROS message
            msg.gripper_status = _robot_data->status.gripper_status;
            msg.finger_width = _robot_data->status.finger_width;
            msg.finger_pose = _robot_data->status.finger_pose;
            msg.status_word = _robot_data->status.status_word;
            msg.position = _robot_data->status.position;
            msg.position_auxiliary = _robot_data->status.position_auxiliary;
            msg.velocity = _robot_data->status.velocity;
            msg.current = _robot_data->status.current;
        }
        _status_publisher->publish(msg);
        rate.sleep();
    }
}