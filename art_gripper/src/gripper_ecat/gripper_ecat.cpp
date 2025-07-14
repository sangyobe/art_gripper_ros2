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

    _subscriber = this->create_subscription<std_msgs::msg::String>(
        "greeting", 10, std::bind(&GripperEcat::OnMessage, this, _1));

    _motor_on_server = this->create_service<art_gripper_interfaces::srv::MotorOn>(
        "MotorOn", std::bind(&GripperEcat::OnMotorOn, this, _1, _2));

    _reset_abs_encoder_server = this->create_service<art_gripper_interfaces::srv::ResetAbsEncoder>(
        "ResetAbsEncoder", std::bind(&GripperEcat::OnResetAbsEncoder, this, _1, _2));

    _target_width_server = this->create_service<art_gripper_interfaces::srv::TargetWidth>(
        "TargetWidth", std::bind(&GripperEcat::OnTargetWidth, this, _1, _2));

    _target_pose_server = this->create_service<art_gripper_interfaces::srv::TargetPose>(
        "TargetPose", std::bind(&GripperEcat::OnTargetPose, this, _1, _2));

    _set_target_current_server = this->create_service<art_gripper_interfaces::srv::SetTargetCurrent>(
        "SetTargetCurrent", std::bind(&GripperEcat::OnSetTargetCurrent, this, _1, _2));

    _set_target_server = this->create_service<art_gripper_interfaces::srv::SetTarget>(
        "SetTarget", std::bind(&GripperEcat::OnSetTarget, this, _1, _2));

    _reset_friction_model_server = this->create_service<art_gripper_interfaces::srv::ResetFrictionModel>(
        "ResetFrictionModel", std::bind(&GripperEcat::OnResetFrictionModel, this, _1, _2));

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

void GripperEcat::OnMessage(const std_msgs::msg::String::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "msg: %s", msg->data.c_str());
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

void GripperEcat::OnTargetWidth(
    const std::shared_ptr<art_gripper_interfaces::srv::TargetWidth::Request> request,
    std::shared_ptr<art_gripper_interfaces::srv::TargetWidth::Response> response)
{
    std::lock_guard<std::mutex> lock(_robot_data->mtx);
    RCLCPP_INFO(this->get_logger(), "TargetWidth: %d, %d, %d, %d", request->finger_target_width, request->finger_width_speed, request->gripping_force, request->contact_detection_sesitivity);
    response->result = 0;
}

void GripperEcat::OnTargetPose(
    const std::shared_ptr<art_gripper_interfaces::srv::TargetPose::Request> request,
    std::shared_ptr<art_gripper_interfaces::srv::TargetPose::Response> response)
{
    std::lock_guard<std::mutex> lock(_robot_data->mtx);
    RCLCPP_INFO(this->get_logger(), "TargetPose: %d, %d", request->finger_target_pose, request->finger_pose_speed);
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