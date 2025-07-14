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