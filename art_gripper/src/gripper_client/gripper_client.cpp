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

void GripperClient::OnMotorOnResponse(rclcpp::Client<art_gripper_interfaces::srv::MotorOn>::SharedFuture future)
{
    try {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "service call result: %d", result->result);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service MotorOn: %s", e.what());
    }
}