#include "gripper_client.h"
#include <chrono>
using namespace std::chrono_literals;
using std::placeholders::_1;

GripperClient::GripperClient()
: Node("gripper_client"), _count(0)
{
    _publisher = this->create_publisher<std_msgs::msg::String>("greeting", 10);
    _timer = this->create_wall_timer(100ms, std::bind(&GripperClient::OnTimer, this));
}

void GripperClient::OnTimer()
{
    auto msg = std_msgs::msg::String();
    msg.data = "Hello " + std::to_string(_count++);
    RCLCPP_INFO(this->get_logger(), "pub: %s", msg.data.c_str());
    _publisher->publish(msg);
}