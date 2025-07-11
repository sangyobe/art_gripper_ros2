#include "gripper_ecat.h"
#include <chrono>
using namespace std::chrono_literals;
using std::placeholders::_1;

GripperEcat::GripperEcat()
: Node("gripper_ecat_driver")
{
    _subscriber = this->create_subscription<std_msgs::msg::String>(
        "greeting", 10, std::bind(&GripperEcat::OnMessage, this, _1));
}

void GripperEcat::OnMessage(const std_msgs::msg::String::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "msg: %s", msg->data.c_str());
}