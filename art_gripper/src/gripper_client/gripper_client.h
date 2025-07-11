#ifndef __GRIPPER_CLIENT_H__
#define __GRIPPER_CLIENT_H__

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "art_gripper_interfaces/msg/gripper_control.hpp"
#include "art_gripper_interfaces/msg/gripper_status.hpp"

class GripperClient : public rclcpp::Node
{
public:
    GripperClient();

private:
    void OnTimer();

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
    size_t _count;

};

#endif // __GRIPPER_CLIENT_H__