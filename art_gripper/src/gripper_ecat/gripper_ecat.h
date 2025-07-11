#ifndef __GRIPPER_ECAT_H__
#define __GRIPPER_ECAT_H__

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "art_gripper_interfaces/msg/gripper_control.hpp"
#include "art_gripper_interfaces/msg/gripper_status.hpp"

class GripperEcat : public rclcpp::Node
{
public:
    GripperEcat();

private:
    void OnMessage(const std_msgs::msg::String::SharedPtr msg) const;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscriber;
};

#endif // __GRIPPER_ECAT_H__