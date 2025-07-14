#ifndef __GRIPPER_CLIENT_H__
#define __GRIPPER_CLIENT_H__

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "art_gripper_interfaces/msg/gripper_control.hpp"
#include "art_gripper_interfaces/msg/gripper_status.hpp"
#include "art_gripper_interfaces/srv/motor_on.hpp"

class GripperClient : public rclcpp::Node
{
public:
    GripperClient();
    void CallMotorOn(const bool on);

private:
    void OnTimer();
    void OnMotorOnResponse(rclcpp::Client<art_gripper_interfaces::srv::MotorOn>::SharedFuture future);

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
    rclcpp::Client<art_gripper_interfaces::srv::MotorOn>::SharedPtr _motor_on_client;
    size_t _count;

};

#endif // __GRIPPER_CLIENT_H__