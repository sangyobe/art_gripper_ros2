#ifndef __GRIPPER_ECAT_H__
#define __GRIPPER_ECAT_H__

#include <memory>
#include <thread>
#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "art_gripper_interfaces/msg/gripper_control.hpp"
#include "art_gripper_interfaces/msg/gripper_status.hpp"
#include "art_gripper_interfaces/srv/motor_on.hpp"
#include "robot_data.h"

class GripperEcat : public rclcpp::Node
{
public:
    GripperEcat();
    ~GripperEcat();

private:
    void OnMessage(const std_msgs::msg::String::SharedPtr msg) const;
    void OnMotorOn(
        const std::shared_ptr<art_gripper_interfaces::srv::MotorOn::Request> request,
        std::shared_ptr<art_gripper_interfaces::srv::MotorOn::Response> response);
    void StatusPublishThread();

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscriber;
    rclcpp::Service<art_gripper_interfaces::srv::MotorOn>::SharedPtr _motor_on_server;
    rclcpp::Publisher<art_gripper_interfaces::msg::GripperStatus>::SharedPtr _status_publisher;

    std::shared_ptr<RobotData> _robot_data;
    std::thread _status_thread;
    std::atomic<bool> _is_running;
};

#endif // __GRIPPER_ECAT_H__