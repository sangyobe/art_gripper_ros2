#include <memory>
#include "gripper_client.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GripperClient>());
    rclcpp::shutdown();
    return 0;
}