#include <memory>
#include "gripper_ecat.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GripperEcat>());
    rclcpp::shutdown();
    return 0;
}