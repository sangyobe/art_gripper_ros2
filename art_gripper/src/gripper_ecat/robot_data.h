#ifndef __ROBOT_DATA_H__
#define __ROBOT_DATA_H__

#include <mutex>
#include <cstdint>
#include <array>

// Based on art_gripper_interfaces/msg/GripperStatus.msg
class GripperStatus
{
public:
    uint8_t gripper_status = 0;
    uint8_t finger_width = 0;
    uint8_t finger_pose = 0;
    std::array<uint16_t, 4> status_word = {0, 0, 0, 0};
    std::array<int32_t, 4> position = {0, 0, 0, 0};
    std::array<int32_t, 4> position_auxiliary = {0, 0, 0, 0};
    std::array<int32_t, 4> velocity = {0, 0, 0, 0};
    std::array<int16_t, 4> current = {0, 0, 0, 0};
};

class RobotData
{
public:
    GripperStatus status;
    std::mutex mtx;
};

#endif // __ROBOT_DATA_H__