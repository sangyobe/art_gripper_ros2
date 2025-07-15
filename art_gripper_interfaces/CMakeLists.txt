cmake_minimum_required(VERSION 3.8)
project(art_gripper_interfaces)

# default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate code
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GripperControl.msg"
  "msg/GripperStatus.msg"
  "msg/GripperInfo.msg"
  "srv/MotorOn.srv"
  "srv/ResetAbsEncoder.srv"
  "srv/SetContactSensitivity.srv"
  "srv/SetGrippingForce.srv"
  "srv/SetTargetFingerWidth.srv"
  "srv/SetTargetFingerWidthSpeed.srv"
  "srv/SetTargetFingerWidthWithSpeed.srv"
  "srv/SetTargetFingerPose.srv"
  "srv/SetTargetFingerPoseSpeed.srv"
  "srv/SetTargetFingerPoseWithSpeed.srv"
  "srv/SetTargetCurrent.srv"
  "srv/SetTarget.srv"
  "srv/ResetFrictionModel.srv"
  "srv/GetGripperInfo.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()