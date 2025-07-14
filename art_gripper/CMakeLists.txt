cmake_minimum_required(VERSION 3.8)
project(art_gripper)

# default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(art_gripper_interfaces REQUIRED)

# add ethercat driver node
file(GLOB SRCS "src/gripper_ecat/*.cpp")
add_executable(gripper_ecat ${SRCS})
ament_target_dependencies(gripper_ecat rclcpp std_msgs art_gripper_interfaces)
target_include_directories(gripper_ecat PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/gripper_ecat>
  $<INSTALL_INTERFACE:include/gripper_ecat>)

# add test client target
file(GLOB SRCS "src/gripper_client/*.cpp")
add_executable(gripper_client ${SRCS})
ament_target_dependencies(gripper_client rclcpp std_msgs art_gripper_interfaces)

# install target
install(TARGETS
  gripper_ecat
  gripper_client
  DESTINATION lib/${PROJECT_NAME})

install(
    FILES src/gripper_ecat/robot_data.h
    DESTINATION include/gripper_ecat
)

ament_package()