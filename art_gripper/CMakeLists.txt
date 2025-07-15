cmake_minimum_required(VERSION 3.10)
project(art_gripper)

# default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-missing-field-initializers -Wno-unused-but-set-variable -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(art_gripper_interfaces REQUIRED)

list(APPEND CMAKE_PREFIX_PATH "/opt/etherlab")
find_package(EtherCAT REQUIRED)
# if(NOT "${EtherCAT_FOUND}")
#     message(FATAL_ERROR "ament_target_dependencies() the passed package name 'EtherCAT' was not found before")
# else()
#     message(FATAL_ERROR "'EtherCAT' was found. ${EtherCAT_LIBRARY} ${EtherCAT_INCLUDE_DIR}")
#     message(STATUS "${EtherCAT_LIBRARY}")
#     message(STATUS "${EtherCAT_INCLUDE_DIR}")
# endif()
# set(EtherCAT_Targets "EtherLab::ethercat")

# add ethercat driver node
file(GLOB SRCS 
  "src/gripper_ecat/*.cpp" 
  "src/gripper_ecat/device/*.cpp" 
  "src/gripper_ecat/thread/*.cpp"
  "src/gripper_ecat/util/*.cpp")
add_executable(gripper_ecat ${SRCS})
ament_target_dependencies(gripper_ecat PUBLIC rclcpp std_msgs art_gripper_interfaces EtherCAT)
target_link_libraries(gripper_ecat PUBLIC EtherLab::ethercat)
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

ament_package()