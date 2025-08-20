#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    # create and return launch description object
    # ros2 run art_gripper gripper_ecat --ros-args -p gripper_status_publish_rate_hz:=1 -p ethercat_state_publish_rate_hz:=1
    return LaunchDescription(
        [
            Node(
                package="art_gripper",
                executable="gripper_ecat",
                name="gripper_ecat_driver_left",
                output="screen",
                parameters=[
                    {"gripper_status_publish_rate_hz": 1},
                    {"ethercat_state_publish_rate_hz": 1},
                ],
                arguments=["--master", "0", "--ros-args", "-r", "__ns:=/ag_left"],
            ),
            Node(
                package="art_gripper",
                executable="gripper_ecat",
                name="gripper_ecat_driver_right",
                output="screen",
                parameters=[
                    {"gripper_status_publish_rate_hz": 1},
                    {"ethercat_state_publish_rate_hz": 1},
                ],
                arguments=["--master", "1", "--ros-args", "-r", "__ns:=/ag_right"],
            )
        ]
    )