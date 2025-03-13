#!/usr/bin/env python3
# Copyright 2024 iRobot Corporation. All Rights Reserved.

# Copied from create3 and modified

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    republisher_ns = LaunchConfiguration('republisher_ns')
    republisher_ns_argument = DeclareLaunchArgument(
        'republisher_ns', 
        default_value='/',
        description='Republisher namespace')
    
    robot_ns = LaunchConfiguration('robot_ns')
    robot_ns_argument = DeclareLaunchArgument(
        'robot_ns', 
        default_value='/ors_irobot',
        description='Create 3 robot namespace')

    start_republisher_node = Node(
        parameters=[
            get_package_share_directory("ors_robot") + '/config/create3_republisher_params.yaml',
            {'robot_namespace': robot_ns}
        ],
        package='ors_robot',
        executable='create3_republisher',
        name='create3_repub',
        output='screen',
        namespace=republisher_ns,
    )

    ld = LaunchDescription()

    ld.add_action(republisher_ns_argument)
    ld.add_action(robot_ns_argument)
    ld.add_action(start_republisher_node)

    return ld
