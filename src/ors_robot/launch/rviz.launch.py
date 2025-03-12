#!/usr/bin/env python3
# Copyright 2023 iRobot Corporation. All Rights Reserved.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Gets the directory of the package and stores it as 'lidar_pkg'
    lidar_pkg = get_package_share_directory('ors_robot')

    # Generate the path to the rviz configuration file
    rviz2_config = PathJoinSubstitution(
        [lidar_pkg, 'rviz', 'ors_robot.rviz'])

    rviz_node = Node(
        package='rviz2', 
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config],
        output='screen'
        )

    return LaunchDescription([
        rviz_node
    ])
