#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

'''
Parameter Description:
---
- Set laser scan directon: 
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop fuction:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': False}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degress.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degress.
'''

def generate_launch_description():
    # Evaluate at launch the value of the launch configuration 'namespace'
    namespace = LaunchConfiguration('namespace')

    # Declares an action to allow users to pass the robot namespace from the
    # CLI into the launch description as an argument.
    namespace_argument = DeclareLaunchArgument(
        'namespace',
        default_value='/ors_irobot',
        description='Robot namespace'
    )

    # LDROBOT LiDAR publisher node
    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='STL27L',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_STL27L'},
            {'topic_name': 'scan'},
            {'frame_id': 'laser_frame'},
            {'port_name': '/dev/ttyUSB0'},
            {'port_baudrate': 921600},
            {'laser_scan_dir': False},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 0.0},
            {'angle_crop_max': 0.0}
        ],
        namespace=namespace
    )

    # plate to laser_frame tf node
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser_stl27l',
        arguments=['0','0','0.18','0','0','0','base_footprint','laser_frame']
    )

    # Launch with a timer between transform and ldlidar_node
    return LaunchDescription([
        namespace_argument,
        static_transform_node,
        TimerAction(
            period=2.0,
            actions=[ldlidar_node]
        )
    ])