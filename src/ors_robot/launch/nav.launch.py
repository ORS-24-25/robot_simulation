import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )

    # Add nav2_params arg for nav2
    nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file',
        default_value='src/ors_robot/config/nav2_params.yaml',
        description='Path to the ROS2 parameters file for the Nav2 stack',
    )

    # Launch nav2 node
    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch'), '/navigation_launch.py'
        ]),
        launch_arguments={
            'params_file': LaunchConfiguration('nav2_params_file'),
            'use_sim_time': use_sim_time
        }.items()
    )

    ld = LaunchDescription()

    ld.add_action(nav2_params_file)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(nav2_node)

    # Launches all named actions
    return ld
