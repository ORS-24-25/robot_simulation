import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description() -> LaunchDescription:
    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'ors_robot'
    file_subpath = 'urdf/robot.urdf.xacro'

    # Add sim launch argument
    sim_arg = DeclareLaunchArgument(
        'sim', 
        default_value='false',
        description='Use simulated config (not using M2M2)',
    )

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure the robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description_raw,
                'use_sim_time': LaunchConfiguration('sim')
            }
        ]
    )

    # Run the node
    return LaunchDescription([
        sim_arg,
        node_robot_state_publisher,
    ]
    )
