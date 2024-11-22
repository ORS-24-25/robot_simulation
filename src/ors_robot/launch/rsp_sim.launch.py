import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'ors_robot'
    # file_subpath = 'urdf/robot.urdf.xacro'
    file_subpath = 'urdf/robot.urdf.xacro'


    # Add launch argument
    sim_arg = DeclareLaunchArgument(
        'sim', 
        default_value='false',
        description='Use simulated config (not using M2M2)',
    )


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    # urdf_file_path = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    # with open(urdf_file_path, 'r') as urdf_file:
    #     robot_description_raw = urdf_file.read()


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                    '-entity', 'ors_robot'],
        output='screen')


    slamtec_publisher = Node(
        package='slamtec_publisher',
        executable='slamtec_publisher',
        # name='slamtec_publisher_node',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('sim')),
    )


    # Run the node
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        sim_arg,
        slamtec_publisher,
    ])
