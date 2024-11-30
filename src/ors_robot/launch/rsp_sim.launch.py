import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'ors_robot'
    file_subpath = 'urdf/robot.urdf.xacro'


    # Add sim launch argument
    sim_arg = DeclareLaunchArgument(
        'sim', 
        default_value='false',
        description='Use simulated config (not using M2M2)',
    )

    # Add gazebo world launch argument
    world_arg = DeclareLaunchArgument(
        'world', 
        default_value='src/ors_robot/worlds/test.world',
        description='Gazebo world file',
    )

    # Add slam_params_file arg for slam_toolbox
    slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value='src/ors_robot/config/mapper_params_online_async.yaml',
        description='Path to the ROS2 parameters file for the SLAM Toolbox',
    )

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description_raw,
                'use_sim_time': True
            }
        ] # add other parameters here if required
    )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                    '-entity', 'ors_robot'],
        output='screen')

    # Launch slam_toolbox node
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch'), '/online_async_launch.py']),
        launch_arguments={
            'params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': 'true'
        }.items(),
        condition=IfCondition(LaunchConfiguration('sim'))
    )


    slamtec_publisher = Node(
        package='slamtec_publisher',
        executable='slamtec_publisher',
        # name='slamtec_publisher_node',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('sim')),
    )

    # TODO: Add depth cam realsense node with launch condition

    # Run the node
    return LaunchDescription([
        world_arg,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        sim_arg,
        slamtec_publisher,
        slam_params_file,
        slam_toolbox,
        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=slamtec_publisher,
        #         on_exit=[
        #             EmitEvent(event=Shutdown(
        #                     reason="Slamtec Publisher failed"
        #             ))
        #         ]
        #     )
        # )
    ])
