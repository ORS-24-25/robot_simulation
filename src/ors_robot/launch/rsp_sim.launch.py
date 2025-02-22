import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

"""
Launch file for robot simulation.

This launch file sets up the robot simulation environment, including the robot state publisher,
Gazebo simulation, SLAM toolbox, and Slamtec publisher.

Functions:
    generate_launch_description(): Generates the launch description for the robot simulation.

Launch Arguments:
    sim (bool): Use simulated config (default: false). Select with sim:=<bool>.
    world (str): Path to the Gazebo world file (default: src/ors_robot/worlds/test.world).
    slam_params_file (str): Path to the ROS2 parameters file for the SLAM Toolbox (default: src/ors_robot/config/mapper_params_online_async.yaml).

Nodes:
    robot_state_publisher: Publishes the robot state to the /robot_description topic.
    gazebo: Launches the Gazebo simulation environment.
    spawn_entity: Spawns the robot entity in the Gazebo simulation.
    slam_toolbox: Launches the SLAM toolbox for mapping and localization.
    slamtec_publisher: Publishes data from the Slamtec sensor (only if sim is false).
"""
def generate_launch_description() -> LaunchDescription:
    """
    Generate the launch description for the ORS robot simulation.
    This function sets up the launch configuration for the ORS robot simulation,
    including the robot state publisher, Gazebo simulation, SLAM toolbox, and
    Slamtec publisher. It declares necessary launch arguments and includes
    relevant nodes and launch files.
    Returns:
        LaunchDescription: The launch description containing all the nodes and
        configurations for the simulation.
    """

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'ors_robot'
    file_subpath = 'urdf/robot.urdf.xacro'

    # Add sim launch argument
    sim_arg = DeclareLaunchArgument(
        'sim', 
        default_value='false',
        description='Use simulated config (not using M2M2)',
    )

    slam_arg = DeclareLaunchArgument(
        'slam_mode',
        default_value='localization',
        description='Set slam_toolbox mode (mapping/localization)',
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

    # Add nav2_params arg for nav2
    nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file',
        default_value='src/ors_robot/config/nav2_params.yaml',
        description='Path to the ROS2 parameters file for the Nav2 stack',
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
                'use_sim_time': LaunchConfiguration('sim')
            }
        ] # add other parameters here if required
    )

    # Only define Gazebo stuff if 'sim' is true
    gazebo_includes = []
    try:
        # Will throw PackageNotFoundError if gazebo_ros not installed
        gazebo_ros_dir = get_package_share_directory('gazebo_ros')

        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')]
            ),
            launch_arguments={'world': LaunchConfiguration('world')}.items(),
            condition=IfCondition(LaunchConfiguration('sim'))
        )

        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'ors_robot'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('sim'))
        )

        gazebo_includes = [gazebo, spawn_entity]
    except PackageNotFoundError:
        # Gazebo not installed, so skip
        pass

    # Launch slam_toolbox node
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch'), '/online_async_launch.py'
        ]),
        launch_arguments={
            'params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': LaunchConfiguration('sim'),
            'mode': LaunchConfiguration('slam_mode')
        }.items()
    )

    # Twist mux node
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[{
            'params_file': os.path.join(get_package_share_directory(pkg_name), 'config', 'twist_mux.yaml'),
            'use_sim_time': LaunchConfiguration('sim')
        }]
    )

    # Launch nav2 node
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch'), '/navigation_launch.py'
        ]),
        launch_arguments={
            'params_file': LaunchConfiguration('nav2_params_file'),
            'use_sim_time': LaunchConfiguration('sim')
        }.items(),
        condition=IfCondition(
            PythonExpression(['\'localization\' in "', LaunchConfiguration('slam_mode'), '"'])
        )
    )

    # Launch rviz2 node configured to check laser scan data
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory(pkg_name), 'rviz', 'ors_robot.rviz')],
    )

    ld_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ldlidar_stl_ros2'),  # name of the lidar package
                'launch',
                'stl27l.launch.py'
            )
        ]),
        condition=UnlessCondition(LaunchConfiguration('sim'))
    )

    tf2_odom_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_footprint_broadcaster',
        arguments=[
            '0', '0', '0',      # x, y, z
            '0', '0', '0',      # roll, pitch, yaw
            'odom',
            'base_link'
        ],
        condition=UnlessCondition(LaunchConfiguration('sim'))
    )

    # Run the node
    return LaunchDescription([
        world_arg,
        sim_arg,
        slam_arg,
        node_robot_state_publisher,
        tf2_odom_broadcaster,
        twist_mux,
        slam_params_file,
        slam_toolbox,
        nav2_params_file,
        nav2,
        ld_lidar,
        rviz2,
    ] + gazebo_includes)
