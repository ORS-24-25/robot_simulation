import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Creating launch argument for Frame ID to associate depth camera data
    # SET frame_id IN THE LAUNCH ARGUMENTS (Note: this will be formatted as <your_frame_id>_link)
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera', # In this case, RealSense will name the camera frames as camera_link ,etc.
        description='Frame ID to associate with the depth camera'
    )

    # Path to the RealSense launch file
    realsense_launch_file = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    # Include RealSense launch file with frame_id param
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'camera_name': LaunchConfiguration('frame_id'),
            'pointcloud.enable': 'true'
        }.items()
    )

    return LaunchDescription([
        frame_id_arg,
        realsense_launch,
    ])