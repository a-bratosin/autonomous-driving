import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Construct the full path to the sllidar launch file
    sllidar_launch_file = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'sllidar_a1_launch.py'
    )

    return LaunchDescription([
        # Include the lidar launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch_file)
        ),
        
        # Start the motor controller node
        Node(
            package='io_scripts',
            executable='motor_controller.py',
            name='control_motoare'
        ),
        
        # Start the IMU publisher node
        Node(
            package='io_scripts',
            executable='imu_publisher.py',
            name='imu_publisher'
        )
    ])
