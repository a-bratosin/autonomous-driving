import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Path to sllidar launch file
    sllidar_launch_dir = get_package_share_directory('sllidar_ros2')
    sllidar_launch_file = os.path.join(sllidar_launch_dir, 'launch', 'sllidar_a1_launch.py')

    # Path to rf2o launch file
    rf2o_launch_dir = get_package_share_directory('rf2o_laser_odometry')
    rf2o_launch_file = os.path.join(rf2o_launch_dir, 'launch', 'rf2o_laser_odometry.launch.py')

    return LaunchDescription([
        # 1. Start LiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch_file)
        ),

        # 2. Start Odometry
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rf2o_launch_file)
        ),

        # 3. Start ESP32 Serial Bridge (Legatura cu microcontrolerul)
        Node(
            package='data_scripts',
            node_executable='esp_bridge_node.py',
            node_name='esp_bridge_node',
            output='screen'
        ),

        # 4. Start EKF
        Node(
            package='data_scripts',
            node_executable='ekf_node.py',
            node_name='ekf_node',
            output='screen'
        ),

        # 5. Start MPC
        Node(
            package='data_scripts',
            node_executable='mpc_node.py',
            node_name='mpc_node',
            output='screen'
        ),

        # 6. Start Plotter
        Node(
            package='data_scripts',
            node_executable='debug_plotter.py',
            node_name='debug_node',
            output='screen'
        )
    ])
