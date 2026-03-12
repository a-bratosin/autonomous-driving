import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the rf2o launch file
    rf2o_launch_dir = get_package_share_directory('rf2o_laser_odometry')
    rf2o_launch_file = os.path.join(rf2o_launch_dir, 'launch', 'rf2o_laser_odometry.launch.py')

    return LaunchDescription([
        # 1. Include the rf2o launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rf2o_launch_file)
        ),

        # 2. Launch ekf_node.py
        Node(
            package='data_scripts',
            executable='ekf_node.py',
            name='ekf_node',
            output='screen'
        ),

        # 3. Launch mpc_node.py
        Node(
            package='data_scripts',
            executable='mpc_node.py',
            name='mpc_node',
            output='screen'
        )
    ])
