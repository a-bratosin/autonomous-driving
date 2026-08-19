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

        #Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='mpc_map_bridge',
        #     arguments=['0','0','0','1.5708','0','0','map','mpc_frame']
        #),

        Node(
            package='data_scripts',
            node_executable='ekf_node.py',
            node_name='ekf_node',
            output='screen'
        ),

        Node(
            package='data_scripts',
            node_executable='mpc_node.py',
            node_name='mpc_node',
            output='screen'
        ),

        Node(
            package='data_scripts',
            node_executable='debug_plotter.py',
            node_name='debug_node',
            output='screen'
        )
    ])
