import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = get_package_share_directory('soarm_navigation')
    simulation_share = get_package_share_directory('soarm_simulation')
    slam_share = get_package_share_directory('slam_toolbox')
    slam_params = os.path.join(navigation_share, 'config', 'hm3d_slam.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='apartment_a'),
        DeclareLaunchArgument('gz_args', default_value='-r -s'),
        DeclareLaunchArgument('rviz', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([simulation_share, 'launch', 'simulation.launch.py'])),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'gz_args': LaunchConfiguration('gz_args'),
                'rviz': LaunchConfiguration('rviz'),
                'publish_static_map': 'false',
                'ground_truth_map_tf': 'false',
            }.items()),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='mid360_scan_frame_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'livox_frame',
                       'kuruma/livox_frame/mid360_approx'],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([slam_share, 'launch', 'online_async_launch.py'])),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_params_file': slam_params,
            }.items()),
    ])
