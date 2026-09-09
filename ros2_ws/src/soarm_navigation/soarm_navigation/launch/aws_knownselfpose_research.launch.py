"""Research-only Nav2 stack for a sensor-built AWS Small House map.

The source-pose publisher and survey live in the pilot harness. This launch
intentionally owns neither Gazebo nor any localization/FAST-LIO process.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('soarm_navigation')
    nav2 = get_package_share_directory('nav2_bringup')
    # The research harness binds this read-only configuration explicitly; it is not a production package default.
    params = os.environ.get('AWS_KNOWNSELFPOSE_NAV2_PARAMS', '/feature/config/aws_knownselfpose_nav2.yaml')
    map_file = LaunchConfiguration('map')
    return LaunchDescription([
        DeclareLaunchArgument('map'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename': map_file, 'use_sim_time': True}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
             parameters=[{'autostart': True, 'node_names': ['map_server'], 'use_sim_time': True}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2, 'launch', 'navigation_launch.py'])),
            launch_arguments={'use_sim_time': 'true', 'autostart': 'true',
                              'params_file': params, 'use_composition': 'False'}.items()),
    ])
