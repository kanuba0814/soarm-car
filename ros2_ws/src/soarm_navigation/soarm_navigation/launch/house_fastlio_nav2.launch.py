import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    navigation_share = get_package_share_directory('soarm_navigation')
    simulation_share = get_package_share_directory('soarm_simulation')
    localization_share = get_package_share_directory('fast_lio_localization')
    nav2_share = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(navigation_share, 'config', 'house_nav2_mppi.yaml')
    map_file = os.path.join(simulation_share, 'maps', 'house_floorplan.yaml')
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'default_nav_to_pose_bt_xml': os.path.join(navigation_share, 'behavior_trees', 'navigate_holonomic.xml'),
            'default_nav_through_poses_bt_xml': os.path.join(navigation_share, 'behavior_trees', 'navigate_through_poses_holonomic.xml'),
        },
        convert_types=False,
    )
    return LaunchDescription([
        DeclareLaunchArgument('gz_args', default_value='-r -s'),
        DeclareLaunchArgument('rviz', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([simulation_share, 'launch', 'kuruma_ros2_control.launch.py'])),
            launch_arguments={
                'world': 'house_floorplan_ros2_control',
                'gz_args': LaunchConfiguration('gz_args'),
                'rviz': LaunchConfiguration('rviz'),
            }.items()),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            parameters=[{'yaml_filename': map_file, 'use_sim_time': True}],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
            parameters=[{'autostart': True, 'node_names': ['map_server'], 'use_sim_time': True}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([localization_share, 'launch', 'velodyne_localization.launch.py'])),
            launch_arguments={'use_sim_time': 'true', 'config_file': 'soarm_house_localization.yaml', 'rviz': 'false'}.items()),
        Node(
            package='soarm_navigation', executable='localization_cmd_gate',
            name='localization_cmd_gate',
            parameters=[{
                'health_timeout': 0.5,
                'command_timeout': 0.5,
                'publish_rate': 20.0,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([nav2_share, 'launch', 'navigation_launch.py'])),
            launch_arguments={
                'use_sim_time': 'true', 'autostart': 'true', 'params_file': configured_params,
                'use_composition': 'False',
            }.items()),
    ])
