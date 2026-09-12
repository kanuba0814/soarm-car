import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    navigation_share = get_package_share_directory('soarm_navigation')
    simulation_share = get_package_share_directory('soarm_simulation')
    localization_share = get_package_share_directory('fast_lio_localization')
    shadow_share = get_package_share_directory('soarm_fixed_map_localization')
    nav2_share = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(navigation_share, 'config', 'house_nav2_mppi.yaml')
    map_file = LaunchConfiguration('occupancy_yaml')
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
        # Legacy standalone launch keeps its filename compatibility default;
        # the room-navigation launch supplies the computed immutable digest.
        DeclareLaunchArgument('occupancy_yaml', default_value=os.path.join(simulation_share, 'maps', 'house_floorplan.yaml')),
        DeclareLaunchArgument('localization_pcd', default_value='/workspaces/soarm-car/artifacts/fastlio_house_floorplan.pcd'),
        DeclareLaunchArgument('bundle_id', default_value='fastlio_house_floorplan.pcd'),
        DeclareLaunchArgument('fixed_map_shadow', default_value='false'),
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
            launch_arguments={'use_sim_time': 'true', 'config_file': 'soarm_house_localization.yaml', 'rviz': 'false',
                              'map_file_path': LaunchConfiguration('localization_pcd'),
                              'bundle_id': LaunchConfiguration('bundle_id')}.items()),
        Node(
            package='soarm_fixed_map_localization', executable='fixed_map_shadow_node',
            name='fixed_map_shadow', parameters=[os.path.join(shadow_share, 'config', 'house_shadow.yaml')],
            condition=IfCondition(LaunchConfiguration('fixed_map_shadow')),
        ),
        Node(
            package='soarm_navigation', executable='localization_cmd_gate',
            name='localization_cmd_gate',
            parameters=[{
                'health_timeout': 0.5,
                'command_timeout': 0.5,
                'publish_rate': 20.0,
                'expected_map_id': LaunchConfiguration('bundle_id'),
            }],
        ),
        Node(
            package='soarm_navigation', executable='body_height_filter',
            name='body_height_filter',
            parameters=[{'use_sim_time': True}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([nav2_share, 'launch', 'navigation_launch.py'])),
            launch_arguments={
                'use_sim_time': 'true', 'autostart': 'true', 'params_file': configured_params,
                'use_composition': 'False',
            }.items()),
    ])
