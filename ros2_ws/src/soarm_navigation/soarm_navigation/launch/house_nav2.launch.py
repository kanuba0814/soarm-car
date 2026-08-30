import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    navigation_share = get_package_share_directory('soarm_navigation')
    simulation_share = get_package_share_directory('soarm_simulation')
    nav2_share = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(navigation_share, 'config', 'house_nav2.yaml')
    behavior_tree = os.path.join(navigation_share, 'behavior_trees', 'navigate_ackermann.xml')
    through_poses_tree = os.path.join(navigation_share, 'behavior_trees', 'navigate_through_poses_ackermann.xml')
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'default_nav_to_pose_bt_xml': behavior_tree,
            'default_nav_through_poses_bt_xml': through_poses_tree,
        },
        convert_types=False,
    )

    return LaunchDescription([
        DeclareLaunchArgument('gz_args', default_value='-r -s'),
        DeclareLaunchArgument('rviz', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([simulation_share, 'launch', 'simulation.launch.py'])),
            launch_arguments={
                'world': 'house_floorplan',
                'gz_args': LaunchConfiguration('gz_args'),
                'rviz': LaunchConfiguration('rviz'),
                'ground_truth_map_tf': 'true',
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([nav2_share, 'launch', 'navigation_launch.py'])),
            launch_arguments={
                'use_sim_time': 'true',
                'autostart': 'true',
                'params_file': configured_params,
                'use_composition': 'False',
            }.items()),
    ])
