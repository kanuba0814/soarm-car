from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    simulation = get_package_share_directory('soarm_simulation')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='house_floorplan_ros2_control'),
        DeclareLaunchArgument('gz_args', default_value='-r -s'),
        DeclareLaunchArgument('bundle_path', default_value='/workspaces/soarm-car/artifacts/visual_map_house'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                simulation, 'launch', 'kuruma_ros2_control.launch.py'])),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'gz_args': LaunchConfiguration('gz_args'),
                'rviz': 'false',
            }.items()),
        Node(
            package='soarm_visual_localization', executable='rgbd_pointcloud',
            parameters=[{
                'depth_scale': 0.001,
                'depth_min_m': 0.2,
                'depth_max_m': 8.0,
                'frame_id': 'camera_link',
            }], output='screen'),
        Node(
            package='soarm_visual_localization', executable='visual_localizer',
            parameters=[{
                'use_sim_time': True,
                'registration_verified': True,
                'bundle_path': LaunchConfiguration('bundle_path'),
                'frame_id': 'camera_link',
            }], output='screen'),
    ])
