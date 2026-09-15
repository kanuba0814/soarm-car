import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def _launch(context):
    camera_name = LaunchConfiguration('camera_name').perform(context)
    config_path = LaunchConfiguration('config_file').perform(context)
    serial = LaunchConfiguration('serial_number').perform(context)
    params = {}
    if config_path:
        with open(config_path, encoding='utf-8') as stream:
            params.update(yaml.safe_load(stream) or {})
    if serial:
        params['serial_number'] = serial
    params['camera_name'] = camera_name
    return [ComposableNodeContainer(
        name='dc1_orbbec_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[ComposableNode(
            package='orbbec_camera',
            plugin='orbbec_camera::OBCameraNodeDriver',
            name=camera_name,
            namespace=camera_name,
            parameters=[params],
        )],
        output='screen',
    )]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('orbbec_camera'), 'config', 'camera_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('serial_number', default_value=''),
        DeclareLaunchArgument('config_file', default_value=default_config),
        OpaqueFunction(function=_launch),
    ])
