from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    upstream_launch = (
        get_package_share_directory('astra_camera') + '/launch/astra.launch.xml')
    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(upstream_launch),
            launch_arguments={
                'camera_name': 'camera',
                'vendor_id': '0x2bc5',
                'product_id': '0x0402',
                'enable_color': 'true',
                'enable_depth': 'true',
                'enable_ir': 'false',
                'enable_point_cloud': 'true',
                'enable_colored_point_cloud': 'false',
                'depth_registration': 'false',
                'color_width': '640',
                'color_height': '480',
                'color_fps': '30',
                'depth_width': '640',
                'depth_height': '480',
                'depth_fps': '30',
            }.items(),
        ),
    ])
