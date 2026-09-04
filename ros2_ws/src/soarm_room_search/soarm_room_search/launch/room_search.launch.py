from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('occupancy_yaml', default_value=''),
        DeclareLaunchArgument('expected_map_id', default_value=''),
        DeclareLaunchArgument('rooms_json', default_value=''),
        DeclareLaunchArgument('rooms_store_path', default_value='/tmp/soarm-rooms.json'),
        DeclareLaunchArgument('asr_command', default_value=''),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/camera/depth/points'),
        DeclareLaunchArgument('camera_frame', default_value='camera_link'),
        DeclareLaunchArgument('gui_port', default_value='8088'),
    ]
    node = Node(
        package='soarm_room_search',
        executable='room_search_node',
        name='room_search',
        parameters=[{
            'occupancy_yaml': LaunchConfiguration('occupancy_yaml'),
            'expected_map_id': LaunchConfiguration('expected_map_id'),
            'rooms_json': LaunchConfiguration('rooms_json'),
            'rooms_store_path': LaunchConfiguration('rooms_store_path'),
            'asr_command': LaunchConfiguration('asr_command'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'gui_port': LaunchConfiguration('gui_port'),
        }],
    )
    return LaunchDescription(arguments + [node])
