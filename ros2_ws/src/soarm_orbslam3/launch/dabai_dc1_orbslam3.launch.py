from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value=''),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('color_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('depth_info_topic', default_value='/camera/depth/camera_info'),
        Node(
            package='soarm_orbslam3',
            executable='orbslam3_rgbd_bridge',
            name='orbslam3_rgbd_bridge',
            namespace='orbslam3',
            output='screen',
            parameters=[{
                'profile': LaunchConfiguration('profile'),
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'color_info_topic': LaunchConfiguration('color_info_topic'),
                'depth_info_topic': LaunchConfiguration('depth_info_topic'),
            }],
        ),
    ])
