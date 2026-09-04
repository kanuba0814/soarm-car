from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        Node(package='soarm_astra', executable='sim_adapter', output='screen'),
        ComposableNodeContainer(
            name='astra_sim_processing',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz',
                    namespace='camera',
                    remappings=[
                        ('image_rect', 'depth/image_raw'),
                        ('camera_info', 'depth/camera_info'),
                        ('points', 'depth/points'),
                    ],
                ),
            ],
            output='screen',
        ),
    ])
