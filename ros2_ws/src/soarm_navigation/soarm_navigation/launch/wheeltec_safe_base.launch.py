from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    serial_baud_rate = LaunchConfiguration('serial_baud_rate')
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/wheeltec_controller'),
        DeclareLaunchArgument('serial_baud_rate', default_value='115200'),
        Node(
            package='soarm_navigation',
            executable='localization_cmd_gate',
            name='localization_cmd_gate',
            parameters=[{
                'health_timeout': 0.5,
                'command_timeout': 0.5,
                'publish_rate': 20.0,
            }],
        ),
        Node(
            package='turn_on_wheeltec_robot',
            executable='wheeltec_robot_node',
            name='wheeltec_robot_node',
            output='screen',
            parameters=[{
                'usart_port_name': serial_port,
                'serial_baud_rate': ParameterValue(serial_baud_rate, value_type=int),
                'robot_frame_id': 'base_footprint',
                'odom_frame_id': 'odom_combined',
                'cmd_vel': 'safe_cmd_vel',
                'akm_cmd_vel': 'none',
                'product_number': 0,
                'odom_x_scale': 1.0,
                'odom_y_scale': 1.0,
                'odom_z_scale_positive': 1.0,
                'odom_z_scale_negative': 1.0,
            }],
            remappings=[
                ('cmd_vel', 'safe_cmd_vel'),
                ('odom', 'wheel/odom_raw'),
            ],
        ),
    ])
