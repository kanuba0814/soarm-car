"""Physical STM32 mecanum chassis safe base: safety gate + serial adapter.

Mirrors ``soarm_navigation``'s ``wheeltec_safe_base.launch.py``: the
fail-closed ``localization_cmd_gate`` owns ``/cmd_vel`` -> ``/safe_cmd_vel``
authorization, and ``stm32_chassis_serial`` is the only writer to the
chassis USART1 link. The teleop, Nav2, and every other command source stay
upstream on ``/cmd_vel``; nothing may bypass the gate (WORKSPACE_INDEX.md).

The chassis only moves while localization health is fresh; check
``ros2 service call /localization_cmd_gate/status std_srvs/srv/Trigger {}``
before driving.
"""

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
            'serial_port', default_value='/dev/ttyUSB0',
            description='Chassis USART1 serial device.'),
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
            package='soarm_teleop',
            executable='stm32_chassis_serial',
            name='stm32_chassis_serial',
            output='screen',
            parameters=[{
                'port': serial_port,
                'baud': ParameterValue(serial_baud_rate, value_type=int),
                'command_timeout': 0.5,
                'send_period': 0.05,
                'status_poll_period': 0.0,
            }],
        ),
    ])
