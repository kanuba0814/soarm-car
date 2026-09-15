"""Keyboard teleop with per-target presets for the shared Twist command seam.

``target`` selects the downstream consumer and conservative speed limits:

- ``fortress``      -> /cmd_vel        (native Fortress MecanumDrive)
- ``ros2_control``  -> /safe_cmd_vel   (kuruma_ros2_control plant, direct)
- ``physical``      -> /cmd_vel        (localization_cmd_gate -> STM32 chassis)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TARGET_PRESETS = {
    'fortress': {'cmd_topic': '/cmd_vel', 'max_linear': 0.5, 'max_angular': 1.2},
    'ros2_control': {'cmd_topic': '/safe_cmd_vel', 'max_linear': 0.5, 'max_angular': 1.2},
    'physical': {'cmd_topic': '/cmd_vel', 'max_linear': 0.3, 'max_angular': 0.8},
}


def launch_setup(context):
    target = LaunchConfiguration('target').perform(context)
    if target not in TARGET_PRESETS:
        raise RuntimeError(
            f'unknown teleop target {target!r}; choose one of {sorted(TARGET_PRESETS)}')
    preset = TARGET_PRESETS[target]

    def override(name, cast):
        raw = LaunchConfiguration(name).perform(context)
        return cast(raw) if raw else preset[name]

    parameters = [{
        'cmd_topic': override('cmd_topic', str),
        'max_linear': override('max_linear', float),
        'max_angular': override('max_angular', float),
    }]
    return [Node(
        package='soarm_teleop',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        parameters=parameters,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'target', default_value='fortress',
            description='Chassis consumer: fortress | ros2_control | physical.'),
        DeclareLaunchArgument(
            'cmd_topic', default_value='',
            description='Override the preset command topic.'),
        DeclareLaunchArgument(
            'max_linear', default_value='',
            description='Override the preset max linear speed (m/s).'),
        DeclareLaunchArgument(
            'max_angular', default_value='',
            description='Override the preset max angular speed (rad/s).'),
        OpaqueFunction(function=launch_setup),
    ])
