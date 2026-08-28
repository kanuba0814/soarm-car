import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


POSES = {
    # SO-ARM100/Simulation/SO101 uses this calibration by default: zero is
    # the midpoint of every joint range.
    'new_calib_mid': {
        'arm_shoulder_pan': 0.0,
        'arm_shoulder_lift': 0.0,
        'arm_elbow_flex': 0.0,
        'arm_wrist_flex': 0.0,
        'arm_wrist_roll': 0.0,
        'arm_gripper_opening': 0.0,
    },
    # Approximate visual reference for the official old-calibration meaning:
    # joint zero corresponds to a horizontally extended arm. This lightweight
    # model does not reproduce the old URDF's complete joint-frame transforms.
    'old_calib_extended': {
        'arm_shoulder_pan': 0.0,
        'arm_shoulder_lift': -1.5708,
        'arm_elbow_flex': 0.0,
        'arm_wrist_flex': 0.0,
        'arm_wrist_roll': 0.0,
        'arm_gripper_opening': 0.0,
    },
    # Non-official display pose retained only for checking proportions and
    # occlusion in RViz. It must never be presented as a calibrated home pose.
    'review_bent': {
        'arm_shoulder_pan': 0.0,
        'arm_shoulder_lift': -0.55,
        'arm_elbow_flex': 0.95,
        'arm_wrist_flex': -0.40,
        'arm_wrist_roll': 0.0,
        'arm_gripper_opening': 0.35,
    },
}


def launch_nodes(context, description, rviz_config):
    pose_name = LaunchConfiguration('pose').perform(context)
    if pose_name not in POSES:
        raise RuntimeError(f'unknown pose {pose_name!r}; choose one of {sorted(POSES)}')
    initial_positions = {
        **POSES[pose_name],
        'layout_arm_x': 0.0,
        'layout_camera_x': 0.0,
    }
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui')),
            parameters=[{'robot_description': description, 'zeros': initial_positions}],
        ),
        Node(package='soarm_fake', executable='state_visualizer'),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            condition=IfCondition(LaunchConfiguration('foxglove')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory('soarm_description')
    description = Command(['xacro ', os.path.join(share, 'urdf', 'soarm_car_layout.urdf.xacro')])
    rviz_config = os.path.join(share, 'rviz', 'model_review.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('pose', default_value='new_calib_mid'),
        OpaqueFunction(function=launch_nodes, args=[description, rviz_config]),
    ])
