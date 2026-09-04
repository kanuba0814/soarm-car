import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('soarm_description')
    description = Command(['xacro ', os.path.join(share, 'urdf', 'soarm_car_layout.urdf.xacro')])
    rviz_config = os.path.join(share, 'rviz', 'model_review.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', condition=IfCondition(LaunchConfiguration('gui')), parameters=[{'robot_description': description}]),
        Node(package='soarm_fake', executable='state_visualizer'),
        Node(package='foxglove_bridge', executable='foxglove_bridge', condition=IfCondition(LaunchConfiguration('foxglove'))),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
