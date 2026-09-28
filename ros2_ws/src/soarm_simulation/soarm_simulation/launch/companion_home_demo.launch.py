import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('soarm_simulation')
    description_share = get_package_share_directory('soarm_description')
    robot = Command(['xacro ', os.path.join(description_share, 'urdf', 'soarm_car_layout.urdf.xacro')])
    runtime_controllers = '/tmp/kuruma_ros2_control_controllers.yaml'
    shutil.copyfile(os.path.join(sim_share, 'config', 'kuruma_ros2_control_controllers.yaml'), runtime_controllers)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world_file')]}.items())
    jsb = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'], output='screen')
    mecanum = Node(package='controller_manager', executable='spawner', arguments=['mecanum_drive_controller', '--param-file', runtime_controllers], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('world_file', description='Absolute generated companion-home SDF path.'),
        gazebo,
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', parameters=[{'robot_description': robot, 'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/gazebo/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        ], output='screen'),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[mecanum])),
        jsb,
    ])
