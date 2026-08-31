import os
import shutil
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('soarm_simulation')
    description_share = get_package_share_directory('soarm_description')
    robot = Command(['xacro ', os.path.join(description_share, 'urdf', 'soarm_car_layout.urdf.xacro')])
    controllers = os.path.join(sim_share, 'config', 'kuruma_ros2_control_controllers.yaml')
    runtime_controllers = '/tmp/kuruma_ros2_control_controllers.yaml'
    shutil.copyfile(controllers, runtime_controllers)
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ['-r -s ', os.path.join(sim_share, 'worlds', 'kuruma_ros2_control.sdf')]}.items())
    jsb = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'], output='screen')
    mecanum = Node(package='controller_manager', executable='spawner', arguments=['mecanum_drive_controller', '--param-file', runtime_controllers], output='screen')
    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.path.join(sim_share, 'models')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(sim_share, 'models')),
        gazebo,
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', parameters=[{'robot_description': robot, 'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', '/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'], output='screen'),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[mecanum])),
        jsb,
    ])
