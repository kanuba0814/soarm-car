import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('soarm_simulation')
    description_share = get_package_share_directory('soarm_description')
    robot = Command(['xacro ', os.path.join(description_share, 'urdf', 'soarm_car_layout.urdf.xacro')])
    controllers = os.path.join(sim_share, 'config', 'kuruma_ros2_control_controllers.yaml')
    runtime_controllers = '/tmp/kuruma_ros2_control_controllers.yaml'
    shutil.copyfile(controllers, runtime_controllers)
    world_path = PathJoinSubstitution([
        sim_share, 'worlds', PythonExpression(["'", LaunchConfiguration('world'), ".sdf'"]),
    ])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': [LaunchConfiguration('gz_args'), ' ', world_path]}.items())
    jsb = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'], output='screen')
    mecanum = Node(package='controller_manager', executable='spawner', arguments=['mecanum_drive_controller', '--param-file', runtime_controllers], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='kuruma_ros2_control', description='ros2_control world basename from share/soarm_simulation/worlds.'),
        DeclareLaunchArgument('gz_args', default_value='-r -s', description='Gazebo arguments; use "-r" to open the GUI.'),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz with robot, TF, odometry path, point cloud and camera displays.'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.path.join(sim_share, 'models')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(sim_share, 'models')),
        gazebo,
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', parameters=[{'robot_description': robot, 'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/gazebo/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_right/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera_right/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        ], output='screen'),
        Node(package='soarm_simulation', executable='sim_adapter', parameters=[
            {'policy_file': os.path.join(sim_share, 'config', 'mission_policy.yaml')},
            {'publish_odometry': False},
            {'arm_enabled': False},
        ]),
        Node(package='soarm_simulation', executable='odom_path'),
        Node(package='soarm_uaf_tm', executable='uaf_tm_node', parameters=[{'depth_topic': '/camera/depth_image', 'camera_info_topic': '/camera/camera_info', 'odom_topic': '/wheel/odom_raw'}], output='screen'),
        Node(
            package='rviz2', executable='rviz2', name='simulation_rviz',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', os.path.join(sim_share, 'config', 'simulation.rviz')],
        ),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[mecanum])),
        jsb,
    ])
