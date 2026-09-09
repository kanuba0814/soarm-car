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
        DeclareLaunchArgument('use_native_livox_converter', default_value='false', description='Use the C++ PointCloud2 to Livox converter instead of SimAdapter Python point handling.'),
        DeclareLaunchArgument('throttle_odom_path', default_value='false', description='Publish the RViz-only /odom_path at at most 2 Hz when it has subscribers.'),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz with robot, TF, odometry path, point cloud and camera displays.'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.path.join(sim_share, 'models')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(sim_share, 'models')),
        gazebo,
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', parameters=[{'robot_description': robot, 'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/gazebo/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/gazebo/livox_imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/mid360/scan/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ], output='screen'),
        Node(package='soarm_simulation', executable='sim_adapter', parameters=[
            {'policy_file': os.path.join(sim_share, 'config', 'mission_policy.yaml')},
            {'publish_odometry': False},
            {'arm_enabled': False},
            {'use_native_livox_converter': LaunchConfiguration('use_native_livox_converter')},
        ]),
        Node(
            package='soarm_native_livox_converter', executable='native_livox_converter',
            condition=IfCondition(LaunchConfiguration('use_native_livox_converter')),
        ),
        Node(
            package='soarm_simulation', executable='odom_path',
            parameters=[{'publish_rate_hz': 2.0}],
            condition=IfCondition(LaunchConfiguration('throttle_odom_path')),
        ),
        Node(
            package='soarm_simulation', executable='odom_path',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('throttle_odom_path'), "' != 'true'"])),
        ),
        Node(
            package='rviz2', executable='rviz2', name='simulation_rviz',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', os.path.join(sim_share, 'config', 'simulation.rviz')],
        ),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[mecanum])),
        jsb,
    ])
