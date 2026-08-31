import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('soarm_simulation')
    description_share = get_package_share_directory('soarm_description')
    robot = Command(['xacro ', os.path.join(description_share, 'urdf', 'soarm_car_layout.urdf.xacro')])
    models_path = os.path.join(sim_share, 'models')
    world_path = PathJoinSubstitution([sim_share, 'worlds', PythonExpression(["'", LaunchConfiguration('world'), ".sdf'"])])
    map_path = PathJoinSubstitution([sim_share, 'maps', PythonExpression(["'", LaunchConfiguration('world'), ".yaml'"])])
    localisation_world = IfCondition(PythonExpression([
        "'", LaunchConfiguration('world'), "' in ['navigation_room', 'house_floorplan']",
    ]))
    ground_truth_map_tf = IfCondition(PythonExpression([
        "'", LaunchConfiguration('world'), "' in ['navigation_room', 'house_floorplan'] and '",
        LaunchConfiguration('ground_truth_map_tf'), "' == 'true'",
    ]))
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        condition=IfCondition(LaunchConfiguration('simulation')),
        launch_arguments={'gz_args': [LaunchConfiguration('gz_args'), ' ', world_path]}.items())
    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', models_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_path),
        DeclareLaunchArgument('world', default_value='kuruma', description='World basename from share/soarm_simulation/worlds; use navigation_room or house_floorplan for localisation.'),
        DeclareLaunchArgument('simulation', default_value='true', description='true starts Fortress; false keeps the hardware-compatible ROS seam only.'),
        DeclareLaunchArgument('gz_args', default_value='-r -s', description='Gazebo arguments; use "-r" to open the GUI.'),
        DeclareLaunchArgument('spawn_robot', default_value='true'),
        DeclareLaunchArgument('ground_truth_map_tf', default_value='true', description='Publish identity map->odom_combined for room preview; disable when AMCL/localisation owns this transform.'),
        DeclareLaunchArgument('publish_sim_odometry', default_value='true', description='Publish wheel-encoder /odom, ground-truth odometry, and odom_combined TF; disable when FAST-LIO owns Nav2 odometry.'),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz with robot, TF, odometry path, point cloud and camera displays.'),
        gazebo,
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': robot}]),
        # Kuruma is native SDF so the Fortress mecanum system applies wheel forces; robot_state_publisher supplies matching sensor TF.
        Node(package='ros_gz_bridge', executable='parameter_bridge', condition=IfCondition(LaunchConfiguration('simulation')), arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/safe_cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/gazebo/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/gazebo/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/gazebo/livox_imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/mid360/scan/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ]),
        Node(package='soarm_simulation', executable='sim_adapter', parameters=[
            {'policy_file': os.path.join(sim_share, 'config', 'mission_policy.yaml')},
            {'publish_odometry': LaunchConfiguration('publish_sim_odometry')},
            {'arm_enabled': False},
        ]),
        Node(package='soarm_simulation', executable='odom_path'),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            condition=localisation_world,
            parameters=[{'yaml_filename': map_path}],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='ground_truth_map_tf',
            condition=ground_truth_map_tf,
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom_combined'],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
            condition=localisation_world,
            parameters=[{'autostart': True, 'node_names': ['map_server']}],
        ),
        Node(
            package='rviz2', executable='rviz2', name='simulation_rviz',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', os.path.join(sim_share, 'config', 'simulation.rviz')],
        ),
    ])
