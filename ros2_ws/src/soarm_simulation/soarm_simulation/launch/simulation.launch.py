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
    navigation_room = IfCondition(PythonExpression(["'", LaunchConfiguration('world'), "' == 'navigation_room'"]))
    ground_truth_map_tf = IfCondition(PythonExpression([
        "'", LaunchConfiguration('world'), "' == 'navigation_room' and '",
        LaunchConfiguration('ground_truth_map_tf'), "' == 'true'",
    ]))
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        condition=IfCondition(LaunchConfiguration('simulation')),
        launch_arguments={'gz_args': [LaunchConfiguration('gz_args'), ' ', world_path]}.items())
    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', models_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_path),
        DeclareLaunchArgument('world', default_value='senior_akm', description='World basename from share/soarm_simulation/worlds; use navigation_room for localisation.'),
        DeclareLaunchArgument('simulation', default_value='true', description='true starts Fortress; false keeps the hardware-compatible ROS seam only.'),
        DeclareLaunchArgument('gz_args', default_value='-r -s', description='Gazebo arguments; use "-r" to open the GUI.'),
        DeclareLaunchArgument('spawn_robot', default_value='true'),
        DeclareLaunchArgument('ground_truth_map_tf', default_value='true', description='Publish identity map->odom_combined for room preview; disable when AMCL/localisation owns this transform.'),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz with robot, TF, odometry path, point cloud and camera displays.'),
        gazebo,
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': robot}]),
        # senior_akm is defined as native SDF so its Ackermann system can apply wheel forces;
        # robot_state_publisher above supplies the matching ROS arm/sensor TF description.
        Node(package='ros_gz_bridge', executable='parameter_bridge', condition=IfCondition(LaunchConfiguration('simulation')), arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/gazebo/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/gazebo/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/gazebo/livox_imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/mid360/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/gazebo/arm_shoulder_pan/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/gazebo/arm_shoulder_lift/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/gazebo/arm_elbow_flex/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/gazebo/arm_wrist_flex/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/gazebo/arm_wrist_roll/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/gazebo/arm_gripper_opening/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        ]),
        Node(package='soarm_simulation', executable='sim_adapter', parameters=[{'policy_file': os.path.join(sim_share, 'config', 'mission_policy.yaml')}]),
        Node(package='soarm_simulation', executable='odom_path'),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            condition=navigation_room,
            parameters=[{'yaml_filename': os.path.join(sim_share, 'maps', 'navigation_room.yaml')}],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='ground_truth_map_tf',
            condition=ground_truth_map_tf,
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom_combined'],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
            condition=navigation_room,
            parameters=[{'autostart': True, 'node_names': ['map_server']}],
        ),
        Node(
            package='rviz2', executable='rviz2', name='simulation_rviz',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', os.path.join(sim_share, 'config', 'simulation.rviz')],
        ),
    ])
