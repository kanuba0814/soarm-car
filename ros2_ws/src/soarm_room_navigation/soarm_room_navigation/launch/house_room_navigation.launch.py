import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start(context):
    from soarm_room_navigation.core import MapBundle
    simulation = get_package_share_directory('soarm_simulation')
    navigation = get_package_share_directory('soarm_navigation')
    occupancy = LaunchConfiguration('occupancy_yaml').perform(context)
    pcd = LaunchConfiguration('localization_pcd').perform(context)
    alignment = LaunchConfiguration('alignment').perform(context)
    bundle = MapBundle.load(occupancy, pcd, alignment=alignment)
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(navigation, 'launch', 'house_fastlio_nav2.launch.py')),
                                 launch_arguments={'occupancy_yaml': occupancy, 'localization_pcd': pcd,
                                                   'bundle_id': bundle.bundle_id,
                                                   'gz_args': LaunchConfiguration('gz_args'),
                                                   # This launch owns the operator RViz profile below.
                                                   'rviz': 'false'}.items()),
        Node(package='rviz2', executable='rviz2', name='room_navigation_rviz',
             condition=IfCondition(LaunchConfiguration('rviz').perform(context)),
             parameters=[{'use_sim_time': True}],
             arguments=['-d', os.path.join(get_package_share_directory('soarm_room_navigation'),
                                            'config', 'room_navigation.rviz')]),
        Node(package='soarm_room_navigation', executable='initial_map_pose', name='initial_map_pose',
             condition=IfCondition(LaunchConfiguration('initialize_pose')),
             parameters=[{'use_sim_time': True, 'initial_x': LaunchConfiguration('initial_x'),
                          'initial_y': LaunchConfiguration('initial_y'), 'initial_yaw': LaunchConfiguration('initial_yaw'),
                          'timeout': LaunchConfiguration('initialization_timeout')}]),
        Node(package='soarm_room_navigation', executable='room_navigation_node', name='room_navigation',
             parameters=[{'use_sim_time': True, 'occupancy_yaml': occupancy, 'localization_pcd': pcd, 'alignment': alignment,
                          'rooms_path': LaunchConfiguration('rooms_path'), 'task_journal': LaunchConfiguration('task_journal'),
                          'gui_port': LaunchConfiguration('gui_port'), 'health_timeout': LaunchConfiguration('health_timeout'),
                          'cancel_timeout': LaunchConfiguration('cancel_timeout'), 'goal_acceptance_timeout': LaunchConfiguration('goal_acceptance_timeout'),
                          'goal_execution_timeout': LaunchConfiguration('goal_execution_timeout'), 'fixture_demo': LaunchConfiguration('fixture_demo')}]),
    ]


def generate_launch_description():
    simulation = get_package_share_directory('soarm_simulation')
    return LaunchDescription([
        DeclareLaunchArgument('occupancy_yaml', default_value=os.path.join(simulation, 'maps', 'house_floorplan.yaml')),
        DeclareLaunchArgument('localization_pcd', default_value=os.path.join(simulation, 'maps', 'house_floorplan_reference.pcd')),
        DeclareLaunchArgument('alignment', default_value='house-origin-v1:T_map_camera_init=identity'),
        DeclareLaunchArgument('gz_args', default_value='-r -s', description='Gazebo arguments; use "-r" for the GUI.'),
        DeclareLaunchArgument('rviz', default_value='false', description='Start the room-navigation RViz profile.'),
        DeclareLaunchArgument('initialize_pose', default_value='false', description='Publish the configured map pose after localization inputs arrive.'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('initialization_timeout', default_value='45.0'),
        DeclareLaunchArgument('gui_port', default_value='8090'),
        DeclareLaunchArgument('rooms_path', default_value='/tmp/soarm-room-navigation-rooms.json'),
        DeclareLaunchArgument('task_journal', default_value='/tmp/soarm-room-navigation-task.json'),
        DeclareLaunchArgument('health_timeout', default_value='0.5'),
        DeclareLaunchArgument('cancel_timeout', default_value='5.0'),
        DeclareLaunchArgument('goal_acceptance_timeout', default_value='5.0'),
        DeclareLaunchArgument('goal_execution_timeout', default_value='120.0'),
        DeclareLaunchArgument('fixture_demo', default_value='false'),
        OpaqueFunction(function=_start),
    ])
