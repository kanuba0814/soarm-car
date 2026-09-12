"""Operator entrypoint for the supported room-navigation simulation mode."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def _start(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode != 'simulation':
        raise RuntimeError('Unsupported mode={!r}; only mode:=simulation is implemented.'.format(mode))
    share = get_package_share_directory('soarm_room_navigation')
    for path in ('rooms_path', 'task_journal'):
        parent = os.path.dirname(LaunchConfiguration(path).perform(context))
        if parent:
            os.makedirs(parent, exist_ok=True)
    arguments = {name: LaunchConfiguration(name) for name in (
        'occupancy_yaml', 'localization_pcd', 'alignment', 'gui_port', 'rooms_path', 'task_journal',
        'health_timeout', 'cancel_timeout', 'goal_acceptance_timeout', 'goal_execution_timeout', 'fixture_demo',
        'gz_args', 'rviz', 'initialize_pose', 'initial_x', 'initial_y', 'initial_yaw', 'initialization_timeout')}
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'house_room_navigation.launch.py')),
        launch_arguments=arguments.items())]


def generate_launch_description():
    simulation = get_package_share_directory('soarm_simulation')
    return LaunchDescription([
        SetEnvironmentVariable('OMP_NUM_THREADS', EnvironmentVariable('OMP_NUM_THREADS', default_value='2')),
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', EnvironmentVariable('OPENBLAS_NUM_THREADS', default_value='2')),
        DeclareLaunchArgument('mode', default_value='simulation', description='Only simulation is supported.'),
        DeclareLaunchArgument('occupancy_yaml', default_value=os.path.join(simulation, 'maps', 'house_floorplan.yaml')),
        DeclareLaunchArgument('localization_pcd', default_value=os.path.join(simulation, 'maps', 'house_floorplan_reference.pcd')),
        DeclareLaunchArgument('alignment', default_value='house-origin-v1:T_map_camera_init=identity'),
        DeclareLaunchArgument('gz_args', default_value='-r', description='Gazebo GUI arguments for simulation mode.'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('initialize_pose', default_value='true'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('initialization_timeout', default_value='45.0'),
        DeclareLaunchArgument('gui_port', default_value='8090'),
        DeclareLaunchArgument('rooms_path', default_value='/workspaces/soarm-car/artifacts/room-navigation-live/rooms-complete-reference.json'),
        DeclareLaunchArgument('task_journal', default_value='/workspaces/soarm-car/artifacts/room-navigation-live/task-complete-reference.json'),
        DeclareLaunchArgument('health_timeout', default_value='0.5'),
        DeclareLaunchArgument('cancel_timeout', default_value='5.0'),
        DeclareLaunchArgument('goal_acceptance_timeout', default_value='5.0'),
        DeclareLaunchArgument('goal_execution_timeout', default_value='120.0'),
        DeclareLaunchArgument('fixture_demo', default_value='false'),
        OpaqueFunction(function=_start),
    ])
