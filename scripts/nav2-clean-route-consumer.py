#!/usr/bin/env python3
"""Drive and checkpoint the bounded saved multiroom Nav2 route."""

import json
import math
import os
import pathlib
import sys
import time
import traceback

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from soarm_interfaces.msg import LocalizationHealth
from shared_nav2_transport import NavigationActionTransport

ROOMS_PATH = pathlib.Path('/workspaces/soarm-car/artifacts/rooms-complete-reference.json')
SIM_LIMIT = 120.0
WALL_LIMIT = 180.0

def yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def angular_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec / 1e9


def atomic_write(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n')
    os.replace(temporary, path)


def finite_or_none(value):
    return value if math.isfinite(value) else None


def integrate(samples, index, start_sim, end_sim):
    usable = [item for item in samples if item[0] is not None and start_sim <= item[0] <= end_sim]
    return sum(
        abs((previous[index] + current[index]) / 2.0) * max(0.0, current[0] - previous[0])
        for previous, current in zip(usable, usable[1:])
    )


def actual_turn(samples, start_sim, end_sim):
    usable = [item for item in samples if start_sim <= item[0] <= end_sim]
    return sum(
        abs(angular_difference(current[3], previous[3]))
        for previous, current in zip(usable, usable[1:])
    )


def nearest_errors(localization, truth, start_sim, end_sim):
    truth_window = [item for item in truth if start_sim <= item[0] <= end_sim]
    if not truth_window:
        return []
    errors = []
    for local in localization:
        if start_sim <= local[0] <= end_sim:
            nearest = min(truth_window, key=lambda item: abs(item[0] - local[0]))
            errors.append(math.hypot(local[1] - nearest[1], local[2] - nearest[2]))
    return errors


def monotonic(values):
    return all(current >= previous for previous, current in zip(values, values[1:]))


def main():
    metrics_path = pathlib.Path(sys.argv[1])
    metrics_path.parent.mkdir(parents=True, exist_ok=True)
    report = {
        'complete': False,
        'passed': False,
        'classification': 'consumer initializing',
        'error': None,
        'limits': {'sim_seconds_per_goal': SIM_LIMIT, 'wall_seconds_per_goal': WALL_LIMIT},
        'route': [],
        'raw_health_samples': [],
        'raw_status_samples': [],
        'telemetry': {},
    }
    atomic_write(metrics_path, report)
    rclpy.init()
    node = rclpy.create_node('nav2_clean_route_consumer')
    state = {
        'clock': None,
        'fastlio': [],
        'truth': [],
        'localization': [],
        'safe': [],
        'health': [],
        'statuses': [],
        'plans': [],
    }
    transport = None

    def clock_callback(message):
        state['clock'] = message.clock.sec + message.clock.nanosec / 1e9

    def fastlio_callback(message):
        state['fastlio'].append(stamp(message))

    def truth_callback(message):
        state['truth'].append((stamp(message), message.pose.pose.position.x,
                               message.pose.pose.position.y, yaw(message.pose.pose.orientation)))

    def localization_callback(message):
        state['localization'].append((stamp(message), message.pose.pose.position.x,
                                      message.pose.pose.position.y, yaw(message.pose.pose.orientation)))

    def safe_callback(message):
        state['safe'].append((state['clock'], message.linear.x, message.linear.y, message.angular.z))

    def health_callback(message):
        sample = {
            'sim_time': state['clock'],
            'state': message.state,
            'rejections': message.consecutive_rejections,
            'fitness': finite_or_none(message.fitness),
            'rmse': finite_or_none(message.rmse),
        }
        previous = state['health'][-1] if state['health'] else None
        if (previous is None or previous['state'] != sample['state'] or
                previous['rejections'] != sample['rejections'] or
                sample['sim_time'] is None or previous['sim_time'] is None or
                sample['sim_time'] - previous['sim_time'] >= 0.5):
            state['health'].append(sample)

    def status_callback(message):
        statuses = [status.status for status in message.status_list]
        if not state['statuses'] or state['statuses'][-1]['statuses'] != statuses:
            state['statuses'].append({'sim_time': state['clock'], 'statuses': statuses})

    def plan_callback(message):
        points = [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]
        state['plans'].append((state['clock'], points))

    node.create_subscription(Clock, '/clock', clock_callback, 20)
    node.create_subscription(Odometry, '/Odometry', fastlio_callback, 30)
    node.create_subscription(Odometry, '/ground_truth/odom', truth_callback, 30)
    node.create_subscription(Odometry, '/localization', localization_callback, 30)
    node.create_subscription(Twist, '/safe_cmd_vel', safe_callback, 30)
    node.create_subscription(LocalizationHealth, '/localization/health', health_callback, 30)
    node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', status_callback, 20)
    node.create_subscription(Path, '/plan', plan_callback, 10)
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    transport = NavigationActionTransport(client)

    def checkpoint(classification):
        report['classification'] = classification
        report['raw_health_samples'] = state['health']
        report['raw_status_samples'] = state['statuses']
        atomic_write(metrics_path, report)

    try:
        if not client.wait_for_server(timeout_sec=45):
            raise RuntimeError('NavigateToPose server unavailable')
        deadline = time.monotonic() + 45
        while (state['clock'] is None or not state['truth'] or not state['fastlio']) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if state['clock'] is None or not state['truth'] or not state['fastlio']:
            raise RuntimeError('sensor/FAST-LIO readiness unavailable')

        initial = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.pose.pose.orientation.w = 1.0
        for _ in range(6):
            pose.header.stamp = node.get_clock().now().to_msg()
            initial.publish(pose)
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
        deadline = time.monotonic() + 90
        while (not state['localization'] or not state['health'] or
               state['health'][-1]['state'] != LocalizationHealth.HEALTHY) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if (not state['localization'] or not state['health'] or
                state['health'][-1]['state'] != LocalizationHealth.HEALTHY):
            raise RuntimeError('initial localization did not become healthy')

        report['telemetry'] = {
            'clock_positive': state['clock'] > 0.0,
            'odom_stamps_positive': all(samples[-1][0] > 0.0 for samples in (state['truth'], state['localization'])),
            'fastlio_stamp_positive': state['fastlio'][-1] > 0.0,
            'stamps_monotonic_at_initialization': all((
                monotonic([item[0] for item in state['truth']]),
                monotonic([item[0] for item in state['localization']]),
                monotonic(state['fastlio']),
            )),
            'initial_clock_s': state['clock'],
        }
        if not all(value for value in report['telemetry'].values() if isinstance(value, bool)):
            raise RuntimeError(f"invalid timestamp telemetry: {report['telemetry']}")
        checkpoint('ready; no goals completed')

        rooms = {room['id']: room for room in json.loads(ROOMS_PATH.read_text())['rooms']}
        catalog = {
            room_id: tuple(float(value) for value in rooms[room_id]['goal_pose'])
            for room_id in ('room-1', 'room-2', 'room-3', 'mian')
        }
        routes = {
            'room-1': (('room-1', catalog['room-1']), ('hub', catalog['mian'])),
            'room-2': (('room-2', catalog['room-2']), ('hub', catalog['mian'])),
            'room-3': (('room-3', catalog['room-3']), ('hub', catalog['mian'])),
            'all': (
                ('room-1', catalog['room-1']),
                ('room-2', catalog['room-2']),
                ('room-3', catalog['room-3']),
                ('mian', catalog['mian']),
            ),
        }
        selected_route = os.environ.get('NAV2_CLEAN_ROUTE', 'all')
        if selected_route not in routes:
            raise ValueError(f'unknown NAV2_CLEAN_ROUTE: {selected_route}')
        route = routes[selected_route]
        report['selected_route'] = selected_route
        checkpoint('ready; catalog-only route recorded')
        for route_id, route_pose in route:
            goal_x, goal_y, goal_yaw = route_pose
            start_wall = time.monotonic()
            start_sim = state['clock']
            start_truth = state['truth'][-1]
            plan_start_count = len(state['plans'])
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x = goal_x
            goal.pose.pose.position.y = goal_y
            goal.pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(goal_yaw / 2.0)
            transport.send(goal)
            response_deadline = time.monotonic() + 20
            while transport.response_pending and time.monotonic() < response_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if transport.response_pending or transport.handle is None:
                raise RuntimeError(f'{route_id}: goal rejected or send timed out')
            cancelled = False
            while (not transport.terminal and time.monotonic() - start_wall < WALL_LIMIT and
                   state['clock'] - start_sim < SIM_LIMIT):
                rclpy.spin_once(node, timeout_sec=0.1)
            if not transport.terminal:
                cancelled = True
                transport.cancel()
                cancel_deadline = time.monotonic() + 15
                while not transport.terminal and time.monotonic() < cancel_deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
            result_status = transport.result if transport.terminal else None
            end_wall = time.monotonic()
            end_sim = state['clock']
            end_truth = state['truth'][-1]
            end_localization = state['localization'][-1]
            window_health = [
                item for item in state['health']
                if item['sim_time'] is not None and start_sim <= item['sim_time'] <= end_sim
            ]
            errors = nearest_errors(state['localization'], state['truth'], start_sim, end_sim)
            plan_points = state['plans'][-1][1] if len(state['plans']) > plan_start_count else []
            plan_step = max(1, len(plan_points) // 20)
            report['route'].append({
                'room_id': route_id,
                'requested_poses': [route_pose],
                'goal': [goal_x, goal_y, goal_yaw],
                'actual_plan': {
                    'point_count': len(plan_points),
                    'length_m': sum(math.hypot(current[0] - previous[0], current[1] - previous[1])
                                    for previous, current in zip(plan_points, plan_points[1:])),
                    'sampled_xy': plan_points[::plan_step],
                },
                'status': result_status,
                'status_name': {
                    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                    GoalStatus.STATUS_ABORTED: 'ABORTED',
                    GoalStatus.STATUS_CANCELED: 'CANCELED',
                }.get(result_status, str(result_status)),
                'cancelled_at_bound': cancelled,
                'elapsed_sim_s': end_sim - start_sim,
                'elapsed_wall_s': end_wall - start_wall,
                'actual_yaw_travel_rad': actual_turn(state['truth'], start_sim, end_sim),
                'commanded_angular_integral_rad': integrate(state['safe'], 3, start_sim, end_sim),
                'truth_start': list(start_truth[1:]),
                'truth_end': list(end_truth[1:]),
                'truth_xy_progress_m': math.hypot(end_truth[1] - start_truth[1], end_truth[2] - start_truth[2]),
                'truth_goal_error_m': math.hypot(end_truth[1] - goal_x, end_truth[2] - goal_y),
                'localization_goal_error_m': math.hypot(end_localization[1] - goal_x, end_localization[2] - goal_y),
                'truth_goal_yaw_error_rad': abs(angular_difference(end_truth[3], goal_yaw)),
                'localization_truth_error_max_m': max(errors, default=None),
                'health_states': sorted(set(item['state'] for item in window_health)),
                'max_rejections': max((item['rejections'] for item in window_health), default=None),
                'fitness_min': min((item['fitness'] for item in window_health if item['fitness'] is not None), default=None),
                'rmse_max': max((item['rmse'] for item in window_health if item['rmse'] is not None), default=None),
            })
            checkpoint(f'{len(report["route"])} of {len(route)} route actions checkpointed')
            if result_status != GoalStatus.STATUS_SUCCEEDED:
                report['complete'] = True
                report['classification'] = f'{route_id} failed; route stopped at first failed prerequisite'
                checkpoint(report['classification'])
                return 2

        localization_ok = all(
            item['health_states'] == [LocalizationHealth.HEALTHY] and
            item['max_rejections'] == 0 and
            item['localization_truth_error_max_m'] is not None and
            item['localization_truth_error_max_m'] <= 0.20
            for item in report['route']
        )
        navigation_ok = all(item['status'] == GoalStatus.STATUS_SUCCEEDED for item in report['route'])
        report['complete'] = True
        report['passed'] = localization_ok and navigation_ok
        report['classification'] = (
            'saved multiroom route passed with healthy localization'
            if report['passed'] else
            'navigation failure with healthy localization'
            if localization_ok else
            'localization degradation observed'
        )
        checkpoint(report['classification'])
        return 0 if report['passed'] else 2
    except BaseException as error:
        if transport is not None and transport.busy:
            try:
                transport.cancel()
                cancel_deadline = time.monotonic() + 15
                while not transport.terminal and time.monotonic() < cancel_deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
            except Exception:
                pass
        report['error'] = f'{type(error).__name__}: {error}'
        report['traceback'] = traceback.format_exc()
        checkpoint('consumer failed; partial route preserved')
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
