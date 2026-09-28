#!/usr/bin/env python3
"""Measure commanded, wheel-odometry, and physical yaw through the safety gate."""

import json
import math
import os
import pathlib
import sys
import time
import traceback

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from soarm_interfaces.msg import LocalizationHealth

COMMAND_RAD_S = 0.20
DURATION_SIM_S = 5.0


def yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def unwrapped_travel(samples):
    return sum(delta(current[1], previous[1]) for previous, current in zip(samples, samples[1:]))


def integrate(samples):
    return sum(
        (previous[1] + current[1]) / 2.0 * max(0.0, current[0] - previous[0])
        for previous, current in zip(samples, samples[1:])
    )


def atomic_write(path, report):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    os.replace(temporary, path)


def main():
    output = pathlib.Path(sys.argv[1])
    output.parent.mkdir(parents=True, exist_ok=True)
    report = {
        'complete': False,
        'command_rad_s': COMMAND_RAD_S,
        'duration_sim_s': DURATION_SIM_S,
        'error': None,
    }
    atomic_write(output, report)
    rclpy.init()
    node = rclpy.create_node('nav2_clean_yaw_probe')
    state = {'clock': None, 'truth': [], 'wheel': [], 'safe': [], 'health': []}

    def clock_callback(message):
        state['clock'] = message.clock.sec + message.clock.nanosec / 1e9

    def odom_callback(key):
        def callback(message):
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            state[key].append((stamp, yaw(message.pose.pose.orientation), message.twist.twist.angular.z))
        return callback

    def safe_callback(message):
        if state['clock'] is not None:
            state['safe'].append((state['clock'], message.angular.z))

    def health_callback(message):
        state['health'].append((state['clock'], message.state, message.consecutive_rejections))

    node.create_subscription(Clock, '/clock', clock_callback, 20)
    node.create_subscription(Odometry, '/ground_truth/odom', odom_callback('truth'), 50)
    node.create_subscription(Odometry, '/wheel/odom_raw', odom_callback('wheel'), 50)
    node.create_subscription(Twist, '/safe_cmd_vel', safe_callback, 50)
    node.create_subscription(LocalizationHealth, '/localization/health', health_callback, 20)
    command_publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    initial_publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    try:
        deadline = time.monotonic() + 60
        while (state['clock'] is None or not state['truth'] or not state['wheel']) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if state['clock'] is None or not state['truth'] or not state['wheel']:
            raise RuntimeError('clock, ground truth, or wheel odometry unavailable')

        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = 'map'
        initial.pose.pose.orientation.w = 1.0
        for _ in range(6):
            initial.header.stamp = node.get_clock().now().to_msg()
            initial_publisher.publish(initial)
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
        deadline = time.monotonic() + 90
        while (not state['health'] or state['health'][-1][1] != LocalizationHealth.HEALTHY) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not state['health'] or state['health'][-1][1] != LocalizationHealth.HEALTHY:
            raise RuntimeError('localization did not become healthy')

        command = Twist()
        command.angular.z = COMMAND_RAD_S
        start_sim = state['clock']
        end_sim = start_sim + DURATION_SIM_S
        while state['clock'] < end_sim:
            command_publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.02)
        command_publisher.publish(Twist())
        settle_end = state['clock'] + 1.0
        while state['clock'] < settle_end:
            command_publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.02)

        truth = [item for item in state['truth'] if start_sim <= item[0] <= end_sim]
        wheel = [item for item in state['wheel'] if start_sim <= item[0] <= end_sim]
        safe = [item for item in state['safe'] if start_sim <= item[0] <= end_sim]
        health = [item for item in state['health'] if item[0] is not None and start_sim <= item[0] <= end_sim]
        if len(truth) < 2 or len(wheel) < 2 or len(safe) < 2:
            raise RuntimeError('insufficient probe samples')
        safe_integral = integrate(safe)
        truth_yaw = unwrapped_travel(truth)
        wheel_yaw = unwrapped_travel(wheel)
        report.update({
            'complete': True,
            'start_sim_s': start_sim,
            'end_sim_s': end_sim,
            'safe_cmd_integral_rad': safe_integral,
            'ground_truth_yaw_rad': truth_yaw,
            'wheel_odom_yaw_rad': wheel_yaw,
            'truth_to_safe_ratio': truth_yaw / safe_integral,
            'wheel_to_safe_ratio': wheel_yaw / safe_integral,
            'truth_to_wheel_ratio': truth_yaw / wheel_yaw,
            'sample_counts': {'safe': len(safe), 'ground_truth': len(truth), 'wheel_odom': len(wheel)},
            'health_states': sorted(set(item[1] for item in health)),
            'max_rejections': max((item[2] for item in health), default=None),
        })
        atomic_write(output, report)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0
    except BaseException as error:
        command_publisher.publish(Twist())
        report['error'] = f'{type(error).__name__}: {error}'
        report['traceback'] = traceback.format_exc()
        atomic_write(output, report)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
