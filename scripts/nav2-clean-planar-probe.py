#!/usr/bin/env python3
"""Measure pure body-X/Y commands against wheel odometry and physical truth."""

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

COMMAND_M_S = 0.15
DURATION_SIM_S = 3.0


def yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def atomic_write(path, report):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    os.replace(temporary, path)


def body_displacement(start, end):
    dx = end[1] - start[1]
    dy = end[2] - start[2]
    cosine = math.cos(start[3])
    sine = math.sin(start[3])
    return cosine * dx + sine * dy, -sine * dx + cosine * dy


def integrate(samples, index):
    return sum(
        (previous[index] + current[index]) / 2.0 * max(0.0, current[0] - previous[0])
        for previous, current in zip(samples, samples[1:])
    )


def main():
    output = pathlib.Path(sys.argv[1])
    output.parent.mkdir(parents=True, exist_ok=True)
    report = {'complete': False, 'command_m_s': COMMAND_M_S,
              'duration_sim_s_per_axis': DURATION_SIM_S, 'error': None, 'axes': []}
    atomic_write(output, report)
    rclpy.init()
    node = rclpy.create_node('nav2_clean_planar_probe')
    state = {'clock': None, 'truth': [], 'wheel': [], 'safe': [], 'health': []}

    def clock_callback(message):
        state['clock'] = message.clock.sec + message.clock.nanosec / 1e9

    def odom_callback(key):
        def callback(message):
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            pose = message.pose.pose
            state[key].append((stamp, pose.position.x, pose.position.y, yaw(pose.orientation)))
        return callback

    def safe_callback(message):
        if state['clock'] is not None:
            state['safe'].append((state['clock'], message.linear.x, message.linear.y))

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

        for axis, index in (('x', 1), ('y', 2)):
            command = Twist()
            setattr(command.linear, axis, COMMAND_M_S)
            start_sim = state['clock']
            start_truth = state['truth'][-1]
            start_wheel = state['wheel'][-1]
            end_sim = start_sim + DURATION_SIM_S
            safe_start = len(state['safe'])
            while state['clock'] < end_sim:
                command_publisher.publish(command)
                rclpy.spin_once(node, timeout_sec=0.02)
            command_publisher.publish(Twist())
            settle_end = state['clock'] + 0.75
            while state['clock'] < settle_end:
                command_publisher.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.02)
            safe = [item for item in state['safe'][safe_start:] if start_sim <= item[0] <= end_sim]
            truth_delta = body_displacement(start_truth, state['truth'][-1])
            wheel_delta = body_displacement(start_wheel, state['wheel'][-1])
            command_integral = integrate(safe, index)
            measured_index = 0 if axis == 'x' else 1
            truth_axis = truth_delta[measured_index]
            wheel_axis = wheel_delta[measured_index]
            report['axes'].append({
                'axis': axis,
                'safe_command_integral_m': command_integral,
                'ground_truth_body_displacement_m': truth_axis,
                'wheel_odom_body_displacement_m': wheel_axis,
                'ground_truth_cross_axis_m': truth_delta[1 - measured_index],
                'wheel_odom_cross_axis_m': wheel_delta[1 - measured_index],
                'truth_to_safe_ratio': truth_axis / command_integral,
                'wheel_to_safe_ratio': wheel_axis / command_integral,
                'truth_to_wheel_ratio': truth_axis / wheel_axis,
                'sample_count': len(safe),
            })
            atomic_write(output, report)
        report['health_states'] = sorted(set(item[1] for item in state['health']))
        report['max_rejections'] = max(item[2] for item in state['health'])
        report['complete'] = True
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
