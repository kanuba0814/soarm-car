#!/usr/bin/env python3
"""CPU-only ROS seam diagnostic for the AWS known-self-pose pilot.

This starts no Gazebo, renderer, or motion publisher. It owns mock /clock,
source-pose, and lifecycle services in a network-none container.
"""
import importlib.util
import math
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

spec = importlib.util.spec_from_file_location('knownselfpose_pilot', Path(__file__).parents[1] / 'scripts/aws_knownselfpose_pilot.py')
pilot_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pilot_module)


class FakeNativePoseStream:
    def __init__(self):
        self.index = 0
        self.records = []
        for index in range(8):
            stamp_ns = 1_000_000_000 + index * 10_000_000
            self.records.append({'source_stamp_ns': stamp_ns, 'entity': {'name': 'kuruma', 'position': {'x': index * .01, 'y': 0.0, 'z': 0.08}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}, 'envelope': {}})

    def drain(self, after_ns):
        while self.index < len(self.records) and self.records[self.index]['source_stamp_ns'] <= after_ns:
            self.index += 1
        if self.index >= len(self.records):
            return []
        record = self.records[self.index]
        self.index += 1
        return [record]

    def snapshot(self):
        return self.records[:self.index], []

    def close(self):
        pass


class MockRuntime(Node):
    def __init__(self):
        super().__init__('knownselfpose_mock_runtime')
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.pose_pub = self.create_publisher(TFMessage, '/world/default/dynamic_pose/info', 10)
        self.odom = []
        self.create_subscription(Odometry, '/known_self_pose/odom', self.odom.append, 10)
        self.state_calls = {name: 0 for name in ('/planner_server', '/controller_server', '/bt_navigator')}
        self.allow_active = False
        self.stamp = 1.0
        self.pose_x = 0.0
        self.lifecycle_services = [self.create_service(GetState, f'{name}/get_state', self.state_callback(name)) for name in self.state_calls]
        self.timer = self.create_timer(.05, self.publish)

    def state_callback(self, name):
        def callback(_request, response):
            self.state_calls[name] += 1
            # Delayed responses make a synchronous implementation starve source TF.
            time.sleep(.05)
            response.current_state.id = State.PRIMARY_STATE_ACTIVE if self.allow_active else State.PRIMARY_STATE_INACTIVE
            response.current_state.label = 'active' if self.allow_active else 'inactive'
            return response
        return callback

    def publish(self):
        clock = Clock(); clock.clock.sec = int(self.stamp); clock.clock.nanosec = int((self.stamp % 1.0) * 1e9)
        self.clock_pub.publish(clock)
        transform = TransformStamped()
        transform.header.stamp = clock.clock
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'kuruma'
        transform.transform.translation.x = self.pose_x
        transform.transform.rotation.w = 1.0
        self.pose_pub.publish(TFMessage(transforms=[transform]))
        self.stamp += .05
        self.pose_x += .01


def main():
    pilot_module.NativePoseStream = FakeNativePoseStream
    pilot = pilot_module.Pilot('/tmp/aws-knownselfpose-node-diagnostic')
    runtime = MockRuntime()
    executor = SingleThreadedExecutor()
    executor.add_node(runtime)
    running = True

    def serve():
        while running and rclpy.ok():
            executor.spin_once(timeout_sec=.05)

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    listener_node = Node('knownselfpose_independent_tf_listener')
    executor.add_node(listener_node)
    buffer = Buffer()
    listener = TransformListener(buffer, listener_node, spin_thread=False)
    try:
        # Inactive must never satisfy readiness even though the word contains "active".
        deadline = time.monotonic() + .45
        try:
            pilot.wait_nav(timeout_seconds=.35)
            raise AssertionError('inactive lifecycle state accepted')
        except RuntimeError:
            pass
        assert all(count >= 1 for count in runtime.state_calls.values()), runtime.state_calls
        assert time.monotonic() <= deadline + .3, 'lifecycle deadline escaped its bounded allowance'

        # While delayed inactive calls are served, pilot's sole executor continues TF callbacks.
        runtime.allow_active = True
        pilot.wait_nav(timeout_seconds=2.0)
        for _ in range(30):
            pilot.spin_once(.05)
            try:
                transform = buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                break
            except Exception:
                transform = None
        assert transform is not None, 'independent TF listener did not receive map->base_link'
        assert transform.header.stamp.sec >= 1, 'TF did not retain source/clock time'
        assert runtime.odom and abs(runtime.odom[-1].twist.twist.linear.x) > .01, 'source finite-difference twist stayed zero'

        # Stop source updates, advance simulated clock, and verify stale source pose is not republished.
        published_stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
        runtime.destroy_timer(runtime.timer)
        # Advance simulated time without publishing another source observation.
        runtime.stamp = published_stamp + pilot_module.SOURCE_FRESHNESS_SECONDS + 1.0
        clock = Clock(); clock.clock.sec = int(runtime.stamp); clock.clock.nanosec = int((runtime.stamp % 1.0) * 1e9)
        runtime.clock_pub.publish(clock)
        for _ in range(8):
            pilot.spin_once(.05)
        assert stamp_seconds(buffer.lookup_transform('map', 'base_link', rclpy.time.Time()).header.stamp) <= published_stamp + .1
        print('PASS node diagnostic: pending lifecycle calls preserved TF, inactive rejected, deadline bounded, source-time TF fresh, stale blocked, body twist nonzero')
    finally:
        listener.unregister()
        executor.remove_node(listener_node)
        listener_node.destroy_node()
        pilot.node.destroy_node()
        runtime.destroy_node()
        executor.shutdown()
        running = False
        thread.join(timeout=1)
        rclpy.shutdown()


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


if __name__ == '__main__':
    main()
