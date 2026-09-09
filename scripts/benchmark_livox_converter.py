#!/usr/bin/env python3
"""Measure real ROS conversion for a captured PointCloud2 replay.

The replay is synthetic: it injects one captured real CDR sample at 10 Hz wall
clock without changing its source stamp. One message is in flight at a time so
receipt monotonic time is attributable to that injection.
"""
import argparse
import copy
import hashlib
import json
import math
import struct
import time
from pathlib import Path

import rclpy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.node import Node
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2


class Replay(Node):
    def __init__(self, cloud, samples):
        super().__init__('captured_cloud_replay_benchmark')
        self.cloud = cloud
        self.samples = samples
        self.publisher = self.create_publisher(PointCloud2, '/mid360/scan/points/points', 10)
        self.adapted = []
        self.livox = []
        self.create_subscription(PointCloud2, '/mid360/points', self.on_adapted, 10)
        self.create_subscription(CustomMsg, '/livox/lidar', self.on_livox, 10)

    def on_adapted(self, message):
        self.adapted.append((time.monotonic(), serialize_message(message)))

    def on_livox(self, message):
        self.livox.append((time.monotonic(), serialize_message(message)))


def python_reference(source):
    source = copy.deepcopy(source)
    fields = {field.name: field for field in source.fields}
    points = []
    for index in range(source.width * source.height):
        offset = index * source.point_step
        if offset + source.point_step > len(source.data):
            break
        xyz = [struct.unpack_from('<f', source.data, offset + fields[name].offset)[0] for name in ('x', 'y', 'z')]
        if all(math.isfinite(value) for value in xyz):
            points.append(xyz)
    source.header.frame_id = 'livox_frame'
    adapted = source
    livox = CustomMsg()
    livox.header = source.header
    livox.timebase = source.header.stamp.sec * 1_000_000_000 + source.header.stamp.nanosec
    livox.point_num = len(points)
    livox.lidar_id = 0
    livox.points = [
        CustomPoint(
            offset_time=0 if len(points) < 2 else index * 100_000_000 // (len(points) - 1),
            x=x, y=y, z=z, reflectivity=128, tag=0x10, line=index % 4,
        )
        for index, (x, y, z) in enumerate(points)
    ]
    return serialize_message(adapted), serialize_message(livox), len(points)


def decoded_field_digest(message):
    return hashlib.sha256(repr(message).encode('utf-8')).hexdigest()


def percentile(values, percentile_value):
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * percentile_value)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cdr', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--samples', type=int, default=100)
    parser.add_argument('--rate-hz', type=float, default=10.0)
    args = parser.parse_args()
    source = deserialize_message(Path(args.cdr).read_bytes(), PointCloud2)
    expected_adapted, expected_livox, point_count = python_reference(source)
    rclpy.init()
    node = Replay(source, args.samples)
    try:
        deadline = time.monotonic() + 10.0
        while (node.publisher.get_subscription_count() < 1 or
               node.count_publishers('/mid360/points') < 1 or
               node.count_publishers('/livox/lidar') < 1) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.publisher.get_subscription_count() < 1:
            raise RuntimeError('converter source subscription did not connect')
        interval = 1.0 / args.rate_hz
        next_send = time.monotonic()
        adapted_latencies = []
        livox_latencies = []
        adapted_digests = set()
        livox_digests = set()
        for sample in range(args.samples):
            while time.monotonic() < next_send:
                rclpy.spin_once(node, timeout_sec=min(0.01, next_send - time.monotonic()))
            before_adapted = len(node.adapted)
            before_livox = len(node.livox)
            injected = time.monotonic()
            node.publisher.publish(source)
            deadline = injected + 5.0
            while (len(node.adapted) == before_adapted or len(node.livox) == before_livox) and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
            if len(node.adapted) == before_adapted or len(node.livox) == before_livox:
                raise RuntimeError(f'timed out waiting for output {sample}')
            adapted_time, adapted = node.adapted[-1]
            livox_time, livox = node.livox[-1]
            actual_adapted = deserialize_message(adapted, PointCloud2)
            expected_adapted_message = deserialize_message(expected_adapted, PointCloud2)
            if actual_adapted != expected_adapted_message:
                raise RuntimeError(f'adapted PointCloud2 field mismatch at sample {sample}')
            actual_message = deserialize_message(livox, CustomMsg)
            expected_message = deserialize_message(expected_livox, CustomMsg)
            if actual_message != expected_message:
                raise RuntimeError(f'Livox CustomMsg field mismatch at sample {sample}')
            adapted_digests.add(decoded_field_digest(actual_adapted))
            livox_digests.add(decoded_field_digest(actual_message))
            adapted_latencies.append(adapted_time - injected)
            livox_latencies.append(livox_time - injected)
            next_send += interval
        output = {
            'label': 'SYNTHETIC_10HZ_REPLAY_OF_CAPTURED_RAW_CDR',
            'samples': args.samples,
            'requested_rate_hz': args.rate_hz,
            'source_stamp': {'sec': source.header.stamp.sec, 'nanosec': source.header.stamp.nanosec},
            'source_layout': {'width': source.width, 'height': source.height, 'point_step': source.point_step, 'row_step': source.row_step, 'data_bytes': len(source.data)},
            'finite_point_count': point_count,
            'output_equality': {'adapted_all_messages': True, 'livox_all_messages': True, 'adapted_reference_cdr_sha256': hashlib.sha256(expected_adapted).hexdigest(), 'livox_reference_cdr_sha256': hashlib.sha256(expected_livox).hexdigest(), 'adapted_decoded_field_sha256': sorted(adapted_digests), 'livox_decoded_field_sha256': sorted(livox_digests)},
            'wall_latency_seconds': {
                'adapted': {'median': percentile(adapted_latencies, 0.5), 'p95': percentile(adapted_latencies, 0.95), 'max': max(adapted_latencies)},
                'livox': {'median': percentile(livox_latencies, 0.5), 'p95': percentile(livox_latencies, 0.95), 'max': max(livox_latencies)},
            },
        }
        Path(args.output).write_text(json.dumps(output, indent=2) + '\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
