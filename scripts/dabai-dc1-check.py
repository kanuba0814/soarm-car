#!/usr/bin/env python3
"""Check actual RGB-D messages, either live or from a rosbag2 directory."""
import argparse
import bisect
import json
import time
from collections import defaultdict

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

IMAGE_TOPICS = ['/camera/color/image_raw', '/camera/depth/image_raw']
INFO_TOPICS = ['/camera/color/camera_info', '/camera/depth/camera_info']

class Check:
    def __init__(self):
        self.stamps = defaultdict(list)
        self.images = {}
        self.infos = {}
        self.valid_depth = []
        self.errors = set()
        self.tf = 0

    def add(self, topic, msg):
        if topic == '/tf_static':
            self.tf += len(msg.transforms)
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        previous = self.stamps[topic]
        if previous and stamp <= previous[-1]:
            self.errors.add(f'{topic}: non-monotonic timestamp')
        previous.append(stamp)
        if topic in IMAGE_TOPICS:
            shape = [msg.width, msg.height, msg.encoding, msg.header.frame_id]
            if topic in self.images and self.images[topic] != shape:
                self.errors.add(f'{topic}: image format changed')
            self.images[topic] = shape
            if len(msg.data) != msg.height * msg.step:
                self.errors.add(f'{topic}: invalid data length')
            if topic == IMAGE_TOPICS[1] and len(previous) % 15 == 1:
                if msg.encoding != '16UC1':
                    self.errors.add('Expected OpenNI millimetre 16UC1 depth')
                else:
                    depth = np.frombuffer(bytes(msg.data), dtype='>u2' if msg.is_bigendian else '<u2')
                    self.valid_depth.append(float(np.count_nonzero(depth) / depth.size))
        else:
            self.infos[topic] = dict(width=msg.width, height=msg.height,
                                    frame_id=msg.header.frame_id, k=list(msg.k), d=list(msg.d))

    def result(self):
        rates = {}
        warnings = []
        for topic in IMAGE_TOPICS + INFO_TOPICS:
            stamps = self.stamps[topic]
            if len(stamps) < 20:
                self.errors.add(f'{topic}: fewer than 20 messages')
            span = stamps[-1] - stamps[0] if len(stamps) > 1 else 0
            rates[topic] = (len(stamps) - 1) / span if span > 0 else 0
            if topic in IMAGE_TOPICS:
                # Recording policy, not an ORB-SLAM3 accuracy guarantee.
                if rates[topic] < 15:
                    self.errors.add(f'{topic}: below recording minimum 15 Hz')
                elif rates[topic] < 25:
                    warnings.append(f'{topic}: {rates[topic]:.2f} Hz, below preferred 25 Hz; recording allowed')
                if len(stamps) > 1 and max(np.diff(stamps)) > 0.5:
                    self.errors.add(f'{topic}: stream gap exceeds 0.5 seconds')
        for image_topic, info_topic in zip(IMAGE_TOPICS, INFO_TOPICS):
            shape, info = self.images.get(image_topic), self.infos.get(info_topic)
            if shape and info:
                if shape[:2] != [info['width'], info['height']] or shape[3] != info['frame_id']:
                    self.errors.add(f'{image_topic}: CameraInfo mismatch')
                if info['k'][0] <= 0 or info['k'][4] <= 0:
                    self.errors.add(f'{info_topic}: invalid intrinsics')
        if not self.valid_depth or np.mean(self.valid_depth) < .01:
            self.errors.add('Depth is empty or less than 1% valid')
        if not self.tf:
            self.errors.add('Missing /tf_static')
        rgb, depth = self.stamps[IMAGE_TOPICS[0]], self.stamps[IMAGE_TOPICS[1]]
        deltas = []
        for stamp in depth:
            i = bisect.bisect_left(rgb, stamp)
            candidates = rgb[max(0, i-1):i+1]
            if candidates and rgb[0] <= stamp <= rgb[-1]:
                deltas.append(min(abs(stamp-x) for x in candidates) * 1000)
        p95 = float(np.percentile(deltas, 95)) if deltas else None
        if p95 is None or p95 > 40:
            self.errors.add('RGB-depth nearest timestamp p95 exceeds 40 ms')
        return dict(passed=not self.errors, errors=sorted(self.errors), warnings=warnings,
                    counts={k: len(v) for k, v in self.stamps.items()}, rates_hz=rates,
                    image_formats=self.images, camera_info=self.infos,
                    valid_depth_mean=float(np.mean(self.valid_depth)) if self.valid_depth else 0,
                    nearest_rgb_depth_p95_ms=p95, static_transforms=self.tf)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag')
    parser.add_argument('--seconds', type=float, default=6)
    args = parser.parse_args()
    check = Check()
    if args.bag:
        import rosbag2_py
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()
                 if t.name in IMAGE_TOPICS + INFO_TOPICS + ['/tf_static']}
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic in types:
                check.add(topic, deserialize_message(data, types[topic]))
    else:
        import rclpy
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from sensor_msgs.msg import Image, CameraInfo
        from tf2_msgs.msg import TFMessage
        rclpy.init()
        node = rclpy.create_node('dc1_recording_preflight')
        subscriptions = []
        for topic in IMAGE_TOPICS + INFO_TOPICS:
            subscriptions.append(node.create_subscription(
                Image if topic in IMAGE_TOPICS else CameraInfo, topic,
                lambda msg, topic=topic: check.add(topic, msg), QoSProfile(depth=30)))
        subscriptions.append(node.create_subscription(TFMessage, '/tf_static',
            lambda msg: check.add('/tf_static', msg),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        node.destroy_node()
        rclpy.shutdown()
    result = check.result()
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 2)

if __name__ == '__main__':
    main()
