#!/usr/bin/env python3
"""Capture one native cloud and measure its exact frozen-body self-mask residuals."""
import json
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, str(Path(__file__).parent))
from exploration_sensor_policy import SELF_BOXES, SELF_MODEL_SHA256, SELF_SPHERES, rotation_matrix


def signed_union_distance(points):
    distances = []
    for box in SELF_BOXES:
        minimum_x, maximum_x, minimum_y, maximum_y, minimum_z, maximum_z = box
        center = np.array([(minimum_x + maximum_x) / 2, (minimum_y + maximum_y) / 2, (minimum_z + maximum_z) / 2])
        half = np.array([(maximum_x - minimum_x) / 2, (maximum_y - minimum_y) / 2, (maximum_z - minimum_z) / 2])
        delta = np.abs(points - center) - half
        distances.append(np.linalg.norm(np.maximum(delta, 0), axis=1) + np.minimum(np.max(delta, axis=1), 0))
    for center_x, center_y, center_z, radius in SELF_SPHERES:
        distances.append(np.linalg.norm(points - np.array([center_x, center_y, center_z]), axis=1) - radius)
    return np.min(np.vstack(distances), axis=0)


class Measure(Node):
    def __init__(self, output):
        super().__init__('exploration_selfmask_measure')
        self.output = Path(output)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.subscription = self.create_subscription(PointCloud2, '/mid360/points', self.cloud, 10)

    def cloud(self, message):
        try:
            transform = self.buffer.lookup_transform('base_link', 'livox_frame', rclpy.time.Time.from_msg(message.header.stamp))
            translation, rotation = transform.transform.translation, transform.transform.rotation
            raw = np.array([(float(p['x']), float(p['y']), float(p['z'])) for p in point_cloud2.read_points(message, field_names=('x', 'y', 'z'), skip_nans=True)])
            matrix = rotation_matrix((rotation.x, rotation.y, rotation.z, rotation.w))
            body = raw @ matrix.T + np.array([translation.x, translation.y, translation.z])
            finite = np.isfinite(raw).all(axis=1) & np.isfinite(body).all(axis=1)
            distance = signed_union_distance(body[finite])
            positive = distance[distance > 0]
            result = {
                'model_sha256': SELF_MODEL_SHA256,
                'source_stamp': message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
                'base_link_from_livox_frame': {'translation': [translation.x, translation.y, translation.z], 'quaternion_xyzw': [rotation.x, rotation.y, rotation.z, rotation.w], 'rotation_matrix': matrix.tolist()},
                'raw_count': int(len(raw)), 'finite_count': int(finite.sum()),
                'signed_distance_m': {'min': float(distance.min()), 'max': float(distance.max()), 'positive_histogram_edges_m': [0, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2], 'positive_histogram_counts': np.histogram(positive, bins=[0, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2])[0].tolist()},
                'tolerance_counts': {str(value): int((distance <= value).sum()) for value in (0.0, 1e-6, 1e-5, 1e-4, 1e-3)},
                'nearest_positive_examples_base_link': body[finite][np.argsort(np.where(distance > 0, distance, np.inf))[:20]].tolist(),
            }
            self.output.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
            rclpy.shutdown()
        except Exception as error:
            self.get_logger().warn(str(error))


def main():
    rclpy.init(); node = Measure(sys.argv[1])
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
