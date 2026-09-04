#!/usr/bin/env python3
import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class ContractCheck(Node):
    def __init__(self):
        super().__init__('astra_contract_check')
        self.results = {}
        self.create_subscription(
            Image, '/camera/color/image_raw', self.on_color, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/depth/image_raw', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/camera/depth/camera_info', self.on_info,
            qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/camera/depth/points', self.on_cloud,
            qos_profile_sensor_data)

    def on_color(self, message):
        assert message.width == 640 and message.height == 480
        assert message.header.frame_id == 'camera_color_optical_frame'
        assert message.encoding.lower() in ('rgb8', 'bgr8')
        assert message.step >= message.width * 3
        assert len(message.data) >= message.step * message.height
        if np.count_nonzero(np.frombuffer(message.data, dtype=np.uint8)) <= 100:
            return
        self.results['color'] = True

    def on_depth(self, message):
        assert message.width == 640 and message.height == 480
        assert message.header.frame_id == 'camera_depth_optical_frame'
        assert message.encoding == '16UC1'
        assert message.step >= message.width * 2
        assert len(message.data) >= message.step * message.height
        byte_order = '>u2' if message.is_bigendian else '<u2'
        depth = np.frombuffer(message.data, dtype=byte_order)
        if np.count_nonzero(depth) <= 100:
            return
        self.results['depth'] = True

    def on_info(self, message):
        assert message.width == 640 and message.height == 480
        assert message.header.frame_id == 'camera_depth_optical_frame'
        assert message.distortion_model
        assert math.isfinite(message.k[0]) and message.k[0] > 0.0
        assert math.isfinite(message.k[4]) and message.k[4] > 0.0
        self.results['camera_info'] = True

    def on_cloud(self, message):
        assert message.width * message.height > 100
        assert message.header.frame_id == 'camera_depth_optical_frame'
        assert message.point_step >= 12
        assert len(message.data) >= message.row_step * message.height
        fields = {field.name: field for field in message.fields}
        for name in ('x', 'y', 'z'):
            assert name in fields
            assert fields[name].datatype == PointField.FLOAT32
        count = message.width * message.height
        float_format = '>f4' if message.is_bigendian else '<f4'
        xyz = np.ndarray(
            shape=(count,),
            dtype={
                'names': ('x', 'y', 'z'),
                'formats': (float_format, float_format, float_format),
                'offsets': (fields['x'].offset, fields['y'].offset, fields['z'].offset),
                'itemsize': message.point_step,
            },
            buffer=message.data,
        )
        finite = np.isfinite(xyz['x']) & np.isfinite(xyz['y']) & np.isfinite(xyz['z'])
        if np.count_nonzero(finite) <= 100:
            return
        self.results['point_cloud'] = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=25.0)
    arguments = parser.parse_args()
    rclpy.init()
    node = ContractCheck()
    deadline = time.monotonic() + arguments.timeout
    try:
        while time.monotonic() < deadline and len(node.results) < 4:
            rclpy.spin_once(node, timeout_sec=0.5)
        missing = sorted({'color', 'depth', 'camera_info', 'point_cloud'} - node.results.keys())
        if missing:
            raise RuntimeError(f'Timed out waiting for valid Astra data: {missing}')
        print('Astra RGB-D, CameraInfo, and finite XYZ payload contract passed.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
