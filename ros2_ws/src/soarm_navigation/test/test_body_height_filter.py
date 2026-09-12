#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import struct
import sys
import unittest

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField


MODULE_PATH = Path(__file__).parents[1] / 'soarm_navigation' / 'body_height_filter.py'
SPEC = importlib.util.spec_from_file_location('body_height_filter', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def transform(z=0.0):
    message = TransformStamped()
    message.transform.translation.z = z
    message.transform.rotation.w = 1.0
    return message


def cloud(records, datatype=PointField.FLOAT32, bigendian=False, organized=False):
    scalar_format = 'd' if datatype == PointField.FLOAT64 else 'f'
    scalar_size = 8 if datatype == PointField.FLOAT64 else 4
    point_step = scalar_size * 3 + 4
    packer = struct.Struct(('>' if bigendian else '<') + scalar_format * 3 + 'I')
    message = PointCloud2()
    message.header.frame_id = 'mid360_link'
    message.header.stamp.sec = 23
    message.header.stamp.nanosec = 456
    message.fields = [
        PointField(name='x', offset=0, datatype=datatype, count=1),
        PointField(name='y', offset=scalar_size, datatype=datatype, count=1),
        PointField(name='z', offset=scalar_size * 2, datatype=datatype, count=1),
        PointField(name='intensity', offset=scalar_size * 3, datatype=PointField.UINT32, count=1),
    ]
    message.is_bigendian = bigendian
    message.point_step = point_step
    message.is_dense = False
    encoded = [packer.pack(*record) for record in records]
    if organized:
        message.width = 2
        message.height = len(encoded) // 2
        message.row_step = point_step * 2 + 3
        message.data = b''.join(
            b''.join(encoded[index:index + 2]) + b'PAD'
            for index in range(0, len(encoded), 2))
    else:
        message.width = len(encoded)
        message.height = 1
        message.row_step = point_step * len(encoded)
        message.data = b''.join(encoded)
    return message, encoded


class FilterRecordsTest(unittest.TestCase):
    def test_rotated_nonunit_transform_preserves_retained_sensor_coordinates(self):
        source, records = cloud([(-.5, 0., 0., 1), (.5, 0., 0., 2)])
        pose = transform()
        pose.transform.rotation.y = 2 * math.sin(math.pi / 4)
        pose.transform.rotation.w = 2 * math.cos(math.pi / 4)
        result = MODULE.filter_cloud_records(source, pose)
        self.assertEqual(bytes(result.data), records[0])

    def test_ambiguous_fields_and_invalid_empty_cloud_tf_are_dropped(self):
        source, _ = cloud([(0., 0., .5, 1)])
        source.fields.append(source.fields[2])
        self.assertIsNone(MODULE.filter_cloud_records(source, transform()))
        empty, _ = cloud([])
        invalid = transform()
        invalid.transform.rotation.w = 0.0
        self.assertIsNone(MODULE.filter_cloud_records(empty, invalid))

    def test_float32_selection_is_body_relative_and_preserves_records(self):
        source, records = cloud([
            (1.0, 2.0, -0.01, 10),
            (2.0, 3.0, 0.11, 11),
            (3.0, 4.0, 1.51, 12),
        ])
        # Odom-frame drift is irrelevant: selection uses only base-frame z.
        output = MODULE.filter_cloud_records(source, transform(z=0.0))
        self.assertEqual(output.header, source.header)
        self.assertEqual(output.fields, source.fields)
        self.assertEqual(output.width, 1)
        self.assertEqual(output.height, 1)
        self.assertEqual(output.row_step, source.point_step)
        self.assertEqual(bytes(output.data), records[1])
        self.assertEqual(output.header.frame_id, 'mid360_link')

    def test_float64_big_endian_organized_padding_retains_exact_records(self):
        source, records = cloud([
            (1.0, 0.0, 0.09, 1),
            (2.0, 0.0, 0.10, 2),
            (3.0, 0.0, 1.50, 3),
            (4.0, 0.0, 1.51, 4),
        ], datatype=PointField.FLOAT64, bigendian=True, organized=True)
        output = MODULE.filter_cloud_records(source, transform())
        self.assertTrue(output.is_bigendian)
        self.assertEqual(output.point_step, source.point_step)
        self.assertEqual(bytes(output.data), records[1] + records[2])
        self.assertEqual(output.width, 2)
        self.assertEqual(output.row_step, 2 * source.point_step)

    def test_nonfinite_coordinates_are_dropped_and_all_floor_is_normal_empty_cloud(self):
        source, _ = cloud([
            (1.0, 0.0, 0.02, 1),
            (math.nan, 0.0, 0.50, 2),
            (1.0, math.inf, 0.50, 3),
        ])
        output = MODULE.filter_cloud_records(source, transform())
        self.assertIsNotNone(output)
        self.assertEqual(output.width, 0)
        self.assertEqual(output.height, 1)
        self.assertEqual(output.row_step, 0)
        self.assertEqual(bytes(output.data), b'')

    def test_translation_changes_selection_without_rewriting_cloud_coordinates(self):
        source, records = cloud([(1.0, 2.0, 0.03, 99)])
        output = MODULE.filter_cloud_records(source, transform(z=0.08))
        self.assertEqual(bytes(output.data), records[0])
        self.assertEqual(output.header.frame_id, source.header.frame_id)

    def test_malformed_or_unsupported_cloud_returns_none(self):
        source, _ = cloud([(1.0, 2.0, 0.5, 1)])
        source.fields[2].datatype = PointField.INT16
        self.assertIsNone(MODULE.filter_cloud_records(source, transform()))
        source, _ = cloud([(1.0, 2.0, 0.5, 1)])
        source.row_step -= 1
        self.assertIsNone(MODULE.filter_cloud_records(source, transform()))
        self.assertIsNone(MODULE.filter_cloud_records(source, transform(), 1.0, 0.0))


class AdapterDropTest(unittest.TestCase):
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    class Logger:
        def warn(self, *args, **kwargs):
            pass

    class MissingTransform:
        def lookup_transform(self, *args):
            raise MODULE.TransformException('missing')

    def test_missing_tf_and_malformed_input_publish_nothing(self):
        source, _ = cloud([(1.0, 2.0, 0.5, 1)])
        node = object.__new__(MODULE.BodyHeightFilter)
        node.publisher = self.Publisher()
        node.tf_buffer = self.MissingTransform()
        node.get_logger = lambda: self.Logger()
        node.filter_and_publish(source)
        self.assertEqual(node.publisher.messages, [])

        node.tf_buffer = type('Buffer', (), {
            'lookup_transform': lambda self, *args: transform(),
        })()
        source.fields[2].datatype = PointField.INT16
        node.filter_and_publish(source)
        self.assertEqual(node.publisher.messages, [])


if __name__ == '__main__':
    unittest.main()
