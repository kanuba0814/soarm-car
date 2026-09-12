#!/usr/bin/env python3
"""Forward only obstacle records whose positions are in a body-height band."""

import math
import struct

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointField, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


MIN_BODY_Z = 0.10
MAX_BODY_Z = 1.50
_FLOAT_FORMATS = {
    PointField.FLOAT32: ('f', 4),
    PointField.FLOAT64: ('d', 8),
}


def _xyz_readers(cloud):
    """Return unpackers for scalar x/y/z fields, or None for unsafe layouts."""
    if cloud.point_step <= 0 or cloud.width < 0 or cloud.height < 0 or (cloud.height == 0 and cloud.width != 0):
        return None
    if cloud.row_step < cloud.width * cloud.point_step:
        return None
    if len(cloud.data) < cloud.row_step * cloud.height:
        return None
    fields = {field.name: field for field in cloud.fields}
    if len(fields) != len(cloud.fields):
        return None
    endian = '>' if cloud.is_bigendian else '<'
    readers = []
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        if field is None or field.count != 1 or field.datatype not in _FLOAT_FORMATS:
            return None
        format_code, size = _FLOAT_FORMATS[field.datatype]
        if field.offset < 0 or field.offset + size > cloud.point_step:
            return None
        readers.append((field.offset, struct.Struct(endian + format_code)))
    return readers


def _body_z_coefficients(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
    values = (translation.x, translation.y, translation.z, qx, qy, qz, qw)
    if not all(math.isfinite(value) for value in values):
        return None
    norm = qx * qx + qy * qy + qz * qz + qw * qw
    if norm <= 0.0 or not math.isfinite(norm):
        return None
    # TF quaternions are normally normalized; normalize minor numerical drift.
    scale = 1.0 / math.sqrt(norm)
    qx, qy, qz, qw = (qx * scale, qy * scale, qz * scale, qw * scale)
    return (2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx),
            1 - 2 * (qx * qx + qy * qy), translation.z)


def filter_cloud_records(cloud, transform, min_body_z=MIN_BODY_Z, max_body_z=MAX_BODY_Z):
    """Return original retained records, selected only using body-frame height."""
    if (not math.isfinite(min_body_z) or not math.isfinite(max_body_z) or
            min_body_z > max_body_z):
        return None
    readers = _xyz_readers(cloud)
    coefficients = _body_z_coefficients(transform)
    if readers is None or coefficients is None:
        return None
    # One TF validation/rotation per cloud, not per lidar record.
    zx, zy, zz, offset_z = coefficients
    retained = bytearray()
    for row in range(cloud.height):
        row_base = row * cloud.row_step
        for column in range(cloud.width):
            record_base = row_base + column * cloud.point_step
            try:
                x, y, z = (reader.unpack_from(cloud.data, record_base + offset)[0]
                           for offset, reader in readers)
            except struct.error:
                return None
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            body_z = zx * x + zy * y + zz * z + offset_z
            if min_body_z <= body_z <= max_body_z:
                retained.extend(cloud.data[record_base:record_base + cloud.point_step])
    output = PointCloud2()
    output.header = cloud.header
    output.height = 1
    output.width = len(retained) // cloud.point_step
    output.fields = cloud.fields
    output.is_bigendian = cloud.is_bigendian
    output.point_step = cloud.point_step
    output.row_step = output.width * output.point_step
    output.data = bytes(retained)
    output.is_dense = cloud.is_dense
    return output


class BodyHeightFilter(Node):
    """TF-gated PointCloud2 selector for the Nav2 voxel observation topic."""

    def __init__(self):
        super().__init__('body_height_filter')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            PointCloud2, '/navigation/obstacle_points', qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PointCloud2, '/mid360/points', self.filter_and_publish, qos_profile_sensor_data)

    def filter_and_publish(self, cloud):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_footprint', cloud.header.frame_id,
                Time.from_msg(cloud.header.stamp), Duration(seconds=0.0))
        except TransformException as error:
            self.get_logger().warn(
                'Dropping obstacle cloud without base_footprint transform: %s' % error,
                throttle_duration_sec=2.0)
            return
        output = filter_cloud_records(cloud, transform)
        if output is None:
            self.get_logger().warn(
                'Dropping malformed or unsupported PointCloud2 obstacle input.',
                throttle_duration_sec=2.0)
            return
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = BodyHeightFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
