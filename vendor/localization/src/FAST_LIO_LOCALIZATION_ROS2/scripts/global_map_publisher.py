#!/usr/bin/env python3

import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2

class MapPublisherNode(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.declare_parameter('map_file_path', '/home/myungw00/ROS2/localization_ws/src/FAST_LIO_LOCALIZATION_ROS2/PCD/highway2.pcd')
        self.declare_parameter('interval', 5)
        path = self.get_parameter('map_file_path').value
        interval = self.get_parameter('interval').value
        
        if not path:
            raise RuntimeError('map_file_path is required for FAST-LIO localization')
        self.global_map = o3d.io.read_point_cloud(path)
        points = np.asarray(self.global_map.points)
        if points.size == 0 or points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise RuntimeError(f'PCD is missing, empty, or non-finite: {path}')
        self.get_logger().info(f'Loaded {len(points)} finite map points from: {path}')

        self.pub_map = self.create_publisher(PointCloud2, '/global_map', 1)
        self.create_timer(interval, self.publish_map)
        self.get_logger().info(f'Interval for publishing map: {interval} seconds')
        self.get_logger().info('Map Publisher Node Initialized')

    def publish_map(self):
        if self.global_map is None:
            self.get_logger().warn('Global map is not loaded; skipping publish')
            return
        points = np.asarray(self.global_map.points)
        if points.size == 0:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        try:
            cloud = point_cloud2.create_cloud_xyz32(header, points.tolist())
        except AttributeError:
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            cloud = point_cloud2.create_cloud(header, fields, points.tolist())
        self.pub_map.publish(cloud)
        # self.get_logger().info('Published global map')


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()        