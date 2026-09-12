"""Publish one configured map pose after FAST-LIO localization inputs are live."""
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class InitialMapPose(Node):
    def __init__(self):
        super().__init__('initial_map_pose')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('timeout', 45.0)
        self.odom_ready = False
        self.map_ready = False
        self.completed = False
        self.initial_pose = None
        self.remaining_publishes = 0
        self.next_publish = None
        values = [float(self.get_parameter(name).value) for name in ('initial_x', 'initial_y', 'initial_yaw', 'timeout')]
        if not all(math.isfinite(value) for value in values) or values[-1] <= 0:
            raise ValueError('Initial pose must be finite and initialization timeout must be positive')
        self.deadline = time.monotonic() + values[-1]
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(Odometry, '/Odometry', self._odometry, 10)
        self.create_subscription(PointCloud2, '/global_map', self._global_map, 1)

    def _odometry(self, _message):
        self.odom_ready = True
        self._publish_when_ready()

    def _global_map(self, _message):
        self.map_ready = True
        self._publish_when_ready()

    def _publish_when_ready(self):
        if self.initial_pose is not None or self.completed or not (self.odom_ready and self.map_ready):
            return
        yaw = float(self.get_parameter('initial_yaw').value)
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = float(self.get_parameter('initial_x').value)
        pose.pose.pose.position.y = float(self.get_parameter('initial_y').value)
        pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
        pose.pose.covariance[0] = 0.25
        pose.pose.covariance[7] = 0.25
        pose.pose.covariance[35] = 0.0685
        self.initial_pose = pose
        self.remaining_publishes = 6
        self.next_publish = time.monotonic()
        self.get_logger().info('Sending configured /initialpose after /Odometry and /global_map.')

    def publish_pending(self):
        if self.remaining_publishes and time.monotonic() >= self.next_publish:
            self.initial_pose.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.initial_pose)
            self.remaining_publishes -= 1
            self.next_publish = time.monotonic() + 0.5
            if not self.remaining_publishes:
                self.completed = True
                self.get_logger().info('Published configured /initialpose six times.')

    def timed_out(self):
        return not self.completed and time.monotonic() >= self.deadline


def main(args=None):
    rclpy.init(args=args)
    node = InitialMapPose()
    try:
        while rclpy.ok() and not node.completed and not node.timed_out():
            node.publish_pending()
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.timed_out():
            node.get_logger().error('Timed out waiting for /Odometry and /global_map; /initialpose was not published.')
            return 1
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
