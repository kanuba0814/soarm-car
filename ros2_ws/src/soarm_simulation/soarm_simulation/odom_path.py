from collections import deque
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class OdomPath(Node):
    """Publish a bounded RViz path from the hardware-compatible odometry seam."""

    def __init__(self):
        super().__init__('odom_path')
        self.declare_parameter('max_poses', 2000)
        self.declare_parameter('publish_rate_hz', 0.0)
        max_poses = int(self.get_parameter('max_poses').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.publish_interval = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.0
        self.last_publish_wall = None
        self.poses = deque(maxlen=max_poses)
        self.publisher = self.create_publisher(Path, '/odom_path', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.on_odom, 20)

    def on_odom(self, message):
        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        self.poses.append(pose)
        if self.publisher.get_subscription_count() == 0:
            return
        now = time.monotonic()
        if self.publish_interval and self.last_publish_wall is not None and now - self.last_publish_wall < self.publish_interval:
            return
        path = Path()
        path.header = message.header
        path.poses = list(self.poses)
        self.publisher.publish(path)
        self.last_publish_wall = now


def main(args=None):
    rclpy.init(args=args)
    node = OdomPath()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
