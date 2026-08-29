from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class OdomPath(Node):
    """Publish a bounded RViz path from the hardware-compatible odometry seam."""

    def __init__(self):
        super().__init__('odom_path')
        self.declare_parameter('max_poses', 2000)
        max_poses = int(self.get_parameter('max_poses').value)
        self.poses = deque(maxlen=max_poses)
        self.publisher = self.create_publisher(Path, '/odom_path', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.on_odom, 20)

    def on_odom(self, message):
        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        self.poses.append(pose)
        path = Path()
        path.header = message.header
        path.poses = list(self.poses)
        self.publisher.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPath()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
