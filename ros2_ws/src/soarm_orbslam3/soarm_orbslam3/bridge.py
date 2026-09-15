from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class Orbslam3RgbdBridge(Node):
    """Boundary node; refuses to run ORB-SLAM3 until physical profile is complete."""
    def __init__(self):
        super().__init__('orbslam3_rgbd_bridge')
        self.declare_parameter('profile', '')
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('color_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_info_topic', '/camera/depth/camera_info')
        self.pub = self.create_publisher(DiagnosticArray, '~/diagnostics', 10)
        self.timer = self.create_timer(1.0, self.publish_blocked)
        self.get_logger().error('ORB-SLAM3 core is not bundled yet; physical DC1 evidence is required before activation')

    def publish_blocked(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(name='orbslam3_rgbd_bridge', level=DiagnosticStatus.ERROR,
                                  message='BLOCKED_UNTIL_HARDWARE_EVIDENCE')
        status.values = [KeyValue(key='map_tf_owner', value='none'),
                         KeyValue(key='profile', value=str(self.get_parameter('profile').value)),
                         KeyValue(key='reason', value='No verified DC1 intrinsics/depth unit/registration')]
        msg.status = [status]
        self.pub.publish(msg)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = Orbslam3RgbdBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
