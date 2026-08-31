from copy import deepcopy

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster


SIMULATOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)
HARDWARE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
)


class AstraSimAdapter(Node):
    """Expose Gazebo RGB-D data through the legacy Astra ROS topic contract."""

    def __init__(self):
        super().__init__('astra_sim_adapter')
        self.color_publisher = self.create_publisher(
            Image, '/camera/color/image_raw', HARDWARE_QOS)
        self.depth_publisher = self.create_publisher(
            Image, '/camera/depth/image_raw', HARDWARE_QOS)
        self.color_info_publisher = self.create_publisher(
            CameraInfo, '/camera/color/camera_info', HARDWARE_QOS)
        self.depth_info_publisher = self.create_publisher(
            CameraInfo, '/camera/depth/camera_info', HARDWARE_QOS)

        self.create_subscription(
            Image, '/simulation/astra/color/image', self.on_color, SIMULATOR_QOS)
        self.create_subscription(
            Image, '/simulation/astra/depth/image', self.on_depth, SIMULATOR_QOS)
        self.create_subscription(
            CameraInfo, '/simulation/astra/camera_info', self.on_camera_info,
            SIMULATOR_QOS)
        self.camera_info = None
        self.publish_optical_transforms()

    def on_color(self, message):
        output = deepcopy(message)
        output.header.frame_id = 'camera_color_optical_frame'
        self.color_publisher.publish(output)
        self.publish_camera_info('color', output.header.stamp)

    def on_depth(self, message):
        output = deepcopy(message)
        output.header.frame_id = 'camera_depth_optical_frame'
        if message.encoding == '32FC1':
            depth_metres = np.frombuffer(message.data, dtype=np.float32)
            depth_millimetres = np.nan_to_num(
                depth_metres, nan=0.0, posinf=0.0, neginf=0.0)
            depth_millimetres = np.clip(
                depth_millimetres * 1000.0, 0.0, 65535.0).astype(np.uint16)
            output.encoding = '16UC1'
            output.is_bigendian = False
            output.step = message.width * 2
            output.data = depth_millimetres.tobytes()
        elif message.encoding != '16UC1':
            self.get_logger().error(
                f'Unsupported simulated depth encoding: {message.encoding}')
            return
        self.depth_publisher.publish(output)
        self.publish_camera_info('depth', output.header.stamp)

    def on_camera_info(self, message):
        self.camera_info = deepcopy(message)

    def publish_camera_info(self, stream, stamp):
        if self.camera_info is None:
            return
        output = deepcopy(self.camera_info)
        output.header.stamp = stamp
        output.header.frame_id = f'camera_{stream}_optical_frame'
        if stream == 'color':
            self.color_info_publisher.publish(output)
        else:
            self.depth_info_publisher.publish(output)

    def publish_optical_transforms(self):
        broadcaster = StaticTransformBroadcaster(self)
        transforms = []
        for stream in ('color', 'depth'):
            frame = TransformStamped()
            frame.header.stamp = self.get_clock().now().to_msg()
            frame.header.frame_id = 'camera_link'
            frame.child_frame_id = f'camera_{stream}_frame'
            frame.transform.rotation.w = 1.0
            transforms.append(frame)

            optical = TransformStamped()
            optical.header.stamp = frame.header.stamp
            optical.header.frame_id = frame.child_frame_id
            optical.child_frame_id = f'camera_{stream}_optical_frame'
            optical.transform.rotation.x = -0.5
            optical.transform.rotation.y = 0.5
            optical.transform.rotation.z = -0.5
            optical.transform.rotation.w = 0.5
            transforms.append(optical)
        broadcaster.sendTransform(transforms)
        self.static_transform_broadcaster = broadcaster


def main(args=None):
    rclpy.init(args=args)
    node = AstraSimAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
