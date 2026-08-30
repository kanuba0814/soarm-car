#!/usr/bin/env python3

import copy
import threading

import numpy as np
import rclpy
import tf2_ros
try:
    import tf_transformations
except ImportError:
    from tf2_ros import transformations as tf_transformations
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class TransformFusionNode(Node):
    """The only Nav2 boundary between FAST-LIO's camera_init/body and map/odom/base."""

    def __init__(self):
        super().__init__('transform_fusion')
        self.declare_parameter('freq_pub_localization', 50.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom_combined')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('body_frame', 'body')
        self.declare_parameter('base_to_body_xyz', [0.0, 0.0, 0.232])
        self.declare_parameter('base_to_body_rpy', [0.0, 0.0, 0.0])
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.body_frame = self.get_parameter('body_frame').value
        xyz = self.get_parameter('base_to_body_xyz').value
        rpy = self.get_parameter('base_to_body_rpy').value
        self.base_to_body = tf_transformations.translation_matrix(xyz) @ tf_transformations.euler_matrix(*rpy)
        self.lock = threading.Lock()
        self.current_odom = None
        self.map_to_camera_init = None
        self.create_subscription(Odometry, '/Odometry', self.save_odom, 10)
        self.create_subscription(Odometry, '/map_to_odom', self.save_correction, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.localization_pub = self.create_publisher(Odometry, '/localization', 20)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_timer(1.0 / self.get_parameter('freq_pub_localization').value, self.publish)

    def save_odom(self, message):
        with self.lock:
            self.current_odom = message

    def save_correction(self, message):
        with self.lock:
            self.map_to_camera_init = message

    @staticmethod
    def pose_matrix(message):
        pose = message.pose.pose
        position = pose.position
        orientation = pose.orientation
        return (tf_transformations.translation_matrix([position.x, position.y, position.z]) @
                tf_transformations.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w]))

    @staticmethod
    def fill_pose(message, matrix):
        xyz = tf_transformations.translation_from_matrix(matrix)
        quat = tf_transformations.quaternion_from_matrix(matrix)
        message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = xyz
        (message.pose.pose.orientation.x, message.pose.pose.orientation.y,
         message.pose.pose.orientation.z, message.pose.pose.orientation.w) = quat

    def send_tf(self, parent, child, matrix, stamp):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        xyz = tf_transformations.translation_from_matrix(matrix)
        quat = tf_transformations.quaternion_from_matrix(matrix)
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = xyz
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = quat
        self.tf_broadcaster.sendTransform(transform)

    def publish(self):
        with self.lock:
            raw_odom = copy.deepcopy(self.current_odom)
            correction = copy.deepcopy(self.map_to_camera_init)
        if raw_odom is None:
            return

        # A=T_baseFootprint_body and O=T_cameraInit_body. Nav2 gets
        # T_odom_base=A*O*inv(A). No map edge exists until ICP accepts M.
        odom_to_base = self.base_to_body @ self.pose_matrix(raw_odom) @ np.linalg.inv(self.base_to_body)
        odom = Odometry()
        odom.header.stamp = raw_odom.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        self.fill_pose(odom, odom_to_base)
        odom.pose.covariance = raw_odom.pose.covariance
        odom.twist = raw_odom.twist
        self.odom_pub.publish(odom)
        self.send_tf(self.odom_frame, self.base_frame, odom_to_base, raw_odom.header.stamp)

        if correction is None:
            return
        map_to_odom = self.pose_matrix(correction) @ np.linalg.inv(self.base_to_body)
        self.send_tf(self.map_frame, self.odom_frame, map_to_odom, raw_odom.header.stamp)
        localization = copy.deepcopy(odom)
        localization.header.frame_id = self.map_frame
        self.fill_pose(localization, map_to_odom @ odom_to_base)
        self.localization_pub.publish(localization)


def main(args=None):
    rclpy.init(args=args)
    node = TransformFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
