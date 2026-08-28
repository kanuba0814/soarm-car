#!/usr/bin/env python3
# coding=utf-8

import copy
import threading

import rclpy
from rclpy.node import Node
import numpy as np

import tf2_ros
try:
    import tf_transformations
except ImportError:
    from tf2_ros import transformations as tf_transformations

from geometry_msgs.msg import (
    Point,
    Quaternion,
    TransformStamped,
    PoseWithCovarianceStamped,
    TwistWithCovarianceStamped,
)
from nav_msgs.msg import Odometry

class TransformFusionNode(Node):
    def __init__(self):
        super().__init__('transform_fusion')
        self.FREQ_PUB_LOCALIZATION = 50.0  # Hz

        self.lock = threading.Lock()
        self.cur_odom_to_baselink = None
        self.cur_map_to_odom = None

        # 구독자
        self.create_subscription(Odometry, '/Odometry', self.cb_save_cur_odom, 1)
        self.create_subscription(Odometry, '/map_to_odom', self.cb_save_map_to_odom, 1)

        # 퍼블리셔, 브로드캐스터
        self.pub_localization = self.create_publisher(Odometry, '/localization', 1)
        # Autoware 桥接：给 EKF 提供位姿与速度
        self.pub_pose_with_cov = self.create_publisher(
            PoseWithCovarianceStamped, '/localization/pose_estimator/pose_with_covariance', 10
        )
        self.pub_twist_with_cov = self.create_publisher(
            TwistWithCovarianceStamped, '/localization/twist_estimator/twist_with_covariance', 10
        )
        self.tf_broadcaster    = tf2_ros.TransformBroadcaster(self)

        # 주기 타이머
        period = 1.0 / self.FREQ_PUB_LOCALIZATION
        self.create_timer(period, self.timer_callback)
        self.get_logger().info('Transform Fusion Node Initialized')

    def cb_save_cur_odom(self, msg: Odometry):
        with self.lock:
            self.cur_odom_to_baselink = msg

    def cb_save_map_to_odom(self, msg: Odometry):
        with self.lock:
            self.cur_map_to_odom = msg

    def pose_to_mat(self, odom_msg: Odometry) -> np.ndarray:
        t = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        trans = tf_transformations.translation_matrix([t.x, t.y, t.z])
        rot   = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        return trans @ rot

    def timer_callback(self):
        with self.lock:
            odom    = copy.deepcopy(self.cur_odom_to_baselink)
            map2odom = copy.deepcopy(self.cur_map_to_odom)
        if odom is None:
            return

        # map -> odom
        T_map_to_odom = self.pose_to_mat(map2odom) if map2odom is not None else np.eye(4)
        trans = tf_transformations.translation_from_matrix(T_map_to_odom)
        quat  = tf_transformations.quaternion_from_matrix(T_map_to_odom)

        # tf 브로드캐스트
        t_msg = TransformStamped()
        t_msg.header.stamp = self.get_clock().now().to_msg()
        t_msg.header.frame_id    = 'map'
        t_msg.child_frame_id     = 'camera_init'
        t_msg.transform.translation.x = trans[0]
        t_msg.transform.translation.y = trans[1]
        t_msg.transform.translation.z = trans[2]
        t_msg.transform.rotation.x    = quat[0]
        t_msg.transform.rotation.y    = quat[1]
        t_msg.transform.rotation.z    = quat[2]
        t_msg.transform.rotation.w    = quat[3]
        self.tf_broadcaster.sendTransform(t_msg)

        # fused localization
        T_odom_to_base = self.pose_to_mat(odom)
        T_map_to_base  = T_map_to_odom @ T_odom_to_base
        xyz   = tf_transformations.translation_from_matrix(T_map_to_base)
        quat2 = tf_transformations.quaternion_from_matrix(T_map_to_base)

        loc_msg = Odometry()
        loc_msg.header.stamp          = odom.header.stamp
        loc_msg.header.frame_id       = 'map'
        loc_msg.child_frame_id        = 'body'
        loc_msg.pose.pose.position    = Point(x=xyz[0], y=xyz[1], z=xyz[2])
        loc_msg.pose.pose.orientation = Quaternion(
            x=quat2[0], y=quat2[1], z=quat2[2], w=quat2[3]
        )
        loc_msg.twist = odom.twist
        self.pub_localization.publish(loc_msg)

        # ─── Autoware 桥接发布 ──────────────────────────────────
        # 1) 位姿：/localization/pose_estimator/pose_with_covariance
        pose_cov_msg = PoseWithCovarianceStamped()
        pose_cov_msg.header.stamp = odom.header.stamp
        pose_cov_msg.header.frame_id = 'map'
        pose_cov_msg.pose.pose = loc_msg.pose.pose
        pose_cov = np.zeros(36)
        pose_cov[0] = pose_cov[7] = pose_cov[14] = 0.01     # 位置方差 (0.1 m)^2
        pose_cov[21] = pose_cov[28] = pose_cov[35] = 0.0025 # 姿态方差 (0.05 rad)^2
        pose_cov_msg.pose.covariance = pose_cov.tolist()
        self.pub_pose_with_cov.publish(pose_cov_msg)

        # 2) 速度：/localization/twist_estimator/twist_with_covariance
        twist_cov_msg = TwistWithCovarianceStamped()
        twist_cov_msg.header.stamp = odom.header.stamp
        twist_cov_msg.header.frame_id = 'base_link'
        twist_cov_msg.twist.twist = odom.twist.twist
        twist_cov = np.zeros(36)
        twist_cov[0] = twist_cov[7] = twist_cov[14] = 0.01     # 线速度方差
        twist_cov[21] = twist_cov[28] = twist_cov[35] = 0.0025 # 角速度方差
        twist_cov_msg.twist.covariance = twist_cov.tolist()
        self.pub_twist_with_cov.publish(twist_cov_msg)

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
