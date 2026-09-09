#!/usr/bin/env python3
"""ROS 2 adapter for the CPU prior-map exploration supervisor.

This node supplies only the stamped PointCloud2/TF, health, and Nav2 action
boundary.  Frontier construction remains in exploration_core.py.
"""
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from exploration_core import ExplorationCore, load_pinned_prior
from prior_map_assisted_exploration import ExplorationSupervisor, PointCloudAdapter
from shared_nav2_transport import NavigationActionTransport

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from soarm_interfaces.msg import LocalizationHealth
from tf2_ros import Buffer, TransformListener


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def transform_point(point, transform):
    x, y, z = point
    q = transform.rotation
    tx, ty, tz = transform.translation.x, transform.translation.y, transform.translation.z
    xx, yy, zz = q.x * q.x, q.y * q.y, q.z * q.z
    xy, xz, yz = q.x * q.y, q.x * q.z, q.y * q.z
    wx, wy, wz = q.w * q.x, q.w * q.y, q.w * q.z
    return (
        (1 - 2 * (yy + zz)) * x + 2 * (xy - wz) * y + 2 * (xz + wy) * z + tx,
        2 * (xy + wz) * x + (1 - 2 * (xx + zz)) * y + 2 * (yz - wx) * z + ty,
        2 * (xz - wy) * x + 2 * (yz + wx) * y + (1 - 2 * (xx + yy)) * z + tz,
    )


class Nav2Adapter:
    def __init__(self, node):
        self.node = node
        self.navigate_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self.path_client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        self.transport = NavigationActionTransport(self.navigate_client)
        self.path_handle_future = None
        self.path_result_future = None
        self.path_result = None
        self.goal_sequence = 0

    @property
    def busy(self):
        return self.transport.busy or self.path_handle_future is not None or self.path_result_future is not None

    @property
    def terminal(self):
        return self.transport.terminal and self.path_handle_future is None and self.path_result_future is None

    def _pose(self, goal):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = goal
        pose.pose.orientation.w = 1.0
        return pose

    def compute_path(self, goal):
        if self.path_result is not None:
            result, self.path_result = self.path_result, None
            return result
        if self.path_handle_future is None and self.path_result_future is None:
            request = ComputePathToPose.Goal()
            request.goal = self._pose(goal)
            self.path_handle_future = self.path_client.send_goal_async(request)
        return None

    def poll_path(self):
        if self.path_result is not None:
            result, self.path_result = self.path_result, None
            return result
        if self.path_handle_future is not None:
            if not self.path_handle_future.done():
                return None
            try:
                handle = self.path_handle_future.result()
            except Exception:
                self.path_handle_future = None
                return {'accepted': False}
            self.path_handle_future = None
            if handle is None or not handle.accepted:
                return {'accepted': False}
            self.path_result_future = handle.get_result_async()
        if self.path_result_future is None or not self.path_result_future.done():
            return None
        try:
            wrapped = self.path_result_future.result()
            path = wrapped.result.path
            result = {'accepted': True, 'status': 'SUCCEEDED' if wrapped.status == GoalStatus.STATUS_SUCCEEDED else 'FAILED', 'frame_id': path.header.frame_id, 'poses': path.poses}
        except Exception:
            result = {'accepted': False}
        self.path_result_future = None
        return result

    def safe(self, _goal):
        return True  # Nav2 costmaps remain the single runtime collision authority.

    def send(self, goal):
        self.goal_sequence += 1
        action = NavigateToPose.Goal()
        action.pose = self._pose(goal)
        self.transport.send(action)
        return 'exploration-%d' % self.goal_sequence

    def poll(self):
        if not self.transport.terminal:
            return None
        return 'SUCCEEDED' if self.transport.result == GoalStatus.STATUS_SUCCEEDED else 'FAILED'

    def cancel(self):
        self.transport.cancel()


class PriorMapExplorationNode(Node):
    def __init__(self):
        super().__init__('prior_map_assisted_exploration')
        self.declare_parameter('prior_manifest', str(Path(__file__).parents[1] / 'config/prior_map_assisted_exploration_input_lock.json'))
        self.declare_parameter('max_validated_rays', 1024)
        self.declare_parameter('expected_health_map_id', 'house_widened_reference.pcd')
        manifest = self.get_parameter('prior_manifest').value
        prior = load_pinned_prior(manifest)
        self.adapter = PointCloudAdapter(prior.map_identity, int(self.get_parameter('max_validated_rays').value))
        self.nav = Nav2Adapter(self)
        self.supervisor = ExplorationSupervisor(
            ExplorationCore(prior), self.nav,
            self.get_parameter('expected_health_map_id').value,
            source_map_id=prior.map_identity,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.revision = 0
        self.create_subscription(LocalizationHealth, '/localization/health', self.health, 10)
        self.create_subscription(PointCloud2, '/mid360/points', self.cloud, 10)
        self.create_timer(.05, self.tick)
        self.supervisor.start(time.monotonic())

    def health(self, message):
        self.supervisor.health('HEALTHY' if message.state == LocalizationHealth.HEALTHY else 'UNHEALTHY', Path(message.map_id).name, time.monotonic())

    def cloud(self, message):
        receipt = time.monotonic()
        stamp = seconds(message.header.stamp)
        try:
            if message.header.frame_id != 'livox_frame':
                raise RuntimeError('expected PointCloud2 frame livox_frame')
            cloud_tf = self.tf_buffer.lookup_transform('map', 'livox_frame', rclpy.time.Time.from_msg(message.header.stamp))
            base_tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time.from_msg(message.header.stamp))
            if seconds(cloud_tf.header.stamp) <= 0 or seconds(base_tf.header.stamp) <= 0:
                raise RuntimeError('unstamped transform')
            points = []
            for point in point_cloud2.read_points(message, field_names=('x', 'y', 'z'), skip_nans=True):
                if len(points) >= self.adapter.max_validated_rays:
                    break
                points.append(transform_point(point, cloud_tf.transform))
            self.revision += 1
            origin = cloud_tf.transform.translation
            base = base_tf.transform.translation
            record = self.adapter.record('mid360-%d' % self.revision, self.revision, stamp, (origin.x, origin.y, origin.z), (base.x, base.y, base.z), points, receipt, self.get_clock().now().nanoseconds / 1e9, True)
        except Exception as error:
            self.get_logger().warn('PointCloud2/TF rejected: %s' % error)
            return
        self.supervisor.enqueue(record)

    def tick(self):
        self.supervisor.tick(time.monotonic())


def main():
    rclpy.init()
    node = PriorMapExplorationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
