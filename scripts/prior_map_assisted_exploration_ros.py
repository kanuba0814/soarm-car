#!/usr/bin/env python3
"""ROS 2 adapter for the CPU prior-map exploration supervisor.

This node supplies only the stamped PointCloud2/TF, health, and Nav2 action
boundary.  Frontier construction remains in exploration_core.py.
"""
import json
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from exploration_core import ExplorationCore, load_pinned_prior
from prior_map_assisted_exploration import ExplorationSupervisor, PointCloudAdapter, State
from shared_nav2_transport import NavigationActionTransport

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from soarm_interfaces.msg import LocalizationHealth
from std_srvs.srv import Trigger
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
        self.path_handle = None
        self.path_result_future = None
        self.path_result = None
        self.path_cancel_requested = False
        self.nav_started = False
        self.goal_sequence = 0

    @property
    def busy(self):
        return self.transport.busy or self.path_handle_future is not None or self.path_handle is not None or self.path_result_future is not None

    @property
    def terminal(self):
        return (self.path_handle_future is None and self.path_handle is None and self.path_result_future is None and (not self.nav_started or self.transport.terminal))

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
            self.path_cancel_requested = False
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
            self.path_handle = handle
            if self.path_cancel_requested:
                handle.cancel_goal_async()
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
        self.path_handle = None
        return result

    def safe(self, _goal):
        return True  # Nav2 costmaps remain the single runtime collision authority.

    def send(self, goal):
        self.nav_started = True
        self.goal_sequence += 1
        action = NavigateToPose.Goal()
        action.pose = self._pose(goal)
        self.transport.send(action)
        return 'exploration-%d' % self.goal_sequence

    def send_prepared(self, goal):
        self.nav_started = True
        self.goal_sequence += 1
        action = NavigateToPose.Goal()
        action.pose = PoseStamped()
        action.pose.header.frame_id = goal['frame_id']
        action.pose.header.stamp = self.node.get_clock().now().to_msg()
        action.pose.pose.position.x = goal['position']['x']
        action.pose.pose.position.y = goal['position']['y']
        action.pose.pose.position.z = goal['position']['z']
        action.pose.pose.orientation.x = goal['orientation']['x']
        action.pose.pose.orientation.y = goal['orientation']['y']
        action.pose.pose.orientation.z = goal['orientation']['z']
        action.pose.pose.orientation.w = goal['orientation']['w']
        self.transport.send(action)
        return 'named-%d' % self.goal_sequence

    def poll(self):
        if not self.transport.terminal:
            return None
        return 'SUCCEEDED' if self.transport.result == GoalStatus.STATUS_SUCCEEDED else 'FAILED'

    def cancel(self):
        self.path_cancel_requested = True
        if self.path_handle is not None:
            self.path_handle.cancel_goal_async()
        self.transport.cancel()

    def poll_cancel(self):
        if self.path_handle_future is not None or self.path_result_future is not None:
            self.poll_path()
        return self.terminal


class PriorMapExplorationNode(Node):
    def __init__(self):
        super().__init__('prior_map_assisted_exploration')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.declare_parameter('prior_manifest', str(Path(__file__).parents[1] / 'config/prior_map_assisted_exploration_input_lock.json'))
        self.declare_parameter('max_validated_rays', 1024)
        self.declare_parameter('ground_z', 0.0)
        self.declare_parameter('floor_tolerance', .10)
        self.declare_parameter('navigation_z_min', -.30)
        self.declare_parameter('navigation_z_max', .50)
        self.declare_parameter('evidence_dir', '')
        self.declare_parameter('named_request', '')
        self.declare_parameter('asset_root', '/tmp/soarm-intel-nav-restore-20260909')
        self.declare_parameter('expected_health_map_id', 'house_widened_reference.pcd')
        manifest = self.get_parameter('prior_manifest').value
        prior = load_pinned_prior(manifest)
        self.adapter = PointCloudAdapter(
            prior.map_identity, int(self.get_parameter('max_validated_rays').value),
            float(self.get_parameter('ground_z').value), float(self.get_parameter('floor_tolerance').value),
            float(self.get_parameter('navigation_z_min').value), float(self.get_parameter('navigation_z_max').value),
        )
        self.nav = Nav2Adapter(self)
        self.supervisor = ExplorationSupervisor(
            ExplorationCore(prior, navigation_z_min=self.adapter.navigation_z_min, navigation_z_max=self.adapter.navigation_z_max), self.nav,
            self.get_parameter('expected_health_map_id').value,
            source_map_id=prior.map_identity,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.revision = 0
        self.create_subscription(LocalizationHealth, '/localization/health', self.health, 10)
        self.create_subscription(PointCloud2, '/mid360/points', self.cloud, 10)
        self.create_subscription(Twist, '/safe_cmd_vel', self.safe_command, 10)
        self.create_subscription(Odometry, '/odom', self.measured_pose, 10)
        evidence_dir = self.get_parameter('evidence_dir').value
        self.evidence_dir = Path(evidence_dir) if evidence_dir else None
        self.stop_samples, self.pose_samples = [], []
        self.named_status = {'state': 'not_requested'}
        self.paused_checkpoint = None
        self.drop_source = False
        self.last_checkpoint, self.last_report, self.last_state = 0.0, 0.0, None
        if self.evidence_dir:
            self.evidence_dir.mkdir(parents=True, exist_ok=False)
            (self.evidence_dir / 'metadata.json').write_text(json.dumps({'map_identity': prior.map_identity, 'health_map_id': self.get_parameter('expected_health_map_id').value, 'use_sim_time': True, 'wallclock_watchdog': 'time.monotonic', 'ground_z': self.adapter.ground_z, 'floor_tolerance': self.adapter.floor_tolerance, 'navigation_z_band': [self.adapter.navigation_z_min, self.adapter.navigation_z_max], 'point_sample_policy': 'uniform_index_across_full_message', 'max_validated_rays': self.adapter.max_validated_rays}, indent=2) + '\n')
        self.create_service(Trigger, '~/pause', self.pause)
        self.create_service(Trigger, '~/resume', self.resume)
        self.create_service(Trigger, '~/stop', self.stop)
        self.create_service(Trigger, '~/inject_source_drop', self.inject_source_drop)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.supervisor.start(time.monotonic())

    def health(self, message):
        self.supervisor.health('HEALTHY' if message.state == LocalizationHealth.HEALTHY else 'UNHEALTHY', message.map_id, time.monotonic())

    def cloud(self, message):
        if self.drop_source:
            return
        receipt = time.monotonic()
        stamp = seconds(message.header.stamp)
        try:
            if message.header.frame_id != 'livox_frame':
                raise RuntimeError('expected PointCloud2 frame livox_frame')
            cloud_tf = self.tf_buffer.lookup_transform('map', 'livox_frame', rclpy.time.Time.from_msg(message.header.stamp))
            base_tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time.from_msg(message.header.stamp))
            if seconds(cloud_tf.header.stamp) <= 0 or seconds(base_tf.header.stamp) <= 0:
                raise RuntimeError('unstamped transform')
            raw_points = list(point_cloud2.read_points(message, field_names=('x', 'y', 'z'), skip_nans=True))
            stride = max(1, math.ceil(len(raw_points) / self.adapter.max_validated_rays))
            sampled = raw_points[::stride][:self.adapter.max_validated_rays]
            points = [transform_point(point, cloud_tf.transform) for point in sampled]
            self.revision += 1
            origin = cloud_tf.transform.translation
            base = base_tf.transform.translation
            record = self.adapter.record('mid360-%d' % self.revision, self.revision, stamp, (origin.x, origin.y, origin.z), (base.x, base.y, base.z), points, receipt, self.get_clock().now().nanoseconds / 1e9, True, 'uniform_index_across_full_message:%d/%d' % (len(sampled), len(raw_points)))
        except Exception as error:
            self.get_logger().warn('PointCloud2/TF rejected: %s' % error)
            return
        self.supervisor.enqueue(record)

    def pause(self, _request, response):
        self.paused_checkpoint = self.supervisor.pause(time.monotonic())
        response.success = self.paused_checkpoint is not None
        response.message = 'cancellation requested; resume requires confirmed cancellation'
        return response

    def resume(self, _request, response):
        try:
            self.supervisor.resume(self.paused_checkpoint, time.monotonic())
            response.success, response.message = True, 'fresh source and a new plan are required'
        except Exception as error:
            response.success, response.message = False, str(error)
        return response

    def stop(self, _request, response):
        self.supervisor.cancel(time.monotonic())
        response.success, response.message = True, 'cancellation requested'
        return response

    def inject_source_drop(self, _request, response):
        self.drop_source = not self.drop_source
        response.success, response.message = True, 'source drop=' + str(self.drop_source)
        return response

    def safe_command(self, message):
        self.stop_samples.append({'receipt': time.monotonic(), 'linear_x': message.linear.x, 'linear_y': message.linear.y, 'angular_z': message.angular.z})
        del self.stop_samples[:-200]

    def measured_pose(self, message):
        self.pose_samples.append({'receipt': time.monotonic(), 'frame_id': message.header.frame_id, 'x': message.pose.pose.position.x, 'y': message.pose.pose.position.y, 'linear_x': message.twist.twist.linear.x, 'linear_y': message.twist.twist.linear.y, 'angular_z': message.twist.twist.angular.z})
        del self.pose_samples[:-200]

    def _dispatch_named_task(self):
        request = self.get_parameter('named_request').value
        if not request or self.named_status['state'] != 'not_requested':
            return
        package_root = Path(__file__).parents[1] / 'ros2_ws' / 'src' / 'soarm_named_place_request'
        sys.path.insert(0, str(package_root))
        from soarm_named_place_request.prepare import prepare, verify_prepared
        asset_root = Path(self.get_parameter('asset_root').value)
        prepared = prepare(Path(request), asset_root)
        verified = verify_prepared(prepared, asset_root)
        if prepared['require_heading'] is not False:
            raise RuntimeError('named task must remain position-only')
        if not self.supervisor.gate.ready(time.monotonic()) or self.nav.busy or self.supervisor.state not in (State.FRONTIERS_EXHAUSTED, State.BUDGET_EXHAUSTED):
            return
        self.named_status = {'state': 'active', 'place_id': prepared['place_id'], 'map_goal': verified, 'goal_uuid': self.nav.send_prepared(verified)}

    def tick(self):
        self.supervisor.tick(time.monotonic())
        if self.supervisor.state in (State.FRONTIERS_EXHAUSTED, State.BUDGET_EXHAUSTED):
            self._dispatch_named_task()
        if self.named_status['state'] == 'active':
            result = self.nav.poll()
            if result is not None:
                self.named_status['state'] = 'succeeded' if result == 'SUCCEEDED' else 'failed'
        if not self.evidence_dir:
            return
        for filename, values in (('events.jsonl', self.supervisor.drain_events()), ('observations.jsonl', self.supervisor.drain_consumed())):
            if values:
                with (self.evidence_dir / filename).open('a') as stream:
                    for value in values:
                        stream.write(json.dumps(value, sort_keys=True) + '\n')
        wall_now = time.monotonic()
        state_changed = self.last_state != self.supervisor.state
        if wall_now - self.last_checkpoint >= 5.0 or state_changed:
            checkpoint = self.evidence_dir / 'checkpoint.json'
            temporary = checkpoint.with_suffix('.tmp')
            temporary.write_text(json.dumps(self.supervisor.core.export_state(), sort_keys=True) + '\n')
            os.replace(temporary, checkpoint)
            self.last_checkpoint = wall_now
        if wall_now - self.last_report >= 1.0 or state_changed:
            report = {'state': self.supervisor.state.value, 'reason': self.supervisor.reason, 'core_state_checkpoint': 'checkpoint.json', 'goals': self.supervisor.goals, 'named_task': self.named_status, 'safe_cmd_vel': self.stop_samples, 'measured_pose': self.pose_samples, 'cadence': {'fsm_hz': 20, 'progress_report_hz_max': 1, 'checkpoint_seconds': 5}}
            temporary = self.evidence_dir / 'report.json.tmp'
            temporary.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
            os.replace(temporary, self.evidence_dir / 'report.json')
            self.last_report, self.last_state = wall_now, self.supervisor.state


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
