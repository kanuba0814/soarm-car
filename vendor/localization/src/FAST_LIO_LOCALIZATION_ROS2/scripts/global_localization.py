#!/usr/bin/env python3
# coding=utf-8

import copy
import json
import threading
import time
import numpy as np
import open3d as o3d

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
import tf_transformations

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from soarm_interfaces.msg import LocalizationHealth

class InputFreshness:
    """Tracks strictly advancing source stamps against captured ROS clock and wall time."""

    def __init__(self, timeout):
        if not np.isfinite(timeout) or timeout <= 0.0:
            raise ValueError('input freshness timeout must be positive')
        self.timeout = float(timeout)
        self.source_ns = {'scan': None, 'odom': None}
        self.receipt = {'scan': None, 'odom': None}
        self.attempt = {'scan': None, 'odom': None}

    @staticmethod
    def valid_stamp(value):
        return isinstance(value, int) and not isinstance(value, bool) and value > 0

    def update(self, name, stamp_ns, node_clock_ns, now):
        attempt = {
            'source_stamp_ns': stamp_ns,
            'node_clock_ns': node_clock_ns,
            'accepted': False,
            'reason': None,
        }
        if not self.valid_stamp(stamp_ns):
            attempt['reason'] = 'invalid_source_stamp'
        elif not self.valid_stamp(node_clock_ns):
            attempt['reason'] = 'invalid_node_clock'
        elif stamp_ns > node_clock_ns:
            attempt['reason'] = 'future'
        else:
            previous = self.source_ns[name]
            if previous is not None and stamp_ns == previous:
                attempt['reason'] = 'duplicate'
            elif previous is not None and stamp_ns < previous:
                self.receipt[name] = None
                attempt['reason'] = 'rollback'
            else:
                self.source_ns[name] = stamp_ns
                self.receipt[name] = now
                attempt['accepted'] = True
                attempt['reason'] = 'accepted'
        self.attempt[name] = attempt
        return attempt['accepted']

    def fresh(self, now):
        return all(receipt is not None and 0.0 <= now - receipt <= self.timeout
                   for receipt in self.receipt.values())

    def diagnostics(self, now):
        return {name: {'accepted_source_ns': self.source_ns[name],
                       'last_receipt_monotonic': self.receipt[name],
                       'age_seconds': None if self.receipt[name] is None else now - self.receipt[name],
                       'last_attempt': self.attempt[name]}
                for name in self.receipt}


from localization_health import (
    DEGRADED,
    HEALTHY,
    LOST,
    REASON_INTERNAL,
    REASON_QUALITY,
    REASON_STALE_INPUT,
    LocalizationHealthTracker,
)


class GlobalLocalizationNode(Node):
    def __init__(self):
        super().__init__('fast_lio_localization')

        # ─── Parameters ───────────────────────────────────────────
        self.declare_parameter('map_voxel_size', 0.1)
        self.declare_parameter('scan_voxel_size', 0.1)
        self.declare_parameter('freq_localization', 0.5)      # Hz
        self.declare_parameter('localization_th', 0.9)
        self.declare_parameter('localization_rmse_th', 1.0e9)
        self.declare_parameter('max_correspondence_distance', 1.0)
        self.declare_parameter('max_correction_translation', 1.0e9)
        self.declare_parameter('max_correction_rotation', 1.0e9)
        self.declare_parameter('planar_correction_gate', False)
        self.declare_parameter('max_correction_z', 1.0e9)
        self.declare_parameter('max_correction_roll_pitch', 1.0e9)
        self.declare_parameter('use_coarse_initialization', True)
        self.declare_parameter('coarse_scale', 5.0)
        self.declare_parameter('fov', 2 * np.pi)
        self.declare_parameter('fov_far', 100.0)
        self.declare_parameter('base_to_body_xyz', [0.0, 0.0, 0.232])
        self.declare_parameter('base_to_body_rpy', [0.0, 0.0, 0.0])
        self.declare_parameter('max_consecutive_rejections', 5)
        self.declare_parameter('quality_rejection_dead_reckoning', False)
        self.declare_parameter('max_unverified_translation', 1.0)
        self.declare_parameter('max_unverified_rotation', 0.5)
        self.declare_parameter('max_unverified_seconds', 20.0)
        self.declare_parameter('health_heartbeat_rate', 10.0)
        self.declare_parameter('map_id', '')
        self.declare_parameter('input_freshness_timeout', 0.5)
        self.declare_parameter('health_diagnostics_trace', False)

        self.map_voxel_size    = self.get_parameter('map_voxel_size').value
        self.scan_voxel_size   = self.get_parameter('scan_voxel_size').value
        self.freq_localization = self.get_parameter('freq_localization').value
        self.localization_th   = self.get_parameter('localization_th').value
        self.localization_rmse_th = self.get_parameter('localization_rmse_th').value
        self.max_correspondence_distance = self.get_parameter('max_correspondence_distance').value
        self.max_correction_translation = self.get_parameter('max_correction_translation').value
        self.max_correction_rotation = self.get_parameter('max_correction_rotation').value
        self.planar_correction_gate = self.get_parameter('planar_correction_gate').value
        self.max_correction_z = self.get_parameter('max_correction_z').value
        self.max_correction_roll_pitch = self.get_parameter('max_correction_roll_pitch').value
        self.use_coarse_initialization = self.get_parameter('use_coarse_initialization').value
        self.coarse_scale = self.get_parameter('coarse_scale').value
        self.FOV               = self.get_parameter('fov').value
        self.FOV_FAR           = self.get_parameter('fov_far').value
        self.base_to_body = (tf_transformations.translation_matrix(self.get_parameter('base_to_body_xyz').value) @
                             tf_transformations.euler_matrix(*self.get_parameter('base_to_body_rpy').value))
        self.health_tracker = LocalizationHealthTracker(
            self.get_parameter('max_consecutive_rejections').value)
        self.quality_rejection_dead_reckoning = self.get_parameter(
            'quality_rejection_dead_reckoning').value
        self.max_unverified_translation = self.get_parameter(
            'max_unverified_translation').value
        self.max_unverified_rotation = self.get_parameter(
            'max_unverified_rotation').value
        self.max_unverified_seconds = self.get_parameter(
            'max_unverified_seconds').value
        if self.quality_rejection_dead_reckoning:
            budgets = (self.max_unverified_translation,
                       self.max_unverified_rotation,
                       self.max_unverified_seconds)
            if not all(np.isfinite(value) and value > 0.0 for value in budgets):
                raise ValueError('dead-reckoning budgets must be finite and positive')
        self.health_heartbeat_rate = self.get_parameter('health_heartbeat_rate').value
        if self.health_heartbeat_rate <= 0.0:
            raise ValueError('health_heartbeat_rate must be positive')
        self.map_id = self.get_parameter('map_id').value
        self.input_freshness = InputFreshness(
            self.get_parameter('input_freshness_timeout').value)
        self.health_diagnostics_trace = bool(
            self.get_parameter('health_diagnostics_trace').value)

        # ─── State Variables ─────────────────────────────────────
        self.global_map    = None
        self.initialized   = False
        self.T_map_to_odom = np.eye(4)
        self.cur_odom      = None
        self.cur_scan      = None
        self.localization_timer = None
        self.last_verified_odom = None
        self.last_verified_stamp_ns = 0

        # ─── Publishers ──────────────────────────────────────────
        self.pub_pc_in_map   = self.create_publisher(PointCloud2, '/cur_scan_in_map', 1)
        self.pub_submap      = self.create_publisher(PointCloud2, '/submap', 1)
        self.pub_map_to_odom = self.create_publisher(Odometry,     '/map_to_odom', 1)
        self.pub_health = self.create_publisher(
            LocalizationHealth, '/localization/health', 1)

        # ─── Subscriptions ───────────────────────────────────────
        self.create_subscription(PointCloud2,                  '/cloud_registered', self.cb_save_cur_scan, 1)
        self.create_subscription(Odometry,                    '/Odometry',         self.cb_save_cur_odom,  1)
        self._map_sub  = self.create_subscription(PointCloud2, '/global_map',               self.cb_init_map,      1)
        self._init_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.cb_init_pose,
            1
        )

        self.create_timer(1.0 / self.health_heartbeat_rate, self.publish_health,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('GlobalLocalizationNode initialized.')

    def pc2_to_array(self, pc_msg: PointCloud2) -> np.ndarray:
        """PointCloud2 → (N×3) NumPy array"""
        pts = []
        for x, y, z in pc2.read_points(pc_msg, field_names=('x','y','z'), skip_nans=True):
            pts.append((x, y, z))
        return np.array(pts, dtype=np.float32)

    def cb_init_map(self, msg: PointCloud2):
        pts = self.pc2_to_array(msg)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        self.global_map = self.voxel_down_sample(pcd, self.map_voxel_size)
        self.get_logger().info('Global map received and downsampled.')
        self.destroy_subscription(self._map_sub)

    def cb_init_pose(self, msg: PoseWithCovarianceStamped):
        if msg.header.frame_id != 'map':
            self.get_logger().warn(
                f'Initial pose must be in map, got {msg.header.frame_id!r}.')
            return
        if self.global_map is None:
            self.get_logger().warn('Waiting for global map before initial localization.')
            return
        if self.health_tracker.state in (DEGRADED, LOST):
            self.initialized = False
            self.health_tracker.reinitialize()
        elif self.initialized:
            return
        if self.cur_scan is None:
            self.get_logger().warn('Waiting for first scan before initial localization.')
            return

        if self.cur_odom is None:
            self.get_logger().warn('Waiting for FAST-LIO odometry before initial localization.')
            return
        # G=T_map_base from /initialpose, O=T_cameraInit_body from FAST-LIO.
        # ICP estimates M=T_map_cameraInit, so M_initial=G*A*inv(O).
        initial = self.pose_to_mat(msg) @ self.base_to_body @ np.linalg.inv(self.pose_to_mat(self.cur_odom))
        success = self.global_localization(initial)
        if success:
            self.initialized = True
            if self.localization_timer is None:
                period = 1.0 / self.freq_localization
                self.localization_timer = self.create_timer(
                    period, self.timer_callback)
            self.get_logger().info('Initial global localization succeeded.')

    def cb_save_cur_odom(self, msg: Odometry):
        stamp_ns = self.stamp_to_nanoseconds(msg.header.stamp)
        node_clock_ns = self.get_clock().now().nanoseconds
        if self.input_freshness.update('odom', stamp_ns, node_clock_ns, time.monotonic()):
            self.cur_odom = msg

    def cb_save_cur_scan(self, msg: PointCloud2):
        source_stamp_ns = self.stamp_to_nanoseconds(msg.header.stamp)
        node_clock_ns = self.get_clock().now().nanoseconds
        if not self.input_freshness.update('scan', source_stamp_ns, node_clock_ns, time.monotonic()):
            return
        msg.header.frame_id = 'camera_init'
        msg.header.stamp    = self.get_clock().now().to_msg()
        self.pub_pc_in_map.publish(msg)

        pts = self.pc2_to_array(msg)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        self.cur_scan = pcd

    def timer_callback(self):
        if self.initialized:
            self.global_localization(self.T_map_to_odom)

    @staticmethod
    def stamp_to_nanoseconds(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def publish_health(self):
        message = LocalizationHealth()
        message.stamp = self.get_clock().now().to_msg()
        message.sequence = self.health_tracker.sequence
        inputs_fresh = self.input_freshness.fresh(time.monotonic())
        message.state = self.health_tracker.state if inputs_fresh else DEGRADED
        message.reason_mask = (self.health_tracker.reason_mask if inputs_fresh else
                               self.health_tracker.reason_mask | REASON_STALE_INPUT)
        if self.health_diagnostics_trace:
            now_wall = time.monotonic()
            self.get_logger().info('health_trace ' + json.dumps({
                'input_freshness': self.input_freshness.diagnostics(now_wall),
                'node_clock_ns': self.get_clock().now().nanoseconds,
                'publish_wall_monotonic': now_wall,
                'state': message.state,
                'reason_mask': message.reason_mask,
            }, sort_keys=True))
        accepted_ns = self.health_tracker.last_accepted_stamp_ns
        message.last_accepted_correction_stamp.sec = accepted_ns // 1_000_000_000
        message.last_accepted_correction_stamp.nanosec = accepted_ns % 1_000_000_000
        message.consecutive_rejections = self.health_tracker.consecutive_rejections
        message.fitness = self.health_tracker.last_fitness
        message.rmse = self.health_tracker.last_rmse
        message.map_id = self.map_id
        self.pub_health.publish(message)

    def record_rejection(self, reason_mask):
        previous_state = self.health_tracker.state
        state = self.health_tracker.rejected(reason_mask)
        if state == DEGRADED and previous_state != DEGRADED:
            self.initialized = False
            self.get_logger().error(
                'Localization DEGRADED after consecutive registration failures; '
                'map-frame output must stop until explicit reinitialization.')
        self.publish_health()

    def dead_reckoning_budget_available(self):
        if (not self.quality_rejection_dead_reckoning or
                self.last_verified_odom is None or self.cur_odom is None or
                self.last_verified_stamp_ns <= 0):
            return False
        current_stamp_ns = self.stamp_to_nanoseconds(self.cur_odom.header.stamp)
        if current_stamp_ns < self.last_verified_stamp_ns:
            return False
        age_seconds = (current_stamp_ns - self.last_verified_stamp_ns) / 1.0e9
        try:
            current_odom = self.pose_to_mat(self.cur_odom)
            delta = np.linalg.inv(self.last_verified_odom) @ current_odom
            _, _, yaw = tf_transformations.euler_from_matrix(delta)
        except (ValueError, np.linalg.LinAlgError):
            return False
        translation = np.linalg.norm(delta[:2, 3])
        return bool(
            np.isfinite(delta).all() and np.isfinite(yaw) and
            translation < self.max_unverified_translation and
            abs(yaw) < self.max_unverified_rotation and
            age_seconds < self.max_unverified_seconds)

    def record_unverified_quality(self, fitness, rmse):
        within_budget = self.dead_reckoning_budget_available()
        previous_state = self.health_tracker.state
        remains_healthy = self.health_tracker.quality_unverified(
            within_budget, fitness, rmse)
        if not remains_healthy and previous_state == HEALTHY:
            self.initialized = False
            self.get_logger().error(
                'Localization DEGRADED after exhausting the bounded '
                'dead-reckoning budget; explicit reinitialization is required.')
        self.publish_health()
        return remains_healthy

    def global_localization(self, pose_est):
        self.get_logger().info('Performing global localization via ICP...')
        if self.cur_scan is None or self.cur_odom is None:
            self.record_rejection(REASON_STALE_INPUT)
            return False
        scan_copy = copy.deepcopy(self.cur_scan)

        submap = self.crop_global_map_in_FOV(scan_copy, pose_est, self.cur_odom)

        if len(submap.points) == 0:
            self.get_logger().warn('ICP submap is empty.')
            self.record_rejection(REASON_QUALITY)
            return False
        T, fitness, rmse = self.registration_candidate(scan_copy, submap, pose_est)
        self.get_logger().info(
            f'ICP fitness: {fitness:.3f}, inlier RMSE: {rmse:.3f} m')

        accepted, rejection_reason = self.registration_is_acceptable(T, fitness, rmse)
        if accepted:
            stamp_ns = self.stamp_to_nanoseconds(self.cur_odom.header.stamp)
            if stamp_ns <= 0:
                stamp_ns = self.get_clock().now().nanoseconds
            if not self.health_tracker.accepted(stamp_ns, fitness, rmse):
                self.get_logger().warn(
                    'Registration candidate ignored while localization is not healthy; '
                    'publish a new initial pose to reinitialize.')
                self.publish_health()
                return False
            self.T_map_to_odom = T
            self.last_verified_odom = self.pose_to_mat(self.cur_odom)
            self.last_verified_stamp_ns = stamp_ns
            odom = Odometry()
            xyz  = tf_transformations.translation_from_matrix(T)
            quat = tf_transformations.quaternion_from_matrix(T)
            # 올바른 Odometry 메시지 필드 설정
            odom.pose.pose.position    = Point(x=xyz[0], y=xyz[1], z=xyz[2])
            odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            odom.header.stamp          = self.cur_odom.header.stamp
            odom.header.frame_id       = 'map'
            self.pub_map_to_odom.publish(odom)
            self.publish_health()
            return True

        if (self.initialized and rejection_reason.startswith('fitness=') and
                self.quality_rejection_dead_reckoning):
            remains_healthy = self.record_unverified_quality(fitness, rmse)
            suffix = ('; continuing within bounded dead reckoning'
                      if remains_healthy else '; dead-reckoning budget exhausted')
            self.get_logger().warn(
                f'Global localization unverified ({rejection_reason}){suffix}.')
            return False

        reason_mask = (REASON_INTERNAL if rejection_reason.startswith('non-finite')
                       else REASON_QUALITY)
        self.record_rejection(reason_mask)
        self.get_logger().warn(f'Global localization rejected ({rejection_reason}).')
        return False

    def registration_candidate(self, scan, submap, pose_est):
        T = pose_est
        # A coarse search is useful only for the first, uncertain pose guess.
        # Re-running it during tracking lets repeated walls pull a valid prior
        # into a different room before the fine pass can refine it.
        if not self.initialized and self.use_coarse_initialization:
            T, _, _ = self.registration_at_scale(
                scan, submap, initial=T, scale=self.coarse_scale)
        return self.registration_at_scale(scan, submap, initial=T, scale=1.0)

    def registration_is_acceptable(self, candidate, fitness, rmse):
        if (not np.isfinite(candidate).all() or
                not np.isfinite(fitness) or not np.isfinite(rmse)):
            return False, 'non-finite registration result'
        # Low overlap may use the bounded dead-reckoning path, but poor
        # residuals and implausible correction jumps are hard failures. Check
        # them first so a low fitness score cannot mask either condition.
        if rmse >= self.localization_rmse_th:
            return False, f'RMSE={rmse:.3f} m'
        if not self.initialized:
            if fitness <= self.localization_th:
                return False, f'fitness={fitness:.3f}'
            return True, ''

        correction_delta = np.linalg.inv(self.T_map_to_odom) @ candidate
        if getattr(self, 'planar_correction_gate', False):
            translation_xy = np.linalg.norm(correction_delta[:2, 3])
            translation_z = abs(correction_delta[2, 3])
            roll, pitch, yaw = tf_transformations.euler_from_matrix(correction_delta)
            roll_pitch = max(abs(roll), abs(pitch))
            yaw = abs(yaw)
            if translation_xy >= self.max_correction_translation:
                return False, f'correction XY jump={translation_xy:.3f} m'
            if translation_z >= self.max_correction_z:
                return False, f'correction Z jump={translation_z:.3f} m'
            if roll_pitch >= self.max_correction_roll_pitch:
                return False, f'correction roll/pitch jump={roll_pitch:.3f} rad'
            if yaw >= self.max_correction_rotation:
                return False, f'correction yaw jump={yaw:.3f} rad'
            if fitness <= self.localization_th:
                return False, f'fitness={fitness:.3f}'
            return True, ''

        translation_delta = np.linalg.norm(correction_delta[:3, 3])
        cosine = np.clip(
            (np.trace(correction_delta[:3, :3]) - 1.0) / 2.0,
            -1.0,
            1.0)
        rotation_delta = np.arccos(cosine)
        if translation_delta >= self.max_correction_translation:
            return False, f'correction jump={translation_delta:.3f} m'
        if rotation_delta >= self.max_correction_rotation:
            return False, f'correction jump={rotation_delta:.3f} rad'
        if fitness <= self.localization_th:
            return False, f'fitness={fitness:.3f}'
        return True, ''

    def crop_global_map_in_FOV(self, scan, pose_est, odom):
        # /cloud_registered is already expressed in camera_init. pose_est is
        # T_map_cameraInit, so applying the current body odometry again would
        # move the FOV twice as the robot travels.
        T_map2scan = np.linalg.inv(pose_est)

        pts = np.asarray(self.global_map.points)
        hom = np.hstack([pts, np.ones((pts.shape[0],1))])
        pts_scan = (T_map2scan @ hom.T).T

        if self.FOV >= 2*np.pi:
            mask = np.linalg.norm(pts_scan[:, :3], axis=1) < self.FOV_FAR
        else:
            ang = np.arctan2(pts_scan[:,1], pts_scan[:,0])
            radius = np.linalg.norm(pts_scan[:, :2], axis=1)
            mask = (radius < self.FOV_FAR) & (np.abs(ang) < self.FOV/2)

        subpts = pts[mask]
        submap = o3d.geometry.PointCloud()
        submap.points = o3d.utility.Vector3dVector(subpts)

        header = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        cloud = pc2.create_cloud_xyz32(header, subpts[::10].tolist())
        self.pub_submap.publish(cloud)

        return submap

    def registration_at_scale(self, scan, submap, initial, scale):
        def down(p): return p.voxel_down_sample(self.scan_voxel_size * scale)
        reg = o3d.pipelines.registration.registration_icp(
            down(scan), down(submap),
            max_correspondence_distance=self.max_correspondence_distance * scale,
            init=initial,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
        )
        return reg.transformation, reg.fitness, reg.inlier_rmse

    @staticmethod
    def pose_to_mat(pose_stamped):
        t = pose_stamped.pose.pose.position
        q = pose_stamped.pose.pose.orientation
        return tf_transformations.translation_matrix([t.x,t.y,t.z]) \
             @ tf_transformations.quaternion_matrix([q.x,q.y,q.z,q.w])

    @staticmethod
    def voxel_down_sample(pcd, vs):
        try:
            return pcd.voxel_down_sample(vs)
        except:
            return o3d.geometry.voxel_down_sample(pcd, vs)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
