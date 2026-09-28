from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from soarm_interfaces.msg import LocalizationHealth
from tf2_ros import TransformBroadcaster

from .core import (CameraProfile, GuardedRGBDOdometry, RGBDFrame, RGBDInputAdapter,
                   VisualMap, VisualState, depth_to_points)


STATE_TO_HEALTH = {
    VisualState.INITIALIZING: LocalizationHealth.INITIALIZING,
    VisualState.TRACKING: LocalizationHealth.HEALTHY,
    VisualState.DEGRADED: LocalizationHealth.DEGRADED,
    VisualState.RELOCALIZING: LocalizationHealth.RELOCALIZING,
    VisualState.LOST: LocalizationHealth.LOST,
}


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def image_array(message: Image) -> np.ndarray:
    channels = {"mono8": 1, "8UC1": 1, "16UC1": 1, "32FC1": 1,
                "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}
    if message.encoding not in channels:
        raise ValueError(f"unsupported image encoding: {message.encoding}")
    count = channels[message.encoding]
    dtype = np.uint16 if message.encoding == "16UC1" else np.float32 if message.encoding == "32FC1" else np.uint8
    itemsize = np.dtype(dtype).itemsize
    row_values = message.step // itemsize
    raw = np.frombuffer(message.data, dtype=dtype)
    expected = message.height * row_values
    if raw.size < expected:
        raise ValueError("image data is shorter than declared step")
    array = raw[:expected].reshape(message.height, row_values)
    if count > 1:
        array = array[:, :message.width * count].reshape(message.height, message.width, count)
    else:
        array = array[:, :message.width]
    if message.encoding in ("bgr8", "bgra8"):
        array = array[..., ::-1]
    return array


def profile_from_info(info: CameraInfo, depth_scale: float, registration_verified: bool,
                      depth_min_m: float, depth_max_m: float) -> CameraProfile:
    fx, fy, cx, cy = float(info.k[0]), float(info.k[4]), float(info.k[2]), float(info.k[5])
    if info.width <= 0 or info.height <= 0 or not np.isfinite([fx, fy, cx, cy]).all() or fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera_info has invalid image dimensions or intrinsics")
    if not np.isfinite(depth_scale) or depth_scale <= 0.0 or depth_min_m <= 0.0 or depth_max_m <= depth_min_m:
        raise ValueError("invalid depth scale or range")
    return CameraProfile(info.width, info.height, fx, fy, cx, cy, depth_scale,
                         depth_min_m, depth_max_m, registration_verified)


def _set_if_present(message, field, value):
    if hasattr(message, field):
        setattr(message, field, value)


def health_reason(reason: str) -> int:
    if "timestamp" in reason:
        return LocalizationHealth.REASON_STALE_INPUT
    if "observability" in reason or "quality" in reason:
        return LocalizationHealth.REASON_QUALITY
    if "registration" in reason or "frame" in reason:
        return LocalizationHealth.REASON_INTERNAL
    if "valid_depth" in reason:
        return LocalizationHealth.REASON_DEGENERATE
    return LocalizationHealth.REASON_NONE


class RGBDPointCloud(Node):
    def __init__(self):
        super().__init__("rgbd_pointcloud")
        self.declare_parameter("depth_topic", "/camera/depth_image")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("points_topic", "/camera/depth/points")
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("depth_min_m", 0.15)
        self.declare_parameter("depth_max_m", 8.0)
        self.declare_parameter("frame_id", "camera_link")
        self.info = None
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.depth_min_m = float(self.get_parameter("depth_min_m").value)
        self.depth_max_m = float(self.get_parameter("depth_max_m").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter("points_topic").value), 10)
        self.create_subscription(CameraInfo, str(self.get_parameter("camera_info_topic").value),
                                 self.on_info, 10)
        self.create_subscription(Image, str(self.get_parameter("depth_topic").value),
                                 self.on_depth, 10)

    def on_info(self, message):
        self.info = message

    def on_depth(self, message):
        if self.info is None:
            return
        try:
            depth = image_array(message)
            profile = profile_from_info(self.info, self.depth_scale, True,
                                        self.depth_min_m, self.depth_max_m)
            points, _ = depth_to_points(depth, profile)
        except (ValueError, FloatingPointError) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)
            return
        fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                  for name, offset in (("x", 0), ("y", 4), ("z", 8))]
        cloud = PointCloud2()
        cloud.header = message.header
        cloud.header.frame_id = self.info.header.frame_id or self.frame_id
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = np.asarray(points, dtype=np.float32).tobytes()
        self.publisher.publish(cloud)


class RGBDTrackerNode(Node):
    def __init__(self, node_name: str, *, localization: bool = False):
        super().__init__(node_name)
        self.localization = localization
        self.declare_parameter("color_topic", "/camera/image")
        self.declare_parameter("depth_topic", "/camera/depth_image")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("odom_topic", "/visual/odometry")
        self.declare_parameter("health_topic", "/localization/health")
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("depth_min_m", 0.15)
        self.declare_parameter("depth_max_m", 8.0)
        self.declare_parameter("registration_verified", False)
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("odom_frame", "visual_odom")
        self.declare_parameter("map_id", "visual_map")
        self.declare_parameter("bundle_path", "")
        self.declare_parameter("keyframe_stride", 5)
        self.declare_parameter("min_inliers", 80)
        self.color = None
        self.info = None
        self.profile = None
        self.sequence = 0
        self.last_health = time.monotonic()
        self.map_id = str(self.get_parameter("map_id").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.bundle_path = str(self.get_parameter("bundle_path").value)
        self.tracker = None
        self.input_adapter = None
        self.visual_map = None
        self.last_keyframe = -1
        self.relocalization_streak = 0
        self.last_relocalized_keyframe = -1
        self.odom_pub = self.create_publisher(Odometry, str(self.get_parameter("odom_topic").value), 10)
        self.health_pub = self.create_publisher(LocalizationHealth, str(self.get_parameter("health_topic").value), 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(CameraInfo, str(self.get_parameter("camera_info_topic").value), self.on_info, 10)
        self.create_subscription(Image, str(self.get_parameter("color_topic").value), self.on_color, 10)
        self.create_subscription(Image, str(self.get_parameter("depth_topic").value), self.on_depth, 10)
        self.create_timer(0.5, self.publish_degraded_if_stale)
        if not localization:
            self.create_timer(2.0, self.save_bundle)
        elif self.bundle_path:
            self.visual_map, loaded_id = VisualMap.load_bundle(self.bundle_path)
            self.map_id = loaded_id

    def on_info(self, message):
        self.info = message
        if self.profile is None and message.width and message.height:
            try:
                self.profile = profile_from_info(
                    message, float(self.get_parameter("depth_scale").value),
                    bool(self.get_parameter("registration_verified").value),
                    float(self.get_parameter("depth_min_m").value),
                    float(self.get_parameter("depth_max_m").value))
            except ValueError as error:
                self.get_logger().warning(str(error), throttle_duration_sec=2.0)
                return
            self.tracker = GuardedRGBDOdometry(
                self.profile, min_inliers=int(self.get_parameter("min_inliers").value))
            self.input_adapter = RGBDInputAdapter(self.profile)
            if not self.localization:
                self.visual_map = VisualMap(self.profile)

    def on_color(self, message):
        self.color = message

    def on_depth(self, message):
        if self.profile is None or self.tracker is None or self.color is None:
            return
        if abs(stamp_seconds(message.header.stamp) - stamp_seconds(self.color.header.stamp)) > 0.15:
            self.publish_health(VisualState.DEGRADED, "rgb_depth_timestamp_mismatch", 0, float("inf"))
            return
        try:
            color = image_array(self.color)
            depth = image_array(message)
            frame = self.input_adapter.make_frame(
                color, depth, stamp_seconds(message.header.stamp),
                message.header.frame_id or self.frame_id,
                color_stamp=stamp_seconds(self.color.header.stamp),
                depth_stamp=stamp_seconds(message.header.stamp),
                camera_info_stamp=(stamp_seconds(self.info.header.stamp)
                                   if self.info is not None else None))
            result = self.tracker.update(frame)
        except ValueError as error:
            self.publish_health(VisualState.DEGRADED, str(error), 0, float("inf"))
            return
        self.sequence += 1
        if result.accepted and self.localization:
            points, _ = depth_to_points(depth, self.profile)
            if self.visual_map is None or not self.visual_map.keyframes:
                result.state = VisualState.LOST
                result.reason = "visual_map_unavailable"
            else:
                keyframe, relocalized_pose, relocalization_rmse = self.visual_map.relocalize(points)
                if keyframe is None or not np.isfinite(relocalization_rmse):
                    self.relocalization_streak = 0
                    self.last_relocalized_keyframe = -1
                    result.state = VisualState.RELOCALIZING
                    result.reason = "relocalization_geometry_rejected"
                else:
                    if keyframe.keyframe_id == self.last_relocalized_keyframe:
                        self.relocalization_streak += 1
                    else:
                        self.relocalization_streak = 1
                        self.last_relocalized_keyframe = keyframe.keyframe_id
                    result.transform = relocalized_pose
                    result.rmse_m = max(result.rmse_m, relocalization_rmse)
                    if self.relocalization_streak < 3:
                        result.state = VisualState.RELOCALIZING
                        result.reason = "relocalization_confirmation_pending"
        if result.accepted and not self.localization and self.visual_map is not None:
            points, _ = depth_to_points(depth, self.profile)
            if self.last_keyframe < 0 or self.sequence - self.last_keyframe >= int(self.get_parameter("keyframe_stride").value):
                self.visual_map.add(frame.stamp, result.transform, points)
                self.last_keyframe = self.sequence
        self.publish_odometry(result.transform, message.header.stamp, frame.frame_id)
        self.publish_health(result.state, result.reason, result.inliers, result.rmse_m)

    def publish_odometry(self, pose, stamp, child_frame):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = child_frame
        odom.pose.pose.position.x = float(pose[0, 3])
        odom.pose.pose.position.y = float(pose[1, 3])
        odom.pose.pose.position.z = float(pose[2, 3])
        rotation = pose[:3, :3]
        trace = np.trace(rotation)
        qw = float(np.sqrt(max(0.0, 1.0 + trace)) / 2.0)
        qx = float(np.copysign(np.sqrt(max(0.0, 1.0 + 2 * rotation[0, 0] - trace)) / 2.0, rotation[2, 1] - rotation[1, 2]))
        qy = float(np.copysign(np.sqrt(max(0.0, 1.0 + 2 * rotation[1, 1] - trace)) / 2.0, rotation[0, 2] - rotation[2, 0]))
        qz = float(np.copysign(np.sqrt(max(0.0, 1.0 + 2 * rotation[2, 2] - trace)) / 2.0, rotation[1, 0] - rotation[0, 1]))
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y = qx, qy
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        self.odom_pub.publish(odom)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = child_frame
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(transform)

    def publish_health(self, state, reason, inliers, rmse):
        message = LocalizationHealth()
        stamp = self.get_clock().now().to_msg()
        if hasattr(message, "stamp"):
            message.stamp = stamp
        elif hasattr(message, "header"):
            message.header.stamp = stamp
        state_value = STATE_TO_HEALTH[state] if isinstance(state, VisualState) else LocalizationHealth.DEGRADED
        _set_if_present(message, "sequence", self.sequence)
        _set_if_present(message, "state", state_value)
        _set_if_present(message, "reason_mask", health_reason(reason))
        _set_if_present(message, "consecutive_rejections", 0 if state_value == LocalizationHealth.HEALTHY else 1)
        _set_if_present(message, "fitness", float(inliers))
        _set_if_present(message, "rmse", float(rmse) if np.isfinite(rmse) else float("inf"))
        _set_if_present(message, "map_id", self.map_id)
        self.health_pub.publish(message)
        self.last_health = time.monotonic()

    def publish_degraded_if_stale(self):
        if time.monotonic() - self.last_health > 0.75:
            self.publish_health(VisualState.DEGRADED, "rgbd_input_stale", 0, float("inf"))

    def save_bundle(self):
        if self.visual_map is None or not self.visual_map.keyframes or not self.bundle_path:
            return
        self.visual_map.save_bundle(self.bundle_path, map_id=self.map_id)


def mapping_main(args=None):
    rclpy.init(args=args)
    node = RGBDTrackerNode("visual_mapper")
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.save_bundle()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def localization_main(args=None):
    rclpy.init(args=args)
    node = RGBDTrackerNode("visual_localizer", localization=True)
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def pointcloud_main(args=None):
    rclpy.init(args=args)
    node = RGBDPointCloud()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
