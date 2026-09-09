#!/usr/bin/env python3
"""Persist bounded diagnostic RGB-D views with source-time native Pose_V interpolation."""
import json
import math
import os
import struct
import subprocess
import sys
import threading
import time
import zlib
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

CAMERA_CHAIN = {
    "model_to_base_link_m": [0.0, 0.0, 0.0763],
    "base_link_to_camera_link_m": [0.180, 0.0, 0.150],
    "camera_optical_convention": "+Z forward, +X right, +Y down; recorded SDF convention, not residual-tuned",
}
POSE_THRESHOLDS = {
    "max_source_bracket_gap_ns": 50_000_000,
    "stability_half_window_ns": 250_000_000,
    "max_source_gap_in_stability_window_ns": 50_000_000,
    "max_translation_from_interpolated_pose_m": 0.010,
    "max_rotation_from_interpolated_pose_rad": 0.010,
    "minimum_stability_samples": 4,
}


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def png(path, array):
    height, width, channels = array.shape
    color = 2 if channels == 3 else 0
    raw = b"".join(b"\0" + row.tobytes() for row in array)
    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xffffffff)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, color, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def image_array(message):
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(message.encoding)
    if channels is None:
        raise AssertionError(f"unsupported RGB encoding {message.encoding}")
    rows = np.frombuffer(bytes(message.data), np.uint8).reshape(message.height, message.step)
    pixels = rows[:, :message.width * channels].reshape(message.height, message.width, channels)
    if message.encoding.startswith("bgr"):
        pixels = pixels[:, :, [2, 1, 0] + ([3] if channels == 4 else [])]
    return pixels[:, :, :3].copy()


def depth_array(message):
    formats = {"32FC1": ("f4", 1.0), "16UC1": ("u2", 0.001)}
    if message.encoding not in formats:
        raise AssertionError(f"unsupported depth encoding {message.encoding}")
    code, scale = formats[message.encoding]
    dtype = np.dtype((">" if message.is_bigendian else "<") + code)
    rows = np.frombuffer(bytes(message.data), dtype).reshape(message.height, message.step // dtype.itemsize)
    return (rows[:, :message.width].astype(np.float32) * scale).copy()


def gazebo_stamp_ns(envelope):
    stamp = envelope.get("header", {}).get("stamp", {})
    return int(stamp.get("sec", 0)) * 1_000_000_000 + int(stamp.get("nsec", stamp.get("nanosec", 0))) if isinstance(stamp, dict) else 0


def entity_vector(entity):
    position, orientation = entity["position"], entity["orientation"]
    return np.array([position[axis] for axis in ("x", "y", "z")], dtype=np.float64), np.array([orientation[axis] for axis in ("x", "y", "z", "w")], dtype=np.float64)


def quaternion_angle(first, second):
    first, second = first / np.linalg.norm(first), second / np.linalg.norm(second)
    return 2 * math.acos(min(1.0, max(-1.0, abs(float(np.dot(first, second))))))


def slerp(first, second, fraction):
    first, second = first / np.linalg.norm(first), second / np.linalg.norm(second)
    dot = float(np.dot(first, second))
    if dot < 0:
        second, dot = -second, -dot
    if dot > .9995:
        return (first + fraction * (second - first)) / np.linalg.norm(first + fraction * (second - first))
    theta = math.acos(min(1.0, max(-1.0, dot)))
    return (math.sin((1 - fraction) * theta) * first + math.sin(fraction * theta) * second) / math.sin(theta)


class NativePoseStream:
    def __init__(self):
        self.records = []
        self.errors = []
        self.lock = threading.Lock()
        self.process = subprocess.Popen(
            ["stdbuf", "-oL", "ign", "topic", "-e", "-t", "/world/default/dynamic_pose/info", "--json-output"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        for line in self.process.stdout:
            try:
                envelope = json.loads(line)
                source_stamp_ns = gazebo_stamp_ns(envelope)
                entity = next((pose for pose in envelope.get("pose", []) if pose.get("name") == "kuruma"), None)
                if not source_stamp_ns or entity is None:
                    self.errors.append({"reason": "missing_source_stamp_or_exact_kuruma", "envelope": envelope})
                    continue
                with self.lock:
                    self.records.append({"source_stamp_ns": source_stamp_ns, "entity": entity, "envelope": envelope})
                    self.records = self.records[-12000:]
            except json.JSONDecodeError as error:
                self.errors.append({"reason": "json_decode", "detail": str(error), "raw_line": line})

    def snapshot(self):
        with self.lock:
            return list(self.records)

    def select_at_image(self, image_stamp_ns):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            records = self.snapshot()
            before = [item for item in records if item["source_stamp_ns"] <= image_stamp_ns]
            after = [item for item in records if item["source_stamp_ns"] >= image_stamp_ns]
            if before and after:
                lower, upper = before[-1], after[0]
                if upper["source_stamp_ns"] > image_stamp_ns:
                    break
                if lower["source_stamp_ns"] == image_stamp_ns:
                    upper = lower
                    break
            time.sleep(.02)
        else:
            raise AssertionError(f"native Pose_V stream did not bracket image source time {image_stamp_ns}")
        gap = upper["source_stamp_ns"] - lower["source_stamp_ns"]
        if gap > POSE_THRESHOLDS["max_source_bracket_gap_ns"]:
            raise AssertionError(f"native Pose_V bracket gap {gap} exceeds threshold")
        lower_position, lower_quaternion = entity_vector(lower["entity"])
        upper_position, upper_quaternion = entity_vector(upper["entity"])
        fraction = 0.0 if gap == 0 else (image_stamp_ns - lower["source_stamp_ns"]) / gap
        position = lower_position + fraction * (upper_position - lower_position)
        quaternion = slerp(lower_quaternion, upper_quaternion, fraction)
        selected = {"source_stamp_ns": image_stamp_ns, "interpolation": {"lower_source_stamp_ns": lower["source_stamp_ns"], "upper_source_stamp_ns": upper["source_stamp_ns"], "bracket_gap_ns": gap, "fraction": fraction}, "entity": {"name": "kuruma", "position": dict(zip(("x", "y", "z"), map(float, position))), "orientation": dict(zip(("x", "y", "z", "w"), map(float, quaternion)))}}
        half_window = POSE_THRESHOLDS["stability_half_window_ns"]
        window_deadline = time.monotonic() + 5
        while time.monotonic() < window_deadline:
            records = sorted(self.snapshot(), key=lambda item: item["source_stamp_ns"])
            lower_bound, upper_bound = image_stamp_ns - half_window, image_stamp_ns + half_window
            lower = [item for item in records if item["source_stamp_ns"] <= lower_bound]
            upper = [item for item in records if item["source_stamp_ns"] >= upper_bound]
            if lower and upper:
                window = [lower[-1]] + [item for item in records if lower_bound < item["source_stamp_ns"] < upper_bound] + [upper[0]]
                break
            time.sleep(.02)
        else:
            raise AssertionError(f"native Pose_V stream lacks complete stability window at {image_stamp_ns}")
        timestamps = [item["source_stamp_ns"] for item in window]
        max_gap = max(right - left for left, right in zip(timestamps, timestamps[1:])) if len(timestamps) > 1 else 0
        selected_position, selected_quaternion = entity_vector(selected["entity"])
        displacements, rotations = [], []
        for item in window:
            position_now, quaternion_now = entity_vector(item["entity"])
            displacements.append(float(np.linalg.norm(position_now - selected_position)))
            rotations.append(quaternion_angle(quaternion_now, selected_quaternion))
        stability = {"source_window_ns": [image_stamp_ns - half_window, image_stamp_ns + half_window], "sample_count": len(window), "max_source_gap_ns": max_gap, "max_translation_from_interpolated_pose_m": max(displacements), "max_rotation_from_interpolated_pose_rad": max(rotations), "continuous": len(window) >= POSE_THRESHOLDS["minimum_stability_samples"] and max_gap <= POSE_THRESHOLDS["max_source_gap_in_stability_window_ns"] and max(displacements) <= POSE_THRESHOLDS["max_translation_from_interpolated_pose_m"] and max(rotations) <= POSE_THRESHOLDS["max_rotation_from_interpolated_pose_rad"]}
        if not stability["continuous"]:
            raise AssertionError(f"native Pose_V stability window rejected: {stability}")
        return selected, stability

    def close(self):
        self.process.terminate()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
        self.thread.join(timeout=3)


def main():
    output, lock_path = map(Path, sys.argv[1:])
    capture_config = json.loads(lock_path.read_text(encoding="utf-8"))
    viewpoints = capture_config.get("development_viewpoints", capture_config.get("diagnostic_viewpoints", []))
    if not 3 <= len(viewpoints) <= 12:
        raise SystemExit("pilot requires between 3 and 12 viewpoints")
    if (output / "frames").exists() or (output / "capture.json").exists():
        raise SystemExit(f"Refusing to overwrite capture artifacts in {output}")
    output.mkdir(parents=True, exist_ok=True)
    (output / "frames").mkdir()
    evaluator_capture = os.environ.get("COMPANION_EVALUATOR_CAPTURE") == "1"
    state = {"clock_ns": 0, "rgb": None, "depth": None, "info": None, "labels": None, "colored": None, "target_ns": None, "stale": {"rgb": 0, "depth": 0, "labels": 0, "colored": 0}, "poses": []}
    rclpy.init()
    node = rclpy.create_node("companion_home_pilot_capture")
    pose_stream = NativePoseStream()

    def receive_image(key, message):
        if state["target_ns"] is None:
            return
        if stamp_ns(message) <= state["target_ns"]:
            state["stale"][key] += 1
        else:
            state[key] = message

    def receive_tf(message):
        for transform in message.transforms:
            if transform.child_frame_id == "kuruma":
                state["poses"].append({"bridge_tf_header_stamp_ns": stamp_ns(transform), "observed_clock_ns": state["clock_ns"], "parent_frame": transform.header.frame_id, "child_frame": transform.child_frame_id})
        state["poses"] = state["poses"][-4000:]

    node.create_subscription(Clock, "/clock", lambda msg: state.__setitem__("clock_ns", msg.clock.sec * 1_000_000_000 + msg.clock.nanosec), 50)
    node.create_subscription(Image, "/camera/image", lambda msg: receive_image("rgb", msg), 20)
    node.create_subscription(Image, "/camera/depth_image", lambda msg: receive_image("depth", msg), 20)
    node.create_subscription(CameraInfo, "/camera/camera_info", lambda msg: state.__setitem__("info", msg), 20)
    if evaluator_capture:
        node.create_subscription(Image, "/eval_segmentation/labels_map", lambda msg: receive_image("labels", msg), 20)
        node.create_subscription(Image, "/eval_segmentation/colored_map", lambda msg: receive_image("colored", msg), 20)
    node.create_subscription(TFMessage, "/world/default/dynamic_pose/info", receive_tf, 50)

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)

    def receive_after_pose():
        state.update(target_ns=None, rgb=None, depth=None, info=None, labels=None, colored=None, stale={"rgb": 0, "depth": 0, "labels": 0, "colored": 0})
        spin(.5)
        fence = state["clock_ns"]
        if fence <= 0:
            raise AssertionError("no /clock observed after pose acknowledgement")
        state["target_ns"] = fence + 500_000_000
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            rgb, depth, info = state["rgb"], state["depth"], state["info"]
            labels, colored = state["labels"], state["colored"]
            required = (rgb, depth, info) if not evaluator_capture else (rgb, depth, info, labels, colored)
            if all(message is not None for message in required):
                stamps = [stamp_ns(message) for message in required if hasattr(message, "header")]
                if max(stamps) - min(stamps) <= 50_000_000 and stamp_ns(info) > state["target_ns"]:
                    return rgb, depth, info, labels, colored, fence
                state["rgb"] = state["depth"] = state["labels"] = state["colored"] = None
        raise AssertionError(f"no fresh synchronized RGB-D after fence {fence}: stale={state['stale']}")

    try:
        startup_deadline = time.monotonic() + 15
        while (state["clock_ns"] <= 0 or not pose_stream.snapshot()) and time.monotonic() < startup_deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if state["clock_ns"] <= 0 or not pose_stream.snapshot():
            raise AssertionError("clock or persistent native Pose_V stream unavailable at startup")
        records = []
        for point in viewpoints:
            x, y, z, _, _, yaw = point["pose"]
            request = ("name: \"kuruma\" position { x: %s y: %s z: %s } orientation { x: 0 y: 0 z: %s w: %s }" % (x, y, z, math.sin(yaw / 2), math.cos(yaw / 2)))
            response = subprocess.run(["ign", "service", "-s", "/world/default/set_pose", "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean", "--timeout", "3000", "--req", request], text=True, capture_output=True)
            if response.returncode or "data: true" not in response.stdout:
                raise AssertionError(response.stdout + response.stderr)
            spin(1.0)
            rgb, depth, info, labels, colored, fence = receive_after_pose()
            image_stamp = stamp_ns(rgb)
            selected_native_pose, stability = pose_stream.select_at_image(image_stamp)
            pixels, meters = image_array(rgb), depth_array(depth)
            valid = np.isfinite(meters) & (meters > 0)
            if info.width != rgb.width or info.height != rgb.height or meters.shape != (rgb.height, rgb.width) or valid.mean() <= .2 or info.k[0] <= 0 or info.k[4] <= 0:
                raise AssertionError(f"invalid RGB-D/calibration at {point['id']}")
            bridge = min(state["poses"], key=lambda pose: abs(pose["observed_clock_ns"] - image_stamp)) if state["poses"] else None
            frame = output / "frames" / point["id"]
            frame.mkdir()
            png(frame / "rgb.png", pixels)
            np.save(frame / "depth_m_float32.npy", meters)
            np.save(frame / "depth_invalid_mask.npy", ~valid)
            preview = np.zeros_like(meters, dtype=np.uint8); finite = meters[valid]
            preview[valid] = np.clip((finite.max() - meters[valid]) / max(finite.max() - finite.min(), .001) * 255, 0, 255).astype(np.uint8)
            png(frame / "depth_preview.png", preview[:, :, None])
            calibration = {"width": info.width, "height": info.height, "k": list(info.k), "p": list(info.p), "d": list(info.d), "r": list(info.r), "distortion_model": info.distortion_model, "header_stamp_ns": stamp_ns(info), "frame_id": info.header.frame_id, "rgb_encoding": rgb.encoding, "rgb_step": rgb.step, "rgb_is_bigendian": bool(rgb.is_bigendian), "depth_encoding": depth.encoding, "depth_step": depth.step, "depth_is_bigendian": bool(depth.is_bigendian), "depth_unit": "m"}
            (frame / "camera_info.json").write_text(json.dumps(calibration, indent=2) + "\n")
            evaluator_record = None
            if evaluator_capture:
                labels_meta = {"width": labels.width, "height": labels.height, "encoding": labels.encoding, "step": labels.step, "is_bigendian": bool(labels.is_bigendian), "stamp_ns": stamp_ns(labels), "frame_id": labels.header.frame_id}
                (frame / "labels_map.raw").write_bytes(bytes(labels.data))
                (frame / "labels_map.json").write_text(json.dumps(labels_meta, indent=2) + "\n")
                colored_pixels = image_array(colored)
                png(frame / "labels_colored.png", colored_pixels)
                evaluator_record = {"labels_map": labels_meta, "colored_map": {"encoding": colored.encoding, "step": colored.step, "stamp_ns": stamp_ns(colored), "frame_id": colored.header.frame_id}, "rgb_label_delta_ns": abs(stamp_ns(rgb) - stamp_ns(labels)), "rgb_colored_delta_ns": abs(stamp_ns(rgb) - stamp_ns(colored))}
            records.append({"id": point["id"], "diagnostic_teleport": True, "evaluator_private": evaluator_record, "requested_pose_world": point["pose"], "native_pose_at_image": selected_native_pose, "native_pose_stability": stability, "bridge_pose_observation": bridge, "post_pose_clock_fence_ns": fence, "post_pose_accept_after_ns": state["target_ns"], "rgb_stamp_ns": image_stamp, "depth_stamp_ns": stamp_ns(depth), "camera_info_stamp_ns": stamp_ns(info), "rgb_depth_sync_delta_ns": abs(stamp_ns(rgb) - stamp_ns(depth)), "stale_messages_discarded": state["stale"], "frames": [rgb.header.frame_id, depth.header.frame_id, info.header.frame_id], "depth_finite_positive_ratio": float(valid.mean()), "depth_min_m": float(finite.min()), "depth_max_m": float(finite.max())})
        (output / "raw_native_dynamic_pose_stream.json").write_text(json.dumps({"records": pose_stream.snapshot(), "parse_errors": pose_stream.errors}, indent=2) + "\n")
        (output / "raw_bridge_dynamic_pose_stream.json").write_text(json.dumps(state["poses"], indent=2) + "\n")
        report = {"capture_run_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"), "consumer_seam": "Gazebo rgbd_camera -> bridge -> rclpy persistence -> source-time native Pose_V interpolation", "transport_isolation": {"docker_network": "none", "ros_localhost_only": True, "ign_ip": "127.0.0.1"}, "camera_chain": CAMERA_CHAIN, "pose_thresholds": POSE_THRESHOLDS, "viewpoints": records, "navigation_claim": "NONE; diagnostic teleports only.", "observed_pose_source": "Persistent native Pose_V source stream with strict source-time interpolation and continuous stability-window checks; bridge receipt clocks are diagnostic only."}
        (output / "capture.json").write_text(json.dumps(report, indent=2) + "\n")
    finally:
        (output / "raw_native_dynamic_pose_stream.json").write_text(json.dumps({"records": pose_stream.snapshot(), "parse_errors": pose_stream.errors}, indent=2) + "\n")
        (output / "raw_bridge_dynamic_pose_stream.json").write_text(json.dumps(state["poses"], indent=2) + "\n")
        pose_stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
