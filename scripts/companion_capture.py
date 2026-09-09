#!/usr/bin/env python3
"""Persist bounded, diagnostic-only RGB-D views with associated simulator observations."""
import json
import math
import struct
import subprocess
import sys
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
    "camera_optical_convention": "+Z forward, +X right, +Y down; camera_link to optical rotation is recorded convention, not inferred from frame_id",
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
    row_values = message.step // dtype.itemsize
    rows = np.frombuffer(bytes(message.data), dtype).reshape(message.height, row_values)
    return (rows[:, :message.width].astype(np.float32) * scale).copy()


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pose_record(transform, observed_clock_ns):
    return {
        "bridge_tf_header_stamp_ns": stamp_ns(transform), "observed_clock_ns": observed_clock_ns, "parent_frame": transform.header.frame_id,
        "child_frame": transform.child_frame_id,
        "translation_m": [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z],
        "quaternion_xyzw": [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w],
        "yaw_rad": yaw_from_quaternion(transform.transform.rotation),
    }


def gazebo_stamp_ns(message):
    header = message.get("header", {})
    stamp = header.get("stamp", {})
    if not isinstance(stamp, dict):
        return 0
    return int(stamp.get("sec", 0)) * 1_000_000_000 + int(stamp.get("nsec", stamp.get("nanosec", 0)))


def native_pose_sample(label):
    response = subprocess.run(
        ["ign", "topic", "-e", "-t", "/world/default/dynamic_pose/info", "-n", "1", "--json-output"],
        text=True, capture_output=True, timeout=15)
    if response.returncode:
        raise AssertionError(f"native Pose_V sample failed for {label}: {response.stderr}")
    decoder = json.JSONDecoder()
    remaining = response.stdout.lstrip()
    envelopes = []
    while remaining:
        envelope, consumed = decoder.raw_decode(remaining)
        envelopes.append(envelope)
        remaining = remaining[consumed:].lstrip()
    envelope = envelopes[-1]
    source_stamp = gazebo_stamp_ns(envelope)
    entity = next((pose for pose in envelope.get("pose", []) if pose.get("name") == "kuruma"), None)
    if not source_stamp or entity is None:
        raise AssertionError(f"native Pose_V lacks source stamp or exact kuruma entity for {label}")
    return {"label": label, "source_stamp_ns": source_stamp, "entity": entity, "envelope": envelope, "all_envelopes_returned": envelopes}


def entity_stationarity(before, after):
    start, end = before["entity"], after["entity"]
    first = start.get("position", {}); second = end.get("position", {})
    translation_delta = math.sqrt(sum((float(second.get(axis, 0)) - float(first.get(axis, 0))) ** 2 for axis in ("x", "y", "z")))
    q1, q2 = start.get("orientation", {}), end.get("orientation", {})
    dot = abs(sum(float(q1.get(axis, 0)) * float(q2.get(axis, 0)) for axis in ("x", "y", "z", "w")))
    angle = 2 * math.acos(min(1.0, max(-1.0, dot)))
    return {"source_interval_ns": [before["source_stamp_ns"], after["source_stamp_ns"]], "translation_delta_m": translation_delta, "rotation_delta_rad": angle, "stationary": translation_delta <= .02 and angle <= .02}


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
    state = {"clock_ns": 0, "rgb": None, "depth": None, "info": None, "target_ns": None,
             "stale": {"rgb": 0, "depth": 0}, "poses": []}
    rclpy.init()
    node = rclpy.create_node("companion_home_pilot_capture")

    def receive_image(key, message):
        if state["target_ns"] is None:
            return
        if stamp_ns(message) <= state["target_ns"]:
            state["stale"][key] += 1
        else:
            state[key] = message

    def receive_tf(message):
        for transform in message.transforms:
            if "kuruma" in transform.child_frame_id:
                state["poses"].append(pose_record(transform, state["clock_ns"]))
        state["poses"] = state["poses"][-4000:]

    node.create_subscription(Clock, "/clock", lambda msg: state.__setitem__("clock_ns", msg.clock.sec * 1_000_000_000 + msg.clock.nanosec), 50)
    node.create_subscription(Image, "/camera/image", lambda msg: receive_image("rgb", msg), 20)
    node.create_subscription(Image, "/camera/depth_image", lambda msg: receive_image("depth", msg), 20)
    node.create_subscription(CameraInfo, "/camera/camera_info", lambda msg: state.__setitem__("info", msg), 20)
    node.create_subscription(TFMessage, "/world/default/dynamic_pose/info", receive_tf, 50)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)

    def receive_after_pose():
        state.update(target_ns=None, rgb=None, depth=None, info=None, stale={"rgb": 0, "depth": 0})
        spin(0.5)
        fence = state["clock_ns"]
        if fence <= 0:
            raise AssertionError("no /clock observed after pose acknowledgement")
        state["target_ns"] = fence + 500_000_000
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            rgb, depth, info = state["rgb"], state["depth"], state["info"]
            if rgb is not None and depth is not None and info is not None:
                delta = abs(stamp_ns(rgb) - stamp_ns(depth))
                if delta <= 50_000_000 and stamp_ns(info) > state["target_ns"]:
                    return rgb, depth, info, fence, delta
                state["rgb"] = state["depth"] = None
        raise AssertionError(f"no fresh synchronized RGB-D after fence {fence}: stale={state['stale']}")

    # Establish subscriptions before the first diagnostic placement; topic discovery alone is insufficient.
    startup_deadline = time.monotonic() + 15
    while state["clock_ns"] <= 0 and time.monotonic() < startup_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if state["clock_ns"] <= 0:
        raise AssertionError("no /clock received within bounded startup wait")

    records = []
    native_samples = []
    for point in viewpoints:
        x, y, z, _, _, yaw = point["pose"]
        request = ("name: \"kuruma\" position { x: %s y: %s z: %s } orientation { x: 0 y: 0 z: %s w: %s }" % (x, y, z, math.sin(yaw / 2), math.cos(yaw / 2)))
        response = subprocess.run(["ign", "service", "-s", "/world/default/set_pose", "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean", "--timeout", "3000", "--req", request], text=True, capture_output=True)
        if response.returncode or "data: true" not in response.stdout:
            raise AssertionError(response.stdout + response.stderr)
        # Do not use the early post-set_pose pose as a stationarity endpoint.
        spin(1.0)
        pre_source = native_pose_sample(point["id"] + "_before_frame")
        rgb, depth, info, fence, sync_delta = receive_after_pose()
        post_source = native_pose_sample(point["id"] + "_after_frame")
        native_samples.extend([pre_source, post_source])
        pixels, meters = image_array(rgb), depth_array(depth)
        if info.width != rgb.width or info.height != rgb.height or meters.shape != (rgb.height, rgb.width):
            raise AssertionError("RGB, depth, and CameraInfo dimensions differ")
        valid = np.isfinite(meters) & (meters > 0)
        if valid.mean() <= .2 or info.k[0] <= 0 or info.k[4] <= 0:
            raise AssertionError(f"invalid metric depth or calibration at {point['id']}")
        image_stamp = stamp_ns(rgb)
        stationarity = entity_stationarity(pre_source, post_source)
        if not (pre_source["source_stamp_ns"] <= image_stamp <= post_source["source_stamp_ns"]):
            raise AssertionError(f"native Pose_V source stamps do not bracket image for {point['id']}")
        if not stationarity["stationary"]:
            raise AssertionError(f"kuruma did not remain stationary around {point['id']}: {stationarity}")
        eligible = [pose for pose in state["poses"] if abs(pose["observed_clock_ns"] - image_stamp) <= 100_000_000]
        if not eligible:
            (output / "raw_dynamic_pose_stream.failure.json").write_text(json.dumps(state["poses"], indent=2) + "\n")
            raise AssertionError(f"no timestamped dynamic_pose observation within 100ms of {point['id']} image; samples={len(state['poses'])}, image_stamp={image_stamp}")
        observed = min(eligible, key=lambda pose: abs(pose["observed_clock_ns"] - image_stamp))
        frame = output / "frames" / point["id"]
        frame.mkdir()
        png(frame / "rgb.png", pixels)
        np.save(frame / "depth_m_float32.npy", meters)
        np.save(frame / "depth_invalid_mask.npy", ~valid)
        preview = np.zeros_like(meters, dtype=np.uint8)
        finite = meters[valid]
        preview[valid] = np.clip((finite.max() - meters[valid]) / max(finite.max() - finite.min(), .001) * 255, 0, 255).astype(np.uint8)
        png(frame / "depth_preview.png", preview[:, :, None])
        calibration = {"width": info.width, "height": info.height, "k": list(info.k), "p": list(info.p), "d": list(info.d), "r": list(info.r), "distortion_model": info.distortion_model,
                       "header_stamp_ns": stamp_ns(info), "frame_id": info.header.frame_id, "rgb_encoding": rgb.encoding, "rgb_step": rgb.step, "rgb_is_bigendian": bool(rgb.is_bigendian), "depth_encoding": depth.encoding, "depth_step": depth.step, "depth_is_bigendian": bool(depth.is_bigendian), "depth_unit": "m"}
        (frame / "camera_info.json").write_text(json.dumps(calibration, indent=2) + "\n")
        record = {"id": point["id"], "diagnostic_teleport": True, "requested_pose_world": point["pose"], "observed_model_pose_bridge": observed,
                  "native_pose_before_frame": {"source_stamp_ns": pre_source["source_stamp_ns"], "entity": pre_source["entity"]}, "native_pose_after_frame": {"source_stamp_ns": post_source["source_stamp_ns"], "entity": post_source["entity"]}, "native_pose_stationarity": stationarity,
                  "bridge_receipt_clock_delta_ns": abs(observed["observed_clock_ns"] - image_stamp), "bridge_tf_header_stamp_ns": observed["bridge_tf_header_stamp_ns"], "post_pose_clock_fence_ns": fence, "post_pose_accept_after_ns": state["target_ns"], "rgb_stamp_ns": image_stamp, "depth_stamp_ns": stamp_ns(depth), "camera_info_stamp_ns": stamp_ns(info), "rgb_depth_sync_delta_ns": sync_delta, "stale_messages_discarded": state["stale"], "frames": [rgb.header.frame_id, depth.header.frame_id, info.header.frame_id], "depth_finite_positive_ratio": float(valid.mean()), "depth_min_m": float(finite.min()), "depth_max_m": float(finite.max())}
        records.append(record)
    (output / "raw_bridge_dynamic_pose_stream.json").write_text(json.dumps(state["poses"], indent=2) + "\n")
    (output / "raw_native_dynamic_pose_samples.json").write_text(json.dumps(native_samples, indent=2) + "\n")
    report = {"capture_run_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"), "consumer_seam": "Gazebo rgbd_camera -> ros_gz_bridge -> rclpy subscriber -> metric depth/calibration/native Pose_V persistence", "transport_isolation": {"docker_network": "none", "ros_localhost_only": True, "ign_ip": "127.0.0.1"}, "camera_chain": CAMERA_CHAIN, "viewpoints": records, "navigation_claim": "NONE; diagnostic teleports only.", "observed_pose_source": "Exact kuruma Pose_V native envelopes bracket each image; TFMessage bridge receipt clocks are retained separately and not treated as source acquisition timestamps."}
    (output / "capture.json").write_text(json.dumps(report, indent=2) + "\n")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
