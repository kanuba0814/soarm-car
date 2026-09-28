#!/usr/bin/env python3
import argparse
import bisect
import json
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from PIL import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


COLOR_TOPIC = "/camera/color/image_raw"
DEPTH_TOPIC = "/camera/depth/image_raw"
COLOR_INFO_TOPIC = "/camera/color/camera_info"
DEPTH_INFO_TOPIC = "/camera/depth/camera_info"


def open_reader(bag, information):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag), storage_id=information["storage_identifier"]
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    return reader


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def image_array(message):
    if message.encoding in ("rgb8", "bgr8"):
        array = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.step
        )[:, : message.width * 3]
        array = array.reshape(message.height, message.width, 3)
        return array[:, :, ::-1] if message.encoding == "bgr8" else array
    if message.encoding in ("16UC1", "mono16"):
        return np.frombuffer(message.data, dtype=np.uint16).reshape(
            message.height, message.step // 2
        )[:, : message.width]
    raise RuntimeError(f"Unsupported encoding: {message.encoding}")


def camera_info_dict(message):
    return {
        "frame_id": message.header.frame_id,
        "width": message.width,
        "height": message.height,
        "distortion_model": message.distortion_model,
        "d": list(message.d),
        "k": list(message.k),
        "r": list(message.r),
        "p": list(message.p),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--count", type=int, default=213)
    parser.add_argument("--max-sync-ms", type=float, default=20.0)
    args = parser.parse_args()

    information = yaml.safe_load((args.bag / "metadata.yaml").read_text())[
        "rosbag2_bagfile_information"
    ]
    reader = open_reader(args.bag, information)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_types = {name: get_message(type_name) for name, type_name in topic_types.items()}

    timestamps = {COLOR_TOPIC: [], DEPTH_TOPIC: []}
    camera_info = {}
    transforms = {}
    topic_indexes = {COLOR_TOPIC: 0, DEPTH_TOPIC: 0}
    while reader.has_next():
        topic, serialized, record_timestamp = reader.read_next()
        if topic in timestamps:
            message = deserialize_message(serialized, message_types[topic])
            timestamps[topic].append(
                {
                    "header_ns": stamp_ns(message),
                    "record_ns": record_timestamp,
                    "topic_index": topic_indexes[topic],
                }
            )
            topic_indexes[topic] += 1
        elif topic in (COLOR_INFO_TOPIC, DEPTH_INFO_TOPIC) and topic not in camera_info:
            camera_info[topic] = camera_info_dict(
                deserialize_message(serialized, message_types[topic])
            )
        elif topic in ("/tf", "/tf_static"):
            message = deserialize_message(serialized, message_types[topic])
            for transform in message.transforms:
                key = f"{transform.header.frame_id}->{transform.child_frame_id}"
                value = transform.transform
                transforms[key] = {
                    "translation": [value.translation.x, value.translation.y, value.translation.z],
                    "rotation_xyzw": [
                        value.rotation.x,
                        value.rotation.y,
                        value.rotation.z,
                        value.rotation.w,
                    ],
                    "source_topic": topic,
                }

    color_messages = timestamps[COLOR_TOPIC]
    depth_messages = timestamps[DEPTH_TOPIC]
    if not color_messages or not depth_messages:
        raise RuntimeError("Bag does not contain color and depth images")
    if args.count < 2 or args.count > len(color_messages):
        raise RuntimeError(f"--count must be between 2 and {len(color_messages)}")

    color_indexes = [
        round(index * (len(color_messages) - 1) / (args.count - 1))
        for index in range(args.count)
    ]
    depth_header_times = [item["header_ns"] for item in depth_messages]
    selected_pairs = []
    for color_index in color_indexes:
        color_item = color_messages[color_index]
        insertion = bisect.bisect_left(depth_header_times, color_item["header_ns"])
        candidates = [index for index in (insertion - 1, insertion) if 0 <= index < len(depth_messages)]
        depth_index = min(
            candidates,
            key=lambda index: abs(depth_messages[index]["header_ns"] - color_item["header_ns"]),
        )
        depth_item = depth_messages[depth_index]
        delta_ms = abs(depth_item["header_ns"] - color_item["header_ns"]) / 1_000_000
        selected_pairs.append(
            {
                "color_topic_index": color_index,
                "depth_topic_index": depth_index,
                "color_header_ns": color_item["header_ns"],
                "depth_header_ns": depth_item["header_ns"],
                "color_record_ns": color_item["record_ns"],
                "depth_record_ns": depth_item["record_ns"],
                "sync_delta_ms": delta_ms,
                "sync_ok": delta_ms <= args.max_sync_ms,
            }
        )

    color_output = args.output / "rgb"
    depth_output = args.output / "depth"
    color_output.mkdir(parents=True, exist_ok=True)
    depth_output.mkdir(parents=True, exist_ok=True)
    color_targets = {
        pair["color_topic_index"]: output_index for output_index, pair in enumerate(selected_pairs)
    }
    depth_targets = {}
    for output_index, pair in enumerate(selected_pairs):
        depth_targets.setdefault(pair["depth_topic_index"], []).append(output_index)

    reader = open_reader(args.bag, information)
    topic_indexes = {COLOR_TOPIC: 0, DEPTH_TOPIC: 0}
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic not in topic_indexes:
            continue
        topic_index = topic_indexes[topic]
        topic_indexes[topic] += 1
        output_indexes = []
        if topic == COLOR_TOPIC and topic_index in color_targets:
            output_indexes = [color_targets[topic_index]]
        elif topic == DEPTH_TOPIC and topic_index in depth_targets:
            output_indexes = depth_targets[topic_index]
        if not output_indexes:
            continue
        message = deserialize_message(serialized, message_types[topic])
        array = image_array(message)
        for output_index in output_indexes:
            destination = (
                color_output if topic == COLOR_TOPIC else depth_output
            ) / f"{output_index:06d}.png"
            if array.dtype == np.uint16:
                Image.fromarray(array, mode="I;16").save(destination)
            else:
                Image.fromarray(array).save(destination)

    for index, pair in enumerate(selected_pairs):
        pair["index"] = index
        pair["rgb"] = f"rgb/{index:06d}.png"
        pair["depth"] = f"depth/{index:06d}.png"

    sync_deltas = [pair["sync_delta_ms"] for pair in selected_pairs]
    manifest = {
        "schema": "soarm.rgbd_sequence.v1",
        "source_bag": args.bag.name,
        "depth_unit": "millimeter_uint16",
        "global_camera_poses_available": False,
        "rgb_depth_registration_verified": False,
        "geometry_usable": all(pair["sync_ok"] for pair in selected_pairs),
        "max_sync_ms": args.max_sync_ms,
        "sync": {
            "mean_ms": float(np.mean(sync_deltas)),
            "p95_ms": float(np.percentile(sync_deltas, 95)),
            "max_ms": max(sync_deltas),
            "pairs_over_limit": sum(not pair["sync_ok"] for pair in selected_pairs),
            "duplicate_depth_frames": len(selected_pairs)
            - len({pair["depth_topic_index"] for pair in selected_pairs}),
        },
        "camera_info": camera_info,
        "camera_internal_transforms": transforms,
        "frames": selected_pairs,
    }
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8"
    )
    print(json.dumps({key: manifest[key] for key in ("schema", "source_bag", "geometry_usable", "sync")}, indent=2))


if __name__ == "__main__":
    main()
