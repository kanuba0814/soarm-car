#!/usr/bin/env python3
import argparse
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from PIL import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--topic", default="/camera/color/image_raw")
    parser.add_argument("--count", type=int, default=6)
    args = parser.parse_args()

    information = yaml.safe_load((args.bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    start_ns = information["starting_time"]["nanoseconds_since_epoch"]
    duration_ns = information["duration"]["nanoseconds"]
    targets = [start_ns + duration_ns * (index + 1) // (args.count + 1) for index in range(args.count)]

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=information["storage_identifier"]),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_type = get_message(topic_types[args.topic])
    args.output.mkdir(parents=True, exist_ok=True)

    target_index = 0
    while reader.has_next() and target_index < len(targets):
        topic, serialized, timestamp = reader.read_next()
        if topic != args.topic or timestamp < targets[target_index]:
            continue
        message = deserialize_message(serialized, message_type)
        output_path = args.output / f"frame-{target_index + 1:02d}.png"
        if message.encoding in ("rgb8", "bgr8"):
            image = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
            image = image[:, :message.width * 3].reshape(message.height, message.width, 3)
            if message.encoding == "bgr8":
                image = image[:, :, ::-1]
            Image.fromarray(image).save(output_path)
        elif message.encoding in ("16UC1", "mono16"):
            image = np.frombuffer(message.data, dtype=np.uint16).reshape(
                message.height, message.step // 2
            )[:, :message.width]
            Image.fromarray(image, mode="I;16").save(output_path)
        else:
            raise RuntimeError(f"Unsupported image encoding: {message.encoding}")
        target_index += 1

    if target_index != len(targets):
        raise RuntimeError(f"Expected {len(targets)} frames, extracted {target_index}")
    print(f"Extracted {target_index} frames to {args.output}")


if __name__ == "__main__":
    main()
