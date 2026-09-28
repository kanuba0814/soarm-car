#!/usr/bin/env python3
import sys
from pathlib import Path

import yaml


bag_directory = Path(sys.argv[1])
metadata_path = bag_directory / "metadata.yaml"
metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
information = metadata["rosbag2_bagfile_information"]
duration = information["duration"]["nanoseconds"] / 1_000_000_000
counts = {
    item["topic_metadata"]["name"]: item["message_count"]
    for item in information["topics_with_message_count"]
}

errors = []
required_rates = {
    "/camera/color/image_raw": 28.0,
    "/camera/depth/image_raw": 28.0,
    "/camera/depth/points": 27.0,
    "/tf": 8.0,
}
if duration < 1.0:
    errors.append("录制时间不足 1 秒，无法可靠检查帧率")
else:
    for topic, minimum_rate in required_rates.items():
        rate = counts.get(topic, 0) / duration
        if rate < minimum_rate:
            errors.append(f"{topic} 仅 {rate:.1f} Hz（要求至少 {minimum_rate:.1f} Hz）")

for info_topic in ("/camera/color/camera_info", "/camera/depth/camera_info"):
    if counts.get(info_topic, 0) < 1:
        errors.append(f"{info_topic} 缺失")

depth_count = counts.get("/camera/depth/image_raw", 0)
point_count = counts.get("/camera/depth/points", 0)
if depth_count and point_count < depth_count * 0.95:
    errors.append("点云数量低于深度图的 95%")

if errors:
    print("录制完整性检查失败：", file=sys.stderr)
    for error in errors:
        print(f"  - {error}", file=sys.stderr)
    raise SystemExit(2)

print(
    f"完整性检查通过：{duration:.1f} 秒，"
    f"彩色 {counts['/camera/color/image_raw'] / duration:.1f} Hz，"
    f"深度 {counts['/camera/depth/image_raw'] / duration:.1f} Hz，"
    f"点云 {counts['/camera/depth/points'] / duration:.1f} Hz"
)
