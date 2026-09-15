"""Dump per-message header and arrival timestamps for the DC1 RGB/depth streams.

Runs inside the ros2-dev image (rosbag2_py). Arrival time is the rosbag2 recv
timestamp; header stamp is what the driver wrote. Their difference is driver
pipeline latency under the recording conditions, not a synchronization proof.
"""
import argparse, csv, json
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

p = argparse.ArgumentParser()
p.add_argument('bag')
p.add_argument('out_csv')
a = p.parse_args()

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id='sqlite3'),
       rosbag2_py.ConverterOptions('', ''))
counts = {}
rows = []
while r.has_next():
    topic, data, recv = r.read_next()
    if topic not in ('/camera/color/image_raw', '/camera/depth/image_raw'):
        continue
    m = deserialize_message(data, Image)
    header_ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    # rosbag2_py Humble returns the recv timestamp as int nanoseconds
    recv_ns = recv.nanoseconds if hasattr(recv, 'nanoseconds') else int(recv)
    stream = 'rgb' if '/color/' in topic else 'depth'
    counts[stream] = counts.get(stream, 0) + 1
    rows.append({'stream': stream, 'index': counts[stream] - 1,
                 'header_ts': f'{header_ts:.9f}', 'recv_ts': f'{recv_ns * 1e-9:.9f}',
                 'path': f'{stream}/{counts[stream] - 1:06d}.png'})

out = Path(a.out_csv)
out.parent.mkdir(parents=True, exist_ok=True)
with out.open('w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=['stream', 'index', 'header_ts', 'recv_ts', 'path'])
    w.writeheader()
    w.writerows(rows)
print(json.dumps({'bag': a.bag, 'rows': len(rows), 'counts': counts}))
