import copy
import hashlib
from pathlib import Path

from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2

from soarm_simulation.sim_adapter import SimAdapter


RAW_CLOUD_SHA256 = 'c9bad8b116c80a725e208f0be9c81b42df5385c6c6b5da4c5196a496ae75568e'


class CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


def captured_cloud():
    root = Path(__file__).resolve().parents[4]
    payload = (root / 'artifacts/intel-renderer-parent-diagnostic/raw-cloud.cdr').read_bytes()
    assert hashlib.sha256(payload).hexdigest() == RAW_CLOUD_SHA256
    return deserialize_message(payload, PointCloud2)


def test_frozen_sim_adapter_on_points_matches_captured_layout_and_content():
    source = captured_cloud()
    assert (source.width, source.height, source.point_step, source.row_step, len(source.data)) == (720, 16, 32, 23040, 368640)
    original = copy.deepcopy(source)
    adapter = type('CaptureAdapter', (), {})()
    adapter.points_pub = CapturePublisher()
    adapter.livox_pub = CapturePublisher()

    SimAdapter.on_points(adapter, source)

    assert len(adapter.points_pub.messages) == 1
    assert len(adapter.livox_pub.messages) == 1
    adapted = adapter.points_pub.messages[0]
    livox = adapter.livox_pub.messages[0]
    assert adapted.header.frame_id == 'livox_frame'
    assert adapted.header.stamp == original.header.stamp
    assert adapted.data == original.data
    assert adapted.fields == original.fields
    assert (adapted.width, adapted.height, adapted.point_step, adapted.row_step) == (720, 16, 32, 23040)
    assert livox.header == adapted.header
    assert livox.timebase == original.header.stamp.sec * 1_000_000_000 + original.header.stamp.nanosec
    assert livox.point_num == len(livox.points) == 11083
    assert livox.lidar_id == 0
    assert all(point.reflectivity == 128 and point.tag == 0x10 and point.line == index % 4 for index, point in enumerate(livox.points))
    assert livox.points[0].offset_time == 0
    assert livox.points[-1].offset_time == 100_000_000
    assert serialize_message(livox)
