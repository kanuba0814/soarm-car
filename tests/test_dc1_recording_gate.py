"""Run inside the ROS image; replay real bag messages through the recording gate."""
import importlib.util
import sys
import unittest
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

spec = importlib.util.spec_from_file_location('dc1_check', Path('/dc1/dabai-dc1-check.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class RecordingGate(unittest.TestCase):
    def replay(self, drop_depth=False, slow=False):
        check = module.Check()
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri='/bag', storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
        counts = {}
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic not in module.IMAGE_TOPICS + module.INFO_TOPICS + ['/tf_static']:
                continue
            counts[topic] = counts.get(topic, 0) + 1
            if drop_depth and topic == module.IMAGE_TOPICS[1]:
                continue
            if topic in [module.IMAGE_TOPICS[0], module.INFO_TOPICS[0]]:
                # Reproduce RGB/CameraInfo at ~24 Hz, or severe loss at ~6 Hz.
                if (slow and counts[topic] % 5 != 0) or (not slow and counts[topic] % 5 == 0):
                    continue
            check.add(topic, deserialize_message(data, types[topic]))
        return check.result()

    def test_24hz_is_recordable_with_warning(self):
        result = self.replay()
        self.assertTrue(result['passed'], result)
        self.assertTrue(result['warnings'])

    def test_missing_depth_is_rejected(self):
        self.assertFalse(self.replay(drop_depth=True)['passed'])

    def test_severe_rgb_loss_is_rejected(self):
        self.assertFalse(self.replay(slow=True)['passed'])

if __name__ == '__main__':
    unittest.main()
