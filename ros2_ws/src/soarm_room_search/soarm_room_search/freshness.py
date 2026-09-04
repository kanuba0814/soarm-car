"""Pure fail-closed RGB-D freshness policy shared by ROS callbacks and tests."""
import time
class SensorFreshness:
    def __init__(self, timeout, sync_tolerance, expected_frame):
        self.timeout = timeout; self.sync_tolerance = sync_tolerance; self.expected_frame = expected_frame
        self.cloud = None; self.info = None; self.tf_ok = False
    def pointcloud(self, stamp, frame): self.cloud = (stamp, frame, time.monotonic())
    def camera_info(self, stamp, frame): self.info = (stamp, frame, time.monotonic())
    def transform(self, available): self.tf_ok = available
    def ready(self, now=None):
        now = time.monotonic() if now is None else now
        if not self.cloud or not self.info or not self.tf_ok: return False
        cloud_stamp, cloud_frame, cloud_seen = self.cloud; info_stamp, info_frame, info_seen = self.info
        return cloud_frame == self.expected_frame and info_frame == self.expected_frame and abs(cloud_stamp-info_stamp) <= self.sync_tolerance and now-cloud_seen <= self.timeout and now-info_seen <= self.timeout
