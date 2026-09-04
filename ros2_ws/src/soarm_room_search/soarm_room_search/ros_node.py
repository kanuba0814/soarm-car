"""ROS thin adapter; all transitions belong to SearchController, never Twist."""
import math
import time
from .controller import SearchController
from .core import State, Workflow, scan_complete
from .freshness import SensorFreshness
try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from soarm_interfaces.msg import LocalizationHealth
    from sensor_msgs.msg import CameraInfo, PointCloud2
    from tf2_ros import Buffer, TransformListener
except ImportError:
    rclpy = None

class Nav2PathVerifier:
    """Fail-closed adapter seam; production implementation must bind ComputePathToPose revision."""
    name = 'Nav2 ComputePathToPose revision verifier'
    def verify(self, pose, revision): return False

class RoomSearchNode(Node if rclpy else object):
    def __init__(self):
        super().__init__('room_search')
        self.declare_parameter('expected_map_id', '')
        self.declare_parameter('health_timeout', .5)
        self.declare_parameter('sensor_timeout', 1.)
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('pointcloud_topic', '/camera/depth/points')
        self.declare_parameter('gui_port', 8088)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.controller = SearchController(Workflow(), self, Nav2PathVerifier(), False)
        self.last_health = 0.; self.healthy = False; self.map_id = ''
        self.sensor_gate = SensorFreshness(float(self.get_parameter('sensor_timeout').value), .1, self.get_parameter('camera_frame').value)
        self.tf_buffer = Buffer(); self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan_samples = []; self.scan_center = None; self.scan_segment = 0
        self.create_subscription(LocalizationHealth, '/localization/health', self._health, 10)
        self.create_subscription(PointCloud2, self.get_parameter('pointcloud_topic').value, self._cloud, 10)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value if self.has_parameter('camera_info_topic') else '/camera/camera_info', self._info, 10)
        self.create_timer(.2, self._safety_tick)
    def _health(self, message):
        self.last_health = time.monotonic()
        self.healthy = message.state == LocalizationHealth.HEALTHY
        self.map_id = getattr(message, 'map_id', '')
    def send(self, pose, accepted, completed):
        if not self._ready(): accepted(None); return
        goal = NavigateToPose.Goal(); goal.pose = PoseStamped(); goal.pose.header.frame_id = 'map'; goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose[:2]; yaw = pose[2]; goal.pose.pose.orientation.z = math.sin(yaw / 2); goal.pose.pose.orientation.w = math.cos(yaw / 2)
        future = self.client.send_goal_async(goal)
        def response(done):
            handle = done.result(); accepted(handle if handle.accepted else None)
            if handle.accepted: handle.get_result_async().add_done_callback(lambda result: completed(result.result.status == GoalStatus.STATUS_SUCCEEDED))
        future.add_done_callback(response)
    def _cloud(self, message):
        self.sensor_gate.pointcloud(message.header.stamp.sec + message.header.stamp.nanosec / 1e9, message.header.frame_id)
    def _info(self, message):
        self.sensor_gate.camera_info(message.header.stamp.sec + message.header.stamp.nanosec / 1e9, message.header.frame_id)
    def _ready(self):
        return self.healthy and self.map_id == self.get_parameter('expected_map_id').value and time.monotonic() - self.last_health < float(self.get_parameter('health_timeout').value)
    def collect_scan_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            q = transform.transform.rotation; yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            p = transform.transform.translation; self.scan_samples.append((p.x, p.y, time.monotonic(), yaw)); self.sensor_gate.transform(True)
        except Exception:
            self.sensor_gate.transform(False)
    def start_scan(self):
        if not self.sensor_gate.ready(): raise RuntimeError('BLOCKED: RGB-D or TF not fresh')
        self.collect_scan_pose()
        if not self.scan_samples: raise RuntimeError('BLOCKED: map to base_footprint unavailable')
        x, y, _, yaw = self.scan_samples[-1]; self.scan_center = (x, y, yaw)
        self.controller.workflow.start_scan(); self.scan_samples = []; self.scan_segment = 0
        self.create_timer(.1, self.collect_scan_pose); self._scan_next()
    def _scan_next(self):
        if self.scan_segment == 4:
            self.finish_scan(); return
        x, y, yaw = self.scan_center; target = (x, y, yaw + (self.scan_segment + 1) * math.pi / 2)
        def accepted(handle):
            if handle is None: self.controller.workflow.state = State.FAILED
        def completed(success):
            if not success: self.controller.workflow.state = State.FAILED; return
            self.scan_segment += 1; self._scan_next()
        self.send(target, accepted, completed)
    def finish_scan(self):
        if not self.sensor_gate.ready() or not scan_complete(self.scan_samples): self.controller.workflow.state = State.BLOCKED; return
        self.controller.workflow.finish_scan(self.scan_samples)
    def sensor_received(self): pass
    def _safety_tick(self):
        active = self.controller.workflow.state in (State.NAVIGATING_TO_SCAN, State.NAVIGATING_TO_DOCK, State.SCANNING)
        sensor_fresh = self.sensor_gate.ready()
        if active and (not self._ready() or not sensor_fresh):
            self.controller.cancel(); self.controller.workflow.state = State.BLOCKED; self.controller.workflow.reason = 'localization/map_id or RGB-D/TF freshness unavailable'
def main():
    if rclpy is None: raise RuntimeError('ROS 2 Humble unavailable; use simulation GUI')
    rclpy.init(); node = RoomSearchNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
