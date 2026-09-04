"""ROS thin adapter; all transitions belong to SearchController, never Twist."""
import math
import time
from .controller import SearchController
from .core import State, Workflow
try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from soarm_interfaces.msg import LocalizationHealth
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
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.controller = SearchController(Workflow(), self, Nav2PathVerifier(), False)
        self.last_health = 0.; self.healthy = False; self.map_id = ''; self.last_sensor = 0.
        self.create_subscription(LocalizationHealth, '/localization/health', self._health, 10)
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
    def _ready(self):
        return self.healthy and self.map_id == self.get_parameter('expected_map_id').value and time.monotonic() - self.last_health < float(self.get_parameter('health_timeout').value)
    def sensor_received(self): self.last_sensor = time.monotonic()
    def _safety_tick(self):
        active = self.controller.workflow.state in (State.NAVIGATING_TO_SCAN, State.NAVIGATING_TO_DOCK, State.SCANNING)
        sensor_fresh = self.last_sensor and time.monotonic() - self.last_sensor < float(self.get_parameter('sensor_timeout').value)
        if active and (not self._ready() or not sensor_fresh):
            self.controller.cancel(); self.controller.workflow.state = State.BLOCKED; self.controller.workflow.reason = 'localization/map_id or RGB-D/TF freshness unavailable'
def main():
    if rclpy is None: raise RuntimeError('ROS 2 Humble unavailable; use simulation GUI')
    rclpy.init(); node = RoomSearchNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
