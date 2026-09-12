"""ROS/Nav2 adapter for the pure room-navigation controller."""
import math
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path

from .controller import RoomNavigationController
from .core import MapBundle, RoomStore, TaskState
from .http_gui import App, server
from .planner import OpenAICompatiblePlanner

try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from action_msgs.srv import CancelGoal
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import PoseStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from soarm_interfaces.msg import LocalizationHealth
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener
except ImportError:
    rclpy = None


class RoomNavigationNode(Node if rclpy else object):
    def __init__(self):
        super().__init__('room_navigation')
        for name, default in [('occupancy_yaml', ''), ('localization_pcd', ''), ('rooms_path', '/tmp/soarm-room-navigation-rooms.json'),
                              ('task_journal', '/tmp/soarm-room-navigation-task.json'), ('gui_port', 8090), ('health_timeout', .5),
                              ('cancel_timeout', 5.0), ('goal_acceptance_timeout', 5.0), ('goal_execution_timeout', 120.0), ('fixture_demo', False), ('alignment', 'house-origin-v1:T_map_camera_init=identity')]:
            self.declare_parameter(name, default)
        self.health_state, self.health_map_id, self.health_receipt = None, '', None
        self.safe_zero, self.safe_receipt, self.gate_armed, self.gate_receipt = False, None, False, None
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.position_only_behavior_tree = str(Path(get_package_share_directory('soarm_navigation')) /
                                              'behavior_trees' / 'navigate_position_only_holonomic.xml')
        self.pending_cancel = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.gate_stop = self.create_client(Trigger, '/localization_cmd_gate/stop')
        self.gate_arm = self.create_client(Trigger, '/localization_cmd_gate/arm')
        self.gate_status = self.create_client(Trigger, '/localization_cmd_gate/status')
        self.gate_status_future = None
        self.tf_buffer, self.tf_listener = Buffer(), None
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LocalizationHealth, '/localization/health', self._health, 10)
        self.create_subscription(Twist, '/safe_cmd_vel', self._safe_command, 10)
        bundle = MapBundle.load(self.get_parameter('occupancy_yaml').value, self.get_parameter('localization_pcd').value,
                                alignment=self.get_parameter('alignment').value)
        rooms_path = self.get_parameter('rooms_path').value
        if Path(rooms_path).is_file(): rooms, revision = RoomStore.load(rooms_path, bundle)
        else: rooms, revision = [], 1
        self.controller = RoomNavigationController(bundle, rooms, self, self.ready, self.zero_output, self.capture_start_pose,
                                                   self.get_parameter('task_journal').value,
                                                   float(self.get_parameter('goal_acceptance_timeout').value),
                                                   float(self.get_parameter('goal_execution_timeout').value))
        self.controller.room_revision = revision
        self.planner = OpenAICompatiblePlanner()
        self.app = App(self.controller, bundle, rooms_path, self.planner, bool(self.get_parameter('fixture_demo').value), 'ROS', self.request_arm, self.ready)
        port = int(self.get_parameter('gui_port').value)
        self.http = ThreadingHTTPServer(('127.0.0.1', port), server(self.app, port))
        threading.Thread(target=self.http.serve_forever, daemon=True).start()
        self.create_timer(.05, self._tick); self.create_timer(.2, self._gate_status_tick)

    def _health(self, message):
        self.health_state, self.health_map_id, self.health_receipt = message.state, message.map_id, time.monotonic()

    def _safe_command(self, message):
        self.safe_zero = max(abs(message.linear.x), abs(message.linear.y), abs(message.angular.z)) <= 1e-6
        self.safe_receipt = time.monotonic()

    def _gate_status_tick(self):
        if self.gate_status_future is not None:
            return
        if self.gate_status.service_is_ready():
            future = self.gate_status.call_async(Trigger.Request())
            self.gate_status_future = future
            future.add_done_callback(self._gate_status_result)
        else:
            self.gate_armed, self.gate_receipt = False, None

    def _gate_status_result(self, future):
        if future is not self.gate_status_future:
            return
        self.gate_status_future = None
        try:
            response = future.result()
            self.gate_armed, self.gate_receipt = bool(response.success and 'armed=True' in response.message), time.monotonic()
        except Exception:
            self.gate_armed, self.gate_receipt = False, None

    def _health_ready(self):
        timeout, now = float(self.get_parameter('health_timeout').value), time.monotonic()
        return bool(self.controller.bundle.intact() and self.controller.rooms and self.client.server_is_ready() and
                    self.health_state == LocalizationHealth.HEALTHY and self.health_map_id == self.controller.bundle.bundle_id and
                    self.health_receipt is not None and 0 <= now - self.health_receipt < timeout)

    def _base_ready(self):
        timeout, now = float(self.get_parameter('health_timeout').value), time.monotonic()
        return self._health_ready() and self.gate_armed and self.gate_receipt is not None and 0 <= now - self.gate_receipt < timeout

    def ready(self):
        return self._readiness_snapshot()['ready']

    def zero_output(self):
        return bool(self.safe_zero and self.safe_receipt is not None and 0 <= time.monotonic() - self.safe_receipt < .5)

    def _fresh_pose(self):
        return self._readiness_snapshot()['pose']

    def _readiness_snapshot(self):
        """Return the complete fail-closed readiness decision for failure logs."""
        timeout, monotonic_now = float(self.get_parameter('health_timeout').value), time.monotonic()
        health_age = None if self.health_receipt is None else monotonic_now - self.health_receipt
        gate_age = None if self.gate_receipt is None else monotonic_now - self.gate_receipt
        snapshot = {
            'bundle_intact': self.controller.bundle.intact(), 'rooms_present': bool(self.controller.rooms),
            'nav2_ready': self.client.server_is_ready(), 'health_state': self.health_state,
            'map_matches': self.health_map_id == self.controller.bundle.bundle_id,
            'health_age': health_age, 'gate_armed': self.gate_armed, 'gate_age': gate_age,
            'tf_age': None, 'tf_error': '', 'pose': None,
        }
        health_ready = bool(snapshot['bundle_intact'] and snapshot['rooms_present'] and snapshot['nav2_ready'] and
                            self.health_state == LocalizationHealth.HEALTHY and snapshot['map_matches'] and
                            health_age is not None and 0 <= health_age < timeout)
        gate_ready = bool(self.gate_armed and gate_age is not None and 0 <= gate_age < timeout)
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            age = (self.get_clock().now().nanoseconds - (transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec)) / 1e9
            snapshot['tf_age'] = age
            if 0 <= age < timeout:
                q, p = transform.transform.rotation, transform.transform.translation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                snapshot['pose'] = (p.x, p.y, yaw)
        except Exception as error:
            snapshot['tf_error'] = type(error).__name__
        snapshot['ready'] = bool(health_ready and gate_ready and snapshot['pose'] is not None)
        return snapshot

    def capture_start_pose(self):
        return self._fresh_pose()

    def request_arm(self):
        if self.controller.recovery_required:
            return False
        if self.controller.state in (TaskState.CANCELING, TaskState.RUNNING_NAV, TaskState.RUNNING_WAIT, TaskState.SETTLING, TaskState.CONFIRMATION, TaskState.PROPOSAL, TaskState.PLANNING):
            return False
        if not self._health_ready() or not self.gate_arm.service_is_ready():
            return False
        self.app.arm_status = 'REQUESTED'
        self.gate_arm.call_async(Trigger.Request()).add_done_callback(self._arm_result)
        return True

    def _arm_result(self, future):
        try:
            response = future.result()
            self.app.arm_status = 'ARMED' if response.success else 'REFUSED: ' + response.message
            if not response.success:
                self.gate_armed, self.gate_receipt = False, None
        except Exception:
            self.app.arm_status = 'REFUSED: gate transport failed'
            self.gate_armed, self.gate_receipt = False, None

    def send(self, pose, require_yaw=True, accepted=None, completed=None):
        # The default preserves callers using the legacy strict-heading transport API.
        if callable(require_yaw):
            require_yaw, accepted, completed = True, require_yaw, accepted
        goal = NavigateToPose.Goal(); goal.pose = PoseStamped(); goal.pose.header.frame_id = 'map'
        if not require_yaw:
            goal.behavior_tree = self.position_only_behavior_tree
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(pose[0]), float(pose[1])
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        def response(future):
            try: handle = future.result()
            except Exception: accepted(None, 'transport'); return
            accepted(handle if handle.accepted else None, 'rejected')
            if handle.accepted: handle.get_result_async().add_done_callback(lambda result: self._result(result, completed))
        self.client.send_goal_async(goal).add_done_callback(response)

    @staticmethod
    def _result(future, completed):
        try:
            status = future.result().status
            completed('succeeded' if status == GoalStatus.STATUS_SUCCEEDED else 'aborted' if status == GoalStatus.STATUS_ABORTED else 'canceled')
        except Exception:
            completed('transport')

    def cancel(self, handle):
        try: handle.cancel_goal_async()
        except Exception: self.controller.safety_block('Nav2 cancel transport failed')

    def cancel_pending(self):
        if self.pending_cancel.service_is_ready():
            self.pending_cancel.call_async(CancelGoal.Request())
        else:
            self.controller.safety_block('Nav2 pending-goal cancel service unavailable')

    def stop_gate(self, callback):
        self.gate_armed, self.gate_receipt = False, None
        if not self.gate_stop.service_is_ready():
            callback(False); return None
        future = self.gate_stop.call_async(Trigger.Request())
        def finished(result):
            try: callback(bool(result.result().success))
            except Exception: callback(False)
        future.add_done_callback(finished)
        return None

    def _tick(self):
        if self.controller.recovery_required:
            if self.gate_stop.service_is_ready():
                self.controller.begin_recovery_stop()
            return
        active = self.controller.state in (TaskState.RUNNING_NAV, TaskState.RUNNING_WAIT, TaskState.SETTLING, TaskState.CONFIRMATION, TaskState.PROPOSAL)
        snapshot = self._readiness_snapshot() if active else None
        if active and not snapshot['ready']:
            self.get_logger().error('Room navigation readiness lost: %s', snapshot)
            self.controller.safety_block('fresh healthy localization, digest-matching map, armed gate, and Nav2 are required')
        self.controller.tick()
        if self.controller.state == TaskState.CANCELING:
            if not hasattr(self, '_cancel_started'): self._cancel_started = time.monotonic()
            if time.monotonic() - self._cancel_started > float(self.get_parameter('cancel_timeout').value): self.controller.cancel_timeout()
        else: self._cancel_started = time.monotonic()


def main(args=None):
    if rclpy is None: raise RuntimeError('ROS 2 Humble unavailable')
    rclpy.init(args=args); node = RoomNavigationNode()
    try: rclpy.spin(node)
    finally:
        node.http.shutdown(); node.app.close(); node.destroy_node(); rclpy.shutdown()
