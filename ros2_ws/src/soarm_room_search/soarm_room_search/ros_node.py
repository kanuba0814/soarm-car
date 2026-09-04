"""Hardware ROS adapter: serialized HTTP commands, Nav2 actions, TF and RGB-D gates."""
import math, queue, threading, time
from http.server import ThreadingHTTPServer
from .controller import SearchController
from .core import MapBundle, RoomStore, State, Workflow, WorkflowError
from .freshness import SensorFreshness
from .http_gui import App, server
from .tabletop import decode_xyz, detect_tabletops
try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, PointCloud2
    from soarm_interfaces.msg import LocalizationHealth
    from tf2_ros import Buffer, TransformListener
except ImportError:
    rclpy = None

class ComputePathVerifier:
    name = 'Nav2 ComputePathToPose async cache'
    def __init__(self, node): self.node = node; self.cache = {}; self.pending = set()
    def verify(self, pose, revision):
        key = (round(pose[0], 3), round(pose[1], 3), revision)
        if key not in self.cache and key not in self.pending:
            self.pending.add(key); self.node.compute_path(pose, revision, key)
        return self.cache.get(key, False)
    def result(self, key, ok): self.pending.discard(key); self.cache[key] = ok

class RoomSearchNode(Node if rclpy else object):
    def __init__(self):
        super().__init__('room_search')
        for name, default in [('occupancy_yaml',''), ('expected_map_id',''), ('rooms_json',''), ('rooms_store_path','/tmp/soarm-rooms.json'), ('camera_info_topic','/camera/camera_info'), ('pointcloud_topic','/camera/depth/points'), ('camera_frame','camera_link'), ('gui_port',8088), ('health_timeout',.5), ('sensor_timeout',1.)]: self.declare_parameter(name, default)
        self.ready_reason = 'NOT_READY: occupancy_yaml, expected_map_id and rooms_json required'
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose'); self.path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.verifier = ComputePathVerifier(self); self.controller = SearchController(Workflow(), self, self.verifier, False)
        self.commands = queue.Queue(); self.active_handle = None; self.last_health = 0.; self.healthy = False; self.map_id = ''; self.points = []; self.scan_samples = []; self.scan_timer = None
        self.sensor_gate = SensorFreshness(float(self.get_parameter('sensor_timeout').value), .1, self.get_parameter('camera_frame').value)
        self.tf_buffer = Buffer(); self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LocalizationHealth, '/localization/health', self._health, 10); self.create_subscription(PointCloud2, self.get_parameter('pointcloud_topic').value, self._cloud, 10); self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value, self._info, 10)
        self._load_configuration(); self.create_timer(.05, self._consume); self.create_timer(.2, self._safety)
        self.app = App(self.get_parameter('rooms_store_path').value, controller=self.controller, enqueue=self.commands.put)
        self.http = ThreadingHTTPServer(('127.0.0.1', int(self.get_parameter('gui_port').value)), server(self.app)); threading.Thread(target=self.http.serve_forever, daemon=True).start()
    def _load_configuration(self):
        yaml_path = self.get_parameter('occupancy_yaml').value; expected = self.get_parameter('expected_map_id').value; rooms = self.get_parameter('rooms_json').value
        if not yaml_path or not expected or not rooms: return
        try:
            bundle = MapBundle.load(yaml_path, expected); self.controller.workflow.load(bundle, RoomStore.load(rooms, bundle)); self.ready_reason = 'READY'
        except Exception as error: self.ready_reason = 'NOT_READY: ' + str(error)
    def status(self):
        result = self.controller.status(); result['readiness'] = self.ready_reason; return result
    def _health(self, message): self.last_health = time.monotonic(); self.healthy = message.state == LocalizationHealth.HEALTHY; self.map_id = getattr(message, 'map_id', '')
    def _info(self, message): self.sensor_gate.camera_info(message.header.stamp.sec + message.header.stamp.nanosec / 1e9, message.header.frame_id)
    def _cloud(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9; self.sensor_gate.pointcloud(stamp, message.header.frame_id)
        if self.controller.workflow.state == State.SCANNING and self.sensor_gate.ready():
            try: self.points.extend(decode_xyz(bytes(message.data), message.point_step, limit=30000))
            except Exception: self.block('invalid PointCloud2 xyz')
    def _ready(self): return self.ready_reason == 'READY' and self.healthy and self.map_id == self.get_parameter('expected_map_id').value and time.monotonic()-self.last_health < float(self.get_parameter('health_timeout').value)
    def _consume(self):
        while not self.commands.empty():
            action, data = self.commands.get_nowait()
            try:
                if not self._ready(): raise WorkflowError('NOT_READY', self.ready_reason)
                work = self.controller.workflow
                if action == 'scan_goal': self.controller.navigate_scan(next(room for room in work.rooms if room.id == work.proposal['room_id']).scan_pose)
                elif action == 'start_scan': self.start_scan()
                elif action == 'finish_scan': self.finish_scan()
                elif action == 'analyze': self.analyze()
                elif action == 'preview': self.controller.preview()
                elif action == 'dock_goal': self.controller.navigate_dock(work.selected['dock'] + (0.,))
                elif action == 'cancel': self.controller.cancel()
            except Exception as error: self.block(str(error))
    def send(self, pose, accepted, completed):
        if self.active_handle is not None: accepted(None); return
        goal = NavigateToPose.Goal(); goal.pose = PoseStamped(); goal.pose.header.frame_id = 'map'; goal.pose.header.stamp = self.get_clock().now().to_msg(); goal.pose.pose.position.x, goal.pose.pose.position.y = pose[:2]; goal.pose.pose.orientation.z = math.sin(pose[2]/2); goal.pose.pose.orientation.w = math.cos(pose[2]/2)
        def response(done):
            handle = done.result(); self.active_handle = handle if handle.accepted else None; accepted(self.active_handle)
            if handle.accepted: handle.get_result_async().add_done_callback(lambda future: self._result(future, completed))
        self.client.send_goal_async(goal).add_done_callback(response)
    def _result(self, future, completed): self.active_handle = None; completed(future.result().status == GoalStatus.STATUS_SUCCEEDED)
    def compute_path(self, pose, revision, key):
        if revision != self.controller.workflow.bundle.revision: self.verifier.result(key, False); return
        goal = ComputePathToPose.Goal(); goal.goal = PoseStamped(); goal.goal.header.frame_id = 'map'; goal.goal.pose.position.x, goal.goal.pose.position.y = pose
        self.path_client.send_goal_async(goal).add_done_callback(lambda future: self._path_goal(future, key))
    def _path_goal(self, future, key):
        handle = future.result()
        if not handle.accepted: self.verifier.result(key, False); return
        handle.get_result_async().add_done_callback(lambda result: self.verifier.result(key, result.result.status == GoalStatus.STATUS_SUCCEEDED and len(result.result.path.poses) > 0))
    def start_scan(self):
        if not self.sensor_gate.ready(): raise WorkflowError('NOT_READY', 'RGB-D/TF freshness required')
        self.controller.workflow.start_scan(); self.points = []; self.scan_samples = []
    def finish_scan(self):
        from .core import scan_complete
        if not self.sensor_gate.ready() or not scan_complete(self.scan_samples): self.block('incomplete scan telemetry'); return
        self.controller.workflow.finish_scan(self.scan_samples)
    def analyze(self):
        if not self.sensor_gate.ready(): raise WorkflowError('NOT_READY', 'RGB-D freshness required')
        desks = detect_tabletops(self.points)
        observations = [dict(desk, dock=dock['dock']) for desk in desks for dock in desk['docks']]
        if not observations: raise WorkflowError('NO_DESK', 'no quality tabletop clusters')
        self.controller.workflow.analyze(observations)
    def block(self, reason):
        if self.active_handle is not None: self.active_handle.cancel_goal_async(); self.active_handle = None
        self.controller.cancel(); self.controller.workflow.state = State.BLOCKED; self.controller.workflow.reason = reason
    def _safety(self):
        if self.controller.workflow.state in (State.NAVIGATING_TO_SCAN, State.SCANNING, State.NAVIGATING_TO_DOCK) and (not self._ready() or not self.sensor_gate.ready()): self.block('localization/map/RGB-D freshness unavailable')
def main():
    if rclpy is None: raise RuntimeError('ROS 2 Humble unavailable')
    rclpy.init(); node = RoomSearchNode()
    try: rclpy.spin(node)
    finally: node.http.shutdown(); node.destroy_node(); rclpy.shutdown()
