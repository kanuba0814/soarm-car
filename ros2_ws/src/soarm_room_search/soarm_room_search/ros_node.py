"""Hardware ROS adapter: serialized HTTP commands, Nav2 actions, TF and RGB-D gates."""
import math, queue, shlex, threading, time
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
        parameters = [
            ('occupancy_yaml', ''),
            ('expected_map_id', ''),
            ('rooms_json', ''),
            ('rooms_store_path', '/tmp/soarm-rooms.json'),
            ('asr_command', ''),
            ('camera_info_topic', '/camera/camera_info'),
            ('pointcloud_topic', '/camera/depth/points'),
            ('camera_frame', 'camera_link'),
            ('gui_port', 8088),
            ('health_timeout', .5),
            ('sensor_timeout', 1.),
        ]
        for name, default in parameters:
            self.declare_parameter(name, default)
        self.ready_reason = 'NOT_READY: occupancy_yaml and expected_map_id required'
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose'); self.path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.verifier = ComputePathVerifier(self); self.controller = SearchController(Workflow(), self, self.verifier, False)
        self.commands = queue.Queue()
        self.active_handle = None
        self.transport_pending = False
        self.transport_generation = 0
        self.last_health = 0.0
        self.healthy = False
        self.map_id = ''
        self.points = []
        self.scan_samples = []
        self.scan_center = None
        self.scan_segment = 0
        self.sensor_gate = SensorFreshness(float(self.get_parameter('sensor_timeout').value), .1, self.get_parameter('camera_frame').value)
        self.tf_buffer = Buffer(); self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LocalizationHealth, '/localization/health', self._health, 10); self.create_subscription(PointCloud2, self.get_parameter('pointcloud_topic').value, self._cloud, 10); self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value, self._info, 10)
        self._load_configuration()
        self.create_timer(.05, self._consume)
        self.create_timer(.1, self._sample_scan_pose)
        self.create_timer(.2, self._safety)
        asr_command = self.get_parameter('asr_command').value
        self.app = App(
            self.get_parameter('rooms_store_path').value,
            asr_argv=shlex.split(asr_command) if asr_command else None,
            controller=self.controller,
            enqueue=lambda action, data: self.commands.put((action, data)),
            status_provider=self.status,
        )
        self.http = ThreadingHTTPServer(('127.0.0.1', int(self.get_parameter('gui_port').value)), server(self.app)); threading.Thread(target=self.http.serve_forever, daemon=True).start()
    def _load_configuration(self):
        yaml_path = self.get_parameter('occupancy_yaml').value
        expected = self.get_parameter('expected_map_id').value
        rooms_path = self.get_parameter('rooms_json').value
        if not yaml_path or not expected:
            return
        try:
            bundle = MapBundle.load(yaml_path, expected)
            self.controller.workflow.bundle = bundle
            if rooms_path:
                self.controller.workflow.load(bundle, RoomStore.load(rooms_path, bundle))
                self.ready_reason = 'READY'
            else:
                self.ready_reason = 'NOT_READY: define and save at least one room in the GUI'
        except Exception as error:
            self.ready_reason = 'NOT_READY: ' + str(error)

    def _configuration_ready(self):
        workflow = self.controller.workflow
        return workflow.bundle is not None and bool(workflow.rooms)

    def status(self):
        if self._configuration_ready() and self.ready_reason.startswith('NOT_READY: define'):
            self.ready_reason = 'READY'
        result = self.controller.status()
        result['readiness'] = self.ready_reason
        return result
    def _health(self, message): self.last_health = time.monotonic(); self.healthy = message.state == LocalizationHealth.HEALTHY; self.map_id = getattr(message, 'map_id', '')
    def _info(self, message): self.sensor_gate.camera_info(message.header.stamp.sec + message.header.stamp.nanosec / 1e9, message.header.frame_id)
    def _cloud(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        self.sensor_gate.pointcloud(stamp, message.header.frame_id)
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                message.header.frame_id,
                rclpy.time.Time.from_msg(message.header.stamp),
            )
            self.sensor_gate.transform(True)
        except Exception:
            self.sensor_gate.transform(False)
            if self.controller.workflow.state == State.SCANNING:
                self.block('PointCloud2 transform to map unavailable')
            return
        if self.controller.workflow.state != State.SCANNING:
            return
        try:
            offsets = {field.name: field.offset for field in message.fields}
            camera_points = decode_xyz(
                bytes(message.data),
                message.point_step,
                offsets.get('x', 0),
                offsets.get('y', 4),
                offsets.get('z', 8),
                limit=30000,
            )
            self.points.extend(self._transform_points(camera_points, transform.transform))
        except Exception as error:
            self.sensor_gate.transform(False)
            self.block('invalid or untransformable PointCloud2: ' + str(error))

    @staticmethod
    def _transform_points(points, transform):
        q = transform.rotation
        tx, ty, tz = transform.translation.x, transform.translation.y, transform.translation.z
        # Quaternion rotation matrix, applied before translation.
        xx, yy, zz = q.x * q.x, q.y * q.y, q.z * q.z
        xy, xz, yz = q.x * q.y, q.x * q.z, q.y * q.z
        wx, wy, wz = q.w * q.x, q.w * q.y, q.w * q.z
        rotated = []
        for x, y, z in points:
            rotated.append((
                (1 - 2 * (yy + zz)) * x + 2 * (xy - wz) * y + 2 * (xz + wy) * z + tx,
                2 * (xy + wz) * x + (1 - 2 * (xx + zz)) * y + 2 * (yz - wx) * z + ty,
                2 * (xz - wy) * x + 2 * (yz + wx) * y + (1 - 2 * (xx + yy)) * z + tz,
            ))
        return rotated
    def _ready(self):
        if self._configuration_ready() and self.ready_reason.startswith('NOT_READY: define'):
            self.ready_reason = 'READY'
        return (
            self._configuration_ready()
            and self.ready_reason == 'READY'
            and self.healthy
            and self.map_id == self.get_parameter('expected_map_id').value
            and time.monotonic() - self.last_health
            < float(self.get_parameter('health_timeout').value)
        )
    def _consume(self):
        while not self.commands.empty():
            action, data = self.commands.get_nowait()
            try:
                if action == 'cancel':
                    self.controller.cancel()
                    continue
                if action == 'draw_save_room':
                    if self.motion_active():
                        raise WorkflowError('GOAL_ACTIVE', 'cannot edit rooms during navigation')
                    self.app.call(action, data)
                    continue
                if not self._ready():
                    raise WorkflowError('NOT_READY', self.ready_reason)
                work = self.controller.workflow
                if action == 'scan_goal': self.controller.navigate_scan(next(room for room in work.rooms if room.id == work.proposal['room_id']).scan_pose)
                elif action == 'start_scan': self.start_scan()
                elif action == 'finish_scan': self.finish_scan()
                elif action == 'analyze': self.analyze()
                elif action == 'preview': self.controller.preview()
                elif action == 'dock_goal':
                    self.controller.navigate_dock(
                        work.selected['dock'] + (work.selected['dock_yaw'],))
            except WorkflowError as error:
                # Operator sequencing and pending path checks are not hardware faults.
                self.controller.workflow.reason = error.code + ': ' + error.message
            except Exception as error:
                self.block(str(error))
    def motion_active(self):
        return (
            self.transport_pending
            or self.active_handle is not None
            or self.controller.goal_pending
            or self.controller.accepted_goal is not None
        )

    def send(self, pose, accepted, completed):
        if self.transport_pending or self.active_handle is not None:
            accepted(None)
            return
        self.transport_pending = True
        self.transport_generation += 1
        generation = self.transport_generation
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose[:2]
        goal.pose.pose.orientation.z = math.sin(pose[2] / 2)
        goal.pose.pose.orientation.w = math.cos(pose[2] / 2)

        def response(done):
            handle = done.result()
            if generation != self.transport_generation:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            self.transport_pending = False
            self.active_handle = handle if handle.accepted else None
            accepted(self.active_handle)
            if handle.accepted:
                handle.get_result_async().add_done_callback(
                    lambda future: self._result(future, completed, generation))

        self.client.send_goal_async(goal).add_done_callback(response)

    def _result(self, future, completed, generation):
        if generation != self.transport_generation:
            return
        self.active_handle = None
        self.transport_pending = False
        completed(future.result().status == GoalStatus.STATUS_SUCCEEDED)

    def cancel_pending(self):
        self.transport_generation += 1
        self.transport_pending = False
        if self.active_handle is not None:
            self.active_handle.cancel_goal_async()
            self.active_handle = None
    def compute_path(self, pose, revision, key):
        if revision != self.controller.workflow.bundle.revision: self.verifier.result(key, False); return
        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x, goal.goal.pose.position.y = pose
        goal.goal.pose.orientation.w = 1.0
        self.path_client.send_goal_async(goal).add_done_callback(lambda future: self._path_goal(future, key))
    def _path_goal(self, future, key):
        handle = future.result()
        if not handle.accepted: self.verifier.result(key, False); return
        handle.get_result_async().add_done_callback(
            lambda future: self._path_result(future, key))

    def _path_result(self, future, key):
        wrapped = future.result()
        ok = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and bool(wrapped.result.path.poses)
        )
        self.verifier.result(key, ok)

    def _lookup_base_pose(self):
        transform = self.tf_buffer.lookup_transform(
            'map', 'base_footprint', rclpy.time.Time())
        q = transform.transform.rotation
        yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z),
        )
        position = transform.transform.translation
        return position.x, position.y, time.monotonic(), yaw

    def _sample_scan_pose(self):
        if self.controller.workflow.state != State.SCANNING:
            return
        try:
            self.scan_samples.append(self._lookup_base_pose())
            self.sensor_gate.transform(True)
        except Exception:
            self.sensor_gate.transform(False)

    def start_scan(self):
        if not self.sensor_gate.ready():
            raise WorkflowError('NOT_READY', 'RGB-D/TF freshness required')
        try:
            initial = self._lookup_base_pose()
        except Exception as error:
            raise WorkflowError('NOT_READY', 'map to base_footprint TF unavailable') from error
        self.controller.workflow.start_scan()
        self.points = []
        self.scan_samples = [initial]
        self.scan_center = (initial[0], initial[1], initial[3])
        self.scan_segment = 0
        self._send_next_scan_segment()

    def _send_next_scan_segment(self):
        if self.scan_segment >= 4:
            self.finish_scan()
            return
        if self.active_handle is not None or self.controller.accepted_goal is not None:
            self.block('concurrent navigation goal during scan')
            return
        x, y, initial_yaw = self.scan_center
        target = (x, y, initial_yaw + (self.scan_segment + 1) * math.pi / 2)

        def accepted(handle):
            if handle is None:
                self.block('Nav2 rejected scan heading goal')

        def completed(success):
            if not success:
                self.block('Nav2 failed scan heading goal')
                return
            self.scan_segment += 1
            self._send_next_scan_segment()

        self.send(target, accepted, completed)
    def finish_scan(self):
        from .core import scan_complete
        if self.scan_segment < 4:
            raise WorkflowError('INVALID_STATE', 'four scan headings are not complete')
        if not self.sensor_gate.ready() or not scan_complete(self.scan_samples):
            self.block('incomplete scan telemetry')
            return
        self.controller.workflow.finish_scan(self.scan_samples)
    def analyze(self):
        if not self.sensor_gate.ready(): raise WorkflowError('NOT_READY', 'RGB-D freshness required')
        desks = detect_tabletops(self.points)
        observations = [
            dict(desk, dock=dock['dock'], dock_yaw=dock['dock_yaw'])
            for desk in desks
            for dock in desk['docks']
        ]
        if not observations: raise WorkflowError('NO_DESK', 'no quality tabletop clusters')
        self.controller.workflow.analyze(observations)
    def block(self, reason):
        self.cancel_pending()
        self.controller.cancel()
        self.controller.workflow.state = State.BLOCKED
        self.controller.workflow.reason = reason
    def _safety(self):
        if self.controller.workflow.state in (State.NAVIGATING_TO_SCAN, State.SCANNING, State.NAVIGATING_TO_DOCK) and (not self._ready() or not self.sensor_gate.ready()): self.block('localization/map/RGB-D freshness unavailable')
def main():
    if rclpy is None: raise RuntimeError('ROS 2 Humble unavailable')
    rclpy.init(); node = RoomSearchNode()
    try: rclpy.spin(node)
    finally: node.http.shutdown(); node.destroy_node(); rclpy.shutdown()
