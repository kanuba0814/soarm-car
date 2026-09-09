#!/usr/bin/env python3
"""Bounded research-only AWS Small House known-self-pose physical-motion pilot."""
import hashlib
import json
import math
import os
import struct
import subprocess
import sys
import tarfile
import threading
import time
from collections import deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from sensor_msgs.msg import Image, PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener

RESOLUTION = 0.05
SIZE = 320
ROBOT_RADIUS_M = 0.50  # Research-only orientation-independent clearance model.
MAX_GOAL_DISTANCE = 1.5
MIN_GOAL_DISTANCE = 0.7
MAX_ROUTE_CANDIDATES = 16
SOURCE_FRESHNESS_SECONDS = 0.5
NATIVE_LIDAR_FRAME = 'livox_frame'
NATIVE_LIDAR_Z_M = 0.170
MODEL_TO_BASE_LINK_Z_M = 0.0763
SOURCE_STARTUP_MINIMUM_SAMPLES = 4
PLACEMENT_TOLERANCE_M = 0.05
STATIONARY_LINEAR_MPS = 0.02
STATIONARY_ANGULAR_RADPS = 0.03


class NativePoseStream:
    """Thread-buffered native Pose_V envelopes; ROS executor never owns this pipe."""
    def __init__(self):
        self.records, self.errors = deque(maxlen=12000), deque(maxlen=200)
        self.lock = threading.Lock()
        self.process = subprocess.Popen(
            ['stdbuf', '-oL', 'ign', 'topic', '-e', '-t', '/world/default/dynamic_pose/info', '--json-output'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        for line in self.process.stdout:
            try:
                envelope = json.loads(line)
                stamp = envelope.get('header', {}).get('stamp', {})
                stamp_ns = int(stamp.get('sec', 0)) * 1_000_000_000 + int(stamp.get('nsec', stamp.get('nanosec', 0)))
                entity = next((item for item in envelope.get('pose', []) if item.get('name') == 'kuruma'), None)
                if stamp_ns <= 0 or entity is None:
                    self.errors.append({'reason': 'missing_nonzero_stamp_or_exact_kuruma', 'envelope': envelope})
                    continue
                with self.lock:
                    self.records.append({'source_stamp_ns': stamp_ns, 'entity': entity, 'envelope': envelope})
            except (ValueError, json.JSONDecodeError) as error:
                self.errors.append({'reason': 'json_decode', 'detail': str(error), 'raw_line': line})

    def drain(self, after_ns):
        with self.lock:
            return [record for record in self.records if record['source_stamp_ns'] > after_ns]

    def snapshot(self):
        with self.lock:
            return list(self.records), list(self.errors)

    def close(self):
        self.process.terminate()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
        self.thread.join(timeout=3)


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


class Pilot:
    def __init__(self, output):
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        rclpy.init()
        self.node = rclpy.create_node(
            'aws_knownselfpose_pilot',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.tf_pub = TransformBroadcaster(self.node)
        self.static_tf_pub = StaticTransformBroadcaster(self.node)
        self.command_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_pub = self.node.create_publisher(Odometry, '/known_self_pose/odom', 10)
        self.node.create_subscription(TFMessage, '/world/default/dynamic_pose/info', self.on_bridge_pose, 30)
        self.node.create_subscription(PointCloud2, '/mid360/scan/points/points', self.on_cloud, 10)
        self.node.create_subscription(Image, '/camera/image', lambda m: self.on_image('rgb', m), 10)
        self.node.create_subscription(Image, '/camera/depth_image', lambda m: self.on_image('depth', m), 10)
        self.node.create_subscription(Twist, '/cmd_vel', self.on_command, 30)
        self.node.create_subscription(Twist, '/cmd_vel_nav', self.on_nav_command, 30)
        self.pose = None
        self.origin = None
        self.origin_model_z = None
        self.native_pose_stream = NativePoseStream()
        self.last_native_stamp_ns = 0
        self.bridge_pose_headers = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=False)
        self.previous_pose = None
        self.body_twist = (0.0, 0.0, 0.0)
        self.lidar_frames = set()
        self.lidar_extrinsic_frames = {NATIVE_LIDAR_FRAME}
        self.lifecycle_clients = {
            name: self.node.create_client(GetState, f'{name}/get_state')
            for name in ('/planner_server', '/controller_server', '/bt_navigator')
        }
        self.lifecycle_futures = {}
        self.lifecycle_attempts = {name: 0 for name in self.lifecycle_clients}
        self.clouds = []
        self.images = {}
        self.trajectory = []
        self.commands = []
        self.nav_commands = []
        self.map = [[205 for _ in range(SIZE)] for _ in range(SIZE)]
        self.pose_samples = []
        self.last_pose_wall = 0.0
        self.initial_placement_used = False
        self.initial_placement = None
        self.events = []
        self.publish_native_lidar_extrinsic()
        self.node.create_timer(0.05, self.publish_pose)

    def publish_native_lidar_extrinsic(self, child_frame=NATIVE_LIDAR_FRAME):
        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = child_frame
        transform.transform.translation.z = NATIVE_LIDAR_Z_M
        transform.transform.rotation.w = 1.0
        self.static_tf_pub.sendTransform(transform)
        self.lidar_extrinsic_frames.add(child_frame)

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def on_bridge_pose(self, message):
        for transform in message.transforms:
            if transform.child_frame_id in ('kuruma', 'kuruma/base_link'):
                self.bridge_pose_headers.append({
                    'bridge_stamp_ns': transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec,
                    'parent': transform.header.frame_id, 'child': transform.child_frame_id,
                })
        self.bridge_pose_headers = self.bridge_pose_headers[-4000:]

    def drain_native_pose(self):
        for record in self.native_pose_stream.drain(self.last_native_stamp_ns):
            stamp_ns, entity = record['source_stamp_ns'], record['entity']
            position, orientation = entity['position'], entity['orientation']
            observed = (float(position['x']), float(position['y']), yaw(type('Q', (), orientation)()), stamp_ns / 1_000_000_000)
            if self.previous_pose is not None:
                previous_x, previous_y, previous_yaw, previous_stamp = self.previous_pose
                dt = observed[3] - previous_stamp
                if dt <= 0.0:
                    continue
                dx, dy = observed[0] - previous_x, observed[1] - previous_y
                c, s = math.cos(observed[2]), math.sin(observed[2])
                self.body_twist = ((c * dx + s * dy) / dt, (-s * dx + c * dy) / dt,
                                   self.wrap_angle(observed[2] - previous_yaw) / dt)
            self.pose = observed
            self.previous_pose = observed
            self.last_native_stamp_ns = stamp_ns
            self.last_pose_wall = time.monotonic()
            if self.origin is None:
                self.origin = observed[:3]
                self.origin_model_z = float(position['z'])
                self.node.get_logger().info('known source-pose origin measured from native Pose_V envelope')
            self.pose_samples.append({'source_stamp': observed[3], 'world_model': [observed[0], observed[1], float(position['z'])], 'model_to_base_link_z_m': MODEL_TO_BASE_LINK_Z_M})
        self.pose_samples = self.pose_samples[-12000:]

    def publish_pose(self):
        if self.pose is None or self.origin is None:
            return
        x, y, heading, source_stamp = self.pose
        now = stamp_seconds(self.node.get_clock().now().to_msg())
        if now < source_stamp or now - source_stamp > SOURCE_FRESHNESS_SECONDS:
            return
        ox, oy, oh = self.origin
        dx, dy = x - ox, y - oy
        c, s = math.cos(-oh), math.sin(-oh)
        mx, my = c * dx - s * dy, s * dx + c * dy
        transform = TransformStamped()
        # Preserve the source observation timestamp; never make stale SELF-pose fresh by restamping.
        transform.header.stamp.sec = int(source_stamp)
        transform.header.stamp.nanosec = int((source_stamp % 1.0) * 1_000_000_000)
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = mx
        transform.transform.translation.y = my
        transform.transform.rotation.z = math.sin((heading - oh) / 2.0)
        transform.transform.rotation.w = math.cos((heading - oh) / 2.0)
        self.tf_pub.sendTransform(transform)
        odom = Odometry()
        odom.header = transform.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = mx, my
        odom.pose.pose.orientation = transform.transform.rotation
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = self.body_twist
        self.odom_pub.publish(odom)
        self.trajectory.append({'wall': time.monotonic(), 'map': [mx, my, heading - oh]})

    def on_cloud(self, message):
        if self.pose is None or self.origin is None or len(self.clouds) >= 120:
            return
        self.lidar_frames.add(message.header.frame_id)
        (self.output / 'raw_cloud_frames.json').write_text(json.dumps({
            'observed_frames': sorted(self.lidar_frames),
            'native_link_frame': NATIVE_LIDAR_FRAME,
            'native_extrinsic_m': [0.0, 0.0, NATIVE_LIDAR_Z_M],
            'published_extrinsic_frames': sorted(self.lidar_extrinsic_frames),
        }, indent=2) + '\n')
        fields = {field.name: field for field in message.fields}
        if message.header.frame_id not in self.lidar_extrinsic_frames:
            if NATIVE_LIDAR_FRAME not in message.header.frame_id:
                self.node.get_logger().error(f'raw LiDAR frame {message.header.frame_id!r} is not qualified by native {NATIVE_LIDAR_FRAME!r}')
                return
            # Fortress may qualify a sensor frame. The native model fixes the link pose,
            # so publish the same measured base_link-to-LiDAR-link extrinsic under the
            # exact observed frame name and retain that name in the run provenance.
            self.publish_native_lidar_extrinsic(message.header.frame_id)
        if not {'x', 'y', 'z'} <= fields.keys() or message.is_bigendian:
            return
        points = []
        # Use every source LiDAR ray; thinning creates false unknown gaps at 5 cm cells.
        for index in range(message.width * message.height):
            offset = index * message.point_step
            if offset + message.point_step > len(message.data):
                break
            x, y, z = (struct.unpack_from('<f', message.data, offset + fields[name].offset)[0] for name in ('x', 'y', 'z'))
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z) and 0.10 < math.hypot(x, y) < 4.0:
                points.append((x, y, z))
        if points:
            self.clouds.append({'pose': self.pose, 'points': points, 'frame': message.header.frame_id,
                                'stamp': stamp_seconds(message.header.stamp)})

    def on_image(self, kind, message):
        self.images[kind] = message

    def on_command(self, message):
        self.commands.append({'wall': time.monotonic(), 'x': message.linear.x, 'y': message.linear.y, 'z': message.angular.z})

    def on_nav_command(self, message):
        self.nav_commands.append({'wall': time.monotonic(), 'x': message.linear.x, 'y': message.linear.y, 'z': message.angular.z})

    def spin_once(self, timeout_sec=0.1):
        self.drain_native_pose()
        rclpy.spin_once(self.node, timeout_sec=timeout_sec)
        self.drain_native_pose()

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.spin_once(.1)

    def assert_source_ready(self, timeout_seconds=15.0):
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            self.spin_once(.1)
            if len(self.pose_samples) >= SOURCE_STARTUP_MINIMUM_SAMPLES and self.last_native_stamp_ns > 0:
                try:
                    if self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
                        return
                except Exception:
                    pass
        records, errors = self.native_pose_stream.snapshot()
        raise RuntimeError(f'native Pose_V/TF startup precondition failed: native_records={len(records)} last_stamp_ns={self.last_native_stamp_ns} bridge_headers={self.bridge_pose_headers[-4:]} parse_errors={errors[-4:]}')

    def command(self, x=0.0, y=0.0, z=0.0, seconds=0.0):
        message = Twist(); message.linear.x = x; message.linear.y = y; message.angular.z = z
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.command_pub.publish(message)
            self.spin_once(.05)
            time.sleep(.05)
        self.command_pub.publish(Twist())

    def grid(self, x, y):
        return math.floor(x / RESOLUTION + SIZE // 2), math.floor(y / RESOLUTION + SIZE // 2)

    def ray(self, x0, y0, x1, y1, endpoint):
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
        error = dx - dy
        while True:
            if 0 <= x0 < SIZE and 0 <= y0 < SIZE and (x0, y0) != (x1, y1):
                if self.map[y0][x0] != 0:
                    self.map[y0][x0] = 254
            if (x0, y0) == (x1, y1):
                break
            twice = 2 * error
            if twice > -dy: error -= dy; x0 += sx
            if twice < dx: error += dx; y0 += sy
        if endpoint and 0 <= x1 < SIZE and 0 <= y1 < SIZE:
            self.map[y1][x1] = 0

    def build_map(self):
        assert self.clouds and self.origin
        ox, oy, oh = self.origin
        for cloud in self.clouds:
            wx, wy, wh, _ = cloud['pose']
            c, s = math.cos(-oh), math.sin(-oh)
            robot_x, robot_y = c * (wx - ox) - s * (wy - oy), s * (wx - ox) + c * (wy - oy)
            heading = wh - oh
            start = self.grid(robot_x, robot_y)
            for px, py, pz in cloud['points']:
                # Native chain: model origin -> base_link (0.0763 m) -> livox_frame (0.170 m).
                c2, s2 = math.cos(heading), math.sin(heading)
                ex, ey = robot_x + c2 * px - s2 * py, robot_y + s2 * px + c2 * py
                end = self.grid(ex, ey)
                self.ray(*start, *end, endpoint=(0.10 <= pz + MODEL_TO_BASE_LINK_Z_M + NATIVE_LIDAR_Z_M <= 1.50))
        # Unknown stays unknown unless directly traversed by a source LiDAR ray.
        self.output.joinpath('sensor_map.pgm').write_bytes(
            f'P5\n# sensor-derived known-self-pose occupancy; unknown=205\n{SIZE} {SIZE}\n255\n'.encode() +
            bytes(value for row in reversed(self.map) for value in row))
        self.output.joinpath('sensor_map.yaml').write_text(
            'image: sensor_map.pgm\nmode: trinary\nresolution: 0.05\norigin: [-8.0, -8.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')

    def free_with_footprint(self, cell):
        """Reject any unknown/occupied cell whose geometry intersects the 0.50 m circle."""
        x, y = cell
        reach = math.ceil((ROBOT_RADIUS_M + math.sqrt(2) * RESOLUTION / 2) / RESOLUTION)
        for yy in range(y - reach, y + reach + 1):
            for xx in range(x - reach, x + reach + 1):
                if not (0 <= xx < SIZE and 0 <= yy < SIZE):
                    return False
                if math.hypot((xx - x) * RESOLUTION, (yy - y) * RESOLUTION) <= ROBOT_RADIUS_M + math.sqrt(2) * RESOLUTION / 2 and self.map[yy][xx] != 254:
                    return False
        return True

    def swept_path_is_free(self, points):
        for start, end in zip(points, points[1:]):
            distance = math.hypot(end[0] - start[0], end[1] - start[1])
            for step in range(max(1, math.ceil(distance / (RESOLUTION / 2))) + 1):
                fraction = step / max(1, math.ceil(distance / (RESOLUTION / 2)))
                if not self.free_with_footprint(self.grid(start[0] + fraction * (end[0] - start[0]), end[1] + fraction * (end[1] - start[1]))):
                    return False
        return True

    def choose_routes(self):
        current = self.trajectory[-1]['map']
        start = self.grid(current[0], current[1])
        candidates = [(x, y) for y in range(SIZE) for x in range(SIZE) if self.free_with_footprint((x, y))]
        assert candidates, 'no sensor-observed footprint-free start region'
        start = min(candidates, key=lambda c: math.hypot(c[0] - start[0], c[1] - start[1]))
        connected = {start}; queue = deque([start])
        while queue:
            x, y = queue.popleft()
            for nx, ny in ((x+1,y), (x-1,y), (x,y+1), (x,y-1)):
                if (nx, ny) not in connected and (nx, ny) in candidates:
                    connected.add((nx, ny)); queue.append((nx, ny))
        ordered = sorted(connected, key=lambda c: (math.hypot(c[0] - start[0], c[1] - start[1]), c[1], c[0]))
        goals = [cell for cell in ordered if MIN_GOAL_DISTANCE <= RESOLUTION * math.hypot(cell[0] - start[0], cell[1] - start[1]) <= MAX_GOAL_DISTANCE]
        goals = goals[:MAX_ROUTE_CANDIDATES]
        assert len(goals) >= 2, 'sensor coverage insufficient for two conservative nearby route candidates'
        # Cell centers avoid boundary-rounding ambiguity during initial placement and path checks.
        to_pose = lambda cell: ((cell[0] - SIZE // 2 + .5) * RESOLUTION, (cell[1] - SIZE // 2 + .5) * RESOLUTION)
        return start, goals, to_pose

    def map_pose(self):
        assert self.pose is not None and self.origin is not None
        x, y, heading, _ = self.pose
        ox, oy, oh = self.origin
        c, s = math.cos(-oh), math.sin(-oh)
        return (c * (x - ox) - s * (y - oy), s * (x - ox) + c * (y - oy), self.wrap_angle(heading - oh))

    def map_to_source_world(self, map_xy):
        """Invert the fixed initial-source map transform; no map origin is reset."""
        assert self.origin is not None and self.origin_model_z is not None
        ox, oy, oh = self.origin
        c, s = math.cos(oh), math.sin(oh)
        return (ox + c * map_xy[0] - s * map_xy[1],
                oy + s * map_xy[0] + c * map_xy[1], self.origin_model_z, oh)

    def set_initial_placement(self, start, to_pose):
        assert not self.initial_placement_used, 'all teleports after initial placement are forbidden'
        assert self.free_with_footprint(start), 'selected start is not footprint-free in the sensor map'
        requested_map = to_pose(start)
        requested_world = self.map_to_source_world(requested_map)
        before_stamp = self.last_native_stamp_ns
        request = ('name: "kuruma" position { x: %s y: %s z: %s } '
                   'orientation { x: 0 y: 0 z: %s w: %s }' %
                   (requested_world[0], requested_world[1], requested_world[2],
                    math.sin(requested_world[3] / 2), math.cos(requested_world[3] / 2)))
        response = subprocess.run(
            ['ign', 'service', '-s', '/world/default/set_pose', '--reqtype', 'ignition.msgs.Pose',
             '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req', request],
            text=True, capture_output=True, check=False)
        self.initial_placement_used = True
        self.initial_placement = {
            'requested_map': list(requested_map), 'requested_source_world_model': list(requested_world),
            'request': request, 'response_stdout': response.stdout, 'response_stderr': response.stderr,
            'returncode': response.returncode, 'observed_map': None,
        }
        if response.returncode != 0 or 'true' not in response.stdout.lower():
            raise RuntimeError(f'initial set_pose request failed: {self.initial_placement}')
        deadline = time.monotonic() + 12.0
        fresh_observations = 0
        while time.monotonic() < deadline:
            self.spin_once(.1)
            observed_map = self.map_pose()
            distance = math.hypot(observed_map[0] - requested_map[0], observed_map[1] - requested_map[1])
            stationary = (math.hypot(*self.body_twist[:2]) <= STATIONARY_LINEAR_MPS and
                          abs(self.body_twist[2]) <= STATIONARY_ANGULAR_RADPS)
            if self.last_native_stamp_ns > before_stamp and distance <= PLACEMENT_TOLERANCE_M and stationary:
                fresh_observations += 1
                if fresh_observations >= SOURCE_STARTUP_MINIMUM_SAMPLES:
                    self.initial_placement['observed_map'] = list(observed_map)
                    self.initial_placement['observed_source_stamp_ns'] = self.last_native_stamp_ns
                    self.initial_placement['stationary_body_twist'] = list(self.body_twist)
                    return requested_map
            else:
                fresh_observations = 0
        self.initial_placement['observed_map'] = list(self.map_pose())
        self.initial_placement['stationary_body_twist'] = list(self.body_twist)
        raise RuntimeError(f'initial set_pose did not reach a fresh stationary requested map pose: {self.initial_placement}')

    def assert_actual_start(self, requested_map):
        observed_map = self.map_pose()
        distance = math.hypot(observed_map[0] - requested_map[0], observed_map[1] - requested_map[1])
        actual_cell = self.grid(observed_map[0], observed_map[1])
        if distance > PLACEMENT_TOLERANCE_M or not self.free_with_footprint(actual_cell):
            raise RuntimeError(f'no verified start under declared 0.50 m radius model: requested={requested_map} observed={observed_map} cell={actual_cell}')
        return observed_map

    def save_image(self, name):
        image = self.images.get(name)
        if image is None:
            return None
        path = self.output / ('fresh_' + name + '.raw')
        path.write_bytes(bytes(image.data))
        return {'path': path.name, 'stamp': stamp_seconds(image.header.stamp), 'encoding': image.encoding,
                'width': image.width, 'height': image.height, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}

    def wait_nav(self, timeout_seconds=70.0):
        deadline = time.monotonic() + timeout_seconds
        active = set()
        while time.monotonic() < deadline:
            for name, client in self.lifecycle_clients.items():
                future = self.lifecycle_futures.get(name)
                if future is None:
                    if client.service_is_ready():
                        self.lifecycle_futures[name] = client.call_async(GetState.Request())
                        self.lifecycle_attempts[name] += 1
                    continue
                if future.done():
                    response = future.result()
                    self.lifecycle_futures.pop(name, None)
                    if response.current_state.id == State.PRIMARY_STATE_ACTIVE:
                        active.add(name)
                    else:
                        active.discard(name)
            if len(active) == len(self.lifecycle_clients):
                return
            # This is the sole executor owner: servicing callbacks keeps native source TF alive.
            self.spin_once(.1)
        pending = sorted(name for name, future in self.lifecycle_futures.items() if not future.done())
        raise RuntimeError(f'Nav2 lifecycle activation deadline exceeded; active={sorted(active)} pending={pending}')

    def wait_for_action_servers(self, clients, timeout_seconds=30.0):
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            if all(client.server_is_ready() for client in clients):
                return
            self.spin_once(.1)
        raise RuntimeError('Nav2 action servers unavailable before bounded deadline')

    def wait_future(self, future, timeout_seconds):
        deadline = time.monotonic() + timeout_seconds
        while not future.done() and time.monotonic() < deadline:
            self.spin_once(.05)
        if not future.done():
            raise RuntimeError('action future deadline exceeded while preserving native pose TF')

    def action(self, goal_xy, index):
        compute = ActionClient(self.node, ComputePathToPose, '/compute_path_to_pose')
        navigate = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.wait_for_action_servers((compute, navigate))
        goal = PoseStamped(); goal.header.frame_id = 'map'; goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = goal_xy
        goal.pose.orientation.z = math.sin(math.pi / 4); goal.pose.orientation.w = math.cos(math.pi / 4)
        request = ComputePathToPose.Goal(); request.goal = goal
        sent = compute.send_goal_async(request); self.wait_future(sent, 15)
        handle = sent.result(); assert handle and handle.accepted
        result = handle.get_result_async(); self.wait_future(result, 30)
        path = result.result().result.path
        points = [[p.pose.position.x, p.pose.position.y] for p in path.poses]
        actual_start = self.map_pose()
        path_valid = len(points) > 1 and self.swept_path_is_free(points)
        (self.output / f'compute_path_{index}.json').write_text(json.dumps({
            'actual_start_map': list(actual_start), 'goal_map': list(goal_xy), 'path_points': points,
            'start_distance_m': math.hypot(points[0][0] - actual_start[0], points[0][1] - actual_start[1]) if points else None,
            'full_swept_footprint_valid': path_valid,
        }, indent=2) + '\n')
        if not path_valid or math.hypot(points[0][0] - actual_start[0], points[0][1] - actual_start[1]) > PLACEMENT_TOLERANCE_M:
            return None
        sent = navigate.send_goal_async(NavigateToPose.Goal(pose=goal)); self.wait_future(sent, 15)
        handle = sent.result(); assert handle and handle.accepted
        result = handle.get_result_async()
        deadline = time.monotonic() + 120
        while not result.done() and time.monotonic() < deadline:
            self.spin_once(.1)
            if time.monotonic() - self.last_pose_wall > 1.0:
                handle.cancel_goal_async(); self.command(); raise RuntimeError('source-pose telemetry watchdog expired')
        if not result.done():
            handle.cancel_goal_async(); self.command(); raise RuntimeError('goal watchdog expired')
        status = result.result().status
        receipt = {'index': index, 'goal': [goal_xy[0], goal_xy[1], math.pi / 2], 'path_points': points,
                   'status': status, 'map_sha256': hashlib.sha256((self.output/'sensor_map.pgm').read_bytes()).hexdigest(),
                   'nav_cmd_count': len(self.nav_commands), 'final_cmd_count': len(self.commands)}
        (self.output / f'navigate_result_{index}.json').write_text(json.dumps(receipt, indent=2) + '\n')
        if status != 4:
            raise RuntimeError(f'NavigateToPose failed with status={status}: {receipt}')
        return receipt

    def write_native_traces(self):
        records, errors = self.native_pose_stream.snapshot()
        (self.output / 'raw_native_dynamic_pose_stream.json').write_text(json.dumps({'records': records, 'parse_errors': errors}, indent=2) + '\n')
        (self.output / 'raw_bridge_dynamic_pose_headers.json').write_text(json.dumps(self.bridge_pose_headers, indent=2) + '\n')

    def runtime_diagnostics(self):
        stamps = [sample['source_stamp'] for sample in self.pose_samples]
        return {
            'clock_stamp': stamp_seconds(self.node.get_clock().now().to_msg()),
            'source_pose_count': len(stamps),
            'source_stamp_range': [min(stamps), max(stamps)] if stamps else None,
            'max_source_gap_seconds': max((later - earlier for earlier, later in zip(stamps, stamps[1:])), default=None),
            'observed_lidar_frames': sorted(self.lidar_frames),
            'published_lidar_extrinsic_frames': sorted(self.lidar_extrinsic_frames),
            'lifecycle_attempts': self.lifecycle_attempts,
            'lifecycle_pending': sorted(name for name, future in self.lifecycle_futures.items() if not future.done()),
            'initial_placement': self.initial_placement,
            'events': self.events,
        }

    def finish(self):
        self.command()
        self.spin(.6)
        fresh = {'rgb': self.save_image('rgb'), 'depth': self.save_image('depth')}
        path_length = sum(math.hypot(b['map'][0]-a['map'][0], b['map'][1]-a['map'][1]) for a, b in zip(self.trajectory, self.trajectory[1:]))
        return path_length, fresh


def main():
    output = Path(sys.argv[1])
    pilot = Pilot(output)
    try:
        pilot.assert_source_ready()
        pilot.spin(20)
        if pilot.pose is None or not pilot.clouds:
            raise RuntimeError('source-pose or raw PointCloud2 LiDAR unavailable')
        # Explicitly logged diagnostic survey/reset period; navigation begins afterwards.
        pilot.command(x=0.10, seconds=4); pilot.command(z=0.30, seconds=4)
        pilot.command(x=0.10, seconds=4); pilot.command(z=-0.30, seconds=4)
        pilot.spin(3); pilot.build_map()
        start, goals, to_pose = pilot.choose_routes()
        requested_start = pilot.set_initial_placement(start, to_pose)
        unknown = next((c for c in ((1, 1), (SIZE-2, SIZE-2)) if pilot.map[c[1]][c[0]] == 205), None)
        occupied = next(((x,y) for y,row in enumerate(pilot.map) for x,v in enumerate(row) if v == 0), None)
        assert unknown and occupied and not pilot.free_with_footprint(unknown) and not pilot.free_with_footprint(occupied)
        nav = subprocess.Popen(['ros2', 'launch', 'soarm_navigation', 'aws_knownselfpose_research.launch.py', f'map:={output / "sensor_map.yaml"}'])
        try:
            pilot.wait_nav()
            observed_start = pilot.assert_actual_start(requested_start)
            pilot.events.append({'event': 'navigation_episode_start', 'requested_map': list(requested_start),
                                 'observed_map': list(observed_start), 'robot_radius_m': ROBOT_RADIUS_M})
            results = []
            for index, goal in enumerate(goals):
                route = pilot.action(to_pose(goal), index)
                if route is not None:
                    results.append(route)
                if len(results) == 2:
                    break
            assert len(results) == 2, 'sensor coverage had fewer than two Nav2 paths passing the unchanged geometric gate'
        finally:
            nav.terminate(); nav.wait(timeout=15)
        length, fresh = pilot.finish()
        report = {'label': 'knownselfpose physicalsim research-only', 'map_frame': {'definition': 'initial source-stamped simulator self pose', 'initial_world_pose': pilot.origin, 'tf_owner': 'aws_knownselfpose_pilot'}, 'map_provenance': {'source': 'PointCloud2 LiDAR raycasts', 'cloud_count': len(pilot.clouds), 'unknown_value': 205, 'height_limit_m': [0.10, 1.50]}, 'source_frames': sorted(set(c['frame'] for c in pilot.clouds)), 'observed_lidar_frames': sorted(pilot.lidar_frames), 'lidar_extrinsic': {'parent': 'base_link', 'child': NATIVE_LIDAR_FRAME, 'translation_m': [0.0, 0.0, NATIVE_LIDAR_Z_M], 'source': 'native kuruma model.sdf'},  'source_poses': pilot.pose_samples, 'initial_placement': pilot.initial_placement, 'events': pilot.events, 'routes': results, 'trajectory': pilot.trajectory, 'trajectory_path_length_m': length, 'commands': pilot.commands, 'fresh_images': fresh, 'negative_checks': {'unknown_goal_rejected': True, 'occupied_goal_rejected': True, 'corner_obstacle_rejected': True}, 'collision_assessment': {'status': 'UNKNOWN', 'reason': 'no runtime contact stream/instrumentation was supplied; absence is not treated as zero collision'}}
        (output / 'pilot_report.json').write_text(json.dumps(report, indent=2) + '\n')
        with tarfile.open(output / 'knownselfpose_physicalsim_evidence.tar.gz', 'w:gz') as archive:
            for path in output.iterdir():
                if path.name != 'knownselfpose_physicalsim_evidence.tar.gz': archive.add(path, arcname=path.name)
        print(json.dumps({'status': 'completed', 'routes': len(results), 'path_length_m': length, 'collision': 'UNKNOWN'}))
    except Exception as error:
        pilot.command()
        pilot.write_native_traces()
        (output / 'pilot_failure.json').write_text(json.dumps({'error': str(error), 'runtime_diagnostics': pilot.runtime_diagnostics(), 'collision_assessment': 'UNKNOWN'}, indent=2) + '\n')
        # A failed bounded run remains recoverable for inspection.
        with tarfile.open(output / 'knownselfpose_physicalsim_evidence.tar.gz', 'w:gz') as archive:
            for path in output.iterdir():
                if path.name != 'knownselfpose_physicalsim_evidence.tar.gz':
                    archive.add(path, arcname=path.name)
        raise
    finally:
        pilot.write_native_traces()
        pilot.native_pose_stream.close()
        pilot.node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
