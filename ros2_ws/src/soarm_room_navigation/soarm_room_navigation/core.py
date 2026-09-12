"""Transport-neutral, fail-closed language room navigation model."""
import base64
import hashlib
import json
import math
import re
import os
import tempfile
from dataclasses import asdict, dataclass
from enum import Enum
from pathlib import Path


class NavigationError(Exception):
    def __init__(self, code, message):
        self.code, self.message = code, message
        super().__init__(message)


class TaskState(str, Enum):
    IDLE = 'IDLE'
    PLANNING = 'PLANNING'
    CLARIFICATION = 'CLARIFICATION'
    PROPOSAL = 'PROPOSAL'
    CONFIRMATION = 'CONFIRMATION'
    RUNNING_NAV = 'RUNNING_NAV'
    RUNNING_WAIT = 'RUNNING_WAIT'
    SETTLING = 'SETTLING'
    FAILURE_AWAITING_INPUT = 'FAILURE_AWAITING_INPUT'
    SAFETY_BLOCKED = 'SAFETY_BLOCKED'
    CANCELING = 'CANCELING'
    CANCELED = 'CANCELED'
    SUCCESS = 'SUCCESS'


TERMINAL = {TaskState.CANCELED, TaskState.SUCCESS, TaskState.FAILURE_AWAITING_INPUT,
            TaskState.SAFETY_BLOCKED}
MAX_ACTIONS, MAX_WAIT_SECONDS = 8, 60


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_pgm(path):
    raw, tokens, offset = Path(path).read_bytes(), [], 0
    while len(tokens) < 4:
        while offset < len(raw) and raw[offset:offset + 1].isspace():
            offset += 1
        if offset >= len(raw):
            raise ValueError('truncated PGM header')
        if raw[offset:offset + 1] == b'#':
            offset = raw.find(b'\n', offset) + 1
            continue
        end = offset
        while end < len(raw) and not raw[end:end + 1].isspace():
            end += 1
        tokens.append(raw[offset:end])
        offset = end
    if tokens[0] != b'P5' or int(tokens[3]) != 255:
        raise ValueError('expected binary P5 PGM with max value 255')
    if offset < len(raw) and raw[offset:offset + 1].isspace():
        offset += 1
    width, height = int(tokens[1]), int(tokens[2])
    pixels = raw[offset:]
    if width <= 0 or height <= 0 or len(pixels) != width * height:
        raise ValueError('invalid PGM dimensions')
    return width, height, pixels


def _yaml_values(path):
    values = {}
    for line in Path(path).read_text().splitlines():
        if ':' in line and not line.lstrip().startswith('#'):
            key, value = line.split(':', 1)
            values[key.strip()] = value.strip().strip('"\'')
    required = {'image', 'resolution', 'origin'}
    if not required <= values.keys():
        raise ValueError('occupancy YAML missing required map fields')
    return values


@dataclass(frozen=True)
class MapBundle:
    yaml_path: str
    image_path: str
    pcd_path: str
    expected_health_map_id: str
    resolution: float
    origin: tuple
    width: int
    height: int
    pixels: bytes
    yaml_sha256: str
    image_sha256: str
    pcd_sha256: str
    bundle_id: str
    negate: int = 0
    free_thresh: float = .25

    @classmethod
    def load(cls, yaml_path, pcd_path, expected_health_map_id=None,
             alignment='house-origin-v1:T_map_camera_init=identity'):
        values = _yaml_values(yaml_path)
        image = Path(yaml_path).parent / values['image']
        width, height, pixels = load_pgm(image)
        origin = tuple(float(value.strip()) for value in values['origin'].strip('[]').split(','))
        if len(origin) != 3 or not all(math.isfinite(value) for value in origin):
            raise ValueError('map origin must be a finite [x, y, yaw] triple')
        resolution = float(values['resolution'])
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('map resolution must be finite and positive')
        pcd = Path(pcd_path)
        if not pcd.is_file() or pcd.stat().st_size == 0:
            raise ValueError('localization PCD is missing or empty')
        manifest = {
            'occupancy_yaml_sha256': digest(yaml_path),
            'occupancy_image_sha256': digest(image),
            'localization_pcd_sha256': digest(pcd),
            'frame_id': 'map',
            'map_origin': origin,
            'resolution': resolution,
            'size': [width, height],
            'alignment': alignment,
        }
        bundle_id = hashlib.sha256(json.dumps(manifest, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
        if expected_health_map_id and expected_health_map_id != bundle_id:
            raise ValueError('configured health map ID must be the computed immutable bundle digest')
        return cls(str(yaml_path), str(image), str(pcd), bundle_id, resolution, origin,
                   width, height, pixels, manifest['occupancy_yaml_sha256'],
                   manifest['occupancy_image_sha256'], manifest['localization_pcd_sha256'], bundle_id,
                   int(values.get('negate', '0')), float(values.get('free_thresh', '.25')))

    def _local(self, x, y):
        dx, dy, yaw = x - self.origin[0], y - self.origin[1], self.origin[2]
        return math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy

    def world_to_cell(self, x, y):
        local_x, local_y = self._local(x, y)
        return math.floor(local_x / self.resolution), math.floor(local_y / self.resolution)

    def cell_to_world(self, cell_x, cell_y):
        local_x, local_y, yaw = (cell_x + .5) * self.resolution, (cell_y + .5) * self.resolution, self.origin[2]
        return (self.origin[0] + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
                self.origin[1] + math.sin(yaw) * local_x + math.cos(yaw) * local_y)

    def in_bounds(self, x, y):
        cell_x, cell_y = self.world_to_cell(x, y)
        return 0 <= cell_x < self.width and 0 <= cell_y < self.height

    def free(self, x, y):
        cell_x, cell_y = self.world_to_cell(x, y)
        if not (0 <= cell_x < self.width and 0 <= cell_y < self.height):
            return False
        pixel = self.pixels[(self.height - 1 - cell_y) * self.width + cell_x]
        occupied = (pixel if self.negate else 255 - pixel) / 255.0
        return occupied <= self.free_thresh

    def footprint_free(self, x, y, yaw=0.0, half_side=.30, margin=.05):
        if not all(math.isfinite(value) for value in (x, y, yaw, half_side, margin)) or half_side <= 0 or margin < 0:
            return False
        extent, step = half_side + margin, self.resolution / 2
        return all(self.free(x + math.cos(yaw) * ix * step - math.sin(yaw) * iy * step,
                             y + math.sin(yaw) * ix * step + math.cos(yaw) * iy * step)
                   for ix in range(math.ceil(-extent / step), math.floor(extent / step) + 1)
                   for iy in range(math.ceil(-extent / step), math.floor(extent / step) + 1))

    def footprint_free_any_yaw(self, x, y, half_side=.30, margin=.05):
        """Conservatively clear the circumscribed footprint for any final yaw."""
        if not all(math.isfinite(value) for value in (x, y, half_side, margin)) or half_side <= 0 or margin < 0:
            return False
        radius = math.sqrt(2) * (half_side + margin)
        local_x, local_y = self._local(x, y)
        # Check every cell intersecting the circle, not sampled points which
        # can miss a small occupied corner depending on floating-point rounding.
        for cell_y in range(math.floor((local_y - radius) / self.resolution),
                            math.floor((local_y + radius) / self.resolution) + 1):
            for cell_x in range(math.floor((local_x - radius) / self.resolution),
                                math.floor((local_x + radius) / self.resolution) + 1):
                near_x = max(cell_x * self.resolution, min(local_x, (cell_x + 1) * self.resolution))
                near_y = max(cell_y * self.resolution, min(local_y, (cell_y + 1) * self.resolution))
                if (local_x - near_x) ** 2 + (local_y - near_y) ** 2 <= radius ** 2 + 1e-12:
                    if not (0 <= cell_x < self.width and 0 <= cell_y < self.height):
                        return False
                    pixel = self.pixels[(self.height - 1 - cell_y) * self.width + cell_x]
                    if (pixel if self.negate else 255 - pixel) / 255.0 > self.free_thresh:
                        return False
        return True

    def safe_polygon_goal(self, polygon, require_yaw=False, yaw=0.0):
        """Resolve an automatic goal to the centroid or nearest safe map cell."""
        if not _valid_polygon(polygon, self):
            raise NavigationError('INVALID_ROOM', 'room polygon must be finite, simple, nonzero-area, and inside the map')
        if not isinstance(yaw, (int, float)) or isinstance(yaw, bool) or not math.isfinite(yaw):
            raise NavigationError('INVALID_ROOM', 'fixed heading must be finite')
        resolved_yaw = yaw if require_yaw else 0.0
        edges = list(zip(polygon, polygon[1:] + polygon[:1]))
        twice_area = sum(a[0] * b[1] - b[0] * a[1] for a, b in edges)
        centroid = (sum((a[0] + b[0]) * (a[0] * b[1] - b[0] * a[1]) for a, b in edges) / (3 * twice_area),
                    sum((a[1] + b[1]) * (a[0] * b[1] - b[0] * a[1]) for a, b in edges) / (3 * twice_area))
        safe = (lambda point: self.footprint_free(*point, resolved_yaw) if require_yaw else self.footprint_free_any_yaw(*point))
        if _point_in_polygon(centroid, polygon) and safe(centroid):
            return centroid + (resolved_yaw,)
        # Bounds must be computed in map-cell axes, including rotated origins.
        cells = [self.world_to_cell(*point) for point in polygon]
        lower_x, upper_x = max(0, min(p[0] for p in cells)), min(self.width - 1, max(p[0] for p in cells))
        lower_y, upper_y = max(0, min(p[1] for p in cells)), min(self.height - 1, max(p[1] for p in cells))
        if (upper_x - lower_x + 1) * (upper_y - lower_y + 1) > 100000:
            raise NavigationError('GOAL_SEARCH_LIMIT', 'automatic search exceeds the bounded map area; choose a manual goal')
        candidates = []
        for cell_y in range(lower_y, upper_y + 1):
            for cell_x in range(lower_x, upper_x + 1):
                point = self.cell_to_world(cell_x, cell_y)
                if _point_in_polygon(point, polygon) and self.free(*point):
                    candidates.append(((point[0] - centroid[0]) ** 2 + (point[1] - centroid[1]) ** 2, cell_y, cell_x, point))
        # Check closest candidates first instead of testing every full footprint.
        for _, _, _, point in sorted(candidates):
            if safe(point):
                return point + (resolved_yaw,)
        raise NavigationError('NO_SAFE_GOAL', 'room has no safe in-polygon goal with full footprint clearance')

    def intact(self):
        try:
            return (digest(self.yaml_path) == self.yaml_sha256 and digest(self.image_path) == self.image_sha256 and
                    digest(self.pcd_path) == self.pcd_sha256)
        except OSError:
            return False

    def map_payload(self):
        return {'bundle_id': self.bundle_id, 'resolution': self.resolution, 'origin': self.origin,
                'width': self.width, 'height': self.height,
                'pixels_b64': base64.b64encode(self.pixels).decode('ascii')}


def _segments_intersect(a, b, c, d):
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    return orient(a, b, c) * orient(a, b, d) <= 0 and orient(c, d, a) * orient(c, d, b) <= 0


def _valid_polygon(polygon, bundle):
    if len(polygon) < 3 or len(polygon) > 64 or len(set(polygon)) != len(polygon):
        return False
    if not all(len(point) == 2 and all(math.isfinite(value) for value in point) and bundle.in_bounds(*point)
               for point in polygon):
        return False
    edges = list(zip(polygon, polygon[1:] + polygon[:1]))
    if abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in edges)) < 1e-9:
        return False
    return not any(_segments_intersect(*edge, *other)
                   for index, edge in enumerate(edges)
                   for other_index, other in enumerate(edges)
                   if index != other_index and abs(index - other_index) not in (1, len(edges) - 1))


@dataclass(frozen=True)
class Room:
    id: str
    aliases: tuple
    polygon: tuple
    goal_pose: tuple
    revision: int = 1
    goal_mode: str = 'manual'
    require_yaw: bool = True

    def valid(self, bundle):
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,48}', self.id) or not self.aliases or len(self.aliases) > 16:
            return False
        if not all(isinstance(alias, str) and alias.strip() and len(alias) <= 64 for alias in self.aliases):
            return False
        if self.goal_mode not in ('auto', 'manual') or not isinstance(self.require_yaw, bool) or not _valid_polygon(self.polygon, bundle):
            return False
        if len(self.goal_pose) != 3 or not all(math.isfinite(value) for value in self.goal_pose):
            return False
        safe = (bundle.footprint_free(*self.goal_pose) if self.require_yaw else
                bundle.footprint_free_any_yaw(*self.goal_pose[:2]))
        return _point_in_polygon(self.goal_pose[:2], self.polygon) and safe


def _point_in_polygon(point, polygon):
    x, y = point
    inside = False
    for first, second in zip(polygon, polygon[1:] + polygon[:1]):
        if (first[1] > y) != (second[1] > y):
            inside ^= x < (second[0] - first[0]) * (y - first[1]) / (second[1] - first[1]) + first[0]
    return inside


class RoomStore:
    @staticmethod
    def save(path, bundle, rooms, revision):
        payload = {'bundle_id': bundle.bundle_id, 'room_revision': revision,
                   'rooms': [asdict(room) for room in rooms]}
        target = Path(path)
        encoded = json.dumps(payload, ensure_ascii=False, sort_keys=True).encode('utf-8')
        descriptor = None
        temporary = None
        try:
            target.parent.mkdir(parents=True, exist_ok=True)
            descriptor, temporary = tempfile.mkstemp(prefix=target.name + '.', suffix='.tmp', dir=target.parent)
            with os.fdopen(descriptor, 'wb') as output:
                descriptor = None
                output.write(encoded)
                output.flush()
                os.fsync(output.fileno())
            os.replace(temporary, target)
            temporary = None
        except OSError as error:
            raise NavigationError('PERSISTENCE_FAILED', 'could not atomically save room catalog') from error
        finally:
            if descriptor is not None:
                os.close(descriptor)
            if temporary is not None:
                try:
                    os.unlink(temporary)
                except OSError:
                    pass

    @staticmethod
    def load(path, bundle):
        data = json.loads(Path(path).read_text())
        if data.get('bundle_id') != bundle.bundle_id:
            raise NavigationError('MAP_MISMATCH', 'semantic rooms belong to a different immutable map bundle')
        rooms = [Room(item['id'], tuple(item['aliases']), tuple(tuple(p) for p in item['polygon']),
                      tuple(item['goal_pose']), int(item.get('revision', 1)),
                      item.get('goal_mode', 'manual'), item.get('require_yaw', True)) for item in data.get('rooms', [])]
        aliases = [alias for room in rooms for alias in room.aliases]
        if len({room.id for room in rooms}) != len(rooms) or len(set(aliases)) != len(aliases) or not all(room.valid(bundle) for room in rooms):
            raise NavigationError('INVALID_ROOM', 'invalid room aliases, geometry, or goal footprint')
        return rooms, int(data.get('room_revision', 1))


@dataclass(frozen=True)
class PlanAction:
    kind: str
    room_id: str = ''
    fallback_room_id: str = ''
    seconds: int = 0


@dataclass(frozen=True)
class Plan:
    proposal_id: str
    actions: tuple
    summary: str
    bundle_id: str
    room_revision: int


def parse_plan(payload, rooms, bundle, proposal_id, room_revision):
    if not isinstance(payload, dict) or set(payload) - {'summary', 'actions', 'clarification'}:
        raise NavigationError('PLANNER_SCHEMA', 'planner returned unsupported fields')
    if 'clarification' in payload:
        if set(payload) != {'clarification'} or not isinstance(payload['clarification'], str) or not payload['clarification'].strip() or len(payload['clarification']) > 240:
            raise NavigationError('PLANNER_SCHEMA', 'invalid clarification')
        return payload['clarification']
    actions = payload.get('actions')
    if not isinstance(actions, list) or not 1 <= len(actions) <= MAX_ACTIONS:
        raise NavigationError('PLANNER_SCHEMA', 'plan requires one to eight actions')
    room_ids = {room.id for room in rooms}
    result = []
    for action in actions:
        if not isinstance(action, dict):
            raise NavigationError('PLANNER_SCHEMA', 'planner action has unsupported fields')
        kind = action.get('action')
        if kind == 'navigate':
            if set(action) not in ({'action', 'room_id'}, {'action', 'room_id', 'fallback_room_id'}):
                raise NavigationError('PLANNER_SCHEMA', 'navigate fields are invalid')
            room_id, fallback = action.get('room_id', ''), action.get('fallback_room_id', '')
            if room_id not in room_ids or (fallback and fallback not in room_ids) or fallback == room_id:
                raise NavigationError('PLANNER_SCHEMA', 'unknown or invalid navigation room')
            result.append(PlanAction(kind, room_id, fallback))
        elif kind == 'wait':
            if set(action) != {'action', 'seconds'}:
                raise NavigationError('PLANNER_SCHEMA', 'wait fields are invalid')
            seconds = action.get('seconds')
            if isinstance(seconds, bool) or not isinstance(seconds, int) or not 1 <= seconds <= MAX_WAIT_SECONDS:
                raise NavigationError('PLANNER_SCHEMA', 'wait must be a bounded integer')
            result.append(PlanAction(kind, seconds=seconds))
        elif kind == 'return_to_start':
            if set(action) not in ({'action'}, {'action', 'fallback_room_id'}):
                raise NavigationError('PLANNER_SCHEMA', 'return fields are invalid')
            fallback = action.get('fallback_room_id', '')
            if fallback and fallback not in room_ids:
                raise NavigationError('PLANNER_SCHEMA', 'unknown return fallback room')
            result.append(PlanAction(kind, fallback_room_id=fallback))
        else:
            raise NavigationError('PLANNER_SCHEMA', 'unsupported action')
    summary = payload.get('summary', '')
    if not isinstance(summary, str) or len(summary) > 240:
        raise NavigationError('PLANNER_SCHEMA', 'invalid plan summary')
    canonical = json.dumps({'actions': [action.__dict__ for action in result], 'summary': summary,
                            'bundle_id': bundle.bundle_id, 'room_revision': room_revision},
                           sort_keys=True, separators=(',', ':'), ensure_ascii=True)
    return Plan(hashlib.sha256(canonical.encode()).hexdigest(), tuple(result), summary, bundle.bundle_id, room_revision)
