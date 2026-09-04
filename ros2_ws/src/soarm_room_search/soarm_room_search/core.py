"""Fail-closed domain model and transport-neutral room-search controller."""
import hashlib
import json
import math
import re
import shutil
import subprocess
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

class State(str, Enum):
    EMPTY = 'EMPTY'; INTENT_PROPOSED = 'INTENT_PROPOSED'; INTENT_CONFIRMED = 'INTENT_CONFIRMED'
    NAVIGATING_TO_SCAN = 'NAVIGATING_TO_SCAN'; AT_SCAN_POSE = 'AT_SCAN_POSE'; SCANNING = 'SCANNING'
    SCAN_READY = 'SCAN_READY'; ANALYZING = 'ANALYZING'; DESK_READY = 'DESK_READY'; DOCK_PREVIEW = 'DOCK_PREVIEW'
    DOCK_CONFIRMED = 'DOCK_CONFIRMED'; NAVIGATING_TO_DOCK = 'NAVIGATING_TO_DOCK'; ARRIVED = 'ARRIVED'
    FAILED = 'FAILED'; CANCELED = 'CANCELED'; BLOCKED = 'BLOCKED'

class WorkflowError(Exception):
    def __init__(self, code, message):
        self.code = code
        self.message = message
        super().__init__(message)

def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def load_pgm(path):
    raw = Path(path).read_bytes()
    tokens, offset = [], 0
    while len(tokens) < 4:
        while offset < len(raw) and raw[offset:offset + 1].isspace(): offset += 1
        if raw[offset:offset + 1] == b'#':
            offset = raw.find(b'\n', offset) + 1
            continue
        end = offset
        while end < len(raw) and not raw[end:end + 1].isspace(): end += 1
        tokens.append(raw[offset:end]); offset = end
    if tokens[0] != b'P5' or int(tokens[3]) != 255: raise ValueError('expected P5 max 255 PGM')
    if offset < len(raw) and raw[offset:offset + 1].isspace(): offset += 1
    width, height = int(tokens[1]), int(tokens[2]); pixels = raw[offset:]
    if len(pixels) != width * height: raise ValueError('invalid PGM dimensions')
    return width, height, pixels

@dataclass
class MapBundle:
    yaml_path: str; image_path: str; map_id: str; resolution: float; origin: tuple; width: int; height: int; pixels: bytes; yaml_sha256: str; image_sha256: str
    negate: int = 0; occupied_thresh: float = .65; free_thresh: float = .25; revision: int = 1
    @classmethod
    def load(cls, yaml_path, map_id):
        values = {}
        for line in Path(yaml_path).read_text().splitlines():
            if ':' in line and not line.lstrip().startswith('#'):
                key, value = line.split(':', 1); values[key.strip()] = value.strip().strip('"\'')
        image = Path(yaml_path).parent / values['image']; width, height, pixels = load_pgm(image)
        origin = tuple(float(v.strip()) for v in values['origin'].strip('[]').split(','))
        return cls(str(yaml_path), str(image), map_id, float(values['resolution']), origin, width, height, pixels, sha256(yaml_path), sha256(image), int(values.get('negate', '0')), float(values.get('occupied_thresh', '.65')), float(values.get('free_thresh', '.25')))
    def world_to_cell(self, x, y):
        return math.floor((x - self.origin[0]) / self.resolution), math.floor((y - self.origin[1]) / self.resolution)
    def free(self, x, y):
        cell_x, cell_y = self.world_to_cell(x, y)
        if not (0 <= cell_x < self.width and 0 <= cell_y < self.height): return False
        pixel = self.pixels[(self.height - 1 - cell_y) * self.width + cell_x]
        occupied = (pixel if self.negate else 255 - pixel) / 255.0
        return occupied <= self.free_thresh
    def footprint_free(self, x, y, radius=.30, margin=.05):
        step, extent = self.resolution / 2, radius + margin
        for ix in range(math.ceil(-extent / step), math.floor(extent / step) + 1):
            for iy in range(math.ceil(-extent / step), math.floor(extent / step) + 1):
                if ix * ix + iy * iy <= (extent / step) ** 2 and not self.free(x + ix * step, y + iy * step): return False
        return True

@dataclass
class Room:
    id: str; aliases: list; polygon: list; scan_pose: tuple
    def contains(self, point):
        x, y = point[:2]; inside = False
        for first, second in zip(self.polygon, self.polygon[1:] + self.polygon[:1]):
            if (first[1] > y) != (second[1] > y) and x < (second[0] - first[0]) * (y - first[1]) / (second[1] - first[1]) + first[0]: inside = not inside
        return inside
    def valid(self, bundle):
        if not self.id.strip() or not self.aliases or any(not alias.strip() for alias in self.aliases): return False
        if len(self.polygon) < 3 or len(set(map(tuple, self.polygon))) != len(self.polygon): return False
        area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(self.polygon, self.polygon[1:] + self.polygon[:1]))
        if abs(area) < 1e-9 or not math.isfinite(self.scan_pose[2]): return False
        if not all(bundle.free(x, y) for x, y in self.polygon): return False
        return self.contains(self.scan_pose) and bundle.footprint_free(*self.scan_pose[:2])

def parse_intent(text, rooms):
    match = re.fullmatch(r'\s*去\s*(.+?)\s*找书桌\s*', text)
    if not match: raise WorkflowError('INTENT_REJECTED', 'only 去 <房间> 找书桌 is permitted')
    matched = [room for room in rooms if match.group(1) in room.aliases]
    if len(matched) != 1: raise WorkflowError('INTENT_REJECTED', 'unknown or ambiguous room alias')
    return {'room_id': matched[0].id, 'text': text}

def scan_complete(samples, max_center_drift=.10, sectors=8):
    if len(samples) < 2 or any(len(sample) != 4 for sample in samples): return False
    origin_x, origin_y, _, previous = samples[0]; total, seen = 0., set()
    for x, y, _, yaw in samples:
        if math.hypot(x - origin_x, y - origin_y) > max_center_drift: return False
        total += (yaw - previous + math.pi) % (2 * math.pi) - math.pi; previous = yaw
        seen.add(int((yaw % (2 * math.pi)) / (2 * math.pi / sectors)))
    return abs(total) >= 2 * math.pi - .25 and len(seen) == sectors

def desk_hypotheses(observations):
    return [item for item in observations if item.get('quality', 0) >= .6 and .5 <= item.get('height', item.get('z', 0)) <= 1.2 and item.get('width', 0) >= .5]

@dataclass
class Workflow:
    state: State = State.EMPTY; bundle: MapBundle = None; rooms: list = field(default_factory=list); proposal: dict = None
    scan_revision: int = 0; detection_revision: int = 0; candidates: list = field(default_factory=list); desks: list = field(default_factory=list); selected: dict = None; reason: str = ''
    def need(self, *states):
        if self.state not in states: raise WorkflowError('INVALID_STATE', 'operation is not permitted in ' + self.state.value)
    def load(self, bundle, rooms):
        aliases = [alias for room in rooms for alias in room.aliases]
        if len({room.id for room in rooms}) != len(rooms) or len(set(aliases)) != len(aliases) or not all(room.valid(bundle) for room in rooms): raise WorkflowError('INVALID_ROOM', 'invalid room geometry, scan pose, id, or alias')
        self.bundle, self.rooms, self.state = bundle, rooms, State.EMPTY
    def propose(self, text): self.need(State.EMPTY, State.CANCELED, State.FAILED, State.BLOCKED); self.proposal = parse_intent(text, self.rooms); self.state = State.INTENT_PROPOSED
    def confirm_intent(self): self.need(State.INTENT_PROPOSED); self.state = State.INTENT_CONFIRMED
    def scan_goal(self): self.need(State.INTENT_CONFIRMED); self.state = State.NAVIGATING_TO_SCAN
    def at_scan(self): self.need(State.NAVIGATING_TO_SCAN); self.state = State.AT_SCAN_POSE
    def start_scan(self): self.need(State.AT_SCAN_POSE); self.state = State.SCANNING
    def finish_scan(self, telemetry):
        self.need(State.SCANNING)
        if not scan_complete(telemetry): raise WorkflowError('INCOMPLETE_SCAN', 'actual telemetry lacks full safe coverage')
        self.scan_revision += 1; self.state = State.SCAN_READY
    def analyze(self, observations): self.need(State.SCAN_READY); self.desks = desk_hypotheses(observations); self.detection_revision += 1; self.state = State.DESK_READY
    def preview(self, verifier, map_revision, room_revision, scan_revision, detection_revision):
        self.need(State.DESK_READY)
        if verifier is None: raise WorkflowError('NO_SAFE_DOCK', 'path verifier is required')
        if (map_revision, room_revision, scan_revision, detection_revision) != (self.bundle.revision, 1, self.scan_revision, self.detection_revision): raise WorkflowError('STALE_REVISION', 'candidate inputs are stale')
        room = next(item for item in self.rooms if item.id == self.proposal['room_id']); safe = []
        for desk in self.desks:
            x, y = desk['dock']; distance = math.hypot(x - desk['x'], y - desk['y'])
            if room.contains((x, y)) and desk.get('quality', 0) >= .6 and self.bundle.footprint_free(x, y) and .45 <= distance <= 1.5 and desk.get('line_of_sight') and verifier.verify((x, y), self.bundle.revision): safe.append(dict(desk, score=round(desk['quality'] * 10 - distance, 6)))
        if not safe: raise WorkflowError('NO_SAFE_DOCK', 'no candidate passed hard safety filters')
        self.candidates = sorted(safe, key=lambda item: (-item['score'], item['x'], item['y'])); self.state = State.DOCK_PREVIEW
    def confirm_dock(self, index=0): self.need(State.DOCK_PREVIEW); self.selected = self.candidates[index]; self.state = State.DOCK_CONFIRMED
    def dock_goal(self): self.need(State.DOCK_CONFIRMED); self.state = State.NAVIGATING_TO_DOCK
    def arrived(self): self.need(State.NAVIGATING_TO_DOCK); self.state = State.ARRIVED
    def cancel(self): self.state = State.CANCELED; self.reason = 'operator canceled'

class CommandASR:
    def __init__(self, argv=None): self.argv = argv
    def propose(self, rooms):
        if not self.argv or not shutil.which(self.argv[0]): raise WorkflowError('NOT_READY', 'configured ASR command is unavailable')
        result = subprocess.run(self.argv, check=False, capture_output=True, text=True, timeout=30)
        if result.returncode: raise WorkflowError('ASR_FAILED', 'configured ASR command failed')
        return parse_intent(result.stdout.strip(), rooms)

class RoomStore:
    @staticmethod
    def save(path, bundle, rooms): Path(path).write_text(json.dumps({'map_id': bundle.map_id, 'yaml_sha256': bundle.yaml_sha256, 'image_sha256': bundle.image_sha256, 'rooms': [room.__dict__ for room in rooms]}, ensure_ascii=False))
