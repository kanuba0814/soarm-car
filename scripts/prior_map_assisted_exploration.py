#!/usr/bin/env python3
"""CPU supervisor for prior-map-assisted exploration.

This module owns freshness, bounded source buffering, and Nav2 action sequencing.
The imported exploration core remains the sole candidate/coverage implementation.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
import time


class State(str, Enum):
    IDLE = 'idle'
    READY = 'ready'
    PLANNING = 'planning'
    NAVIGATING = 'navigating'
    CANCELLING = 'cancelling'
    FRONTIERS_EXHAUSTED = 'frontiers_exhausted'
    BLOCKED = 'blocked'
    BUDGET_EXHAUSTED = 'budget_exhausted'
    CANCELLED = 'cancelled'
    SENSOR_FAULT = 'sensor_fault'


TERMINAL = {
    State.FRONTIERS_EXHAUSTED, State.BLOCKED, State.BUDGET_EXHAUSTED,
    State.CANCELLED, State.SENSOR_FAULT,
}


@dataclass(frozen=True)
class SourceRecord:
    observation: dict
    receipt: float
    clock: float
    tf_ok: bool


class FreshnessGate:
    """Fail closed on identity, replay, paused simulation clock, or missing TF."""

    def __init__(self, expected_health_map_id, source_map_id=None, timeout=1.0, resume_records=2):
        self.expected_health_map_id = expected_health_map_id
        self.source_map_id = source_map_id or expected_health_map_id
        self.timeout = timeout
        self.resume_records = resume_records
        self.health_receipt = None
        self.health_ok = False
        self.last_sensor_stamp = None
        self.last_receipt = None
        self.last_clock = None
        self.consecutive = 0
        self.reason = 'health unavailable'

    def health(self, state, map_id, receipt):
        self.health_receipt = receipt
        self.health_ok = state == 'HEALTHY' and map_id == self.expected_health_map_id
        if not self.health_ok:
            self.reason = 'health state or map identity mismatch'

    def source(self, record):
        observation = record.observation
        stamp = observation.get('source_stamp')
        valid = (
            observation.get('map_identity') == self.source_map_id and
            observation.get('frame') == 'map' and
            isinstance(stamp, (int, float)) and math.isfinite(stamp) and
            record.tf_ok and
            isinstance(record.receipt, (int, float)) and math.isfinite(record.receipt) and
            isinstance(record.clock, (int, float)) and math.isfinite(record.clock) and
            (self.last_sensor_stamp is None or stamp > self.last_sensor_stamp) and
            (self.last_receipt is None or record.receipt > self.last_receipt) and
            (self.last_clock is None or record.clock > self.last_clock)
        )
        if valid:
            self.last_sensor_stamp, self.last_receipt, self.last_clock = stamp, record.receipt, record.clock
            self.consecutive += 1
            self.reason = ''
        else:
            self.consecutive = 0
            self.reason = 'stale/replayed source, map/frame mismatch, missing TF, or paused clock'
        return valid

    def ready(self, now):
        if not self.health_ok or self.health_receipt is None or now - self.health_receipt < 0 or now - self.health_receipt > self.timeout:
            self.reason = 'health unavailable, stale, or wrong map identity'
            return False
        if self.consecutive < self.resume_records:
            self.reason = 'insufficient consecutive fresh source records'
            return False
        return True


class LocalizationBootstrap:
    """Bounded known-origin seed; waits only for clock and local FAST-LIO odometry."""

    def __init__(self, sends=6, interval=.5):
        self.sends, self.interval = sends, interval
        self.clock = None
        self.odom_stamp = None
        self.last_clock = None
        self.last_odom_stamp = None
        self.sent = 0
        self.last_send = None
        self.complete = False
        self.conditions = []

    def clock_sample(self, value):
        self.clock = value

    def odometry_sample(self, stamp):
        self.odom_stamp = stamp

    def healthy(self, matching):
        if matching:
            self.complete = True

    def due(self, wall_now):
        if self.complete or self.sent >= self.sends:
            return False
        advancing_clock = self.clock is not None and self.clock > 0 and (self.last_clock is None or self.clock > self.last_clock)
        advancing_odom = self.odom_stamp is not None and self.odom_stamp > 0 and (self.last_odom_stamp is None or self.odom_stamp > self.last_odom_stamp)
        self.conditions = [{'clock': self.clock, 'odom_stamp': self.odom_stamp, 'advancing_clock': advancing_clock, 'advancing_odom': advancing_odom}]
        if not (advancing_clock and advancing_odom):
            return False
        if self.last_send is not None and wall_now - self.last_send < self.interval:
            return False
        self.last_clock, self.last_odom_stamp, self.last_send = self.clock, self.odom_stamp, wall_now
        self.sent += 1
        return True


class PointCloudAdapter:
    """Build core observations from already TF-validated map-frame point samples."""

    def __init__(self, map_id, max_validated_rays=1024, ground_z=0.0,
                 floor_tolerance=.10, navigation_z_min=-.30, navigation_z_max=.50):
        if not 1 <= max_validated_rays <= 1024:
            raise ValueError('max_validated_rays must be in [1, 1024]')
        if not all(math.isfinite(value) for value in (ground_z, floor_tolerance, navigation_z_min, navigation_z_max)) or floor_tolerance < 0 or navigation_z_min > navigation_z_max:
            raise ValueError('invalid prior-floor or navigation-band calibration')
        self.map_id, self.max_validated_rays = map_id, max_validated_rays
        self.ground_z, self.floor_tolerance = ground_z, floor_tolerance
        self.navigation_z_min, self.navigation_z_max = navigation_z_min, navigation_z_max

    def classify(self, point, ground_reference_z=None):
        """Explicit ground references use relative bands; legacy fixtures retain map-Z bands."""
        if ground_reference_z is None:
            if abs(point[2] - self.ground_z) <= self.floor_tolerance:
                return 'floor'
            return 'obstacle' if self.navigation_z_min <= point[2] <= self.navigation_z_max else 'floor'
        relative_z = point[2] - ground_reference_z
        if abs(relative_z) <= self.floor_tolerance:
            return 'floor'
        return 'obstacle' if self.navigation_z_min <= relative_z <= self.navigation_z_max else 'floor'

    def record(self, identifier, revision, sensor_stamp, sensor_origin, robot_pose, points, receipt, clock, tf_ok, sample_policy='full_message_uniform_index', ground_reference_z=None, policy=None):
        ground = self.ground_z if ground_reference_z is None else ground_reference_z
        if not isinstance(ground, (int, float)) or not math.isfinite(ground):
            raise ValueError('nonfinite ground reference')
        returns = []
        for point in points:
            if len(returns) >= self.max_validated_rays:
                break
            if len(point) != 3 or not all(math.isfinite(value) for value in point):
                raise ValueError('nonfinite PointCloud2 point')
            returns.append({'xyz': list(point), 'kind': self.classify(point, ground_reference_z)})
        return SourceRecord({
            'id': identifier,
            'revision': revision,
            'map_identity': self.map_id,
            'frame': 'map',
            'source_stamp': sensor_stamp,
            'sensor_origin': list(sensor_origin),
            'robot_pose': list(robot_pose),
            'ground_reference_z': ground,
            'returns': returns,
            'sample_policy': sample_policy,
            'sensor_policy': policy or {},
        }, receipt, clock, tf_ok)


class ExplorationSupervisor:
    """Single-owner supervisor: core shortlist -> ComputePath -> NavigateToPose."""

    def __init__(self, core, navigator, expected_health_map_id, *, source_map_id=None, max_goals=6, wall_limit=1200.0,
                 per_goal_wall=180.0, startup_grace=45.0, separation_m=.2, arrival_tolerance=.2):
        self.core = core
        self.navigator = navigator
        self.gate = FreshnessGate(expected_health_map_id, source_map_id)
        self.records = deque(maxlen=1024)
        self.consumed = deque(maxlen=1024)
        self.max_goals, self.wall_limit, self.per_goal_wall, self.startup_grace = max_goals, wall_limit, per_goal_wall, startup_grace
        self.separation_m, self.arrival_tolerance = separation_m, arrival_tolerance
        self.state, self.reason = State.IDLE, 'not started'
        self.robot_pose = None
        self.started_at = None
        self.active = None
        self.goals = []
        self.failures = 0
        self.events = deque(maxlen=256)
        self._last_progress = None
        self._last_observation_receipt = None
        self._pending_terminal = None
        self.dispatch_authorized = True

    def start(self, now):
        if self.state != State.IDLE:
            raise RuntimeError('supervisor already started')
        self.started_at = now
        self._transition(State.READY, 'waiting for fresh source records', now)

    def health(self, state, map_id, receipt):
        self.gate.health(state, map_id, receipt)

    def require_gate_arm(self):
        """Hold goal dispatch until the existing safety gate confirms its arm RPC."""
        self.dispatch_authorized = False

    def authorize_dispatch(self):
        if self.state != State.READY or self.active is not None or self.navigator.busy or not self.navigator.terminal:
            raise RuntimeError('gate arm requires no active navigation action')
        self.dispatch_authorized = True

    def source_fault(self, now, reason='supervisor source-drop injection'):
        """Fault the owner directly; dropping a callback alone must not permit another goal."""
        if self.state not in TERMINAL:
            self._request_cancel(State.SENSOR_FAULT, reason, now)

    def enqueue(self, record):
        self.records.append(record)

    def pause(self, now):
        if self.state in TERMINAL:
            return None
        checkpoint = self.core.export_state()
        self._request_cancel(State.CANCELLED, 'pause requested', now)
        return checkpoint

    def resume(self, checkpoint, now):
        if self.state != State.CANCELLED or self._pending_terminal is not None:
            raise RuntimeError('resume requires confirmed cancellation')
        self.core.import_state(checkpoint)
        self.gate.consecutive = 0
        self.records.clear()
        self.active = None
        self.dispatch_authorized = False
        self._transition(State.READY, 'resume requires fresh healthy source records and a new plan', now)

    def cancel(self, now):
        if self.state in TERMINAL:
            return
        self._request_cancel(State.CANCELLED, 'operator cancellation', now)

    def drain_events(self):
        items = list(self.events)
        self.events.clear()
        return items

    def drain_consumed(self):
        items = list(self.consumed)
        self.consumed.clear()
        return items

    def _transition(self, state, reason, now):
        self.state, self.reason = state, reason
        self.events.append({'time': now, 'state': state.value, 'reason': reason})

    def _request_cancel(self, terminal, reason, now):
        self._pending_terminal = (terminal, reason)
        if self.navigator.busy:
            self.navigator.cancel()
            self._transition(State.CANCELLING, reason, now)
        else:
            self._transition(terminal, reason, now)

    def _finish_cancel(self, now):
        poll_cancel = getattr(self.navigator, 'poll_cancel', None)
        if poll_cancel is not None:
            poll_cancel()
        if self._pending_terminal is None or not self.navigator.terminal:
            return False
        state, reason = self._pending_terminal
        self._pending_terminal = None
        self._transition(state, reason, now)
        return True

    def _start_navigation(self, candidate, shortlist_revision, now, plan):
        plan_ok = plan.get('accepted') and plan.get('status') == 'SUCCEEDED' and plan.get('frame_id') == 'map' and bool(plan.get('poses'))
        current = self.core.candidates(self.robot_pose)
        still_safe = any(item['cluster_id'] == candidate['cluster_id'] for item in current.get('candidates', []))
        if (not plan_ok or self.core.revision != shortlist_revision or not still_safe or
                not self.navigator.safe(candidate['goal'])):
            self.core.report_result(candidate['cluster_id'], 0, False)
            self.failures += 1
            self._transition(State.READY, 'candidate failed path or revalidation', now)
            return
        goal_uuid = self.navigator.send(candidate['goal'])
        self.active = {'candidate': candidate, 'goal_uuid': goal_uuid, 'started_at': now, 'coverage_before': len(getattr(self.core, 'observed', []))}
        self._transition(State.NAVIGATING, 'NavigateToPose active after accepted nonempty ComputePathToPose', now)

    def _progress(self, now, shortlist=None):
        if self._last_progress is not None and now - self._last_progress < 1.0:
            return
        self._last_progress = now
        shortlist = shortlist or {}
        self.events.append({'time': now, 'state': self.state.value, 'progress': {
            'observed_area_cells': shortlist.get('observed_area_cells', len(getattr(self.core, 'observed', []))),
            'reference_prior_connected_area_cells': shortlist.get('reference_prior_connected_area_cells'),
            'remaining_frontiers': len(shortlist.get('candidates', [])),
            'selected_goal': self.active['candidate']['cluster_id'] if self.active else None,
            'failures': self.failures,
            'goals': len(self.goals),
        }})

    def tick(self, now):
        if self.state in TERMINAL:
            return self.state
        if self.started_at is None:
            raise RuntimeError('supervisor not started')
        if now - self.started_at > self.wall_limit:
            self._request_cancel(State.BUDGET_EXHAUSTED, 'whole-run wall budget exhausted', now)
        if self.state == State.READY and now - self.started_at > self.startup_grace and not self.gate.ready(now):
            self._request_cancel(State.SENSOR_FAULT, 'startup data grace expired: ' + self.gate.reason, now)
        if self.state != State.CANCELLING and self.records:
            newest = self.records[-1]
            if (self._last_observation_receipt is None or
                    newest.receipt - self._last_observation_receipt >= .5):
                self.records.clear()
                if not self.gate.source(newest):
                    self._request_cancel(State.SENSOR_FAULT, self.gate.reason, now)
                    return self.state
                self.robot_pose = newest.observation['robot_pose']
                self.core.apply_observation(newest.observation)
                self.consumed.append(newest.observation)
                self._last_observation_receipt = newest.receipt
        if self.state == State.CANCELLING:
            self._finish_cancel(now)
            return self.state
        if self.state in (State.PLANNING, State.NAVIGATING) and not self.gate.ready(now):
            self._request_cancel(State.SENSOR_FAULT, self.gate.reason, now)
            return self.state
        if self.state == State.PLANNING:
            plan = self.navigator.poll_path()
            if plan is None:
                return self.state
            pending = self.active
            self.active = None
            self._start_navigation(pending['candidate'], pending['revision'], now, plan)
            return self.state
        if self.state == State.NAVIGATING:
            result = self.navigator.poll()
            if result is None:
                if now - self.active['started_at'] > self.per_goal_wall:
                    self._request_cancel(State.BUDGET_EXHAUSTED, 'per-goal wall budget exhausted', now)
                return self.state
            candidate = self.active['candidate']
            gained = len(getattr(self.core, 'observed', [])) - self.active['coverage_before']
            succeeded = result == 'SUCCEEDED' and gained > 0
            self.core.report_result(candidate['cluster_id'], gained, succeeded)
            self.goals.append({'cluster_id': candidate['cluster_id'], 'goal_uuid': self.active['goal_uuid'], 'status': result, 'coverage_delta': gained})
            self.active = None
            if not succeeded:
                self.failures += 1
            self._transition(State.READY, 'goal terminal; selecting fresh candidate', now)
        if not self.gate.ready(now):
            self._progress(now)
            return self.state
        if not self.dispatch_authorized:
            self._progress(now)
            return self.state
        if len(self.goals) >= self.max_goals:
            self._request_cancel(State.BUDGET_EXHAUSTED, 'exploration goal budget exhausted', now)
            return self.state
        shortlist = self.core.candidates(self.robot_pose)
        self._progress(now, shortlist)
        status = shortlist['status']
        if status in ('no_frontier', 'exhausted'):
            self._transition(State.FRONTIERS_EXHAUSTED, status, now)
            return self.state
        if status == 'blocked':
            self._transition(State.BLOCKED, status, now)
            return self.state
        candidate = shortlist['candidates'][0]
        plan = self.navigator.compute_path(candidate['goal'])
        if plan is None:
            self.active = {'candidate': candidate, 'revision': shortlist['revision']}
            self._transition(State.PLANNING, 'ComputePathToPose pending', now)
            return self.state
        self._start_navigation(candidate, shortlist['revision'], now, plan)
        return self.state
