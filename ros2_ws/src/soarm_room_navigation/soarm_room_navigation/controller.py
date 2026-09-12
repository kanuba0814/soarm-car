"""Serialized, terminal-aware execution controller for room navigation."""
import json
import math
import os
import threading
import time

from .core import NavigationError, Plan, TaskState


NONTERMINAL = {TaskState.PLANNING, TaskState.PROPOSAL, TaskState.CONFIRMATION,
               TaskState.RUNNING_NAV, TaskState.RUNNING_WAIT, TaskState.SETTLING, TaskState.CANCELING}


class TaskJournal:
    """Crash marker only; no executable plan is retained for replay."""
    def __init__(self, path):
        self.path = path

    def interrupted(self):
        try:
            with open(self.path, encoding='utf-8') as source:
                record = json.load(source)
                if not isinstance(record, dict) or not isinstance(record.get('nonterminal'), bool):
                    return True
                return record['nonterminal']
        except FileNotFoundError:
            return False
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return True

    def write(self, state, proposal_id=None, unresolved_recovery=False):
        directory = os.path.dirname(self.path) or '.'
        os.makedirs(directory, exist_ok=True)
        temporary = self.path + '.tmp'
        with open(temporary, 'w', encoding='utf-8') as output:
            json.dump({'nonterminal': state in NONTERMINAL or unresolved_recovery, 'state': state.value,
                       'proposal_id': proposal_id}, output, sort_keys=True)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, self.path)


class RoomNavigationController:
    def __init__(self, bundle, rooms, transport, ready, zero_output, start_pose, journal_path='', acceptance_timeout=5.0, execution_timeout=120.0):
        if not all(math.isfinite(value) and value > 0 for value in (acceptance_timeout, execution_timeout)):
            raise ValueError('navigation deadlines must be positive')
        self.acceptance_timeout, self.execution_timeout = acceptance_timeout, execution_timeout
        self.bundle, self.rooms, self.transport = bundle, list(rooms), transport
        self.ready, self.zero_output, self.start_pose = ready, zero_output, start_pose
        self.lock = threading.RLock()
        self.state, self.reason = TaskState.IDLE, ''
        self.room_revision, self.plan, self.step = 1, None, 0
        self.generation, self.handle, self.pending, self.wait_deadline = 0, None, False, None
        self.acceptance_deadline, self.execution_deadline = None, None
        self.goal_token, self.cancel_token, self.timeout_fallback = 0, None, None
        self.fallback_used, self.audit, self.execution_start = False, [], None
        self.proposal_deadline, self.used_proposals = None, set()
        self.cancel_target, self.cancel_terminal, self.gate_ack, self.cancel_epoch = None, False, False, 0
        self.journal = TaskJournal(journal_path) if journal_path else None
        self.recovery_required = bool(self.journal and self.journal.interrupted())
        if self.recovery_required:
            self.state = TaskState.FAILURE_AWAITING_INPUT
            self.reason = 'INTERRUPTED: prior nonterminal task was not resumed after restart'
        self._persist_locked()

    def _persist_locked(self):
        if self.journal:
            self.journal.write(self.state, self.plan.proposal_id if self.plan else None, self.recovery_required)

    def begin_recovery_stop(self):
        with self.lock:
            if self.recovery_required:
                self.recovery_required = False
                self._begin_cancel_locked(TaskState.FAILURE_AWAITING_INPUT,
                                          'INTERRUPTED: stopping any action left by the previous process')

    def status(self):
        with self.lock:
            return {'state': self.state.value, 'reason': self.reason,
                    'proposal_id': self.plan.proposal_id if self.plan else None,
                    'proposal_expires_in_s': max(0, int(self.proposal_deadline - time.monotonic())) if self.proposal_deadline else None,
                    'step': self.step, 'room_revision': self.room_revision, 'bundle_id': self.bundle.bundle_id,
                    'pending_goal': self.pending, 'active_goal': self.handle is not None,
                    'recovery_required': self.recovery_required,
                    'plan': self._plan_status(), 'audit': self.audit[-16:]}

    def _plan_status(self):
        if not self.plan:
            return None
        return {'summary': self.plan.summary, 'actions': [action.__dict__ for action in self.plan.actions]}

    def replace_rooms(self, rooms):
        with self.lock:
            if self.state in NONTERMINAL:
                raise NavigationError('GOAL_ACTIVE', 'room edits are blocked while a task is active')
            self.rooms, self.room_revision = list(rooms), self.room_revision + 1
            self.plan, self.proposal_deadline = None, None
            self.state, self.reason = TaskState.IDLE, 'rooms updated; submit a new plan'
            self._persist_locked()

    def begin_planning(self):
        with self.lock:
            if self.recovery_required:
                raise NavigationError('RECOVERY_REQUIRED', 'interrupted task must be stopped before a new proposal')
            if self.state in (TaskState.RUNNING_WAIT, TaskState.SETTLING, TaskState.CONFIRMATION, TaskState.RUNNING_NAV, TaskState.CANCELING) or self.pending or self.handle is not None:
                raise NavigationError('GOAL_ACTIVE', 'planning is blocked until the current task reaches a terminal state')
            self.generation += 1
            self.plan, self.proposal_deadline = None, None
            self.state, self.reason = TaskState.PLANNING, 'planning request in progress'
            self.audit.append('planning:' + str(self.generation))
            self._persist_locked()
            return self.generation, self.room_revision, tuple(self.rooms)

    def complete_planning(self, generation, plan_or_clarification):
        with self.lock:
            if generation != self.generation or self.state != TaskState.PLANNING:
                return self.status()
            if isinstance(plan_or_clarification, str):
                self.plan, self.state, self.reason = None, TaskState.CLARIFICATION, plan_or_clarification
            elif isinstance(plan_or_clarification, Plan):
                self.plan, self.step, self.state = plan_or_clarification, 0, TaskState.PROPOSAL
                self.fallback_used, self.execution_start = False, None
                self.proposal_deadline = time.monotonic() + 120.0
                self.reason = 'review the immutable plan before confirmation'
                self.audit.append('proposal:' + self.plan.proposal_id)
            else:
                self.state, self.reason = TaskState.FAILURE_AWAITING_INPUT, 'planner result is not a valid plan'
            self._persist_locked()
            return self.status()

    def planning_failed(self, generation, error):
        with self.lock:
            if generation == self.generation and self.state == TaskState.PLANNING:
                self.state, self.reason = TaskState.FAILURE_AWAITING_INPUT, error.message
                self._persist_locked()

    def propose(self, plan_or_clarification):
        """Synchronous test seam; HTTP production uses begin/complete_planning."""
        generation, _, _ = self.begin_planning()
        return self.complete_planning(generation, plan_or_clarification)

    def confirm(self, proposal_id):
        with self.lock:
            if (self.state != TaskState.PROPOSAL or self.plan is None or proposal_id != self.plan.proposal_id or
                    proposal_id in self.used_proposals or not self.proposal_deadline or time.monotonic() >= self.proposal_deadline):
                raise NavigationError('STALE_CONFIRMATION', 'confirmation must name one unexpired current proposal')
            if self.plan.bundle_id != self.bundle.bundle_id or self.plan.room_revision != self.room_revision or not self.ready():
                self._begin_cancel_locked(TaskState.SAFETY_BLOCKED, 'map, rooms, or readiness changed before confirmation')
                raise NavigationError('NOT_READY', self.reason)
            self.execution_start = self.start_pose()
            if self.execution_start is None or not self.bundle.footprint_free(*self.execution_start):
                self._begin_cancel_locked(TaskState.SAFETY_BLOCKED, 'fresh captured start pose is unavailable or unsafe')
                raise NavigationError('NOT_READY', self.reason)
            self.used_proposals.add(proposal_id)
            self.state, self.reason = TaskState.CONFIRMATION, 'confirmed'
            self.audit.append('confirmed:' + proposal_id)
            self._persist_locked()
            self._advance_locked()
            return self.status()

    def tick(self, now=None):
        with self.lock:
            now = time.monotonic() if now is None else now
            if self.state == TaskState.SETTLING:
                if not self.ready():
                    self._begin_cancel_locked(TaskState.SAFETY_BLOCKED, 'safety changed while settling')
                elif self.zero_output():
                    self._advance_locked()
                elif now >= self.settle_deadline:
                    self._begin_cancel_locked(TaskState.SAFETY_BLOCKED, 'safe output did not settle after arrival')
            elif self.state == TaskState.RUNNING_WAIT:
                if not self.ready() or not self.zero_output():
                    self._begin_cancel_locked(TaskState.SAFETY_BLOCKED, 'health, map, gate, or zero output changed during wait')
                elif now >= self.wait_deadline:
                    self.step += 1
                    self._advance_locked()
            elif self.state == TaskState.CANCELING and self.timeout_fallback:
                if not self.ready():
                    self._preempt_timeout_fallback_locked(
                        TaskState.SAFETY_BLOCKED, 'safety changed while draining timed-out navigation')
            elif self.state == TaskState.RUNNING_NAV:
                if self.pending and now >= self.acceptance_deadline:
                    self._begin_cancel_locked(TaskState.SAFETY_BLOCKED, 'Nav2 goal acceptance deadline expired')
                elif self.handle is not None and now >= self.execution_deadline:
                    self._begin_execution_timeout_locked()

    def safety_block(self, reason):
        with self.lock:
            if self.state == TaskState.CANCELING and self.timeout_fallback:
                self._preempt_timeout_fallback_locked(TaskState.SAFETY_BLOCKED, reason)
            elif self.state in NONTERMINAL - {TaskState.CANCELING, TaskState.PLANNING}:
                self._begin_cancel_locked(TaskState.SAFETY_BLOCKED, reason)

    def stop(self):
        with self.lock:
            # Stop is never reported terminal until the gate acknowledges its latch,
            # even when there is no current Nav2 handle to drain.
            self.generation += 1
            if self.state == TaskState.CANCELING and self.timeout_fallback:
                self._preempt_timeout_fallback_locked(TaskState.CANCELED, 'operator stop requested')
            elif self.state != TaskState.CANCELING:
                self._begin_cancel_locked(TaskState.CANCELED, 'operator stop requested')
            return self.status()

    def _preempt_timeout_fallback_locked(self, terminal_state, reason):
        # Keep the same unresolved goal/token, but revoke the automatic fallback.
        self.timeout_fallback = None
        self.cancel_target, self.reason = terminal_state, reason
        self.cancel_epoch += 1
        self.audit.append('timeout-fallback-revoked:' + reason)
        self._request_gate_stop_locked()
        self._finish_cancel_locked()
        self._persist_locked()

    def gate_stop_result(self, acknowledged, epoch=None):
        with self.lock:
            if epoch is not None and epoch != self.cancel_epoch:
                return
            if not acknowledged:
                if self.state == TaskState.CANCELING:
                    self.reason = 'operator stop gate did not acknowledge latch; cancellation remains unresolved'
                self._persist_locked()
                return
            self.gate_ack = True
            self._finish_cancel_locked()

    def _request_gate_stop_locked(self):
        self.gate_ack = False
        epoch = self.cancel_epoch
        result = self.transport.stop_gate(lambda acknowledged: self.gate_stop_result(acknowledged, epoch))
        if result is True:
            self.gate_ack = True

    def _begin_cancel_locked(self, terminal_state, reason):
        if self.state == TaskState.CANCELING:
            return
        self.generation += 1
        self.state, self.reason, self.wait_deadline = TaskState.CANCELING, reason, None
        self.cancel_epoch += 1
        self.cancel_target, self.cancel_terminal, self.cancel_token = terminal_state, False, self.goal_token
        self.acceptance_deadline = self.execution_deadline = None
        self.audit.append('cancel:' + reason)
        self._request_gate_stop_locked()
        if self.handle is not None:
            self.transport.cancel(self.handle)
        elif self.pending:
            self.transport.cancel_pending()
        else:
            self.cancel_terminal = True
            self._finish_cancel_locked()
        self._persist_locked()

    def _finish_cancel_locked(self):
        if self.state == TaskState.CANCELING and self.cancel_terminal and self.gate_ack:
            self.pending, self.handle = False, None
            if self.timeout_fallback:
                fallback, self.timeout_fallback = self.timeout_fallback, None
                self._send_fallback_locked(fallback)
                return
            self.state = self.cancel_target
            self.reason = ('INTERRUPTED: stop requested; explicit gate arm and a new plan are required'
                           if 'INTERRUPTED' in self.reason else
                           'navigation action and command gate reached terminal stop state')
            self.audit.append('canceled-terminal')
            self._persist_locked()

    def cancel_timeout(self):
        with self.lock:
            if self.state == TaskState.CANCELING:
                self.reason = 'cancellation deadline expired; Nav2 action and gate drain remain unresolved'
                self.audit.append('cancel-timeout-unresolved')
                self._persist_locked()

    def _begin_execution_timeout_locked(self):
        action = self.plan.actions[self.step]
        fallback = action.fallback_room_id if action.kind in ('navigate', 'return_to_start') else ''
        if not fallback or self.fallback_used or not self.ready():
            self._begin_cancel_locked(TaskState.SAFETY_BLOCKED, 'Nav2 execution deadline expired')
            return
        self.state, self.reason, self.wait_deadline = TaskState.CANCELING, 'Nav2 execution deadline expired; draining before fallback', None
        self.cancel_target, self.cancel_terminal, self.cancel_token = TaskState.SAFETY_BLOCKED, False, self.goal_token
        self.timeout_fallback, self.gate_ack = fallback, True
        self.acceptance_deadline = self.execution_deadline = None
        self.audit.append('execution-timeout:' + fallback)
        self.transport.cancel(self.handle)
        self._persist_locked()

    def _advance_locked(self):
        if not self.ready():
            self._begin_cancel_locked(TaskState.SAFETY_BLOCKED, 'localization or map identity is not ready')
            return
        if self.step >= len(self.plan.actions):
            self.state, self.reason = TaskState.SUCCESS, 'all confirmed plan actions completed'
            self.audit.append('success')
            self._persist_locked()
            return
        action = self.plan.actions[self.step]
        if action.kind == 'wait':
            if not self.zero_output():
                self._begin_cancel_locked(TaskState.SAFETY_BLOCKED, 'safe velocity output is not zero before wait')
                return
            self.state, self.wait_deadline = TaskState.RUNNING_WAIT, time.monotonic() + action.seconds
            self.audit.append('wait:' + str(action.seconds))
            self._persist_locked()
            return
        room = None if action.kind == 'return_to_start' else self._room(action.room_id)
        pose = self.execution_start if room is None else room.goal_pose
        require_yaw = False if room is None else room.require_yaw
        safe = pose is not None and (self.bundle.footprint_free(*pose) if require_yaw else
                                     self.bundle.footprint_free_any_yaw(*pose[:2]))
        if not safe:
            self._begin_cancel_locked(TaskState.SAFETY_BLOCKED, 'captured return pose or goal footprint is unsafe')
            return
        self.fallback_used = False
        self._send_navigation_locked(pose, require_yaw, 'navigate:' + action.kind)

    def _send_navigation_locked(self, pose, require_yaw, audit):
        self.state, self.pending, self.handle = TaskState.RUNNING_NAV, True, None
        self.generation += 1
        generation = self.generation
        self.goal_token += 1
        token = self.goal_token
        self.acceptance_deadline = time.monotonic() + self.acceptance_timeout
        self.execution_deadline = None
        self.audit.append(audit)
        self._persist_locked()
        self.transport.send(pose, require_yaw, lambda handle, failure='rejected': self._accepted(handle, generation, token, failure),
                            lambda result: self._result(result, generation, token))

    def _room(self, room_id):
        for room in self.rooms:
            if room.id == room_id:
                return room
        raise NavigationError('UNKNOWN_ROOM', 'plan room is no longer defined')

    def _accepted(self, handle, generation, token, failure='rejected'):
        with self.lock:
            if self.state == TaskState.CANCELING and token == self.cancel_token:
                self.pending = False
                if handle is not None:
                    self.handle = handle
                    self.transport.cancel(handle)
                else:
                    self.cancel_terminal = True
                    self._finish_cancel_locked()
                self._persist_locked()
                return
            if generation != self.generation or token != self.goal_token:
                if handle is not None:
                    self.transport.cancel(handle)
                return
            self.pending = False
            if handle is None:
                self._navigation_failure_locked('Nav2 goal ' + failure, safety=failure != 'rejected')
            else:
                self.handle = handle
                self.execution_deadline = time.monotonic() + self.execution_timeout
                self._persist_locked()

    def _result(self, result, generation, token):
        with self.lock:
            if self.state == TaskState.CANCELING and token == self.cancel_token:
                self.pending, self.handle, self.cancel_terminal = False, None, True
                self._finish_cancel_locked()
                return
            if generation != self.generation or token != self.goal_token:
                return
            self.pending, self.handle, self.acceptance_deadline, self.execution_deadline = False, None, None, None
            if result == 'succeeded':
                self.step += 1
                if self.zero_output():
                    self._advance_locked()
                else:
                    # Action completion can precede the final zero-command callback.
                    self.state = TaskState.SETTLING
                    self.settle_deadline = time.monotonic() + 1.0
                    self.reason = 'arrival confirmed; awaiting fresh zero output'
                    self._persist_locked()
            elif result == 'aborted':
                self._navigation_failure_locked('navigation aborted', safety=False)
            else:
                self._navigation_failure_locked('navigation safety or transport failure: ' + str(result), safety=True)

    def _navigation_failure_locked(self, reason, safety):
        if safety:
            self._begin_cancel_locked(TaskState.SAFETY_BLOCKED, reason)
            return
        action = self.plan.actions[self.step] if self.plan and self.step < len(self.plan.actions) else None
        fallback = action.fallback_room_id if action and action.kind in ('navigate', 'return_to_start') else ''
        if fallback and not self.fallback_used and self.ready():
            self.fallback_used = True
            self._send_fallback_locked(fallback)
            return
        self.state, self.reason = TaskState.FAILURE_AWAITING_INPUT, reason
        self.audit.append('failure:' + reason)
        self._persist_locked()

    def _send_fallback_locked(self, fallback):
        replacement = self._room(fallback)
        safe = (self.bundle.footprint_free(*replacement.goal_pose) if replacement.require_yaw else
                self.bundle.footprint_free_any_yaw(*replacement.goal_pose[:2]))
        if not safe:
            self._begin_cancel_locked(TaskState.SAFETY_BLOCKED, 'fallback goal footprint is unsafe')
            return
        self._send_navigation_locked(replacement.goal_pose, replacement.require_yaw, 'fallback:' + fallback)
