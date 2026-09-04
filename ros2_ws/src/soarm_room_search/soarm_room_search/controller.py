"""One serialized workflow owner shared by demo and ROS transports."""
import threading
from .core import State, WorkflowError

class SimulationPathVerifier:
    name = 'SIMULATION mock path verifier'
    def verify(self, pose, revision): return True

class SearchController:
    def __init__(self, workflow, navigation, verifier=None, simulation=False):
        self.workflow = workflow; self.navigation = navigation; self.verifier = verifier; self.simulation = simulation
        self.lock = threading.RLock(); self.accepted_goal = None; self.goal_pending = False; self.room_revision = 1
    def status(self):
        return {'state': self.workflow.state.value, 'reason': self.workflow.reason, 'simulation': self.simulation, 'path_verifier': getattr(self.verifier, 'name', None), 'candidates': self.workflow.candidates}
    def _start(self, pose, completion):
        if self.goal_pending or self.accepted_goal is not None: raise WorkflowError('GOAL_ACTIVE', 'previous Nav2 goal is not terminal')
        self.goal_pending = True
        self.navigation.send(pose, self._accepted, completion)
    def _accepted(self, handle):
        with self.lock:
            self.goal_pending = False
            if handle is None: self.workflow.state = State.FAILED; self.workflow.reason = 'Nav2 rejected goal'; return
            self.accepted_goal = handle
    def _terminal(self, outcome, success):
        with self.lock:
            self.accepted_goal = None; self.goal_pending = False
            if not success: self.workflow.state = State.FAILED; self.workflow.reason = 'Nav2 ' + outcome; return
            if outcome == 'scan': self.workflow.at_scan()
            elif outcome == 'dock': self.workflow.arrived()
    def navigate_scan(self, pose):
        with self.lock:
            if self.goal_pending or self.accepted_goal is not None:
                raise WorkflowError('GOAL_ACTIVE', 'previous Nav2 goal is not terminal')
            self.workflow.scan_goal()
            self._start(pose, lambda ok: self._terminal('scan', ok))
    def navigate_dock(self, pose):
        with self.lock:
            if self.goal_pending or self.accepted_goal is not None:
                raise WorkflowError('GOAL_ACTIVE', 'previous Nav2 goal is not terminal')
            self.workflow.dock_goal()
            self._start(pose, lambda ok: self._terminal('dock', ok))
    def preview(self):
        with self.lock:
            workflow = self.workflow
            workflow.preview(self.verifier, workflow.bundle.revision, self.room_revision, workflow.scan_revision, workflow.detection_revision)
    def cancel(self):
        with self.lock:
            if self.accepted_goal is not None: self.accepted_goal.cancel_goal_async()
            self.workflow.cancel()
