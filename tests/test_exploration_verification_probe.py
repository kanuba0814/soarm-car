import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from exploration_verification_probe import VerificationPolicy


class VerificationPolicyTests(unittest.TestCase):
    def ready(self, policy):
        policy.start(0., 10.)
        policy.clock(11., .1)
        policy.health(1, 11., .1)

    def moving_goal(self, policy, uuid='a'):
        policy.status(uuid, 2, 11., .2)
        self.assertIsNone(policy.odom(11., 0., 0., 0., .2))
        return policy.odom(12., .05, 0., 0., .3)

    def test_moving_pause_cancel_stop_passes(self):
        policy = VerificationPolicy('pause')
        self.ready(policy)
        motion = self.moving_goal(policy)
        self.assertTrue(policy.should_trigger(motion, .3))
        policy.trigger(.3)
        policy.rpc_response(True, 'cancellation requested')
        policy.status('a', 5, 11., .4)
        stopped = False
        for stamp, wall in ((13., .5), (14., .8), (15., 1.1)):
            policy.clock(stamp, wall)
            policy.health(1, stamp, wall)
            motion = policy.odom(stamp, .05, 0., 0., wall)
            stopped = policy.stopped_sample(True, wall, motion, wall)
        self.assertTrue(stopped)

    def test_idle_trigger_is_rejected_and_yaw_motion_is_accepted(self):
        policy = VerificationPolicy('pause')
        self.ready(policy)
        policy.status('a', 2, 11., .2)
        policy.odom(11., 0., 0., 0., .2)
        self.assertFalse(policy.should_trigger(policy.odom(12., .001, 0., 0., .3), .3))
        self.assertTrue(policy.should_trigger(policy.odom(13., .001, 0., .1, .4), .4))

    def test_succeeded_before_cancel_is_missed_window(self):
        policy = VerificationPolicy('pause')
        self.ready(policy)
        self.moving_goal(policy)
        policy.status('a', 4, 11., .4)
        self.assertEqual(policy.finalize(.4, 120), 'MISSED_WINDOW')

    def test_stale_odom_or_zero_velocity_alone_is_not_stop_proof(self):
        policy = VerificationPolicy('pause')
        self.ready(policy)
        self.moving_goal(policy)
        policy.trigger(.3)
        policy.rpc_response(True, 'ok')
        policy.status('a', 5, 11., .4)
        self.assertFalse(policy.stopped_sample(True, None, None, .5))
        self.assertIsNone(policy.odom(12., .05, 0., 0., .6))
        policy.clock(13., .7)
        policy.health(1, 13., .7)
        stationary = policy.odom(13., .05, 0., 0., .7)
        self.assertFalse(policy.stopped_sample(True, None, stationary, .7))

    def test_duplicate_uuid_and_late_goal_are_not_new_control_targets(self):
        policy = VerificationPolicy('pause')
        self.ready(policy)
        self.moving_goal(policy)
        policy.status('a', 2, 11., .4)
        policy.status('b', 2, 11., .4)
        self.assertEqual(policy.active_uuid, 'a')
        self.assertIn('late_or_concurrent_goal:b', policy.flags)

    def test_old_goal_is_rejected_and_timeout_is_explicit(self):
        policy = VerificationPolicy('pause')
        self.ready(policy)
        policy.status('old', 2, 10., .2)
        self.assertIsNone(policy.active_uuid)
        self.assertEqual(policy.finalize(120., 120.), 'TIMEOUT')


if __name__ == '__main__':
    unittest.main()
