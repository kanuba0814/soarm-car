"""Interactive keyboard teleop publishing Twist on the shared command seam.

Every chassis target in this workspace consumes ``geometry_msgs/msg/Twist``
on one of two topics, so one teleop covers all of them:

==========================  ============  ====================================
target                      topic         consumer
==========================  ============  ====================================
Fortress native simulation  ``/cmd_vel``  ``MecanumDrive`` via ros_gz bridge
kuruma_ros2_control plant   ``/safe_cmd_vel``  ``mecanum_drive_controller``
physical STM32 chassis      ``/cmd_vel``  ``localization_cmd_gate`` -> serial
==========================  ============  ====================================

On the physical car the teleop deliberately stays behind the fail-closed
safety gate: it never talks to the chassis serial link directly
(WORKSPACE_INDEX.md, ROS integration constraints).

Key model: a key event refreshes that key for ``hold_timeout`` seconds.
Held keys keep producing events through terminal auto-repeat, so motion
sustains while a key is held and everything decays to zero once input
stops. This deadman-style expiry replaces the run-on behavior of classic
persist-until-next-key teleops, which is the wrong default for a physical
chassis behind a laggy SSH link.
"""

import math
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# (forward, left) direction multipliers; the 3x3 grid around the home row.
TRANSLATION_KEYS = {
    'u': (1.0, 1.0), 'i': (1.0, 0.0), 'o': (1.0, -1.0),
    'j': (0.0, 1.0), 'l': (0.0, -1.0),
    'm': (-1.0, 1.0), ',': (-1.0, 0.0), '.': (-1.0, -1.0),
}
ROTATE_KEYS = {'q': 1.0, 'e': -1.0}  # q = CCW (positive angular.z), e = CW
STOP_KEYS = {'k', ' '}
SPEED_DOWN_KEY = 'z'
SPEED_UP_KEY = 'x'

HELP_TEXT = """\
--------------------------- SOARM keyboard teleop ---------------------------
translation:  u  i  o      rotate in place:  q (CCW)   e (CW)
              j  k  l      stop:             k or space
              m  ,  .      speed:             z (-)     x (+)

Hold a key to keep moving (terminal auto-repeat); release and motion decays
to zero. Combine translation with tapped q/e to arc. Ctrl-C quits safely.
-----------------------------------------------------------------------------\
"""

STATUS_WIDTH = 78


class TeleopPolicy:
    """Key-to-Twist mapping, terminal- and ROS-agnostic."""

    def __init__(self, max_linear, max_angular, hold_timeout=1.0, speed_levels=10):
        for name, value in (('max_linear', max_linear), ('max_angular', max_angular),
                            ('hold_timeout', hold_timeout)):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if not isinstance(speed_levels, int) or not 1 <= speed_levels <= 100:
            raise ValueError('speed_levels must be an int in [1, 100]')
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        self.hold_timeout = float(hold_timeout)
        self.speed_levels = speed_levels
        self.speed_level = speed_levels
        self._key_receipts = {}

    @property
    def scale(self):
        return self.speed_level / self.speed_levels

    def handle_key(self, key, now):
        """Apply one key event; returns True when the key was recognized."""
        if key in STOP_KEYS:
            self._key_receipts.clear()
            return True
        if key == SPEED_UP_KEY:
            self.speed_level = min(self.speed_levels, self.speed_level + 1)
            return True
        if key == SPEED_DOWN_KEY:
            self.speed_level = max(1, self.speed_level - 1)
            return True
        if key in TRANSLATION_KEYS or key in ROTATE_KEYS:
            self._key_receipts[key] = now
            return True
        return False

    def active_keys(self, now):
        return sorted(key for key, receipt in self._key_receipts.items()
                      if 0.0 <= now - receipt < self.hold_timeout)

    def command(self, now):
        """Return (linear_x, linear_y, angular_z) after pruning expired keys."""
        expired = [key for key, receipt in self._key_receipts.items()
                   if not 0.0 <= now - receipt < self.hold_timeout]
        for key in expired:
            del self._key_receipts[key]
        forward = sum(TRANSLATION_KEYS[key][0] for key in self._key_receipts if key in TRANSLATION_KEYS)
        left = sum(TRANSLATION_KEYS[key][1] for key in self._key_receipts if key in TRANSLATION_KEYS)
        rotate = sum(ROTATE_KEYS[key] for key in self._key_receipts if key in ROTATE_KEYS)
        forward = max(-1.0, min(1.0, forward))
        left = max(-1.0, min(1.0, left))
        rotate = max(-1.0, min(1.0, rotate))
        magnitude = math.hypot(forward, left)
        if magnitude > 1.0:
            forward, left = forward / magnitude, left / magnitude
        return (forward * self.max_linear * self.scale,
                left * self.max_linear * self.scale,
                rotate * self.max_angular * self.scale)


def build_twist(command):
    message = Twist()
    message.linear.x, message.linear.y = command[0], command[1]
    message.angular.z = command[2]
    return message


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('max_linear', 0.3)
        self.declare_parameter('max_angular', 0.8)
        self.declare_parameter('hold_timeout', 1.0)
        self.declare_parameter('speed_levels', 10)
        self.declare_parameter('require_tty', True)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError('publish_rate must be finite and positive')
        if self.get_parameter('require_tty').value and not sys.stdin.isatty():
            raise RuntimeError(
                'keyboard_teleop needs an interactive terminal for key input; '
                'set require_tty:=false to feed keys from a pipe (test mode)')
        self.policy = TeleopPolicy(
            float(self.get_parameter('max_linear').value),
            float(self.get_parameter('max_angular').value),
            float(self.get_parameter('hold_timeout').value),
            int(self.get_parameter('speed_levels').value),
        )
        self.lock = threading.Lock()
        self.publisher = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_timer(1.0 / publish_rate, self.publish_command)
        self._stdin_fd = None
        self._saved_termios = None

    def apply_key(self, key, now=None):
        with self.lock:
            return self.policy.handle_key(key, time.monotonic() if now is None else now)

    def publish_command(self):
        with self.lock:
            command = self.policy.command(time.monotonic())
        self.publisher.publish(build_twist(command))

    def print_status(self):
        with self.lock:
            command = self.policy.command(time.monotonic())
            keys = ''.join(self.policy.active_keys(time.monotonic())) or '-'
            scale = self.policy.scale
        line = (f'\r{scale * 100:3.0f}%  '
                f'vx {command[0]:+.2f}  vy {command[1]:+.2f} m/s  '
                f'wz {command[2]:+.2f} rad/s  keys [{keys}]')
        sys.stdout.write(line.ljust(STATUS_WIDTH)[:STATUS_WIDTH])
        sys.stdout.flush()

    def key_loop(self):
        if sys.stdin.isatty():
            self._stdin_fd = sys.stdin.fileno()
            self._saved_termios = termios.tcgetattr(self._stdin_fd)
            tty.setcbreak(self._stdin_fd)
        try:
            while rclpy.ok():
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not ready:
                    continue
                key = sys.stdin.read(1)
                if not key:
                    self.get_logger().warn('stdin closed; stopping teleop.')
                    self.publisher.publish(build_twist((0.0, 0.0, 0.0)))
                    rclpy.try_shutdown()
                    return
                if self.apply_key(key):
                    self.print_status()
        finally:
            if self._saved_termios is not None and self._stdin_fd is not None:
                termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._saved_termios)

    def restore_terminal(self):
        if self._saved_termios is not None and self._stdin_fd is not None:
            try:
                termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._saved_termios)
            except termios.error:
                pass


def main(args=None):
    rclpy.init(args=args)
    try:
        node = KeyboardTeleop()
    except Exception:
        rclpy.try_shutdown()
        raise
    print(HELP_TEXT)
    reader = threading.Thread(target=node.key_loop)
    reader.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publisher.publish(build_twist((0.0, 0.0, 0.0)))
        except Exception:
            pass  # the reader may already have shut the context down on EOF
        rclpy.try_shutdown()
        reader.join(timeout=2.0)
        node.restore_terminal()
        node.destroy_node()
        print()


if __name__ == '__main__':
    main()
