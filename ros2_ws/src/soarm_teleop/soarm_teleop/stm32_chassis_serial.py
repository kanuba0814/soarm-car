"""Serial adapter bridging ``/safe_cmd_vel`` to the STM32F405 mecanum chassis.

This node is the only writer to the chassis USART1 link (115200-8N1, binary
frames per ``chassis_protocol``). It owns the ``MODE_PC`` lifecycle so the
SBUS remote handset regains control when the adapter exits:

- on connect: ``CMD_STOP`` then ``CMD_SET_MODE(MODE_PC)``
  (the delivery doc requires a stop before every mode switch);
- on shutdown: ``CMD_STOP`` then ``CMD_SET_MODE(MODE_REMOTE)``.

While commands are fresh the adapter streams ``CMD_SET_VELOCITY`` frames at
``send_period`` (default 50 ms, inside the doc's 50-100 ms recommendation);
each frame refreshes the firmware's 500 ms command watchdog. When commands
go stale it sends one ``CMD_STOP`` and then stays silent - it never
heartbeats past a dead command source, so the firmware watchdog stays an
honest fail-safe. Units: Twist arrives in m/s and rad/s, the chassis wants
mm/s and mrad/s. ``wz_scale`` compensates the uncalibrated yaw rate
(delivery doc section 8.2) and defaults to 1.0.
"""

import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from . import chassis_protocol as protocol

MAX_RX_BUFFER = 4096
MIN_STATUS_POLL_PERIOD = 0.2  # delivery doc section 8.5: poll at most 5 Hz


class ChassisLinkPolicy:
    """Frame scheduling for the chassis link, ROS- and transport-agnostic."""

    def __init__(self, command_timeout=0.5, wz_scale=1.0,
                 max_linear=0.5, max_angular=2.0, status_poll_period=0.0):
        for name, value in (('command_timeout', command_timeout),
                            ('max_linear', max_linear), ('max_angular', max_angular)):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if not math.isfinite(wz_scale) or wz_scale <= 0.0:
            raise ValueError('wz_scale must be finite and positive')
        if status_poll_period and status_poll_period < MIN_STATUS_POLL_PERIOD:
            raise ValueError(f'status_poll_period must be 0 or >= {MIN_STATUS_POLL_PERIOD} s')
        self.command_timeout = float(command_timeout)
        self.wz_scale = float(wz_scale)
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        self.status_poll_period = float(status_poll_period)
        self.command = None
        self.command_time = None
        self.stopped_for_stale = False
        self.last_status_poll = None
        self.rx_buffer = b''

    def reset(self):
        """Drop command state; the link owner must re-handshake afterwards."""
        self.command = None
        self.command_time = None
        self.stopped_for_stale = False
        self.last_status_poll = None
        self.rx_buffer = b''

    def update_command(self, vx, vy, wz, now):
        self.command = (vx, vy, wz)
        self.command_time = now

    def enter_pc_mode_frames(self):
        return [protocol.encode_stop(), protocol.encode_set_mode(protocol.MODE_PC)]

    def release_frames(self):
        return [protocol.encode_stop(), protocol.encode_set_mode(protocol.MODE_REMOTE)]

    def command_fresh(self, now):
        return (self.command is not None and self.command_time is not None and
                0.0 <= now - self.command_time < self.command_timeout)

    def _to_chassis_units(self, command):
        vx, vy, wz = command
        linear = [v if math.isfinite(v) else 0.0 for v in (vx, vy)]
        wz = wz if math.isfinite(wz) else 0.0
        vx_mm = max(-self.max_linear, min(self.max_linear, linear[0])) * 1000.0
        vy_mm = max(-self.max_linear, min(self.max_linear, linear[1])) * 1000.0
        wz_mrad = max(-self.max_angular, min(self.max_angular, wz)) * 1000.0 * self.wz_scale
        return vx_mm, vy_mm, wz_mrad

    def tick(self, now):
        """Return the frames that should be written for this period."""
        frames = []
        if self.command_fresh(now):
            frames.append(protocol.encode_set_velocity(*self._to_chassis_units(self.command)))
            self.stopped_for_stale = False
        elif not self.stopped_for_stale:
            frames.append(protocol.encode_stop())
            self.stopped_for_stale = True
        if self.status_poll_period > 0.0 and (
                self.last_status_poll is None or
                now - self.last_status_poll >= self.status_poll_period):
            frames.append(protocol.encode_get_status())
            self.last_status_poll = now
        return frames

    def feed_rx(self, data):
        """Buffer inbound bytes; returns decoded ack payloads as dicts."""
        self.rx_buffer = (self.rx_buffer + bytes(data))[-MAX_RX_BUFFER:]
        frames, remainder = protocol.parse_frames(self.rx_buffer)
        self.rx_buffer = remainder
        acks = []
        for cmd, payload in frames:
            if cmd == protocol.CMD_STATUS_ACK:
                acks.append(('status', protocol.decode_status_ack(payload)))
            elif cmd == protocol.CMD_MOVE_ACK:
                acks.append(('move', protocol.decode_move_ack(payload)))
        return acks


class STM32ChassisSerial(Node):

    def __init__(self):
        super().__init__('stm32_chassis_serial')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('send_period', 0.05)
        self.declare_parameter('wz_scale', 1.0)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 2.0)
        self.declare_parameter('status_poll_period', 0.0)
        self.declare_parameter('reconnect_period', 1.0)
        self.port_name = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        send_period = float(self.get_parameter('send_period').value)
        reconnect_period = float(self.get_parameter('reconnect_period').value)
        for name, value in (('send_period', send_period),
                            ('reconnect_period', reconnect_period)):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if send_period > 0.4:
            raise ValueError('send_period must stay below the 500 ms chassis watchdog')
        self.reconnect_period = reconnect_period
        self.policy = ChassisLinkPolicy(
            command_timeout=float(self.get_parameter('command_timeout').value),
            wz_scale=float(self.get_parameter('wz_scale').value),
            max_linear=float(self.get_parameter('max_linear').value),
            max_angular=float(self.get_parameter('max_angular').value),
            status_poll_period=float(self.get_parameter('status_poll_period').value),
        )
        self.serial = None
        self.last_connect_attempt = 0.0
        self.create_subscription(Twist, '/safe_cmd_vel', self.save_command, 10)
        self.create_timer(send_period, self.service_link)

    def save_command(self, message):
        self.policy.update_command(
            message.linear.x, message.linear.y, message.angular.z, time.monotonic())

    def open_port(self):
        try:
            import serial
        except ImportError:
            self.get_logger().fatal(
                'pyserial is not installed in this environment (apt: python3-serial)')
            return False
        try:
            self.serial = serial.Serial(self.port_name, self.baud,
                                        timeout=0.0, write_timeout=0.2)
        except Exception as error:
            self.get_logger().warning(
                f'cannot open chassis serial port {self.port_name}: {error}', throttle_duration_sec=5.0)
            self.serial = None
            return False
        return True

    def close_port(self):
        if self.serial is None:
            return
        try:
            self.serial.close()
        except Exception:  # pragma: no cover - close best effort
            pass
        self.serial = None
        self.policy.reset()

    def service_link(self):
        now = time.monotonic()
        if self.serial is None:
            if now - self.last_connect_attempt < self.reconnect_period:
                return
            self.last_connect_attempt = now
            if not self.open_port():
                return
            if not self.write_frames(self.policy.enter_pc_mode_frames()):
                return
            self.get_logger().info(
                f'chassis link open on {self.port_name}; stopped and entered MODE_PC')
            try:
                self.serial.reset_input_buffer()
            except Exception:  # pragma: no cover - best effort flush
                pass
        self.read_inbound()
        self.write_frames(self.policy.tick(now))

    def write_frames(self, frames):
        if self.serial is None:
            return False
        try:
            for frame in frames:
                self.serial.write(frame)
            return True
        except Exception as error:
            self.get_logger().error(f'chassis serial write failed: {error}')
            self.close_port()
            return False

    def read_inbound(self):
        try:
            chunk = self.serial.read(256)
        except Exception as error:
            self.get_logger().error(f'chassis serial read failed: {error}')
            self.close_port()
            return
        for kind, payload in self.policy.feed_rx(chunk or b''):
            if kind == 'status':
                self.handle_status_ack(payload)
            else:
                self.get_logger().info(
                    'chassis move segment finished: '
                    f"odom_x={payload['odom_x_mm']:.0f}mm odom_y={payload['odom_y_mm']:.0f}mm "
                    f"queue_left={payload['queue_left']}")

    def handle_status_ack(self, payload):
        target = ', '.join(f'{value:+.2f}' for value in payload['w_target'])
        actual = ', '.join(f'{value:+.2f}' for value in payload['w_actual'])
        self.get_logger().debug(
            f"chassis mode={payload['mode']} w_target=[{target}] w_actual=[{actual}] rad/s")
        if payload['mode'] != protocol.MODE_PC:
            self.get_logger().error(
                f"chassis reported mode={payload['mode']} while the adapter holds MODE_PC")

    def release_chassis(self):
        """Hand the chassis back to the remote handset (doc section 4.1)."""
        if self.serial is None:
            return
        try:
            for frame in self.policy.release_frames():
                self.serial.write(frame)
            self.serial.flush()
            self.get_logger().info('chassis stopped and returned to MODE_REMOTE')
        except Exception as error:  # pragma: no cover - release best effort
            self.get_logger().error(f'chassis release failed: {error}')
        self.close_port()


def main(args=None):
    rclpy.init(args=args)
    node = STM32ChassisSerial()

    def request_shutdown(signum, frame):
        del signum, frame
        rclpy.try_shutdown()

    signal.signal(signal.SIGTERM, request_shutdown)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.release_chassis()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
