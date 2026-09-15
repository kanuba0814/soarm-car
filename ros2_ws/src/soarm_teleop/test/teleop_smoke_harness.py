#!/usr/bin/env python3
"""Headless smoke harness: keyboard topic wiring + chassis PTY loopback.

Runs inside the ros2-dev container against the built workspace. It never
touches real hardware: the chassis end of the serial link is a PTY pair,
and the harness acts as the gate by publishing /safe_cmd_vel directly.

Checks:
1. keyboard_teleop (keys fed from a pipe) publishes nonzero forward Twist
   on /cmd_vel and stops on the stop key / EOF.
2. stm32_chassis_serial handshake order (STOP -> MODE_PC), velocity frame
   unit conversion (m/s -> mm/s), stale-command stop, and the shutdown
   handback (STOP -> MODE_REMOTE).
"""

import os
import pty
import select
import signal
import struct
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from soarm_teleop import chassis_protocol  # noqa: E402

ROS_ENV = 'source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash'


def run_in_shell(command, timeout):
    return subprocess.run(['bash', '-lc', f'{ROS_ENV} && {command}'],
                          capture_output=True, text=True, timeout=timeout)


def check_keyboard_topic():
    rclpy.init()
    node = Node('teleop_smoke_probe')
    samples = []
    node.create_subscription(Twist, '/cmd_vel',
                             lambda message: samples.append(message.linear.x), 20)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    time.sleep(1.5)
    script = "(printf 'i'; sleep 1.2; printf 'k'; sleep 0.4) | " \
             "timeout 15 ros2 run soarm_teleop keyboard_teleop --ros-args -p require_tty:=false"
    result = run_in_shell(script, timeout=30)
    time.sleep(0.5)
    rclpy.shutdown()
    spinner.join(timeout=5.0)
    node.destroy_node()
    if result.returncode != 0:
        print('keyboard_teleop exited nonzero:', result.returncode)
        print(result.stdout)
        print(result.stderr)
        return False
    moving = [x for x in samples if x > 0.29]
    tail_all_zero = len(samples) >= 3 and all(abs(x) < 1e-9 for x in samples[-3:])
    print(f'keyboard /cmd_vel samples={len(samples)} moving={len(moving)} '
          f'tail_all_zero={tail_all_zero}')
    return len(moving) >= 10 and tail_all_zero


def check_chassis_pty():
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    received = bytearray()
    stop_reader = threading.Event()

    def reader():
        while not stop_reader.is_set():
            ready, _, _ = select.select([master_fd], [], [], 0.1)
            if ready:
                try:
                    received.extend(os.read(master_fd, 512))
                except OSError:
                    break

    reader_thread = threading.Thread(target=reader, daemon=True)
    reader_thread.start()

    adapter = subprocess.Popen(
        ['bash', '-lc',
         f'{ROS_ENV} && ros2 run soarm_teleop stm32_chassis_serial '
         f'--ros-args -p port:={slave_name}'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        time.sleep(2.5)  # node start, port open, handshake
        publisher = subprocess.Popen(
            ['bash', '-lc',
             f"{ROS_ENV} && ros2 topic pub -r 20 /safe_cmd_vel "
             f"geometry_msgs/msg/Twist '{{linear: {{x: 0.1}}}}'"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        try:
            time.sleep(2.5)  # stream velocity frames
        finally:
            os.killpg(publisher.pid, signal.SIGINT)
            publisher.wait(timeout=10)
        time.sleep(1.5)  # command goes stale -> single STOP
        os.killpg(adapter.pid, signal.SIGINT)
        adapter_output = adapter.communicate(timeout=15)[0].decode('utf-8', 'replace')
    finally:
        if adapter.poll() is None:
            os.killpg(adapter.pid, signal.SIGKILL)
            adapter.wait(timeout=10)
        stop_reader.set()
        reader_thread.join(timeout=2.0)

    if adapter.returncode != 0:
        print('stm32_chassis_serial exited nonzero:', adapter.returncode)
        print(adapter_output)
        return False

    frames, _ = chassis_protocol.parse_frames(bytes(received))
    kinds = [cmd for cmd, _ in frames]
    print('chassis frames:', [f'{cmd:02x}' for cmd in kinds])

    velocity_indices = [i for i, cmd in enumerate(kinds)
                        if cmd == chassis_protocol.CMD_SET_VELOCITY]
    velocity_payloads = [struct.unpack('<fff', frames[i][1]) for i in velocity_indices]
    checks = {
        'handshake_stop_then_pc_mode':
            kinds[:2] == [chassis_protocol.CMD_STOP, chassis_protocol.CMD_SET_MODE]
            and frames[1][1] == bytes([chassis_protocol.MODE_PC]),
        'velocity_frames_stream_100mm_s':
            bool(velocity_payloads) and
            all(abs(vx - 100.0) < 1e-3 and abs(vy) < 1e-3 and abs(wz) < 1e-3
                for vx, vy, wz in velocity_payloads),
        'stop_after_last_velocity':
            bool(velocity_indices) and
            any(cmd == chassis_protocol.CMD_STOP
                for cmd in kinds[velocity_indices[-1] + 1:]),
        'shutdown_stop_then_remote_mode':
            kinds[-2:] == [chassis_protocol.CMD_STOP, chassis_protocol.CMD_SET_MODE]
            and frames[-1][1] == bytes([chassis_protocol.MODE_REMOTE]),
    }
    for name, ok in checks.items():
        print(f'  {name}: {"ok" if ok else "FAILED"}')
    return all(checks.values())


def main():
    print('== keyboard_teleop /cmd_vel wiring ==')
    keyboard_ok = check_keyboard_topic()
    print('== stm32_chassis_serial PTY loopback ==')
    chassis_ok = check_chassis_pty()
    if not (keyboard_ok and chassis_ok):
        sys.exit(1)
    print('teleop smoke harness passed')


if __name__ == '__main__':
    main()
