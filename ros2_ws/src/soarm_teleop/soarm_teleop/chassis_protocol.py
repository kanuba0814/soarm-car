"""Frame codec for the STM32F405 mecanum chassis serial protocol.

Frame layout (all fields little endian)::

    AA 55  LEN  CMD  DATA...  CRC  FE

- ``LEN`` counts ``CMD + DATA`` and must be 1..64, so ``DATA`` is at most
  63 bytes and the whole frame is ``LEN + 5`` bytes.
- ``CRC`` is the bytewise XOR of ``LEN + CMD + DATA``.
- The firmware drops malformed frames silently; there are no error frames.

Commands and struct layouts mirror ``protocol.h`` from the delivery bundle
``artifacts/下位机代码+测试脚本+接口文档.zip``; ``上位机接口交付说明.md``
in the same bundle is the authoritative reference. The example frames in
its section 5 are reproduced verbatim in this package's unit tests.
"""

import math
import struct

FRAME_HEAD = b'\xAA\x55'
FRAME_TAIL = b'\xFE'

CMD_HEARTBEAT = 0x00
CMD_SET_VELOCITY = 0x01
CMD_SET_MODE = 0x02
CMD_STOP = 0x03
CMD_MOVE_DISTANCE = 0x04
CMD_GET_STATUS = 0x10
CMD_STATUS_ACK = 0x11
CMD_MOVE_ACK = 0x12

MODE_REMOTE = 0
MODE_PC = 1
MODE_AUTO = 2

MIN_FRAME_LEN = 1
MAX_FRAME_LEN = 64
MAX_DATA_BYTES = MAX_FRAME_LEN - 1

STATUS_ACK_DATA_BYTES = 52
MOVE_ACK_DATA_BYTES = 9


def frame_crc(len_cmd_data):
    crc = 0
    for value in len_cmd_data:
        crc ^= value
    return crc & 0xFF


def encode_frame(cmd, data=b''):
    if not 0 <= int(cmd) <= 0xFF:
        raise ValueError(f'command id out of range: {cmd!r}')
    data = bytes(data)
    if len(data) > MAX_DATA_BYTES:
        raise ValueError(f'DATA is limited to {MAX_DATA_BYTES} bytes, got {len(data)}')
    body = bytes([1 + len(data), int(cmd)]) + data
    return FRAME_HEAD + body + bytes([frame_crc(body), FRAME_TAIL[0]])


def encode_heartbeat():
    return encode_frame(CMD_HEARTBEAT)


def encode_stop():
    return encode_frame(CMD_STOP)


def encode_set_mode(mode):
    if mode not in (MODE_REMOTE, MODE_PC, MODE_AUTO):
        raise ValueError(f'unknown chassis mode: {mode!r}')
    return encode_frame(CMD_SET_MODE, bytes([mode]))


def _pack_floats(values):
    packed = []
    for value in values:
        if not math.isfinite(value):
            raise ValueError(f'chassis commands must be finite, got {value!r}')
        packed.append(float(value))
    return struct.pack('<' + 'f' * len(packed), *packed)


def encode_set_velocity(vx_mm_s, vy_mm_s, wz_mrad_s):
    """Body is ``float vx, vy, wz``: mm/s, mm/s, mrad/s (CCW positive)."""
    return encode_frame(CMD_SET_VELOCITY, _pack_floats((vx_mm_s, vy_mm_s, wz_mrad_s)))


def encode_move_distance(dx_mm, dy_mm):
    """Body is ``float dx, dy`` in mm; positive dx = forward, dy = left."""
    return encode_frame(CMD_MOVE_DISTANCE, _pack_floats((dx_mm, dy_mm)))


def encode_get_status():
    return encode_frame(CMD_GET_STATUS)


def parse_frames(data):
    """Extract complete valid frames from ``data``.

    Returns ``(frames, remainder)`` where ``frames`` is a list of
    ``(cmd, data)`` tuples and ``remainder`` is the unconsumed tail (a
    possibly partial frame plus any bytes after it). Malformed frames are
    skipped byte-by-byte past the candidate head, mirroring the firmware's
    silent-drop behavior.
    """
    frames = []
    index = 0
    end = len(data)
    while index + 2 <= end:
        head = data.find(FRAME_HEAD, index)
        if head < 0:
            # No candidate head: drop scanned bytes, but keep a trailing lone
            # 0xAA that may be the first half of a split frame head.
            keep = 1 if data[end - 1:end] == FRAME_HEAD[:1] else 0
            return frames, bytes(data[end - keep:])
        if head + 3 > end:
            return frames, bytes(data[head:])
        length = data[head + 2]
        if not MIN_FRAME_LEN <= length <= MAX_FRAME_LEN:
            index = head + 1
            continue
        total = length + 5
        if head + total > end:
            return frames, bytes(data[head:])
        frame = data[head:head + total]
        if frame_crc(frame[2:-2]) == frame[-2] and frame[-1] == FRAME_TAIL[0]:
            frames.append((frame[3], bytes(frame[4:3 + length])))
            index = head + total
        else:
            index = head + 1
    return frames, bytes(data[index:])


def decode_status_ack(data):
    """Decode ``StatusFeedback_t``.

    Only ``mode`` and the wheel speed arrays are filled by the current
    firmware; ``vx``/``vy``/``wz``/``battery``/``status``/``error`` are
    placeholders that must not be relied on (delivery doc section 8.8).
    """
    data = bytes(data)
    if len(data) != STATUS_ACK_DATA_BYTES:
        raise ValueError(f'status ack payload must be {STATUS_ACK_DATA_BYTES} bytes, got {len(data)}')
    mode, _, _ = struct.unpack_from('<BBH', data, 16)
    w_target = struct.unpack_from('<4f', data, 20)
    w_actual = struct.unpack_from('<4f', data, 36)
    return {
        'mode': mode,
        'w_target': list(w_target),
        'w_actual': list(w_actual),
    }


def decode_move_ack(data):
    """Decode ``MoveAckFb_t``: odometry in mm plus the remaining queue depth."""
    data = bytes(data)
    if len(data) != MOVE_ACK_DATA_BYTES:
        raise ValueError(f'move ack payload must be {MOVE_ACK_DATA_BYTES} bytes, got {len(data)}')
    odom_x, odom_y = struct.unpack_from('<ff', data, 0)
    return {
        'odom_x_mm': odom_x,
        'odom_y_mm': odom_y,
        'queue_left': data[8],
    }
