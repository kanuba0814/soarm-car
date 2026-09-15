#!/usr/bin/env python3

import importlib
import importlib.util
import struct
import sys
import unittest
from pathlib import Path

PACKAGE_DIR = Path(__file__).parents[1] / 'soarm_teleop'


def ensure_package():
    if 'soarm_teleop' not in sys.modules:
        spec = importlib.util.spec_from_file_location(
            'soarm_teleop', PACKAGE_DIR / '__init__.py',
            submodule_search_locations=[str(PACKAGE_DIR)])
        package = importlib.util.module_from_spec(spec)
        sys.modules['soarm_teleop'] = package
        spec.loader.exec_module(package)
    return sys.modules['soarm_teleop']


M = importlib.import_module('soarm_teleop.chassis_protocol')


class EncodeDocExampleFrames(unittest.TestCase):
    """Byte-exact frames from 上位机接口交付说明.md section 5."""

    def test_set_mode_pc(self):
        self.assertEqual(M.encode_set_mode(M.MODE_PC).hex(), 'aa5502020101fe')

    def test_stop(self):
        self.assertEqual(M.encode_stop().hex(), 'aa55010302fe')

    def test_set_velocity_1000mm(self):
        self.assertEqual(M.encode_set_velocity(1000.0, 0.0, 0.0).hex(),
                         'aa550d01' + '00007a44' + '00000000' + '00000000' + '32fe')

    def test_move_distance_1000mm(self):
        self.assertEqual(M.encode_move_distance(1000.0, 0.0).hex(),
                         'aa550904' + '00007a44' + '00000000' + '33fe')

    def test_get_status(self):
        self.assertEqual(M.encode_get_status().hex(), 'aa550110' + '11fe')

    def test_heartbeat(self):
        self.assertEqual(M.encode_heartbeat().hex(), 'aa55010001fe')


class EncodeValidation(unittest.TestCase):

    def test_data_is_limited_to_63_bytes(self):
        self.assertEqual(len(M.encode_frame(0x01, bytes(63))), 64 + 5)
        with self.assertRaises(ValueError):
            M.encode_frame(0x01, bytes(64))

    def test_unknown_mode_rejected(self):
        for mode in (-1, 3, 255):
            with self.assertRaises(ValueError):
                M.encode_set_mode(mode)

    def test_nonfinite_numbers_rejected(self):
        for bad in (float('nan'), float('inf'), float('-inf')):
            with self.assertRaises(ValueError):
                M.encode_set_velocity(bad, 0.0, 0.0)
            with self.assertRaises(ValueError):
                M.encode_move_distance(0.0, bad)

    def test_velocity_payload_roundtrip(self):
        frame = M.encode_set_velocity(100.5, -200.0, 750.0)
        frames, remainder = M.parse_frames(frame)
        self.assertEqual(len(frames), 1)
        cmd, payload = frames[0]
        self.assertEqual(cmd, M.CMD_SET_VELOCITY)
        self.assertEqual(struct.unpack('<fff', payload), (100.5, -200.0, 750.0))
        self.assertEqual(remainder, b'')


class ParseFrames(unittest.TestCase):

    def test_two_frames_back_to_back(self):
        stream = M.encode_stop() + M.encode_set_mode(M.MODE_PC)
        frames, remainder = M.parse_frames(stream)
        self.assertEqual(frames, [(M.CMD_STOP, b''), (M.CMD_SET_MODE, b'\x01')])
        self.assertEqual(remainder, b'')

    def test_garbage_prefix_is_skipped(self):
        stream = b'\x00\x01junk' + M.encode_stop()
        frames, remainder = M.parse_frames(stream)
        self.assertEqual(frames, [(M.CMD_STOP, b'')])
        self.assertEqual(remainder, b'')

    def test_bad_crc_frame_dropped_but_stream_resyncs(self):
        corrupted = bytearray(M.encode_stop())
        corrupted[-2] ^= 0xFF
        stream = bytes(corrupted) + M.encode_heartbeat()
        frames, _ = M.parse_frames(stream)
        self.assertEqual(frames, [(M.CMD_HEARTBEAT, b'')])

    def test_bad_tail_frame_dropped(self):
        corrupted = bytearray(M.encode_stop())
        corrupted[-1] = 0x00
        frames, _ = M.parse_frames(bytes(corrupted))
        self.assertEqual(frames, [])

    def test_len_zero_dropped(self):
        frames, remainder = M.parse_frames(b'\xaa\x55\x00\x00\xfe' + M.encode_stop())
        self.assertEqual(frames, [(M.CMD_STOP, b'')])
        self.assertEqual(remainder, b'')

    def test_len_over_64_dropped(self):
        stream = b'\xaa\x55\x41' + bytes(20) + M.encode_heartbeat()
        frames, remainder = M.parse_frames(stream)
        self.assertEqual(frames, [(M.CMD_HEARTBEAT, b'')])
        self.assertEqual(remainder, b'')

    def test_incomplete_frame_kept_as_remainder(self):
        whole = M.encode_set_velocity(1.0, 2.0, 3.0)
        frames, remainder = M.parse_frames(whole[:-1])
        self.assertEqual(frames, [])
        self.assertEqual(remainder, whole[:-1])
        frames, remainder = M.parse_frames(remainder + whole[-1:])
        self.assertEqual(len(frames), 1)
        self.assertEqual(remainder, b'')

    def test_split_frame_head_byte(self):
        whole = M.encode_stop()
        frames, remainder = M.parse_frames(b'\xff' + whole[:1])
        self.assertEqual((frames, remainder), ([], b'\xaa'))
        frames, remainder = M.parse_frames(remainder + whole[1:])
        self.assertEqual(frames, [(M.CMD_STOP, b'')])
        self.assertEqual(remainder, b'')

    def test_partial_frame_kept_behind_complete_frame(self):
        whole = M.encode_heartbeat()
        stream = whole + b'\xaa\x55\x01'
        frames, remainder = M.parse_frames(stream)
        self.assertEqual(frames, [(M.CMD_HEARTBEAT, b'')])
        self.assertEqual(remainder, b'\xaa\x55\x01')

    def test_parses_document_status_ack_frame(self):
        payload = bytearray(52)
        struct.pack_into('<BBH', payload, 16, M.MODE_PC, 0, 0)
        struct.pack_into('<4f', payload, 20, 1.5, -1.5, 2.5, -2.5)
        struct.pack_into('<4f', payload, 36, 0.5, 0.25, 0.125, 0.0625)
        frame = M.encode_frame(M.CMD_STATUS_ACK, bytes(payload))
        frames, _ = M.parse_frames(frame)
        self.assertEqual(frames, [(M.CMD_STATUS_ACK, bytes(payload))])


class DecodeAcks(unittest.TestCase):

    @staticmethod
    def make_status_payload(mode=M.MODE_PC):
        payload = bytearray(52)
        struct.pack_into('<BBH', payload, 16, mode, 0, 0)
        struct.pack_into('<4f', payload, 20, 1.0, 2.0, 3.0, 4.0)
        struct.pack_into('<4f', payload, 36, -0.125, 0.25, -0.5, 0.75)
        return bytes(payload)

    def test_status_ack_fields(self):
        decoded = M.decode_status_ack(self.make_status_payload(mode=M.MODE_REMOTE))
        self.assertEqual(decoded['mode'], M.MODE_REMOTE)
        self.assertEqual(decoded['w_target'], [1.0, 2.0, 3.0, 4.0])
        self.assertEqual(decoded['w_actual'], [-0.125, 0.25, -0.5, 0.75])

    def test_status_ack_requires_52_bytes(self):
        with self.assertRaises(ValueError):
            M.decode_status_ack(bytes(51))
        with self.assertRaises(ValueError):
            M.decode_status_ack(bytes(53))

    def test_move_ack_fields(self):
        payload = struct.pack('<ffB', 1234.5, -678.5, 2)
        decoded = M.decode_move_ack(payload)
        self.assertEqual(decoded, {'odom_x_mm': 1234.5, 'odom_y_mm': -678.5, 'queue_left': 2})

    def test_move_ack_requires_9_bytes(self):
        with self.assertRaises(ValueError):
            M.decode_move_ack(bytes(8))


if __name__ == '__main__':
    unittest.main()
