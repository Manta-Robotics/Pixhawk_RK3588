import importlib.util
from pathlib import Path
import struct
import unittest


spec = importlib.util.spec_from_file_location('reader_radio', Path(__file__).resolve().parents[1]/'backend/uwb_usb_reader.py')
reader = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reader)

POSITION = bytes.fromhex('FF FF FF FF 00 25 0E 43 20 01 01 02 00 00 15 BD 00 00 15 BD 00 00 00 30 01 52 00 19 12 34 17 01 00 00 00 00 00')
# Actual receiver packet captured while the tag was occluded.
HEARTBEAT = bytes.fromhex('ff ff ff ff 00 12 b7 ba 20 02 01 02 00 00 15 bd 00 96')


class RadioParserTests(unittest.TestCase):
    def test_captured_heartbeat_and_position_survive_every_split(self):
        stream = HEARTBEAT+POSITION+HEARTBEAT+POSITION
        for split in range(len(stream)+1):
            p = reader.AlxParser()
            events = p.feed(stream[:split])+p.feed(stream[split:])
            self.assertEqual([e['type'] for e in events], ['heartbeat', 'position']*2)
            self.assertEqual(p.counts['goodCount'], 2)
            self.assertEqual(p.counts['heartbeatCount'], 2)
            self.assertEqual(p.counts['checksumErrors'], 0)

    def test_bad_heartbeat_cannot_invalidate_position_and_resynchronizes(self):
        bad = bytearray(HEARTBEAT)
        bad[12] ^= 1
        p = reader.AlxParser()
        events = p.feed(bytes(bad)+POSITION)
        self.assertEqual([e['type'] for e in events], ['position'])
        self.assertEqual(p.counts['heartbeatCount'], 0)
        self.assertEqual(p.counts['checksumErrors'], 1)

    def test_incomplete_unsupported_header_does_not_wait_for_a_position_sized_packet(self):
        bad = bytearray(POSITION[:12])
        bad[11] = 3
        p = reader.AlxParser()
        events = p.feed(bytes(bad)+HEARTBEAT)
        self.assertEqual([e['type'] for e in events], ['heartbeat'])
        self.assertEqual(p.counts['unsupportedVersions'], 1)

    def test_one_byte_reads_noise_and_bad_position_recover_to_the_next_frame(self):
        bad = bytearray(POSITION)
        bad[-1] ^= 1
        stream = b'noise'+HEARTBEAT+bytes(bad)+POSITION
        p = reader.AlxParser()
        events = []
        for value in stream:
            events.extend(p.feed(bytes([value])))
        self.assertEqual([e['type'] for e in events], ['heartbeat', 'position'])
        self.assertEqual(p.counts['checksumErrors'], 1)
        self.assertLessEqual(len(p.buffer), 3)

    def test_heartbeat_only_never_invents_position_and_radio_recovery_decodes_immediately(self):
        p = reader.AlxParser()
        self.assertTrue(all(e['type']=='heartbeat' for e in p.feed(HEARTBEAT*8)))
        self.assertEqual(p.counts['goodCount'], 0)
        position = p.feed(POSITION)
        self.assertEqual(len(position), 1)
        self.assertEqual(position[0]['distanceCm'], 48)
        self.assertEqual(position[0]['tagId'], 5565)


if __name__ == '__main__':
    unittest.main()
