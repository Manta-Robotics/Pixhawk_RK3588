import importlib.util
from pathlib import Path
import struct
import unittest

path=Path(__file__).resolve().parents[1]/'backend/uwb_usb_reader.py'
spec=importlib.util.spec_from_file_location('uwb_usb_reader',path)
reader=importlib.util.module_from_spec(spec)
spec.loader.exec_module(reader)

FRAME=bytes.fromhex('FF FF FF FF 00 25 0E 43 20 01 01 02 00 00 15 BD 00 00 15 BD 00 00 00 30 01 52 00 19 12 34 17 01 00 00 00 00 00')

class ParserTests(unittest.TestCase):
    def test_every_possible_split(self):
        for split in range(len(FRAME)+1):
            parser=reader.AlxParser()
            events=parser.feed(FRAME[:split])+parser.feed(FRAME[split:])
            self.assertEqual(len(events),1)
            self.assertEqual(events[0]['distanceCm'],48)
            self.assertEqual(events[0]['azimuthDeg'],338)

    def test_noise_bad_checksum_and_adjacent_frames_recover(self):
        parser=reader.AlxParser()
        bad=bytearray(FRAME);bad[20]^=1
        events=parser.feed(b'noise\xff'+bytes(bad)+FRAME+FRAME)
        self.assertEqual(len(events),2)
        self.assertEqual(parser.counts['checksumErrors'],1)
        self.assertLessEqual(len(parser.buffer),3)

    def test_heartbeat_is_not_a_position_and_unknown_version_is_rejected(self):
        parser=reader.AlxParser()
        hb=struct.pack('>4sHHHHI',b'\xff'*4,16,12,0x2002,0x0101,5565)
        events=parser.feed(hb)
        self.assertEqual(events[0]['type'],'heartbeat')
        self.assertEqual(parser.counts['goodCount'],0)
        unsupported=bytearray(FRAME);unsupported[11]=3
        checksum=0
        for value in unsupported[:-1]:checksum^=value
        unsupported[-1]=checksum
        self.assertEqual(parser.feed(unsupported),[])
        self.assertEqual(parser.counts['unsupportedVersions'],1)

    def test_extended_heartbeat_never_becomes_a_position(self):
        parser=reader.AlxParser()
        body=struct.pack('>4sHHHHI',b'\xff'*4,18,12,0x2002,0x0102,5565)+b'\x00'
        checksum=0
        for value in body:checksum^=value
        hb=body+bytes([checksum])
        events=parser.feed(hb)
        self.assertEqual(events[0]['type'],'heartbeat')
        self.assertEqual(parser.counts['goodCount'],0)

    def test_unbounded_noise_does_not_grow_receive_buffer(self):
        parser=reader.AlxParser()
        self.assertEqual(parser.feed(b'\x00'*100000),[])
        self.assertEqual(len(parser.buffer),3)
        self.assertEqual(len(parser.feed(FRAME)),1)

if __name__=='__main__':unittest.main()
