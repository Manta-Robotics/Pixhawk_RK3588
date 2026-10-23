import struct
import unittest


def parse_vendor_position_frame(frame: bytes) -> dict:
    """Decode the UWB package's 0x2001 big-endian position frame."""
    if len(frame) < 12 or frame[:4] != b"\xff\xff\xff\xff":
        raise ValueError("bad frame header")
    packet_length = struct.unpack_from(">H", frame, 4)[0]
    if packet_length != len(frame):
        raise ValueError("bad frame length")
    if not frame or frame[-1] != _xor(frame[:-1]):
        raise ValueError("bad frame checksum")
    command = struct.unpack_from(">H", frame, 8)[0]
    if command != 0x2001 or packet_length != 37:
        raise ValueError("not a position frame")
    return {
        "sequence_id": struct.unpack_from(">H", frame, 6)[0],
        "anchor_id": struct.unpack_from(">I", frame, 12)[0],
        "tag_id": struct.unpack_from(">I", frame, 16)[0],
        "distance_m": struct.unpack_from(">I", frame, 20)[0] / 100.0,
        "azimuth_deg": struct.unpack_from(">h", frame, 24)[0],
        "elevation_deg": struct.unpack_from(">h", frame, 26)[0],
        "tag_status": struct.unpack_from(">H", frame, 28)[0],
    }


def _xor(data: bytes) -> int:
    result = 0
    for value in data:
        result ^= value
    return result


class VendorProtocolTests(unittest.TestCase):
    def test_decodes_first_package_sample(self):
        frame = bytes.fromhex(
            "FF FF FF FF 00 25 00 0B 20 01 01 00 00 00 AA A2 "
            "00 00 AA A1 00 00 00 19 00 12 FF CA 12 34 00 0B "
            "00 00 00 00 1E"
        )
        self.assertEqual(
            parse_vendor_position_frame(frame),
            {
                "sequence_id": 11,
                "anchor_id": 0xAAA2,
                "tag_id": 0xAAA1,
                "distance_m": 0.25,
                "azimuth_deg": 18,
                "elevation_deg": -54,
                "tag_status": 0x1234,
            },
        )

    def test_decodes_second_package_sample(self):
        frame = bytes.fromhex(
            "FF FF FF FF 00 25 00 27 20 01 01 00 00 00 AA A2 "
            "00 00 AA A1 00 00 00 62 FF D9 00 0D 12 34 00 27 "
            "00 00 00 00 69"
        )
        result = parse_vendor_position_frame(frame)
        self.assertEqual(result["sequence_id"], 39)
        self.assertEqual(result["distance_m"], 0.98)
        self.assertEqual(result["azimuth_deg"], -39)
        self.assertEqual(result["elevation_deg"], 13)

    def test_rejects_corrupted_checksum(self):
        frame = bytearray.fromhex(
            "FF FF FF FF 00 25 00 0B 20 01 01 00 00 00 AA A2 "
            "00 00 AA A1 00 00 00 19 00 12 FF CA 12 34 00 0B "
            "00 00 00 00 1E"
        )
        frame[20] ^= 1
        with self.assertRaisesRegex(ValueError, "checksum"):
            parse_vendor_position_frame(bytes(frame))


if __name__ == "__main__":
    unittest.main()
