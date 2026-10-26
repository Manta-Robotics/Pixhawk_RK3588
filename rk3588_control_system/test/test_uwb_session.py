"""Legacy wire framing and session reset in the Python differential reference."""
import pathlib
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'backend'))
from mavlink_bridge import MAVLinkBridge, FlightData, UWBData
from uwb_fusion import DualUwbFusion
from uwb_measurement_filter import GatedUwbKalman
from uwb_session import UwbSession


class UwbSessionTests(unittest.TestCase):
    def setUp(self):
        b = self.bridge = MAVLinkBridge.__new__(MAVLinkBridge)
        b.flight_data, b.uwb_data = FlightData(), UWBData()
        b.uwb_filter_enabled = True
        b.uwb_fusion = DualUwbFusion(enabled_receiver_ids=(1,), require_gated_measurements=True,
                                     mount_yaw_deg=(12, 90), receiver_position_x_m=(.1, .2))
        b._uwb_measurement_filters = {i: GatedUwbKalman(mount_yaw_deg=r.mount_yaw_deg)
                                      for i, r in b.uwb_fusion.receivers.items()}
        b._uwb_filters = {}
        b.connected = True
        b.master = None
        b.command_socket = b.telemetry_socket = Mock()
        b.mission_planner_clients, b.mission_planner_server = [], None
        b._send_node_packet = Mock()
        self.now, self.seq = 1000, 254

    def value(self, stamp, field, value):
        seq = self.seq
        self.seq = (seq + 1) % 256
        msg = SimpleNamespace(get_type=lambda: 'NAMED_VALUE_FLOAT',
                              get_srcSystem=lambda: 1, get_srcComponent=lambda: 1,
                              get_seq=lambda: seq, name='UWB1_' + field, value=value,
                              time_boot_ms=stamp & 0xFFFFFFFF)
        self.bridge.flight_data.attitude_updated_at = self.now
        with patch('mavlink_bridge.time.time', return_value=self.now / 1000), \
                patch('mavlink_bridge.time.monotonic', return_value=self.now / 1000):
            self.bridge._parse_message(msg)

    def pair(self, stamp, distance=3, legacy=True):
        self.value(stamp, 'DIST', distance)
        self.value(stamp + int(legacy), 'EL', 0)
        self.value(stamp + 2 * int(legacy), 'AZ', 0)

    def test_legacy_batch_processes_near_observation_and_rejects_replay(self):
        self.pair(100)
        self.pair(120, .2)
        r = self.bridge.uwb_fusion.receivers[1]
        self.assertEqual(r.raw_distance_m, .2)
        self.assertEqual(r.measurement_status, 'suspect')
        self.assertFalse(r.fresh(self.now, 450))
        self.now += 20
        self.pair(100)
        self.assertEqual(r.raw_distance_m, .2)

    def test_legacy_gap_and_strict_protocol_do_not_mix_frames(self):
        self.value(100, 'DIST', 3)
        self.seq = (self.seq + 1) % 256
        self.value(101, 'EL', 0)
        self.value(102, 'AZ', 0)
        self.assertIsNone(self.bridge.uwb_fusion.receivers[1].measurement_updated_at)
        self.value(110, 'PROTO', 2)
        self.pair(120)
        self.assertIsNone(self.bridge.uwb_fusion.receivers[1].measurement_updated_at)
        self.pair(140, legacy=False)
        self.assertEqual(self.bridge.uwb_fusion.receivers[1].measurement_updated_at, self.now)

    def test_received_telemetry_between_fields_is_not_treated_as_packet_loss(self):
        self.value(100, 'DIST', 3)
        seq = self.seq
        self.seq = (seq + 1) % 256
        self.bridge._parse_message(SimpleNamespace(
            get_type=lambda: 'SYSTEM_TIME', get_srcSystem=lambda: 1,
            get_srcComponent=lambda: 1, get_seq=lambda: seq))
        self.value(101, 'EL', 0)
        self.value(102, 'AZ', 0)
        self.assertEqual(self.bridge.uwb_fusion.receivers[1].measurement_updated_at, self.now)

    def test_disconnect_resets_clocks_protocol_filters_and_preserves_geometry(self):
        self.value(90, 'PROTO', 2)
        self.pair(10000, legacy=False)
        self.value(10001, 'VALID', 0)
        with patch('mavlink_bridge.time.time', return_value=self.now / 1000):
            self.bridge.disconnect()
        r = self.bridge.uwb_fusion.receivers[1]
        self.assertIsNone(r.raw_distance_m)
        self.assertIsNone(r.measurement_updated_at)
        self.assertFalse(r.newest_samples)
        self.assertFalse(r.strict_sources)
        self.assertEqual(r.mount_yaw_deg, 12)
        self.assertEqual(r.position_x_m, .1)
        self.assertIsNone(self.bridge.flight_data.attitude_updated_at)
        self.assertIsNone(self.bridge._uwb_measurement_filters[1].axes)
        self.pair(100, .2)
        self.assertTrue(r.fresh(self.now, 450))
        self.assertAlmostEqual(r.distance_m, .2)

    def test_disconnect_cannot_complete_a_pre_disconnect_partial_frame(self):
        self.value(100, 'DIST', 3)
        with patch('mavlink_bridge.time.time', return_value=self.now / 1000):
            self.bridge.disconnect()
        self.value(101, 'EL', 0)
        self.value(102, 'AZ', 0)
        self.assertIsNone(self.bridge.uwb_fusion.receivers[1].measurement_updated_at)
        self.pair(120)
        self.assertEqual(self.bridge.uwb_fusion.receivers[1].measurement_updated_at, self.now)

    def test_near_then_far_preserves_stop_even_with_missing_angles(self):
        self.pair(80)
        self.value(100, 'DIST', .2)
        self.pair(120)
        state = self.bridge._uwb_clearance.payload()
        self.assertTrue(state['clearanceStop'])
        self.assertAlmostEqual(state['clearanceDistanceM'], .2)
        self.assertEqual(self.bridge.uwb_fusion.receivers[1].raw_distance_m, 3)
        for i in range(1, 8):
            self.now += 100
            self.pair(120 + i * 100)
        self.assertTrue(self.bridge._uwb_clearance.payload()['clearanceStop'])
        self.bridge._uwb_clearance.acknowledge(self.now)
        for i in range(1, 7):
            self.now += 100
            self.pair(820 + i * 100)
        self.assertFalse(self.bridge._uwb_clearance.payload()['clearanceStop'])

    def sync(self, boot, nonce=None):
        self.bridge.target_system = self.bridge.target_component = 1
        session = self.bridge._uwb_session
        requested = session.request(self.now)
        msg = SimpleNamespace(get_type=lambda: 'TIMESYNC', get_srcSystem=lambda: 1,
                              get_srcComponent=lambda: 1, get_seq=lambda: self.seq,
                              tc1=boot * 1_000_000 + 1, ts1=requested if nonce is None else nonce)
        self.seq = (self.seq + 1) % 256
        with patch('mavlink_bridge.time.time', return_value=self.now / 1000), \
                patch('mavlink_bridge.time.monotonic', return_value=self.now / 1000):
            self.bridge._parse_message(msg)

    def test_fresh_reply_fences_old_complete_frame_after_reboot(self):
        self.bridge._uwb_session = UwbSession()
        self.sync(9999)
        self.pair(10000)
        self.bridge.disconnect()
        self.bridge.connected = True
        self.pair(10000)
        self.assertIsNone(self.bridge.uwb_fusion.receivers[1].raw_distance_m)
        self.sync(99)
        self.pair(10000)
        self.assertIsNone(self.bridge.uwb_fusion.receivers[1].raw_distance_m)
        self.pair(100, .2)
        self.assertTrue(self.bridge.uwb_fusion.receivers[1].fresh(self.now, 450))

    def test_old_tail_cannot_seed_pair_before_first_new_distance(self):
        self.bridge._uwb_session = UwbSession()
        self.sync(99)
        self.value(100, 'EL', 0)
        self.value(100, 'AZ', 30)
        self.value(100, 'DIST', .2)
        r = self.bridge.uwb_fusion.receivers[1]
        self.assertIsNone(r.measurement_updated_at)
        self.value(101, 'EL', 0)
        self.value(102, 'AZ', 0)
        self.assertEqual(r.raw_azimuth_deg, 0)

    def test_old_nonce_and_expired_response_cannot_open_session(self):
        self.bridge._uwb_session = UwbSession()
        self.sync(99, nonce=-1)
        self.pair(100)
        self.assertIsNone(self.bridge.uwb_fusion.receivers[1].raw_distance_m)
        session = self.bridge._uwb_session
        self.assertIsNone(session.reply(100_000_000, session.nonce, self.now + 501))
        old_nonce = session.nonce
        session.reset()
        self.sync(99, nonce=old_nonce)
        self.assertFalse(session.ready)


if __name__ == '__main__':
    unittest.main()
