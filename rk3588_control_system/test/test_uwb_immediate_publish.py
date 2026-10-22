import pathlib
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'backend'))
from mavlink_bridge import MAVLinkBridge


class ImmediateUwbTests(unittest.TestCase):
    def setUp(self):
        self.bridge = MAVLinkBridge(
            serial_port='test-only', baudrate=115200, bridge_host='127.0.0.1',
            command_port=0, telemetry_port=9, telemetry_rate_hz=1,
            heartbeat_interval=1, rover_left_output_channel=1, rover_right_output_channel=3,
            rover_throttle_min=-100, rover_throttle_max=100,
            rover_steering_min=-45, rover_steering_max=45,
            uwb_dual_config={'enabled_receiver_ids': [1]},
            uwb_measurement_filter_config={'enabled': False})
        self.bridge._send_node_packet = Mock()

    def tearDown(self):
        self.bridge.command_socket.close()
        self.bridge.telemetry_socket.close()

    def test_complete_measurement_publishes_before_periodic_deadline_without_gps_or_imu(self):
        self.bridge._last_telemetry_sent = 10**20
        self.bridge._update_uwb_named_value('UWB1_DIST', 1.25)
        self.bridge._send_node_packet.assert_not_called()
        self.bridge._update_uwb_named_value('UWB1_AZ', 3)
        kind, payload = self.bridge._send_node_packet.call_args.args
        self.assertEqual(kind, 'uwb')
        self.assertTrue(payload['fresh'])
        self.assertEqual(payload['sensors'][0]['rawDistanceM'], 1.25)
        self.assertEqual(self.bridge.flight_data.gps_fix_type, 0)
        self.assertIsNone(self.bridge.flight_data.attitude_updated_at)
        self.assertEqual(self.bridge._send_node_packet.call_count, 1)

    def test_metadata_does_not_publish_a_fake_measurement_and_rejection_is_immediate(self):
        self.bridge._update_uwb_named_value('UWB1_DIST', 1.25)
        self.bridge._update_uwb_named_value('UWB1_AZ', 3)
        self.bridge._send_node_packet.reset_mock()
        self.bridge._update_uwb_named_value('UWB1_GOOD', 100)
        self.bridge._update_uwb_named_value('UWB1_SEQ', 100)
        self.bridge._send_node_packet.assert_not_called()
        self.bridge._update_uwb_named_value('UWB1_VALID', 0)
        self.assertFalse(self.bridge._send_node_packet.call_args.args[1]['fresh'])

    def test_busy_mavlink_and_command_sources_cannot_monopolize_the_loop(self):
        self.bridge.master = SimpleNamespace(recv_match=Mock(return_value=object()))
        self.bridge._broadcast_mission_planner = Mock()
        self.bridge._parse_message = Mock()
        with patch('mavlink_bridge.time.monotonic', return_value=0):
            self.bridge._poll_mavlink()
        self.assertEqual(self.bridge._parse_message.call_count, 256)
        self.bridge.command_socket.close()
        self.bridge.command_socket = Mock(recvfrom=Mock(return_value=(b'{}', None)))
        self.bridge._handle_command_packet = Mock()
        self.bridge._poll_commands()
        self.assertEqual(self.bridge._handle_command_packet.call_count, 32)


if __name__ == '__main__':
    unittest.main()
