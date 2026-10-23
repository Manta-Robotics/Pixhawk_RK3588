"""Regression checks for the real bridge's startup telemetry negotiation."""
import os
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'backend'))
from mavlink_bridge import MAVLinkBridge, MAV_CMD_SET_MESSAGE_INTERVAL


def packet(kind='COMMAND_ACK', command=MAV_CMD_SET_MESSAGE_INTERVAL, result=0, system=1, component=1):
    return SimpleNamespace(get_type=lambda: kind, get_srcSystem=lambda: system,
                           get_srcComponent=lambda: component, command=command, result=result)


class TelemetryIntervalsTests(unittest.TestCase):
    def bridge(self, messages):
        bridge = MAVLinkBridge.__new__(MAVLinkBridge)
        bridge.master = SimpleNamespace(mav=Mock(), recv_match=Mock(side_effect=messages))
        bridge.target_system = bridge.target_component = 1
        bridge._parse_message = Mock()
        bridge._broadcast_mission_planner = Mock()
        return bridge

    def test_interval_requests_wait_for_own_fcu_ack_and_preserve_incoming_telemetry(self):
        messages = [packet('ATTITUDE'), packet(system=2), packet(component=191),
                    packet(command=400), packet()]
        bridge = self.bridge(messages)
        self.assertTrue(bridge._set_message_interval(30, 20))
        self.assertEqual(bridge._parse_message.call_count, len(messages))
        self.assertEqual(bridge._broadcast_mission_planner.call_count, len(messages))
        self.assertEqual(bridge.master.mav.command_long_send.call_args.args[4:6], (30.0, 50000.0))

    def test_in_progress_requires_final_ack(self):
        bridge = self.bridge([packet(result=5), packet()])
        self.assertTrue(bridge._set_message_interval(30, 20))
        self.assertEqual(bridge.master.recv_match.call_count, 2)

    def test_temporary_rejection_retries(self):
        bridge = self.bridge([packet(result=1), packet()])
        self.assertTrue(bridge._set_message_interval(30, 20))
        self.assertEqual(bridge.master.mav.command_long_send.call_count, 2)

    def test_unsupported_message_does_not_retry_or_claim_success(self):
        bridge = self.bridge([packet(result=3)])
        self.assertFalse(bridge._set_message_interval(105, 10))
        self.assertEqual(bridge.master.mav.command_long_send.call_count, 1)

    def test_missing_ack_retries_then_fails_without_hanging(self):
        bridge = self.bridge([])
        bridge.master.recv_match.side_effect = None
        bridge.master.recv_match.return_value = None
        with patch('mavlink_bridge.time.monotonic', side_effect=[0, 0.1, 0.2, 2, 3, 3.1, 3.2, 5]):
            self.assertFalse(bridge._set_message_interval(30, 20))
        self.assertEqual(bridge.master.mav.command_long_send.call_count, 2)

    def test_stream_setup_never_sends_the_next_request_before_ack(self):
        bridge = self.bridge([])
        bridge.rover_left_output_channel = 1
        bridge.rover_right_output_channel = 3
        requests = []
        pending = []
        def send(*args):
            self.assertFalse(pending, 'a previous request is still unacknowledged')
            self.assertEqual(args[2], MAV_CMD_SET_MESSAGE_INTERVAL)
            requests.append((int(args[4]), int(args[5])))
            pending.append(packet())
        def receive(**kwargs):
            self.assertTrue(pending)
            return pending.pop()
        bridge.master.mav.command_long_send.side_effect = send
        bridge.master.recv_match.side_effect = receive
        bridge._configure_message_intervals()
        self.assertFalse(pending)
        self.assertIn((30, 50000), requests)
        self.assertIn((36, 50000), requests)
        self.assertIn((33, 100000), requests)
        self.assertFalse(any(message_id == 400 for message_id, _ in requests))

    def test_disconnected_or_invalid_rate_does_not_send(self):
        bridge = self.bridge([])
        self.assertFalse(bridge._set_message_interval(30, 0))
        bridge.master.mav.command_long_send.assert_not_called()
        bridge.master = None
        self.assertFalse(bridge._set_message_interval(30, 20))


if __name__ == '__main__':
    unittest.main()
