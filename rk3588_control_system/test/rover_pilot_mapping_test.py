"""Exercise real bridge methods offline; no socket, serial port, or motor commands."""
import ast
import json
import logging
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace
from typing import Any
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'backend'))
from rover_pilot_mapping import motor_target_axes, valid_pilot_type

# Extract production methods without importing the hardware bridge's startup code.
tree = ast.parse((ROOT / 'backend/mavlink_bridge.py').read_text(encoding='utf-8'))
bridge_class = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'MAVLinkBridge')
methods = {'_send_rover_drive', '_handle_command_packet', '_parse_message', '_request_pilot_steer_type'}
nodes = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'rover_manual_control_axes']
nodes += [node for node in bridge_class.body if isinstance(node, ast.FunctionDef) and node.name in methods]
scope = {'Any': Any, 'math': math, 'json': json, 'time': time, 'logger': logging.getLogger('test'),
         'motor_target_axes': motor_target_axes, 'valid_pilot_type': valid_pilot_type}
exec(compile(ast.Module(body=nodes, type_ignores=[]), 'production_bridge', 'exec'), scope)
BridgeMethods = type('BridgeMethods', (), {name: scope[name] for name in methods})


def fake_bridge(pilot_type=0):
    bridge = BridgeMethods()
    bridge.master = SimpleNamespace(mav=Mock())
    bridge.pilot_steer_type = pilot_type
    bridge._pilot_param_requested_at = 0
    bridge.rover_throttle_min, bridge.rover_throttle_max = -100, 100
    bridge.rover_steering_min, bridge.rover_steering_max = -45, 45
    bridge.target_system, bridge.target_component = 1, 1
    bridge._send_node_log = Mock()
    return bridge


def decoded_motors(steering, throttle, pilot_type):
    # FCU pilot decoding followed by Rover's linear skid-steer mixer.
    if pilot_type in (0, 2) and throttle < 0:
        steering = -steering
    if pilot_type == 1:
        throttle, steering = (steering + throttle) / 2, (steering - throttle) / 2
    return throttle + steering, throttle - steering


class PilotMappingTests(unittest.TestCase):
    def test_all_firmware_conventions_keep_the_selected_propeller_in_both_directions(self):
        for pilot in range(4):
            for left, right in [(600, 0), (0, 600), (-600, 0), (0, -600), (600, 600), (-600, -600)]:
                bridge = fake_bridge(pilot)
                bridge._handle_command_packet(json.dumps({'command': 'ROVER_DRIVE', 'params': {
                    'throttle': (left + right) / 20,
                    'steering': (left - right) / 2000 * 45,
                    'motorTargetSemantics': True}}).encode())
                target, unused_x, y, z, unused_r, buttons = bridge.master.mav.manual_control_send.call_args.args
                self.assertEqual(decoded_motors(y, z, pilot), (left, right))
                self.assertEqual((target, unused_x, unused_r, buttons), (1, 32767, 32767, 0))

    def test_unknown_parameter_holds_non_neutral_request_at_neutral(self):
        bridge = fake_bridge(None)
        bridge._send_rover_drive(-30, -13.5, True)
        self.assertEqual(bridge.master.mav.manual_control_send.call_args.args[2:4], (0, 0))
        bridge._send_node_log.assert_called_once()

    def test_emergency_neutral_works_before_parameter_read(self):
        bridge = fake_bridge(None)
        self.assertTrue(bridge._send_rover_drive(0, 0, True))
        self.assertEqual(bridge.master.mav.manual_control_send.call_args.args[2:4], (0, 0))
        bridge._send_node_log.assert_not_called()

    def test_non_manual_packets_keep_the_existing_follow_protocol(self):
        bridge = fake_bridge(0)
        bridge._handle_command_packet(json.dumps({'command': 'ROVER_DRIVE', 'params': {
            'throttle': -30, 'steering': -13.5}}).encode())
        self.assertEqual(bridge.master.mav.manual_control_send.call_args.args[2:4], (-300, -300))

    def test_parameter_from_other_vehicle_or_component_cannot_change_mapping(self):
        bridge = fake_bridge(None)
        for system, component, name, value, expected in [(9, 1, 'PILOT_STEER_TYPE', 0, None),
                (1, 9, 'PILOT_STEER_TYPE', 0, None), (1, 1, 'OTHER_PARAM', 0, None),
                (1, 1, b'PILOT_STEER_TYPE\x00', 0, 0), (1, 1, 'PILOT_STEER_TYPE', 3, 3),
                (1, 1, 'PILOT_STEER_TYPE', 1.5, None)]:
            msg = SimpleNamespace(param_id=name, param_value=value, get_type=lambda: 'PARAM_VALUE',
                                  get_srcSystem=lambda: system, get_srcComponent=lambda: component)
            bridge._parse_message(msg)
            self.assertEqual(bridge.pilot_steer_type, expected)

    def test_parameter_discovery_sends_only_read_request_and_is_rate_limited(self):
        bridge = fake_bridge(None)
        bridge._request_pilot_steer_type()
        bridge._request_pilot_steer_type()
        self.assertEqual(bridge.master.mav.method_calls,
                         [('param_request_read_send', (1, 1, b'PILOT_STEER_TYPE', -1), {})])

    def test_bad_parameter_values_are_rejected(self):
        for value in [None, 'bad', float('nan'), float('inf'), -1, 1.1, 4]:
            self.assertIsNone(valid_pilot_type(value))


if __name__ == '__main__':
    unittest.main()
