import json
import unittest
from unittest.mock import Mock

try:
    from backend.mavlink_bridge import (
        MAVLinkBridge,
        is_target_fcu_heartbeat,
        is_vehicle_heartbeat,
        logger,
        rover_manual_control_axes,
    )
except SystemExit as exc:
    raise unittest.SkipTest(f'pymavlink bridge dependencies unavailable: {exc}') from exc

logger.disabled = True


class FakeHeartbeat:
    def __init__(self, system=1, component=1, vehicle_type=10, autopilot=3):
        self.type = vehicle_type
        self.autopilot = autopilot
        self._system = system
        self._component = component

    def get_type(self):
        return 'HEARTBEAT'

    def get_srcSystem(self):
        return self._system

    def get_srcComponent(self):
        return self._component


class RoverManualControlTest(unittest.TestCase):
    def test_accepts_only_the_target_vehicle_heartbeat(self):
        self.assertTrue(is_target_fcu_heartbeat(FakeHeartbeat(), 1, 1))
        self.assertFalse(is_target_fcu_heartbeat(FakeHeartbeat(system=255), 1, 1))
        self.assertFalse(is_target_fcu_heartbeat(FakeHeartbeat(component=190), 1, 1))

    def test_rejects_mission_planner_gcs_heartbeat(self):
        gcs = FakeHeartbeat(system=255, component=190, vehicle_type=6, autopilot=8)
        self.assertFalse(is_vehicle_heartbeat(gcs))

    def test_axis_endpoints_and_neutral(self):
        self.assertEqual(rover_manual_control_axes(-100, -45), (-1000, -1000))
        self.assertEqual(rover_manual_control_axes(0, 0), (0, 0))
        self.assertEqual(rover_manual_control_axes(100, 45), (1000, 1000))
        self.assertEqual(rover_manual_control_axes(-5, 0), (0, -50))
        self.assertEqual(rover_manual_control_axes(5, 0), (0, 50))

    def test_axis_values_are_clamped(self):
        self.assertEqual(rover_manual_control_axes(-200, 90), (1000, -1000))
        self.assertEqual(rover_manual_control_axes(200, -90), (-1000, 1000))

    def test_send_uses_rover_manual_control_axes(self):
        bridge = MAVLinkBridge.__new__(MAVLinkBridge)
        bridge.master = Mock()
        bridge.target_system = 1
        bridge.rover_throttle_min = -100.0
        bridge.rover_throttle_max = 100.0
        bridge.rover_steering_min = -45.0
        bridge.rover_steering_max = 45.0

        self.assertTrue(bridge._send_rover_drive(25, -22.5))
        bridge.master.mav.manual_control_send.assert_called_once_with(
            1,
            32767,
            -500,
            250,
            32767,
            0,
        )

    def test_emergency_stop_neutralizes_then_disarms(self):
        bridge = MAVLinkBridge.__new__(MAVLinkBridge)
        bridge._send_rover_drive = Mock(return_value=True)
        bridge._arm_disarm = Mock(return_value=True)
        packet = json.dumps({'command': 'EMERGENCY_STOP', 'params': {}}).encode('utf-8')

        bridge._handle_command_packet(packet)

        bridge._send_rover_drive.assert_called_once_with(0.0, 0.0)
        bridge._arm_disarm.assert_called_once_with(False)


if __name__ == '__main__':
    unittest.main()
