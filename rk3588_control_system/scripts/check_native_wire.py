#!/usr/bin/env python3
"""Exercise the native bridge with real MAVLink frames; never opens hardware."""
import json
import copy
import math
from pathlib import Path
import random
import subprocess
import sys
from unittest.mock import Mock, patch

from pymavlink.dialects.v20 import ardupilotmega as mav

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from mavlink_bridge import MAVLinkBridge, rover_manual_control_axes
from rover_pilot_mapping import motor_target_axes
from rover_acro_mapping import acro_axes, decode_axis
from check_native_parity import compare


def main():
    requests, checks = [], []
    now = 1_000_000
    sender = mav.MAVLink(None, srcSystem=1, srcComponent=1)
    decoder = mav.MAVLink(None)

    def append(op, check=lambda r: None, **args):
        requests.append(dict(op=op, wall_ms=now, mono_ms=now, **args))
        checks.append(check)

    def receive(message, check=lambda r: None, **kwargs):
        frame = message.pack(sender, **kwargs)
        sender.seq = (sender.seq + 1) % 256
        append('receive', check, hex=frame.hex())
        return frame

    def equal(actual, expected):
        compare(expected, actual)

    def command(name, params, check):
        def check_command(result):
            assert result['result']['ok'], result['result']
            frames = [decoder.parse_char(bytes.fromhex(x)) for x in result['frames']]
            check(frames, result)
        append('command', check_command, request=dict(command=name, params=params))

    # Clock corrections affect only published epochs, never sample age or
    # duration. Cover nested sensors, RPC state, second/millisecond timestamps,
    # null/zero sentinels, stale frames, and frames already in the future.
    for offset in [-3_600_000, -3000, -600, 0, 600, 3000, 3_600_000]:
        clock = 1_792_894_900_000
        packet = dict(timestamp=clock/1000, payload=dict(
            updatedAt=clock-20, timestamp=clock-10, receivedMeasurementAt=clock-900,
            sensors=[dict(rawDistanceUpdatedAt=clock+100, measurementUpdatedAt=clock-50)],
            nullAt=None, zeroAt=0, solveMs=15, wait_timeout_ms=5000, goodCount=42))
        expected = copy.deepcopy(packet)
        expected['timestamp'] += offset/1000
        for key in ['updatedAt', 'timestamp', 'receivedMeasurementAt']:
            expected['payload'][key] += offset
        for key in expected['payload']['sensors'][0]:
            expected['payload']['sensors'][0][key] += offset
        requests.append(dict(op='export_clock', wall_ms=clock, system_ms=clock+offset, packet=packet))
        checks.append(lambda r, expected=expected: equal(r['result'], expected))

    hb = mav.MAVLink_heartbeat_message(mav.MAV_TYPE_GROUND_ROVER, mav.MAV_AUTOPILOT_ARDUPILOTMEGA, 129, 0, 4, 3)
    receive(mav.MAVLink_heartbeat_message(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0, 3), lambda r: equal(r['connected'], False))
    receive(hb, lambda r: equal((r['connected'], r['state']['flightMode'], r['state']['armed']), (True, 'MANUAL', True)))
    sender.srcSystem = 2
    receive(mav.MAVLink_heartbeat_message(mav.MAV_TYPE_GROUND_ROVER, mav.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 15, 4, 3), lambda r: equal(r['state']['flightMode'], 'MANUAL'))
    sender.srcSystem = 1
    # Split a valid frame, reject a corrupt CRC, and accept MAVLink 1 as well.
    attitude = mav.MAVLink_attitude_message(100, .1, .2, .3, 0, 0, .4)
    frame = attitude.pack(sender)
    append('receive', hex=frame[:9].hex())
    append('receive', lambda r: equal(round(r['state']['attitude']['yaw'], 4), round(math.degrees(.3), 4)), hex=frame[9:].hex())
    broken = bytearray(frame); broken[-1] ^= 0x80
    now += 10
    append('receive', lambda r, expected=now-10: equal(r['state']['attitude']['updatedAt'], expected), hex=broken.hex())
    receive(attitude, lambda r: equal(r['state']['attitude']['updatedAt'], 1_000_010), force_mavlink1=True)

    rng = random.Random(724)
    for pilot in range(4):
        receive(mav.MAVLink_param_value_message(b'PILOT_STEER_TYPE', pilot, mav.MAV_PARAM_TYPE_REAL32, 1, 0))
        for _ in range(100):
            throttle, steering = rng.uniform(-140, 140), rng.uniform(-70, 70)
            expected = motor_target_axes(*rover_manual_control_axes(throttle, steering), pilot)
            command('ROVER_DRIVE', dict(throttle=throttle, steering=steering, motorTargetSemantics=True),
                    lambda frames, r, expected=expected: equal((frames[-1].y, frames[-1].z, frames[-1].x, frames[-1].r), (*expected, 32767, 32767)))
    command('ROVER_DRIVE', dict(throttle=40, steering=12), lambda f, r: None)
    now += 501
    def assert_neutral(r):
        frames = [decoder.parse_char(bytes.fromhex(x)) for x in r['frames']]
        controls = [m for m in frames if m.get_type() == 'MANUAL_CONTROL']
        assert controls and controls[-1].y == controls[-1].z == 0, r['frames']
    append('tick', assert_neutral)
    command('ROVER_BODY_VELOCITY', dict(forwardSpeedMps=2, yawRateRadS=.2),
            lambda f, r: equal((f[-1].coordinate_frame, f[-1].type_mask, f[-1].vx), (8, 1511, 2)))
    now += 501
    def assert_velocity_neutral(r):
        frames = [decoder.parse_char(bytes.fromhex(x)) for x in r['frames']]
        controls = [m for m in frames if m.get_type() == 'SET_POSITION_TARGET_LOCAL_NED']
        assert controls and controls[-1].vx == controls[-1].vy == controls[-1].yaw_rate == 0
    append('tick', assert_velocity_neutral)
    command('ARM', {}, lambda f, r: equal((f[-1].command, f[-1].param1, f[-1].param2), (400, 1, 0)))
    command('SET_MODE', {'mode': 'ACRO'}, lambda f, r: equal((f[-1].command, f[-1].param1, f[-1].param2), (176, 1, 1)))
    # Physical-unit ACRO mapping, including calibrated dead zones and reverse
    # channels, must match the old encoder without exceeding requested limits.
    receive(mav.MAVLink_heartbeat_message(mav.MAV_TYPE_GROUND_ROVER, mav.MAV_AUTOPILOT_ARDUPILOTMEGA, 129, 1, 4, 3))
    receive(mav.MAVLink_global_position_int_message(0, 300000000, 1200000000, 0, 0, 0, 0, 0, 0))
    receive(mav.MAVLink_gps_raw_int_message(0, 3, 300000000, 1200000000, 0, 100, 100, 0, 0, 12))
    receive(mav.MAVLink_ekf_status_report_message(18, .01, .01, .01, .01, .01))
    for pilot in [0, 1]:
        values = dict(PILOT_STEER_TYPE=pilot, SPEED_MAX=2, CRUISE_SPEED=1,
                      CRUISE_THROTTLE=50, ACRO_TURN_RATE=90, RCMAP_ROLL=1, RCMAP_THROTTLE=3)
        for channel in [1, 3]:
            values.update({f'RC{channel}_{key}': value for key, value in
                           dict(MIN=980, MAX=1990, TRIM=1505, DZ=40, REVERSED=pilot).items()})
        for name, value in values.items():
            receive(mav.MAVLink_param_value_message(name.encode(), value, mav.MAV_PARAM_TYPE_REAL32, len(values), 0))
        for _ in range(300):
            speed, yaw = rng.uniform(-3, 3), rng.uniform(-20, 20)
            expected = acro_axes(speed, yaw, values)
            # Python's set iteration can choose a different axis integer when
            # two encode to the same calibrated PWM. Compare FCU-decoded
            # physical values, which must be identical and bounded.
            def check_acro(frames, result, expected=expected, values=values.copy()):
                equal((decode_axis(frames[-1].y, values, 1), decode_axis(frames[-1].z, values, 3)),
                      (decode_axis(expected[0], values, 1), decode_axis(expected[1], values, 3)))
            command('ROVER_ACRO', dict(forwardSpeedMps=speed, yawRateDegS=yaw), check_acro)
    receive(mav.MAVLink_esc_telemetry_1_to_4_message([35, 0, 40, 0], [2400]*4, [100]*4, [1]*4, [1000]*4, [1]*4),
            lambda r: equal((r['state']['temperature']['motorLeft'], r['state']['temperature']['motorRight'],
                             r['state']['hardware']['motors']['left']['feedbackSource']), (35, 40, 'esc_telemetry')))
    command('IMU_CALIBRATION_START', {'type': 'ACCEL'}, lambda f, r: equal((f[-1].command, f[-1].param5, r['state']['imuCalibration']['status']), (241, 1, 'STARTING')))
    for code, label in enumerate(['LEVEL', 'LEFT', 'RIGHT', 'NOSEDOWN', 'NOSEUP', 'BACK'], 1):
        receive(mav.MAVLink_command_long_message(255, 0, 42429, 0, code, 0, 0, 0, 0, 0, 0),
                lambda r, label=label: equal((r['state']['imuCalibration']['status'], r['state']['imuCalibration']['step']), ('AWAITING_POSITION', label)))
        command('IMU_CALIBRATION_CONFIRM', {'positionCode': code}, lambda f, r, code=code: equal((f[-1].command, f[-1].param1), (42429, code)))
        receive(mav.MAVLink_command_ack_message(42429, 0), lambda r: equal(r['state']['imuCalibration']['status'], 'IN_PROGRESS'))
    receive(mav.MAVLink_command_long_message(255, 0, 42429, 0, 16777215, 0, 0, 0, 0, 0, 0),
            lambda r: equal((r['state']['imuCalibration']['status'], r['state']['imuCalibration']['active']), ('SUCCESS', False)))
    command('IMU_CALIBRATION_START', {'type': 'LEVEL'}, lambda f, r: equal(f[-1].param5, 2))
    receive(mav.MAVLink_command_ack_message(241, 0), lambda r: equal(r['state']['imuCalibration']['status'], 'SUCCESS'))

    # Compare every telemetry/UWB field to the retained Python reference.
    for gated in [False, True]:
        config = {'uwb_dual': {'enabled_receiver_ids': [1]}, 'uwb_measurement_filter': {'enabled': gated}}
        append('reset', config=config)
        reference = MAVLinkBridge('offline-test', 115200, '127.0.0.1', 0, 9, 100, 1, 1, 3, -100, 100, -45, 45,
                                 uwb_dual_config=config['uwb_dual'], uwb_measurement_filter_config=config['uwb_measurement_filter'])
        reference.connected = True
        reference._send_node_packet = Mock()
        with patch('mavlink_bridge.time.time', side_effect=lambda: now/1000), patch('mavlink_bridge.time.monotonic', side_effect=lambda: now/1000):
            messages = [hb, attitude,
                        mav.MAVLink_global_position_int_message(0, 310000000, 1200000000, 1000, 2000, 100, 20, 0, 0),
                        mav.MAVLink_gps_raw_int_message(0, 3, 310000000, 1200000000, 1000, 100, 200, 105, 12000, 18),
                        mav.MAVLink_ekf_status_report_message(18, .1, .2, .3, .4, .5),
                        mav.MAVLink_sys_status_message(0, 0, 0, 0, 12000, 500, 80, 0, 0, 0, 0, 0, 0),
                        mav.MAVLink_servo_output_raw_message(0, 0, 1510, 1500, 1490, 1500, 0, 0, 0, 0)]
            for i in range(80):
                messages.extend([mav.MAVLink_named_value_float_message(i, b'UWB1_DIST', 3+.01*i),
                                 mav.MAVLink_named_value_float_message(i, b'UWB1_AZ', 5*math.sin(i/20))])
            messages.append(mav.MAVLink_named_value_float_message(81, b'UWB1_VALID', 0))
            for message in messages:
                now += 10
                frame = message.pack(sender)
                decoded = decoder.parse_char(frame)
                reference._parse_message(decoded)
                expected = copy.deepcopy(reference._telemetry_payload())
                # Matching timestamp precision and neutral defaults are part of
                # the API contract; initial empty calibration state is separate.
                for key in ['imuCalibration', 'acroControl']:
                    expected.pop(key)
                def compare_state(r, expected=expected):
                    actual = {k: v for k, v in r['state'].items() if k not in ['imuCalibration', 'acroControl']}
                    compare(expected, actual)
                append('receive', compare_state, hex=frame.hex())
        reference.command_socket.close(); reference.telemetry_socket.close()

    now += 4000
    append('tick', lambda r: equal((r['connected'], r['state']['armed']), (False, False)))
    append('command', lambda r: equal(r['result']['ok'], False), request={'command': 'ARM'})
    run = subprocess.run([sys.argv[1]], input=''.join(json.dumps(r)+'\n' for r in requests), text=True, capture_output=True, timeout=60)
    if run.returncode:
        completed = len(run.stdout.splitlines())
        raise AssertionError((run.returncode, completed, requests[completed], run.stderr))
    results = [json.loads(line) for line in run.stdout.splitlines()]
    assert len(results) == len(requests), (len(results), len(requests), run.stderr)
    for i, (result, check) in enumerate(zip(results, checks)):
        try:
            assert 'error' not in result, result
            check(result)
        except Exception as exc:
            raise AssertionError(f'wire case {i}: {requests[i]}: {exc}') from exc
    print(f'Native MAVLink: {len(results)} wire, telemetry, mapping, calibration and watchdog cases passed')


if __name__ == '__main__':
    main()
