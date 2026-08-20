#!/usr/bin/env python3

import argparse
import json
import sys
import time
from pathlib import Path

from pymavlink import mavutil


PROJECT_DIR = Path(__file__).resolve().parents[1]
CONFIG_PATH = PROJECT_DIR / 'config' / 'system.config.json'
MOTOR_CONFIG_PATH = PROJECT_DIR / 'config' / 'motor_config.json'


def load_json(path: Path) -> dict:
    with path.open('r', encoding='utf-8') as handle:
        return json.load(handle)


def build_expected_params(config: dict, motor_config: dict) -> dict[str, int]:
    left_channel = int(config.get('rover_left_channel', 1))
    right_channel = int(config.get('rover_right_channel', 3))
    left_input_channel = int(config.get('rover_left_input_channel', 1))
    right_input_channel = int(config.get('rover_right_input_channel', 3))
    motors = {
        int(item['channel']): item
        for item in motor_config.get('motors', [])
        if isinstance(item, dict) and str(item.get('channel', '')).isdigit()
    }

    expected = {
        'PILOT_STEER_TYPE': 1,
        'RCMAP_ROLL': left_input_channel,
        'RCMAP_THROTTLE': right_input_channel,
    }
    for output_channel, input_channel, function in (
        (left_channel, left_input_channel, 73),
        (right_channel, right_input_channel, 74),
    ):
        motor = motors.get(output_channel, {})
        expected.update({
            f'RC{input_channel}_MIN': int(motor.get('min_pwm', 1000)),
            f'RC{input_channel}_TRIM': int(motor.get('center_pwm', 1500)),
            f'RC{input_channel}_MAX': int(motor.get('max_pwm', 2000)),
            f'RC{input_channel}_REVERSED': 0,
            f'RC{input_channel}_OPTION': 0,
            f'SERVO{output_channel}_FUNCTION': function,
            f'SERVO{output_channel}_REVERSED': 1 if motor.get('servo_reversed', False) else 0,
            f'SERVO{output_channel}_MIN': int(motor.get('min_pwm', 1000)),
            f'SERVO{output_channel}_TRIM': int(motor.get('center_pwm', 1500)),
            f'SERVO{output_channel}_MAX': int(motor.get('max_pwm', 2000)),
        })
    return expected


def param_name(raw_name) -> str:
    if isinstance(raw_name, bytes):
        return raw_name.decode('utf-8', 'ignore').rstrip('\x00')
    return str(raw_name).rstrip('\x00')


def fetch_params(master, names, timeout=8.0):
    unique_names = list(dict.fromkeys(names))
    values = {}
    for name in unique_names:
        master.param_fetch_one(name)

    deadline = time.time() + timeout
    while time.time() < deadline and len(values) < len(unique_names):
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if not msg:
            continue
        name = param_name(msg.param_id)
        if name in unique_names:
            values[name] = msg.param_value

    return values


def set_param(master, name, value, timeout=5.0):
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode('utf-8'),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )

    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if not msg:
            continue
        returned_name = param_name(msg.param_id)
        if returned_name == name:
            return msg.param_value

    raise TimeoutError(f'No PARAM_VALUE ack for {name}')


def main() -> int:
    parser = argparse.ArgumentParser(description='Check or apply Pixhawk skid-steer direction mapping.')
    parser.add_argument(
        '--apply',
        action='store_true',
        help='Disable RC reversal and apply explicit Pixhawk SERVOx_REVERSED values from motor_config.json',
    )
    args = parser.parse_args()

    config = load_json(CONFIG_PATH)
    motor_config = load_json(MOTOR_CONFIG_PATH)
    expected_params = build_expected_params(config, motor_config)
    serial_port = str(config.get('serial_port', '/dev/ttyS1'))
    baud_rate = int(config.get('baud_rate', 57600))

    print('Pixhawk SERVO direction mapping (all software/RC reversal disabled):')
    active_channels = {
        int(config.get('rover_left_channel', 1)),
        int(config.get('rover_right_channel', 3)),
    }
    for motor in motor_config.get('motors', []):
        if int(motor.get('channel', 0)) in active_channels:
            print(f"  output {motor['channel']}: SERVO_REVERSED={int(bool(motor.get('servo_reversed', False)))}")

    print(f'Connecting to {serial_port} @ {baud_rate}')
    master = mavutil.mavlink_connection(serial_port, baud=baud_rate, source_system=255)
    heartbeat = master.wait_heartbeat(timeout=10)
    print(f'Heartbeat from system={master.target_system} component={master.target_component}')

    armed_flag = getattr(mavutil.mavlink, 'MAV_MODE_FLAG_SAFETY_ARMED', 128)
    if args.apply and int(getattr(heartbeat, 'base_mode', 0) or 0) & armed_flag:
        print('Refusing to apply motor parameters while Pixhawk is armed.', file=sys.stderr)
        master.close()
        return 2

    try:
        all_names = list(expected_params.keys())
        before = fetch_params(master, all_names)

        print('Current critical params:')
        for name in expected_params:
            print(f'  {name}={before.get(name)}')

        if args.apply:
            print('Applying normalized skid-steer mapping...')
            for name, value in expected_params.items():
                result = set_param(master, name, value)
                print(f'  {name} -> {result}')

        after = fetch_params(master, all_names)

        print('Current rover I/O summary:')
        for name in all_names:
            print(f'  {name}={after.get(name)}')

        mismatches = [
            name for name, expected in expected_params.items()
            if round(float(after.get(name, -1))) != expected
        ]
        if mismatches:
            print(f'Mismatch remains: {", ".join(mismatches)}', file=sys.stderr)
            return 1

        print('Skid-steer direction mapping is correct.')
        return 0
    finally:
        master.close()


if __name__ == '__main__':
    raise SystemExit(main())
