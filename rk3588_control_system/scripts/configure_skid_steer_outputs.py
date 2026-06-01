#!/usr/bin/env python3

import argparse
import json
import sys
import time
from pathlib import Path

from pymavlink import mavutil


PROJECT_DIR = Path(__file__).resolve().parents[1]
CONFIG_PATH = PROJECT_DIR / 'config' / 'system.config.json'
EXPECTED_PARAMS = {
    'SERVO1_FUNCTION': 73,
    'SERVO3_FUNCTION': 74,
}
INFO_PARAMS = [
    'RCMAP_ROLL',
    'RCMAP_THROTTLE',
    'SERVO1_MIN',
    'SERVO1_TRIM',
    'SERVO1_MAX',
    'SERVO3_MIN',
    'SERVO3_TRIM',
    'SERVO3_MAX',
]


def load_config() -> dict:
    with CONFIG_PATH.open('r', encoding='utf-8') as handle:
        return json.load(handle)


def param_name(raw_name) -> str:
    if isinstance(raw_name, bytes):
        return raw_name.decode('utf-8', 'ignore').rstrip('\x00')
    return str(raw_name).rstrip('\x00')


def fetch_params(master, names, timeout=8.0):
    values = {}
    for name in names:
        master.param_fetch_one(name)

    deadline = time.time() + timeout
    while time.time() < deadline and len(values) < len(names):
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if not msg:
            continue
        name = param_name(msg.param_id)
        if name in names:
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
    parser = argparse.ArgumentParser(description='Check or apply Pixhawk skid-steer output mapping.')
    parser.add_argument('--apply', action='store_true', help='Write SERVO1_FUNCTION=73 and SERVO3_FUNCTION=74')
    args = parser.parse_args()

    config = load_config()
    serial_port = str(config.get('serial_port', '/dev/ttyS1'))
    baud_rate = int(config.get('baud_rate', 57600))

    print(f'Connecting to {serial_port} @ {baud_rate}')
    master = mavutil.mavlink_connection(serial_port, baud=baud_rate, source_system=255)
    master.wait_heartbeat(timeout=10)
    print(f'Heartbeat from system={master.target_system} component={master.target_component}')

    try:
        all_names = list(EXPECTED_PARAMS.keys()) + INFO_PARAMS
        before = fetch_params(master, all_names)

        print('Current critical params:')
        for name in EXPECTED_PARAMS:
            print(f'  {name}={before.get(name)}')

        if args.apply:
            print('Applying skid-steer output mapping...')
            for name, value in EXPECTED_PARAMS.items():
                result = set_param(master, name, value)
                print(f'  {name} -> {result}')

        after = fetch_params(master, all_names)

        print('Current rover I/O summary:')
        for name in all_names:
            print(f'  {name}={after.get(name)}')

        mismatches = [
            name for name, expected in EXPECTED_PARAMS.items()
            if round(float(after.get(name, -1))) != expected
        ]
        if mismatches:
            print(f'Mismatch remains: {", ".join(mismatches)}', file=sys.stderr)
            return 1

        print('Skid-steer output mapping is correct.')
        return 0
    finally:
        master.close()


if __name__ == '__main__':
    raise SystemExit(main())