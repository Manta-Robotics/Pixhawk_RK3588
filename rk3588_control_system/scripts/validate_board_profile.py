#!/usr/bin/env python3
"""Reject cross-chip boot overlays before an installer modifies boot files."""
import argparse
import json
import os
import re
from pathlib import Path
from rockchip_runtime import detect_soc


def validate_overlays(config, soc):
    if not soc:
        return []
    serial = str(config.get('serial_port', ''))
    usb_serial = serial.startswith(('/dev/ttyUSB', '/dev/ttyACM', '/dev/serial/'))
    profiles = [] if usb_serial else [('pixhawk', config.get('pixhawk', {}), 'uart_overlay')]
    if config.get('camera', {}).get('enabled', True):
        profiles.append(('camera', config['camera'], 'overlay'))
    gimbal = config.get('gimbal', {})
    if gimbal.get('control_transport', 'uart') == 'uart':
        profiles.append(('gimbal', gimbal, 'uart_overlay'))
    errors = []
    for name, profile, key in profiles:
        overlay = str(profile.get(key, ''))
        chip = re.search(r'rk\d{4}', overlay.lower())
        if chip and chip.group() != soc:
            errors.append(f'{name}.{key}={overlay} targets {chip.group()}, but the running board is {soc}; configure this board\'s UART/camera before installing overlays')
    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('config', nargs='?', default=os.environ.get('MANTA_CONFIG_FILE') or os.environ.get('MANTA_CONFIG_PATH') or str(Path(__file__).resolve().parents[1] / 'config/system.config.json'))
    args = parser.parse_args()
    errors = validate_overlays(json.loads(Path(args.config).read_text(encoding='utf-8')), detect_soc())
    for error in errors:
        print('[board-profile] ' + error)
    return 1 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
