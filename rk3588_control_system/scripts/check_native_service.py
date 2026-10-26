#!/usr/bin/env python3
"""Linux PTY/UDP integration test. No physical device or actuator is opened."""
import json
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import sys
import tempfile
import time
import tty

from pymavlink.dialects.v20 import ardupilotmega as mav


def main():
    binary = str(Path(sys.argv[1]).resolve())
    master, slave = os.openpty()
    tty.setraw(slave)
    command_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    command_client.bind(('127.0.0.1', 0))
    command_client.settimeout(1)
    telemetry = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    telemetry.bind(('127.0.0.1', 0))
    telemetry.setblocking(False)
    reserve = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    reserve.bind(('127.0.0.1', 0))
    command_port = reserve.getsockname()[1]
    reserve.close()
    mp_reserve = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    mp_reserve.bind(('127.0.0.1', 0))
    mp_port = mp_reserve.getsockname()[1]
    mp_reserve.close()
    sender, parser = mav.MAVLink(None, srcSystem=1, srcComponent=1), mav.MAVLink(None)
    config = dict(serial_port=os.ttyname(slave), baud_rate=115200, bridge_host='127.0.0.1',
                  bridge_command_port=command_port, bridge_telemetry_port=telemetry.getsockname()[1],
                  telemetry_rate=20, heartbeat_interval=.2, rover_command_timeout_ms=300,
                  mission_planner_tcp=dict(enabled=True, host='127.0.0.1', port=mp_port, allowed_clients=['127.0.0.1']))
    process = None
    frames, packets = [], []
    heartbeat_at = 0
    boot_at = time.monotonic() - 1
    def send(message):
        os.write(master, message.pack(sender)); sender.seq = (sender.seq+1) % 256

    def pump(duration, heartbeat=True):
        nonlocal heartbeat_at
        until = time.monotonic()+duration
        while time.monotonic() < until:
            if heartbeat and time.monotonic()-heartbeat_at >= .1:
                send(mav.MAVLink_heartbeat_message(mav.MAV_TYPE_GROUND_ROVER, mav.MAV_AUTOPILOT_ARDUPILOTMEGA, 129, 0, 4, 3))
                heartbeat_at = time.monotonic()
            ready, _, _ = select.select([master, telemetry], [], [], .01)
            if master in ready:
                data = os.read(master, 65536)
                for byte in data:
                    msg = parser.parse_char(bytes([byte]))
                    if msg:
                        frames.append(msg)
                        if msg.get_type() == 'COMMAND_LONG' and msg.command == 511:
                            send(mav.MAVLink_command_ack_message(511, 0))
                        if msg.get_type() == 'TIMESYNC' and msg.tc1 == 0:
                            send(mav.MAVLink_timesync_message(int((time.monotonic()-boot_at)*1e9), msg.ts1))
            if telemetry in ready:
                packets.append(json.loads(telemetry.recv(65536)))

    def command(name, params=None):
        request = dict(command=name, params=params or {}, requestId=f'{len(frames)}-{time.monotonic()}')
        command_client.sendto(json.dumps(request).encode(), ('127.0.0.1', command_port))
        response = json.loads(command_client.recv(65536))
        assert response['payload']['requestId'] == request['requestId']
        return response['payload']['result']

    try:
        with tempfile.TemporaryDirectory(prefix='manta-native-') as directory:
            path = Path(directory)/'config.json'; path.write_text(json.dumps(config))
            check = subprocess.run([binary, '--config', str(path), '--check-config'], capture_output=True, text=True, timeout=5)
            assert check.returncode == 0, check.stderr
            for invalid in [dict(bridge_host='0.0.0.0'), dict(rover_command_timeout_ms=0),
                            dict(uwb_follow={'prediction': {'braking_deceleration_mps2': 0}}),
                            dict(uwb_follow={'execution_mode': 'acro', 'max_speed_mps': 3})]:
                path.write_text(json.dumps({**config, **invalid}))
                rejected = subprocess.run([binary, '--config', str(path), '--check-config'], capture_output=True, text=True, timeout=5)
                assert rejected.returncode != 0, invalid
            path.write_text(json.dumps(config))
            log = open(Path(directory)/'service.log', 'w+')
            # Pre-open stale bytes must not survive the service's input flush.
            send(mav.MAVLink_heartbeat_message(10, 3, 129, 0, 4, 3))
            for name, value in [('DIST', .2), ('EL', 0), ('AZ', 30)]:
                send(mav.MAVLink_named_value_float_message(10000, f'UWB1_{name}'.encode(), value))
            process = subprocess.Popen([binary, '--config', str(path)], stdout=log, stderr=log)
            pump(.7)
            assert process.poll() is None
            assert any(p['type'] == 'connection' and p['payload']['connected'] for p in packets), packets[-2:]
            assert any(m.get_type() == 'HEARTBEAT' for m in frames)
            assert any(m.get_type() == 'COMMAND_LONG' and m.command == 511 for m in frames)
            assert any(m.get_type() == 'TIMESYNC' and m.tc1 == 0 for m in frames)
            assert not any(p['payload'].get('clearanceStop') or p['payload'].get('fresh')
                           for p in packets if p['type'] == 'uwb')
            with socket.create_connection(('127.0.0.1', mp_port), timeout=1) as planner:
                pump(.2)
                received = mav.MAVLink(None).parse_buffer(planner.recv(65536))
                assert any(m.get_type() == 'HEARTBEAT' for m in received)
                gcs = mav.MAVLink(None, srcSystem=255, srcComponent=190)
                frame = mav.MAVLink_command_long_message(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0).pack(gcs)
                planner.sendall(frame);pump(.1)
                assert any(m.get_type() == 'COMMAND_LONG' and m.command == 512 for m in frames)
            assert command('ROVER_DRIVE', dict(throttle=35, steering=5))['ok']
            pump(.1)
            assert any(m.get_type() == 'MANUAL_CONTROL' and m.z == 350 for m in frames)
            mark = len(frames)
            # No command publisher remains active; service alone owns expiry.
            pump(.4)
            assert any(m.get_type() == 'MANUAL_CONTROL' and m.y == m.z == 0 for m in frames[mark:])
            assert not command('ROVER_DRIVE', dict(throttle=None, steering=10))['ok']
            assert not command('MOTOR_CONTROL', dict(channel=1, pwm=1900))['ok']
            assert command('IMU_CALIBRATION_START', {'type': 'ACCEL'})['ok']
            pump(.1)
            send(mav.MAVLink_command_long_message(255, 0, 42429, 0, 2, 0, 0, 0, 0, 0, 0));pump(.1)
            assert any(p['type'] == 'telemetry' and p['payload']['imuCalibration']['step'] == 'LEFT' for p in packets)
            # A second instance must fail before it can write the same serial.
            duplicate = subprocess.run([binary, '--config', str(path)], capture_output=True, text=True, timeout=5)
            assert duplicate.returncode != 0 and 'command port' in duplicate.stderr
            assert command('ROVER_DRIVE', dict(throttle=40, steering=0))['ok'];pump(.05)
            mark = len(frames);process.send_signal(signal.SIGTERM);pump(.15, heartbeat=False)
            assert process.wait(timeout=3) == 0
            assert any(m.get_type() == 'MANUAL_CONTROL' and m.y == m.z == 0 for m in frames[mark:])
            log.close()
    finally:
        if process and process.poll() is None:
            process.terminate()
            try: process.wait(timeout=3)
            except subprocess.TimeoutExpired: process.kill(); process.wait()
        os.close(master);os.close(slave);command_client.close();telemetry.close()
    print('Native service: PTY MAVLink, Mission Planner TCP, UDP RPC, independent lease, calibration, duplicate ownership and shutdown passed')


if __name__ == '__main__':
    main()
