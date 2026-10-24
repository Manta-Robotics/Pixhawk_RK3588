#!/usr/bin/env python3
"""Real C++ daemon + Node HTTP + PTY autopilot, without physical hardware."""
import concurrent.futures
import json
import os
from pathlib import Path
import select
import socket
import subprocess
import sys
import tempfile
import threading
import time
import tty
import urllib.error
import urllib.request
from pymavlink.dialects.v20 import ardupilotmega as mav

ROOT = Path(__file__).resolve().parents[1]


def free_port(kind=socket.SOCK_DGRAM):
    with socket.socket(socket.AF_INET, kind) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


class Autopilot:
    def __init__(self):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        self.mode, self.armed, self.radio, self.accept_mode = 0, False, True, True
        self.distance, self.angle, self.running = 4.0, 0.0, True
        self.frames, self.error = [], None
        self.sender = mav.MAVLink(None, srcSystem=1, srcComponent=1)
        self.parser = mav.MAVLink(None)
        self.parameters = dict(PILOT_STEER_TYPE=0, SPEED_MAX=2, CRUISE_SPEED=.8,
                               CRUISE_THROTTLE=50, ACRO_TURN_RATE=90, RCMAP_ROLL=1, RCMAP_THROTTLE=3)
        for channel in (1, 3):
            self.parameters.update({f'RC{channel}_{key}': value for key, value in
                                    dict(MIN=1000, MAX=2000, TRIM=1500, DZ=30, REVERSED=0).items()})
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, message):
        os.write(self.master, message.pack(self.sender))
        self.sender.seq = (self.sender.seq+1) % 256

    def run(self):
        next_sensor = 0
        try:
            while self.running:
                now = time.monotonic()
                if now >= next_sensor:
                    next_sensor = now+.05
                    stamp = int(now*1000) % 2**32
                    self.send(mav.MAVLink_heartbeat_message(mav.MAV_TYPE_GROUND_ROVER, mav.MAV_AUTOPILOT_ARDUPILOTMEGA, 129 if self.armed else 1, self.mode, 4, 3))
                    self.send(mav.MAVLink_attitude_message(stamp, 0, 0, 0, 0, 0, 0))
                    self.send(mav.MAVLink_global_position_int_message(stamp, 300000000, 1200000000, 0, 0, 0, 0, 0, 0))
                    self.send(mav.MAVLink_gps_raw_int_message(stamp*1000, 3, 300000000, 1200000000, 0, 100, 100, 0, 0, 12))
                    self.send(mav.MAVLink_ekf_status_report_message(18, .01, .01, .01, .01, .01))
                    self.send(mav.MAVLink_servo_output_raw_message(stamp, 0, 1500, 1500, 1500, 1500, 0, 0, 0, 0))
                    if self.radio:
                        for name, value in [('UWB1_DIST', self.distance), ('UWB1_AZ', self.angle), ('UWB1_EL', 0)]:
                            self.send(mav.MAVLink_named_value_float_message(stamp, name.encode(), value))
                if not select.select([self.master], [], [], .005)[0]:
                    continue
                for byte in os.read(self.master, 65536):
                    message = self.parser.parse_char(bytes([byte]))
                    if not message:
                        continue
                    self.frames.append((time.monotonic(), message))
                    if message.get_type() == 'PARAM_REQUEST_READ':
                        key = message.param_id
                        if key in self.parameters:
                            self.send(mav.MAVLink_param_value_message(key.encode(), self.parameters[key], mav.MAV_PARAM_TYPE_REAL32, len(self.parameters), 0))
                    elif message.get_type() == 'COMMAND_LONG':
                        if message.command == 176 and self.accept_mode:
                            self.mode = int(message.param2)
                        if message.command == 400:
                            assert message.param2 == 0, 'forced arming must never be used'
                            self.armed = bool(message.param1)
                        self.send(mav.MAVLink_command_ack_message(message.command, 0))
        except Exception as error:
            self.error = error

    def close(self):
        self.running = False
        self.thread.join(timeout=2)
        os.close(self.master)
        os.close(self.slave)


def main():
    binary = str(Path(sys.argv[1]).resolve())
    node = str(Path(sys.argv[2]).resolve()) if len(sys.argv) > 2 else 'node'
    fcu = Autopilot()
    native = web = None
    with tempfile.TemporaryDirectory(prefix='manta-native-stack-') as directory:
        config = json.loads((ROOT/'config/system.config.json').read_text())
        config.update(serial_port=os.ttyname(fcu.slave), web_host='127.0.0.1', web_port=free_port(socket.SOCK_STREAM),
                      bridge_command_port=free_port(), bridge_telemetry_port=free_port(), logs_dir=f'{directory}/logs')
        config['recordings']['dir'] = f'{directory}/recordings'
        config['camera']['enabled'] = False
        config['gimbal']['enabled'] = False
        config['gimbal']['auto_connect'] = False
        config['gimbal']['face']['keep_warm'] = False
        config['gimbal']['beacon'].update(yaw_source_sign=1, yaw_source_scale=1, yaw_offset_deg=0)
        config['uwb_follow']['relative_nmpc']['enabled'] = False
        config['uwb_follow']['gps_speed_closed_loop']['enabled'] = False
        path = Path(directory)/'config.json'
        path.write_text(json.dumps(config))
        log = open(Path(directory)/'stack.log', 'w+')
        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client.settimeout(2)

        def rpc(command, params=None):
            request_id = str(time.monotonic_ns())
            client.sendto(json.dumps(dict(command=command, params=params or {}, requestId=request_id)).encode(), ('127.0.0.1', config['bridge_command_port']))
            while True:
                payload = json.loads(client.recv(65536))['payload']
                if payload['requestId'] == request_id:
                    return payload['result']

        def wait(predicate, seconds=5):
            end = time.monotonic()+seconds
            while time.monotonic() < end:
                assert fcu.error is None, fcu.error
                assert native.poll() is None, 'native process exited'
                if predicate():
                    return
                time.sleep(.05)
            raise AssertionError('condition timed out; state='+json.dumps(rpc('CONTROL_STATUS'))+' telemetry='+json.dumps(http('/api/status')[1]['data']['telemetry']))

        def http(route, body=None):
            request = urllib.request.Request(f'http://127.0.0.1:{config["web_port"]}{route}',
                                             data=None if body is None else json.dumps(body).encode(),
                                             headers={'Content-Type': 'application/json'})
            try:
                with urllib.request.urlopen(request, timeout=22) as response:
                    return response.status, json.load(response)
            except urllib.error.HTTPError as error:
                return error.code, json.load(error)

        def start_web():
            process = subprocess.Popen([node, 'backend/server.js'], cwd=ROOT,
                                       env={**os.environ, 'MANTA_CONFIG_FILE': str(path)}, stdout=log, stderr=log)
            end = time.monotonic()+8
            while time.monotonic() < end:
                assert process.poll() is None, 'Web process exited'
                try:
                    if http('/health')[0] == 200:
                        return process
                except OSError:
                    pass
                time.sleep(.05)
            raise AssertionError('Web startup timed out')

        try:
            native = subprocess.Popen([binary, '--config', str(path)], cwd=ROOT, stdout=log, stderr=log)
            time.sleep(.2)
            wait(lambda: rpc('CONTROL_STATUS')['state'].get('reason') == 'method_selected')
            web = start_web()
            status, result = http('/api/uwb-follow/start', {})
            assert status == 200 and result['state']['active'], result
            mark = time.monotonic()
            wait(lambda: any(at > mark and m.get_type() == 'MANUAL_CONTROL' and m.z > 0 for at, m in fcu.frames))
            # A graceful Web shutdown must not send a vehicle stop or disarm.
            web.terminate(); web.wait(timeout=5); web = None
            mark = time.monotonic(); time.sleep(1)
            assert rpc('CONTROL_STATUS')['state']['active'] and fcu.armed
            assert any(at > mark+.5 and m.get_type() == 'MANUAL_CONTROL' and m.z > 0 for at, m in fcu.frames)
            web = start_web()
            wait(lambda: http('/api/uwb-follow/state')[1]['state']['active'])
            status, result = http('/api/control/rover', {'throttle': 30, 'steering': 0})
            assert status == 200, result
            assert not rpc('CONTROL_STATUS')['state']['active']
            time.sleep(.7)
            assert rpc('CONTROL_STATUS')['roverControl']['throttle'] == 0
            # Radio loss preserves the bounded wait, disarms, then requires a
            # multi-sample reacquisition before automatic mode/arm startup.
            assert http('/api/uwb-follow/start', {})[0] == 200
            fcu.radio = False
            wait(lambda: rpc('CONTROL_STATUS')['state'].get('status') == 'waiting_target', 3)
            wait(lambda: rpc('CONTROL_STATUS')['state'].get('autoResumePending') is True, 7)
            wait(lambda: not fcu.armed)
            fcu.radio = True
            wait(lambda: rpc('CONTROL_STATUS')['state']['active'], 6)
            assert fcu.armed
            assert http('/api/uwb-follow/stop', {})[0] == 200
            wait(lambda: not fcu.armed)
            time.sleep(.7)
            assert not rpc('CONTROL_STATUS')['state']['active']
            # Stop cancels an in-flight start; a late mode acknowledgement
            # must not complete that cancelled transaction or arm the FCU.
            fcu.mode = 1; fcu.accept_mode = False; time.sleep(.15)
            with concurrent.futures.ThreadPoolExecutor() as pool:
                starting = pool.submit(http, '/api/uwb-follow/start', {})
                wait(lambda: rpc('CONTROL_STATUS')['busy'])
                assert http('/api/uwb-follow/stop', {})[0] == 200
                status, response = starting.result(timeout=5)
                assert status == 409, response
            fcu.mode = 0; fcu.accept_mode = True; time.sleep(.5)
            assert not fcu.armed and not rpc('CONTROL_STATUS')['state']['active']
            assert http('/api/uwb-follow/method', {'method': 'neural'})[0] == 200
            assert http('/api/uwb-follow/start', {})[0] == 200
            assert rpc('CONTROL_STATUS')['state']['trackingMethod'] == 'neural'
            assert http('/api/uwb-follow/stop', {})[0] == 200
            print('Native stack: HTTP start/stop, Web restart independence, manual override/lease, UWB loss/reacquisition, cancelled startup and neural method passed')
        except Exception:
            log.flush(); log.seek(0); print(log.read()[-12000:], file=sys.stderr)
            raise
        finally:
            for process in (web, native):
                if process and process.poll() is None:
                    process.terminate()
                    try: process.wait(timeout=5)
                    except subprocess.TimeoutExpired: process.kill(); process.wait()
            client.close(); log.close(); fcu.close()


if __name__ == '__main__':
    main()
