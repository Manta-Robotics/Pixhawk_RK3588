#!/usr/bin/env python3

import json
import os
import select
import subprocess
import threading
import time
from email.utils import formatdate
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from socketserver import ThreadingMixIn
from urllib.parse import urlsplit

PROJECT_DIR = Path(__file__).resolve().parents[1]
CONFIG = json.loads((PROJECT_DIR / 'config' / 'system.config.json').read_text(encoding='utf-8'))
HOTSPOT = CONFIG.get('hotspot', {})
CAMERA = CONFIG.get('camera', {})
SNAPSHOT_PORT = int(HOTSPOT.get('camera_port', 8090))
SNAPSHOT_HOST = str(HOTSPOT.get('camera_host', CAMERA.get('proxy_host', '127.0.0.1'))).strip() or '127.0.0.1'
FRAME_WIDTH = int(CAMERA.get('width', 1920))
FRAME_HEIGHT = int(CAMERA.get('height', 1080))
FRAME_FPS = int(CAMERA.get('fps', 15))
CAMERA_INPUT_FORMAT = str(CAMERA.get('input_format', 'auto')).strip().lower()
FRAME_CACHE_SECONDS = max(0.1, float(CAMERA.get('refresh_ms', 1500)) / 1000.0)
FRAME_STALL_SECONDS = max(1.0, float(CAMERA.get('stall_reconnect_seconds', 3.0)))
SENSOR_CONTROLS = CAMERA.get('sensor_controls', {}) if isinstance(CAMERA.get('sensor_controls', {}), dict) else {}
_REAPPLY_RAW = CAMERA.get('sensor_control_reapply_seconds', [1.5, 4.0])
SENSOR_REAPPLY = [float(x) for x in (_REAPPLY_RAW if isinstance(_REAPPLY_RAW, list) else [])]
SENSOR_ENFORCE_SECONDS = max(0.2, float(CAMERA.get('sensor_control_enforce_seconds', 1.0)))
SNAPSHOT_FILTER = str(CAMERA.get('snapshot_filter', '')).strip()
FORCED_DEVICE = str(CAMERA.get('device', 'auto'))
FRAME_LOCK = threading.Lock()
FRAME_CACHE = {'timestamp': 0.0, 'bytes': b'', 'device': '', 'name': '', 'error': ''}
FRAME_READY = threading.Event()
CONTROL_DEVICE_MAP = {}


def enumerate_video_devices():
    devices = []
    video_dir = Path('/sys/class/video4linux')
    if not video_dir.exists():
        return devices

    for entry in sorted(video_dir.iterdir()):
        if not entry.name.startswith('video'):
            continue
        name_file = entry / 'name'
        name = name_file.read_text(encoding='utf-8').strip() if name_file.exists() else ''
        devices.append({'device': f'/dev/{entry.name}', 'name': name})
    return devices


def select_camera_device():
    if FORCED_DEVICE and FORCED_DEVICE.lower() != 'auto':
        return {'device': FORCED_DEVICE, 'name': 'forced'}

    candidates = []
    for entry in enumerate_video_devices():
        name = entry['name'].lower()
        if 'hdmirx' in name or 'hdmi' in name:
            continue

        score = 10
        if 'rkisp_mainpath' in name:
            score = 100
        elif 'uvc' in name or 'usb' in name:
            score = 80
        elif 'camera' in name or 'rkisp' in name:
            score = 60

        candidates.append((score, entry))

    if not candidates:
        return None

    candidates.sort(key=lambda item: item[0], reverse=True)
    return candidates[0][1]


def find_control_device(control_names):
    """Return mapping {control_name: subdev_path}. Each control may live on a different subdev."""
    requested = [name for name in control_names if name]
    if not requested:
        return {}

    missing = [name for name in requested if name not in CONTROL_DEVICE_MAP]
    if missing:
        for device in sorted(Path('/dev').glob('v4l-subdev*')):
            result = subprocess.run(
                ['v4l2-ctl', '-d', str(device), '--list-ctrls'],
                capture_output=True,
                text=True,
                check=False,
            )
            if result.returncode != 0:
                continue
            output = result.stdout
            for name in list(missing):
                if name not in CONTROL_DEVICE_MAP and name in output:
                    CONTROL_DEVICE_MAP[name] = str(device)
                    missing.remove(name)
            if not missing:
                break
    return {name: CONTROL_DEVICE_MAP[name] for name in requested if name in CONTROL_DEVICE_MAP}


def normalize_control_value(value):
    text = str(value).strip()
    try:
        return int(text)
    except ValueError:
        try:
            return float(text)
        except ValueError:
            return text


def read_sensor_controls(control_device, names):
    if not control_device or not names:
        return None

    result = subprocess.run(
        ['v4l2-ctl', '-d', control_device, '--get-ctrl', ','.join(names)],
        capture_output=True,
        text=True,
        check=False
    )
    if result.returncode != 0:
        return {}

    current = {}
    for line in result.stdout.splitlines():
        if ':' not in line:
            continue

        name, value = line.split(':', 1)
        name = name.strip()
        if name in SENSOR_CONTROLS:
            current[name] = normalize_control_value(value)

    return current


def apply_sensor_controls(force=False):
    if not SENSOR_CONTROLS:
        return False

    device_map = find_control_device(SENSOR_CONTROLS.keys())
    if not device_map:
        return False

    # Group desired controls by the subdev that exposes them.
    grouped = {}
    for name, value in SENSOR_CONTROLS.items():
        if value is None or name not in device_map:
            continue
        device = device_map[name]
        grouped.setdefault(device, {})[name] = normalize_control_value(value)

    if not grouped:
        return False

    changed = False
    for device, desired in grouped.items():
        if not force:
            current = read_sensor_controls(device, list(desired.keys()))
            if current and all(current.get(n) == desired.get(n) for n in desired):
                continue
        assignments = [f'{n}={v}' for n, v in desired.items()]
        subprocess.run(
            ['v4l2-ctl', '-d', device, '--set-ctrl', ','.join(assignments)],
            capture_output=True,
            text=True,
            check=False,
        )
        changed = True
    return changed


def capture_snapshot():
    selected = select_camera_device()
    if not selected:
        raise RuntimeError('No active camera device detected. Enable the configured camera overlay and reboot first.')

    apply_sensor_controls(force=True)

    command = [
        'ffmpeg',
        '-hide_banner',
        '-loglevel', 'error',
        '-fflags', '+genpts',
        '-f', 'video4linux2',
        '-framerate', str(FRAME_FPS),
        '-video_size', f'{FRAME_WIDTH}x{FRAME_HEIGHT}',
    ]
    if CAMERA_INPUT_FORMAT and CAMERA_INPUT_FORMAT != 'auto':
        command.extend(['-input_format', CAMERA_INPUT_FORMAT])
    command.extend(['-i', selected['device']])

    if SNAPSHOT_FILTER:
        command.extend(['-vf', SNAPSHOT_FILTER])

    command.extend([
        '-frames:v', '1',
        '-an',
        '-c:v', 'mjpeg',
        '-q:v', '5',
        '-f', 'image2pipe',
        'pipe:1'
    ])

    result = subprocess.run(command, capture_output=True, timeout=12, check=False)
    if result.returncode != 0 or not result.stdout:
        error_text = result.stderr.decode('utf-8', errors='ignore').strip() or 'ffmpeg failed to capture a frame.'
        raise RuntimeError(error_text)

    return selected, result.stdout


def build_capture_command(device):
    command = [
        'ffmpeg',
        '-hide_banner',
        '-loglevel', 'error',
        '-fflags', '+genpts+discardcorrupt',
        '-flags', 'low_delay',
        '-thread_queue_size', '64',
        '-f', 'video4linux2',
        '-framerate', str(FRAME_FPS),
        '-video_size', f'{FRAME_WIDTH}x{FRAME_HEIGHT}',
    ]
    if CAMERA_INPUT_FORMAT and CAMERA_INPUT_FORMAT != 'auto':
        command.extend(['-input_format', CAMERA_INPUT_FORMAT])
    command.extend(['-i', device])

    if SNAPSHOT_FILTER:
        command.extend(['-vf', SNAPSHOT_FILTER])

    command.extend([
        '-r', str(FRAME_FPS),
        '-an',
        '-c:v', 'mjpeg',
        '-q:v', '5',
        '-f', 'image2pipe',
        'pipe:1'
    ])
    return command


def capture_loop():
    while True:
        process = None
        try:
            selected = select_camera_device()
            if not selected:
                raise RuntimeError('No active camera device detected. Enable the configured camera overlay and reboot first.')

            apply_sensor_controls(force=True)

            process = subprocess.Popen(
                build_capture_command(selected['device']),
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0
            )
            if process.stdout:
                os.set_blocking(process.stdout.fileno(), False)

            for delay in SENSOR_REAPPLY:
                threading.Timer(delay, lambda: apply_sensor_controls(force=True)).start()

            frame_buffer = bytearray()
            last_frame_at = time.time()
            next_enforce_at = time.time() + SENSOR_ENFORCE_SECONDS
            while True:
                if process.poll() is not None:
                    break

                if not process.stdout:
                    break

                now = time.time()
                if now >= next_enforce_at:
                    apply_sensor_controls(force=False)
                    next_enforce_at = now + SENSOR_ENFORCE_SECONDS

                ready, _, _ = select.select([process.stdout.fileno()], [], [], 0.6)
                if not ready:
                    if (time.time() - last_frame_at) > FRAME_STALL_SECONDS:
                        raise RuntimeError('camera stream stalled; reconnecting ffmpeg')
                    continue

                chunk = os.read(process.stdout.fileno(), 32768)
                if not chunk:
                    break

                frame_buffer.extend(chunk)

                while True:
                    start = frame_buffer.find(b'\xff\xd8')
                    if start == -1:
                        if len(frame_buffer) > 1048576:
                            frame_buffer.clear()
                        break

                    end = frame_buffer.find(b'\xff\xd9', start + 2)
                    if end == -1:
                        if start > 0:
                            del frame_buffer[:start]
                        break

                    payload = bytes(frame_buffer[start:end + 2])
                    del frame_buffer[:end + 2]

                    if len(payload) < 2048:
                        continue

                    with FRAME_LOCK:
                        FRAME_CACHE.update({
                            'timestamp': time.time(),
                            'bytes': payload,
                            'device': selected['device'],
                            'name': selected['name'],
                            'error': ''
                        })
                    last_frame_at = time.time()
                    FRAME_READY.set()

            raise RuntimeError('ffmpeg capture process exited unexpectedly.')
        except Exception as error:
            with FRAME_LOCK:
                FRAME_CACHE['error'] = str(error)
            FRAME_READY.clear()
        finally:
            if process and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    process.kill()

        time.sleep(0.5)


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class SnapshotHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        print('[camera] ' + fmt % args)

    def _send_common_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

    def _send_json(self, payload, status=200):
        encoded = json.dumps(payload).encode('utf-8')
        self.send_response(status)
        self._send_common_headers()
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def do_OPTIONS(self):
        self.send_response(204)
        self._send_common_headers()
        self.end_headers()

    def do_GET(self):
        path = urlsplit(self.path).path

        if path == '/healthz':
            with FRAME_LOCK:
                frame_age = time.time() - FRAME_CACHE['timestamp'] if FRAME_CACHE['timestamp'] else None
                ok = bool(
                    FRAME_CACHE['bytes']
                    and not FRAME_CACHE['error']
                    and frame_age is not None
                    and frame_age <= max(FRAME_STALL_SECONDS * 2.0, FRAME_CACHE_SECONDS + 2.0)
                )
                payload = {
                    'ok': ok,
                    'cachedDevice': FRAME_CACHE['device'],
                    'cachedName': FRAME_CACHE['name'],
                    'lastError': FRAME_CACHE['error'],
                    'lastFrameAgeSeconds': frame_age,
                    'videoDevices': enumerate_video_devices()
                }
            self._send_json(payload)
            return

        if path in ('/stream', '/stream.mjpg'):
            if not FRAME_READY.wait(timeout=3):
                with FRAME_LOCK:
                    error_message = FRAME_CACHE['error'] or 'Camera frame is not ready yet.'
                self._send_json({'ok': False, 'message': error_message, 'videoDevices': enumerate_video_devices()}, status=503)
                return

            try:
                self.send_response(200)
                self._send_common_headers()
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=ffmpeg')
                self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
                self.send_header('Connection', 'close')
                self.end_headers()

                last_sent_at = 0.0
                while True:
                    with FRAME_LOCK:
                        frame_bytes = FRAME_CACHE['bytes']
                        frame_timestamp = FRAME_CACHE['timestamp']
                        frame_error = FRAME_CACHE['error']

                    if not frame_bytes:
                        if frame_error:
                            raise RuntimeError(frame_error)
                        time.sleep(0.05)
                        continue

                    if frame_timestamp <= last_sent_at:
                        time.sleep(0.02)
                        continue

                    header = (
                        b'--ffmpeg\r\n'
                        b'Content-Type: image/jpeg\r\n'
                        + f'Content-Length: {len(frame_bytes)}\r\n'.encode('ascii')
                        + f'Date: {formatdate(usegmt=True)}\r\n\r\n'.encode('ascii')
                    )
                    self.wfile.write(header)
                    self.wfile.write(frame_bytes)
                    self.wfile.write(b'\r\n')
                    self.wfile.flush()
                    last_sent_at = frame_timestamp
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            return

        if path not in ('/', '/snapshot.jpg', '/frame.jpg'):
            self.send_response(302)
            self.send_header('Location', '/snapshot.jpg')
            self.end_headers()
            return

        try:
            with FRAME_LOCK:
                if FRAME_CACHE['bytes'] and time.time() - FRAME_CACHE['timestamp'] < FRAME_CACHE_SECONDS:
                    payload = FRAME_CACHE['bytes']
                else:
                    selected, payload = capture_snapshot()
                    FRAME_CACHE.update({
                        'timestamp': time.time(),
                        'bytes': payload,
                        'device': selected['device'],
                        'name': selected['name'],
                        'error': ''
                    })

            self.send_response(200)
            self._send_common_headers()
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
            self.end_headers()
            self.wfile.write(payload)
        except Exception as error:
            with FRAME_LOCK:
                FRAME_CACHE['error'] = str(error)
            self._send_json({'ok': False, 'message': str(error), 'videoDevices': enumerate_video_devices()}, status=503)


if __name__ == '__main__':
    capture_thread = threading.Thread(target=capture_loop, name='camera-capture', daemon=True)
    capture_thread.start()
    server = ThreadingHTTPServer((SNAPSHOT_HOST, SNAPSHOT_PORT), SnapshotHandler)
    print(f'[camera] Listening on http://{SNAPSHOT_HOST}:{SNAPSHOT_PORT}/snapshot.jpg and /stream.mjpg')
    server.serve_forever()
