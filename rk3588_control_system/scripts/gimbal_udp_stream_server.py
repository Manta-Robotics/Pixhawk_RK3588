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
GIMBAL = CONFIG.get('gimbal', {}) if isinstance(CONFIG.get('gimbal', {}), dict) else {}
VIDEO = GIMBAL.get('video', {}) if isinstance(GIMBAL.get('video', {}), dict) else {}

PROXY_PORT = int(VIDEO.get('proxy_port', 8091))
PROXY_HOST = str(VIDEO.get('proxy_host', '127.0.0.1')).strip() or '127.0.0.1'
UDP_INPUT = str(VIDEO.get('udp_input', f"udp://0.0.0.0:{int(GIMBAL.get('udp_video_port', 9554))}")).strip()
RTSP_INPUT = str(VIDEO.get('rtsp_input', VIDEO.get('input_url', ''))).strip()
VIDEO_TRANSPORT = str(VIDEO.get('transport', '')).strip().lower()
ENV_INPUT = str(os.environ.get('GIMBAL_VIDEO_INPUT', '')).strip()
INPUT_URL = ENV_INPUT or (RTSP_INPUT if (RTSP_INPUT and (VIDEO_TRANSPORT == 'rtsp' or RTSP_INPUT.lower().startswith('rtsp://'))) else UDP_INPUT)
INPUT_IS_RTSP = INPUT_URL.lower().startswith('rtsp://')
FRAME_FPS = int(VIDEO.get('fps', 25))
FRAME_STALL_SECONDS = max(1.0, float(VIDEO.get('stall_reconnect_seconds', 2.0)))

FRAME_LOCK = threading.Lock()
FRAME_CACHE = {'timestamp': 0.0, 'bytes': b'', 'error': ''}
FRAME_READY = threading.Event()


def build_capture_command() -> list[str]:
    command = [
        'ffmpeg',
        '-hide_banner',
        '-loglevel', 'error',
        '-fflags', '+genpts+discardcorrupt',
        '-flags', 'low_delay',
        '-thread_queue_size', '512',
    ]
    if INPUT_IS_RTSP:
        command.extend(['-rtsp_transport', str(VIDEO.get('rtsp_transport', 'tcp'))])
    command.extend([
        '-i', UDP_INPUT,
        '-r', str(FRAME_FPS),
        '-an',
        '-c:v', 'mjpeg',
        '-q:v', '6',
        '-f', 'image2pipe',
        'pipe:1'
    ])
    command[command.index('-i') + 1] = INPUT_URL
    return command


def capture_loop() -> None:
    while True:
        process = None
        try:
            process = subprocess.Popen(
                build_capture_command(),
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0,
            )
            if process.stdout:
                os.set_blocking(process.stdout.fileno(), False)

            frame_buffer = bytearray()
            last_frame_at = time.time()

            while True:
                if process.poll() is not None:
                    break
                if not process.stdout:
                    break

                ready, _, _ = select.select([process.stdout.fileno()], [], [], 0.6)
                if not ready:
                    if time.time() - last_frame_at > FRAME_STALL_SECONDS:
                        raise RuntimeError(f'gimbal {"rtsp" if INPUT_IS_RTSP else "udp"} stream stalled; reconnecting')
                    continue

                chunk = os.read(process.stdout.fileno(), 65536)
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
                    if len(payload) < 1024:
                        continue

                    with FRAME_LOCK:
                        FRAME_CACHE.update({'timestamp': time.time(), 'bytes': payload, 'error': ''})
                    FRAME_READY.set()
                    last_frame_at = time.time()

            raise RuntimeError('ffmpeg gimbal process exited unexpectedly')
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

        time.sleep(0.4)


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class GimbalStreamHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        print('[gimbal-stream] ' + fmt % args)

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

        if path in ('/healthz', '/health'):
            with FRAME_LOCK:
                self._send_json({'ok': True, 'inputUrl': INPUT_URL, 'inputType': 'rtsp' if INPUT_IS_RTSP else 'udp', 'udpInput': UDP_INPUT, 'lastError': FRAME_CACHE['error'], 'ready': bool(FRAME_CACHE['bytes'])})
            return

        if path in ('/stream', '/stream.mjpg'):
            if not FRAME_READY.wait(timeout=4):
                with FRAME_LOCK:
                    err = FRAME_CACHE['error'] or 'Gimbal stream not ready yet.'
                self._send_json({'ok': False, 'message': err, 'udpInput': UDP_INPUT}, status=503)
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
                        frame = FRAME_CACHE['bytes']
                        ts = FRAME_CACHE['timestamp']
                        err = FRAME_CACHE['error']

                    if not frame:
                        if err:
                            raise RuntimeError(err)
                        time.sleep(0.05)
                        continue

                    if ts <= last_sent_at:
                        time.sleep(0.02)
                        continue

                    header = (
                        b'--ffmpeg\r\n'
                        b'Content-Type: image/jpeg\r\n'
                        + f'Content-Length: {len(frame)}\r\n'.encode('ascii')
                        + f'Date: {formatdate(usegmt=True)}\r\n\r\n'.encode('ascii')
                    )
                    self.wfile.write(header)
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
                    self.wfile.flush()
                    last_sent_at = ts
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            return

        if path in ('/', '/snapshot.jpg', '/frame.jpg'):
            with FRAME_LOCK:
                frame = FRAME_CACHE['bytes']
                err = FRAME_CACHE['error']
            if not frame:
                self._send_json({'ok': False, 'message': err or 'Gimbal stream not ready yet.', 'udpInput': UDP_INPUT}, status=503)
                return

            self.send_response(200)
            self._send_common_headers()
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(frame)))
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
            self.end_headers()
            self.wfile.write(frame)
            return

        self.send_response(404)
        self.end_headers()


if __name__ == '__main__':
    capture_thread = threading.Thread(target=capture_loop, name='gimbal-udp-capture', daemon=True)
    capture_thread.start()
    server = ThreadingHTTPServer((PROXY_HOST, PROXY_PORT), GimbalStreamHandler)
    print(f'[gimbal-stream] Input: {INPUT_URL}')
    print(f'[gimbal-stream] Listening on http://{PROXY_HOST}:{PROXY_PORT}/stream.mjpg')
    server.serve_forever()
