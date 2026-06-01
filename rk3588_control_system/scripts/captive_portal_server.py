#!/usr/bin/env python3

import http.server
import json
from pathlib import Path
from socketserver import ThreadingMixIn
from urllib.parse import urlsplit

PROJECT_DIR = Path(__file__).resolve().parents[1]
CONFIG = json.loads((PROJECT_DIR / 'config' / 'system.config.json').read_text(encoding='utf-8'))
HOTSPOT = CONFIG.get('hotspot', {})
PORTAL_IP = str(HOTSPOT.get('portal_ip', '10.42.0.1'))
PORTAL_PORT = int(HOTSPOT.get('portal_port', 80))
DASHBOARD_PORT = int(CONFIG.get('web_port', 3000))
TARGET_URL = f'http://{PORTAL_IP}:{DASHBOARD_PORT}'
LISTEN_HOST = '0.0.0.0'

CAPTIVE_PROBE_PATHS = {
    '/generate_204',
    '/gen_204',
    '/hotspot-detect.html',
    '/connecttest.txt',
    '/ncsi.txt',
    '/library/test/success.html',
    '/success.txt',
    '/mobile/status.php',
    '/kindle-wifi/wifiredirect.html',
}

PORTAL_HTML = f'''<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Manta Control Portal</title>
  <meta http-equiv="refresh" content="2; url={TARGET_URL}">
  <style>
    body {{ font-family: Arial, sans-serif; background: #09131c; color: #eef6ff; margin: 0; min-height: 100vh; display: grid; place-items: center; }}
    .card {{ max-width: 420px; margin: 24px; padding: 24px; border-radius: 18px; background: rgba(17, 32, 46, 0.94); border: 1px solid rgba(124, 224, 255, 0.24); }}
    h1 {{ font-size: 24px; margin: 0 0 12px; }}
    p {{ line-height: 1.6; color: #bdd0df; }}
    a {{ display: inline-block; margin-top: 16px; padding: 12px 16px; border-radius: 999px; background: #1d6fa5; color: #fff; text-decoration: none; }}
  </style>
</head>
<body>
  <main class="card">
    <h1>Manta Control</h1>
    <p>Your phone is connected to the LubanCat hotspot. Tap the button below to open the control dashboard.</p>
    <a href="{TARGET_URL}">Open Dashboard</a>
  </main>
</body>
</html>
'''


class ThreadingHTTPServer(ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True


class PortalHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        print('[portal] ' + fmt % args)

    def _send_html(self, body, status=200):
        encoded = body.encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _redirect(self, location):
        self.send_response(302)
        self.send_header('Location', location)
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()

    def do_GET(self):
        path = urlsplit(self.path).path

        if path == '/healthz':
            payload = json.dumps({'ok': True, 'targetUrl': TARGET_URL}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return

        if path in CAPTIVE_PROBE_PATHS:
            self._redirect('/portal')
            return

        if path in ('/', '/index.html', '/portal'):
            self._send_html(PORTAL_HTML)
            return

        self._redirect(TARGET_URL)


if __name__ == '__main__':
    server = ThreadingHTTPServer((LISTEN_HOST, PORTAL_PORT), PortalHandler)
    print(f'[portal] Listening on http://{LISTEN_HOST}:{PORTAL_PORT} -> {TARGET_URL}')
    server.serve_forever()
