import importlib.util
import json
import os
from pathlib import Path
import tempfile
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


def module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'scripts' / f'{name}.py')
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


device_api = module('device_api')
device_env = module('generate_device_env')


class DeviceAccessTests(unittest.TestCase):
    def test_installer_generates_one_private_pin_and_preserves_it(self):
        with patch.dict(os.environ, {}, clear=True):
            first = device_env.generated_values({})
            self.assertRegex(first['MANTA_CONTROL_PIN'], r'^[0-9]{6}$')
            self.assertEqual(device_env.generated_values(first), first)
            with tempfile.TemporaryDirectory() as directory:
                filename = Path(directory) / 'manta.env'
                device_env.write_env(filename, first)
                self.assertEqual(device_env.parse_env(filename.read_text()), first)
                if os.name != 'nt':
                    self.assertEqual(filename.stat().st_mode & 0o777, 0o600)

    def test_pin_file_and_environment_validation(self):
        with patch.dict(os.environ, {'MANTA_CONTROL_PIN': '012345'}):
            self.assertEqual(device_api.control_pin(), '012345')
        with patch.dict(os.environ, {'MANTA_CONTROL_PIN': '123'}):
            with self.assertRaises(ValueError):
                device_api.control_pin()
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / 'pin'
            filename.write_text('123456\n')
            self.assertEqual(device_api.control_pin(filename), '123456')

    def test_readonly_tool_pairs_reads_with_cookie_and_logs_out_without_claiming(self):
        requests = []

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_args):
                pass

            def do_POST(self):
                requests.append((self.path, self.headers.get('Cookie')))
                body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                if self.path == '/api/auth/pair':
                    self.assert_pin = body['pin'] == '123456'
                    self.send_response(200 if self.assert_pin else 401)
                    self.send_header('Set-Cookie', 'manta_session=test-session; Path=/; HttpOnly; SameSite=Strict')
                else:
                    self.send_response(200 if self.path == '/api/auth/logout' else 403)
                self.end_headers()
                self.wfile.write(b'{"success":true}')

            def do_GET(self):
                requests.append((self.path, self.headers.get('Cookie')))
                self.send_response(200 if self.headers.get('Cookie') == 'manta_session=test-session' else 401)
                self.end_headers()
                self.wfile.write(b'{"data":{"isConnected":false}}')

        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            origin = f'http://127.0.0.1:{server.server_port}'
            with patch.dict(os.environ, {'MANTA_CONTROL_PIN': '123456'}):
                with device_api.DeviceApiSession(origin) as session:
                    with session.open(origin + '/api/status') as response:
                        self.assertFalse(json.load(response)['data']['isConnected'])
                    with self.assertRaises(ValueError):
                        session.open('http://example.invalid/api/status')
            self.assertEqual([route for route, _cookie in requests], ['/api/auth/pair', '/api/status', '/api/auth/logout'])
            self.assertTrue(all(cookie == 'manta_session=test-session' for _route, cookie in requests[1:]))
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == '__main__':
    unittest.main()
