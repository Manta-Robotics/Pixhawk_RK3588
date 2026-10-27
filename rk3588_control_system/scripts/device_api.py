#!/usr/bin/env python3
"""Private, read-only paired API sessions for local maintenance tools."""
import argparse
import json
import os
import re
import shlex
from http.cookiejar import CookieJar
from pathlib import Path
from urllib.parse import urljoin, urlsplit
from urllib.request import HTTPCookieProcessor, Request, build_opener


def control_pin(pin_file=None):
    if pin_file:
        pin = Path(pin_file).read_text(encoding='utf-8').strip()
    else:
        pin = os.environ.get('MANTA_CONTROL_PIN')
        if pin is None:
            environment = Path('/etc/manta/manta.env')
            if environment.is_file():
                for line in environment.read_text(encoding='utf-8').splitlines():
                    if line.startswith('MANTA_CONTROL_PIN='):
                        parts = shlex.split(line.split('=', 1)[1], comments=True)
                        pin = parts[0] if len(parts) == 1 else ''
                        break
            if pin is None:
                private_file = Path(__file__).resolve().parents[1] / 'data/control-pin'
                if private_file.is_file():
                    pin = private_file.read_text(encoding='utf-8').strip()
    if not re.fullmatch(r'[0-9]{6}', pin or ''):
        raise ValueError('Set MANTA_CONTROL_PIN or provide --pin-file with the device control PIN')
    return pin


class DeviceApiSession:
    """Creates its own session, never claims control, and logs out on exit."""
    def __init__(self, url, pin_file=None):
        parsed = urlsplit(url)
        if parsed.scheme not in ('http', 'https') or not parsed.netloc or parsed.username:
            raise ValueError('A device HTTP(S) URL is required')
        self.origin = f'{parsed.scheme}://{parsed.netloc}'
        self.pin_file = pin_file
        self.cookies = CookieJar()
        self.opener = build_opener(HTTPCookieProcessor(self.cookies))

    def _post(self, route, body):
        request = Request(self.origin + route, data=json.dumps(body).encode(),
                          headers={'Content-Type': 'application/json'}, method='POST')
        with self.opener.open(request, timeout=4) as response:
            return json.load(response)

    def __enter__(self):
        self._post('/api/auth/pair', {'pin': control_pin(self.pin_file)})
        return self

    def open(self, url, timeout=2):
        if urlsplit(url)._replace(path='', query='', fragment='').geturl().rstrip('/') != self.origin:
            raise ValueError('Read-only device session cannot access another origin')
        return self.opener.open(url, timeout=timeout)

    def __exit__(self, *_args):
        try:
            self._post('/api/auth/logout', {})
        except OSError:
            pass
        finally:
            self.cookies.clear()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:3000/api/status')
    parser.add_argument('--pin-file')
    args = parser.parse_args()
    try:
        with DeviceApiSession(args.url, args.pin_file) as session:
            with session.open(args.url) as response:
                print(json.dumps(json.load(response)))
    except (OSError, ValueError) as error:
        parser.exit(2, f'Device read failed: {error}\n')


if __name__ == '__main__':
    main()
