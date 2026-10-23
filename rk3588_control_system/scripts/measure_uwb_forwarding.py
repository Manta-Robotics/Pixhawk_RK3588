#!/usr/bin/env python3
"""Read-only UWB rate measurement through the running bridge's HTTP cache.

Does not open a serial port, send commands, or change telemetry requests.
Polls at up to 100 Hz; received/accepted rates are observed lower bounds if
HTTP polling cannot keep up. FC counters remain independent of poll frequency.
"""
import argparse
import json
import time
from urllib.request import urlopen
from urllib.parse import urljoin


class CounterWindow:
    def __init__(self):
        self.first = self.last = None
        self.resets = 0

    def update(self, value, at):
        if value is None:
            return
        if self.last is not None and value < self.last[0]:
            self.first = self.last = None
            self.resets += 1
        if self.last is None or value != self.last[0]:
            self.last = (value, at)
            if self.first is None:
                self.first = self.last

    def hz(self):
        if self.first is None or self.last[1] <= self.first[1]:
            return None
        return (self.last[0] - self.first[0]) / (self.last[1] - self.first[1])


def fetch(url):
    with urlopen(url, timeout=2) as response:
        payload = json.load(response)
    return payload.get('telemetry', payload)


def connected(url):
    # /api/telemetry contains only telemetry. Connection state lives in the
    # status envelope, not in telemetry.connected.
    with urlopen(urljoin(url, 'status'), timeout=2) as response:
        payload = json.load(response)
    return payload.get('data', {}).get('pixhawkStatus') == 'connected'


def measure(url, duration, interval):
    telemetry = fetch(url)
    if not connected(url):
        raise RuntimeError('Flight controller is disconnected; no live measurement was taken.')
    rows = {}
    for sensor in telemetry.get('uwb', {}).get('sensors', []):
        rows[sensor['receiverId']] = {
            'good': CounterWindow(), 'sent': CounterWindow(),
            'last_received': sensor.get('receivedMeasurementAt'),
            'last_accepted': sensor.get('measurementUpdatedAt'),
            'received': 0, 'accepted': 0,
        }
    started = time.monotonic()
    requests = 0
    disconnected = False
    next_connection_check = started + 1
    while time.monotonic() - started < duration:
        telemetry = fetch(url)
        at = time.monotonic()
        requests += 1
        if at >= next_connection_check:
            if not connected(url):
                disconnected = True
                break
            next_connection_check = time.monotonic() + 1
        for sensor in telemetry.get('uwb', {}).get('sensors', []):
            row = rows.get(sensor['receiverId'])
            if row is None:
                continue
            row['good'].update(sensor.get('goodCount'), at)
            row['sent'].update(sensor.get('sentCount'), at)
            for clock, count in (('receivedMeasurementAt', 'received'), ('measurementUpdatedAt', 'accepted')):
                value = sensor.get(clock)
                key = 'last_' + count
                if value is not None and value != row[key]:
                    row[count] += 1
                    row[key] = value
            row['maxAvailable'] = sensor.get('maxAvailable')
            row['checksumErrors'] = sensor.get('checksumErrors')
        time.sleep(max(0, interval - (time.monotonic() - at)))
    elapsed = time.monotonic() - started
    return {
        'seconds': elapsed, 'http_poll_hz': requests / elapsed,
        'disconnected_during_measurement': disconnected,
        'note': 'Simulation is not used. FC sentCount counts forwarding attempts, not delivery acknowledgements. Observed sample rates can undercount when polling is slow.',
        'receivers': [{
            'receiverId': receiver_id,
            'fc_valid_frame_hz': row['good'].hz(),
            'fc_forward_attempt_hz': row['sent'].hz(),
            'bridge_received_observed_hz': row['received'] / elapsed,
            'bridge_accepted_observed_hz': row['accepted'] / elapsed,
            'counter_resets': row['good'].resets + row['sent'].resets,
            'maxAvailable': row.get('maxAvailable'),
            'checksumErrors': row.get('checksumErrors'),
        } for receiver_id, row in rows.items()],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:3000/api/telemetry')
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--interval', type=float, default=0.01)
    args = parser.parse_args()
    if args.seconds < 5 or args.interval < 0.005:
        parser.error('--seconds must be >= 5 and --interval must be >= 0.005')
    try:
        result = measure(args.url, args.seconds, args.interval)
    except (OSError, ValueError, RuntimeError) as error:
        parser.exit(2, f'{error}\n')
    print(json.dumps(result, indent=2))
    return 2 if result['disconnected_during_measurement'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
