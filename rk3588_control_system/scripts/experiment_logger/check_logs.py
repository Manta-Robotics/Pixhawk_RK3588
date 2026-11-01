#!/usr/bin/env python3
"""Read-only health/rate summary for a recorded session; no third-party dependencies."""
import argparse
import collections
import csv
import json
from pathlib import Path
import statistics


def timing(rows, key):
    stamps = sorted(set(int(r[key]) for r in rows if r.get(key)))
    if len(stamps) < 2:
        return {'samples': len(stamps), 'hz': None}
    gaps = [(b-a)/1e9 for a, b in zip(stamps, stamps[1:])]
    return {'samples': len(stamps), 'hz': (len(stamps)-1)/((stamps[-1]-stamps[0])/1e9),
            'max_gap_s': max(gaps), 'median_gap_s': statistics.median(gaps),
            'p95_gap_s': sorted(gaps)[min(len(gaps)-1, int(.95*len(gaps)))]}


def report(directory, seconds=60):
    root = Path(directory)
    current = json.loads((root/'current.json').read_text())
    session = Path(current['session'])
    samples = collections.deque(maxlen=10000)
    with (session/'samples.csv').open(newline='') as f:
        for row in csv.DictReader(f):
            samples.append(row)
    cutoff = int(samples[-1]['mono_ns'])-seconds*10**9 if samples else 0
    samples = [row for row in samples if int(row['mono_ns']) >= cutoff]
    imu = collections.defaultdict(lambda: collections.deque(maxlen=10000))
    if (session/'imu.csv').exists():
        with (session/'imu.csv').open(newline='') as f:
            for row in csv.DictReader(f):
                if int(row['mono_ns']) >= cutoff:
                    imu[row['source']].append(row)
    status = json.loads((root/'status.json').read_text())
    last = samples[-1] if samples else {}
    result = {'session': str(session), 'window_seconds': seconds, 'samples': timing(samples, 'mono_ns'),
              'imu': {source: timing(rows, 'source_mono_ns') for source, rows in imu.items()},
              'motor_labels': dict(collections.Counter(r['motor_state'] for r in samples)),
              'latest': {key: last.get(key) for key in ['armed','left_pwm','right_pwm','rotation_feedback',
                          'left_power_w','right_power_w','uwb_fresh','vision_active','gimbal_connected','follow_active']},
              'loss': {key: status.get(key) for key in ['sequence_gaps','sender_drops','queue_drops','udp_socket_drops']},
              'ros': status.get('ros')}
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', default='/home/gongcheng/manta_logs')
    parser.add_argument('--seconds', type=int, default=60)
    args = parser.parse_args()
    print(json.dumps(report(args.directory, args.seconds), indent=2, ensure_ascii=False))
