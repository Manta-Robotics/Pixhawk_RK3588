#!/usr/bin/env python3
"""Replay the same observations through the reference and native admission cores.

No hardware is opened. Run with the absolute path to manta-replay.
"""
import json
import math
from pathlib import Path
import random
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from uwb_measurement_filter import GatedUwbKalman
from uwb_fusion import DualUwbFusion


def compare(expected, actual, path=''):
    if isinstance(expected, dict):
        assert isinstance(actual, dict) and expected.keys() == actual.keys(), (path, expected, actual)
        for key in expected:
            compare(expected[key], actual[key], path + '.' + key)
    elif isinstance(expected, (list, tuple)):
        assert len(expected) == len(actual), (path, expected, actual)
        for index, value in enumerate(expected):
            compare(value, actual[index], path + f'[{index}]')
    elif isinstance(expected, (float, int)) and not isinstance(expected, bool):
        assert isinstance(actual, (float, int)) and math.isclose(expected, actual, rel_tol=1e-8, abs_tol=1e-8), (path, expected, actual)
    else:
        assert expected == actual, (path, expected, actual)


def main():
    rng = random.Random(82473)
    requests, expected = [], []
    # Vary calibration, heading wrap, adaptive modes, quarantine, raw-window
    # recovery, radio loss, target identity, invalid input and clock reset.
    for options in ({}, {'bearing_sign': -1, 'bearing_scale': .8148, 'bearing_offset_deg': -4.816, 'mount_yaw_deg': 90},
                    {'config': {'occlusion_enabled': False}}, {'config': {'adaptive_enabled': False}}):
        requests.append({'op': 'reset', 'options': options})
        expected.append({'ok': True})
        reference = GatedUwbKalman(**options)
        at = 1000
        for i in range(1200):
            at += rng.choice([20, 25, 25, 50])
            distance = 3 + .2 * math.sin(i / 13) + rng.gauss(0, .02)
            azimuth = 12 * math.sin(i / 60) + rng.gauss(0, .3)
            if 200 <= i < 240 or 500 <= i < 540:
                distance += 5
                azimuth += 70
            if i == 800:
                at = 500
            observation = dict(distance_m=distance, azimuth_deg=azimuth, now_ms=at,
                               heading_deg=((i / 20 + 170) + 180) % 360 - 180,
                               sequence_id=i, tag_id=1 if i < 600 else 2,
                               valid=not 400 <= i < 410)
            if i % 137 == 0:
                observation['distance_m'] = 25
            if i % 139 == 0:
                observation['azimuth_deg'] = 181
            requests.append(observation)
            expected.append(reference.update(**observation))
            if i % 17 == 0:
                requests.append(observation)
                expected.append(reference.update(**observation))
            if i % 31 == 0:
                requests.append({'op': 'result', 'now_ms': at + 12, 'heading_deg': observation['heading_deg']})
                expected.append(reference.result(at + 12, observation['heading_deg']))
    for gated in (False, True):
        options = dict(require_gated_measurements=gated, mount_yaw_deg=[0, 90], receiver_position_x_m=[.1, -.1])
        fusion = DualUwbFusion(**options)
        filters = {i: GatedUwbKalman(mount_yaw_deg=(i-1)*90) for i in (1, 2)}
        requests.append({'op': 'fusion_reset', 'options': options}); expected.append({'ok': True})
        for index in range(300):
            now = 1000 + index * 25
            for receiver_id in (1, 2):
                distance = 2 + rng.gauss(0, .03)
                angle = 45 - (receiver_id-1)*90 + rng.gauss(0, .3)
                if index > 100 and receiver_id == 2: angle += 40
                if index > 200 and receiver_id == 1: angle -= 55
                for field, value in [('VALID', int(index % 91 != 0)), ('DIST', distance), ('AZ', angle), ('EL', 5), ('GOOD', index)]:
                    name = f'UWB{receiver_id}_{field}'
                    requests.append({'op': 'fusion_update', 'name': name, 'value': value, 'now_ms': now, 'source': [1, 1]})
                    expected.append(fusion.update(name, value, now_ms=now, source=(1,1)))
                    if not gated and field in ('DIST','AZ','EL'):
                        fusion.set_filtered(receiver_id, field, value)
                        requests.append({'op':'fusion_set','receiver_id':receiver_id,'field':field,'value':value});expected.append(None)
                if gated:
                    admitted = filters[receiver_id].update(distance, angle, now, 0, valid=index % 91 != 0)
                    fusion.apply_measurement_result(receiver_id, admitted)
                    requests.append({'op':'fusion_apply','receiver_id':receiver_id,'result':admitted});expected.append(None)
            requests.append({'op':'fusion_result','now_ms':now});expected.append(fusion.fused(now))
        requests.append({'op':'fusion_result','now_ms':now+5000});expected.append(fusion.fused(now+5000))
    result = subprocess.run([sys.argv[1]], input=''.join(json.dumps(r) + '\n' for r in requests),
                            text=True, capture_output=True, check=True, timeout=60)
    actual = [json.loads(line) for line in result.stdout.splitlines()]
    assert len(actual) == len(expected), (len(actual), len(expected), result.stderr)
    for index, (left, right) in enumerate(zip(expected, actual)):
        compare(left, right, f'frame[{index}] {requests[index]}')
    print(f'Native/Python parity: {len(expected)} frames passed (all output fields, 1e-8 tolerance).')


if __name__ == '__main__':
    main()
