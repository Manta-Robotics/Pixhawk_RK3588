#!/usr/bin/env python3
"""Real MAVLink -> native UWB pipeline -> native follow regression tests.

Usage: python scripts/check_native_uwb_contract.py WIRE_REPLAY FOLLOW_REPLAY
Only JSONL replay executables are launched; no device or network is opened.
"""
import copy
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys
import unittest

from pymavlink.dialects.v20 import ardupilotmega as mav

ROOT = Path(__file__).resolve().parents[1]
WIRE, FOLLOW = (str(Path(p).resolve()) for p in sys.argv[1:3])


def replay(binary, requests):
    result = subprocess.run(
        [binary], input=''.join(json.dumps(r) + '\n' for r in requests),
        text=True, capture_output=True, check=True, timeout=30, cwd=ROOT)
    rows = [json.loads(line) for line in result.stdout.splitlines()]
    assert len(rows) == len(requests), result.stderr
    assert all('error' not in row for row in rows if isinstance(row, dict)), rows
    return rows


class UwbWireContractTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((ROOT / 'config/system.config.json').read_text())
        # Existing admission regressions exercise the optional guarded mode.
        # The deployed smoothing-only mode is covered separately below.
        self.config['uwb_measurement_filter']['trust_checks_enabled'] = True
        self.sender = mav.MAVLink(None, srcSystem=1, srcComponent=1)
        self.now = 1_000_000
        self.requests = [dict(op='reset', config=self.config)]
        self.needs_sync = True
        self.sync_heading = True
        self.source_base, self.source_at = 100, self.now
        self.receive([
            mav.MAVLink_heartbeat_message(10, 3, 129, 0, 4, 3),
            mav.MAVLink_param_value_message(b'PILOT_STEER_TYPE', 0, 9, 1, 0),
            self.attitude()])

    def attitude(self):
        message = mav.MAVLink_attitude_message(self.source_now(), 0, 0, 0, 0, 0, 0)
        message.auto_clock = True
        return message

    def source_now(self):
        return (self.source_base + self.now - self.source_at) & 0xFFFFFFFF

    @staticmethod
    def value(stamp, field, value, receiver=1):
        return mav.MAVLink_named_value_float_message(
            stamp, f'UWB{receiver}_{field}'.encode(), value)

    def pair(self, stamp, distance=3, angle=0, receiver=1):
        return [self.value(stamp, field, value, receiver) for field, value in
                [('DIST', distance), ('EL', 0), ('AZ', angle)]]

    def legacy_pair(self, stamp, distance=3, validity=False):
        fields = [self.value(stamp & 0xFFFFFFFF, 'DIST', distance),
                  self.value((stamp + 1) & 0xFFFFFFFF, 'EL', 0)]
        if validity:
            fields.append(self.value((stamp + 2) & 0xFFFFFFFF, 'VALID', 1))
        fields.append(self.value((stamp + (3 if validity else 2)) & 0xFFFFFFFF, 'AZ', 0))
        return fields

    def receive(self, messages, split=False):
        if self.needs_sync:
            first = next((m for m in messages if m.get_type() == 'NAMED_VALUE_FLOAT'), None)
            if first is not None:
                self.sync_session((first.time_boot_ms - 1) & 0xFFFFFFFF,
                                  heading=self.sync_heading)
        packet = b''
        for message in messages:
            if getattr(message, 'auto_clock', False):
                message.time_boot_ms = self.source_now()
            packet += message.pack(self.sender)
            self.sender.seq = (self.sender.seq + 1) % 256
        chunks = [packet[:7], packet[7:]] if split else [packet]
        for chunk in chunks:
            self.requests.append(dict(op='receive', hex=chunk.hex(),
                                      wall_ms=self.now, mono_ms=self.now))

    def sync_session(self, boot_ms, heading=True, **kwargs):
        self.needs_sync = False
        self.source_base, self.source_at = boot_ms, self.now
        self.requests.append(dict(op='sync', boot_ms=boot_ms, seq=self.sender.seq,
                                  wall_ms=self.now, mono_ms=self.now, **kwargs))
        self.sender.seq = (self.sender.seq + 1) % 256
        if heading:
            self.receive([self.attitude()])

    def results(self):
        return replay(WIRE, self.requests)

    def pose(self, stamp, yaw, rate=0):
        return mav.MAVLink_attitude_message(stamp & 0xFFFFFFFF, 0, 0,
                                            math.radians(yaw), 0, 0, math.radians(rate))

    def test_batch_and_unbatched_moving_observations_have_same_filter_result(self):
        def run(batch, legacy):
            self.setUp()
            self.sync_session(999, heading=False)
            self.config['gimbal']['beacon'].update(yaw_source_sign=1, yaw_source_scale=1, yaw_offset_deg=0)
            indices = []
            for group in range(15):
                messages = []
                for i in range(group * 5, group * 5 + 5):
                    stamp = 1000 + i * 20
                    distance = 3 + i * .02
                    angle = i * .1
                    fields = self.legacy_pair(stamp, distance) if legacy else self.pair(stamp, distance, angle)
                    if legacy:
                        fields[-1].value = angle
                    messages += [self.pose(stamp, 0)] + fields
                    if not batch:
                        self.now = 1_000_000 + i * 20 + 1
                        self.receive(messages)
                        messages = []
                if batch:
                    self.now = 1_000_000 + (group * 5 + 4) * 20 + 1
                    self.receive(messages)
                indices.append(len(self.requests) - 1)
            records = [e['payload'] for row in self.results() for e in row['events']
                       if e['type'] == 'uwb_measurement' and e['payload'].get('complete')]
            return records
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                immediate, batched = run(False, legacy), run(True, legacy)
                self.assertEqual(len(immediate), 75)
                self.assertEqual(len(batched), 75)
                for a, b in zip(immediate, batched):
                    self.assertEqual(a['status'], 'accepted')
                    self.assertEqual(b['status'], 'accepted')
                    for key in ('filteredDistanceM', 'filteredAzimuthDeg', 'filterTimeMs'):
                        self.assertAlmostEqual(a[key], b[key], places=8)
                self.assertAlmostEqual(batched[0]['transportDelayMs'], 80)
                self.assertEqual(batched[0]['receivedAt'], batched[4]['receivedAt'])
                self.assertEqual(batched[4]['filterTimeMs'] - batched[0]['filterTimeMs'], 80)

    def test_turning_pose_is_interpolated_at_sample_time_including_yaw_wrap(self):
        self.sync_session(999, heading=False)
        self.config['gimbal']['beacon'].update(yaw_source_sign=1, yaw_source_scale=1, yaw_offset_deg=0)
        self.now += 120
        self.receive([self.pose(1000, 179, 20), self.pose(1100, -179, 20)] + self.pair(1050, 3, 0))
        row = self.results()[-1]
        record = next(e['payload'] for e in row['events'] if e['type'] == 'uwb_measurement')
        self.assertEqual(record['headingMethod'], 'interpolated')
        self.assertAlmostEqual(abs(record['headingDeg']), 180, places=5)
        self.assertAlmostEqual(record['transportDelayMs'], 69)
        self.assertAlmostEqual(record['filteredAzimuthDeg'], 0, places=5)
        # Published body bearing uses the current -179 degree vehicle heading,
        # while the immutable record retains the original measurement frame.
        self.assertAlmostEqual(row['state']['uwb']['azimuthDeg'], -1, places=4)

    def test_pose_extrapolation_is_bounded_and_old_pose_never_replaces_live_yaw(self):
        self.sync_session(999, heading=False)
        self.now += 160
        self.receive([self.pose(1000, 10, 50)] + self.pair(1080))
        good = len(self.requests) - 1
        self.receive(self.pair(1150))
        bad = len(self.requests) - 1
        self.receive([self.pose(1150, 17.5, 50), self.pose(1050, 12.5, 50)])
        rows = self.results()
        records = lambda row: [e['payload'] for e in row['events'] if e['type'] == 'uwb_measurement']
        self.assertEqual(records(rows[good])[0]['headingMethod'], 'yaw_rate_extrapolated')
        self.assertAlmostEqual(records(rows[good])[0]['headingDeg'], 14, places=5)
        self.assertEqual(records(rows[bad])[0]['status'], 'rejected')
        self.assertEqual(records(rows[bad])[0]['reason'], 'heading_unavailable')
        self.assertAlmostEqual(rows[-1]['state']['attitude']['yaw'], 17.5, places=5)

    def test_clock_refresh_does_not_change_sample_spacing_or_reuse_old_pose(self):
        self.sync_session(0xFFFFFFD0, heading=False)
        self.now += 20
        self.receive([self.pose(0xFFFFFFE0, 0)] + self.pair(0xFFFFFFE0))
        first = len(self.requests) - 1
        self.now += 1000
        self.requests.append(dict(op='tick', wall_ms=self.now, mono_ms=self.now))
        self.now += 20  # A different RTT must not change the filter clock offset.
        self.sync_session(972, heading=False)
        self.receive([self.pose(982, 0)] + self.pair(982))
        rows = self.results()
        def sample(row):
            return next(e['payload'] for e in row['events'] if e['type'] == 'uwb_measurement')
        self.assertEqual(sample(rows[-1])['filterTimeMs'] - sample(rows[first])['filterTimeMs'], 1014)
        self.requests.append(dict(op='disconnect', wall_ms=self.now, mono_ms=self.now))
        self.reconnect(heartbeat_only=True)
        self.sync_session(99, heading=False)
        self.receive(self.pair(100))
        last = self.results()[-1]
        self.assertEqual(sample(last)['headingMethod'], 'unavailable')
        self.assertFalse(last['state']['uwb']['fresh'])

    def test_records_preserve_every_batch_frame_and_do_not_label_old_values_as_new(self):
        self.receive(self.pair(100) + [self.value(120, 'DIST', .2)] + self.pair(140))
        rows = self.results()
        records = [e['payload'] for row in rows for e in row['events'] if e['type'] == 'uwb_measurement']
        self.assertEqual([r['status'] for r in records], ['accepted', 'incomplete', 'accepted'])
        self.assertEqual([r['recordId'] for r in records], [1, 2, 3])
        self.assertAlmostEqual(records[1]['rawDistanceM'], .2)
        self.assertIsNone(records[1]['rawAzimuthDeg'])
        self.assertEqual(records[0]['sourceTimeKind'], 'lua_processing')
        self.assertLessEqual(records[0]['sampleAt'], records[0]['receivedAt'])
        self.assertTrue(rows[-1]['state']['uwb']['clearanceStop'])

    def test_multiple_positions_in_one_read_are_all_filtered_and_published(self):
        self.receive(self.pair(100) + self.pair(120, 3.1))
        result = self.results()[-1]
        updates = [event['payload'] for event in result['events'] if event['type'] == 'uwb']
        self.assertGreaterEqual(len(updates), 2)
        self.assertGreater(updates[-1]['distanceM'], updates[0]['distanceM'])
        self.assertEqual(updates[-1]['measurementStatus'], 'trusted')
        self.assertLessEqual(updates[-1]['measurementUpdatedAt'], self.now)

    def test_partial_record_is_finalized_once_on_timeout_or_disconnect(self):
        for disconnect in (False, True):
            with self.subTest(disconnect=disconnect):
                self.setUp()
                self.receive([self.value(100, 'DIST', .2)])
                received = self.now
                self.now += 101
                self.requests.append(dict(op='disconnect' if disconnect else 'tick',
                                          wall_ms=self.now, mono_ms=self.now))
                self.requests.append(dict(op='tick', wall_ms=self.now+1, mono_ms=self.now+1))
                if not disconnect:
                    self.receive([self.value(100, 'EL', 0), self.value(100, 'AZ', 0)])
                rows = self.results()
                records = [e['payload'] for row in rows for e in row['events']
                           if e['type'] == 'uwb_measurement']
                self.assertEqual(len(records), 1)
                self.assertEqual(records[0]['reason'], 'session_closed' if disconnect else 'field_timeout')
                self.assertEqual(records[0]['receivedAt'], received)
                self.assertEqual(records[0]['finalizedAt'], received+101)
                self.assertIsNone(records[0]['rawAzimuthDeg'])
                self.assertIsNone(rows[-1]['state']['uwb']['measurementUpdatedAt'])

    def test_superseded_partial_record_retains_its_actual_receipt_time(self):
        self.receive([self.value(100, 'DIST', .2)])
        received = self.now
        self.now += 40
        self.receive(self.pair(140))
        records = [e['payload'] for e in self.results()[-1]['events']
                   if e['type'] == 'uwb_measurement']
        self.assertEqual(records[0]['reason'], 'superseded_partial_frame')
        self.assertEqual(records[0]['receivedAt'], received)
        self.assertEqual(records[0]['finalizedAt'], received+40)
        self.assertEqual(records[1]['receivedAt'], received+40)

    def test_delayed_sample_keeps_its_age_instead_of_becoming_fresh_on_receipt(self):
        self.config['uwb_dual']['freshness_ms'] = 600
        self.sync_session(999, heading=False)
        self.now += 400
        self.receive([self.pose(1000, 0)] + self.pair(1000))
        received = self.now
        self.requests.append(dict(op='tick', wall_ms=received+301, mono_ms=received+301))
        rows = self.results()
        record = next(e['payload'] for e in rows[-2]['events'] if e['type'] == 'uwb_measurement')
        self.assertEqual(record['status'], 'accepted')
        self.assertEqual(record['transportDelayMs'], 399)
        self.assertEqual(rows[-2]['state']['uwb']['measurementSampleAt'], received-399)
        # 700 ms source age, despite only 301 ms since receiving the frame.
        self.assertFalse(rows[-1]['state']['uwb']['fresh'])

    def test_turning_batch_uses_each_samples_heading_with_real_bearing_calibration(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                self.setUp()
                self.sync_session(999, heading=False)
                calibration = self.config['gimbal']['beacon']
                scale = calibration['yaw_source_sign'] * calibration['yaw_source_scale']
                offset = calibration['yaw_offset_deg']
                if not legacy:
                    self.receive([self.value(999, 'PROTO', 2)])
                for group in range(10):
                    poses, fields = [], []
                    for i in range(group*5, group*5+5):
                        stamp, yaw = 1000+i*20, 10+i*.6
                        angle = (-yaw-offset)/scale  # Stationary target at world bearing 0.
                        poses.append(self.pose(stamp, yaw, 30))
                        frame = self.legacy_pair(stamp) if legacy else self.pair(stamp)
                        frame[-1].value = angle
                        fields += frame
                    self.now = 1_000_001+(group*5+4)*20
                    # All five poses precede the five delayed observations:
                    # applying only the latest heading would invent target motion.
                    self.receive(poses+fields)
                rows = self.results()
                records = [e['payload'] for row in rows for e in row['events']
                           if e['type'] == 'uwb_measurement' and e['payload'].get('complete')]
                self.assertEqual(len(records), 50)
                for i, record in enumerate(records):
                    self.assertEqual(record['status'], 'accepted')
                    self.assertAlmostEqual(record['filteredDistanceM'], 3, places=5)
                    self.assertAlmostEqual(record['filteredBearingDeg'], -(10+i*.6), places=4)
                body = rows[-1]['state']['uwb']['azimuthDeg']*scale+offset
                self.assertAlmostEqual(body, -(10+49*.6), places=4)

    def test_control_consumers_use_source_age_and_sample_spacing(self):
        self.sync_session(999, heading=False)
        self.now += 120
        self.receive([self.pose(1000, 0), self.pose(1100, 0)] +
                     self.pair(1000, 3) + self.pair(1100, 3.1))
        row = self.results()[-1]
        observations = [e['payload'] for e in row['events']
                        if e['type'] == 'uwb' and e['payload']['fresh']]
        self.assertEqual(len(observations), 2)
        config = copy.deepcopy(self.config['uwb_follow'])
        config.update(execution_mode='relative_imu', uwb_timeout_ms=100,
                      bearing_sign=1, bearing_scale=1, bearing_offset_deg=0)
        config['relative_nmpc']['enabled'] = False
        requests = [dict(op='reset', config=config)]
        for u in observations:
            telemetry = copy.deepcopy(row['state'])
            telemetry['uwb'] = u
            requests.append(dict(op='preflight', now=self.now, telemetry=telemetry, connected=True))
            requests.append(dict(op='relative', now=self.now, telemetry=telemetry,
                                 preflight=dict(distanceM=u['distanceM'], azimuthDeg=0,
                                                measuredYawRateDegS=0)))
        result = replay(FOLLOW, requests)
        self.assertEqual(result[1]['reason'], 'uwb_stale')  # 119 ms old at receipt.
        self.assertTrue(result[3]['safe'])                 # Next frame is only 19 ms old.
        self.assertEqual(result[2]['rangeRateMps'], 0)
        self.assertGreater(result[4]['rangeRateMps'], 0)   # Equal receipt times cannot erase dt.

    def test_batch_near_range_reaches_controller_and_stops_propulsion(self):
        self.check_batch_near_range(self.pair)

    def test_legacy_batch_near_range_reaches_controller_and_stops_propulsion(self):
        def interleaved_pair(stamp, distance=3):
            dist, el, az = self.legacy_pair(stamp, distance)
            return [dist, self.attitude(), el, self.attitude(), az]
        self.check_batch_near_range(interleaved_pair)

    def check_batch_near_range(self, pair, near_fields=None, trailing_far=False):
        sample_indices = []
        for i in range(21):
            self.now = 1_000_000 + i * 100
            messages = [self.attitude()] + pair(1000 + i * 100)
            if i == 20:
                messages += near_fields if near_fields is not None else pair(3020, .2)
                if trailing_far:
                    messages += pair(3040) + pair(3060)
            self.receive(messages)
            sample_indices.append(len(self.requests) - 1)
        rows = self.results()
        last = rows[-1]['state']['uwb']
        self.assertTrue(last['clearanceStop'])
        self.assertAlmostEqual(last['clearanceDistanceM'], .2)
        if trailing_far:
            self.assertAlmostEqual(last['sensors'][0]['rawDistanceM'], 3)
        else:
            self.assertAlmostEqual(last['sensors'][0]['rawDistanceM'], .2)
            self.assertEqual(last['measurementStatus'], 'suspect')
            self.assertFalse(last['fresh'])
        config = copy.deepcopy(self.config['uwb_follow'])
        for source, dest in [('yaw_source_sign', 'bearing_sign'),
                             ('yaw_source_scale', 'bearing_scale'),
                             ('yaw_offset_deg', 'bearing_offset_deg')]:
            config[dest] = self.config['gimbal']['beacon'][source]
        # Also cover the non-NMPC relative controller. Do not rely on a solver
        # failing or timing out to produce a zero command in this test.
        for enabled in (True, False):
            config['relative_nmpc']['enabled'] = enabled
            config['relative_nmpc']['solve_budget_ms'] = 1000
            requests = [dict(op='reset', config=config)]
            for i, index in enumerate(sample_indices):
                requests.append(dict(op='intermittent', now=1_000_000 + i * 100,
                                     telemetry=rows[index]['state'], connected=True,
                                     controlMode='relative_manual'))
            controls = replay(FOLLOW, requests)
            self.assertGreater(controls[-2]['throttlePercent'], 0, controls[-2])
            self.assertTrue(controls[-1]['pausePropulsion'], controls[-1])
            self.assertEqual(controls[-1]['throttlePercent'], 0)
            self.assertEqual(controls[-1]['steeringDegrees'], 0)

    def test_near_then_far_in_one_read_keeps_stop_for_both_controllers(self):
        for legacy in (False, True):
            for incomplete in ('complete', 'dist_only', 'lost_el', 'six_ms'):
                with self.subTest(legacy=legacy, incomplete=incomplete):
                    self.setUp()
                    pair = self.legacy_pair if legacy else self.pair
                    near = pair(3020, .2)
                    if incomplete == 'dist_only':
                        near = near[:1]
                    elif incomplete == 'lost_el':
                        # A lost wire packet, not merely a field with another name.
                        near = [near[0], near[-1]]
                        original_receive = self.receive
                        def receive_with_loss(messages, split=False):
                            if near[0] in messages:
                                index = messages.index(near[0]) + 1
                                original_receive(messages[:index], split)
                                self.sender.seq = (self.sender.seq + 1) % 256
                                original_receive(messages[index:], split)
                            else:
                                original_receive(messages, split)
                        self.receive = receive_with_loss
                    elif incomplete == 'six_ms':
                        near = [self.value(3020, 'DIST', .2), self.value(3025, 'EL', 0),
                                self.value(3026, 'AZ', 0)]
                    try:
                        self.check_batch_near_range(pair, near, trailing_far=True)
                    finally:
                        self.__dict__.pop('receive', None)

    def test_clearance_requires_consumption_and_later_sustained_safe_frames(self):
        self.receive(self.pair(80) + self.pair(100, .2) + self.pair(120))
        for i in range(1, 9):
            self.now += 100
            self.receive([self.attitude()] + self.pair(120 + i * 100))
        unconsumed = len(self.requests) - 1
        self.requests.append(dict(op='ack_uwb_clearance', wall_ms=self.now, mono_ms=self.now))
        self.receive(self.pair(940) + self.pair(960))
        same_batch = len(self.requests) - 1
        for i in range(1, 7):
            self.now += 100
            self.receive([self.attitude()] + self.pair(960 + i * 100))
        rows = self.results()
        self.assertTrue(rows[unconsumed]['state']['uwb']['clearanceStop'])
        self.assertTrue(rows[same_batch]['state']['uwb']['clearanceStop'])
        self.assertTrue(rows[-2]['state']['uwb']['clearanceStop'])
        self.assertFalse(rows[-1]['state']['uwb']['clearanceStop'])

    def test_safe_recovery_gap_and_new_near_restart_clearance_confirmation(self):
        self.config['uwb_measurement_filter']['enabled'] = False
        self.receive(self.pair(100, .2))
        self.requests.append(dict(op='ack_uwb_clearance', wall_ms=self.now, mono_ms=self.now))
        for i in range(1, 6):
            self.now += 100
            self.receive(self.pair(100 + i * 100, 3))
        self.now += 200  # A gap must restart the 500 ms safe window.
        self.receive(self.pair(800, 3))
        gap = len(self.requests) - 1
        self.now += 100
        self.receive([self.value(900, 'DIST', .2)] + self.pair(920, 3))
        for i in range(1, 8):
            self.now += 100
            self.receive(self.pair(920 + i * 100, 3))
        rows = self.results()
        self.assertTrue(rows[gap]['state']['uwb']['clearanceStop'])
        self.assertTrue(rows[-1]['state']['uwb']['clearanceStop'])  # New stop not consumed.

    def test_disabled_receiver_and_stale_distance_cannot_latch_stop(self):
        self.receive(self.pair(100))
        self.receive(self.pair(80, .2) + self.pair(100, .2, receiver=2))
        self.assertFalse(self.results()[-1]['state']['uwb']['clearanceStop'])

    def test_finite_near_distance_below_measurement_minimum_still_stops(self):
        for distance in (0, .05, -1, float('nan'), float('inf')):
            with self.subTest(distance=distance):
                self.setUp()
                self.receive([self.value(100, 'DIST', distance)])
                u = self.results()[-1]['state']['uwb']
                self.assertEqual(u['clearanceStop'], distance in (0, .05))
                self.assertFalse(u['fresh'])

    def test_reconnect_old_complete_frames_cannot_poison_reboot_clock(self):
        for pair in (self.pair, self.legacy_pair):
            with self.subTest(legacy=pair == self.legacy_pair):
                self.setUp()
                self.receive(pair(10000))
                self.requests.append(dict(op='disconnect', wall_ms=self.now, mono_ms=self.now))
                self.reconnect()
                self.needs_sync = False
                self.receive(pair(10000))  # Already queued behind the reconnect heartbeat.
                before_sync = len(self.requests) - 1
                self.sync_session(99)
                self.receive(pair(10000))  # A delayed high-uptime old observation after reply.
                after_sync = len(self.requests) - 1
                self.receive(pair(100, .2))
                rows = self.results()
                for index in (before_sync, after_sync):
                    self.assertIsNone(rows[index]['state']['uwb']['sensors'][0]['rawDistanceM'])
                self.assertTrue(rows[-1]['state']['uwb']['fresh'])
                self.assertTrue(rows[-1]['state']['uwb']['clearanceStop'])

    def test_reconnect_old_tail_cannot_pair_with_new_distance_at_same_stamp(self):
        for old_after_sync in (False, True):
            with self.subTest(old_after_sync=old_after_sync):
                self.setUp()
                self.receive([self.value(100, 'DIST', 3)])
                self.requests.append(dict(op='disconnect', wall_ms=self.now, mono_ms=self.now))
                self.reconnect()
                self.needs_sync = False
                if old_after_sync:
                    self.sync_session(99)
                self.receive([self.value(100, 'EL', 0), self.value(100, 'AZ', 30)])
                if not old_after_sync:
                    self.sync_session(99)
                self.receive([self.value(100, 'DIST', .2)])
                partial = len(self.requests) - 1
                self.receive([self.value(100, 'EL', 0), self.value(100, 'AZ', 0)])
                rows = self.results()
                self.assertIsNone(rows[partial]['state']['uwb']['measurementUpdatedAt'])
                self.assertTrue(rows[partial]['state']['uwb']['clearanceStop'])
                self.assertAlmostEqual(rows[-1]['state']['uwb']['azimuthDeg'], 0)

    def test_fresh_diagnostics_before_first_distance_are_preserved(self):
        self.receive([self.value(100, 'TAG', 123), self.value(101, 'VALID', 0)])
        first = len(self.requests) - 1
        self.receive([self.value(120, 'VALID', 1)] + self.legacy_pair(121))
        rows = self.results()
        sensor = rows[first]['state']['uwb']['sensors'][0]
        self.assertEqual(sensor['tagId'], 123)
        self.assertFalse(sensor['valid'])
        self.assertIsNone(sensor['rawDistanceM'])
        self.assertTrue(rows[-1]['state']['uwb']['fresh'])

    def test_session_requires_fresh_matching_target_timesync_reply(self):
        for bad in ('wrong_nonce', 'wrong_source', 'expired', 'before_fence'):
            with self.subTest(bad=bad):
                self.setUp()
                self.needs_sync = False
                if bad == 'expired':
                    self.now += 501
                kwargs = {'nonce': -1} if bad == 'wrong_nonce' else {'sysid': 2} if bad == 'wrong_source' else {}
                self.sync_session(100, **kwargs)
                self.receive(self.pair(100 if bad == 'before_fence' else 120))
                self.assertIsNone(self.results()[-1]['state']['uwb']['measurementUpdatedAt'])

    def test_periodic_timesync_refresh_handles_drift_wrap_and_peer_reboot(self):
        self.receive(self.legacy_pair(0xFFFFFFF0))
        self.now += 1100
        self.requests.append(dict(op='tick', wall_ms=self.now, mono_ms=self.now))
        self.sync_session(1100)
        self.receive(self.legacy_pair(1110))
        refreshed = len(self.requests) - 1
        self.now += 2000
        self.receive([mav.MAVLink_heartbeat_message(10, 3, 129, 0, 4, 3)])
        self.requests.append(dict(op='tick', wall_ms=self.now, mono_ms=self.now))
        self.sync_session(100)
        self.receive(self.legacy_pair(1110))  # Delayed pre-reboot frame exceeds fresh clock.
        stale = len(self.requests) - 1
        self.receive(self.legacy_pair(110, .2))
        rows = self.results()
        self.assertTrue(rows[refreshed]['state']['uwb']['fresh'])
        self.assertIsNone(rows[stale]['state']['uwb']['measurementUpdatedAt'])
        self.assertTrue(rows[-1]['state']['uwb']['fresh'])

    def test_nan_or_infinity_clears_pending_fields_and_poisoned_frame_cannot_revive(self):
        for invalid in (float('nan'), float('inf'), -float('inf')):
            for field in ('DIST', 'AZ', 'EL'):
                with self.subTest(invalid=invalid, field=field):
                    self.setUp()
                    self.receive([self.value(100, 'DIST', 3), self.value(100, field, invalid)]
                                 + self.pair(100))
                    bad_index = len(self.requests) - 1
                    self.now += 20
                    self.receive(self.pair(120))
                    rows = self.results()
                    bad = rows[bad_index]['state']['uwb']
                    self.assertFalse(bad['fresh'])
                    self.assertIsNone(bad['measurementUpdatedAt'])
                    self.assertEqual(bad['sensors'][0]['filterReason'], 'non_finite_measurement')
                    self.assertTrue(rows[-1]['state']['uwb']['fresh'])

    def test_nan_new_distance_cannot_pair_with_old_pending_distance(self):
        self.receive([self.value(100, 'DIST', 3)])  # lost AZ
        self.now += 20
        self.receive([self.value(120, 'DIST', float('nan')), self.value(120, 'AZ', 0)])
        u = self.results()[-1]['state']['uwb']
        self.assertFalse(u['fresh'])
        self.assertIsNone(u['measurementUpdatedAt'])

    def test_lost_fields_cannot_pair_across_frames_or_sources(self):
        for split, source_change in ((False, False), (True, False), (False, True)):
            with self.subTest(split=split, source_change=source_change):
                self.setUp()
                self.receive([self.value(100, 'DIST', 3)], split)
                self.now += 50
                if source_change:
                    self.sender.srcComponent = 2
                self.receive([self.value(100 if source_change else 150, 'AZ', 20)], split)
                bad_index = len(self.requests) - 1
                self.now += 50
                component = self.sender.srcComponent
                self.sender.srcComponent = 1
                self.receive([self.pose(200, 0)])
                self.sender.srcComponent = component
                self.receive(self.pair(200), split)
                rows = self.results()
                self.assertIsNone(rows[bad_index]['state']['uwb']['measurementUpdatedAt'])
                self.assertTrue(rows[-1]['state']['uwb']['fresh'])

    def test_complete_duplicate_does_not_refresh_measurement_or_raw_range(self):
        self.receive(self.pair(100))
        self.now += 900
        self.receive([self.attitude()] + self.pair(100))
        u = self.results()[-1]['state']['uwb']
        self.assertFalse(u['fresh'])
        self.assertEqual(u['sensors'][0]['measurementUpdatedAt'], 1_000_000)
        self.assertEqual(u['sensors'][0]['rawDistanceUpdatedAt'], 1_000_000)

    def test_nonconsecutive_old_frame_replay_cannot_refresh_or_interrupt_current_frame(self):
        self.receive(self.pair(100))
        self.now += 25
        self.receive(self.pair(125))
        self.now += 900
        self.receive([self.attitude()] + self.pair(100))
        replay_index = len(self.requests) - 1
        self.now += 25
        self.receive([self.value(1050, 'DIST', 3)] + self.pair(125)
                     + [self.value(1050, 'EL', 0), self.value(1050, 'AZ', 0)])
        rows = self.results()
        u = rows[replay_index]['state']['uwb']
        self.assertFalse(u['fresh'])
        self.assertEqual(u['sensors'][0]['measurementUpdatedAt'], 1_000_025)
        self.assertEqual(u['sensors'][0]['rawDistanceUpdatedAt'], 1_000_025)
        self.assertTrue(rows[-1]['state']['uwb']['fresh'])

    def test_missing_elevation_never_commits_and_cannot_reuse_previous_elevation(self):
        self.receive(self.pair(100))
        self.now += 900
        self.receive([self.attitude(), self.value(1000, 'DIST', 3), self.value(1000, 'AZ', 0)])
        missing_index = len(self.requests) - 1
        self.now += 20
        self.receive([self.value(1020, 'EL', 10)])  # another frame cannot complete it
        rows = self.results()
        for index in (missing_index, len(rows) - 1):
            self.assertFalse(rows[index]['state']['uwb']['fresh'])
            self.assertEqual(rows[index]['state']['uwb']['sensors'][0]['measurementUpdatedAt'], 1_000_000)

    def test_every_field_order_commits_only_after_all_three_valid_fields(self):
        for order in itertools.permutations([('DIST', 3), ('AZ', 0), ('EL', 25)]):
            with self.subTest(order=order):
                self.setUp()
                self.receive(self.pair(80))  # Establish this receiver's new-session DIST boundary.
                self.now += 20
                for field, value in order:
                    self.receive([self.value(100, field, value)])
                rows = self.results()
                self.assertEqual(rows[-3]['state']['uwb']['measurementUpdatedAt'], 1_000_000)
                self.assertEqual(rows[-2]['state']['uwb']['measurementUpdatedAt'], 1_000_000)
                self.assertTrue(rows[-1]['state']['uwb']['fresh'])
                self.assertEqual(rows[-1]['state']['uwb']['elevationDeg'], 25)

    def test_late_out_of_range_elevation_poisoning_cannot_be_revived(self):
        for completed in (False, True):
            with self.subTest(completed=completed):
                self.setUp()
                fields = self.pair(100) if completed else [self.value(100, 'DIST', 3), self.value(100, 'AZ', 0)]
                self.receive(fields)
                self.now += 20
                self.receive([self.value(100, 'EL', 100)] + self.pair(100))
                bad_index = len(self.requests) - 1
                self.now += 20
                self.receive(self.pair(140))
                rows = self.results()
                self.assertFalse(rows[bad_index]['state']['uwb']['fresh'])
                self.assertEqual(rows[bad_index]['state']['uwb']['sensors'][0]['filterReason'], 'elevation_out_of_range')
                self.assertTrue(rows[-1]['state']['uwb']['fresh'])

    def test_source_timestamp_wrap_is_not_a_duplicate(self):
        self.receive(self.pair(0xFFFFFFEC) + self.pair(0, 3.1))
        u = self.results()[-1]['state']['uwb']
        self.assertTrue(u['fresh'])
        self.assertGreater(u['distanceM'], 3)

    def test_actual_old_and_new_lua_senders_across_send_ticks(self):
        # Always-run wire tests below do not require Lua. When lupa is installed,
        # also execute the frozen old sender and current deployment source.
        sys.path.insert(0, str(ROOT / 'test'))
        from test_pixhawk_uwb_lua import HARNESS, LuaRuntime, frame, SCRIPT
        if LuaRuntime is None:
            self.skipTest('optional Lua integration needs lupa')
        for script in (ROOT / 'test/fixtures/uwb_aoa_legacy.lua', SCRIPT):
            with self.subTest(script=script.name):
                self.setUp()
                lua = LuaRuntime(unpack_returned_tuples=True, encoding=None)
                lua.execute(HARNESS)
                lua.execute(b'''function millis() return {
                    toint=function() return now_ms end, tofloat=function() return now_ms end} end
                function gcs:send_named_float(name, value)
                    messages[#messages+1] = {name=name, value=value, at=now_ms}
                    now_ms=now_ms+1
                end''')
                lua.globals().send_clock_step = 1
                callback, _ = lua.execute(script.read_bytes())
                lua.globals().callback = callback
                previous = 0
                indices = []
                for i in range(10):
                    lua.globals().feed(1, frame(i, 300, 0, 0))
                    for at in (100 + i * 40, 110 + i * 40):
                        lua.globals().run_tick(at)
                    messages = list(lua.globals().messages.values())[previous:]
                    previous = len(lua.globals().messages)
                    self.now = 1_000_000 + i * 40
                    self.receive([self.attitude()] + [
                        mav.MAVLink_named_value_float_message(int(m[b'at']), m[b'name'], m[b'value'])
                        for m in messages], split=True)
                    indices.append(len(self.requests) - 1)
                rows = self.results()
                for index in indices:
                    self.assertTrue(rows[index]['state']['uwb']['fresh'])
                    self.assertEqual(rows[index]['state']['uwb']['measurementUpdatedAt'],
                                     self.requests[index]['wall_ms'])

    def test_legacy_lua_cross_tick_fields_and_wrap_are_accepted(self):
        for stamp, validity, split in ((100, False, False), (100, True, True),
                                       (0xFFFFFFFE, True, True)):
            with self.subTest(stamp=stamp, validity=validity, split=split):
                self.setUp()
                self.sender.seq = 254  # Also cross the MAVLink packet-sequence wrap.
                self.receive(self.legacy_pair(stamp, validity=validity), split)
                u = self.results()[-1]['state']['uwb']
                self.assertTrue(u['fresh'])
                self.assertAlmostEqual(u['distanceM'], 3)

    def test_legacy_missing_packets_or_fields_never_mix_observations(self):
        for case in ('missing_el', 'missing_az_and_next_dist', 'only_next_az', 'seq_gap',
                     'wrong_order', 'wrong_source', 'long_source_span'):
            with self.subTest(case=case):
                self.setUp()
                self.receive([self.value(100, 'DIST', 3)])
                if case == 'missing_el':
                    self.receive([self.value(102, 'AZ', 0)])
                elif case == 'missing_az_and_next_dist':
                    self.receive([self.value(101, 'EL', 0),
                                  self.value(121, 'EL', 0), self.value(122, 'AZ', 0)])
                elif case == 'only_next_az':
                    self.receive([self.value(101, 'EL', 0), self.value(122, 'AZ', 0)])
                elif case == 'seq_gap':
                    self.sender.seq = (self.sender.seq + 1) % 256
                    self.receive([self.value(101, 'EL', 0), self.value(102, 'AZ', 0)])
                elif case == 'wrong_order':
                    self.receive([self.value(101, 'AZ', 0), self.value(102, 'EL', 0)])
                elif case == 'wrong_source':
                    self.sender.srcComponent = 2
                    self.receive([self.value(101, 'EL', 0), self.value(102, 'AZ', 0)])
                else:
                    self.receive([self.value(106, 'EL', 0), self.value(107, 'AZ', 0)])
                bad = len(self.requests) - 1
                self.now += 100
                component = self.sender.srcComponent
                self.sender.srcComponent = 1
                self.receive([self.pose(200, 0)])
                self.sender.srcComponent = component
                self.receive(self.legacy_pair(200))
                rows = self.results()
                self.assertIsNone(rows[bad]['state']['uwb']['measurementUpdatedAt'])
                self.assertTrue(rows[-1]['state']['uwb']['fresh'])

    def test_legacy_five_ms_boundary_and_packet_loss_preserve_near_stop(self):
        for span, gap in ((5, False), (6, False), (5, True)):
            with self.subTest(span=span, gap=gap):
                self.setUp()
                self.receive([self.value(100, 'DIST', .2), self.value(101, 'EL', 0)])
                if gap:
                    self.sender.seq = (self.sender.seq + 1) % 256
                self.receive([self.value(100 + span, 'AZ', 0)])
                u = self.results()[-1]['state']['uwb']
                self.assertEqual(u['fresh'], span == 5 and not gap)
                self.assertTrue(u['clearanceStop'])

    def test_lost_legacy_fields_across_adjacent_frames_never_cross_pair(self):
        # All incomplete subsets of two actual 20 ms legacy send periods,
        # retaining the real wire sequence gaps created by dropped packets.
        for first_mask, second_mask in itertools.product(range(1, 7), repeat=2):
            with self.subTest(first=first_mask, second=second_mask):
                self.setUp()
                self.sync_session(99)
                for stamp, mask in ((100, first_mask), (120, second_mask)):
                    for i, message in enumerate(self.legacy_pair(stamp, .2)):
                        if mask & (1 << i):
                            self.receive([message])
                        else:
                            self.sender.seq = (self.sender.seq + 1) % 256
                u = self.results()[-1]['state']['uwb']
                self.assertIsNone(u['measurementUpdatedAt'])
                self.assertEqual(u['clearanceStop'], bool((first_mask | second_mask) & 1))

    def test_strict_protocol_cannot_fall_back_to_legacy_pairing(self):
        self.receive([self.value(90, 'PROTO', 2), self.value(91, 'PROTO', 1)])
        self.receive(self.legacy_pair(100))  # Even 1 ms apart must remain distinct.
        bad = len(self.requests) - 1
        self.receive(self.pair(120))
        rows = self.results()
        self.assertIsNone(rows[bad]['state']['uwb']['measurementUpdatedAt'])
        self.assertTrue(rows[-1]['state']['uwb']['fresh'])

    def test_legacy_duplicate_and_poisoned_observation_cannot_revive(self):
        self.receive(self.legacy_pair(100))
        self.now += 900
        self.receive([self.attitude()] + self.legacy_pair(100))
        duplicate = len(self.requests) - 1
        self.receive([self.value(1000, 'DIST', 3), self.value(1001, 'EL', float('nan')),
                      self.value(1002, 'AZ', 0)])
        rows = self.results()
        self.assertFalse(rows[duplicate]['state']['uwb']['fresh'])
        self.assertEqual(rows[duplicate]['state']['uwb']['sensors'][0]['rawDistanceUpdatedAt'], 1_000_000)
        self.assertFalse(rows[-1]['state']['uwb']['fresh'])
        self.assertEqual(rows[-1]['state']['uwb']['sensors'][0]['measurementUpdatedAt'], 1_000_000)

    def reconnect(self, elapsed=20, heartbeat_only=False):
        self.now += elapsed
        self.needs_sync = True
        self.sync_heading = not heartbeat_only
        self.receive([mav.MAVLink_heartbeat_message(10, 3, 129, 0, 4, 3)]
                     + ([] if heartbeat_only else [self.attitude()]))

    def test_disconnect_clears_all_uwb_state_and_same_or_lower_clock_recovers(self):
        for stamp in (100, 1):
            for gated in (True, False):
                with self.subTest(stamp=stamp, gated=gated):
                    self.setUp()
                    self.config['uwb_measurement_filter']['enabled'] = gated
                    self.receive([self.value(90, 'PROTO', 2), self.value(90, 'TAG', 123)] + self.pair(100))
                    self.requests.append(dict(op='disconnect', wall_ms=self.now+1, mono_ms=self.now+1))
                    disconnected = len(self.requests) - 1
                    self.reconnect(20, heartbeat_only=True)
                    reconnected = len(self.requests) - 1
                    self.receive([self.attitude()] + self.legacy_pair(stamp, .2))
                    rows = self.results()
                    for index in (disconnected, reconnected):
                        u = rows[index]['state']['uwb']; s = u['sensors'][0]
                        self.assertFalse(u['fresh'])
                        for key in ('rawDistanceM', 'distanceM', 'measurementUpdatedAt', 'receivedMeasurementAt', 'tagId', 'trustDiagnostics'):
                            self.assertIsNone(s[key], (key, s))
                    self.assertTrue(any(e['type'] == 'uwb' and not e['payload']['fresh']
                                        for e in rows[disconnected]['events']))
                    u = rows[-1]['state']['uwb']
                    self.assertTrue(u['fresh'], u)
                    self.assertAlmostEqual(u['distanceM'], .2, places=6)

    def test_disconnect_drops_partial_and_quarantined_frames(self):
        for poisoned in (False, True):
            with self.subTest(poisoned=poisoned):
                self.setUp()
                self.receive([self.value(100, 'DIST', 3)] +
                             ([self.value(100, 'EL', float('nan'))] if poisoned else []))
                self.requests.append(dict(op='disconnect', wall_ms=self.now+1, mono_ms=self.now+1))
                self.reconnect()
                self.receive([self.value(100, 'EL', 0), self.value(100, 'AZ', 0)])
                incomplete = len(self.requests) - 1
                self.receive(self.pair(120, .2))
                rows = self.results()
                self.assertIsNone(rows[incomplete]['state']['uwb']['measurementUpdatedAt'])
                self.assertTrue(rows[-1]['state']['uwb']['fresh'])

    def test_heartbeat_timeout_also_resets_source_clock_high_water(self):
        self.receive(self.pair(10000))
        self.now += 4000
        self.requests.append(dict(op='tick', wall_ms=self.now, mono_ms=self.now))
        timed_out = len(self.requests) - 1
        self.reconnect()
        self.receive(self.pair(100, .2))
        rows = self.results()
        self.assertFalse(rows[timed_out]['connected'])
        self.assertIsNone(rows[timed_out]['state']['uwb']['sensors'][0]['rawDistanceM'])
        self.assertTrue(rows[-1]['state']['uwb']['fresh'])

    def test_new_session_requires_new_heading_and_discards_quarantine_history(self):
        for i in range(15):
            self.now = 1_000_000 + i * 100
            self.receive([self.attitude()] + self.pair(1000 + i * 100))
        self.receive(self.pair(2420, 8))
        quarantined = len(self.requests) - 1
        self.requests.append(dict(op='disconnect', wall_ms=self.now+1, mono_ms=self.now+1))
        self.reconnect(20, heartbeat_only=True)
        self.receive(self.pair(100, .2))
        no_heading = len(self.requests) - 1
        self.receive([self.attitude()] + self.pair(120, .2))
        rows = self.results()
        self.assertEqual(rows[quarantined]['state']['uwb']['measurementStatus'], 'suspect')
        self.assertFalse(rows[no_heading]['state']['uwb']['fresh'])
        self.assertEqual(rows[no_heading]['state']['uwb']['sensors'][0]['filterReason'], 'heading_unavailable')
        self.assertTrue(rows[-1]['state']['uwb']['fresh'])
        self.assertAlmostEqual(rows[-1]['state']['uwb']['distanceM'], .2, places=6)

    def test_receivers_cannot_supply_each_others_missing_fields(self):
        self.config['uwb_dual']['enabled_receiver_ids'] = [1, 2]
        self.receive([self.value(100, 'DIST', 3, 1), self.value(100, 'EL', 0, 1),
                      self.value(100, 'EL', 0, 2), self.value(100, 'AZ', 0, 2)])
        self.assertFalse(self.results()[-1]['state']['uwb']['fresh'])
        self.receive([self.value(100, 'AZ', 0, 1)] + self.pair(120, receiver=2))
        u = self.results()[-1]['state']['uwb']
        self.assertEqual(u['receiverCount'], 2)

    def recovery_observe(self, legacy, distance, angle=0):
        self.now += 25
        fields = self.legacy_pair(self.now, distance, validity=True) if legacy else [
            self.value(self.now, 'VALID', 1)] + self.pair(self.now, distance)
        fields[-1].value = angle
        self.receive([mav.MAVLink_heartbeat_message(10, 3, 129, 0, 4, 3), self.attitude()] + fields)
        return len(self.requests)-1

    def test_moving_track_recovers_after_jump_or_radio_gap_for_both_lua_protocols(self):
        for legacy in (False, True):
            for radio_gap in (False, True):
                with self.subTest(legacy=legacy, radio_gap=radio_gap):
                    self.setUp()
                    if not legacy:
                        self.receive([self.value(self.now, 'PROTO', 2)])
                    for _ in range(80):
                        self.recovery_observe(legacy, 3)
                    if radio_gap:
                        self.now += 25
                        self.receive([self.value(self.now, 'VALID', 0)])
                        self.now += 1000
                    indices = [self.recovery_observe(legacy, (3.5 if radio_gap else 5)+.5*i*.025)
                               for i in range(200)]
                    rows = self.results()
                    for i, index in enumerate(indices):
                        u = rows[index]['state']['uwb']
                        self.assertEqual(u['fresh'], i >= 60, (i, u))
                    recovered = rows[indices[60]]['state']['uwb']['sensors'][0]
                    self.assertEqual(recovered['filterReason'], 'stable_track_reacquired')
                    self.assertEqual(recovered['filterRecoveryCount'], 1)
                    self.assertEqual(rows[indices[-1]]['state']['uwb']['sensors'][0]['filterRecoveryCount'], 1)

    def test_recovery_rejects_persistent_jumps_even_with_a_moving_mean(self):
        for legacy in (False, True):
            for pattern in ('alternating', 'sawtooth', 'reversing'):
                with self.subTest(legacy=legacy, pattern=pattern):
                    self.setUp()
                    if not legacy:
                        self.receive([self.value(self.now, 'PROTO', 2)])
                    for _ in range(80):
                        self.recovery_observe(legacy, 3)
                    indices = []
                    for i in range(200):
                        offset = (1 if i % 2 else -1) if pattern == 'alternating' else (
                            (i % 20)*.08 if pattern == 'sawtooth' else abs((i % 40)-20)*.08)
                        indices.append(self.recovery_observe(legacy, 6+.0125*i+offset, 30))
                    rows = self.results()
                    self.assertTrue(all(not rows[j]['state']['uwb']['fresh'] for j in indices))
                    self.assertEqual(rows[indices[-1]]['state']['uwb']['sensors'][0]['filterRecoveryCount'], 0)

    def test_quarantined_new_track_requires_full_stability_window_then_stays_trusted(self):
        def observe(angle):
            self.now += 25
            self.receive([mav.MAVLink_heartbeat_message(10, 3, 129, 0, 4, 3),
                          self.attitude()] + self.pair(self.now, 3, angle))
            return len(self.requests) - 1

        for _ in range(80):
            observe(0)
        before = len(self.requests) - 1
        pending = [observe(45) for _ in range(60)]  # Only 1475 ms of new evidence.
        recovered = observe(45)  # First 1500 ms confirmation.
        stable = [observe(45) for _ in range(40)]
        rows = self.results()
        self.assertTrue(rows[before]['state']['uwb']['fresh'])
        for index in pending:
            self.assertFalse(rows[index]['state']['uwb']['fresh'])
        sensor = rows[recovered]['state']['uwb']['sensors'][0]
        self.assertTrue(rows[recovered]['state']['uwb']['fresh'])
        self.assertEqual(sensor['filterReason'], 'stable_track_reacquired')
        self.assertEqual(sensor['filterRecoveryCount'], 1)
        self.assertAlmostEqual(sensor['azimuthDeg'], 45)
        self.assertTrue(all(rows[i]['state']['uwb']['fresh'] for i in stable))


class SmoothingOnlyContractTests(unittest.TestCase):
    # Run the same real-wire invariants with statistical checks disabled.
    attitude = UwbWireContractTests.attitude
    source_now = UwbWireContractTests.source_now
    value = staticmethod(UwbWireContractTests.value)
    pair = UwbWireContractTests.pair
    legacy_pair = UwbWireContractTests.legacy_pair
    receive = UwbWireContractTests.receive
    sync_session = UwbWireContractTests.sync_session
    results = UwbWireContractTests.results
    pose = UwbWireContractTests.pose
    reconnect = UwbWireContractTests.reconnect
    check_batch_near_range = UwbWireContractTests.check_batch_near_range

    def setUp(self):
        UwbWireContractTests.setUp(self)
        self.config['uwb_measurement_filter']['trust_checks_enabled'] = False

    test_no_cross_frame_pairing = UwbWireContractTests.test_lost_fields_cannot_pair_across_frames_or_sources
    test_legacy_packet_loss = UwbWireContractTests.test_legacy_missing_packets_or_fields_never_mix_observations
    test_legacy_five_ms_boundary = UwbWireContractTests.test_legacy_five_ms_boundary_and_packet_loss_preserve_near_stop
    test_legacy_adjacent_frame_loss = UwbWireContractTests.test_lost_legacy_fields_across_adjacent_frames_never_cross_pair
    test_actual_lua_senders = UwbWireContractTests.test_actual_old_and_new_lua_senders_across_send_ticks
    test_strict_no_legacy_fallback = UwbWireContractTests.test_strict_protocol_cannot_fall_back_to_legacy_pairing
    test_near_then_far = UwbWireContractTests.test_near_then_far_in_one_read_keeps_stop_for_both_controllers
    test_reconnect_old_frames = UwbWireContractTests.test_reconnect_old_complete_frames_cannot_poison_reboot_clock
    test_reconnect_old_tails = UwbWireContractTests.test_reconnect_old_tail_cannot_pair_with_new_distance_at_same_stamp
    test_disconnect_session_reset = UwbWireContractTests.test_disconnect_clears_all_uwb_state_and_same_or_lower_clock_recovers
    test_duplicates = UwbWireContractTests.test_complete_duplicate_does_not_refresh_measurement_or_raw_range
    test_nonfinite = UwbWireContractTests.test_nan_or_infinity_clears_pending_fields_and_poisoned_frame_cannot_revive
    test_sample_time_alignment = UwbWireContractTests.test_batch_and_unbatched_moving_observations_have_same_filter_result

    def test_jumps_and_outside_fov_are_smoothed_without_quarantine_or_wait_timeout(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                self.setUp()
                indices = []
                for i in range(320):  # Longer than the five-second wait timeout.
                    self.now += 25
                    distance = 3.5 if i < 40 else (3.0 if i % 2 else 6.0)
                    angle = 22 if i < 40 else (85 if i % 2 else -85)
                    fields = self.legacy_pair(self.now, distance) if legacy else self.pair(self.now, distance, angle)
                    fields[-1].value = angle
                    self.receive([mav.MAVLink_heartbeat_message(10, 3, 129, 0, 4, 3), self.attitude()] + fields)
                    indices.append((len(self.requests)-1, self.now))
                rows = self.results()
                config = copy.deepcopy(self.config['uwb_follow'])
                config['relative_nmpc']['enabled'] = False
                controls = [dict(op='reset', config=config)]
                shadow_suspect = False
                for index, now in indices:
                    t = rows[index]['state']
                    u, sensor = t['uwb'], t['uwb']['sensors'][0]
                    self.assertTrue(u['fresh'], u)
                    self.assertFalse(u['trustChecksEnabled'])
                    self.assertEqual(sensor['rejectedCount'], 0)
                    self.assertEqual(sensor['occlusionDiagnostics']['state'], 'disabled')
                    shadow_suspect |= sensor['trustDiagnostics']['status'] == 'suspect'
                    self.assertFalse(sensor['trustDiagnostics']['controlGating'])
                    controls.append(dict(op='intermittent', now=now, telemetry=t,
                                         connected=True, controlMode='relative_manual'))
                self.assertTrue(shadow_suspect)
                records = [e['payload'] for row in rows for e in row['events'] if e['type'] == 'uwb_measurement']
                self.assertTrue(any(r.get('trustDiagnostics', {}).get('status') == 'suspect'
                                    and r['status'] == 'accepted' for r in records))
                for result in replay(FOLLOW, controls)[1:]:
                    self.assertTrue(result['safe'], result)
                    self.assertFalse(result.get('pausePropulsion', False), result)
                    self.assertNotEqual(result['reason'], 'prediction_wait_timeout')

    def test_normal_braking_and_turning_obey_deceleration_but_near_stop_is_immediate(self):
        for trigger in ('range', 'angle'):
            with self.subTest(trigger=trigger):
                config = copy.deepcopy(self.config['uwb_follow'])
                config.update(bearing_sign=1, bearing_scale=1, bearing_offset_deg=0,
                              bearing_filter_alpha=1)
                config['relative_nmpc']['enabled'] = False
                config['gps_speed_closed_loop']['enabled'] = False
                requests = [dict(op='reset', config=config)]
                for i in range(65):
                    now = 1000 + i * 100
                    distance = .2 if i == 64 else (2 if trigger == 'range' and i >= 60 else 6)
                    angle = 90 if trigger == 'angle' and i >= 60 else 0
                    requests.append(dict(op='relative', now=now,
                        telemetry=dict(uwb=dict(measurementSampleAt=now)),
                        preflight=dict(distanceM=distance, azimuthDeg=angle, measuredYawRateDegS=0),
                        policy=dict(nominalThrottlePercent=60, headingErrorDeg=0, yawRateDegS=0)))
                rows = replay(FOLLOW, requests)[1:]
                self.assertGreater(rows[59]['throttlePercent'], 20)
                for i in range(60, 64):
                    self.assertGreater(rows[i]['throttlePercent'], 0)
                    self.assertLessEqual(rows[i-1]['throttlePercent'] - rows[i]['throttlePercent'],
                                         rows[i]['throttleDecelLimitPercentS'] * .1 + 1e-8)
                self.assertEqual(rows[64]['throttlePercent'], 0)

    def test_both_turn_directions_keep_low_thrust_and_slew_without_stalling(self):
        config = copy.deepcopy(self.config['uwb_follow'])
        config.update(bearing_sign=1, bearing_scale=1, bearing_offset_deg=0, bearing_filter_alpha=1)
        config['relative_nmpc']['enabled'] = False
        config['gps_speed_closed_loop']['enabled'] = False
        paths = []
        for sign in (-1, 1):
            requests = [dict(op='reset', config=config)]
            for i in range(120):
                now = 1000 + i*100
                # Turn, straighten, then reverse turn direction without a stop.
                angle = 0 if i < 50 or 85 <= i < 95 else sign * (80 if i < 85 else -80)
                requests.append(dict(op='relative', now=now,
                    telemetry=dict(uwb=dict(measurementSampleAt=now)),
                    preflight=dict(distanceM=6, azimuthDeg=angle, measuredYawRateDegS=0)))
            rows = replay(FOLLOW, requests)[1:]
            self.assertGreater(rows[50]['throttlePercent'], 0)
            self.assertGreater(sign * rows[50]['steeringDegrees'], 0)
            self.assertAlmostEqual(rows[84]['throttlePercent'], config['relative_turn_min_throttle_percent'])
            self.assertLess(sign * rows[-1]['steeringDegrees'], 0)
            for before, after in zip(rows, rows[1:]):
                delta = after['throttlePercent'] - before['throttlePercent']
                self.assertLessEqual(delta, config['relative_throttle_accel_percent_s']*.1+1e-8)
                self.assertGreaterEqual(delta, -after['throttleDecelLimitPercentS']*.1-1e-8)
                self.assertLessEqual(abs(after['steeringDegrees']-before['steeringDegrees']),
                                     config['steering_slew_deg_s']*.1+1e-8)
            paths.append(rows)
        for left, right in zip(*paths):
            self.assertAlmostEqual(left['throttlePercent'], right['throttlePercent'])
            self.assertAlmostEqual(left['steeringDegrees'], -right['steeringDegrees'])

    def test_saturation_reserves_turn_difference_instead_of_clipping_a_motor(self):
        for sign in (-1, 1):
            result = replay(FOLLOW, [dict(op='allocate_forward', throttle=100, steering=sign*18)])[0]
            self.assertEqual(result['throttle'], 60)
            self.assertEqual(result['steering'], sign*18)
            self.assertEqual(result['leftDemandPercent']-result['rightDemandPercent'], sign*80)
            self.assertTrue(result['throttleReducedForTurn'])
        requests = [dict(op='allocate_forward', throttle=t, steering=s, motorLimit=limit)
                    for limit in (30, 70, 100) for t in (0, 5, 20, 60, 100) for s in (-45, -18, 0, 18, 45)]
        for request, result in zip(requests, replay(FOLLOW, requests)):
            left, right = result['leftDemandPercent'], result['rightDemandPercent']
            self.assertGreaterEqual(min(left, right), -1e-8)
            self.assertLessEqual(max(left, right), request['motorLimit']+1e-8)
            self.assertGreaterEqual((left-right)*request['steering'], -1e-8)
            if request['throttle'] == 0:
                self.assertEqual(left+right, 0)

    def test_full_throttle_turn_reversal_and_range_jump_keep_allocated_outputs_continuous(self):
        config = copy.deepcopy(self.config['uwb_follow'])
        config.update(relative_throttle_limit_percent=100, bearing_sign=1, bearing_scale=1,
                      bearing_offset_deg=0, bearing_filter_alpha=1)
        config['relative_nmpc']['enabled'] = False
        config['gps_speed_closed_loop']['enabled'] = False
        requests = [dict(op='reset', config=config)]
        for i in range(190):
            now = 1000+i*100
            yaw = 0 if i < 70 or i >= 150 else (14 if i < 110 else -14)
            requests.append(dict(op='relative_allocated', now=now,
                telemetry=dict(uwb=dict(measurementSampleAt=now)),
                preflight=dict(distanceM=15 if 100 <= i < 110 else 6, azimuthDeg=0, measuredYawRateDegS=0),
                policy=dict(nominalThrottlePercent=100, headingErrorDeg=0, yawRateDegS=yaw)))
        rows = replay(FOLLOW, requests)[1:]
        self.assertAlmostEqual(rows[69]['driveAllocation']['throttle'], 100)
        self.assertGreater(rows[70]['driveAllocation']['steering'], 0)  # Respond in first 100 ms step.
        self.assertEqual(rows[99]['driveAllocation']['leftDemandPercent'], 100)
        self.assertAlmostEqual(rows[99]['driveAllocation']['rightDemandPercent'], 20)
        self.assertLess(rows[149]['driveAllocation']['steering'], 0)
        self.assertGreater(rows[150]['driveAllocation']['throttle'], 0)
        self.assertLess(rows[150]['driveAllocation']['throttle'], 100)  # No rebound from hidden saturation.
        for before, after in zip(rows, rows[1:]):
            a, b = before['driveAllocation'], after['driveAllocation']
            self.assertLessEqual(b['throttle']-a['throttle'], 2+1e-8)
            self.assertGreaterEqual(b['throttle']-a['throttle'], -4-1e-8)
            for field in ('leftDemandPercent', 'rightDemandPercent'):
                self.assertGreaterEqual(b[field], 0)
                self.assertLessEqual(b[field], 100+1e-8)
                self.assertLessEqual(abs(b[field]-a[field]), 8+1e-8)

    def test_imu_yaw_feedback_reduces_turn_command_and_opposes_overshoot(self):
        config = copy.deepcopy(self.config['uwb_follow'])
        config.update(bearing_sign=1, bearing_scale=1, bearing_offset_deg=0, bearing_filter_alpha=1)
        config['relative_nmpc']['enabled'] = False
        config['gps_speed_closed_loop']['enabled'] = False
        outputs = []
        for measured in (0, 5, 25):
            requests = [dict(op='reset', config=config)]
            for i in range(30):
                now = 1000+i*100
                requests.append(dict(op='relative', now=now,
                    telemetry=dict(uwb=dict(measurementSampleAt=now)),
                    preflight=dict(distanceM=6, azimuthDeg=10, measuredYawRateDegS=measured)))
            result = replay(FOLLOW, requests)[-1]
            self.assertEqual(result['measuredYawRateDegS'], measured)
            outputs.append(result['steeringDegrees'])
        self.assertGreater(outputs[0], outputs[1])
        self.assertGreater(outputs[1], outputs[2])
        self.assertLess(outputs[2], 0)


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]], verbosity=2)
