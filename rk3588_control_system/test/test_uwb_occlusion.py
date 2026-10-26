import math
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'backend'))
from uwb_measurement_filter import GatedUwbKalman
from uwb_fusion import DualUwbFusion


class OcclusionTests(unittest.TestCase):
    def warm(self, **config):
        f = GatedUwbKalman(config)
        for at in range(1000, 3000, 25):
            f.update(1, 0, at, 0, sequence_id=at)
        return f

    def test_displaced_cluster_stays_quarantined_until_long_confirmation(self):
        old, guarded = self.warm(occlusion_enabled=False), self.warm()
        admitted_old = 0
        clock = guarded.last_accepted_at
        for at in range(3000, 4500, 25):
            admitted_old += old.update(1, 45, at, 0, sequence_id=at)['accepted']
            result = guarded.update(1, 45, at, 0, sequence_id=at)
            self.assertFalse(result['accepted'])
            self.assertEqual(result['accepted_at'], clock)
            self.assertEqual(guarded.raw_window, [])
        self.assertGreater(admitted_old, 30)  # The ordinary short gate admits too early.
        self.assertEqual(result['occlusion_diagnostics']['state'], 'quarantined')
        self.assertEqual(result['recovery_count'], 0)
        result = guarded.update(1, 45, 4500, 0, sequence_id=4500)
        self.assertTrue(result['accepted'])
        self.assertEqual(result['reason'], 'stable_track_reacquired')
        self.assertEqual(result['recovery_count'], 1)
        self.assertAlmostEqual(result['distance_m'], 1)
        self.assertAlmostEqual(result['azimuth_deg'], 45)
        self.assertTrue(all(abs(axis.velocity) < 1e-10 for axis in guarded.axes))

    def test_burst_return_requires_250_ms_fresh_evidence_then_keeps_40_hz(self):
        f = self.warm()
        for at in range(3000, 4000, 25):
            self.assertFalse(f.update(1, 50 if at % 50 else -50, at, 0)['accepted'])
        clock = f.last_accepted_at
        for at in range(4000, 4250, 25):
            result = f.update(1, 0, at, 0)
            self.assertFalse(result['accepted'])
            self.assertEqual(result['accepted_at'], clock)
        for at in range(4250, 5250, 25):
            self.assertTrue(f.update(1, 0, at, 0)['accepted'])
        self.assertEqual(f.last_accepted_at, 5225)

    def test_isolated_spike_returns_immediately_without_smoothing_or_clock_poisoning(self):
        f = self.warm()
        smooth_at = f.angular_filter.at
        self.assertFalse(f.update(1, 50, 3000, 0)['accepted'])
        self.assertEqual(f.angular_filter.at, smooth_at)
        self.assertTrue(f.update(1, 0, 3025, 0)['accepted'])
        self.assertAlmostEqual(f.result(3025, 0)['azimuth_deg'], 0)

    def test_bad_radio_interrupts_confirmation_and_duplicate_packets_add_no_hits(self):
        f = self.warm()
        for at in (3000, 3025):
            f.update(1, 50, at, 0)
        for at in range(3050, 3251, 25):
            f.update(1, 0, at, 0, sequence_id=at)
        before = f.occlusion_guard.return_hits
        for _ in range(40):
            f.update(1, 0, 3250, 0, sequence_id=3250)
        self.assertEqual(f.occlusion_guard.return_hits, before)
        f.update(1, 0, 3275, 0, valid=False)
        self.assertFalse(f.update(1, 0, 3300, 0)['accepted'])
        self.assertEqual(f.occlusion_guard.return_hits, 1)
        for at in range(3325, 3550, 25):
            self.assertFalse(f.update(1, 0, at, 0)['accepted'])
        self.assertTrue(f.update(1, 0, 3550, 0)['accepted'])

    def test_40_hz_linear_motion_and_boat_rotation_are_not_occlusion(self):
        f = GatedUwbKalman()
        for i in range(400):
            x, y, heading = 3 + .05 * i, .03 * i, i * .2
            angle = math.degrees(math.atan2(y, x)) - heading
            result = f.update(math.hypot(x, y), angle, 1000 + i * 25, heading)
            # Stay inside the configured 20 m range for this test.
            if math.hypot(x, y) <= 20:
                self.assertTrue(result['accepted'])
                self.assertEqual(result['occlusion_diagnostics']['state'], 'clear')

    def test_fusion_keeps_raw_near_range_veto_but_not_suspect_position(self):
        f = self.warm()
        fusion = DualUwbFusion(enabled_receiver_ids=(1,), require_gated_measurements=True)
        for at, distance, angle in [(3000, 1, 0), (3025, .2, 55), (3050, .2, 55)]:
            fusion.update('UWB1_DIST', distance, now_ms=at)
            fusion.update('UWB1_AZ', angle, now_ms=at)
            fusion.apply_measurement_result(1, f.update(distance, angle, at, 0))
        sensor = fusion.fused(3050)['sensors'][0]
        self.assertFalse(sensor['fresh'])
        self.assertEqual(sensor['rawDistanceM'], .2)
        self.assertEqual(sensor['measurementUpdatedAt'], 3000)
        self.assertEqual(sensor['receivedMeasurementAt'], 3050)
        self.assertEqual(sensor['occlusionDiagnostics']['state'], 'quarantined')

    def test_identity_and_clock_reset_clear_quarantine(self):
        for cause in ('tag', 'clock'):
            f = self.warm()
            f.tag_id = 1
            for at in (3000, 3025):
                f.update(1, 50, at, 0, tag_id=1)
            result = f.update(1, 20, 3050 if cause == 'tag' else 900, 0,
                              tag_id=2 if cause == 'tag' else 1)
            self.assertFalse(result['accepted'])
            self.assertEqual(result['occlusion_diagnostics']['state'], 'clear')
            self.assertTrue(f.update(1, 20, 3100, 0, tag_id=2 if cause == 'tag' else 1)['accepted'])

    def test_configuration_rejects_non_boolean_enable(self):
        with self.assertRaises(ValueError):
            GatedUwbKalman({'occlusion_enabled': 'true'})

    def test_nearby_angular_jump_requires_long_confirmation_despite_short_distance(self):
        f = GatedUwbKalman()
        for at in range(1000, 3000, 25):
            f.update(.5, 0, at, 0)
        # 30 degrees at 0.5 m is only 0.26 m displacement. It lies inside
        # the positional return corridor but is not a return in bearing.
        for at in range(3000, 4500, 25):
            self.assertFalse(f.update(.5, 30, at, 0)['accepted'])
        self.assertEqual(f.last_accepted_at, 2975)
        self.assertTrue(f.update(.5, 30, 4500, 0)['accepted'])

    def test_explicit_radio_loss_cannot_hide_jump_behind_long_receive_gap(self):
        f = self.warm()
        f.reject(3000, 'radio_invalid', 0)
        for at in range(5000, 6500, 25):
            self.assertFalse(f.update(1, 45, at, 0)['accepted'])
        self.assertEqual(f.last_accepted_at, 2975)
        self.assertEqual(f.occlusion_guard.diagnostics['state'], 'quarantined')
        self.assertTrue(f.update(1, 45, 6500, 0)['accepted'])

    def test_unstable_bursts_cannot_reanchor_even_after_long_rejection(self):
        f = self.warm()
        for at in range(3000, 23000, 25):
            result = f.update(1, 45 if at % 1000 < 500 else -45, at, 0, sequence_id=at)
            self.assertFalse(result['accepted'])
        self.assertEqual(f.last_accepted_at, 2975)
        self.assertEqual(result['recovery_count'], 0)

    def test_new_track_confirmation_restarts_on_invalid_gap_and_duplicate_packets(self):
        for interrupt in ('radio_invalid', 'heading_unavailable', 'outside_receiver_fov',
                          'distance_out_of_range', 'gap'):
            with self.subTest(interrupt=interrupt):
                f = self.warm()
                for at in range(3000, 4001, 25):
                    self.assertFalse(f.update(.75, -20, at, 0, sequence_id=at)['accepted'])
                before = len(f.occlusion_guard.reacquisition)
                for _ in range(100):
                    self.assertFalse(f.update(.75, -20, 4000, 0, sequence_id=4000)['accepted'])
                self.assertEqual(len(f.occlusion_guard.reacquisition), before)
                if interrupt != 'gap':
                    f.reject(4025, interrupt, 0)
                for at in range(4250, 5750, 25):
                    self.assertFalse(f.update(.75, -20, at, 0, sequence_id=at)['accepted'])
                result = f.update(.75, -20, 5750, 0, sequence_id=5750)
                self.assertTrue(result['accepted'])
                self.assertEqual(result['reason'], 'stable_track_reacquired')

    def test_new_track_rebuilds_drifted_posterior_and_retains_real_covariance(self):
        f = self.warm()
        for at in range(3000, 5000, 25):
            f.update(1, 45 if at % 50 else -45, at, 0)
        for axis in f.axes:
            axis.position, axis.velocity, axis.p00 = 100, 4, 1e6
        for index, at in enumerate(range(5000, 6500, 25)):
            distance = .73 + .02 * math.sin(index)
            self.assertFalse(f.update(distance, -20 + .5 * math.sin(index), at, 0)['accepted'])
        result = f.update(.73, -20, 6500, 0)
        self.assertTrue(result['accepted'])
        self.assertAlmostEqual(result['distance_m'], .73)
        self.assertAlmostEqual(result['azimuth_deg'], -20)
        self.assertGreater(result['uncertainty_m'], 0)
        self.assertLess(result['uncertainty_m'], .5)
        self.assertLess(math.hypot(*(axis.velocity for axis in f.axes)), .1)
        # Reanchoring does not authorize the next discontinuity.
        self.assertFalse(f.update(4, 60, 6525, 0)['accepted'])
        self.assertEqual(f.last_accepted_at, 6500)

    def test_continuous_motion_recovers_with_velocity_after_displacement_or_radio_gap(self):
        for speed in (.17, .5, 1., 2.):
            for radio_gap in (False, True):
                with self.subTest(speed=speed, radio_gap=radio_gap):
                    f = self.warm()
                    start = 4000 if radio_gap else 3000
                    if radio_gap:
                        f.reject(3000, 'radio_invalid', 0)
                    for at in range(start, start+5000, 25):
                        result = f.update(3+speed*(at-start)/1000, 20, at, 0, sequence_id=at)
                        self.assertEqual(result['accepted'], at >= start+1500)
                        if at == start+1500:
                            self.assertEqual(result['reason'], 'stable_track_reacquired')
                            self.assertAlmostEqual(math.hypot(*(a.velocity for a in f.axes)), speed)
                    self.assertEqual(f.recovery_count, 1)

    def test_repeated_jumps_and_reversals_never_form_a_recovery_track(self):
        for pattern in ('alternating', 'sawtooth', 'reversing'):
            with self.subTest(pattern=pattern):
                f = self.warm()
                for i, at in enumerate(range(3000, 8000, 25)):
                    offset = (1 if i % 2 else -1) if pattern == 'alternating' else (
                        (i % 20)*.08 if pattern == 'sawtooth' else abs((i % 40)-20)*.08)
                    self.assertFalse(f.update(4+offset, 30, at, 0, sequence_id=at)['accepted'])
                self.assertEqual(f.recovery_count, 0)

    def test_gentle_acceleration_turning_and_measurement_noise_can_recover(self):
        f = self.warm()
        for i, at in enumerate(range(3000, 8000, 25)):
            t = (at-3000)/1000
            x, y = 4+.5*t+.1*t*t, .2*math.sin(t)
            heading = 6*t
            r = f.update(math.hypot(x, y)+.01*math.sin(i),
                         math.degrees(math.atan2(y, x))-heading+.1*math.cos(i),
                         at, heading, sequence_id=at)
            self.assertEqual(r['accepted'], at >= 4500)

    def test_motion_above_speed_limit_cannot_confirm(self):
        f = self.warm(max_relative_speed_mps=.4)
        for at in range(3000, 8000, 25):
            self.assertFalse(f.update(4+.8*(at-3000)/1000, 20, at, 0, sequence_id=at)['accepted'])

    def test_between_retained_samples_spikes_interrupt_motion_confirmation(self):
        f = self.warm()
        for at in range(3000, 8000, 5):
            distance = 4+.5*(at-3000)/1000
            if at % 100 == 5:
                distance += 1
            self.assertFalse(f.update(distance, 20, at, 0, sequence_id=at)['accepted'])

    def test_motion_confirmation_restarts_after_a_gap(self):
        f = self.warm()
        for at in list(range(3000, 4001, 25))+list(range(4250, 6000, 25)):
            r = f.update(3+.5*(at-3000)/1000, 20, at, 0, sequence_id=at)
            self.assertEqual(r['accepted'], at >= 5750)

    def test_batched_distinct_samples_do_not_reset_or_accelerate_confirmation(self):
        f = self.warm()
        for at in range(3000, 4500, 25):
            for offset in range(3):
                result = f.update(.75, -20, at, 0, sequence_id=at * 3 + offset)
                self.assertFalse(result['accepted'])
        self.assertTrue(f.update(.75, -20, 4500, 0, sequence_id=13500)['accepted'])
        self.assertEqual(f.recovery_count, 1)


if __name__ == '__main__':
    unittest.main()
