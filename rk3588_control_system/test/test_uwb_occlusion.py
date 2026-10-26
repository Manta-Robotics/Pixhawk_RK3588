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

    def test_coherent_wrong_cluster_cannot_use_raw_recovery_even_after_covariance_grows(self):
        old, guarded = self.warm(occlusion_enabled=False), self.warm()
        admitted_old = 0
        clock = guarded.last_accepted_at
        for at in range(3000, 13000, 25):
            admitted_old += old.update(1, 45, at, 0, sequence_id=at)['accepted']
            result = guarded.update(1, 45, at, 0, sequence_id=at)
            self.assertFalse(result['accepted'])
            self.assertEqual(result['accepted_at'], clock)
            self.assertEqual(guarded.raw_window, [])
        self.assertGreater(admitted_old, 300)  # Reproduces the original failure.
        self.assertEqual(result['occlusion_diagnostics']['state'], 'quarantined')
        self.assertEqual(result['recovery_count'], 0)

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

    def test_nearby_angular_jump_cannot_recover_using_distance_corridor_alone(self):
        f = GatedUwbKalman()
        for at in range(1000, 3000, 25):
            f.update(.5, 0, at, 0)
        # 30 degrees at 0.5 m is only 0.26 m displacement. It lies inside
        # the positional return corridor but is not a return in bearing.
        for at in range(3000, 6000, 25):
            self.assertFalse(f.update(.5, 30, at, 0)['accepted'])
        self.assertEqual(f.last_accepted_at, 2975)

    def test_explicit_radio_loss_cannot_hide_jump_behind_long_receive_gap(self):
        f = self.warm()
        f.reject(3000, 'radio_invalid', 0)
        for at in range(5000, 8000, 25):
            self.assertFalse(f.update(1, 45, at, 0)['accepted'])
        self.assertEqual(f.last_accepted_at, 2975)
        self.assertEqual(f.occlusion_guard.diagnostics['state'], 'quarantined')


if __name__ == '__main__':
    unittest.main()
