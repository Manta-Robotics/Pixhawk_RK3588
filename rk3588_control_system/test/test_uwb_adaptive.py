import math
import pathlib
import random
import statistics
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'backend'))
from uwb_angular_filter import AdaptiveAngularFilter, wrap
from uwb_measurement_filter import GatedUwbKalman


class AdaptiveBearingTests(unittest.TestCase):
    def test_stationary_noise_reduction_without_dropping_observations(self):
        rng, f = random.Random(49), AdaptiveAngularFilter()
        raw, smooth = [], []
        for i in range(1200):
            angle = 20 + rng.gauss(0, 2)
            value = f.observe(angle, angle, 1000 + i * 25, 2)
            if i > 80:
                raw.append(angle); smooth.append(value)
            self.assertEqual(f.at, 1000 + i * 25)
        self.assertLess(statistics.pstdev(smooth), statistics.pstdev(raw) * .5)
        self.assertLess(abs(statistics.mean(smooth) - 20), .5)

    def test_motion_releases_stationary_filter_and_bounds_extra_lag(self):
        f = AdaptiveAngularFilter()
        for at in range(1000, 3000, 25):
            f.observe(0, 0, at, 2)
        self.assertEqual(f.mode, 'stationary')
        released_at, errors = None, []
        for i in range(81):
            angle = i * .75  # 30 degrees/s
            value = f.observe(angle, angle, 3000 + i * 25, 2)
            if f.mode == 'moving' and released_at is None:
                released_at = i * 25
            if i >= 20:
                errors.append(abs(wrap(value - angle)))
        self.assertLessEqual(released_at, 250)
        self.assertLess(max(errors), 3)

    def test_wrap_and_gap_do_not_interpolate_through_wrong_hemisphere(self):
        f = AdaptiveAngularFilter()
        f.observe(179, 179, 1000, 2)
        result = f.observe(-179, -179, 1025, 2)
        self.assertGreater(abs(result), 170)
        self.assertEqual(f.observe(30, 30, 1400, 2), 30)
        self.assertEqual(f.mode, 'acquiring')

    def test_gate_covariance_range_and_clocks_identical_with_adaptive_on_or_off(self):
        filters = [GatedUwbKalman({'adaptive_enabled': enabled}) for enabled in (False, True)]
        rng = random.Random(73)
        for i in range(500):
            at = 1000 + i * 25
            angle = 10 * math.sin(i / 100) + rng.gauss(0, 1)
            distance = 15 if i in (100, 300) else 3
            results = [f.update(distance, angle, at, 15, sequence_id=i, tag_id='a') for f in filters]
            for key in ('accepted', 'accepted_at', 'accepted_count', 'rejected_count', 'reason', 'nis', 'distance_m'):
                self.assertEqual(results[0][key], results[1][key], key)
            self.assertEqual([(a.position, a.velocity, a.p00, a.p01, a.p11) for a in filters[0].axes],
                             [(a.position, a.velocity, a.p00, a.p01, a.p11) for a in filters[1].axes])
            result = results[1]
            self.assertAlmostEqual(result['uncertainty_m'], results[0]['uncertainty_m'] + result['smoothing_displacement_m'])
            offset = math.radians(wrap(result['azimuth_deg'] - result['unsmoothed_azimuth_deg']))
            self.assertAlmostEqual(result['smoothing_displacement_m'], 2 * result['distance_m'] * abs(math.sin(offset / 2)))

    def test_rejected_or_duplicate_frames_cannot_advance_smoother(self):
        f = GatedUwbKalman()
        f.update(3, 0, 1000, 0, sequence_id=1)
        snapshot = (f.angular_filter.at, f.angular_filter.angle, list(f.angular_filter.points))
        for result in (f.update(3, 20, 1025, 0, valid=False),
                       f.update(3, 20, 1050, 0, sequence_id=1),
                       f.reject(1075, 'outside_receiver_fov', 0)):
            self.assertFalse(result['accepted'])
            self.assertEqual(result['accepted_at'], 1000)
            self.assertEqual((f.angular_filter.at, f.angular_filter.angle, f.angular_filter.points), snapshot)

    def test_world_frame_avoids_lagging_vessel_rotation(self):
        f = GatedUwbKalman()
        for i in range(200):
            heading = i * .2
            result = f.update(3, 20 - heading, 1000 + i * 25, heading, sequence_id=i)
            self.assertTrue(result['accepted'])
            self.assertAlmostEqual(result['azimuth_deg'], 20 - heading, places=6)

    def test_identity_and_clock_reset_clear_smoothing_history(self):
        for cause in ('tag', 'clock'):
            f = GatedUwbKalman()
            f.update(3, 0, 1000, 0, tag_id='a')
            result = f.update(3, 10, 1025 if cause == 'tag' else 900, 0, tag_id='b' if cause == 'tag' else 'a')
            self.assertFalse(result['accepted'])
            self.assertIsNone(f.angular_filter.angle)
            result = f.update(3, 10, 1050, 0, tag_id='b' if cause == 'tag' else 'a')
            self.assertTrue(result['accepted'])
            self.assertAlmostEqual(result['azimuth_deg'], 10)


if __name__ == '__main__':
    unittest.main()
