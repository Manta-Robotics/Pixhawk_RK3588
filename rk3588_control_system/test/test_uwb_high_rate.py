import math
import pathlib
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'backend'))
from uwb_measurement_filter import Axis, GatedUwbKalman
from uwb_fusion import DualUwbFusion
from backend.mavlink_bridge import MAVLinkBridge, FlightData, UWBData


class HighRateUwbTests(unittest.TestCase):
    def test_missing_observation_is_not_mislabeled_as_outside_antenna_fov(self):
        fusion = DualUwbFusion(enabled_receiver_ids=(1,), require_gated_measurements=True)
        sensor = fusion.fused(1000)['sensors'][0]
        self.assertIsNone(sensor['withinFov'])
        self.assertEqual(sensor['fusionExclusionReason'], 'no_complete_measurement')

    def test_prediction_covariance_does_not_depend_on_poll_partition(self):
        whole, split = Axis(3, .02, .3), Axis(3, .02, .3)
        whole.predict(.1, 4)
        for _ in range(4):
            split.predict(.025, 4)
        for key in ('position', 'velocity', 'p00', 'p01', 'p11'):
            self.assertAlmostEqual(getattr(whole, key), getattr(split, key), places=12)

    def test_consensus_recovers_at_10_40_and_50_hz_but_not_before_180_ms(self):
        for period in (100, 25, 20):
            with self.subTest(period=period):
                f = GatedUwbKalman()
                for at in range(1000, 2001, period):
                    f.update(3, 0, at, 0, sequence_id=at)
                # An obsolete, over-confident posterior must not permanently
                # lock out a genuine consistent raw track at a higher rate.
                for axis in f.axes:
                    axis.position += 4
                    axis.p00, axis.p01, axis.p11 = .001, 0, .001
                recovered_at = None
                for at in range(3000, 3501, period):
                    result = f.update(3, 0, at, 0, sequence_id=at)
                    if at - 3000 < 180:
                        self.assertFalse(result['accepted'])
                    if result['reason'] == 'raw_window_reacquired':
                        recovered_at = at
                        break
                self.assertIsNotNone(recovered_at)
                self.assertLessEqual(recovered_at - 3000, 400)
                self.assertLessEqual(len(f.raw_window), 5)

    def test_40_hz_continuous_valid_motion_is_accepted_and_isolated_spike_is_not(self):
        f = GatedUwbKalman()
        accepted = 0
        for index in range(400):
            distance = 3 + .15 * math.sin(index * .025)
            angle = 10 * math.sin(index * .025)
            result = f.update(distance, angle, 1000 + index * 25, 0, sequence_id=index)
            accepted += bool(result['accepted'])
        self.assertEqual(accepted, 400)
        before = f.last_accepted_at
        spike = f.update(15, -60, 11000, 0, sequence_id=400)
        self.assertFalse(spike['accepted'])
        self.assertEqual(f.last_accepted_at, before)
        self.assertGreater(spike['nis'], f.config['nis_threshold'])

    def test_fov_uses_current_observation_not_posterior_overshoot(self):
        fusion = DualUwbFusion(enabled_receiver_ids=(1,), require_gated_measurements=True)
        fusion.update('UWB1_DIST', 3, now_ms=1000)
        fusion.update('UWB1_AZ', 69, now_ms=1000)
        fusion.apply_measurement_result(1, {
            'accepted': True, 'status': 'trusted', 'reason': 'kalman_updated',
            'accepted_at': 1000, 'distance_m': 3, 'azimuth_deg': 71,
        })
        result = fusion.fused(1000)
        self.assertTrue(result['fresh'])
        self.assertTrue(result['sensors'][0]['withinFov'])
        fusion.update('UWB1_AZ', 71, now_ms=1025)
        result = fusion.fused(1025)
        self.assertFalse(result['fresh'])
        self.assertEqual(result['sensors'][0]['fusionExclusionReason'], 'outside_receiver_fov')
        self.assertEqual(result['sensors'][0]['measurementUpdatedAt'], 1000)

    def test_out_of_fov_frame_is_rejected_before_filter_correction_or_recovery(self):
        bridge = MAVLinkBridge.__new__(MAVLinkBridge)
        bridge.flight_data, bridge.uwb_data = FlightData(), UWBData()
        bridge.uwb_filter_enabled = True
        bridge.uwb_fusion = DualUwbFusion(enabled_receiver_ids=(1,), require_gated_measurements=True)
        bridge._uwb_measurement_filters = {1: GatedUwbKalman(), 2: GatedUwbKalman()}
        def pair(at, angle):
            bridge.flight_data.attitude_updated_at = at
            with patch('backend.mavlink_bridge.time.time', return_value=at / 1000):
                bridge._update_uwb_named_value('UWB1_DIST', 3, sample_time_boot_ms=at)
                bridge._update_uwb_named_value('UWB1_AZ', angle, sample_time_boot_ms=at)
        pair(1000, 0)
        f = bridge._uwb_measurement_filters[1]
        history = list(f.raw_window)
        for at in range(1025, 1326, 25):
            pair(at, 80)
        receiver = bridge.uwb_fusion.receivers[1]
        self.assertEqual(receiver.filter_reason, 'outside_receiver_fov')
        self.assertEqual(receiver.measurement_updated_at, 1000)
        self.assertEqual(f.raw_window, history)
        self.assertEqual(f.accepted_count, 1)
        pair(1350, 0)
        self.assertEqual(receiver.measurement_updated_at, 1350)
        self.assertTrue(bridge.uwb_fusion.fused(1350)['fresh'])


if __name__ == '__main__':
    unittest.main()
