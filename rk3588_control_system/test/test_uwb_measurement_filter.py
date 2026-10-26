import math
import os
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'backend'))
from uwb_measurement_filter import GatedUwbKalman
from uwb_fusion import DualUwbFusion
from backend.mavlink_bridge import MAVLinkBridge, FlightData, UWBData


class MeasurementFilterTests(unittest.TestCase):
    def test_disabled_trust_checks_accept_jumps_but_keep_smoothing_and_validation(self):
        f = GatedUwbKalman({'trust_checks_enabled': False})
        self.warm(f)
        changed = f.update(15, 85, 2200, 0, sequence_id=12)
        self.assertTrue(changed['accepted'])
        self.assertGreater(changed['nis'], 9.21)
        self.assertEqual(changed['occlusion_diagnostics']['state'], 'disabled')
        self.assertEqual(changed['trust_diagnostics']['status'], 'suspect')
        self.assertFalse(changed['trust_diagnostics']['controlGating'])
        self.assertNotAlmostEqual(changed['distance_m'], 15)
        self.assertTrue(f.update(15, 85, 2250, 0, sequence_id=12)['duplicate'])
        for i, (distance, angle, valid, reason) in enumerate([
                (float('nan'), 0, True, 'distance_out_of_range'),
                (3, 181, True, 'azimuth_out_of_range'),
                (3, 0, False, 'radio_invalid')]):
            result = f.update(distance, angle, 2300 + i*25, 0, valid=valid, sequence_id=13+i)
            self.assertFalse(result['accepted'])
            self.assertEqual(result['reason'], reason)
        self.assertTrue(f.update(4, -85, 2400, 0, sequence_id=20)['accepted'])

    def test_shadow_quarantine_cannot_change_control_acceptance_or_radio_recovery(self):
        f = GatedUwbKalman({'trust_checks_enabled': False})
        for i in range(200):
            result = f.update(3 if i < 40 else (5 if i % 2 else 7), 0, 1000+i*25, 0, sequence_id=i)
            self.assertTrue(result['accepted'])
        self.assertEqual(result['trust_diagnostics']['status'], 'suspect')
        self.assertEqual(f.rejected_count, 0)
        self.assertFalse(f.reject(6000, 'radio_invalid', 0)['accepted'])
        self.assertTrue(f.update(8, 0, 6025, 0, sequence_id=201)['accepted'])

    def warm(self, f):
        for i in range(12): f.update(3, 0, 1000 + 100 * i, 0, sequence_id=i)

    def test_trusted_noise_is_smoothed_and_position_covariance_contracts(self):
        f = GatedUwbKalman()
        first = f.update(3, 0, 1000, 0)
        second = f.update(3.1, 0, 1100, 0)
        self.assertTrue(second['accepted'])
        self.assertGreater(second['distance_m'], 3)
        self.assertLess(second['distance_m'], 3.1)
        self.assertLess(second['uncertainty_m'], first['uncertainty_m'])

    def test_outlier_is_checked_before_correction_and_only_inflates_prediction(self):
        f = GatedUwbKalman(); self.warm(f)
        old_at, old_count = f.last_accepted_at, f.accepted_count
        before = f.result(2100, 0)['uncertainty_m']
        expected = [axis.predicted(0.1, 4)[0] for axis in f.axes]
        result = f.update(15, 0, 2200, 0, sequence_id=12)
        self.assertFalse(result['accepted']); self.assertEqual(result['reason'], 'innovation_outlier')
        self.assertGreater(result['nis'], 9.21)
        self.assertEqual(f.last_accepted_at, old_at); self.assertEqual(f.accepted_count, old_count)
        self.assertEqual([axis.position for axis in f.axes], expected)
        self.assertGreater(result['uncertainty_m'], before)
        self.assertLess(result['distance_m'], 4)
        recovered = f.update(3.02, 0, 2300, 0, sequence_id=13)
        self.assertTrue(recovered['accepted']); self.assertEqual(f.last_accepted_at, 2300)

    def test_basic_invalid_inputs_never_initialize_or_correct_a_filter(self):
        for distance, bearing, kwargs, reason in [
            (float('nan'), 0, {}, 'distance_out_of_range'), (25, 0, {}, 'distance_out_of_range'),
            (3, 220, {}, 'azimuth_out_of_range'), (3, 0, {'elevation_deg': 100}, 'elevation_out_of_range'),
            (3, 0, {'valid': False}, 'radio_invalid')
        ]:
            f = GatedUwbKalman(); result = f.update(distance, bearing, 1000, 0, **kwargs)
            self.assertFalse(result['accepted']); self.assertEqual(result['reason'], reason)
            self.assertIsNone(f.axes); self.assertIsNone(f.last_accepted_at)
        self.assertFalse(GatedUwbKalman().update(3, 0, 1000, None)['accepted'])

    def test_duplicate_sequences_cannot_rejuvenate_a_stale_measurement(self):
        f = GatedUwbKalman(); f.update(3, 0, 1000, 0, sequence_id=7)
        state = [(a.position, a.velocity, a.p00, a.p01, a.p11) for a in f.axes]
        result = f.update(3.2, 0, 1200, 0, sequence_id=7)
        self.assertTrue(result['duplicate']); self.assertEqual(f.last_accepted_at, 1000)
        self.assertEqual([(a.position, a.velocity, a.p00, a.p01, a.p11) for a in f.axes], state)
        self.assertEqual(f.accepted_count, 1)

    def test_angular_wrap_is_small_cartesian_motion_and_heading_rotation_is_compensated(self):
        f = GatedUwbKalman()
        f.update(3, 179, 1000, 0)
        result = f.update(3, -179, 1100, 0)
        self.assertTrue(result['accepted']); self.assertLess(result['innovation_m'], 0.12)
        g = GatedUwbKalman(); g.update(3, 0, 1000, 0)
        result = g.update(3, -90, 1100, 90)
        self.assertTrue(result['accepted']); self.assertLess(result['innovation_m'], 1e-8)
        self.assertAlmostEqual(result['azimuth_deg'], -90)

    def test_sign_scale_and_mount_are_inverted_consistently(self):
        f = GatedUwbKalman(bearing_sign=-1, bearing_scale=0.8, bearing_offset_deg=-5, mount_yaw_deg=20)
        result = f.update(4, 30, 1000, 45)
        self.assertTrue(result['accepted']); self.assertAlmostEqual(result['azimuth_deg'], 30)
        self.assertAlmostEqual(result['distance_m'], 4)

    def test_uncertainty_grows_with_missing_data_and_never_freshens_accepted_time(self):
        f = GatedUwbKalman(); f.update(3, 0, 1000, 0)
        p0 = f.result(1000, 0)['uncertainty_m']; p1 = f.result(1400, 0)['uncertainty_m']
        p2 = f.result(2000, 0)['uncertainty_m']
        self.assertLess(p0, p1); self.assertLess(p1, p2); self.assertEqual(f.last_accepted_at, 1000)
        self.assertEqual(f.result(2000, 0)['accepted_at'], 1000)

    def test_rejections_preserve_positive_finite_covariance(self):
        f = GatedUwbKalman(); self.warm(f)
        for i in range(200):
            f.update(19, 100, 2200 + i * 100, 0, sequence_id=100 + i)
            for a in f.axes:
                self.assertTrue(all(math.isfinite(v) for v in (a.position, a.velocity, a.p00, a.p01, a.p11)))
                self.assertGreaterEqual(a.p00 * a.p11 - a.p01 ** 2, -1e-6)

    def test_identity_and_clock_changes_invalidate_previous_measurement(self):
        f = GatedUwbKalman(); f.update(3, 0, 1000, 0, tag_id=1)
        result = f.update(3, 0, 1100, 0, tag_id=2)
        self.assertFalse(result['accepted']); self.assertEqual(result['reason'], 'target_identity_changed')
        self.assertIsNone(f.last_accepted_at)
        f.update(3, 0, 1200, 0, tag_id=2)
        self.assertEqual(f.update(3, 0, 900, 0)['reason'], 'measurement_clock_reset')


class BridgeGateTests(unittest.TestCase):
    def setUp(self):
        self.bridge = MAVLinkBridge.__new__(MAVLinkBridge)
        self.bridge.flight_data = FlightData()
        self.bridge.uwb_data = UWBData()
        self.bridge.uwb_filter_enabled = True
        self.bridge.uwb_raw_control = False  # This fixture exercises filtered admission.
        self.bridge.uwb_fusion = DualUwbFusion(enabled_receiver_ids=(1,), require_gated_measurements=True)
        self.bridge._uwb_measurement_filters = {1: GatedUwbKalman(), 2: GatedUwbKalman()}
        class ForbiddenLegacyFilter:
            def update(self, value): raise AssertionError('legacy pre-gate filter was used')
        self.bridge._uwb_filters = {(1, key): ForbiddenLegacyFilter() for key in ('DIST', 'AZ', 'EL')}

    def frame(self, now, distance=3, bearing=0, seq=None):
        self.bridge.flight_data.attitude_updated_at = now
        with patch('backend.mavlink_bridge.time.time', return_value=now / 1000):
            self.bridge._update_uwb_named_value('UWB1_VALID', 1)
            if seq is not None: self.bridge._update_uwb_named_value('UWB1_SEQ', seq)
            self.bridge._update_uwb_named_value('UWB1_DIST', distance)
            self.bridge._update_uwb_named_value('UWB1_AZ', bearing)

    def test_real_bridge_gates_raw_before_kalman_and_rejected_data_never_enters_fusion_history(self):
        for i in range(12): self.frame(1000 + 100 * i, seq=i)
        r = self.bridge.uwb_fusion.receivers[1]; f = self.bridge._uwb_measurement_filters[1]
        self.assertTrue(r.fresh(2100, 800)); old_at = r.measurement_updated_at
        history = list(self.bridge.uwb_fusion._distance_history[1]); old_count = f.accepted_count
        self.frame(2200, distance=15, seq=12)
        self.assertEqual(r.raw_distance_m, 15); self.assertLess(r.distance_m, 4)
        self.assertEqual(r.measurement_status, 'suspect'); self.assertFalse(r.fresh(2200, 800))
        self.assertEqual(r.measurement_updated_at, old_at); self.assertEqual(f.accepted_count, old_count)
        self.assertEqual(list(self.bridge.uwb_fusion._distance_history[1]), history)
        self.assertFalse(self.bridge.uwb_fusion.fused(2200)['fresh'])
        self.frame(2300, distance=3.02, seq=13)
        self.assertTrue(r.fresh(2300, 800)); self.assertEqual(r.measurement_updated_at, 2300)

    def test_complete_pair_required_and_invalid_angles_are_not_wrapped_into_valid_measurements(self):
        with patch('backend.mavlink_bridge.time.time', return_value=1):
            self.bridge.flight_data.attitude_updated_at = 1000
            self.bridge._update_uwb_named_value('UWB1_DIST', 3)
            self.assertIsNone(self.bridge.uwb_fusion.receivers[1].measurement_updated_at)
            self.bridge._update_uwb_named_value('UWB1_AZ', 400)
        self.assertEqual(self.bridge.uwb_fusion.receivers[1].filter_reason, 'azimuth_out_of_range')
        self.assertIsNone(self.bridge._uwb_measurement_filters[1].axes)

    def wire_value(self, now, name, value, sample_time):
        self.bridge.flight_data.attitude_updated_at = now
        msg = SimpleNamespace(
            get_type=lambda: 'NAMED_VALUE_FLOAT', get_srcSystem=lambda: 1,
            get_srcComponent=lambda: 1, name=name, value=value,
            time_boot_ms=sample_time,
        )
        with patch('backend.mavlink_bridge.time.time', return_value=now / 1000):
            self.bridge._parse_message(msg)

    def wire_pair(self, now, sample_time, distance=3):
        self.wire_value(now, 'UWB1_VALID', 1, sample_time)
        self.wire_value(now, 'UWB1_DIST', distance, sample_time)
        self.wire_value(now, 'UWB1_EL', 0, sample_time)
        self.wire_value(now, 'UWB1_AZ', 0, sample_time)

    def test_distinct_frames_at_one_receive_time_are_not_duplicates(self):
        self.wire_pair(1000, 100)
        self.wire_pair(1000, 120, distance=3.1)
        self.assertEqual(self.bridge._uwb_measurement_filters[1].accepted_count, 2)
        self.assertGreater(self.bridge.uwb_fusion.receivers[1].distance_m, 3)

    def test_near_frame_in_same_batch_invalidates_far_posterior(self):
        self.wire_pair(1000, 100)
        self.wire_pair(1000, 120, distance=.2)
        sensor = self.bridge.uwb_fusion.receivers[1]
        self.assertFalse(sensor.fresh(1000, 800))
        self.assertEqual(sensor.raw_distance_m, .2)
        self.assertEqual(sensor.measurement_status, 'suspect')

    def test_dropped_fields_cannot_form_a_cross_frame_pair(self):
        self.wire_value(1000, 'UWB1_DIST', 3, 100)
        self.wire_value(1050, 'UWB1_AZ', 20, 150)
        sensor = self.bridge.uwb_fusion.receivers[1]
        self.assertIsNone(sensor.measurement_updated_at)
        self.assertIsNone(self.bridge._uwb_measurement_filters[1].axes)
        self.wire_pair(1100, 200)
        self.assertEqual(sensor.measurement_updated_at, 1100)

    def test_nonfinite_field_poisons_whole_observation(self):
        for i, field in enumerate(('DIST', 'AZ', 'EL')):
            stamp, now = 100 + i * 100, 1000 + i * 100
            self.wire_value(now, 'UWB1_DIST', 3, stamp)
            self.wire_value(now, 'UWB1_' + field, float('nan'), stamp)
            self.wire_value(now, 'UWB1_DIST', 3, stamp)
            self.wire_value(now, 'UWB1_AZ', 0, stamp)
            self.assertIsNone(self.bridge.uwb_fusion.receivers[1].measurement_updated_at)
        self.wire_pair(1400, 500)
        self.assertEqual(self.bridge.uwb_fusion.receivers[1].measurement_updated_at, 1400)

    def test_new_wire_samples_stay_fresh_with_five_second_diagnostic_sequence(self):
        self.wire_value(1000, 'UWB1_SEQ', 7, 100)
        receiver = self.bridge.uwb_fusion.receivers[1]
        for i in range(40):
            now = 1000 + i * 100
            self.wire_pair(now, 100 + i * 100)
            self.assertTrue(receiver.fresh(now, 800))
            self.assertEqual(receiver.measurement_updated_at, now)
        self.assertEqual(receiver.sequence_id, 7)
        self.assertEqual(self.bridge._uwb_measurement_filters[1].accepted_count, 40)

    def test_replayed_wire_pair_does_not_refresh_accepted_measurement(self):
        self.wire_pair(1000, 100)
        self.wire_value(1500, 'UWB1_SEQ', 8, 600)
        self.wire_pair(2000, 100)
        receiver = self.bridge.uwb_fusion.receivers[1]
        self.assertEqual(receiver.measurement_updated_at, 1000)
        self.assertFalse(receiver.fresh(2000, 800))
        self.assertEqual(self.bridge._uwb_measurement_filters[1].accepted_count, 1)

    def test_wire_metadata_and_incomplete_or_outlier_samples_cannot_refresh_measurement(self):
        self.wire_pair(1000, 100)
        for name in ('SEQ', 'GOOD', 'HB', 'EL', 'VALID'):
            self.wire_value(2000, 'UWB1_' + name, 1, 1100)
        self.wire_value(2000, 'UWB1_DIST', 3, 1100)
        receiver = self.bridge.uwb_fusion.receivers[1]
        self.assertEqual(receiver.measurement_updated_at, 1000)
        self.assertFalse(receiver.fresh(2000, 800))
        self.wire_pair(2100, 1200)
        for i in range(12):
            self.wire_pair(2200 + i * 100, 1300 + i * 100)
        accepted_at = receiver.measurement_updated_at
        self.wire_pair(3400, 2500, distance=15)
        self.assertEqual(receiver.measurement_updated_at, accepted_at)
        self.assertEqual(receiver.filter_reason, 'innovation_outlier')

    def test_sender_clock_wrap_is_a_new_measurement(self):
        self.wire_pair(1000, 4294967200)
        self.wire_pair(1100, 0)
        self.assertEqual(self.bridge.uwb_fusion.receivers[1].measurement_updated_at, 1100)

    def test_intervening_frame_does_not_allow_old_replay_to_refresh_clocks(self):
        self.wire_pair(1000, 100)
        self.wire_pair(1025, 125)
        self.wire_pair(2000, 100)
        receiver = self.bridge.uwb_fusion.receivers[1]
        self.assertEqual(receiver.measurement_updated_at, 1025)
        self.assertEqual(receiver.distance_sample_at, 1025)
        self.assertFalse(receiver.fresh(2000, 800))

    def test_missing_or_late_invalid_elevation_cannot_commit_or_revive(self):
        self.wire_value(1000, 'UWB1_DIST', 3, 100)
        self.wire_value(1000, 'UWB1_AZ', 0, 100)
        receiver = self.bridge.uwb_fusion.receivers[1]
        self.assertIsNone(receiver.measurement_updated_at)
        self.wire_value(1025, 'UWB1_EL', 100, 100)
        self.wire_pair(1025, 100)
        self.assertIsNone(receiver.measurement_updated_at)
        self.assertEqual(receiver.filter_reason, 'elevation_out_of_range')
        self.wire_pair(1050, 150)
        self.assertEqual(receiver.measurement_updated_at, 1050)
        self.wire_value(1075, 'UWB1_EL', -100, 150)
        self.assertFalse(receiver.fresh(1075, 800))
        self.wire_pair(1075, 150)
        self.assertFalse(receiver.fresh(1075, 800))

if __name__ == '__main__': unittest.main()
