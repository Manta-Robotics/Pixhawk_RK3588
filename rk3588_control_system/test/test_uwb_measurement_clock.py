import os
import sys
import unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'backend'))
from uwb_fusion import DualUwbFusion


class MeasurementClockTests(unittest.TestCase):
    def test_different_old_partial_frames_cannot_form_a_fresh_observation(self):
        f = DualUwbFusion()
        f.update('UWB1_DIST', 3, now_ms=1000)
        f.update('UWB1_AZ', 10, now_ms=2000)
        self.assertIsNone(f.receivers[1].measurement_updated_at)
        f.update('UWB1_DIST', 3, now_ms=2010)
        self.assertEqual(f.receivers[1].measurement_updated_at, 2010)

    def test_metadata_cannot_refresh_a_complete_measurement(self):
        f = DualUwbFusion()
        f.update('UWB1_DIST', 3, now_ms=1000)
        f.set_filtered(1, 'DIST', 3)
        self.assertIsNone(f.receivers[1].measurement_updated_at)
        f.update('UWB1_AZ', 10, now_ms=1010)
        f.set_filtered(1, 'AZ', 10)
        self.assertEqual(f.receivers[1].measurement_updated_at, 1010)
        for field in ['VALID', 'HB', 'SEQ', 'GOOD', 'EL']:
            f.update('UWB1_' + field, 1, now_ms=2000)
        self.assertEqual(f.receivers[1].measurement_updated_at, 1010)
        self.assertEqual(f.receivers[1].telemetry(2000, 1500)['measurementUpdatedAt'], 1010)

    def test_invalid_measurement_clears_incomplete_pair(self):
        f = DualUwbFusion()
        f.update('UWB1_DIST', 3, now_ms=1000)
        f.update('UWB1_VALID', 0, now_ms=1010)
        f.update('UWB1_AZ', 10, now_ms=1020)
        self.assertIsNone(f.receivers[1].measurement_updated_at)
        f.update('UWB1_DIST', 3, now_ms=1030)
        self.assertEqual(f.receivers[1].measurement_updated_at, 1030)

if __name__ == '__main__':
    unittest.main()
