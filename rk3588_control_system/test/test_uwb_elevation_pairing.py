import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'backend'))
from uwb_fusion import DualUwbFusion


class ElevationPairingTest(unittest.TestCase):
    def frame(self, fusion, clock, elevation=None):
        fusion.update('UWB1_DIST', 2, now_ms=clock, sample_time_boot_ms=clock)
        if elevation is not None:
            fusion.update('UWB1_EL', elevation, now_ms=clock, sample_time_boot_ms=clock)
        fusion.update('UWB1_AZ', 10, now_ms=clock, sample_time_boot_ms=clock)
        return fusion.receivers[1].raw_observation

    def test_dropped_elevation_does_not_renew_the_previous_vertical_angle(self):
        fusion = DualUwbFusion(ordered_observations=True)
        first = self.frame(fusion, 1000, 25)
        second = self.frame(fusion, 1025)
        self.assertEqual(first['elevationDeg'], 25)
        self.assertIsNone(second['elevationDeg'])
        self.assertEqual(second['azimuthDeg'], 10)
        self.assertEqual(second['at'], 1025)

    def test_elevation_from_another_source_frame_is_not_part_of_the_observation(self):
        fusion = DualUwbFusion(ordered_observations=True)
        fusion.update('UWB1_DIST', 2, now_ms=1000, sample_time_boot_ms=1000)
        fusion.update('UWB1_EL', 25, now_ms=1001, sample_time_boot_ms=975)
        fusion.update('UWB1_AZ', 10, now_ms=1002, sample_time_boot_ms=1000)
        self.assertIsNone(fusion.receivers[1].raw_observation['elevationDeg'])

    def test_zero_and_rollover_elevation_are_valid_when_source_clocks_match(self):
        fusion = DualUwbFusion(ordered_observations=True)
        for clock in [1000, 2 ** 32 - 2, 2]:
            self.assertEqual(self.frame(fusion, clock, 0)['elevationDeg'], 0)

    def test_fused_output_keeps_existing_filtering_but_does_not_reuse_a_missing_elevation(self):
        fusion = DualUwbFusion(ordered_observations=True, enabled_receiver_ids=[1], mount_yaw_deg=[0, 0])
        fusion.update('UWB1_DIST', 2, now_ms=1000)
        fusion.set_filtered(1, 'DIST', 2)
        fusion.update('UWB1_EL', 25, now_ms=1000)
        fusion.set_filtered(1, 'EL', 20)
        fusion.update('UWB1_AZ', 10, now_ms=1000)
        fusion.set_filtered(1, 'AZ', 10)
        self.assertEqual(fusion.fused(1000)['elevation_deg'], 20)
        self.frame(fusion, 1025)
        self.assertTrue(fusion.fused(1025)['fresh'])
        self.assertIsNone(fusion.fused(1025)['elevation_deg'])


if __name__ == '__main__':
    unittest.main()
