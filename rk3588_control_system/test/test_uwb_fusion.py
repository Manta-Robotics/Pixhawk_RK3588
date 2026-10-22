import math
import os
import sys
import unittest


sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "backend"))

from uwb_fusion import DualUwbFusion


class DualUwbFusionTests(unittest.TestCase):
    def feed(self, fusion, receiver, distance, azimuth, good=1, now_ms=1000):
        prefix = f"UWB{receiver}_"
        self.assertTrue(fusion.update(prefix + "DIST", distance, now_ms=now_ms))
        fusion.set_filtered(receiver, "DIST", distance)
        self.assertTrue(fusion.update(prefix + "AZ", azimuth, now_ms=now_ms))
        fusion.set_filtered(receiver, "AZ", azimuth)
        self.assertTrue(fusion.update(prefix + "GOOD", good, now_ms=now_ms))

    def test_applies_ninety_degree_mount_before_fusion(self):
        fusion = DualUwbFusion((0, 90), receiver_fov_half_deg=80)
        self.feed(fusion, 1, 4.0, 30.0, good=5)
        self.feed(fusion, 2, 4.0, -60.0, good=5)
        result = fusion.fused(now_ms=1100)
        self.assertEqual(result["receiver_count"], 2)
        self.assertAlmostEqual(result["azimuth_deg"], 30.0, places=6)
        self.assertAlmostEqual(result["distance_m"], 4.0, places=6)

    def test_circular_mean_crosses_180_without_flipping_to_zero(self):
        fusion = DualUwbFusion((170, -170))
        self.feed(fusion, 1, 2.0, 9.0)
        self.feed(fusion, 2, 2.0, -9.0)
        result = fusion.fused(now_ms=1100)
        self.assertLess(abs(abs(result["azimuth_deg"]) - 180.0), 1e-6)

    def test_falls_back_to_one_fresh_receiver(self):
        fusion = DualUwbFusion((0, 90), freshness_ms=500)
        self.feed(fusion, 1, 3.0, 10.0, now_ms=100)
        self.feed(fusion, 2, 5.0, -20.0, now_ms=900)
        result = fusion.fused(now_ms=1000)
        self.assertEqual(result["receiver_count"], 1)
        self.assertAlmostEqual(result["azimuth_deg"], 70.0)
        self.assertAlmostEqual(result["distance_m"], 5.0)

    def test_legacy_names_are_assigned_by_mavlink_source(self):
        fusion = DualUwbFusion((0, 90), receiver_fov_half_deg=80)
        self.assertTrue(fusion.update("UWB_DIST", 2.0, source=(21, 1), now_ms=1000))
        fusion.set_filtered(1, "DIST", 2.0)
        self.assertTrue(fusion.update("UWB_AZ", 15.0, source=(21, 1), now_ms=1000))
        fusion.set_filtered(1, "AZ", 15.0)
        self.assertTrue(fusion.update("UWB_DIST", 2.0, source=(22, 1), now_ms=1000))
        fusion.set_filtered(2, "DIST", 2.0)
        self.assertTrue(fusion.update("UWB_AZ", -75.0, source=(22, 1), now_ms=1000))
        fusion.set_filtered(2, "AZ", -75.0)
        result = fusion.fused(now_ms=1100)
        self.assertEqual(result["receiver_count"], 2)
        self.assertAlmostEqual(result["azimuth_deg"], 15.0)

    def test_accepts_suffix_at_end_for_sender_compatibility(self):
        fusion = DualUwbFusion()
        self.assertEqual(fusion.parse_name("UWB_AZ2"), (2, "AZ"))
        self.assertEqual(fusion.parse_name("UWB2_AZ"), (2, "AZ"))
        self.assertIsNone(fusion.parse_name("NOT_UWB"))

    def test_preserves_vendor_packet_metadata_per_receiver(self):
        fusion = DualUwbFusion((-45, 45))
        for name, value in (
            ("UWB2_SEQ", 39),
            ("UWB2_ANCH", 0xAAA2),
            ("UWB2_TAG", 0xAAA1),
            ("UWB2_STAT", 0x1234),
            ("UWB2_RX", 160),
            ("UWB2_HB", 10),
            ("UWB2_CRC", 2),
            ("UWB2_LEN", 1),
        ):
            self.assertTrue(fusion.update(name, value, now_ms=1000))
        sensor = fusion.fused(now_ms=1000)["sensors"][1]
        self.assertEqual(sensor["sequenceId"], 39)
        self.assertEqual(sensor["anchorId"], 0xAAA2)
        self.assertEqual(sensor["tagId"], 0xAAA1)
        self.assertEqual(sensor["tagStatus"], 0x1234)
        self.assertEqual(sensor["rxBytes"], 160)
        self.assertEqual(sensor["heartbeatCount"], 10)
        self.assertEqual(sensor["checksumErrors"], 2)
        self.assertEqual(sensor["lengthErrors"], 1)

    def test_valid_zero_excludes_receiver_until_valid_frame_returns(self):
        fusion = DualUwbFusion((0, 90))
        self.feed(fusion, 1, 3.0, 20.0)
        fusion.update("UWB1_VALID", 0, now_ms=1050)
        self.assertEqual(fusion.fused(now_ms=1100)["receiver_count"], 0)
        fusion.update("UWB1_VALID", 1, now_ms=1150)
        self.assertEqual(fusion.fused(now_ms=1200)["receiver_count"], 1)

    def test_overlap_prefers_receiver_closer_to_antenna_center(self):
        fusion = DualUwbFusion((0, 90), overlap_disagreement_deg=30)
        self.feed(fusion, 1, 4.0, 20.0)
        self.feed(fusion, 2, 4.0, -60.0)
        result = fusion.fused(now_ms=1100)
        self.assertEqual(result["fusion_mode"], "overlap_weighted_fusion")
        self.assertEqual(result["receiver_count"], 2)
        self.assertLess(result["azimuth_deg"], 24.0)

    def test_overlap_rejects_inconsistent_receiver(self):
        fusion = DualUwbFusion((0, 90), overlap_disagreement_deg=30)
        self.feed(fusion, 1, 4.0, 0.0)
        self.feed(fusion, 2, 4.0, 0.0)
        result = fusion.fused(now_ms=1100)
        self.assertEqual(result["observing_receiver_count"], 2)
        self.assertEqual(result["receiver_count"], 1)
        self.assertEqual(result["fusion_mode"], "hysteretic_receiver_select")
        self.assertAlmostEqual(result["azimuth_deg"], 0.0)

    def test_outlier_rejection_prefers_stable_receiver_over_central_unstable_one(self):
        fusion = DualUwbFusion((0, 90), overlap_disagreement_deg=30)
        for sample in range(20):
            unstable_distance = 1.0 if sample % 2 == 0 else 20.0
            self.feed(fusion, 1, unstable_distance, 0.0, now_ms=1000 + sample)
            self.feed(fusion, 2, 2.0 + (sample % 3) * 0.01, -45.0, now_ms=1000 + sample)

        result = fusion.fused(now_ms=1100)
        self.assertEqual(result["fusion_mode"], "hysteretic_receiver_select")
        self.assertEqual(result["active_receiver_ids"], [2])
        self.assertAlmostEqual(result["distance_m"], 2.01, places=6)
        self.assertLess(result["sensors"][0]["stabilityWeight"], 0.1)
        self.assertGreater(result["sensors"][1]["stabilityWeight"], 0.9)

    def test_rejects_zero_distance_instead_of_reusing_old_bearing(self):
        fusion = DualUwbFusion((45, -45), minimum_distance_m=0.15)
        self.assertEqual(fusion.update("UWB1_DIST", 0.0, now_ms=1000), (1, "REJECTED"))
        fusion.set_filtered(1, "AZ", 0.0)
        result = fusion.fused(now_ms=1050)
        self.assertEqual(result["receiver_count"], 0)
        self.assertEqual(result["sensors"][0]["invalidReason"], "distance_out_of_range")

    def test_transforms_sensor_origin_before_cartesian_fusion(self):
        fusion = DualUwbFusion(
            (0, 0),
            receiver_position_x_m=(0.2, 0.2),
            receiver_position_y_m=(-0.1, 0.1),
        )
        self.feed(fusion, 1, 1.0, 0.0)
        self.feed(fusion, 2, 1.0, 0.0)
        result = fusion.fused(now_ms=1100)
        self.assertAlmostEqual(result["relative_x_m"], 1.2, places=6)
        self.assertAlmostEqual(result["relative_y_m"], 0.0, places=6)
        self.assertAlmostEqual(result["distance_m"], 1.2, places=6)

    def test_outlier_link_switch_requires_confidence_to_persist(self):
        fusion = DualUwbFusion(
            (0, 90),
            overlap_disagreement_deg=30,
            switch_confidence_ratio=1.1,
            switch_confirm_ms=180,
            switch_blend_ms=0,
        )
        self.feed(fusion, 1, 2.0, 0.0, now_ms=1000)
        self.feed(fusion, 2, 2.0, 80.0, now_ms=1000)
        self.assertEqual(fusion.fused(now_ms=1010)["active_receiver_ids"], [1])
        self.feed(fusion, 1, 2.0, 50.0, now_ms=1100)
        self.feed(fusion, 2, 2.0, 0.0, now_ms=1100)
        self.assertEqual(fusion.fused(now_ms=1110)["active_receiver_ids"], [1])
        self.assertEqual(fusion.fused(now_ms=1300)["active_receiver_ids"], [2])

    def test_rejects_large_cartesian_range_disagreement_even_when_bearings_agree(self):
        fusion = DualUwbFusion((0, 0), overlap_position_disagreement_m=0.45)
        self.feed(fusion, 1, 0.8, 0.0, now_ms=1000)
        self.feed(fusion, 2, 4.0, 0.0, now_ms=1000)
        result = fusion.fused(now_ms=1100)
        self.assertEqual(result["fusion_mode"], "hysteretic_receiver_select")
        self.assertEqual(result["receiver_count"], 1)
        self.assertGreater(result["position_disagreement_m"], 3.0)


if __name__ == "__main__":
    unittest.main()
