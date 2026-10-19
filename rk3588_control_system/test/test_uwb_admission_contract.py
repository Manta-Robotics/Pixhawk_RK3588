import pathlib
import sys
import unittest
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'backend'))
from uwb_measurement_filter import GatedUwbKalman
from uwb_fusion import DualUwbFusion


class AdmissionContractTest(unittest.TestCase):
    def test_filter_contract_survives_fusion_and_rejection_does_not_refresh_clock(self):
        gate = GatedUwbKalman()
        fusion = DualUwbFusion(enabled_receiver_ids=(1,), require_gated_measurements=True)
        def feed(at, value):
            fusion.update('UWB1_DIST', value, now_ms=at)
            fusion.update('UWB1_AZ', 0, now_ms=at)
            result = gate.update(value, 0, at, 0, sequence_id=at)
            fusion.apply_measurement_result(1, result)
            return fusion.fused(at)['sensors'][0]
        for at in range(1000, 2200, 100):
            sensor = feed(at, 3)
        self.assertEqual(sensor['filterAdmissionVersion'], 1)
        self.assertEqual(sensor['measurementStatus'], 'trusted')
        self.assertEqual(sensor['measurementUpdatedAt'], sensor['receivedMeasurementAt'])
        rejected = feed(2200, 15)
        self.assertEqual(rejected['measurementStatus'], 'suspect')
        self.assertEqual(rejected['measurementUpdatedAt'], 2100)
        self.assertEqual(rejected['receivedMeasurementAt'], 2200)
        restored = feed(2300, 3)
        self.assertEqual(restored['measurementStatus'], 'trusted')
        self.assertEqual(restored['measurementUpdatedAt'], 2300)
        fusion.apply_measurement_result(1, gate.update(3, 0, 2400, 0, sequence_id=2300))
        self.assertEqual(fusion.fused(2400)['sensors'][0]['measurementUpdatedAt'], 2300)


if __name__ == '__main__':
    unittest.main()
