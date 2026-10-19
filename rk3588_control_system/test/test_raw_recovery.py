import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'backend'))
from uwb_measurement_filter import GatedUwbKalman
from uwb_fusion import DualUwbFusion


class RawRecoveryTests(unittest.TestCase):
    def test_invalid_raw_event_is_visible_without_faking_complete_or_accepted_time(self):
        fusion=DualUwbFusion(require_gated_measurements=True)
        fusion.update('UWB1_DIST',3,now_ms=1000);fusion.update('UWB1_AZ',0,now_ms=1001)
        before=fusion.receivers[1].received_measurement_at
        fusion.update('UWB1_DIST',-4,now_ms=1100)
        sensor=fusion.receivers[1].telemetry(1100,700)
        self.assertEqual(sensor['receivedMeasurementAt'],before)
        self.assertEqual(sensor['rawRejectedAt'],1100)
        self.assertIsNone(sensor['measurementUpdatedAt'])

    def corrupted(self):
        f=GatedUwbKalman()
        for n in range(15):f.update(3,0,1000+100*n,0,sequence_id=n)
        # Force an obsolete low-covariance estimate to reproduce gate lockout.
        for axis in f.axes:axis.position+=4;axis.p00=.001;axis.p01=0;axis.p11=.001
        return f

    def test_raw_consensus_rebuilds_rejecting_filter(self):
        f=self.corrupted()
        for n in range(3):result=f.update(3+.01*n,0,3000+100*n,0,sequence_id=20+n)
        self.assertTrue(result['accepted'])
        self.assertEqual(result['reason'],'raw_window_reacquired')
        self.assertEqual(result['recovery_count'],1)
        self.assertLess(result['uncertainty_m'],.5)
        self.assertAlmostEqual(result['distance_m'],3.02,places=3)
        self.assertEqual(result['accepted_at'],3200)

    def test_one_invalid_point_does_not_destroy_previous_evidence(self):
        f=self.corrupted()
        f.update(3,0,3000,0,sequence_id=20)
        invalid=f.update(-4,0,3100,0,sequence_id=21)
        self.assertFalse(invalid['accepted'])
        f.update(3.02,0,3200,0,sequence_id=22)
        result=f.update(3.03,0,3300,0,sequence_id=23)
        self.assertEqual(result['reason'],'raw_window_reacquired')

    def test_duplicates_and_old_candidates_cannot_reinitialize(self):
        f=self.corrupted()
        f.update(3,0,3000,0,sequence_id=20)
        for _ in range(20):f.update(3,0,3000,0,sequence_id=20)
        self.assertEqual(len(f.raw_window),1)
        for t,d in [(4000,3),(4100,12),(4200,1)]:f.update(d,0,t,0,sequence_id=t)
        self.assertEqual(f.recovery_count,0)
        self.assertTrue(all(p['at']>=4000 for p in f.raw_window))

    def test_identity_change_discards_candidate_history(self):
        f=self.corrupted();f.tag_id=1
        f.update(3,0,3000,0,sequence_id=20,tag_id=1)
        result=f.update(3,0,3100,0,sequence_id=21,tag_id=2)
        self.assertEqual(result['reason'],'target_identity_changed')
        self.assertEqual(f.raw_window,[])

    def test_rejection_does_not_multiply_covariance_per_packet(self):
        f=self.corrupted()
        before=[(a.p00,a.p01,a.p11) for a in f.axes]
        for _ in range(100):f.reject(2400,'radio_invalid',0)
        self.assertEqual(before,[(a.p00,a.p01,a.p11) for a in f.axes])

    def test_distant_raw_track_can_reinitialize_without_faking_precision(self):
        f=self.corrupted()
        for n in range(3):result=f.update(9+.01*n,0,3000+100*n,0,sequence_id=20+n)
        self.assertEqual(result['reason'],'raw_window_reacquired')
        self.assertGreater(result['uncertainty_m'],.5)
        initial_uncertainty=result['uncertainty_m']
        # Repeated genuine samples, not covariance clipping, improve precision.
        for n in range(3,25):result=f.update(9+.01*n,0,3000+100*n,0,sequence_id=20+n)
        self.assertLess(result['uncertainty_m'],initial_uncertainty)
        self.assertGreater(result['uncertainty_m'],.5)


if __name__=='__main__':unittest.main()
