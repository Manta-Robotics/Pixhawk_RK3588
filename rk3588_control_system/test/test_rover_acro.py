import math
import os
import sys
import time
import unittest
from unittest.mock import Mock
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'backend'))
from rover_acro_mapping import required_parameters, validate_parameters, acro_axes, decode_axis
from mavlink_bridge import MAVLinkBridge, FlightData

def parameters():
    values={'PILOT_STEER_TYPE':0, 'SPEED_MAX':2, 'CRUISE_SPEED':2, 'CRUISE_THROTTLE':50,
            'ACRO_TURN_RATE':180, 'RCMAP_ROLL':1, 'RCMAP_THROTTLE':3}
    for channel in (1,3):
        values.update({f'RC{channel}_{k}':v for k,v in {'MIN':1100,'MAX':1900,'TRIM':1500,'DZ':30,'REVERSED':0}.items()})
    return values

class AcroMappingTests(unittest.TestCase):
    def test_all_speed_yaw_combinations_match_firmware_decode_and_do_not_exceed_limits(self):
        p=parameters()
        for speed in (-5,-2,-.2,0,.05,.2,.8,1,2,5):
            for yaw in (-40,-14,-5,0,5,14,40):
                y,z=acro_axes(speed,yaw,p)
                actual_speed=decode_axis(z,p,3)*p['SPEED_MAX']
                actual_yaw=decode_axis(y,p,1)*p['ACRO_TURN_RATE']*(-1 if actual_speed<0 else 1)
                requested_speed=max(-2,min(2,speed));requested_yaw=max(-14,min(14,yaw))
                self.assertLessEqual(abs(actual_speed),abs(requested_speed)+1e-8)
                self.assertLessEqual(abs(actual_yaw),abs(requested_yaw)+1e-8)
                self.assertAlmostEqual(actual_speed,requested_speed,delta=.007)
                self.assertAlmostEqual(actual_yaw,requested_yaw,delta=.6)

    def test_asymmetric_calibration_reversal_and_rcmap_are_inverted(self):
        p=parameters();p.update(RCMAP_ROLL=3,RCMAP_THROTTLE=1,RC1_TRIM=1460,RC3_TRIM=1530,RC1_REVERSED=1,RC3_REVERSED=1)
        y,z=acro_axes(1,10,p)
        self.assertAlmostEqual(decode_axis(z,p,1)*2,1,delta=.008)
        self.assertAlmostEqual(decode_axis(y,p,3)*180,10,delta=.7)
        y,z=acro_axes(0,0,p)
        self.assertEqual(decode_axis(z,p,1),0);self.assertEqual(decode_axis(y,p,3),0)

    def test_invalid_configuration_and_nonfinite_targets_are_rejected(self):
        for key,value in [('SPEED_MAX',0),('SPEED_MAX',3),('ACRO_TURN_RATE',0),('CRUISE_THROTTLE',0),('PILOT_STEER_TYPE',2),('RC3_DZ',500)]:
            p=parameters();p[key]=value
            with self.assertRaises(ValueError):validate_parameters(p)
        for speed,yaw in [(math.nan,0),(0,math.inf)]:
            with self.assertRaises(ValueError):acro_axes(speed,yaw,parameters())
        p=parameters();p.pop('RC1_MIN')
        with self.assertRaises(ValueError):validate_parameters(p)

    def bridge(self):
        b=MAVLinkBridge.__new__(MAVLinkBridge);b.master=Mock();b.target_system=1
        b._acro_parameters=parameters();b._acro_parameter_times={k:time.monotonic() for k in required_parameters(parameters())}
        b.flight_data=FlightData();b.flight_data.flight_mode='ACRO';b.flight_data.gps_fix_type=3;b.flight_data.ekf_healthy=True
        b.flight_data.position_updated_at=b.flight_data.ekf_updated_at=int(time.time()*1000)
        b._send_rover_drive=Mock();b._send_node_log=Mock()
        return b

    def test_bridge_sends_only_calibrated_axes_with_fresh_velocity_and_parameters(self):
        b=self.bridge();self.assertTrue(b._send_rover_acro(2,14))
        args=b.master.mav.manual_control_send.call_args.args
        self.assertAlmostEqual(decode_axis(args[3],b._acro_parameters,3)*2,2)
        self.assertLessEqual(decode_axis(args[2],b._acro_parameters,1)*180,14)
        b._send_rover_drive.assert_not_called()

    def test_mode_speed_feedback_or_parameter_loss_holds_neutral(self):
        for fault in ('mode','velocity','parameters'):
            b=self.bridge()
            if fault=='mode':b.flight_data.flight_mode='MANUAL'
            if fault=='velocity':b.flight_data.ekf_healthy=False
            if fault=='parameters':b._acro_parameter_times['SPEED_MAX']=time.monotonic()-40
            self.assertFalse(b._send_rover_acro(1,10))
            b.master.mav.manual_control_send.assert_not_called()
            b._send_rover_drive.assert_called_once_with(0.,0.)

    def test_zero_is_allowed_disarmed_without_velocity_for_stationary_validation(self):
        b=self.bridge();b.flight_data.ekf_healthy=False
        self.assertTrue(b._send_rover_acro(0,0))
        self.assertEqual(b.master.mav.manual_control_send.call_args.args[2:4],(0,0))

if __name__=='__main__':unittest.main()
