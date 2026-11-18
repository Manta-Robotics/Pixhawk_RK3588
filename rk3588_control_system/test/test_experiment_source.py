import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from experiment_tap import canonical_sample


def sample(kind, fields, calibration=None, autopilot=True, channels=(1, 3)):
    return canonical_sample({
        'autopilot':autopilot, 'sample_id':'mav/1', 'obs_ns':'1000000000', 'boot_id':'test-boot',
        'message':dict(mavpackettype=kind, **fields)
    }, channels, calibration or {})


class SourceTests(unittest.TestCase):
    def test_axis_transform_and_highres_flags(self):
        self.assertEqual(sample('RAW_IMU',{'xacc':200})['quality'],'invalid')
        fields={'xacc':1, 'yacc':2, 'zacc':3, 'xgyro':0, 'fields_updated':15}
        actual=sample('HIGHRES_IMU',fields,{'imu_frd_axes':[-2,1,3]})
        self.assertEqual(actual['values']['ax_m_s2'],-2)
        self.assertEqual(actual['values']['gy_rad_s'],0)
        self.assertIsNone(actual['values']['gx_rad_s'])
        self.assertAlmostEqual(math.sqrt(14),actual['values']['acc_norm_m_s2'])
        unknown=sample('HIGHRES_IMU',fields)
        self.assertEqual(unknown['meta']['normalized_sensor']['ax_m_s2'],1)
        self.assertNotIn('ax_m_s2',unknown['values'])
    def test_channel_port_and_optional_rotor_units(self):
        fields={'port':1,'servo1_raw':0,'servo2_raw':1510}
        self.assertIsNone(sample('SERVO_OUTPUT_RAW',fields,autopilot=False))
        self.assertEqual(sample('SERVO_OUTPUT_RAW',fields)['quality'],'invalid')
        paired=sample('SERVO_OUTPUT_RAW',fields,channels=(9,10))
        self.assertEqual(paired['values'],{'left_pwm_us':None,'right_pwm_us':1510})
        esc={'rpm':[0,200,400,600]}
        self.assertEqual(sample('ESC_TELEMETRY_1_TO_4',esc,{'esc_rpm_units':'erpm'})['values'],{'left_erpm':0,'right_erpm':400})
        self.assertEqual(sample('ESC_TELEMETRY_1_TO_4',esc,{'esc_rpm_units':'erpm','pole_pairs':{'right':4}})['values']['right_rpm'],100)
    def test_heading_and_ground_speed_are_distinct(self):
        self.assertEqual(sample('ATTITUDE',{'yaw':0,'yawspeed':0})['values']['heading_deg'],0)
        self.assertEqual(sample('GLOBAL_POSITION_INT',{'vx':3,'vy':4,'hdg':9000})['values'],{'ground_speed_m_s':0.05})
