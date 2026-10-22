"""ACRO speed/yaw-rate requests -> calibrated MAVLink MANUAL_CONTROL axes.

Matches ArduRover 4.7.1 Mode::calc_speed_max, get_pilot_input and RC angle
dead-zone decoding. No motor PWM or flight-controller parameter is written.
"""
import math

BASE_PARAMETERS = ('PILOT_STEER_TYPE', 'SPEED_MAX', 'CRUISE_SPEED',
                   'CRUISE_THROTTLE', 'ACRO_TURN_RATE', 'RCMAP_ROLL', 'RCMAP_THROTTLE')
RC_FIELDS = ('MIN', 'MAX', 'TRIM', 'DZ', 'REVERSED')

def required_parameters(values):
    names = set(BASE_PARAMETERS)
    for key in ('RCMAP_ROLL', 'RCMAP_THROTTLE'):
        channel = values.get(key)
        if isinstance(channel, (float, int)) and math.isfinite(channel) and channel == int(channel) and 1 <= channel <= 16:
            names.update(f'RC{int(channel)}_{field}' for field in RC_FIELDS)
    return names

def validate_parameters(values, speed_limit=2.0):
    if not required_parameters(values).issubset(values):
        raise ValueError('acro_parameters_missing')
    if any(not isinstance(v, (float, int)) or not math.isfinite(v) for v in values.values()):
        raise ValueError('acro_parameters_invalid')
    if values['PILOT_STEER_TYPE'] not in (0, 1):
        raise ValueError('acro_pilot_type_unsupported')
    if not 0 < values['SPEED_MAX'] <= speed_limit or not 5 <= values['CRUISE_THROTTLE'] <= 100:
        raise ValueError('acro_speed_limit_not_configured')
    if values['CRUISE_SPEED'] <= 0 or values['ACRO_TURN_RATE'] <= 0:
        raise ValueError('acro_parameters_invalid')
    for key in ('RCMAP_ROLL', 'RCMAP_THROTTLE'):
        channel = values[key]
        if channel != int(channel) or not 1 <= channel <= 16:
            raise ValueError('acro_rcmap_invalid')
        rc = {field: values[f'RC{int(channel)}_{field}'] for field in RC_FIELDS}
        if (not 800 <= rc['MIN'] < rc['TRIM'] < rc['MAX'] <= 2200
                or not 0 <= rc['DZ'] < min(rc['TRIM']-rc['MIN'], rc['MAX']-rc['TRIM'])
                or rc['REVERSED'] not in (0, 1)):
            raise ValueError('acro_rc_calibration_invalid')

def decode_axis(axis, values, channel):
    prefix = f'RC{int(channel)}_'
    low, high, trim, dz, reverse = (values[prefix+k] for k in RC_FIELDS)
    pwm = int(low + (high-low)*(axis+1000)/2000)
    if pwm > trim+dz:
        result = (pwm-trim-dz)/(high-trim-dz)
    elif pwm < trim-dz:
        result = (pwm-trim+dz)/(trim-dz-low)
    else:
        result = 0.0
    return -result if reverse else result

def encode_axis(value, values, channel):
    prefix = f'RC{int(channel)}_'
    low, high, trim, dz, reverse = (values[prefix+k] for k in RC_FIELDS)
    physical = -value if reverse else value
    pwm = trim if physical == 0 else (trim+dz+physical*(high-trim-dz) if physical > 0
                                     else trim-dz+physical*(trim-dz-low))
    ideal = (pwm-low)*2000/(high-low)-1000
    # Quantize toward neutral: calibration rounding must never raise the speed
    # or yaw-rate request above its physical-unit limit.
    candidates = {max(-1000, min(1000, math.floor(ideal)+delta)) for delta in range(-3, 5)}
    allowed = [(axis, decode_axis(axis, values, channel)) for axis in candidates]
    allowed = [(axis, decoded) for axis, decoded in allowed
               if abs(decoded) <= abs(value)+1e-9 and (decoded*value >= 0)]
    if not allowed:
        raise ValueError('acro_rc_quantization_failed')
    return min(allowed, key=lambda item: abs(item[1]-value))[0]

def acro_axes(forward_speed_mps, yaw_rate_deg_s, values, speed_limit=2.0, yaw_limit=14.0):
    validate_parameters(values, speed_limit)
    if not math.isfinite(forward_speed_mps) or not math.isfinite(yaw_rate_deg_s):
        raise ValueError('acro_target_invalid')
    speed = max(-speed_limit, min(speed_limit, forward_speed_mps))
    yaw = max(-yaw_limit, min(yaw_limit, yaw_rate_deg_s))
    throttle = speed/values['SPEED_MAX']
    throttle = max(-1, min(1, throttle))
    steering = yaw/values['ACRO_TURN_RATE']
    steering = max(-1, min(1, steering))
    if throttle < 0:
        steering = -steering  # Mode::get_pilot_input reverses steering in reverse.
    return (encode_axis(steering, values, values['RCMAP_ROLL']),
            encode_axis(throttle, values, values['RCMAP_THROTTLE']))
