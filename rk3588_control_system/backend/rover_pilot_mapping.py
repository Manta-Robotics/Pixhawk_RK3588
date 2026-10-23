"""Convert desired Rover mixer axes to the FCU's pilot input convention."""
import math


def motor_target_axes(steering_axis, throttle_axis, pilot_steer_type):
    if not steering_axis and not throttle_axis:
        return 0, 0
    if pilot_steer_type not in (0, 1, 2, 3):
        raise ValueError('PILOT_STEER_TYPE is unknown or unsupported')
    if pilot_steer_type in (0, 2) and throttle_axis < 0:
        steering_axis = -steering_axis
    elif pilot_steer_type == 1:
        steering_axis, throttle_axis = throttle_axis + steering_axis, throttle_axis - steering_axis
    return tuple(max(-1000, min(1000, round(value))) for value in (steering_axis, throttle_axis))


def valid_pilot_type(value):
    try:
        value = float(value)
        return int(value) if math.isfinite(value) and value in (0, 1, 2, 3) else None
    except (TypeError, ValueError):
        return None
