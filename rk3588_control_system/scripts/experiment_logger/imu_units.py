"""Normalize only the axes updated by this individual MAVLink sample."""
from motion_labels import number


def normalize_imu(message):
    kind = message['mavpackettype']
    if kind == 'HIGHRES_IMU':
        acc_scale, gyro_scale = 1.0, 1.0
    elif kind.startswith('SCALED_IMU'):
        acc_scale, gyro_scale = 9.80665 / 1000, .001
    else:
        # RAW_IMU units are dialect dependent; retain counts without inventing SI.
        return {}
    result = {}
    flags = (message.get('fields_updated') or 0) if kind == 'HIGHRES_IMU' else None
    for index, (axis, field) in enumerate(zip('xyz', ['ax_m_s2', 'ay_m_s2', 'az_m_s2'])):
        value = number(message.get(axis + 'acc'))
        result[field] = value * acc_scale if value is not None and (flags is None or flags & (1 << index)) else None
    for index, (axis, field) in enumerate(zip('xyz', ['gx_rad_s', 'gy_rad_s', 'gz_rad_s'])):
        value = number(message.get(axis + 'gyro'))
        result[field] = value * gyro_scale if value is not None and (flags is None or flags & (1 << (index + 3))) else None
    return result
