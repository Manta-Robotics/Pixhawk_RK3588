"""Nonblocking raw diagnostic copy and canonical source stamps at MAVLink ingress."""
import json
import math
import os
import re
import socket
import sys
import time
import uuid
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1] / "scripts" / "experiment_logger"))
from imu_units import normalize_imu

_socket = None
_sequence = 0
_drops = 0
_instance = uuid.uuid4().hex
try:
    BOOT = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
except OSError:
    BOOT = None
TYPES = {'HEARTBEAT', 'ATTITUDE', 'ATTITUDE_QUATERNION', 'HIGHRES_IMU',
         'RAW_IMU', 'SCALED_IMU', 'SCALED_IMU2', 'SCALED_IMU3',
         'SERVO_OUTPUT_RAW', 'SYS_STATUS', 'BATTERY_STATUS', 'GLOBAL_POSITION_INT',
         'GPS_RAW_INT', 'NAMED_VALUE_FLOAT'}


def finite(value):
    return value if isinstance(value, (int, float)) and math.isfinite(value) else None


def record_mavlink(msg, system_id, component_id):
    global _socket, _sequence, _drops
    try:
        kind = msg.get_type()
        if kind not in TYPES and not kind.startswith('ESC_TELEMETRY_'):
            return None
        if msg.get_srcSystem() != system_id:
            return None
        at = time.monotonic_ns()
        _sequence += 1
        packet = {'kind':'mavlink', 'sequence':_sequence, 'sender_drops':_drops,
                  'wall_ns':time.time_ns(), 'mono_ns':at, 'boot_id':BOOT,
                  'sample_id':f'{_instance}/{_sequence}', 'obs_ns':str(at),
                  'system_id':msg.get_srcSystem(), 'component_id':msg.get_srcComponent(),
                  'autopilot':msg.get_srcComponent() == component_id, 'message':msg.to_dict()}
    except (AttributeError, TypeError, ValueError):
        return None
    try:
        if _socket is None:
            _socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            _socket.setblocking(False)
        raw = json.dumps(packet, separators=(',', ':'), default=lambda x: list(x) if isinstance(x, bytes) else str(x)).encode()
        _socket.sendto(raw, ('127.0.0.1',int(os.environ.get('MANTA_EXPERIMENT_RAW_PORT', 14660))))
    except (OSError, ValueError, TypeError):
        _drops += 1
    return packet


def canonical_sample(packet, channels, calibration, flight_mode=None):
    if not packet or not packet['autopilot']:
        return None
    m = packet['message']
    kind = m['mavpackettype']
    meta = {'message_type': kind, 'device_time': m.get('time_usec', m.get('time_boot_ms'))}
    values = {}
    if kind == 'HEARTBEAT':
        if flight_mode is None:
            return None
        source = 'fc'
        values = {'fc_mode':flight_mode, 'armed': bool(m['base_mode'] & 128)}
    elif kind == 'ATTITUDE':
        source = 'att'
        yaw, rate = finite(m.get('yaw')), finite(m.get('yawspeed'))
        values = {'heading_deg':math.degrees(yaw)%360 if yaw is not None else None,
                  'yaw_rate_deg_s':math.degrees(rate) if rate is not None else None}
    elif kind == 'GLOBAL_POSITION_INT':
        source = 'nav'
        x,y = finite(m.get('vx')),finite(m.get('vy'))
        values['ground_speed_m_s'] = math.hypot(x,y)/100 if None not in (x,y) else None
    elif kind.endswith('_IMU') or kind.startswith('SCALED_IMU'):
        source = 'imu'
        meta.update(sensor_id=m.get('id',kind), axes='sensor',raw_units={'RAW_IMU':'counts','HIGHRES_IMU':'m/s2,rad/s'}.get(kind,'mG,mrad/s'))
        normalized = normalize_imu(m)
        axes = calibration.get('imu_frd_axes')
        # Signed permutation from sensor XYZ into vessel forward/right/down.
        if isinstance(axes,list) and len(axes)==3 and all(type(i) is int for i in axes) and sorted(map(abs,axes))==[1,2,3]:
            meta.update(axes='FRD',installation_transform=axes)
            for fields in [('ax_m_s2','ay_m_s2','az_m_s2'),('gx_rad_s','gy_rad_s','gz_rad_s')]:
                for key,i in zip(fields,axes):
                    v=normalized.get(fields[abs(i)-1])
                    values[key]=v*(1 if i>0 else -1) if v is not None else None
            xyz=[values[k] for k in ('ax_m_s2','ay_m_s2','az_m_s2')]
            values['acc_norm_m_s2']=math.hypot(*xyz) if None not in xyz else None
        else:
            meta.update(normalized_sensor=normalized,installation_transform=None)
    elif kind == 'SERVO_OUTPUT_RAW':
        source = 'pwm'
        for side,ch in zip(('left','right'),channels):
            if (ch-1)//8!=m.get('port'):
                continue
            v=finite(m.get(f'servo{(ch-1)%8+1}_raw'))
            values[side+'_pwm_us']=v if v is not None and 500<=v<=2500 else None
    elif kind.startswith('ESC_TELEMETRY_'):
        source='esc'
        group=re.fullmatch(r'ESC_TELEMETRY_(\d+)_TO_(\d+)',kind)
        if not group:return None
        units=calibration.get('esc_rpm_units')
        for side,ch in zip(('left','right'),channels):
            idx=ch-int(group[1]);rpm=m.get('rpm',[])
            if not 0<=idx<len(rpm):continue
            value=finite(rpm[idx])
            meta[side+'_raw_rpm']=value
            poles=calibration.get('pole_pairs',{}).get(side)
            if units=='erpm' and value is not None:
                values[side+'_erpm']=value
                if isinstance(poles,int) and poles>0:values[side+'_rpm']=value/poles
            elif units=='mechanical':values[side+'_rpm']=value
    else:
        return None
    return {'source':source, 'sample_id':packet['sample_id'],'obs_ns':packet['obs_ns'],'avail_ns':None,
            'boot_id':packet['boot_id'],'time_kind':'host_receive','quality':'measured' if (any(v is not None for v in values.values()) or
                any(v is not None for v in meta.get('normalized_sensor', {}).values())) else 'invalid',
            'values':values,'meta':meta}
