"""Nonblocking, receive-only copy of selected MAVLink messages to the logger."""
import json
import socket
import time

_socket = None
_sequence = 0
_drops = 0
TYPES = {'HEARTBEAT', 'ATTITUDE', 'ATTITUDE_QUATERNION', 'HIGHRES_IMU',
         'RAW_IMU', 'SCALED_IMU', 'SCALED_IMU2', 'SCALED_IMU3',
         'SERVO_OUTPUT_RAW', 'SYS_STATUS', 'BATTERY_STATUS', 'GLOBAL_POSITION_INT',
         'GPS_RAW_INT', 'NAMED_VALUE_FLOAT'}


def record_mavlink(msg, system_id, component_id):
    global _socket, _sequence, _drops
    try:
        kind = msg.get_type()
        if kind not in TYPES and not kind.startswith('ESC_TELEMETRY_'):
            return
        if msg.get_srcSystem() != system_id:
            return
        if _socket is None:
            _socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            _socket.setblocking(False)
        _sequence += 1
        packet = {'kind': 'mavlink', 'sequence': _sequence, 'sender_drops': _drops,
                  'wall_ns': time.time_ns(), 'mono_ns': time.monotonic_ns(),
                  'system_id': msg.get_srcSystem(), 'component_id': msg.get_srcComponent(),
                  'autopilot': msg.get_srcComponent() == component_id,
                  'message': msg.to_dict()}
        raw = json.dumps(packet, separators=(',', ':'), default=lambda x: list(x) if isinstance(x, bytes) else str(x)).encode()
        _socket.sendto(raw, ('127.0.0.1', 14660))
    except Exception:
        _drops += 1
