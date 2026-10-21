#!/usr/bin/env python3
"""
MAVLink bridge for Pixhawk communication.

Responsibilities:
1) Connect to Pixhawk over serial using pymavlink.
2) Receive control commands from Node.js over local UDP.
3) Send telemetry to Node.js over local UDP.
"""

import json
import logging
import math
import os
import socket
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

from uwb_fusion import DualUwbFusion
from uwb_measurement_filter import GatedUwbKalman, DEFAULTS as UWB_FILTER_DEFAULTS
from rover_pilot_mapping import motor_target_axes, valid_pilot_type
from rover_acro_mapping import required_parameters, validate_parameters, acro_axes

try:
    from pymavlink import mavutil
except ImportError:
    print('ERROR: pymavlink is not installed. Run: python3 -m pip install -r requirements.txt')
    sys.exit(1)

BASE_DIR = Path(__file__).resolve().parent.parent
CONFIG_PATH = Path(os.environ.get('MANTA_CONFIG_FILE', BASE_DIR / 'config' / 'system.config.json'))
LOGS_DIR = Path(os.environ.get('MANTA_LOGS_DIR', BASE_DIR / 'logs'))
LOGS_DIR.mkdir(parents=True, exist_ok=True)
MAVLINK_LOG_PATH = LOGS_DIR / 'mavlink.log'

_log_handlers = [logging.StreamHandler()]
if not os.environ.get('INVOCATION_ID'):
    try:
        _log_handlers.append(logging.FileHandler(MAVLINK_LOG_PATH))
    except PermissionError:
        pass

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [%(levelname)s] %(message)s',
    handlers=_log_handlers,
)
logger = logging.getLogger('mavlink-bridge')
_NODE_ALERT_KEYS_SEEN = set()

ACCELCAL_POSITION_LABELS = {
    1: 'LEVEL',
    2: 'LEFT',
    3: 'RIGHT',
    4: 'NOSEDOWN',
    5: 'NOSEUP',
    6: 'BACK',
}
ACCELCAL_POSITION_HINTS = {
    1: 'Place the rover level and keep it still, then confirm the pose in the UI.',
    2: 'Place the rover with the left side down, keep it still, then confirm the pose in the UI.',
    3: 'Place the rover with the right side down, keep it still, then confirm the pose in the UI.',
    4: 'Place the rover nose down and vertical, then confirm the pose in the UI.',
    5: 'Place the rover nose up and vertical, then confirm the pose in the UI.',
    6: 'Place the rover tail down and vertical, then confirm the pose in the UI.',
}
ACCELCAL_POSITION_SUCCESS = 16777215
ACCELCAL_POSITION_FAILED = 16777216
MAV_CMD_PREFLIGHT_CALIBRATION = getattr(mavutil.mavlink, 'MAV_CMD_PREFLIGHT_CALIBRATION', 241)
MAV_CMD_ACCELCAL_VEHICLE_POS = getattr(mavutil.mavlink, 'MAV_CMD_ACCELCAL_VEHICLE_POS', 42429)
MAV_CMD_SET_MESSAGE_INTERVAL = getattr(mavutil.mavlink, 'MAV_CMD_SET_MESSAGE_INTERVAL', 511)
MAV_RESULT_ACCEPTED = getattr(mavutil.mavlink, 'MAV_RESULT_ACCEPTED', 0)
MAV_RESULT_IN_PROGRESS = getattr(mavutil.mavlink, 'MAV_RESULT_IN_PROGRESS', 5)
MAV_RESULT_CANCELLED = getattr(mavutil.mavlink, 'MAV_RESULT_CANCELLED', 6)


def load_config() -> Dict[str, Any]:
    if not CONFIG_PATH.exists():
        logger.warning('Config file not found at %s, using defaults', CONFIG_PATH)
        return {}

    try:
        return json.loads(CONFIG_PATH.read_text(encoding='utf-8'))
    except Exception as exc:
        logger.error('Failed to read config: %s', exc)
        return {}


def rover_manual_control_axes(
    throttle: float,
    steering: float,
    throttle_min: float = -100.0,
    throttle_max: float = 100.0,
    steering_min: float = -45.0,
    steering_max: float = 45.0,
) -> tuple[int, int]:
    """Convert Manta commands to ArduRover's centered y/z axes (neutral=0)."""
    if throttle_max <= throttle_min or steering_max <= steering_min:
        raise ValueError('Rover control ranges must have a positive span')

    clamped_throttle = max(throttle_min, min(throttle_max, float(throttle)))
    clamped_steering = max(steering_min, min(steering_max, float(steering)))
    throttle_axis = round(-1000.0 + (clamped_throttle - throttle_min) * 2000.0 / (throttle_max - throttle_min))
    steering_axis = round(-1000.0 + (clamped_steering - steering_min) * 2000.0 / (steering_max - steering_min))
    return int(max(-1000, min(1000, steering_axis))), int(max(-1000, min(1000, throttle_axis)))


def is_vehicle_heartbeat(msg: Any) -> bool:
    """Reject GCS heartbeats that may be routed through the flight controller."""
    if msg is None or msg.get_type() != 'HEARTBEAT':
        return False
    vehicle_type = int(getattr(msg, 'type', mavutil.mavlink.MAV_TYPE_GCS))
    autopilot_type = int(getattr(msg, 'autopilot', mavutil.mavlink.MAV_AUTOPILOT_INVALID))
    return (
        vehicle_type != mavutil.mavlink.MAV_TYPE_GCS
        and autopilot_type != mavutil.mavlink.MAV_AUTOPILOT_INVALID
    )


def is_target_fcu_heartbeat(msg: Any, target_system: int, target_component: int) -> bool:
    if not is_vehicle_heartbeat(msg):
        return False
    return (
        int(msg.get_srcSystem()) == int(target_system)
        and int(msg.get_srcComponent()) == int(target_component)
    )


@dataclass
class FlightData:
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    yaw_rate_dps: float = 0.0
    attitude_updated_at: Optional[int] = None
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    battery_voltage: float = 0.0
    battery_current: float = 0.0
    battery_percentage: float = 100.0
    servo_outputs: Dict[str, int] = field(default_factory=lambda: {
        'ch1': 0,
        'ch2': 0,
        'ch3': 0,
        'ch4': 0,
    })
    servo_outputs_updated_at: Optional[int] = None
    board_temperature: Optional[float] = None
    motor_left_temperature: Optional[float] = None
    motor_right_temperature: Optional[float] = None
    motor_left_esc_updated_at: Optional[int] = None
    motor_right_esc_updated_at: Optional[int] = None
    gps_satellites: int = 0
    gps_hdop: float = 999.0
    gps_fix_type: int = 0
    gps_latitude: float = 0.0
    gps_longitude: float = 0.0
    gps_altitude: float = 0.0
    gps_horizontal_accuracy: Optional[float] = None
    gps_vertical_accuracy: Optional[float] = None
    gps_ground_speed: Optional[float] = None
    gps_course: Optional[float] = None
    gps_updated_at: Optional[int] = None
    position_source: str = ''
    position_updated_at: Optional[int] = None
    ekf_flags: int = 0
    ekf_healthy: bool = False
    ekf_velocity_variance: Optional[float] = None
    ekf_pos_horiz_variance: Optional[float] = None
    ekf_compass_variance: Optional[float] = None
    ekf_updated_at: Optional[int] = None
    flight_mode: str = 'STABILIZE'
    system_status: str = 'STANDBY'
    armed: bool = False
    updated_at: float = field(default_factory=time.time)


@dataclass
class UWBData:
    raw_distance_m: Optional[float] = None
    raw_azimuth_deg: Optional[float] = None
    raw_elevation_deg: Optional[float] = None
    distance_m: Optional[float] = None
    azimuth_deg: Optional[float] = None
    elevation_deg: Optional[float] = None
    good_count: int = 0
    updated_at: Optional[int] = None


class ScalarKalmanFilter:
    """Small scalar Kalman filter with optional circular angle residuals."""

    def __init__(self, process_noise: float, measurement_noise: float, angle: bool = False) -> None:
        self.process_noise = max(1e-9, float(process_noise))
        self.measurement_noise = max(1e-9, float(measurement_noise))
        self.angle = angle
        self.estimate: Optional[float] = None
        self.uncertainty = 1.0

    @staticmethod
    def _wrap_angle(value: float) -> float:
        return (value + 180.0) % 360.0 - 180.0

    def update(self, measurement: float) -> float:
        measurement = float(measurement)
        if self.estimate is None:
            self.estimate = self._wrap_angle(measurement) if self.angle else measurement
            return self.estimate

        predicted_uncertainty = self.uncertainty + self.process_noise
        gain = predicted_uncertainty / (predicted_uncertainty + self.measurement_noise)
        residual = measurement - self.estimate
        if self.angle:
            residual = self._wrap_angle(residual)
        self.estimate += gain * residual
        if self.angle:
            self.estimate = self._wrap_angle(self.estimate)
        self.uncertainty = (1.0 - gain) * predicted_uncertainty
        return self.estimate


@dataclass
class IMUCalibrationState:
    active: bool = False
    mode: str = 'IDLE'
    status: str = 'IDLE'
    step: str = ''
    step_code: Optional[int] = None
    instructions: str = 'Idle'
    progress: Optional[int] = None
    last_ack_command: Optional[int] = None
    last_ack_result: str = ''
    updated_at: float = field(default_factory=time.time)


class MAVLinkBridge:
    def __init__(
        self,
        serial_port: str,
        baudrate: int,
        bridge_host: str,
        command_port: int,
        telemetry_port: int,
        telemetry_rate_hz: int,
        heartbeat_interval: float,
        rover_left_output_channel: int,
        rover_right_output_channel: int,
        rover_throttle_min: float,
        rover_throttle_max: float,
        rover_steering_min: float,
        rover_steering_max: float,
        uwb_dual_config: Optional[Dict[str, Any]] = None,
        mission_planner_tcp_config: Optional[Dict[str, Any]] = None,
        uwb_measurement_filter_config: Optional[Dict[str, Any]] = None,
        uwb_bearing_config: Optional[Dict[str, Any]] = None,
    ) -> None:
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.bridge_host = bridge_host
        self.command_port = command_port
        self.telemetry_port = telemetry_port
        self.telemetry_interval = 1.0 / max(1, telemetry_rate_hz)
        self.heartbeat_interval = max(0.2, heartbeat_interval)
        self.rover_left_output_channel = max(1, min(32, int(rover_left_output_channel)))
        self.rover_right_output_channel = max(1, min(32, int(rover_right_output_channel)))
        self.rover_throttle_min = float(rover_throttle_min)
        self.rover_throttle_max = float(rover_throttle_max)
        self.rover_steering_min = float(rover_steering_min)
        self.rover_steering_max = float(rover_steering_max)
        uwb_dual_config = uwb_dual_config or {}
        self.uwb_filter_config = {**UWB_FILTER_DEFAULTS, **(uwb_measurement_filter_config or {})}
        self.uwb_filter_enabled = self.uwb_filter_config["enabled"] is True
        uwb_bearing_config = uwb_bearing_config or {}
        mission_planner_tcp_config = mission_planner_tcp_config or {}

        self.master: Optional[mavutil.mavfile] = None
        self.connected = False
        self.target_system = 1
        self.target_component = 1
        self.pilot_steer_type = None
        self._pilot_param_requested_at = 0.0
        self._acro_parameters = {}
        self._acro_parameter_times = {}
        self._acro_requests = {}
        self._acro_requested_at = 0.0
        self.flight_data = FlightData()
        self.uwb_data = UWBData()
        self.uwb_fusion = DualUwbFusion(
            mount_yaw_deg=uwb_dual_config.get('mount_yaw_deg', (0.0, 90.0)),
            freshness_ms=uwb_dual_config.get('freshness_ms', 1500),
            receiver_fov_half_deg=uwb_dual_config.get('receiver_fov_half_deg', 70),
            overlap_disagreement_deg=uwb_dual_config.get('overlap_disagreement_deg', 30),
            overlap_position_disagreement_m=uwb_dual_config.get('overlap_position_disagreement_m', 0.45),
            receiver_position_x_m=uwb_dual_config.get('position_x_m', (0.0, 0.0)),
            receiver_position_y_m=uwb_dual_config.get('position_y_m', (0.0, 0.0)),
            minimum_distance_m=uwb_dual_config.get('minimum_distance_m', 0.15),
            maximum_distance_m=uwb_dual_config.get('maximum_distance_m', 20.0),
            switch_confidence_ratio=uwb_dual_config.get('switch_confidence_ratio', 1.35),
            switch_confirm_ms=uwb_dual_config.get('switch_confirm_ms', 180),
            switch_blend_ms=uwb_dual_config.get('switch_blend_ms', 180),
            enabled_receiver_ids=uwb_dual_config.get('enabled_receiver_ids', (1, 2)),
            require_gated_measurements=self.uwb_filter_enabled,
        )
        self._uwb_measurement_filters = {
            receiver_id: GatedUwbKalman(
                self.uwb_filter_config,
                minimum_distance_m=self.uwb_fusion.minimum_distance_m,
                maximum_distance_m=self.uwb_fusion.maximum_distance_m,
                bearing_sign=uwb_bearing_config.get('yaw_source_sign', -1),
                bearing_scale=uwb_bearing_config.get('yaw_source_scale', 0.8148),
                bearing_offset_deg=uwb_bearing_config.get('yaw_offset_deg', -4.816),
                mount_yaw_deg=receiver.mount_yaw_deg,
            ) for receiver_id, receiver in self.uwb_fusion.receivers.items()
        }
        self.imu_calibration = IMUCalibrationState()
        self._uwb_filters = {
            (receiver_id, field_name): ScalarKalmanFilter(
                process_noise=0.04 if field_name == 'DIST' else 3.0,
                measurement_noise=0.09 if field_name == 'DIST' else 6.0,
                angle=field_name == 'AZ',
            )
            for receiver_id in (1, 2)
            for field_name in ('DIST', 'AZ', 'EL')
        }

        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_socket.bind((self.bridge_host, self.command_port))
        self.command_socket.setblocking(False)

        self.telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.node_addr = ('127.0.0.1', self.telemetry_port)

        self.mission_planner_tcp_enabled = bool(mission_planner_tcp_config.get('enabled', False))
        self.mission_planner_tcp_host = str(mission_planner_tcp_config.get('host', self.bridge_host))
        self.mission_planner_tcp_port = int(mission_planner_tcp_config.get('port', 5760))
        self.mission_planner_allowed_clients = {
            str(address) for address in mission_planner_tcp_config.get('allowed_clients', []) if address
        }
        self.mission_planner_server: Optional[socket.socket] = None
        self.mission_planner_clients: list[socket.socket] = []
        self._mission_planner_tx: Dict[socket.socket, bytearray] = {}
        if self.mission_planner_tcp_enabled:
            self.mission_planner_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.mission_planner_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.mission_planner_server.bind((self.mission_planner_tcp_host, self.mission_planner_tcp_port))
            self.mission_planner_server.listen(2)
            self.mission_planner_server.setblocking(False)

        self._last_telemetry_sent = 0.0
        self._last_heartbeat_sent = 0.0
        self._last_fcu_heartbeat_at = 0.0
        self._last_mode_reported = ''
        self._last_armed_reported: Optional[bool] = None
        self._last_system_status_reported = ''
        self._last_battery_state = 'UNKNOWN'
        self._last_gps_fix_reported: Optional[bool] = None
        self._last_log_sent_at: Dict[str, float] = {}

    def _param_name(self, raw_name: Any) -> str:
        if isinstance(raw_name, bytes):
            return raw_name.decode('utf-8', 'ignore').rstrip('\x00')
        return str(raw_name).rstrip('\x00')

    def _update_uwb_named_value(
        self,
        name: str,
        value: float,
        source: Optional[tuple[int, int]] = None,
    ) -> None:
        now_ms = int(time.time() * 1000)
        updated = self.uwb_fusion.update(name, value, source=source, now_ms=now_ms)
        if updated is None:
            parsed = self.uwb_fusion.parse_name(name)
            if self.uwb_filter_enabled and parsed and parsed[1] in ('DIST', 'AZ', 'EL') and not math.isfinite(value):
                receiver_id = parsed[0] or self.uwb_fusion._legacy_receiver(source)
                result = self._uwb_measurement_filters[receiver_id].reject(now_ms, 'non_finite_measurement')
                self.uwb_fusion.apply_measurement_result(receiver_id, result)
                self._refresh_uwb_fusion()
            return
        receiver_id, field_name = updated
        if self.uwb_filter_enabled:
            receiver = self.uwb_fusion.receivers[receiver_id]
            kalman = self._uwb_measurement_filters[receiver_id]
            heading_at = self.flight_data.attitude_updated_at
            heading = self.flight_data.yaw if heading_at is not None and 0 <= now_ms - heading_at <= 700 else None
            if field_name == 'REJECTED':
                self.uwb_fusion.apply_measurement_result(receiver_id, kalman.reject(now_ms, receiver.invalid_reason, heading))
            elif field_name == 'VALID' and receiver.valid is False:
                self.uwb_fusion.apply_measurement_result(receiver_id, kalman.reject(now_ms, 'radio_invalid', heading))
            elif field_name in ('DIST', 'AZ') and receiver.received_measurement_at == now_ms and not receiver.pending_distance and not receiver.pending_azimuth:
                result = kalman.update(receiver.raw_distance_m, receiver.raw_azimuth_deg, now_ms, heading,
                                       valid=receiver.valid, sequence_id=receiver.sequence_id, tag_id=receiver.tag_id)
                self.uwb_fusion.apply_measurement_result(receiver_id, result)
            elif field_name == 'EL' and abs(value) <= 90:
                # Elevation is separate from the horizontal Cartesian state.
                self.uwb_fusion.set_filtered(receiver_id, 'EL', value)
            self._refresh_uwb_fusion()
            return
        if field_name in {'GOOD', 'VALID', 'SEQ', 'ANCH', 'TAG', 'STAT', 'RX', 'HB', 'CRC', 'LEN', 'REJECTED'}:
            self._refresh_uwb_fusion()
            return
        filtered = self._uwb_filters[(receiver_id, field_name)].update(value)
        self.uwb_fusion.set_filtered(receiver_id, field_name, filtered)
        self._refresh_uwb_fusion()

    def _refresh_uwb_fusion(self) -> None:
        now_ms = int(time.time() * 1000)
        if self.uwb_filter_enabled:
            for receiver_id, kalman in self._uwb_measurement_filters.items():
                self.uwb_fusion.receivers[receiver_id].filter_uncertainty_m = kalman.result(now_ms)['uncertainty_m']
        fused = self.uwb_fusion.fused(now_ms)
        self.uwb_data.distance_m = fused['distance_m']
        self.uwb_data.azimuth_deg = fused['azimuth_deg']
        self.uwb_data.elevation_deg = fused['elevation_deg']
        self.uwb_data.good_count = fused['good_count']
        self.uwb_data.updated_at = fused['updated_at']
        active = [sensor for sensor in fused['sensors'] if sensor['rawDistanceM'] is not None and sensor['rawAzimuthDeg'] is not None]
        if active:
            newest = max(active, key=lambda sensor: sensor.get('receivedMeasurementAt') or sensor['updatedAt'] or 0)
            self.uwb_data.raw_distance_m = newest['rawDistanceM']
            self.uwb_data.raw_azimuth_deg = newest['rawAzimuthDeg']
            self.uwb_data.raw_elevation_deg = newest['rawElevationDeg']

    def _set_message_interval(self, message_id: int, rate_hz: float,
                              ack_timeout: float = 0.8, attempts: int = 2) -> bool:
        """Serialize interval requests: ACK 511 does not identify its message ID.

        Keep parsing telemetry while waiting, so startup cannot drop a heartbeat
        or a complete UWB observation. This runs before the normal receive loop.
        """
        if self.master is None or rate_hz <= 0:
            return False

        interval_us = max(1000, int(round(1_000_000 / rate_hz)))

        for attempt in range(attempts):
            try:
                self.master.mav.command_long_send(
                    self.target_system, self.target_component,
                    MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    float(message_id), float(interval_us), 0, 0, 0, 0, 0,
                )
                deadline = time.monotonic() + ack_timeout
                while time.monotonic() < deadline:
                    msg = self.master.recv_match(blocking=True, timeout=min(0.05, max(0, deadline - time.monotonic())))
                    if msg is None:
                        continue
                    self._broadcast_mission_planner(msg)
                    self._parse_message(msg)
                    if (msg.get_type() != 'COMMAND_ACK'
                            or msg.get_srcSystem() != self.target_system
                            or msg.get_srcComponent() != self.target_component
                            or int(getattr(msg, 'command', -1)) != MAV_CMD_SET_MESSAGE_INTERVAL):
                        continue
                    result = int(getattr(msg, 'result', -1))
                    if result == MAV_RESULT_IN_PROGRESS:
                        continue
                    if result == MAV_RESULT_ACCEPTED:
                        logger.info('FCU accepted telemetry interval: message=%d requested=%.1f Hz', message_id, rate_hz)
                        return True
                    logger.warning('FCU rejected telemetry interval: message=%d result=%s',
                                   message_id, self._mav_result_name(result))
                    # A temporary rejection may be retried; denied/unsupported
                    # messages must not block the rest of telemetry setup.
                    if result != getattr(mavutil.mavlink, 'MAV_RESULT_TEMPORARILY_REJECTED', 1):
                        return False
                    break
                else:
                    logger.warning('Telemetry interval ACK timeout: message=%d attempt=%d', message_id, attempt + 1)
            except Exception as exc:
                logger.warning('Failed to set message interval for %s: %s', message_id, exc)
        return False

    def _configure_message_intervals(self) -> None:
        rates = {
            getattr(mavutil.mavlink, 'MAVLINK_MSG_ID_ATTITUDE', 30): 20.0,
            getattr(mavutil.mavlink, 'MAVLINK_MSG_ID_SERVO_OUTPUT_RAW', 36): 20.0,
            getattr(mavutil.mavlink, 'MAVLINK_MSG_ID_GLOBAL_POSITION_INT', 33): 10.0,
            getattr(mavutil.mavlink, 'MAVLINK_MSG_ID_GPS_RAW_INT', 24): 5.0,
            getattr(mavutil.mavlink, 'MAVLINK_MSG_ID_SYS_STATUS', 1): 5.0,
            getattr(mavutil.mavlink, 'MAVLINK_MSG_ID_EKF_STATUS_REPORT', 193): 5.0,
            getattr(mavutil.mavlink, 'MAVLINK_MSG_ID_HIGHRES_IMU', 105): 10.0,
        }

        esc_groups = {
            (self.rover_left_output_channel - 1) // 4,
            (self.rover_right_output_channel - 1) // 4,
        }
        for group in esc_groups:
            message_name = f'MAVLINK_MSG_ID_ESC_TELEMETRY_{group * 4 + 1}_TO_{group * 4 + 4}'
            message_id = getattr(mavutil.mavlink, message_name, None)
            if message_id is not None:
                rates[int(message_id)] = 5.0

        accepted = sum(self._set_message_interval(int(message_id), float(rate_hz))
                       for message_id, rate_hz in rates.items())
        logger.info('FCU telemetry interval setup: %d/%d requests accepted', accepted, len(rates))

    def connect(self, timeout: float = 8.0) -> bool:
        try:
            logger.info('Connecting to Pixhawk on %s @ %d', self.serial_port, self.baudrate)
            self.master = mavutil.mavlink_connection(
                self.serial_port,
                baud=self.baudrate,
                autoreconnect=True,
                source_system=255,
            )

            deadline = time.monotonic() + timeout
            hb = None
            while time.monotonic() < deadline:
                candidate = self.master.recv_match(
                    type='HEARTBEAT',
                    blocking=True,
                    timeout=max(0.1, deadline - time.monotonic()),
                )
                if is_vehicle_heartbeat(candidate):
                    hb = candidate
                    break
            if hb is None:
                logger.error('Heartbeat timeout during connect')
                self._send_node_log(
                    'WARNING',
                    f'FCU heartbeat timeout on {self.serial_port} @ {self.baudrate}',
                    key='connect-heartbeat-timeout',
                    min_interval=2.0,
                )
                self.connected = False
                return False

            self.target_system = int(getattr(hb, 'srcSystem', 1) or 1)
            self.target_component = int(getattr(hb, 'srcComponent', 1) or 1)
            self.connected = True
            self.pilot_steer_type = None
            self._pilot_param_requested_at = 0.0
            self._acro_parameters.clear()
            self._acro_parameter_times.clear()
            self._acro_requests.clear()
            self._last_fcu_heartbeat_at = time.time()

            logger.info(
                'Connected to Pixhawk (target system=%d, component=%d)',
                self.target_system,
                self.target_component,
            )
            self._send_node_log(
                'INFO',
                f'Connected to Pixhawk on {self.serial_port} @ {self.baudrate}',
                key='connect-success',
                min_interval=1.0,
            )
            logger.info(
                'Rover MANUAL_CONTROL ready; expected Pixhawk mixer outputs: SERVO%d_FUNCTION=73, SERVO%d_FUNCTION=74',
                self.rover_left_output_channel,
                self.rover_right_output_channel,
            )
            self._configure_message_intervals()
            self._last_fcu_heartbeat_at = time.time()
            self._send_node_packet('connection', {'connected': True})
            return True
        except Exception as exc:
            logger.error('Connection failed: %s', exc)
            if self.serial_port.startswith('/dev/') and 'No such file or directory' in str(exc):
                logger.error(
                    'Serial device %s not found. Check system.config.json serial_port and enable 40-pin UART overlay in /boot/firmware/ubuntuEnv.txt (e.g. rk3588-lubancat-uart3-m0-overlay), then reboot.',
                    self.serial_port,
                )
                self._send_node_log(
                    'ERROR',
                    f'Serial device not found: {self.serial_port}',
                    key='connect-serial-missing',
                    min_interval=2.0,
                )
            else:
                self._send_node_log(
                    'ERROR',
                    f'Bridge connect failed on {self.serial_port}: {exc}',
                    key='connect-failed',
                    min_interval=2.0,
                )
            self.connected = False
            return False

    def disconnect(self) -> None:
        if self.connected:
            self._send_node_packet('connection', {'connected': False})

        self.connected = False
        try:
            if self.master is not None:
                self.master.close()
        except Exception:
            pass

        try:
            self.command_socket.close()
        except Exception:
            pass

        try:
            self.telemetry_socket.close()
        except Exception:
            pass

        for client in list(self.mission_planner_clients):
            self._close_mission_planner_client(client)
        try:
            if self.mission_planner_server is not None:
                self.mission_planner_server.close()
        except Exception:
            pass

        logger.info('Bridge disconnected')

    def _close_mission_planner_client(self, client: socket.socket) -> None:
        try:
            peer = client.getpeername()
        except Exception:
            peer = ('unknown', 0)
        try:
            client.close()
        except Exception:
            pass
        if client in self.mission_planner_clients:
            self.mission_planner_clients.remove(client)
        self._mission_planner_tx.pop(client, None)
        logger.info('Mission Planner TCP client disconnected: %s:%s', peer[0], peer[1])

    def _poll_mission_planner(self) -> None:
        if self.mission_planner_server is None or self.master is None:
            return

        while True:
            try:
                client, peer = self.mission_planner_server.accept()
            except BlockingIOError:
                break
            except OSError as exc:
                logger.debug('Mission Planner TCP accept failed: %s', exc)
                break

            if self.mission_planner_allowed_clients and peer[0] not in self.mission_planner_allowed_clients:
                logger.warning('Rejected Mission Planner TCP client: %s:%s', peer[0], peer[1])
                client.close()
                continue
            client.setblocking(False)
            self.mission_planner_clients.append(client)
            self._mission_planner_tx[client] = bytearray()
            logger.info('Mission Planner TCP client connected: %s:%s', peer[0], peer[1])

        for client in list(self.mission_planner_clients):
            try:
                data = client.recv(65536)
                if not data:
                    self._close_mission_planner_client(client)
                    continue
                self.master.write(data)
            except BlockingIOError:
                pass
            except OSError:
                self._close_mission_planner_client(client)
                continue

            pending = self._mission_planner_tx.get(client)
            if not pending:
                continue
            try:
                sent = client.send(pending)
                if sent > 0:
                    del pending[:sent]
            except BlockingIOError:
                pass
            except OSError:
                self._close_mission_planner_client(client)

    def _broadcast_mission_planner(self, msg: Any) -> None:
        if not self.mission_planner_clients:
            return
        try:
            raw = bytes(msg.get_msgbuf())
        except Exception:
            return
        for client in list(self.mission_planner_clients):
            pending = self._mission_planner_tx.get(client)
            if pending is None:
                continue
            pending.extend(raw)
            if len(pending) > 1024 * 1024:
                logger.warning('Closing slow Mission Planner TCP client (send queue exceeded 1 MiB)')
                self._close_mission_planner_client(client)

    def _send_node_packet(self, packet_type: str, payload: Dict[str, Any]) -> None:
        packet = {
            'type': packet_type,
            'payload': payload,
            'timestamp': time.time(),
        }
        try:
            data = json.dumps(packet).encode('utf-8')
            self.telemetry_socket.sendto(data, self.node_addr)
        except Exception as exc:
            logger.debug('Failed to send packet to node: %s', exc)

    def _send_node_log(self, level: str, message: str, key: Optional[str] = None, min_interval: float = 0.0) -> None:
        now = time.time()
        log_key = key or f'{level}:{message}'
        if 'uwb: no frame' in str(message).lower():
            log_key = 'statustext:uwb-no-frame'
        alert_level = str(level).upper() in {'WARNING', 'ERROR', 'CRITICAL'}
        if alert_level and log_key in _NODE_ALERT_KEYS_SEEN:
            return
        last_at = self._last_log_sent_at.get(log_key, 0.0)
        if min_interval > 0.0 and now - last_at < min_interval:
            return

        self._last_log_sent_at[log_key] = now
        if alert_level:
            _NODE_ALERT_KEYS_SEEN.add(log_key)
        self._send_node_packet('log', {'level': level, 'message': message})

    def _send_heartbeat(self) -> None:
        if self.master is None:
            return
        try:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )
            self._last_heartbeat_sent = time.time()
        except Exception as exc:
            logger.debug('Heartbeat send failed: %s', exc)

    def _request_pilot_steer_type(self) -> None:
        interval = 10.0 if self.pilot_steer_type is not None else 1.0
        if self.master is None or time.monotonic() - self._pilot_param_requested_at < interval:
            return
        self._pilot_param_requested_at = time.monotonic()
        try:
            self.master.mav.param_request_read_send(
                self.target_system, self.target_component, b'PILOT_STEER_TYPE', -1)
        except Exception as exc:
            logger.debug('Pilot parameter read failed: %s', exc)

    def _request_acro_parameters(self) -> None:
        now = time.monotonic()
        if self.master is None or now - self._acro_requested_at < 0.08:
            return
        candidates = [name for name in required_parameters(self._acro_parameters)
                      if now - self._acro_parameter_times.get(name, -1000) >= 10
                      and now - self._acro_requests.get(name, -1000) >= 1]
        if not candidates:
            return
        name = min(sorted(candidates), key=lambda item: self._acro_requests.get(item, -1000))
        self._acro_requested_at = now
        self._acro_requests[name] = now
        try:
            self.master.mav.param_request_read_send(self.target_system, self.target_component, name.encode(), -1)
        except Exception as exc:
            logger.debug('ACRO parameter read failed: %s', exc)

    def _acro_control_status(self) -> Dict[str, Any]:
        reason = None
        try:
            validate_parameters(self._acro_parameters)
            now = time.monotonic()
            if any(now - self._acro_parameter_times.get(name, -1000) > 30
                   for name in required_parameters(self._acro_parameters)):
                reason = 'acro_parameters_stale'
        except ValueError as exc:
            reason = str(exc)
        now_ms = int(time.time()*1000)
        velocity_available = (self.flight_data.gps_fix_type >= 2 and self.flight_data.ekf_healthy
                              and self.flight_data.position_updated_at is not None
                              and 0 <= now_ms-self.flight_data.position_updated_at <= 700
                              and self.flight_data.ekf_updated_at is not None
                              and 0 <= now_ms-self.flight_data.ekf_updated_at <= 700)
        return {'ready': reason is None, 'reason': reason, 'updatedAt': now_ms,
                'velocityAvailable': velocity_available,
                'speedLimitMps': 2.0, 'fcuSpeedMaxMps': self._acro_parameters.get('SPEED_MAX'),
                'fcuTurnRateDegS': self._acro_parameters.get('ACRO_TURN_RATE')}

    def _send_rover_acro(self, forward_speed: float, yaw_rate_deg_s: float) -> bool:
        if self.master is None:
            return False
        status = self._acro_control_status()
        if (not status['ready'] or self.flight_data.flight_mode != 'ACRO'
                or ((forward_speed != 0 or yaw_rate_deg_s != 0) and not status['velocityAvailable'])):
            self._send_rover_drive(0.0, 0.0)
            self._send_node_log('WARNING', 'ACRO control held neutral: '+str(status['reason'] or 'acro_mode_or_velocity_unavailable'),
                                key='acro-not-ready', min_interval=2.0)
            return False
        try:
            steering, throttle = acro_axes(forward_speed, yaw_rate_deg_s, self._acro_parameters)
            self.master.mav.manual_control_send(self.target_system, 32767, steering, throttle, 32767, 0)
            return True
        except Exception as exc:
            self._send_rover_drive(0.0, 0.0)
            self._send_node_log('ERROR', 'Invalid ACRO control: '+str(exc), key='acro-invalid', min_interval=2.0)
            return False

    def _send_rover_drive(self, throttle: float, steering: float, motor_target_semantics: bool = False) -> bool:
        if self.master is None:
            return False

        steering_axis, throttle_axis = rover_manual_control_axes(
            throttle,
            steering,
            self.rover_throttle_min,
            self.rover_throttle_max,
            self.rover_steering_min,
            self.rover_steering_max,
        )
        if motor_target_semantics:
            try:
                steering_axis, throttle_axis = motor_target_axes(steering_axis, throttle_axis, self.pilot_steer_type)
            except ValueError:
                steering_axis = throttle_axis = 0
                self._send_node_log('ERROR', 'Manual propulsion held neutral: waiting for valid PILOT_STEER_TYPE',
                                    key='pilot-steer-type-missing', min_interval=2.0)
        try:
            self.master.mav.manual_control_send(
                self.target_system,
                32767,
                steering_axis,
                throttle_axis,
                32767,
                0,
            )
        except Exception as exc:
            logger.error('Failed to send rover MANUAL_CONTROL: %s', exc)
            return False

        logger.info(
            'Rover MANUAL_CONTROL sent: throttle=%.1f%% (z=%d), steering=%.1fdeg (y=%d)',
            float(throttle),
            throttle_axis,
            float(steering),
            steering_axis,
        )
        return True

    def _send_rover_velocity_ned(self, vx: float, vy: float) -> bool:
        if self.master is None or not math.isfinite(vx) or not math.isfinite(vy):
            return False

        vx = max(-5.0, min(5.0, float(vx)))
        vy = max(-5.0, min(5.0, float(vy)))
        try:
            self.master.mav.set_position_target_local_ned_send(
                int(time.monotonic() * 1000) & 0xffffffff,
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                3559,  # vx/vy only; ignore position, vz, acceleration, yaw and yaw-rate
                0.0,
                0.0,
                0.0,
                vx,
                vy,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            return True
        except Exception as exc:
            logger.error('Failed to send rover NED velocity: %s', exc)
            return False

    def _send_rover_body_velocity(self, forward_speed: float, yaw_rate: float) -> bool:
        """Command a non-holonomic rover/boat with forward speed and yaw rate."""
        if self.master is None or not math.isfinite(forward_speed) or not math.isfinite(yaw_rate):
            return False

        forward_speed = max(-5.0, min(5.0, float(forward_speed)))
        yaw_rate = max(-math.radians(120.0), min(math.radians(120.0), float(yaw_rate)))
        try:
            self.master.mav.set_position_target_local_ned_send(
                int(time.monotonic() * 1000) & 0xffffffff,
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                1511,  # body vx, zero vy and yaw-rate; ignore position, vz, acceleration and yaw
                0.0,
                0.0,
                0.0,
                forward_speed,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                yaw_rate,
            )
            return True
        except Exception as exc:
            logger.error('Failed to send rover body velocity/yaw-rate: %s', exc)
            return False

    def _arm_disarm(self, arm: bool) -> bool:
        if self.master is None:
            return False
        try:
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1.0 if arm else 0.0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            logger.info('%s command sent', 'ARM' if arm else 'DISARM')
            return True
        except Exception as exc:
            logger.error('Failed to send arm/disarm: %s', exc)
            return False

    def _set_flight_mode(self, mode_name: str) -> bool:
        if self.master is None:
            return False
        normalized_mode = str(mode_name).upper().strip()
        try:
            mode_mapping = self.master.mode_mapping() or {}
            mode_id = mode_mapping.get(normalized_mode)
            if mode_id is None:
                logger.error('Unsupported flight mode: %s', normalized_mode)
                return False
            self.master.set_mode(int(mode_id))
            logger.info('SET_MODE %s command sent (custom mode %s)', normalized_mode, mode_id)
            return True
        except Exception as exc:
            logger.error('Failed to set flight mode %s: %s', normalized_mode, exc)
            return False

    def _update_imu_calibration(self, **changes: Any) -> None:
        for key, value in changes.items():
            setattr(self.imu_calibration, key, value)
        self.imu_calibration.updated_at = time.time()

    def _mav_result_name(self, result_code: int) -> str:
        result_enum = mavutil.mavlink.enums.get('MAV_RESULT', {}).get(result_code)
        return result_enum.name if result_enum else str(result_code)

    def _start_imu_calibration(self, calibration_type: str) -> bool:
        if self.master is None:
            return False

        normalized_type = calibration_type.upper().strip()
        if normalized_type not in ('ACCEL', 'LEVEL'):
            logger.error('Unsupported IMU calibration type: %s', calibration_type)
            return False

        accel_mode = 1.0 if normalized_type == 'ACCEL' else 2.0
        instructions = (
            '6-point IMU calibration started. Waiting for the FCU pose request.'
            if normalized_type == 'ACCEL'
            else 'Level calibration started. Keep the rover level and still.'
        )

        try:
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                MAV_CMD_PREFLIGHT_CALIBRATION,
                0,
                0,
                0,
                0,
                0,
                accel_mode,
                0,
                0,
            )
            self._update_imu_calibration(
                active=True,
                mode=normalized_type,
                status='STARTING',
                step='',
                step_code=None,
                instructions=instructions,
                progress=0 if normalized_type == 'ACCEL' else None,
                last_ack_command=MAV_CMD_PREFLIGHT_CALIBRATION,
                last_ack_result='',
            )
            logger.info('IMU calibration start sent: %s', normalized_type)
            return True
        except Exception as exc:
            logger.error('Failed to start IMU calibration: %s', exc)
            self._update_imu_calibration(
                active=False,
                mode=normalized_type,
                status='FAILED',
                instructions=f'Failed to send calibration command: {exc}',
                last_ack_command=MAV_CMD_PREFLIGHT_CALIBRATION,
                last_ack_result='SEND_FAILED',
            )
            return False

    def _confirm_imu_calibration_position(self, position_code: int) -> bool:
        if self.master is None:
            return False

        normalized_code = int(position_code)
        step_name = ACCELCAL_POSITION_LABELS.get(normalized_code)
        if step_name is None:
            logger.error('Unsupported IMU calibration position: %s', position_code)
            return False

        try:
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                MAV_CMD_ACCELCAL_VEHICLE_POS,
                0,
                float(normalized_code),
                0,
                0,
                0,
                0,
                0,
                0,
            )
            self._update_imu_calibration(
                active=True,
                mode='ACCEL',
                status='CONFIRMING_POSITION',
                step=step_name,
                step_code=normalized_code,
                instructions=f'{step_name} confirmation sent. Waiting for the next FCU step.',
                last_ack_command=MAV_CMD_ACCELCAL_VEHICLE_POS,
                last_ack_result='',
            )
            logger.info('IMU calibration pose confirmed: %s', step_name)
            return True
        except Exception as exc:
            logger.error('Failed to confirm IMU calibration pose: %s', exc)
            self._update_imu_calibration(
                active=True,
                mode='ACCEL',
                status='FAILED',
                instructions=f'Failed to send pose confirmation: {exc}',
                last_ack_command=MAV_CMD_ACCELCAL_VEHICLE_POS,
                last_ack_result='SEND_FAILED',
            )
            return False

    def _handle_imu_pose_request(self, position_code: int) -> None:
        if position_code == ACCELCAL_POSITION_SUCCESS:
            self._update_imu_calibration(
                active=False,
                mode='ACCEL',
                status='SUCCESS',
                step='SUCCESS',
                step_code=position_code,
                instructions='6-point IMU calibration completed. Check the attitude and accelerometer readings.',
                progress=100,
                last_ack_command=MAV_CMD_ACCELCAL_VEHICLE_POS,
                last_ack_result='SUCCESS',
            )
            self._send_node_log('INFO', 'IMU accelerometer calibration completed', key='imu-cal-success', min_interval=0.5)
            return

        if position_code == ACCELCAL_POSITION_FAILED:
            self._update_imu_calibration(
                active=False,
                mode='ACCEL',
                status='FAILED',
                step='FAILED',
                step_code=position_code,
                instructions='6-point IMU calibration failed. Restart it and keep the rover still at every step.',
                last_ack_command=MAV_CMD_ACCELCAL_VEHICLE_POS,
                last_ack_result='FAILED',
            )
            self._send_node_log('WARNING', 'IMU accelerometer calibration failed', key='imu-cal-failed', min_interval=0.5)
            return

        step_name = ACCELCAL_POSITION_LABELS.get(position_code)
        if step_name is None:
            return

        self._update_imu_calibration(
            active=True,
            mode='ACCEL',
            status='AWAITING_POSITION',
            step=step_name,
            step_code=position_code,
            instructions=ACCELCAL_POSITION_HINTS.get(position_code, f'Place the rover in the {step_name} pose, then confirm it.'),
            last_ack_command=MAV_CMD_ACCELCAL_VEHICLE_POS,
            last_ack_result='',
        )
        self._send_node_log('INFO', f'IMU calibration waiting for pose: {step_name}', key=f'imu-cal-step:{step_name}', min_interval=0.2)

    def _handle_imu_ack(self, command: int, result: int, progress: Optional[int]) -> None:
        if command not in (MAV_CMD_PREFLIGHT_CALIBRATION, MAV_CMD_ACCELCAL_VEHICLE_POS):
            return

        result_name = self._mav_result_name(result)
        progress_value = progress if isinstance(progress, int) and 0 <= progress <= 100 else self.imu_calibration.progress

        if command == MAV_CMD_PREFLIGHT_CALIBRATION:
            if result in (MAV_RESULT_ACCEPTED, MAV_RESULT_IN_PROGRESS):
                if self.imu_calibration.mode == 'LEVEL':
                    self._update_imu_calibration(
                        active=result != MAV_RESULT_ACCEPTED,
                        status='SUCCESS' if result == MAV_RESULT_ACCEPTED else 'IN_PROGRESS',
                        instructions='Level calibration completed. Keep the rover still for a few more seconds.'
                        if result == MAV_RESULT_ACCEPTED
                        else 'Level calibration in progress. Keep the rover level and still.',
                        progress=progress_value,
                        last_ack_command=command,
                        last_ack_result=result_name,
                    )
                else:
                    self._update_imu_calibration(
                        active=True,
                        status='IN_PROGRESS',
                        instructions='6-point IMU calibration is in progress. Waiting for the FCU pose request.',
                        progress=progress_value,
                        last_ack_command=command,
                        last_ack_result=result_name,
                    )
            else:
                self._update_imu_calibration(
                    active=False,
                    status='CANCELLED' if result == MAV_RESULT_CANCELLED else 'FAILED',
                    instructions=f'IMU calibration did not start: {result_name}',
                    progress=progress_value,
                    last_ack_command=command,
                    last_ack_result=result_name,
                )

        if command == MAV_CMD_ACCELCAL_VEHICLE_POS:
            if result in (MAV_RESULT_ACCEPTED, MAV_RESULT_IN_PROGRESS):
                step_name = self.imu_calibration.step or ACCELCAL_POSITION_LABELS.get(self.imu_calibration.step_code or 0, '')
                instructions = (
                    f'{step_name} confirmed. Waiting for the next FCU step.'
                    if step_name
                    else 'Pose confirmation sent. Waiting for the next FCU step.'
                )
                self._update_imu_calibration(
                    active=True,
                    status='IN_PROGRESS',
                    instructions=instructions,
                    progress=progress_value,
                    last_ack_command=command,
                    last_ack_result=result_name,
                )
            else:
                self._update_imu_calibration(
                    active=False,
                    status='CANCELLED' if result == MAV_RESULT_CANCELLED else 'FAILED',
                    instructions=f'Pose confirmation failed: {result_name}',
                    progress=progress_value,
                    last_ack_command=command,
                    last_ack_result=result_name,
                )

        self._send_node_log(
            'INFO',
            f'IMU calibration ACK: command={command}, result={result_name}',
            key=f'imu-cal-ack:{command}:{result_name}',
            min_interval=0.2,
        )

    def _update_battery_state(
        self,
        voltage: Optional[float] = None,
        current: Optional[float] = None,
        percentage: Optional[int] = None,
    ) -> None:
        if voltage is not None and voltage > 0:
            self.flight_data.battery_voltage = float(voltage)

        if current is not None and current >= 0:
            self.flight_data.battery_current = float(current)

        if percentage is not None and percentage >= 0:
            self.flight_data.battery_percentage = int(percentage)

        if self.flight_data.battery_percentage <= 10:
            battery_state = 'CRITICAL'
        elif self.flight_data.battery_percentage <= 20:
            battery_state = 'LOW'
        else:
            battery_state = 'NORMAL'

        if battery_state != self._last_battery_state:
            if battery_state == 'CRITICAL':
                self._send_node_log(
                    'CRITICAL',
                    f'Battery critical: {self.flight_data.battery_percentage:.0f}% ({self.flight_data.battery_voltage:.2f}V)',
                    key='battery-state',
                    min_interval=5.0,
                )
            elif battery_state == 'LOW':
                self._send_node_log(
                    'WARNING',
                    f'Battery low: {self.flight_data.battery_percentage:.0f}% ({self.flight_data.battery_voltage:.2f}V)',
                    key='battery-state',
                    min_interval=5.0,
                )
            elif self._last_battery_state in ('LOW', 'CRITICAL'):
                self._send_node_log(
                    'INFO',
                    f'Battery recovered: {self.flight_data.battery_percentage:.0f}% ({self.flight_data.battery_voltage:.2f}V)',
                    key='battery-state',
                    min_interval=5.0,
                )
            self._last_battery_state = battery_state

    def _esc_temperature_for_channel(self, temperatures: Any, base_channel: int, target_channel: int) -> Optional[float]:
        index = int(target_channel) - int(base_channel)
        if index < 0:
            return None

        try:
            raw_value = list(temperatures)[index]
        except (IndexError, TypeError):
            return None

        numeric = int(raw_value or 0)
        if numeric <= 0:
            return None

        return float(numeric)

    def _update_esc_telemetry(self, msg: Any, base_channel: int) -> None:
        temperatures = getattr(msg, 'temperature', None)
        if temperatures is None:
            return

        left_temperature = self._esc_temperature_for_channel(
            temperatures,
            base_channel,
            self.rover_left_output_channel,
        )
        if left_temperature is not None:
            self.flight_data.motor_left_temperature = left_temperature

        right_temperature = self._esc_temperature_for_channel(
            temperatures,
            base_channel,
            self.rover_right_output_channel,
        )
        if right_temperature is not None:
            self.flight_data.motor_right_temperature = right_temperature

        now_ms = int(time.time() * 1000)
        feedback_fields = ('count', 'voltage', 'current', 'totalcurrent', 'rpm', 'temperature')

        def channel_has_feedback(target_channel: int) -> bool:
            index = int(target_channel) - int(base_channel)
            if index < 0 or index > 3:
                return False
            for field_name in feedback_fields:
                values = getattr(msg, field_name, None)
                try:
                    if float(list(values)[index] or 0) != 0:
                        return True
                except (IndexError, TypeError, ValueError):
                    continue
            return False

        if channel_has_feedback(self.rover_left_output_channel):
            self.flight_data.motor_left_esc_updated_at = now_ms
        if channel_has_feedback(self.rover_right_output_channel):
            self.flight_data.motor_right_esc_updated_at = now_ms

    def _handle_command_packet(self, raw_data: bytes) -> None:
        try:
            packet = json.loads(raw_data.decode('utf-8'))
        except Exception as exc:
            logger.error('Invalid command packet: %s', exc)
            return

        command = str(packet.get('command', '')).upper().strip()
        params = packet.get('params', {}) or {}

        if command == 'ROVER_ACRO':
            try:
                self._send_rover_acro(float(params.get('forwardSpeedMps', 0)), float(params.get('yawRateDegS', 0)))
            except (ValueError, TypeError):
                self._send_rover_drive(0.0, 0.0)
            return

        if command == 'MOTOR_CONTROL':
            logger.warning('Ignored raw MOTOR_CONTROL; use ROVER_DRIVE so Pixhawk SERVO reversal remains authoritative')
            return

        if command == 'ROVER_DRIVE':
            if 'throttle' in params or 'steering' in params:
                throttle = float(params.get('throttle', 0.0))
                steering = float(params.get('steering', 0.0))
            else:
                # Transitional compatibility for a Node process from the prior PWM-shaped protocol.
                throttle_pwm = int(params.get('throttlePwm', 1500))
                steering_pwm = int(params.get('steeringPwm', 1500))
                throttle = self.rover_throttle_min + (throttle_pwm - 1000) * (
                    self.rover_throttle_max - self.rover_throttle_min
                ) / 1000.0
                steering = self.rover_steering_min + (steering_pwm - 1000) * (
                    self.rover_steering_max - self.rover_steering_min
                ) / 1000.0
            self._send_rover_drive(throttle, steering, bool(params.get('motorTargetSemantics', False)))
            return

        if command == 'ROVER_VELOCITY_NED':
            self._send_rover_velocity_ned(
                float(params.get('vx', 0.0)),
                float(params.get('vy', 0.0)),
            )
            return

        if command == 'ROVER_BODY_VELOCITY':
            self._send_rover_body_velocity(
                float(params.get('forwardSpeedMps', 0.0)),
                float(params.get('yawRateRadS', 0.0)),
            )
            return

        if command == 'SET_MODE':
            self._set_flight_mode(str(params.get('mode', '')))
            return

        if command == 'ARM':
            self._arm_disarm(True)
            return

        if command == 'DISARM':
            self._arm_disarm(False)
            return

        if command == 'IMU_CALIBRATION_START':
            calibration_type = str(params.get('type', 'ACCEL'))
            self._start_imu_calibration(calibration_type)
            return

        if command == 'IMU_CALIBRATION_CONFIRM':
            position_code = int(params.get('positionCode', 0))
            self._confirm_imu_calibration_position(position_code)
            return

        if command == 'EMERGENCY_STOP':
            self._send_rover_drive(0.0, 0.0)
            self._arm_disarm(False)
            return

        logger.warning('Unknown command from node: %s', command)

    def _poll_commands(self) -> None:
        while True:
            try:
                raw_data, _addr = self.command_socket.recvfrom(65535)
            except BlockingIOError:
                break
            except Exception as exc:
                logger.debug('Command socket recv error: %s', exc)
                break

            self._handle_command_packet(raw_data)

    def _parse_message(self, msg: Any) -> None:
        msg_type = msg.get_type()

        if msg_type == 'PARAM_VALUE':
            param_id = getattr(msg, 'param_id', '')
            if isinstance(param_id, bytes):
                param_id = param_id.decode('ascii', errors='ignore')
            name = param_id.rstrip('\x00')
            if (msg.get_srcSystem() == self.target_system and msg.get_srcComponent() == self.target_component
                    and name in required_parameters(self._acro_parameters)):
                value = float(getattr(msg, 'param_value', float('nan')))
                if math.isfinite(value):
                    self._acro_parameters[name] = value
                    self._acro_parameter_times[name] = time.monotonic()
            if (param_id.rstrip('\x00') == 'PILOT_STEER_TYPE'
                    and msg.get_srcSystem() == self.target_system
                    and msg.get_srcComponent() == self.target_component):
                self.pilot_steer_type = valid_pilot_type(getattr(msg, 'param_value', None))
            return

        if msg_type == 'HEARTBEAT':
            if not is_target_fcu_heartbeat(msg, self.target_system, self.target_component):
                return
            self._last_fcu_heartbeat_at = time.time()
            self.flight_data.armed = bool(getattr(msg, 'base_mode', 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.flight_data.flight_mode = mavutil.mode_string_v10(msg)
            status_code = int(getattr(msg, 'system_status', 0))
            status_enum = mavutil.mavlink.enums.get('MAV_STATE', {}).get(status_code)
            self.flight_data.system_status = status_enum.name if status_enum else str(status_code)

            if self.flight_data.flight_mode != self._last_mode_reported:
                if self._last_mode_reported:
                    self._send_node_log(
                        'INFO',
                        f'Flight mode changed: {self._last_mode_reported} -> {self.flight_data.flight_mode}',
                        key='mode-change',
                        min_interval=0.5,
                    )
                else:
                    self._send_node_log('INFO', f'Flight mode: {self.flight_data.flight_mode}', key='mode-init')
                self._last_mode_reported = self.flight_data.flight_mode

            if self._last_armed_reported is None or self.flight_data.armed != self._last_armed_reported:
                self._send_node_log(
                    'COMMAND' if self.flight_data.armed else 'WARNING',
                    'Vehicle armed' if self.flight_data.armed else 'Vehicle disarmed',
                    key='arm-state-change',
                    min_interval=0.5,
                )
                self._last_armed_reported = self.flight_data.armed

            if self.flight_data.system_status != self._last_system_status_reported:
                if self._last_system_status_reported:
                    self._send_node_log(
                        'INFO',
                        f'System status changed: {self._last_system_status_reported} -> {self.flight_data.system_status}',
                        key='sys-status-change',
                        min_interval=1.0,
                    )
                self._last_system_status_reported = self.flight_data.system_status

        elif msg_type == 'GPS_RAW_INT':
            self.flight_data.gps_satellites = int(getattr(msg, 'satellites_visible', 0) or 0)
            self.flight_data.gps_fix_type = int(getattr(msg, 'fix_type', 0) or 0)
            eph = int(getattr(msg, 'eph', 65535) or 65535)
            self.flight_data.gps_hdop = eph / 100.0 if eph != 65535 else self.flight_data.gps_hdop

            raw_latitude = float(getattr(msg, 'lat', 0) or 0) / 1e7
            raw_longitude = float(getattr(msg, 'lon', 0) or 0) / 1e7
            raw_altitude = float(getattr(msg, 'alt', 0) or 0) / 1000.0
            has_fix = self.flight_data.gps_fix_type >= 2
            if has_fix and (abs(raw_latitude) > 0.000001 or abs(raw_longitude) > 0.000001):
                self.flight_data.gps_latitude = raw_latitude
                self.flight_data.gps_longitude = raw_longitude
                self.flight_data.gps_altitude = raw_altitude

            horizontal_accuracy = int(getattr(msg, 'h_acc', 0) or 0)
            vertical_accuracy = int(getattr(msg, 'v_acc', 0) or 0)
            velocity = int(getattr(msg, 'vel', 65535))
            course = int(getattr(msg, 'cog', 65535))
            self.flight_data.gps_horizontal_accuracy = (
                horizontal_accuracy / 1000.0
                if has_fix and 0 < horizontal_accuracy < 1_000_000
                else None
            )
            self.flight_data.gps_vertical_accuracy = (
                vertical_accuracy / 1000.0
                if has_fix and 0 < vertical_accuracy < 1_000_000
                else None
            )
            self.flight_data.gps_ground_speed = velocity / 100.0 if has_fix and velocity != 65535 else None
            self.flight_data.gps_course = course / 100.0 if has_fix and course != 65535 else None
            self.flight_data.gps_updated_at = int(time.time() * 1000)

            gps_fix_ok = has_fix
            if self._last_gps_fix_reported is None or gps_fix_ok != self._last_gps_fix_reported:
                self._send_node_log(
                    'INFO' if gps_fix_ok else 'WARNING',
                    f'GPS fix {"acquired" if gps_fix_ok else "lost"} '
                    f'(type={self.flight_data.gps_fix_type}, sat={self.flight_data.gps_satellites}, '
                    f'hdop={self.flight_data.gps_hdop:.1f})',
                    key='gps-fix-change',
                    min_interval=2.0,
                )
                self._last_gps_fix_reported = gps_fix_ok

        elif msg_type == 'GLOBAL_POSITION_INT':
            self.flight_data.latitude = float(getattr(msg, 'lat', 0)) / 1e7
            self.flight_data.longitude = float(getattr(msg, 'lon', 0)) / 1e7
            self.flight_data.altitude = float(getattr(msg, 'relative_alt', 0)) / 1000.0
            self.flight_data.vx = float(getattr(msg, 'vx', 0)) / 100.0
            self.flight_data.vy = float(getattr(msg, 'vy', 0)) / 100.0
            self.flight_data.vz = float(getattr(msg, 'vz', 0)) / 100.0
            self.flight_data.position_source = 'GLOBAL_POSITION_INT'
            self.flight_data.position_updated_at = int(time.time() * 1000)

        elif msg_type == 'EKF_STATUS_REPORT':
            flags = int(getattr(msg, 'flags', 0) or 0)
            velocity_horiz = int(getattr(mavutil.mavlink, 'ESTIMATOR_VELOCITY_HORIZ', 2))
            pos_horiz_abs = int(getattr(mavutil.mavlink, 'ESTIMATOR_POS_HORIZ_ABS', 16))
            const_pos_mode = int(getattr(mavutil.mavlink, 'ESTIMATOR_CONST_POS_MODE', 128))
            gps_glitch = int(getattr(mavutil.mavlink, 'ESTIMATOR_GPS_GLITCH', 32768))
            required = velocity_horiz | pos_horiz_abs
            self.flight_data.ekf_flags = flags
            self.flight_data.ekf_healthy = (
                flags & required == required
                and flags & const_pos_mode == 0
                and flags & gps_glitch == 0
            )
            self.flight_data.ekf_velocity_variance = float(getattr(msg, 'velocity_variance', 0.0))
            self.flight_data.ekf_pos_horiz_variance = float(getattr(msg, 'pos_horiz_variance', 0.0))
            self.flight_data.ekf_compass_variance = float(getattr(msg, 'compass_variance', 0.0))
            self.flight_data.ekf_updated_at = int(time.time() * 1000)

        elif msg_type == 'ATTITUDE':
            self.flight_data.roll = math.degrees(float(getattr(msg, 'roll', 0.0)))
            self.flight_data.pitch = math.degrees(float(getattr(msg, 'pitch', 0.0)))
            self.flight_data.yaw = math.degrees(float(getattr(msg, 'yaw', 0.0)))
            self.flight_data.yaw_rate_dps = math.degrees(float(getattr(msg, 'yawspeed', 0.0)))
            self.flight_data.attitude_updated_at = int(time.time() * 1000)

        elif msg_type == 'HIGHRES_IMU':
            self.flight_data.board_temperature = float(getattr(msg, 'temperature', self.flight_data.board_temperature or 0.0))

        elif msg_type in ('SCALED_PRESSURE', 'SCALED_PRESSURE2', 'SCALED_PRESSURE3'):
            temperature_cdeg = getattr(msg, 'temperature', None)
            if temperature_cdeg is not None:
                self.flight_data.board_temperature = float(temperature_cdeg) / 100.0

        elif msg_type == 'SYS_STATUS':
            voltage = float(getattr(msg, 'voltage_battery', 0)) / 1000.0
            current = float(getattr(msg, 'current_battery', -1))
            percent = int(getattr(msg, 'battery_remaining', -1))
            self._update_battery_state(
                voltage=voltage if voltage > 0 else None,
                current=current / 100.0 if current >= 0 else None,
                percentage=percent if percent >= 0 else None,
            )

        elif msg_type == 'BATTERY_STATUS':
            raw_cells = getattr(msg, 'voltages', None) or []
            cell_voltages = [int(cell) for cell in raw_cells if int(cell or 0) not in (0, 65535)]
            total_voltage = sum(cell_voltages) / 1000.0 if cell_voltages else None
            current_raw = int(getattr(msg, 'current_battery', -1) or -1)
            percentage = int(getattr(msg, 'battery_remaining', -1) or -1)
            self._update_battery_state(
                voltage=total_voltage,
                current=current_raw / 100.0 if current_raw >= 0 else None,
                percentage=percentage if percentage >= 0 else None,
            )

        elif msg_type == 'COMMAND_ACK':
            command = int(getattr(msg, 'command', 0) or 0)
            result = int(getattr(msg, 'result', 0) or 0)
            if command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                result_name = self._mav_result_name(result)
                logger.info('ARM/DISARM ACK: %s (%d)', result_name, result)
                self._send_node_log(
                    'INFO' if result == MAV_RESULT_ACCEPTED else 'WARNING',
                    f'FCU ARM/DISARM ACK: {result_name} ({result})',
                    key=f'arm-ack:{result}',
                    min_interval=0.2,
                )
            progress_raw = getattr(msg, 'progress', None)
            progress = None
            if progress_raw is not None:
                progress_value = int(progress_raw)
                progress = progress_value if 0 <= progress_value <= 100 else None
            self._handle_imu_ack(command, result, progress)

        elif msg_type in ('COMMAND_LONG', 'COMMAND_INT'):
            command = int(getattr(msg, 'command', 0) or 0)
            if command == MAV_CMD_ACCELCAL_VEHICLE_POS:
                position_code = int(round(float(getattr(msg, 'param1', 0) or 0)))
                self._handle_imu_pose_request(position_code)

        elif msg_type == 'NAMED_VALUE_FLOAT':
            name = self._param_name(getattr(msg, 'name', ''))
            try:
                value = float(getattr(msg, 'value', 0.0))
            except (TypeError, ValueError):
                value = math.nan
            source = None
            try:
                source = (int(msg.get_srcSystem()), int(msg.get_srcComponent()))
            except (AttributeError, TypeError, ValueError):
                pass
            self._update_uwb_named_value(name, value, source=source)

        elif msg_type == 'STATUSTEXT':
            raw_text = getattr(msg, 'text', '')
            if isinstance(raw_text, bytes):
                text = raw_text.decode('utf-8', errors='ignore').strip('\x00 ').strip()
            else:
                text = str(raw_text).strip()

            if text:
                severity = int(getattr(msg, 'severity', 6) or 6)
                if severity <= 2:
                    level = 'CRITICAL'
                elif severity <= 4:
                    level = 'WARNING'
                elif severity <= 6:
                    level = 'INFO'
                else:
                    level = 'DEBUG'

                self._send_node_log(level, f'FCU: {text}', key=f'statustext:{text}', min_interval=2.0)

                lower_text = text.lower()
                if self.imu_calibration.active and any(keyword in lower_text for keyword in ('calib', 'accel', 'imu', 'trim', 'place vehicle')):
                    next_status = self.imu_calibration.status
                    next_active = self.imu_calibration.active
                    if 'success' in lower_text or 'completed' in lower_text:
                        next_status = 'SUCCESS'
                        next_active = False
                    elif 'failed' in lower_text or 'error' in lower_text:
                        next_status = 'FAILED'
                        next_active = False

                    self._update_imu_calibration(
                        active=next_active,
                        status=next_status,
                        instructions=text,
                    )

        elif msg_type == 'SERVO_OUTPUT_RAW':
            self.flight_data.servo_outputs['ch1'] = int(getattr(msg, 'servo1_raw', 0) or 0)
            self.flight_data.servo_outputs['ch2'] = int(getattr(msg, 'servo2_raw', 0) or 0)
            self.flight_data.servo_outputs['ch3'] = int(getattr(msg, 'servo3_raw', 0) or 0)
            self.flight_data.servo_outputs['ch4'] = int(getattr(msg, 'servo4_raw', 0) or 0)
            self.flight_data.servo_outputs_updated_at = int(time.time() * 1000)

        elif msg_type == 'ESC_TELEMETRY_1_TO_4':
            self._update_esc_telemetry(msg, 1)

        elif msg_type == 'ESC_TELEMETRY_5_TO_8':
            self._update_esc_telemetry(msg, 5)

        elif msg_type == 'ESC_TELEMETRY_9_TO_12':
            self._update_esc_telemetry(msg, 9)

        elif msg_type == 'ESC_TELEMETRY_13_TO_16':
            self._update_esc_telemetry(msg, 13)

        elif msg_type == 'ESC_TELEMETRY_17_TO_20':
            self._update_esc_telemetry(msg, 17)

        elif msg_type == 'ESC_TELEMETRY_21_TO_24':
            self._update_esc_telemetry(msg, 21)

        elif msg_type == 'ESC_TELEMETRY_25_TO_28':
            self._update_esc_telemetry(msg, 25)

        elif msg_type == 'ESC_TELEMETRY_29_TO_32':
            self._update_esc_telemetry(msg, 29)

        self.flight_data.updated_at = time.time()

    def _poll_mavlink(self) -> None:
        if self.master is None:
            return

        while True:
            try:
                msg = self.master.recv_match(blocking=False)
            except Exception as exc:
                logger.debug('recv_match error: %s', exc)
                break

            if msg is None:
                break

            self._broadcast_mission_planner(msg)
            self._parse_message(msg)

    def _telemetry_payload(self) -> Dict[str, Any]:
        temperature_payload: Dict[str, Any] = {}
        if self.flight_data.board_temperature is not None:
            temperature_payload['flightController'] = self.flight_data.board_temperature
        if self.flight_data.motor_left_temperature is not None:
            temperature_payload['motorLeft'] = self.flight_data.motor_left_temperature
        if self.flight_data.motor_right_temperature is not None:
            temperature_payload['motorRight'] = self.flight_data.motor_right_temperature

        now_ms = int(time.time() * 1000)
        self._refresh_uwb_fusion()
        fused_uwb = self.uwb_fusion.fused(now_ms)
        uwb_fresh = fused_uwb['fresh']

        return {
            'position': {
                'lat': self.flight_data.latitude,
                'lon': self.flight_data.longitude,
                'alt': self.flight_data.altitude,
                'source': self.flight_data.position_source,
                'updatedAt': self.flight_data.position_updated_at,
            },
            'attitude': {
                'roll': self.flight_data.roll,
                'pitch': self.flight_data.pitch,
                'yaw': self.flight_data.yaw,
                'yawRateDegS': self.flight_data.yaw_rate_dps,
                'updatedAt': self.flight_data.attitude_updated_at,
            },
            'velocity': {
                'vx': self.flight_data.vx,
                'vy': self.flight_data.vy,
                'vz': self.flight_data.vz,
            },
            'ekf': {
                'healthy': self.flight_data.ekf_healthy,
                'flags': self.flight_data.ekf_flags,
                'velocityVariance': self.flight_data.ekf_velocity_variance,
                'positionHorizontalVariance': self.flight_data.ekf_pos_horiz_variance,
                'compassVariance': self.flight_data.ekf_compass_variance,
                'updatedAt': self.flight_data.ekf_updated_at,
                'source': 'EKF_STATUS_REPORT',
            },
            'battery': {
                'voltage': self.flight_data.battery_voltage,
                'current': self.flight_data.battery_current,
                'percentage': self.flight_data.battery_percentage,
            },
            'servoOutputs': self.flight_data.servo_outputs,
            'temperature': temperature_payload,
            'hardware': {
                'pixhawk': {
                    'online': self.connected,
                    'source': 'heartbeat',
                    'lastSeenAt': int(self._last_fcu_heartbeat_at * 1000) if self._last_fcu_heartbeat_at else None,
                },
                'motors': {
                    'left': {
                        'channel': self.rover_left_output_channel,
                        'outputPwm': self.flight_data.servo_outputs.get(f'ch{self.rover_left_output_channel}', 0),
                        'outputUpdatedAt': self.flight_data.servo_outputs_updated_at,
                        'feedbackUpdatedAt': self.flight_data.motor_left_esc_updated_at,
                        'feedbackSource': 'esc_telemetry' if self.flight_data.motor_left_esc_updated_at else 'none',
                    },
                    'right': {
                        'channel': self.rover_right_output_channel,
                        'outputPwm': self.flight_data.servo_outputs.get(f'ch{self.rover_right_output_channel}', 0),
                        'outputUpdatedAt': self.flight_data.servo_outputs_updated_at,
                        'feedbackUpdatedAt': self.flight_data.motor_right_esc_updated_at,
                        'feedbackSource': 'esc_telemetry' if self.flight_data.motor_right_esc_updated_at else 'none',
                    },
                },
            },
            'gps': {
                'satellites': self.flight_data.gps_satellites,
                'hdop': self.flight_data.gps_hdop,
                'fixType': self.flight_data.gps_fix_type,
                'latitude': self.flight_data.gps_latitude,
                'longitude': self.flight_data.gps_longitude,
                'altitude': self.flight_data.gps_altitude,
                'horizontalAccuracy': self.flight_data.gps_horizontal_accuracy,
                'verticalAccuracy': self.flight_data.gps_vertical_accuracy,
                'groundSpeed': self.flight_data.gps_ground_speed,
                'course': self.flight_data.gps_course,
                'updatedAt': self.flight_data.gps_updated_at,
            },
            'uwb': {
                'online': uwb_fresh,
                'fresh': uwb_fresh,
                'distanceM': self.uwb_data.distance_m,
                'azimuthDeg': self.uwb_data.azimuth_deg,
                'elevationDeg': self.uwb_data.elevation_deg,
                'rawDistanceM': self.uwb_data.raw_distance_m,
                'rawAzimuthDeg': self.uwb_data.raw_azimuth_deg,
                'rawElevationDeg': self.uwb_data.raw_elevation_deg,
                'goodCount': self.uwb_data.good_count,
                'updatedAt': self.uwb_data.updated_at,
                'measurementUpdatedAt': max((sensor.get('measurementUpdatedAt') or 0 for sensor in fused_uwb['sensors'] if sensor['receiverId'] in fused_uwb['active_receiver_ids']), default=0) or None,
                'receiverCount': fused_uwb['receiver_count'],
                'observingReceiverCount': fused_uwb['observing_receiver_count'],
                'activeReceiverIds': fused_uwb['active_receiver_ids'],
                'fusionMode': fused_uwb['fusion_mode'],
                'bearingDisagreementDeg': fused_uwb['bearing_disagreement_deg'],
                'positionDisagreementM': fused_uwb['position_disagreement_m'],
                'relativeXM': fused_uwb['relative_x_m'],
                'relativeYM': fused_uwb['relative_y_m'],
                'sensors': fused_uwb['sensors'],
                'source': (
                    'TELEM1_NAMED_VALUE_FLOAT'
                    if self.uwb_fusion.enabled_receiver_ids == {1}
                    else 'TELEM2_NAMED_VALUE_FLOAT'
                    if self.uwb_fusion.enabled_receiver_ids == {2}
                    else 'TELEM1_TELEM2_NAMED_VALUE_FLOAT'
                ),
                'filter': 'raw_validation_innovation_gate_cv_kalman_then_fusion' if self.uwb_filter_enabled else 'fast_per_receiver_kalman_then_body_xy_fusion',
                'measurementStatus': 'trusted' if uwb_fresh else 'suspect' if any(s.get('measurementStatus') == 'suspect' for s in fused_uwb['sensors']) else 'unavailable',
                'rejectionReason': None if uwb_fresh else next((s.get('filterReason') for s in fused_uwb['sensors'] if s.get('measurementStatus') == 'suspect'), None),
                'measurementNis': max((s['filterNis'] for s in fused_uwb['sensors'] if s.get('filterNis') is not None and (not uwb_fresh or s['receiverId'] in fused_uwb['active_receiver_ids'])), default=None),
            },
            'imuCalibration': {
                'active': self.imu_calibration.active,
                'mode': self.imu_calibration.mode,
                'status': self.imu_calibration.status,
                'step': self.imu_calibration.step,
                'stepCode': self.imu_calibration.step_code,
                'instructions': self.imu_calibration.instructions,
                'progress': self.imu_calibration.progress,
                'lastAckCommand': self.imu_calibration.last_ack_command,
                'lastAckResult': self.imu_calibration.last_ack_result,
                'updatedAt': self.imu_calibration.updated_at,
            },
            'flightMode': self.flight_data.flight_mode,
            'pilotSteering': {'type': self.pilot_steer_type, 'ready': self.pilot_steer_type is not None},
            'acroControl': self._acro_control_status(),
            'systemStatus': self.flight_data.system_status,
            'armed': self.flight_data.armed,
        }

    def run(self) -> None:
        logger.info('Command listener: udp://%s:%d', self.bridge_host, self.command_port)
        logger.info('Telemetry target: udp://127.0.0.1:%d', self.telemetry_port)
        if self.mission_planner_server is not None:
            logger.info(
                'Mission Planner TCP endpoint: tcp://%s:%d',
                self.mission_planner_tcp_host,
                self.mission_planner_tcp_port,
            )

        try:
            while self.connected:
                now = time.time()

                self._poll_commands()
                self._poll_mission_planner()
                self._poll_mavlink()
                self._request_pilot_steer_type()
                self._request_acro_parameters()

                if self._last_fcu_heartbeat_at and now - self._last_fcu_heartbeat_at > max(3.0, self.heartbeat_interval * 3.5):
                    self._send_node_log(
                        'ERROR',
                        'Pixhawk heartbeat lost; control link closed',
                        key='runtime-heartbeat-lost',
                    )
                    self._send_node_packet('connection', {'connected': False})
                    self.connected = False
                    break

                if now - self._last_heartbeat_sent >= self.heartbeat_interval:
                    self._send_heartbeat()

                if now - self._last_telemetry_sent >= self.telemetry_interval:
                    self._send_node_packet('telemetry', self._telemetry_payload())
                    self._last_telemetry_sent = now

                time.sleep(0.005)
        except KeyboardInterrupt:
            logger.info('Keyboard interrupt received, stopping bridge')
        finally:
            self.disconnect()


def main() -> None:
    config = load_config()

    serial_port = str(config.get('serial_port', '/dev/ttyUSB0'))
    baudrate = int(config.get('baud_rate', 57600))
    bridge_host = str(config.get('bridge_host', '127.0.0.1'))
    command_port = int(config.get('bridge_command_port', 14551))
    telemetry_port = int(config.get('bridge_telemetry_port', 14552))
    telemetry_rate_hz = int(config.get('telemetry_rate', 10))
    heartbeat_interval = float(config.get('heartbeat_interval', 1.0))
    rover_left_output_channel = int(config.get('rover_left_channel', 1))
    rover_right_output_channel = int(config.get('rover_right_channel', 3))
    rover_throttle_min = float(config.get('rover_throttle_min', -100))
    rover_throttle_max = float(config.get('rover_throttle_max', 100))
    rover_steering_min = float(config.get('rover_steering_min', -45))
    rover_steering_max = float(config.get('rover_steering_max', 45))
    uwb_dual_config = config.get('uwb_dual', {})
    uwb_measurement_filter_config = config.get('uwb_measurement_filter', {})
    uwb_bearing_config = config.get('gimbal', {}).get('beacon', {})
    mission_planner_tcp_config = config.get('mission_planner_tcp', {})

    logger.info('Bridge will keep retrying until Pixhawk heartbeat is received')

    while True:
        bridge = MAVLinkBridge(
            serial_port=serial_port,
            baudrate=baudrate,
            bridge_host=bridge_host,
            command_port=command_port,
            telemetry_port=telemetry_port,
            telemetry_rate_hz=telemetry_rate_hz,
            heartbeat_interval=heartbeat_interval,
            rover_left_output_channel=rover_left_output_channel,
            rover_right_output_channel=rover_right_output_channel,
            rover_throttle_min=rover_throttle_min,
            rover_throttle_max=rover_throttle_max,
            rover_steering_min=rover_steering_min,
            rover_steering_max=rover_steering_max,
            uwb_dual_config=uwb_dual_config,
            uwb_measurement_filter_config=uwb_measurement_filter_config,
            uwb_bearing_config=uwb_bearing_config,
            mission_planner_tcp_config=mission_planner_tcp_config,
        )

        if bridge.connect(timeout=8.0):
            bridge.run()
            logger.warning('Bridge disconnected from Pixhawk, retrying in 2 seconds...')
            time.sleep(2.0)
            continue

        bridge.disconnect()
        logger.warning('No heartbeat yet, retrying in 2 seconds...')
        time.sleep(2.0)


if __name__ == '__main__':
    main()
