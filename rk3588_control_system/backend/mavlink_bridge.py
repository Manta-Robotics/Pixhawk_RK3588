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
import socket
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

try:
    from pymavlink import mavutil
except ImportError:
    print('ERROR: pymavlink is not installed. Run: python3 -m pip install -r requirements.txt')
    sys.exit(1)

BASE_DIR = Path(__file__).resolve().parent.parent
CONFIG_PATH = BASE_DIR / 'config' / 'system.config.json'
LOGS_DIR = BASE_DIR / 'logs'
LOGS_DIR.mkdir(parents=True, exist_ok=True)
MAVLINK_LOG_PATH = LOGS_DIR / 'mavlink.log'

_log_handlers = [logging.StreamHandler()]
try:
    _log_handlers.append(logging.FileHandler(MAVLINK_LOG_PATH))
except PermissionError:
    # systemd already captures stdout into the same file as root; skip the
    # in-process file handler if the existing log isn't writable by us.
    pass

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [%(levelname)s] %(message)s',
    handlers=_log_handlers,
)
logger = logging.getLogger('mavlink-bridge')

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
MAV_CMD_DO_SET_SERVO = getattr(mavutil.mavlink, 'MAV_CMD_DO_SET_SERVO', 183)
MAV_CMD_SET_MESSAGE_INTERVAL = getattr(mavutil.mavlink, 'MAV_CMD_SET_MESSAGE_INTERVAL', 511)
MAV_RESULT_ACCEPTED = getattr(mavutil.mavlink, 'MAV_RESULT_ACCEPTED', 0)
MAV_RESULT_IN_PROGRESS = getattr(mavutil.mavlink, 'MAV_RESULT_IN_PROGRESS', 5)
MAV_RESULT_CANCELLED = getattr(mavutil.mavlink, 'MAV_RESULT_CANCELLED', 6)
MAV_PARAM_TYPE_REAL32 = getattr(mavutil.mavlink, 'MAV_PARAM_TYPE_REAL32', 9)


def load_config() -> Dict[str, Any]:
    if not CONFIG_PATH.exists():
        logger.warning('Config file not found at %s, using defaults', CONFIG_PATH)
        return {}

    try:
        return json.loads(CONFIG_PATH.read_text(encoding='utf-8'))
    except Exception as exc:
        logger.error('Failed to read config: %s', exc)
        return {}


@dataclass
class FlightData:
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
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
    board_temperature: Optional[float] = None
    motor_left_temperature: Optional[float] = None
    motor_right_temperature: Optional[float] = None
    gps_satellites: int = 0
    gps_hdop: float = 999.0
    flight_mode: str = 'STABILIZE'
    system_status: str = 'STANDBY'
    armed: bool = False
    updated_at: float = field(default_factory=time.time)


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

        self.master: Optional[mavutil.mavfile] = None
        self.connected = False
        self.target_system = 1
        self.target_component = 1
        self.flight_data = FlightData()
        self.imu_calibration = IMUCalibrationState()

        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_socket.bind((self.bridge_host, self.command_port))
        self.command_socket.setblocking(False)

        self.telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.node_addr = ('127.0.0.1', self.telemetry_port)

        self._last_telemetry_sent = 0.0
        self._last_heartbeat_sent = 0.0
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

    def _fetch_param(self, name: str, timeout: float = 2.0) -> Optional[float]:
        if self.master is None:
            return None

        try:
            self.master.param_fetch_one(name)
        except Exception as exc:
            logger.debug('Failed to request param %s: %s', name, exc)
            return None

        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.4)
            except Exception as exc:
                logger.debug('Failed while fetching param %s: %s', name, exc)
                return None

            if not msg:
                continue

            if self._param_name(getattr(msg, 'param_id', '')) == name:
                return float(getattr(msg, 'param_value', 0.0) or 0.0)

        return None

    def _set_param(self, name: str, value: float, timeout: float = 3.0) -> Optional[float]:
        if self.master is None:
            return None

        try:
            self.master.mav.param_set_send(
                self.target_system,
                self.target_component,
                name.encode('utf-8'),
                float(value),
                MAV_PARAM_TYPE_REAL32,
            )
        except Exception as exc:
            logger.debug('Failed to send param_set for %s: %s', name, exc)
            return None

        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
            except Exception as exc:
                logger.debug('Failed while waiting param ack for %s: %s', name, exc)
                return None

            if not msg:
                continue

            if self._param_name(getattr(msg, 'param_id', '')) == name:
                return float(getattr(msg, 'param_value', 0.0) or 0.0)

        return None

    def _ensure_direct_motor_outputs(self) -> None:
        if self.master is None:
            return

        target_params = {
            f'SERVO{self.rover_left_output_channel}_FUNCTION': 0,
            f'SERVO{self.rover_right_output_channel}_FUNCTION': 0,
        }

        for name, target_value in target_params.items():
            current_value = self._fetch_param(name)
            if current_value is None:
                logger.warning('Unable to read %s while preparing direct motor control', name)
                continue

            if int(round(current_value)) == target_value:
                continue

            applied_value = self._set_param(name, float(target_value))
            if applied_value is None:
                logger.warning('Unable to set %s=%s for direct motor control', name, target_value)
                continue

            logger.warning('Updated %s from %s to %s for direct motor control', name, int(round(current_value)), int(round(applied_value)))
            self._send_node_log(
                'WARNING',
                f'{name} changed from {int(round(current_value))} to {int(round(applied_value))} for direct motor PWM control',
                key=f'direct-output:{name}',
                min_interval=1.0,
            )

    def _set_message_interval(self, message_id: int, rate_hz: float) -> None:
        if self.master is None or rate_hz <= 0:
            return

        interval_us = max(1000, int(round(1_000_000 / rate_hz)))

        try:
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(message_id),
                float(interval_us),
                0,
                0,
                0,
                0,
                0,
            )
        except Exception as exc:
            logger.debug('Failed to set message interval for %s: %s', message_id, exc)

    def _configure_message_intervals(self) -> None:
        rates = {
            getattr(mavutil.mavlink, 'MAVLINK_MSG_ID_ATTITUDE', 30): max(80.0, 1.0 / self.telemetry_interval),
            getattr(mavutil.mavlink, 'MAVLINK_MSG_ID_HIGHRES_IMU', 105): 50.0,
            getattr(mavutil.mavlink, 'MAVLINK_MSG_ID_SERVO_OUTPUT_RAW', 36): 50.0,
            getattr(mavutil.mavlink, 'MAVLINK_MSG_ID_GLOBAL_POSITION_INT', 33): 20.0,
            getattr(mavutil.mavlink, 'MAVLINK_MSG_ID_GPS_RAW_INT', 24): 10.0,
            getattr(mavutil.mavlink, 'MAVLINK_MSG_ID_SYS_STATUS', 1): 10.0,
        }

        for message_id, rate_hz in rates.items():
            self._set_message_interval(int(message_id), float(rate_hz))

        logger.info('Requested FCU message intervals for attitude and output telemetry')

    def connect(self, timeout: float = 8.0) -> bool:
        try:
            logger.info('Connecting to Pixhawk on %s @ %d', self.serial_port, self.baudrate)
            self.master = mavutil.mavlink_connection(
                self.serial_port,
                baud=self.baudrate,
                autoreconnect=True,
                source_system=255,
            )

            hb = self.master.wait_heartbeat(timeout=timeout)
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
            self._ensure_direct_motor_outputs()
            self._configure_message_intervals()
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

        logger.info('Bridge disconnected')

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
        last_at = self._last_log_sent_at.get(log_key, 0.0)
        if min_interval > 0.0 and now - last_at < min_interval:
            return

        self._last_log_sent_at[log_key] = now
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

    def _send_motor_command(self, channel: int, pwm: int) -> bool:
        if self.master is None:
            return False

        if channel < 1 or channel > 32:
            logger.error('Invalid channel %s for motor command', channel)
            return False

        pwm_value = int(max(1000, min(2000, pwm)))

        try:
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                MAV_CMD_DO_SET_SERVO,
                0,
                float(channel),
                float(pwm_value),
                0,
                0,
                0,
                0,
                0,
            )
            logger.info('Motor command sent: channel=%d pwm=%d', channel, pwm_value)
            return True
        except Exception as exc:
            logger.error('Failed to send motor command: %s', exc)
            return False

    def _send_rc_override(self, overrides: Dict[int, int]) -> bool:
        if self.master is None:
            return False

        channels = [65535] * 8
        for channel, pwm in overrides.items():
            if channel < 1 or channel > 8:
                logger.error('Invalid RC input channel %s for override', channel)
                return False
            channels[channel - 1] = int(max(1000, min(2000, pwm)))

        try:
            self.master.mav.rc_channels_override_send(
                self.target_system,
                self.target_component,
                channels[0],
                channels[1],
                channels[2],
                channels[3],
                channels[4],
                channels[5],
                channels[6],
                channels[7],
            )
            return True
        except Exception as exc:
            logger.error('Failed to send RC override: %s', exc)
            return False

    def _send_rover_drive(self, throttle_channel: int, throttle_pwm: int, steering_channel: int, steering_pwm: int) -> bool:
        if throttle_channel == steering_channel:
            logger.error('Rover throttle/steering channels must be different')
            return False

        if not self._send_rc_override({
            int(throttle_channel): int(throttle_pwm),
            int(steering_channel): int(steering_pwm),
        }):
            return False

        logger.info(
            'Rover drive input sent: throttle ch%d=%d, steering ch%d=%d',
            int(throttle_channel),
            int(throttle_pwm),
            int(steering_channel),
            int(steering_pwm),
        )
        return True

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

    def _handle_command_packet(self, raw_data: bytes) -> None:
        try:
            packet = json.loads(raw_data.decode('utf-8'))
        except Exception as exc:
            logger.error('Invalid command packet: %s', exc)
            return

        command = str(packet.get('command', '')).upper().strip()
        params = packet.get('params', {}) or {}

        if command == 'MOTOR_CONTROL':
            channel = int(params.get('channel', 0))
            pwm = int(params.get('pwm', 1500))
            self._send_motor_command(channel, pwm)
            return

        if command == 'ROVER_DRIVE':
            throttle_channel = int(params.get('throttleChannel', 3))
            throttle_pwm = int(params.get('throttlePwm', 1500))
            steering_channel = int(params.get('steeringChannel', 1))
            steering_pwm = int(params.get('steeringPwm', 1500))
            self._send_rover_drive(throttle_channel, throttle_pwm, steering_channel, steering_pwm)
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
            channels = params.get('channels', []) or []
            pwm = int(params.get('pwm', 1000))

            for channel in channels:
                try:
                    self._send_motor_command(int(channel), pwm)
                except (TypeError, ValueError):
                    logger.warning('Invalid emergency stop channel: %s', channel)

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

        if msg_type == 'HEARTBEAT':
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
            eph = int(getattr(msg, 'eph', 65535) or 65535)
            self.flight_data.gps_hdop = eph / 100.0 if eph != 65535 else self.flight_data.gps_hdop

            gps_fix_ok = self.flight_data.gps_satellites >= 4
            if self._last_gps_fix_reported is None or gps_fix_ok != self._last_gps_fix_reported:
                self._send_node_log(
                    'INFO' if gps_fix_ok else 'WARNING',
                    f'GPS fix {"acquired" if gps_fix_ok else "lost"} (sat={self.flight_data.gps_satellites}, hdop={self.flight_data.gps_hdop:.1f})',
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

        elif msg_type == 'ATTITUDE':
            self.flight_data.roll = math.degrees(float(getattr(msg, 'roll', 0.0)))
            self.flight_data.pitch = math.degrees(float(getattr(msg, 'pitch', 0.0)))
            self.flight_data.yaw = math.degrees(float(getattr(msg, 'yaw', 0.0)))

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

            self._parse_message(msg)

    def _telemetry_payload(self) -> Dict[str, Any]:
        temperature_payload: Dict[str, Any] = {}
        if self.flight_data.board_temperature is not None:
            temperature_payload['flightController'] = self.flight_data.board_temperature
        if self.flight_data.motor_left_temperature is not None:
            temperature_payload['motorLeft'] = self.flight_data.motor_left_temperature
        if self.flight_data.motor_right_temperature is not None:
            temperature_payload['motorRight'] = self.flight_data.motor_right_temperature

        return {
            'position': {
                'lat': self.flight_data.latitude,
                'lon': self.flight_data.longitude,
                'alt': self.flight_data.altitude,
            },
            'attitude': {
                'roll': self.flight_data.roll,
                'pitch': self.flight_data.pitch,
                'yaw': self.flight_data.yaw,
            },
            'velocity': {
                'vx': self.flight_data.vx,
                'vy': self.flight_data.vy,
                'vz': self.flight_data.vz,
            },
            'battery': {
                'voltage': self.flight_data.battery_voltage,
                'current': self.flight_data.battery_current,
                'percentage': self.flight_data.battery_percentage,
            },
            'servoOutputs': self.flight_data.servo_outputs,
            'temperature': temperature_payload,
            'gps': {
                'satellites': self.flight_data.gps_satellites,
                'hdop': self.flight_data.gps_hdop,
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
            'systemStatus': self.flight_data.system_status,
            'armed': self.flight_data.armed,
        }

    def run(self) -> None:
        logger.info('Command listener: udp://%s:%d', self.bridge_host, self.command_port)
        logger.info('Telemetry target: udp://127.0.0.1:%d', self.telemetry_port)

        try:
            while self.connected:
                now = time.time()

                self._poll_commands()
                self._poll_mavlink()

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