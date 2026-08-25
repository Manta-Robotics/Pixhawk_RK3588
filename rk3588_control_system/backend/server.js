/**
 * RK3588 + Pixhawk Rover Control System - Node.js Backend Server
 * Direct web access + Socket.io + UDP bridge for Python MAVLink process.
 */

import express from 'express';
import { Server } from 'socket.io';
import { createServer } from 'http';
import path from 'path';
import { fileURLToPath } from 'url';
import fs from 'fs';
import bodyParser from 'body-parser';
import dgram from 'dgram';
import { spawn, spawnSync } from 'child_process';
import net from 'net';
import os from 'os';
import http from 'http';
import https from 'https';
import { Bonjour } from 'bonjour-service';
import { validateSystemConfig } from '../scripts/validate_config.mjs';
import { isSameOriginRequest, rejectCrossOrigin } from './origin_policy.js';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const PROJECT_ROOT = path.resolve(__dirname, '..');

function readJsonFile(relativePath, fallback, options = {}) {
  const filePath = path.join(PROJECT_ROOT, relativePath);
  try {
    return JSON.parse(fs.readFileSync(filePath, 'utf8'));
  } catch (error) {
    const message = `[Config] Failed to read ${relativePath}: ${error.message}`;
    if (options.required) throw new Error(message);
    console.error(message);
    return fallback;
  }
}

const config = readJsonFile('config/system.config.json', {}, { required: true });
const configErrors = validateSystemConfig(config);
if (configErrors.length) {
  throw new Error(`[Config] Invalid system.config.json:\n- ${configErrors.join('\n- ')}`);
}
const motorConfig = readJsonFile('config/motor_config.json', { motors: [] });

const LOGS_DIR = path.resolve(PROJECT_ROOT, config.logs_dir || './logs');
const SYSTEM_LOG_FILE = path.join(LOGS_DIR, 'system.log');
const FLIGHT_CSV_FILE = path.join(LOGS_DIR, 'flight_data.csv');
const RECORDINGS_DIR = path.resolve(PROJECT_ROOT, (config.recordings && config.recordings.dir) || './recordings/gimbal');
const THERMAL_CLASS_DIR = '/sys/class/thermal';
const NETWORK_CLASS_DIR = '/sys/class/net';

const WEB_HOST = config.web_host || '0.0.0.0';
const WEB_PORT = Number(config.web_port || 3000);
const BRIDGE_HOST = config.bridge_host || '127.0.0.1';
const BRIDGE_COMMAND_PORT = Number(config.bridge_command_port || 14551);
const BRIDGE_TELEMETRY_PORT = Number(config.bridge_telemetry_port || 14552);
const SNAPSHOT_PORT = Number((config.hotspot && config.hotspot.camera_port) || 8090);
const PREFERRED_WIRELESS_INTERFACE = String(config.wireless_interface || 'wlan0');
const CAN_INTERFACE = String(config.can_interface || 'can0');
const MANTA_HOST = String(config.manta_host || 'manta.local');
const MANTA_SERVICE_NAME = String(config.manta_service_name || 'Manta');
const cameraConfig = config.camera || {};
const gimbalConfig = config.gimbal || {};

const PWM_MIN = Number(config.min_motor_pwm || 1000);
const PWM_MAX = Number(config.max_motor_pwm || 2000);
const PWM_CENTER = Number(config.default_motor_pwm || 1500);

const ROVER_THROTTLE_MIN = Number(config.rover_throttle_min ?? -100);
const ROVER_THROTTLE_MAX = Number(config.rover_throttle_max ?? 100);
const ROVER_STEERING_MIN = Number(config.rover_steering_min ?? -45);
const ROVER_STEERING_MAX = Number(config.rover_steering_max ?? 45);
const ROVER_LEFT_CHANNEL = Number(config.rover_left_channel ?? 1);
const ROVER_RIGHT_CHANNEL = Number(config.rover_right_channel ?? 3);
const ROVER_STEERING_INPUT_CHANNEL = Number(config.rover_steering_input_channel ?? 1);
const ROVER_THROTTLE_INPUT_CHANNEL = Number(config.rover_throttle_input_channel ?? 3);
const HARDWARE_FEEDBACK_TIMEOUT_MS = Math.max(1000, Number(config.hardware_feedback_timeout_ms || 3000));
const IMU_CALIBRATION_POSITIONS = {
  1: 'LEVEL',
  2: 'LEFT',
  3: 'RIGHT',
  4: 'NOSEDOWN',
  5: 'NOSEUP',
  6: 'BACK'
};

if (!fs.existsSync(LOGS_DIR)) {
  fs.mkdirSync(LOGS_DIR, { recursive: true });
}

if (!fs.existsSync(RECORDINGS_DIR)) {
  fs.mkdirSync(RECORDINGS_DIR, { recursive: true });
}

if (!fs.existsSync(FLIGHT_CSV_FILE)) {
  fs.writeFileSync(
    FLIGHT_CSV_FILE,
    'timestamp,lat,lon,alt,roll,pitch,yaw,vx,vy,vz,voltage,current,percentage,flight_mode,armed\n'
  );
}

const enabledChannels = new Set(
  (motorConfig.motors || [])
    .filter((motor) => motor.enabled !== false)
    .map((motor) => Number(motor.channel))
    .filter((channel) => Number.isInteger(channel) && channel >= 1 && channel <= 8)
);

if (enabledChannels.size === 0) {
  for (let channel = 1; channel <= 8; channel += 1) {
    enabledChannels.add(channel);
  }
}

if (!enabledChannels.has(ROVER_LEFT_CHANNEL) || !enabledChannels.has(ROVER_RIGHT_CHANNEL)) {
  const leftEnabled = enabledChannels.has(ROVER_LEFT_CHANNEL);
  const rightEnabled = enabledChannels.has(ROVER_RIGHT_CHANNEL);
  console.warn(
    `[Config] Rover channels not enabled in motor_config: left=${ROVER_LEFT_CHANNEL} (${leftEnabled}), right=${ROVER_RIGHT_CHANNEL} (${rightEnabled})`
  );
}

const app = express();
app.disable('x-powered-by');
const httpServer = createServer(app);
const io = new Server(httpServer, {
  allowRequest: (request, callback) => callback(null, isSameOriginRequest(request)),
  transports: ['polling', 'websocket'],
  perMessageDeflate: false
});

const commandSocket = dgram.createSocket('udp4');
const telemetrySocket = dgram.createSocket('udp4');
const FFMPEG_BIN = process.env.FFMPEG_BIN || 'ffmpeg';
const AMAP_JS_KEY = String(process.env.MANTA_AMAP_JS_KEY || '').trim();
const AMAP_SECURITY_CODE = String(process.env.MANTA_AMAP_SECURITY_CODE || '').trim();

const GIMBAL_FRAME_LENGTH = 44;
const GIMBAL_COMMAND_HZ = Math.max(1, Number(gimbalConfig.command_hz || 25));
const GIMBAL_COMMAND_INTERVAL_MS = Math.round(1000 / GIMBAL_COMMAND_HZ);
const GIMBAL_SERIAL_PORT = String(gimbalConfig.serial_port || '/dev/ttyS3');
const GIMBAL_BAUD_RATE = Number(gimbalConfig.baud_rate || 115200);
const GIMBAL_AXIS = gimbalConfig.axis || {};
const GIMBAL_MAX_PIXEL_X = Math.max(1, Number(GIMBAL_AXIS.max_pixel_x || 960));
const GIMBAL_MAX_PIXEL_Y = Math.max(1, Number(GIMBAL_AXIS.max_pixel_y || 540));
const GIMBAL_MAX_RATE_DPS = Math.max(1, Number(GIMBAL_AXIS.max_rate_dps || 100));
const GIMBAL_YAW_SIGN = Number(GIMBAL_AXIS.yaw_sign || 1) < 0 ? -1 : 1;
const GIMBAL_PITCH_SIGN = Number(GIMBAL_AXIS.pitch_sign || 1) < 0 ? -1 : 1;
const GIMBAL_CLICK_RATE_GAIN = Number(GIMBAL_AXIS.click_rate_gain || 0.025);
const GIMBAL_CLICK_RATE_DPS = Math.max(1, Number(GIMBAL_AXIS.click_rate_dps || 65));
const GIMBAL_CLICK_YAW_FOV_DEG = Math.max(1, Number(GIMBAL_AXIS.click_yaw_fov_deg || 90));
const GIMBAL_CLICK_PITCH_FOV_DEG = Math.max(1, Number(GIMBAL_AXIS.click_pitch_fov_deg || 54));
const GIMBAL_CLICK_DURATION_SCALE = Math.max(0.1, Number(GIMBAL_AXIS.click_duration_scale || 1.0));
const GIMBAL_CLICK_EXTRA_MS = Math.max(0, Number(GIMBAL_AXIS.click_extra_ms || 0));
const GIMBAL_TRACK_RATE_GAIN = Number(GIMBAL_AXIS.track_rate_gain || 0.06);
const GIMBAL_TRACK_ANGLE_GAIN = Number(GIMBAL_AXIS.track_angle_gain || 2.0);
const GIMBAL_DEADZONE_PX = Math.max(0, Number(GIMBAL_AXIS.deadzone_px || 12));
const GIMBAL_TRACK_HOLD_ZONE_PX = Math.max(GIMBAL_DEADZONE_PX, Number(GIMBAL_AXIS.track_hold_zone_px || 42));
const GIMBAL_TRACK_FAST_ZONE_PX = Math.max(GIMBAL_TRACK_HOLD_ZONE_PX + 1, Number(GIMBAL_AXIS.track_fast_zone_px || 260));
const GIMBAL_RATE_SLEW_DPS = Math.max(1, Number(GIMBAL_AXIS.rate_slew_dps || 8));
const GIMBAL_TRACK_HOLD_MS = Math.max(120, Number(GIMBAL_AXIS.track_hold_ms || 340));
const GIMBAL_TRACK_PULSE_MS = Math.max(60, Number(GIMBAL_AXIS.track_pulse_ms || 120));
const GIMBAL_TRACK_COOLDOWN_MS = Math.max(80, Number(GIMBAL_AXIS.track_cooldown_ms || 380));
const GIMBAL_TRACK_CONFIRM_FRAMES = Math.max(1, Number(GIMBAL_AXIS.track_confirm_frames || 2));
const GIMBAL_TRACK_MIN_RATE_DPS = Math.max(1, Number(GIMBAL_AXIS.track_min_rate_dps || 7));
const GIMBAL_TRACK_MAX_PULSE_RATE_DPS = Math.max(GIMBAL_TRACK_MIN_RATE_DPS, Number(GIMBAL_AXIS.track_max_pulse_rate_dps || 24));
const GIMBAL_TRACK_CONTROL_MODE = String(GIMBAL_AXIS.track_control_mode || 'smooth_rate').trim().toLowerCase();
const GIMBAL_TRACK_TARGET_TIMEOUT_MS = Math.max(200, Number(GIMBAL_AXIS.track_target_timeout_ms || 650));
const GIMBAL_TRACK_RATE_STEP_DPS = Math.max(0.2, Number(GIMBAL_AXIS.track_rate_step_dps || 1.2));
const GIMBAL_TRACK_MINOR_AXIS_SCALE = Math.max(0, Math.min(1, Number(GIMBAL_AXIS.track_minor_axis_scale ?? 0.45)));
const GIMBAL_TRACK_HOLD_ENTER_PX = Math.max(2, Number(GIMBAL_AXIS.track_hold_enter_px || 24));
const GIMBAL_TRACK_HOLD_EXIT_PX = Math.max(GIMBAL_TRACK_HOLD_ENTER_PX + 1, Number(GIMBAL_AXIS.track_hold_exit_px || 44));
const GIMBAL_TRACK_HOLD_ENTER_X_PX = Math.max(2, Number(GIMBAL_AXIS.track_hold_enter_x_px || GIMBAL_TRACK_HOLD_ENTER_PX));
const GIMBAL_TRACK_HOLD_ENTER_Y_PX = Math.max(2, Number(GIMBAL_AXIS.track_hold_enter_y_px || GIMBAL_TRACK_HOLD_ENTER_PX));
const GIMBAL_TRACK_HOLD_EXIT_X_PX = Math.max(GIMBAL_TRACK_HOLD_ENTER_X_PX + 1, Number(GIMBAL_AXIS.track_hold_exit_x_px || GIMBAL_TRACK_HOLD_EXIT_PX));
const GIMBAL_TRACK_HOLD_EXIT_Y_PX = Math.max(GIMBAL_TRACK_HOLD_ENTER_Y_PX + 1, Number(GIMBAL_AXIS.track_hold_exit_y_px || GIMBAL_TRACK_HOLD_EXIT_PX));
const GIMBAL_FACE_TRACK = gimbalConfig.face || {};
const GIMBAL_FACE_TRACK_HOLD_ENTER_X_PX = Math.max(2, Number(GIMBAL_FACE_TRACK.track_hold_enter_x_px || GIMBAL_TRACK_HOLD_ENTER_X_PX));
const GIMBAL_FACE_TRACK_HOLD_ENTER_Y_PX = Math.max(2, Number(GIMBAL_FACE_TRACK.track_hold_enter_y_px || GIMBAL_TRACK_HOLD_ENTER_Y_PX));
const GIMBAL_FACE_TRACK_HOLD_EXIT_X_PX = Math.max(GIMBAL_FACE_TRACK_HOLD_ENTER_X_PX + 1, Number(GIMBAL_FACE_TRACK.track_hold_exit_x_px || GIMBAL_TRACK_HOLD_EXIT_X_PX));
const GIMBAL_FACE_TRACK_HOLD_EXIT_Y_PX = Math.max(GIMBAL_FACE_TRACK_HOLD_ENTER_Y_PX + 1, Number(GIMBAL_FACE_TRACK.track_hold_exit_y_px || GIMBAL_TRACK_HOLD_EXIT_Y_PX));
const GIMBAL_TRACK_HOLD_SPEED_PX_S = Math.max(1, Number(GIMBAL_AXIS.track_hold_speed_px_s || 55));
const GIMBAL_TRACK_HOLD_ENTER_SPEED_PX_S = Math.max(1, Number(GIMBAL_AXIS.track_hold_enter_speed_px_s || GIMBAL_TRACK_HOLD_SPEED_PX_S));
const GIMBAL_TRACK_HOLD_EXIT_SPEED_PX_S = Math.max(GIMBAL_TRACK_HOLD_ENTER_SPEED_PX_S + 1, Number(GIMBAL_AXIS.track_hold_exit_speed_px_s || GIMBAL_TRACK_HOLD_SPEED_PX_S * 1.25));
const GIMBAL_TRACK_NEAR_GAIN = Math.max(0.1, Number(GIMBAL_AXIS.track_near_angle_gain || 0.9));
const GIMBAL_TRACK_FAR_GAIN = Math.max(GIMBAL_TRACK_NEAR_GAIN, Number(GIMBAL_AXIS.track_far_angle_gain || GIMBAL_TRACK_ANGLE_GAIN));
const GIMBAL_TRACK_FEEDFORWARD_GAIN = Math.max(0, Number(GIMBAL_AXIS.track_feedforward_gain ?? 0.28));
const GIMBAL_TRACK_VELOCITY_ALPHA = clamp(Number(GIMBAL_AXIS.track_velocity_alpha ?? 0.62), 0.05, 1);
const GIMBAL_TRACK_VELOCITY_DEADBAND_PX_S = Math.max(0, Number(GIMBAL_AXIS.track_velocity_deadband_px_s ?? 18));
const GIMBAL_TRACK_VELOCITY_DECAY = clamp(Number(GIMBAL_AXIS.track_velocity_decay ?? 0.86), 0.1, 1);
const GIMBAL_TRACK_GYRO_DAMPING = Math.max(0, Number(GIMBAL_AXIS.track_gyro_damping ?? 0.55));
const GIMBAL_TRACK_PREDICTION_MS = Math.max(0, Number(GIMBAL_AXIS.track_prediction_ms || 180));
const GIMBAL_TRACK_LATENCY_LEAD_MS = Math.max(0, Number(GIMBAL_AXIS.track_latency_lead_ms || 0));
const GIMBAL_TRACK_DETECTOR_AGE_LEAD = clamp(Number(GIMBAL_AXIS.track_detector_age_lead ?? 0), 0, 1.5);
const GIMBAL_TRACK_SPEED_LEAD_MS = Math.max(0, Number(GIMBAL_AXIS.track_speed_lead_ms || 0));
const GIMBAL_TRACK_FAST_SPEED_PX_S = Math.max(1, Number(GIMBAL_AXIS.track_fast_speed_px_s || 900));
const GIMBAL_TRACK_RECOVERY_MS = Math.max(500, Number(GIMBAL_AXIS.track_recovery_ms || 2000));
const GIMBAL_TRACK_PREDICTION_MAX_RATE_DPS = Math.max(1, Number(GIMBAL_AXIS.track_prediction_max_rate_dps || 14));
const GIMBAL_TRACK_MAX_ACCEL_DPS2 = Math.max(1, Number(GIMBAL_AXIS.track_max_accel_dps2 || 120));
const GIMBAL_TRACK_MAX_JERK_DPS3 = Math.max(1, Number(GIMBAL_AXIS.track_max_jerk_dps3 || 720));
const GIMBAL_TRACK_BRAKE_ACCEL_DPS2 = Math.max(GIMBAL_TRACK_MAX_ACCEL_DPS2, Number(GIMBAL_AXIS.track_brake_accel_dps2 || 220));
const GIMBAL_TRACK_BRAKE_JERK_DPS3 = Math.max(GIMBAL_TRACK_MAX_JERK_DPS3, Number(GIMBAL_AXIS.track_brake_jerk_dps3 || 1400));
const GIMBAL_TRACK_RESPONSE_SECONDS = Math.max(0.04, Number(GIMBAL_AXIS.track_response_seconds || 0.16));
const GIMBAL_TRACK_COUNTER_BRAKE_MS = clamp(Math.round(Number(GIMBAL_AXIS.track_counter_brake_ms || 120)), 0, 360);
const GIMBAL_TRACK_OUTPUT_DEADBAND_DPS = Math.max(0, Number(GIMBAL_AXIS.track_output_deadband_dps ?? 0));
const GIMBAL_TRACK_COUNTER_BRAKE_GAIN = Math.max(0, Number(GIMBAL_AXIS.track_counter_brake_gain ?? 0.18));
const GIMBAL_TRACK_COUNTER_BRAKE_MIN_DPS = Math.max(0.5, Number(GIMBAL_AXIS.track_counter_brake_min_dps || 7));
const GIMBAL_TRACK_COUNTER_BRAKE_MAX_DPS = Math.max(GIMBAL_TRACK_COUNTER_BRAKE_MIN_DPS, Number(GIMBAL_AXIS.track_counter_brake_max_dps || 18));
const GIMBAL_YAW_MIN_DEG = Number(GIMBAL_AXIS.yaw_min_deg ?? -150);
const GIMBAL_YAW_MAX_DEG = Number(GIMBAL_AXIS.yaw_max_deg ?? 150);
const GIMBAL_PITCH_MIN_DEG = Number(GIMBAL_AXIS.pitch_min_deg ?? -150);
const GIMBAL_PITCH_MAX_DEG = Number(GIMBAL_AXIS.pitch_max_deg ?? 150);
const GIMBAL_SOFT_LIMIT_BRAKE_DEG = Math.max(0.5, Number(GIMBAL_AXIS.soft_limit_brake_deg || 8));
const GIMBAL_CLICK_TARGET_HOLD_MS = Math.max(40, Number(GIMBAL_AXIS.click_target_hold_ms || 120));
const GIMBAL_CLICK_CONTROL_MODE = String(GIMBAL_AXIS.click_control_mode || 'rate').trim().toLowerCase();
const GIMBAL_CLICK_HOLD_MIN_MS = Math.max(60, Number(GIMBAL_AXIS.click_hold_min_ms || 120));
const GIMBAL_CLICK_HOLD_MAX_MS = Math.max(GIMBAL_CLICK_HOLD_MIN_MS, Number(GIMBAL_AXIS.click_hold_max_ms || 520));
const gimbalCalibration = gimbalConfig.calibration || {};
const GIMBAL_CALIB_WIDTH = Math.max(1, Number(gimbalCalibration.width || 1920));
const GIMBAL_CALIB_HEIGHT = Math.max(1, Number(gimbalCalibration.height || 1080));
const GIMBAL_CALIB_FX = Number(gimbalCalibration.fx || ((GIMBAL_CALIB_WIDTH * 0.5) / Math.tan((GIMBAL_CLICK_YAW_FOV_DEG * Math.PI / 180) * 0.5)));
const GIMBAL_CALIB_FY = Number(gimbalCalibration.fy || ((GIMBAL_CALIB_HEIGHT * 0.5) / Math.tan((GIMBAL_CLICK_PITCH_FOV_DEG * Math.PI / 180) * 0.5)));
const GIMBAL_CALIB_CX = Number(gimbalCalibration.cx ?? (GIMBAL_CALIB_WIDTH * 0.5));
const GIMBAL_CALIB_CY = Number(gimbalCalibration.cy ?? (GIMBAL_CALIB_HEIGHT * 0.5));
const GIMBAL_CALIB_DIST = Array.isArray(gimbalCalibration.distortion)
  ? gimbalCalibration.distortion.map((value) => Number(value) || 0)
  : [0, 0, 0, 0, 0];
const gimbalVideoConfig = gimbalConfig.video || {};
const GIMBAL_STREAM_PROXY_PORT = Number(gimbalVideoConfig.proxy_port || 8091);
const GIMBAL_LOCAL_STREAM_URL = String(gimbalVideoConfig.local_stream_url || `http://127.0.0.1:${GIMBAL_STREAM_PROXY_PORT}/stream.mjpg`);
const GIMBAL_VIDEO_TRANSPORT = String(gimbalVideoConfig.transport || '').trim().toLowerCase();
const GIMBAL_RTSP_INPUT = String(gimbalVideoConfig.recognition_input || gimbalVideoConfig.rtsp_input || gimbalVideoConfig.input_url || '').trim();
const GIMBAL_VIDEO_INPUT = GIMBAL_RTSP_INPUT || String(gimbalVideoConfig.input_url || '').trim();
const GIMBAL_RECORD_INPUT = String(gimbalVideoConfig.record_input || gimbalVideoConfig.recording_input || GIMBAL_VIDEO_INPUT).trim();
const GIMBAL_RECORD_STREAM_INDEX = clamp(Math.round(Number(gimbalVideoConfig.record_stream_index || 1)), 0, 3);
const GIMBAL_RECORD_STREAM_QUALITY = clamp(Math.round(Number(gimbalVideoConfig.record_stream_quality || 0)), 0, 5);
const GIMBAL_RECORD_CODEC = String(gimbalVideoConfig.record_codec || 'h264_rkmpp').trim() || 'h264_rkmpp';
const GIMBAL_RECORD_BITRATE = String(gimbalVideoConfig.record_bitrate || '12M').trim();
const gimbalFocusConfig = gimbalConfig.focus || {};
let gimbalStream = null;
let gimbalReadStream = null;
let gimbalCommandTimer = null;
let gimbalHoldUntil = 0;
let gimbalLastFrame = Buffer.alloc(GIMBAL_FRAME_LENGTH, 0);
let gimbalTxEnabled = false;
let gimbalTrackProcess = null;
let gimbalTrackRestartTimer = null;
let gimbalTrackStopRequested = false;
let gimbalLastRateX = 0;
let gimbalLastRateY = 0;
let gimbalStopFramesRemaining = 0;
let gimbalPendingHomeSource = '';
let gimbalTrackNextMoveAt = 0;
let gimbalTrackLastDirection = '';
let gimbalTrackDirectionCount = 0;
let gimbalTrackDesiredRateX = 0;
let gimbalTrackDesiredRateY = 0;
let gimbalTrackRateAccelX = 0;
let gimbalTrackRateAccelY = 0;
let gimbalTrackFilteredVx = 0;
let gimbalTrackFilteredVy = 0;
let gimbalTrackLastMovingDesiredX = 0;
let gimbalTrackLastMovingDesiredY = 0;
let gimbalTrackCounterBrakeUntil = 0;
let gimbalTrackCounterBrakeX = 0;
let gimbalTrackCounterBrakeY = 0;
let gimbalTrackCounterBrakeSettled = true;
let gimbalTrackCommandPauseUntil = 0;
let gimbalTrackTarget = null;
let gimbalTrackHolding = true;
let gimbalTrackLastTargetAt = 0;
let gimbalRxBuffer = Buffer.alloc(0);
let gimbalFeedbackParseBuffer = Buffer.alloc(0);
let gimbalLastRx = { ascii: '', hex: '', updatedAt: null };
const gimbalState = {
  enabled: Boolean(gimbalConfig.enabled),
  connected: false,
  portOpen: false,
  linkStatus: 'offline',
  transport: String(gimbalConfig.control_transport || 'uart'),
  serialPort: GIMBAL_SERIAL_PORT,
  baudRate: GIMBAL_BAUD_RATE,
  mode: 'idle',
  lastCommand: 'idle',
  lastError: '',
  lastRx: null,
  feedback: null,
  limits: {
    yawMinDeg: GIMBAL_YAW_MIN_DEG,
    yawMaxDeg: GIMBAL_YAW_MAX_DEG,
    pitchMinDeg: GIMBAL_PITCH_MIN_DEG,
    pitchMaxDeg: GIMBAL_PITCH_MAX_DEG
  },
  trackMode: 'face',
  lastTarget: null,
  trackingActive: false,
  trackWorkerActive: false,
  trackStatus: { locked: false, status: 'idle', message: 'idle', detections: 0, updatedAt: null },
  videoSource: String(gimbalVideoConfig.source_url || '/api/gimbal/stream'),
  videoTransport: 'rtsp',
  videoInput: GIMBAL_VIDEO_INPUT,
  updatedAt: Date.now()
};
const GIMBAL_AUTO_CONNECT = gimbalConfig.auto_connect === true;
const GIMBAL_AUTO_HOME_ON_CONNECT = gimbalConfig.auto_home_on_connect === true;

function updateGimbalDiagnostics() {
  const now = Date.now();
  const feedbackUpdatedAt = Number(gimbalState.feedback && gimbalState.feedback.updatedAt) || 0;
  const feedbackFresh = Boolean(
    feedbackUpdatedAt &&
    gimbalState.feedback &&
    gimbalState.feedback.checksumValid !== false &&
    now - feedbackUpdatedAt <= HARDWARE_FEEDBACK_TIMEOUT_MS
  );
  gimbalState.portOpen = Boolean(gimbalStream);
  gimbalState.connected = Boolean(gimbalState.portOpen && feedbackFresh);
  gimbalState.linkStatus = gimbalState.connected ? 'feedback' : gimbalState.portOpen ? 'port_only' : 'offline';

  if (!fs.existsSync(GIMBAL_SERIAL_PORT)) {
    gimbalState.lastError = `${GIMBAL_SERIAL_PORT} not present; reboot after enabling UART3`;
    gimbalState.connected = false;
    gimbalState.portOpen = false;
    gimbalState.linkStatus = 'missing';
  } else if (gimbalState.portOpen && !feedbackFresh) {
    gimbalState.lastError = `No valid gimbal feedback within ${HARDWARE_FEEDBACK_TIMEOUT_MS} ms`;
  } else if (gimbalState.connected && /^No valid gimbal feedback/.test(gimbalState.lastError)) {
    gimbalState.lastError = '';
  }
}

function createDefaultImuCalibrationState() {
  return {
    active: false,
    mode: 'IDLE',
    status: 'IDLE',
    step: '',
    stepCode: null,
    instructions: 'Idle',
    progress: null,
    lastAckCommand: null,
    lastAckResult: '',
    updatedAt: null
  };
}

function listNetworkInterfaces() {
  try {
    return fs.readdirSync(NETWORK_CLASS_DIR).filter(Boolean);
  } catch (_error) {
    return [];
  }
}

function readInterfaceState(name) {
  if (!name) {
    return { interface: '', present: false, state: 'missing', online: false, type: 'unknown' };
  }

  const basePath = path.join(NETWORK_CLASS_DIR, name);
  if (!fs.existsSync(basePath)) {
    return { interface: name, present: false, state: 'missing', online: false, type: 'unknown' };
  }

  let state = 'unknown';
  try {
    state = fs.readFileSync(path.join(basePath, 'operstate'), 'utf8').trim() || 'unknown';
  } catch (_error) {
    state = 'unknown';
  }

  const isWireless = fs.existsSync(path.join(basePath, 'wireless')) || name.startsWith('wl') || name.startsWith('p2p');
  const isCan = name.startsWith('can');
  const isEthernet = name.startsWith('eth') || name.startsWith('en');

  return {
    interface: name,
    present: true,
    state,
    online: state === 'up' || state === 'unknown' || state === 'dormant',
    type: isWireless ? 'wireless' : isCan ? 'can' : isEthernet ? 'ethernet' : 'other'
  };
}

function readInterfaceIPv4(name) {
  if (!name) {
    return '';
  }

  const interfaces = os.networkInterfaces();
  const entries = Array.isArray(interfaces[name]) ? interfaces[name] : [];
  const ipv4 = entries.find((entry) => entry && entry.family === 'IPv4' && entry.internal === false);
  return ipv4 && ipv4.address ? ipv4.address : '';
}

function buildAccessUrls(connectivity) {
  const urls = [];
  const seen = new Set();

  const candidates = [
    {
      label: 'wireless',
      ip: String((config.hotspot && config.hotspot.portal_ip) || ''),
      online: Boolean(connectivity && connectivity.wireless && connectivity.wireless.online)
    },
    {
      label: 'ethernet',
      ip: readInterfaceIPv4(connectivity && connectivity.ethernet && connectivity.ethernet.interface),
      online: Boolean(connectivity && connectivity.ethernet && connectivity.ethernet.online)
    }
  ];

  candidates.forEach((candidate) => {
    const ip = String(candidate.ip || '').trim();
    if (!candidate.online || !ip || seen.has(ip)) {
      return;
    }

    seen.add(ip);
    urls.push({
      label: candidate.label,
      ip,
      dashboardUrl: `http://${ip}:${WEB_PORT}`,
      cameraUrl: `http://${ip}:${WEB_PORT}/api/camera/stream`
    });
  });

  return urls;
}

function detectWirelessInterface() {
  const interfaces = listNetworkInterfaces();
  const wirelessInterfaces = interfaces.filter((name) => {
    const basePath = path.join(NETWORK_CLASS_DIR, name);
    return fs.existsSync(path.join(basePath, 'wireless')) || name.startsWith('wl') || name.startsWith('p2p');
  });

  const activeWireless = wirelessInterfaces.find((name) => readInterfaceState(name).online);
  if (activeWireless) {
    return activeWireless;
  }

  if (interfaces.includes(PREFERRED_WIRELESS_INTERFACE)) {
    return PREFERRED_WIRELESS_INTERFACE;
  }

  return wirelessInterfaces[0] || PREFERRED_WIRELESS_INTERFACE;
}

function detectEthernetInterface() {
  return listNetworkInterfaces().find((name) => name.startsWith('eth') || name.startsWith('en')) || 'eth0';
}

function inspectHostname(hostname) {
  if (!hostname) {
    return { hostname: '', resolvable: false, assumed: false, isLocalName: false, matchesLocalHost: false };
  }

  const localHostname = String(os.hostname() || '').trim().toLowerCase();
  const normalized = String(hostname).trim().toLowerCase();
  const matchesLocalHost = normalized === localHostname || normalized === `${localHostname}.local`;

  if (hostname === 'localhost' || net.isIP(hostname)) {
    return { hostname, resolvable: true, assumed: true, isLocalName: hostname === 'localhost', matchesLocalHost };
  }

  const isLocalName = hostname.endsWith('.local');
  return {
    hostname,
    resolvable: false,
    assumed: false,
    isLocalName,
    matchesLocalHost
  };
}

function readConnectivityState() {
  const wireless = readInterfaceState(detectWirelessInterface());
  const ethernet = readInterfaceState(detectEthernetInterface());

  return {
    wireless: {
      ...wireless,
      ipv4: readInterfaceIPv4(wireless.interface)
    },
    ethernet: {
      ...ethernet,
      ipv4: readInterfaceIPv4(ethernet.interface)
    },
    can: readInterfaceState(CAN_INTERFACE)
  };
}

function readLocalVideoDevices() {
  const videoClassDir = '/sys/class/video4linux';

  try {
    return fs.readdirSync(videoClassDir)
      .filter((entry) => entry.startsWith('video'))
      .map((entry) => {
        const nameFile = path.join(videoClassDir, entry, 'name');
        let name = '';

        try {
          name = fs.readFileSync(nameFile, 'utf8').trim();
        } catch (_error) {
          name = '';
        }

        return {
          device: `/dev/${entry}`,
          name
        };
      });
  } catch (_error) {
    return [];
  }
}

function buildDirectCameraUrl(req, pathname) {
  const forwardedHost = String(req && req.headers && req.headers['x-forwarded-host'] || '').trim();
  const requestHost = String(req && req.get && req.get('host') || '').trim();
  const host = forwardedHost || requestHost;

  if (!host) {
    return '';
  }

  const parts = host.split(':');
  const hostname = parts[0] || host;
  const protocol = req && req.protocol === 'https' ? 'https' : 'http';
  return `${protocol}://${hostname}:${SNAPSHOT_PORT}${pathname}`;
}

function readLocalCameraHealth() {
  const url = `http://127.0.0.1:${SNAPSHOT_PORT}/healthz`;
  try {
    const result = spawnSync('curl', ['-sS', '--max-time', '0.6', url], {
      encoding: 'utf8',
      maxBuffer: 1024 * 1024
    });
    if (result.status !== 0) {
      const message = String(result.stderr || result.stdout || '').trim() || `Camera health check failed with exit ${result.status}`;
      return { ok: false, message };
    }
    const payload = JSON.parse(String(result.stdout || '{}'));
    return {
      ok: Boolean(payload.ok),
      message: String(payload.lastError || payload.message || ''),
      cachedDevice: String(payload.cachedDevice || ''),
      cachedName: String(payload.cachedName || ''),
      lastFrameAgeSeconds: typeof payload.lastFrameAgeSeconds === 'number' && Number.isFinite(payload.lastFrameAgeSeconds)
        ? payload.lastFrameAgeSeconds
        : null
    };
  } catch (error) {
    return { ok: false, message: error.message };
  }
}

function readCameraState(connectivity, req = null) {
  const isLocalCameraTransport = String(cameraConfig.transport || '').trim().toLowerCase() === 'local';
  const directStreamUrl = isLocalCameraTransport ? '' : buildDirectCameraUrl(req, '/stream.mjpg');
  const directOpenUrl = isLocalCameraTransport ? '' : buildDirectCameraUrl(req, '/stream.mjpg');
  const sourceUrl = String(directStreamUrl || cameraConfig.source_url || `http://${MANTA_HOST}:8080/stream`);
  const openUrl = String(directOpenUrl || cameraConfig.open_url || `http://${MANTA_HOST}:8080`);
  const localVideoDevices = readLocalVideoDevices();
  const localCameraDevices = localVideoDevices.filter((entry) => entry.name && !entry.name.toLowerCase().includes('hdmirx'));
  const overlay = String(cameraConfig.overlay || '');
  const sensor = String(cameraConfig.sensor || 'camera');
  const port = String(cameraConfig.port || '');
  const localHealth = isLocalCameraTransport ? readLocalCameraHealth() : null;
  const isLocalProxySource = sourceUrl.startsWith('/');
  const usesDirectCameraUrl = Boolean(directStreamUrl);
  let hostname = '';

  try {
    hostname = new URL(sourceUrl).hostname;
  } catch (_error) {
    hostname = '';
  }

  const hostState = inspectHostname(hostname || MANTA_HOST);
  let reason = '';

  if (cameraConfig.enabled === false) {
    reason = 'Camera is disabled in config.';
  } else if (isLocalCameraTransport && localHealth && !localHealth.ok) {
    reason = localHealth.message || 'Local camera stream is not producing JPEG frames.';
  } else if (isLocalProxySource) {
    reason = 'Using the local camera proxy stream.';
  } else if (usesDirectCameraUrl) {
    reason = 'Using the direct camera stream.';
  } else if (localVideoDevices.length > 0 && localCameraDevices.length === 0) {
    reason = `Only ${localVideoDevices.map((entry) => entry.name || entry.device).join(', ')} is present. ${overlay ? `The configured overlay is ${overlay}. ` : ''}${port ? `${sensor.toUpperCase()} is configured for ${port.toUpperCase()}. ` : ''}On LubanCat camera setups the working MIPI node usually appears as rkisp_mainpath. In ubuntuEnv.txt the camera overlay must be appended to the overlays= line, not left as a standalone line. Then check the ribbon orientation, camera power, and reboot. If ${overlay || 'the configured overlay'} is not installed under /boot/firmware/dtbs/rockchip/overlay, the current kernel image does not yet provide this sensor profile.`;
  } else if (!(connectivity && connectivity.wireless && connectivity.wireless.online) && !(connectivity && connectivity.ethernet && connectivity.ethernet.online)) {
    reason = 'No active network link to Manta. Connect wlan0 or Ethernet first.';
  } else if (hostState.matchesLocalHost) {
    reason = `Camera host ${hostState.hostname || MANTA_HOST} points to this RK3588. Replace it with the real Manta IP or hostname.`;
  } else if (hostState.isLocalName) {
    reason = `Network link is up. If the image stays blank, ${hostState.hostname || MANTA_HOST} may not resolve on this link; use the Manta IP in camera.source_url.`;
  } else if (!hostState.resolvable) {
    reason = `Verify the camera host ${(hostState.hostname || MANTA_HOST)} or replace it with the Manta IP.`;
  } else {
    reason = 'Waiting for the camera stream response.';
  }

  return {
    enabled: cameraConfig.enabled !== false,
    label: String(cameraConfig.label || 'Manta Camera'),
    transport: String(cameraConfig.transport || CAN_INTERFACE),
    sensor: String(cameraConfig.sensor || ''),
    device: String(cameraConfig.device || 'auto'),
    overlay,
    canInterface: CAN_INTERFACE,
    sourceType: String(cameraConfig.source_type || 'image'),
    sourceUrl,
    openUrl,
    refreshMs: Number(cameraConfig.refresh_ms || 1500),
    online: cameraConfig.enabled !== false && (
      isLocalCameraTransport
        ? Boolean(localHealth && localHealth.ok)
        : Boolean((connectivity && connectivity.wireless && connectivity.wireless.online) || (connectivity && connectivity.ethernet && connectivity.ethernet.online))
    ),
    host: MANTA_HOST,
    hostState,
    localVideoDevices,
    localHealth,
    reason
  };
}

function refreshPeripheralState() {
  const connectivity = readConnectivityState();
  systemState.connectivity = connectivity;
  systemState.accessUrls = buildAccessUrls(connectivity);
  systemState.camera = readCameraState(connectivity);
}

const systemState = {
  isConnected: false,
  pixhawkStatus: 'disconnected',
  vehicleType: 'rover',
  telemetry: {
    position: { lat: 0, lon: 0, alt: 0, source: '', updatedAt: null },
    attitude: { roll: 0, pitch: 0, yaw: 0 },
    velocity: { vx: 0, vy: 0, vz: 0 },
    battery: { voltage: 0, current: 0, percentage: 100 },
    servoOutputs: { ch1: 0, ch2: 0, ch3: 0, ch4: 0 },
    temperature: { hostBoard: null, flightController: null, motorLeft: null, motorRight: null },
    gps: {
      satellites: 0,
      hdop: 999,
      fixType: 0,
      latitude: 0,
      longitude: 0,
      altitude: 0,
      horizontalAccuracy: null,
      verticalAccuracy: null,
      groundSpeed: null,
      course: null,
      updatedAt: null
    },
    uwb: {
      online: false,
      fresh: false,
      distanceM: null,
      azimuthDeg: null,
      elevationDeg: null,
      rawDistanceM: null,
      rawAzimuthDeg: null,
      rawElevationDeg: null,
      goodCount: 0,
      updatedAt: null,
      source: 'TELEM3_NAMED_VALUE_FLOAT',
      filter: 'scalar_kalman'
    },
    imuCalibration: createDefaultImuCalibrationState(),
    flightMode: 'MANUAL',
    systemStatus: 'STANDBY',
    armed: false
  },
  roverControl: {
    throttle: 0,
    steering: 0,
    leftPwm: PWM_CENTER,
    rightPwm: PWM_CENTER
  },
  motorStatus: {
    ch1: PWM_CENTER, ch2: PWM_CENTER, ch3: PWM_CENTER, ch4: PWM_CENTER,
    ch5: PWM_CENTER, ch6: PWM_CENTER, ch7: PWM_CENTER, ch8: PWM_CENTER
  },
  hardware: {
    pixhawk: { online: false, status: 'offline', source: 'heartbeat', lastSeenAt: null },
    motors: {
      left: { online: false, outputOnline: false, status: 'offline', channel: ROVER_LEFT_CHANNEL, outputPwm: 0, outputUpdatedAt: null, feedbackUpdatedAt: null, feedbackSource: 'none' },
      right: { online: false, outputOnline: false, status: 'offline', channel: ROVER_RIGHT_CHANNEL, outputPwm: 0, outputUpdatedAt: null, feedbackUpdatedAt: null, feedbackSource: 'none' }
    },
    gimbal: { online: false, portOpen: false, status: 'offline', source: 'feedback', lastSeenAt: null }
  },
  connectivity: readConnectivityState(),
  accessUrls: [],
  camera: readCameraState(readConnectivityState()),
  vision: { active: false, detections: { w: 0, h: 0, rects: [], t: 0 } },
  logs: []
};

refreshPeripheralState();

app.use(rejectCrossOrigin);
app.use((_req, res, next) => {
  res.setHeader('X-Content-Type-Options', 'nosniff');
  res.setHeader('X-Frame-Options', 'SAMEORIGIN');
  res.setHeader('Referrer-Policy', 'no-referrer');
  res.setHeader(
    'Content-Security-Policy',
    "default-src 'self' data: blob:; connect-src 'self' ws: wss: https://*.amap.com https://*.autonavi.com; img-src 'self' data: blob: https://*.amap.com https://*.autonavi.com; media-src 'self' blob:; style-src 'self' 'unsafe-inline' https://*.amap.com https://*.autonavi.com; script-src 'self' 'unsafe-inline' https://webapi.amap.com; worker-src 'self' blob:"
  );
  next();
});
app.use(bodyParser.json({ limit: '1mb' }));
app.use(bodyParser.urlencoded({ extended: true }));
app.get(['/', '/index.html'], (_req, res) => {
  res.setHeader('Cache-Control', 'no-store');
  res.sendFile(path.join(PROJECT_ROOT, 'frontend', 'mobile-preview-kimi-k26.html'));
});
app.use(express.static(path.join(PROJECT_ROOT, 'frontend')));

app.get('/api/map/config', (_req, res) => {
  res.setHeader('Cache-Control', 'no-store');
  res.json({
    success: true,
    data: {
      provider: 'amap',
      enabled: Boolean(AMAP_JS_KEY && AMAP_SECURITY_CODE),
      jsKey: AMAP_JS_KEY,
      coordinateSystem: 'wgs84',
      serviceHost: '/_AMapService'
    }
  });
});

app.use('/_AMapService', (req, res) => {
  if (!AMAP_SECURITY_CODE) {
    res.status(503).json({ success: false, message: 'Amap security proxy is not configured' });
    return;
  }
  if (req.method !== 'GET' || !/^\/v[34]\//.test(req.url)) {
    res.status(404).json({ success: false, message: 'Unsupported Amap proxy path' });
    return;
  }

  const target = new URL(req.url, 'https://restapi.amap.com');
  target.searchParams.set('jscode', AMAP_SECURITY_CODE);
  const upstream = https.get(target, {
    headers: { Accept: req.get('Accept') || 'application/json', 'User-Agent': 'MANTA-RK3588/1.0' },
    timeout: 8000
  }, (upstreamResponse) => {
    res.status(upstreamResponse.statusCode || 502);
    const contentType = upstreamResponse.headers['content-type'];
    if (contentType) res.setHeader('Content-Type', contentType);
    upstreamResponse.pipe(res);
  });
  upstream.on('timeout', () => upstream.destroy(new Error('Amap proxy timeout')));
  upstream.on('error', (error) => {
    if (!res.headersSent) res.status(502).json({ success: false, message: 'Amap proxy unavailable' });
    else res.end();
    addLog('WARNING', `Amap proxy error: ${error.message}`);
  });
});

const telemetryCsvBuffer = [];
let telemetryCsvFlushTimer = null;
const seenAlertLogs = new Set();

function asFiniteNumber(value, fallback = 0) {
  const numeric = Number(value);
  return Number.isFinite(numeric) ? numeric : fallback;
}

function isFreshTimestamp(value, timeoutMs = HARDWARE_FEEDBACK_TIMEOUT_MS) {
  const timestamp = Number(value) || 0;
  return Boolean(timestamp && Date.now() - timestamp <= timeoutMs);
}

function refreshHardwareState() {
  const hardware = systemState.hardware || {};
  const pixhawk = hardware.pixhawk || {};
  const pixhawkOnline = Boolean(systemState.isConnected && isFreshTimestamp(pixhawk.lastSeenAt));
  systemState.isConnected = pixhawkOnline;
  systemState.pixhawkStatus = pixhawkOnline ? 'connected' : 'disconnected';
  hardware.pixhawk = {
    ...pixhawk,
    online: pixhawkOnline,
    status: pixhawkOnline ? 'heartbeat' : 'offline',
    source: 'heartbeat'
  };

  const incomingMotors = hardware.motors || {};
  hardware.motors = {};
  for (const [side, channel] of [['left', ROVER_LEFT_CHANNEL], ['right', ROVER_RIGHT_CHANNEL]]) {
    const motor = incomingMotors[side] || {};
    const feedbackOnline = Boolean(pixhawkOnline && isFreshTimestamp(motor.feedbackUpdatedAt));
    const outputOnline = Boolean(
      pixhawkOnline &&
      isFreshTimestamp(motor.outputUpdatedAt) &&
      Number(motor.outputPwm) > 0
    );
    hardware.motors[side] = {
      ...motor,
      channel,
      online: feedbackOnline,
      outputOnline,
      status: feedbackOnline ? 'feedback' : outputOnline ? 'output_only' : 'offline',
      feedbackSource: feedbackOnline ? (motor.feedbackSource || 'esc_telemetry') : 'none'
    };
  }

  updateGimbalDiagnostics();
  hardware.gimbal = {
    online: Boolean(gimbalState.connected),
    portOpen: Boolean(gimbalState.portOpen),
    status: gimbalState.linkStatus,
    source: 'validated_status_frame',
    lastSeenAt: Number(gimbalState.feedback && gimbalState.feedback.updatedAt) || null
  };
  systemState.hardware = hardware;
  return hardware;
}

function mergeOptionalFiniteNumber(value, fallback = null) {
  if (value === null || typeof value === 'undefined' || value === '') {
    return fallback;
  }

  const numeric = Number(value);
  return Number.isFinite(numeric) ? numeric : fallback;
}

function mergeNullableFiniteNumber(source, key, fallback = null) {
  if (!source || !Object.prototype.hasOwnProperty.call(source, key)) {
    return fallback;
  }
  if (source[key] === null || source[key] === '') {
    return null;
  }
  return asFiniteNumber(source[key], fallback);
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function parseGimbalByte(value, fallback = null) {
  if (value === null || typeof value === 'undefined' || value === '') return fallback;
  const text = String(value).trim();
  const numeric = text.toLowerCase().startsWith('0x') ? Number.parseInt(text.slice(2), 16) : Number(text);
  if (!Number.isFinite(numeric)) return fallback;
  return clamp(Math.round(numeric), 0, 255);
}

function toPwm(value) {
  return clamp(Math.round(value), PWM_MIN, PWM_MAX);
}

function normalizeCalibrationType(value) {
  const normalized = String(value || 'ACCEL').trim().toUpperCase();
  return normalized === 'LEVEL' ? 'LEVEL' : normalized === 'ACCEL' ? 'ACCEL' : '';
}

function calibrationAllowedError() {
  if (!systemState.isConnected) {
    return 'FCU is offline. IMU calibration cannot start.';
  }

  if (systemState.telemetry.armed) {
    return 'Disarm the rover before running IMU calibration.';
  }

  return '';
}

function addLog(level, message) {
  const normalizedLevel = String(level || 'INFO').toUpperCase();
  const normalizedMessage = String(message || '').trim();
  const alertLevel = normalizedLevel === 'WARNING' || normalizedLevel === 'ERROR' || normalizedLevel === 'CRITICAL';
  const alertKey = /\bUWB:\s*no frame\b/i.test(normalizedMessage)
    ? `${normalizedLevel}:FCU_UWB_NO_FRAME`
    : `${normalizedLevel}:${normalizedMessage}`;
  if (alertLevel && seenAlertLogs.has(alertKey)) {
    return null;
  }
  if (alertLevel) seenAlertLogs.add(alertKey);

  const timestamp = new Date().toISOString();
  const entry = { timestamp, level: normalizedLevel, message: normalizedMessage };

  systemState.logs.push(entry);
  if (systemState.logs.length > 1000) {
    systemState.logs = systemState.logs.slice(-1000);
  }

  try {
    fs.appendFileSync(SYSTEM_LOG_FILE, `[${timestamp}] [${level}] ${message}\n`);
  } catch (error) {
    console.error(`[Log] Failed to write log file: ${error.message}`);
  }

  io.emit('log_entry', entry);
  console.log(`[${normalizedLevel}] ${normalizedMessage}`);
  return entry;
}

function emitTelemetryUpdate() {
  io.volatile.emit('telemetry_update', systemState.telemetry);
}

function flushTelemetryCsv(force = false) {
  if (telemetryCsvBuffer.length === 0) {
    return;
  }

  if (telemetryCsvFlushTimer) {
    clearTimeout(telemetryCsvFlushTimer);
    telemetryCsvFlushTimer = null;
  }

  const lines = telemetryCsvBuffer.splice(0, telemetryCsvBuffer.length);
  const payload = `${lines.join('\n')}\n`;

  try {
    if (force) {
      fs.appendFileSync(FLIGHT_CSV_FILE, payload);
      return;
    }

    fs.appendFile(FLIGHT_CSV_FILE, payload, (error) => {
      if (error) {
        addLog('ERROR', `Failed to write telemetry CSV: ${error.message}`);
      }
    });
  } catch (error) {
    addLog('ERROR', `Failed to write telemetry CSV: ${error.message}`);
  }
}

function scheduleTelemetryCsvFlush() {
  if (telemetryCsvFlushTimer) {
    return;
  }

  telemetryCsvFlushTimer = setTimeout(() => {
    telemetryCsvFlushTimer = null;
    flushTelemetryCsv(false);
  }, 250);
}

function appendTelemetryCsv(telemetry) {
  const csvLine = [
    new Date().toISOString(),
    asFiniteNumber(telemetry.position.lat, 0).toFixed(7),
    asFiniteNumber(telemetry.position.lon, 0).toFixed(7),
    asFiniteNumber(telemetry.position.alt, 0).toFixed(2),
    asFiniteNumber(telemetry.attitude.roll, 0).toFixed(2),
    asFiniteNumber(telemetry.attitude.pitch, 0).toFixed(2),
    asFiniteNumber(telemetry.attitude.yaw, 0).toFixed(2),
    asFiniteNumber(telemetry.velocity.vx, 0).toFixed(2),
    asFiniteNumber(telemetry.velocity.vy, 0).toFixed(2),
    asFiniteNumber(telemetry.velocity.vz, 0).toFixed(2),
    asFiniteNumber(telemetry.battery.voltage, 0).toFixed(2),
    asFiniteNumber(telemetry.battery.current, 0).toFixed(2),
    asFiniteNumber(telemetry.battery.percentage, 0).toFixed(0),
    telemetry.flightMode,
    telemetry.armed ? '1' : '0'
  ].join(',');

  telemetryCsvBuffer.push(csvLine);
  if (telemetryCsvBuffer.length >= 20) {
    flushTelemetryCsv(false);
    return;
  }

  scheduleTelemetryCsvFlush();
}

function sendMavlinkCommand(command, params = {}) {
  const payload = { command, params, timestamp: Date.now() };
  const buffer = Buffer.from(JSON.stringify(payload));
  commandSocket.send(buffer, BRIDGE_COMMAND_PORT, BRIDGE_HOST, (error) => {
    if (error) {
      addLog('ERROR', `Failed to send bridge command ${command}: ${error.message}`);
    }
  });
}

function writeInt16LEClamped(buffer, offset, value, min, max) {
  const numeric = clamp(Math.round(asFiniteNumber(value, 0)), min, max);
  buffer.writeInt16LE(numeric, offset);
  return numeric;
}

function buildGimbalFrame(options = {}) {
  const frame = Buffer.alloc(GIMBAL_FRAME_LENGTH, 0);
  frame[0] = 0xfb;
  frame[1] = 0x2c;
  frame[2] = Number(options.command || 0) & 0xff;
  if (Number.isFinite(options.param1)) writeInt16LEClamped(frame, 3, options.param1, -32768, 32767);
  if (Number.isFinite(options.param2)) writeInt16LEClamped(frame, 5, options.param2, -32768, 32767);
  if (Number.isFinite(options.joystickCommand)) frame[37] = Number(options.joystickCommand) & 0xff;
  if (Number.isFinite(options.joystickX)) writeInt16LEClamped(frame, 38, options.joystickX, -32768, 32767);
  if (Number.isFinite(options.joystickY)) writeInt16LEClamped(frame, 40, options.joystickY, -32768, 32767);
  let checksum = 0;
  for (let index = 2; index <= 41; index += 1) checksum ^= frame[index];
  frame[42] = checksum & 0xff;
  frame[43] = 0xf0;
  return frame;
}

function buildGimbalJsonFrame(payloadText) {
  const payload = Buffer.from(String(payloadText || ''), 'ascii');
  const maxPayloadLength = 35;
  const payloadLength = Math.min(payload.length, maxPayloadLength);
  const frame = Buffer.alloc(GIMBAL_FRAME_LENGTH, 0);
  frame[0] = 0xfb;
  frame[1] = 0x2c;
  frame[2] = 0x90;
  frame.writeUInt16LE(payloadLength, 3);
  payload.copy(frame, 7, 0, payloadLength);
  let checksum = 0;
  for (let index = 2; index <= 41; index += 1) checksum ^= frame[index];
  frame[42] = checksum & 0xff;
  frame[43] = 0xf0;
  return { frame, payloadLength };
}

function emitGimbalState() {
  updateGimbalDiagnostics();
  gimbalState.lastRx = gimbalLastRx.updatedAt ? gimbalLastRx : null;
  gimbalState.updatedAt = Date.now();
  io.emit('gimbal_state', { ...gimbalState });
}

function parseGimbalFeedbackFrames() {
  while (gimbalFeedbackParseBuffer.length >= 2) {
    const headerIndex = gimbalFeedbackParseBuffer.indexOf(Buffer.from([0xfc, 0x2c]));
    if (headerIndex < 0) {
      gimbalFeedbackParseBuffer = gimbalFeedbackParseBuffer.subarray(Math.max(0, gimbalFeedbackParseBuffer.length - 1));
      return;
    }
    if (headerIndex > 0) gimbalFeedbackParseBuffer = gimbalFeedbackParseBuffer.subarray(headerIndex);
    if (gimbalFeedbackParseBuffer.length < 64) return;

    const frame = gimbalFeedbackParseBuffer.subarray(0, 64);
    if (frame[63] !== 0xf0) {
      gimbalFeedbackParseBuffer = gimbalFeedbackParseBuffer.subarray(1);
      continue;
    }
    if (frame[2] !== 0x01) {
      gimbalFeedbackParseBuffer = gimbalFeedbackParseBuffer.subarray(1);
      continue;
    }
    let checksum = 0;
    for (let index = 2; index <= 61; index += 1) checksum ^= frame[index];
    const checksumValid = (checksum & 0xff) === frame[62];
    const updatedAt = Date.now();
    const status2 = frame.readUInt16LE(6);
    gimbalState.feedback = {
      payloadType: frame[2],
      selfTest: frame[3],
      status: frame.readUInt16LE(4),
      status2,
      cameraRecording: Boolean(status2 & (1 << 4)),
      tfCardInserted: Boolean(status2 & (1 << 6)),
      servoMode: frame[8],
      yawDeg: frame.readInt16LE(9) / 100,
      pitchDeg: frame.readInt16LE(11) / 100,
      rollDeg: frame.readInt16LE(13) / 100,
      laserRangeM: frame.readUInt16LE(16),
      gyroYawDps: frame.readInt16LE(41) / 100,
      gyroPitchDps: frame.readInt16LE(43) / 100,
      gyroRollDps: frame.readInt16LE(45) / 100,
      displaySource: frame[47],
      digitalZoom: frame[48] / 10,
      checksumValid,
      updatedAt
    };
    gimbalFeedbackParseBuffer = gimbalFeedbackParseBuffer.subarray(64);
  }
}

function rememberGimbalRx(data) {
  if (!data || data.length === 0) return;
  const chunk = Buffer.from(data);
  gimbalRxBuffer = Buffer.concat([gimbalRxBuffer, chunk]);
  if (gimbalRxBuffer.length > 8192) {
    gimbalRxBuffer = gimbalRxBuffer.subarray(gimbalRxBuffer.length - 8192);
  }
  gimbalFeedbackParseBuffer = Buffer.concat([gimbalFeedbackParseBuffer, chunk]);
  if (gimbalFeedbackParseBuffer.length > 4096) {
    gimbalFeedbackParseBuffer = gimbalFeedbackParseBuffer.subarray(gimbalFeedbackParseBuffer.length - 4096);
  }
  parseGimbalFeedbackFrames();
  gimbalLastRx = {
    ascii: chunk.toString('utf8').replace(/\0/g, ''),
    hex: chunk.toString('hex'),
    updatedAt: Date.now()
  };
  if (process.env.MANTA_LOG_GIMBAL_RX === '1') {
    addLog('GIMBAL_RX', gimbalLastRx.ascii || gimbalLastRx.hex);
  }
  emitGimbalState();
}

function openGimbalReadStream() {
  if (gimbalReadStream) return;
  try {
    gimbalReadStream = fs.createReadStream(GIMBAL_SERIAL_PORT, { flags: 'r' });
    gimbalReadStream.on('data', rememberGimbalRx);
    gimbalReadStream.on('error', (error) => {
      addLog('GIMBAL_ERR', `read stream: ${error.message}`);
      gimbalReadStream = null;
    });
    gimbalReadStream.on('close', () => {
      gimbalReadStream = null;
    });
  } catch (error) {
    addLog('GIMBAL_ERR', `open read stream: ${error.message}`);
  }
}

function configureGimbalSerial() {
  try {
    const child = spawn('stty', ['-F', GIMBAL_SERIAL_PORT, String(GIMBAL_BAUD_RATE), 'cs8', '-parenb', '-cstopb', '-ixon', '-ixoff', 'raw'], {
      stdio: 'ignore'
    });
    child.on('error', (error) => {
      gimbalState.lastError = `stty failed: ${error.message}`;
      addLog('GIMBAL_ERR', gimbalState.lastError);
      emitGimbalState();
    });
  } catch (error) {
    gimbalState.lastError = `stty failed: ${error.message}`;
    addLog('GIMBAL_ERR', gimbalState.lastError);
  }
}

function openGimbalPort(source = 'auto') {
  if (!gimbalState.enabled) {
      gimbalState.lastError = 'Gimbal disabled in config';
    emitGimbalState();
    return false;
  }
  if (source === 'auto' && !GIMBAL_AUTO_CONNECT) {
    gimbalState.lastError = 'Gimbal auto-connect disabled; press Connect to open serial';
    emitGimbalState();
    return false;
  }
  if (gimbalStream) return true;
  try {
    configureGimbalSerial();
    gimbalStream = fs.createWriteStream(GIMBAL_SERIAL_PORT, { flags: 'w' });
    gimbalStream.on('open', () => {
      gimbalState.portOpen = true;
      gimbalState.connected = false;
      gimbalState.lastError = '';
      addLog('GIMBAL', `Serial opened ${GIMBAL_SERIAL_PORT} @ ${GIMBAL_BAUD_RATE}`);
      openGimbalReadStream();
      emitGimbalState();
        if (gimbalPendingHomeSource) {
          const pendingSource = gimbalPendingHomeSource;
          gimbalPendingHomeSource = '';
          sendGimbalHome(pendingSource);
        }
        if (GIMBAL_AUTO_HOME_ON_CONNECT) {
          sendGimbalHome('connect');
        }
    });
    gimbalStream.on('error', (error) => {
      gimbalState.connected = false;
      gimbalState.portOpen = false;
      gimbalState.lastError = error.message;
      addLog('GIMBAL_ERR', error.message);
      emitGimbalState();
      gimbalStream = null;
    });
    gimbalStream.on('close', () => {
      gimbalState.connected = false;
      gimbalState.portOpen = false;
      gimbalStream = null;
      emitGimbalState();
    });
    return true;
  } catch (error) {
    gimbalState.connected = false;
    gimbalState.portOpen = false;
    gimbalState.lastError = error.message;
    addLog('GIMBAL_ERR', error.message);
    emitGimbalState();
    return false;
  }
}

function writeGimbalFrame(frame) {
  gimbalLastFrame = frame;
  if (!gimbalTxEnabled) return false;
    if (!gimbalState.enabled || !gimbalStream) return false;
  try {
    gimbalStream.write(frame);
    return true;
  } catch (error) {
    gimbalState.lastError = error.message;
    addLog('GIMBAL_ERR', error.message);
    emitGimbalState();
    return false;
  }
}

function writeGimbalFrameBurst(frame, count = 1) {
  if (!gimbalState.enabled || !gimbalStream) return false;
  try {
    for (let index = 0; index < count; index += 1) {
      gimbalStream.write(frame);
    }
    return true;
  } catch (error) {
    gimbalState.lastError = error.message;
    addLog('GIMBAL_ERR', error.message);
    emitGimbalState();
    return false;
  }
}

function disconnectGimbalPort() {
  stopGimbalTracking(false);
  gimbalPendingHomeSource = '';
  gimbalHoldUntil = 0;
  gimbalLastFrame = buildGimbalFrame();
  gimbalLastRateX = 0;
  gimbalLastRateY = 0;
  gimbalTrackDesiredRateX = 0;
  gimbalTrackDesiredRateY = 0;
  gimbalTrackRateAccelX = 0;
  gimbalTrackRateAccelY = 0;
  gimbalTrackTarget = null;
  gimbalTrackHolding = true;
  gimbalTrackLastTargetAt = 0;
  gimbalStopFramesRemaining = 0;
  if (gimbalCommandTimer) {
    clearInterval(gimbalCommandTimer);
    gimbalCommandTimer = null;
  }
  if (gimbalStream) {
    try { gimbalStream.end(); } catch (_) {}
    gimbalStream = null;
  }
  if (gimbalReadStream) {
    try { gimbalReadStream.destroy(); } catch (_) {}
    gimbalReadStream = null;
  }
  gimbalState.connected = false;
  gimbalState.portOpen = false;
  gimbalState.mode = 'idle';
  gimbalState.lastCommand = 'disconnected';
  emitGimbalState();
}

function smoothStep01(value) {
  const x = clamp(value, 0, 1);
  return x * x * (3 - 2 * x);
}

function applyGimbalSoftLimit(rate, angleDeg, minDeg, maxDeg) {
  if (!Number.isFinite(angleDeg) || !Number.isFinite(rate) || rate === 0) return rate;
  const brakeDeg = Math.max(
    GIMBAL_SOFT_LIMIT_BRAKE_DEG,
    clamp(Math.abs(rate) * 0.12 + 1.2, 1.5, 10)
  );
  if (rate > 0) {
    const remaining = maxDeg - angleDeg;
    if (remaining <= 0) return 0;
    if (remaining < brakeDeg) return rate * smoothStep01(remaining / brakeDeg);
  } else {
    const remaining = angleDeg - minDeg;
    if (remaining <= 0) return 0;
    if (remaining < brakeDeg) return rate * smoothStep01(remaining / brakeDeg);
  }
  return rate;
}

function limitGimbalTrackRates(rateX, rateY) {
  const feedback = gimbalState.feedback;
  if (!feedback || !feedback.checksumValid || Date.now() - feedback.updatedAt > 300) {
    return { x: rateX, y: rateY, limited: false };
  }
  const x = applyGimbalSoftLimit(rateX, feedback.yawDeg, GIMBAL_YAW_MIN_DEG, GIMBAL_YAW_MAX_DEG);
  const y = applyGimbalSoftLimit(rateY, feedback.pitchDeg, GIMBAL_PITCH_MIN_DEG, GIMBAL_PITCH_MAX_DEG);
  return { x, y, limited: x !== rateX || y !== rateY };
}

function computeGimbalTrackDesiredRate(now) {
  if (!gimbalTrackTarget || !gimbalTrackLastTargetAt) {
    return { x: 0, y: 0, gated: true, stale: true };
  }
  const ageMs = now - gimbalTrackLastTargetAt;
  if (ageMs > GIMBAL_TRACK_TARGET_TIMEOUT_MS) {
    return { x: 0, y: 0, gated: true, stale: true };
  }

  const detectorAgeMs = Math.max(0, asFiniteNumber(gimbalTrackTarget.detectorAgeMs, 0));
  const targetSpeed = Math.hypot(gimbalTrackTarget.vx, gimbalTrackTarget.vy);
  const speedLeadMix = smoothStep01(targetSpeed / GIMBAL_TRACK_FAST_SPEED_PX_S);
  const leadMs = clamp(
    ageMs
      + GIMBAL_TRACK_LATENCY_LEAD_MS
      + detectorAgeMs * GIMBAL_TRACK_DETECTOR_AGE_LEAD
      + GIMBAL_TRACK_SPEED_LEAD_MS * speedLeadMix,
    0,
    GIMBAL_TRACK_PREDICTION_MS
  );
  const predictionSeconds = leadMs / 1000;
  const predictedX = clamp(gimbalTrackTarget.x + gimbalTrackTarget.vx * predictionSeconds, -GIMBAL_MAX_PIXEL_X, GIMBAL_MAX_PIXEL_X);
  const predictedY = clamp(gimbalTrackTarget.y + gimbalTrackTarget.vy * predictionSeconds, -GIMBAL_MAX_PIXEL_Y, GIMBAL_MAX_PIXEL_Y);
  const faceMode = gimbalState.trackMode === 'face';
  const holdEnterX = faceMode ? GIMBAL_FACE_TRACK_HOLD_ENTER_X_PX : GIMBAL_TRACK_HOLD_ENTER_X_PX;
  const holdEnterY = faceMode ? GIMBAL_FACE_TRACK_HOLD_ENTER_Y_PX : GIMBAL_TRACK_HOLD_ENTER_Y_PX;
  const holdExitX = faceMode ? GIMBAL_FACE_TRACK_HOLD_EXIT_X_PX : GIMBAL_TRACK_HOLD_EXIT_X_PX;
  const holdExitY = faceMode ? GIMBAL_FACE_TRACK_HOLD_EXIT_Y_PX : GIMBAL_TRACK_HOLD_EXIT_Y_PX;
  const error = Math.max(Math.abs(predictedX), Math.abs(predictedY));
  const inHoldEnterZone = Math.abs(predictedX) <= holdEnterX
    && Math.abs(predictedY) <= holdEnterY;
  const outsideHoldExitZone = Math.abs(predictedX) >= holdExitX
    || Math.abs(predictedY) >= holdExitY;

  if (gimbalTrackHolding) {
    if (outsideHoldExitZone || targetSpeed >= GIMBAL_TRACK_HOLD_EXIT_SPEED_PX_S) {
      gimbalTrackHolding = false;
    }
  } else if (inHoldEnterZone && targetSpeed <= GIMBAL_TRACK_HOLD_ENTER_SPEED_PX_S) {
    gimbalTrackHolding = true;
  }
  if (gimbalTrackHolding) {
    return { x: 0, y: 0, gated: true, predictedX, predictedY, targetSpeed, stale: false };
  }

  function softError(value, holdEnter) {
    const deadband = holdEnter * 0.65;
    return Math.abs(value) <= deadband ? 0 : Math.sign(value) * (Math.abs(value) - deadband);
  }
  const activeX = softError(predictedX, holdEnterX);
  const activeY = softError(predictedY, holdEnterY);
  const angles = gimbalAnglesFromPixelDelta(activeX, activeY);
  const holdExit = Math.max(holdExitX, holdExitY);
  const gainMix = smoothStep01((error - holdExit) / Math.max(1, GIMBAL_TRACK_FAST_ZONE_PX - holdExit));
  const angleGain = GIMBAL_TRACK_NEAR_GAIN + (GIMBAL_TRACK_FAR_GAIN - GIMBAL_TRACK_NEAR_GAIN) * gainMix;
  const quality = Number.isFinite(gimbalTrackTarget.flowQuality) ? clamp(gimbalTrackTarget.flowQuality, 0, 1) : 1;
  const coastScale = gimbalTrackTarget.coasting ? 0.30 : 1;
  const feedforwardScale = GIMBAL_TRACK_FEEDFORWARD_GAIN * coastScale * (0.35 + 0.65 * quality);
  const yawVelocityDps = Math.atan2(gimbalTrackTarget.vx, Math.max(GIMBAL_CALIB_FX, 1)) * 180 / Math.PI;
  const pitchVelocityDps = Math.atan2(gimbalTrackTarget.vy, Math.max(GIMBAL_CALIB_FY, 1)) * 180 / Math.PI;
  const feedback = gimbalState.feedback;
  const feedbackFresh = feedback && feedback.checksumValid && now - feedback.updatedAt <= 300;
  const gyroYawDps = feedbackFresh ? asFiniteNumber(feedback.gyroYawDps, 0) : 0;
  const gyroPitchDps = feedbackFresh ? asFiniteNumber(feedback.gyroPitchDps, 0) : 0;
  let rateX = angles.yawDeg * angleGain + yawVelocityDps * feedforwardScale - gyroYawDps * GIMBAL_TRACK_GYRO_DAMPING;
  let rateY = angles.pitchDeg * angleGain + pitchVelocityDps * feedforwardScale - gyroPitchDps * GIMBAL_TRACK_GYRO_DAMPING;
  const rateMagnitude = Math.hypot(rateX, rateY);
  if (rateMagnitude > GIMBAL_MAX_RATE_DPS) {
    rateX *= GIMBAL_MAX_RATE_DPS / rateMagnitude;
    rateY *= GIMBAL_MAX_RATE_DPS / rateMagnitude;
  }
  const limited = limitGimbalTrackRates(rateX, rateY);
  rateX = limited.x;
  rateY = limited.y;
  if (gimbalTrackTarget.coasting) {
    const recoveryProgress = clamp(gimbalTrackTarget.predictionAgeMs / GIMBAL_TRACK_RECOVERY_MS, 0, 1);
    const recoveryRateLimit = GIMBAL_TRACK_PREDICTION_MAX_RATE_DPS * (1 - recoveryProgress * 0.65);
    const recoveryMagnitude = Math.hypot(rateX, rateY);
    if (recoveryMagnitude > recoveryRateLimit) {
      rateX *= recoveryRateLimit / recoveryMagnitude;
      rateY *= recoveryRateLimit / recoveryMagnitude;
    }
  }
  return {
    x: rateX,
    y: rateY,
    gated: false,
    predictedX,
    predictedY,
    targetSpeed,
    detectorAgeMs,
    leadMs,
    angleGain,
    limited: limited.limited,
    stale: false
  };
}

function advanceGimbalTrackAxis(currentRate, currentAccel, desiredRate, dt) {
  const braking = Math.abs(currentRate) > 0.5
    && (Math.sign(desiredRate) !== Math.sign(currentRate) || Math.abs(desiredRate) < Math.abs(currentRate));
  const accelLimit = braking ? GIMBAL_TRACK_BRAKE_ACCEL_DPS2 : GIMBAL_TRACK_MAX_ACCEL_DPS2;
  const jerkLimit = braking ? GIMBAL_TRACK_BRAKE_JERK_DPS3 : GIMBAL_TRACK_MAX_JERK_DPS3;
  const targetAccel = clamp(
    (desiredRate - currentRate) / GIMBAL_TRACK_RESPONSE_SECONDS,
    -accelLimit,
    accelLimit
  );
  const maxAccelStep = jerkLimit * dt;
  let accel = currentAccel + clamp(targetAccel - currentAccel, -maxAccelStep, maxAccelStep);
  let rate = currentRate + accel * dt;
  if ((desiredRate - currentRate) * (desiredRate - rate) <= 0) {
    rate = desiredRate;
    accel = 0;
  }
  if (Math.abs(desiredRate) < 0.01 && Math.abs(rate) < 0.35) {
    rate = 0;
    accel = 0;
  }
  if (GIMBAL_TRACK_OUTPUT_DEADBAND_DPS > 0
    && Math.abs(desiredRate) < GIMBAL_TRACK_OUTPUT_DEADBAND_DPS
    && Math.abs(rate) < GIMBAL_TRACK_OUTPUT_DEADBAND_DPS) {
    rate = 0;
    accel = 0;
  }
  return { rate, accel };
}

function resetGimbalTrackMotionState() {
  gimbalTrackDesiredRateX = 0;
  gimbalTrackDesiredRateY = 0;
  gimbalTrackRateAccelX = 0;
  gimbalTrackRateAccelY = 0;
  gimbalTrackFilteredVx = 0;
  gimbalTrackFilteredVy = 0;
  gimbalTrackLastMovingDesiredX = 0;
  gimbalTrackLastMovingDesiredY = 0;
  gimbalTrackCounterBrakeUntil = 0;
  gimbalTrackCounterBrakeX = 0;
  gimbalTrackCounterBrakeY = 0;
  gimbalTrackCounterBrakeSettled = true;
}

function applyGimbalTrackCounterBrake(desired, now) {
  const rawMagnitude = Math.hypot(desired.x, desired.y);
  if (rawMagnitude >= GIMBAL_TRACK_COUNTER_BRAKE_MIN_DPS) {
    gimbalTrackLastMovingDesiredX = desired.x;
    gimbalTrackLastMovingDesiredY = desired.y;
    gimbalTrackCounterBrakeUntil = 0;
    gimbalTrackCounterBrakeSettled = false;
    return { ...desired, counterBrake: false };
  }

  if (
    GIMBAL_TRACK_COUNTER_BRAKE_MS > 0
    && !gimbalTrackCounterBrakeSettled
    && now >= gimbalTrackCounterBrakeUntil
  ) {
    const lastMagnitude = Math.hypot(gimbalTrackLastMovingDesiredX, gimbalTrackLastMovingDesiredY);
    if (lastMagnitude >= GIMBAL_TRACK_COUNTER_BRAKE_MIN_DPS) {
      const brakeMagnitude = clamp(
        lastMagnitude * GIMBAL_TRACK_COUNTER_BRAKE_GAIN,
        GIMBAL_TRACK_COUNTER_BRAKE_MIN_DPS,
        GIMBAL_TRACK_COUNTER_BRAKE_MAX_DPS
      );
      const scale = brakeMagnitude / Math.max(lastMagnitude, 1);
      gimbalTrackCounterBrakeX = -gimbalTrackLastMovingDesiredX * scale;
      gimbalTrackCounterBrakeY = -gimbalTrackLastMovingDesiredY * scale;
      gimbalTrackCounterBrakeUntil = now + GIMBAL_TRACK_COUNTER_BRAKE_MS;
      gimbalTrackCounterBrakeSettled = true;
    }
  }

  if (now < gimbalTrackCounterBrakeUntil) {
    return {
      ...desired,
      x: gimbalTrackCounterBrakeX,
      y: gimbalTrackCounterBrakeY,
      counterBrake: true
    };
  }

  return { ...desired, counterBrake: false };
}

function startGimbalCounterBrake(source = 'manual-stop', mode = 'brake') {
  const currentMagnitude = Math.hypot(gimbalLastRateX, gimbalLastRateY);
  if (GIMBAL_TRACK_COUNTER_BRAKE_MS <= 0 || currentMagnitude < GIMBAL_TRACK_COUNTER_BRAKE_MIN_DPS) {
    return false;
  }
  const brakeMagnitude = clamp(
    currentMagnitude * GIMBAL_TRACK_COUNTER_BRAKE_GAIN,
    GIMBAL_TRACK_COUNTER_BRAKE_MIN_DPS,
    GIMBAL_TRACK_COUNTER_BRAKE_MAX_DPS
  );
  const scale = brakeMagnitude / Math.max(currentMagnitude, 1);
  const brakeX = Math.round(-gimbalLastRateX * scale);
  const brakeY = Math.round(-gimbalLastRateY * scale);
  const frame = buildGimbalFrame({ joystickCommand: 0x70, joystickX: brakeX, joystickY: brakeY });
  setGimbalFrame(frame, `counter-brake:${source}`, mode, GIMBAL_TRACK_COUNTER_BRAKE_MS, {
    x: brakeX,
    y: brakeY,
    source,
    holdMs: GIMBAL_TRACK_COUNTER_BRAKE_MS
  });
  if (gimbalState.trackingActive && mode === 'track') {
    gimbalTrackCommandPauseUntil = Date.now() + GIMBAL_TRACK_COUNTER_BRAKE_MS;
  }
  return true;
}

function startGimbalLoop() {
  if (gimbalCommandTimer) return;
  gimbalLastFrame = buildGimbalFrame();
  gimbalCommandTimer = setInterval(() => {
    const now = Date.now();
    if (gimbalState.trackingActive && now < gimbalTrackCommandPauseUntil) {
      gimbalTrackDesiredRateX = 0;
      gimbalTrackDesiredRateY = 0;
      gimbalTrackRateAccelX = 0;
      gimbalTrackRateAccelY = 0;
      gimbalLastRateX = 0;
      gimbalLastRateY = 0;
      gimbalTxEnabled = true;
      gimbalStopFramesRemaining = 0;
    } else if (gimbalState.trackingActive) {
      gimbalHoldUntil = 0;
      const desired = applyGimbalTrackCounterBrake(computeGimbalTrackDesiredRate(now), now);
      gimbalTrackDesiredRateX = desired.x;
      gimbalTrackDesiredRateY = desired.y;
      const dt = GIMBAL_COMMAND_INTERVAL_MS / 1000;
      const nextX = advanceGimbalTrackAxis(gimbalLastRateX, gimbalTrackRateAccelX, gimbalTrackDesiredRateX, dt);
      const nextY = advanceGimbalTrackAxis(gimbalLastRateY, gimbalTrackRateAccelY, gimbalTrackDesiredRateY, dt);
      gimbalLastRateX = nextX.rate;
      gimbalLastRateY = nextY.rate;
      gimbalTrackRateAccelX = nextX.accel;
      gimbalTrackRateAccelY = nextY.accel;
      gimbalLastFrame = buildGimbalFrame({
        joystickCommand: 0x70,
        joystickX: Math.round(gimbalLastRateX),
        joystickY: Math.round(gimbalLastRateY)
      });
      gimbalTxEnabled = true;
      gimbalStopFramesRemaining = 0;
    } else if (gimbalHoldUntil && now > gimbalHoldUntil) {
      gimbalHoldUntil = 0;
      gimbalLastFrame = buildGimbalFrame({ joystickCommand: 0x70, joystickX: 0, joystickY: 0 });
      gimbalLastRateX = 0;
      gimbalLastRateY = 0;
      gimbalState.mode = gimbalState.trackingActive ? 'track' : 'idle';
      gimbalState.lastCommand = gimbalState.trackingActive ? 'track' : 'idle';
      if (!gimbalState.trackingActive) {
        gimbalTxEnabled = true;
        gimbalLastRateX = 0;
        gimbalLastRateY = 0;
        gimbalStopFramesRemaining = 4;
      }
      emitGimbalState();
    }
    if (writeGimbalFrame(gimbalLastFrame) && gimbalStopFramesRemaining > 0) {
      gimbalStopFramesRemaining -= 1;
      if (gimbalStopFramesRemaining === 0 && !gimbalState.trackingActive && !gimbalHoldUntil) {
        gimbalTxEnabled = false;
        gimbalLastFrame = buildGimbalFrame();
      }
    }
  }, GIMBAL_COMMAND_INTERVAL_MS);
}

function setGimbalFrame(frame, label, mode, holdMs = 0, target = null) {
  startGimbalLoop();
  gimbalLastFrame = frame;
    gimbalTxEnabled = true;
  gimbalStopFramesRemaining = 0;
  gimbalHoldUntil = holdMs > 0 ? Date.now() + holdMs : 0;
  gimbalState.lastCommand = label;
  gimbalState.mode = mode;
  gimbalState.lastTarget = target;
  emitGimbalState();
  writeGimbalFrame(frame);
}

function stopGimbalTracking(resetState = true) {
  gimbalTrackStopRequested = true;
  const shouldCounterBrake = resetState && Math.hypot(gimbalLastRateX, gimbalLastRateY) >= GIMBAL_TRACK_COUNTER_BRAKE_MIN_DPS;
  if (gimbalTrackRestartTimer) {
    clearTimeout(gimbalTrackRestartTimer);
    gimbalTrackRestartTimer = null;
  }
  if (gimbalTrackProcess && gimbalTrackProcess.exitCode === null && !gimbalTrackProcess.killed) {
    try { gimbalTrackProcess.kill('SIGTERM'); } catch (_) {}
  }
  gimbalState.trackingActive = false;
  gimbalState.trackWorkerActive = false;
  gimbalState.trackStatus = { locked: false, status: 'idle', message: 'idle', detections: 0, updatedAt: Date.now() };
  gimbalState.lastTarget = null;
  gimbalTrackTarget = null;
  gimbalTrackHolding = true;
  gimbalTrackLastTargetAt = 0;
  gimbalTrackCommandPauseUntil = 0;
  if (resetState) {
    const braked = shouldCounterBrake && startGimbalCounterBrake('track-stop', 'brake');
    resetGimbalTrackMotionState();
    if (!braked) {
      gimbalTxEnabled = false;
      gimbalState.mode = 'idle';
      gimbalLastFrame = buildGimbalFrame();
      gimbalStopFramesRemaining = 0;
    }
    gimbalLastRateX = 0;
    gimbalLastRateY = 0;
    emitGimbalState();
  }
  cleanupStaleGimbalTrackWorkers('stop');
  return { ok: true };
}

function updateGimbalTrackStatus(update = {}) {
  const next = {
    locked: false,
    status: 'lost',
    mode: gimbalState.trackMode,
    message: gimbalTrackLostMessage(),
    detections: 0,
    workerActive: isGimbalTrackingActive(),
    updatedAt: Date.now(),
    ...update
  };
  gimbalState.trackWorkerActive = isGimbalTrackingActive();
  gimbalState.trackStatus = next;
  if (!next.locked) {
    gimbalState.lastTarget = next;
  }
  io.emit('gimbal_track_status', next);
  emitGimbalState();
  return next;
}

function holdGimbalTrackIdle(source = 'swimmer-lost') {
  gimbalTrackDesiredRateX = 0;
  gimbalTrackDesiredRateY = 0;
  gimbalTrackRateAccelX = 0;
  gimbalTrackRateAccelY = 0;
  gimbalTrackTarget = null;
  gimbalTrackHolding = true;
  gimbalTrackLastTargetAt = 0;
  gimbalTrackLastDirection = '';
  gimbalTrackDirectionCount = 0;
  if (startGimbalCounterBrake(source, 'track')) {
    return gimbalLastFrame;
  }
  const frame = buildGimbalFrame({ joystickCommand: 0x70, joystickX: 0, joystickY: 0 });
  setGimbalFrame(frame, source, 'track', 0, { locked: false, status: 'lost', mode: gimbalState.trackMode, message: gimbalTrackLostMessage() });
  return frame;
}

function sendGimbalCancelTrack(source = 'web') {
  const frame = buildGimbalFrame({ command: 0x3b, param1: 0, param2: 0 });
  setGimbalFrame(frame, `cancel-track:${source}`, 'idle', 180, { command: 0x3b });
  writeGimbalFrameBurst(frame, 5);
  addLog('GIMBAL', `Cancel track command sent (${source})`);
  return { command: 0x3b, holdMs: 180 };
}

function sendGimbalHome(source = 'web', options = {}) {
  const preserveTracking = Boolean(options.preserveTracking && gimbalState.trackingActive);
  if (!preserveTracking) {
    stopGimbalTracking(false);
    sendGimbalDetectorPaused(true, `${source}:prehome`);
    sendGimbalCancelTrack(`${source}:prehome`);
  } else {
    gimbalTrackCommandPauseUntil = Date.now() + 900;
    gimbalTrackTarget = null;
    gimbalTrackHolding = true;
    gimbalTrackLastTargetAt = 0;
    resetGimbalTrackMotionState();
    gimbalLastRateX = 0;
    gimbalLastRateY = 0;
    gimbalState.lastTarget = null;
  }
  gimbalHoldUntil = 0;
  gimbalStopFramesRemaining = 0;
  if (!preserveTracking) {
    gimbalLastRateX = 0;
    gimbalLastRateY = 0;
  }
  const disableFrame = buildGimbalFrame({ joystickCommand: 0x00, joystickX: 0, joystickY: 0 });
  writeGimbalFrameBurst(disableFrame, 5);
  const frame = buildGimbalFrame({ command: 0x71 });
  setGimbalFrame(frame, `home:${source}`, preserveTracking ? 'track' : 'home', 900, null);
  writeGimbalFrameBurst(frame, 3);
  updateGimbalDiagnostics();
  addLog('GIMBAL', `Home command sent (${source}) preserveTracking=${preserveTracking}`);
}

function stopGimbalSerial(source = 'web') {
  stopGimbalTracking(false);
  sendGimbalDetectorPaused(true, `${source}:prestop`);
  sendGimbalCancelTrack(`${source}:prestop`);
  gimbalPendingHomeSource = '';
  gimbalTxEnabled = true;
  gimbalHoldUntil = 0;
  gimbalLastFrame = buildGimbalFrame({ joystickCommand: 0x00, joystickX: 0, joystickY: 0 });
  gimbalLastRateX = 0;
  gimbalLastRateY = 0;
  resetGimbalTrackMotionState();
  gimbalTrackCommandPauseUntil = 0;
  gimbalStopFramesRemaining = 5;
  gimbalState.mode = 'idle';
  gimbalState.lastCommand = `stop:${source}`;
  gimbalState.lastTarget = null;
  if (gimbalStream) {
    const stopFrame = buildGimbalFrame({ joystickCommand: 0x00, joystickX: 0, joystickY: 0 });
    writeGimbalFrameBurst(stopFrame, 5);
  }
  updateGimbalDiagnostics();
  emitGimbalState();
  addLog('GIMBAL', `Serial stopped (${source})`);
}

function normalizeGimbalDelta(dx, dy) {
  return {
    x: clamp(Math.round(asFiniteNumber(dx, 0) * GIMBAL_YAW_SIGN), -GIMBAL_MAX_PIXEL_X, GIMBAL_MAX_PIXEL_X),
    y: clamp(Math.round(asFiniteNumber(dy, 0) * GIMBAL_PITCH_SIGN), -GIMBAL_MAX_PIXEL_Y, GIMBAL_MAX_PIXEL_Y)
  };
}

function sendGimbalClickTarget(dx, dy, holdMs = null) {
  const delta = normalizeGimbalDelta(dx, dy);
  if (GIMBAL_CLICK_CONTROL_MODE === 'rate') {
    const plan = planGimbalClickMove(delta.x, delta.y);
    const clickHoldMs = holdMs === null ? plan.holdMs : Math.min(holdMs, plan.holdMs);
    const frame = buildGimbalFrame({ joystickCommand: 0x70, joystickX: plan.rateX, joystickY: plan.rateY });
    setGimbalFrame(frame, 'click-relative', 'click', clickHoldMs, { ...delta, ...plan, controlMode: 'rate', holdMs: clickHoldMs });
    addLog('GIMBAL', `Click relative dx=${delta.x} dy=${delta.y} yaw=${plan.yawDeg.toFixed(1)} pitch=${plan.pitchDeg.toFixed(1)} rateX=${plan.rateX} rateY=${plan.rateY} holdMs=${clickHoldMs}`);
    return { ...delta, ...plan, controlMode: 'rate', holdMs: clickHoldMs };
  }

  if (GIMBAL_CLICK_CONTROL_MODE === 'select_target') {
    const clickHoldMs = holdMs === null ? GIMBAL_CLICK_TARGET_HOLD_MS : Math.max(40, Number(holdMs || GIMBAL_CLICK_TARGET_HOLD_MS));
    const frame = buildGimbalFrame({ joystickCommand: 0x60, joystickX: delta.x, joystickY: delta.y });
    setGimbalFrame(frame, 'select-target', 'click', clickHoldMs, { ...delta, controlMode: 'select_target', holdMs: clickHoldMs });
    writeGimbalFrameBurst(frame, 3);
    addLog('GIMBAL', `Select target via joystick field dx=${delta.x} dy=${delta.y} holdMs=${clickHoldMs}`);
    return { ...delta, controlMode: 'select_target', holdMs: clickHoldMs };
  }

  if (GIMBAL_CLICK_CONTROL_MODE === 'point_track') {
    const clickHoldMs = holdMs === null ? GIMBAL_CLICK_TARGET_HOLD_MS : Math.max(40, Number(holdMs || GIMBAL_CLICK_TARGET_HOLD_MS));
    const frame = buildGimbalFrame({ command: 0x3a, param1: delta.x, param2: delta.y });
    setGimbalFrame(frame, 'point-track', 'click', clickHoldMs, { ...delta, controlMode: 'point_track', command: 0x3a, holdMs: clickHoldMs });
    writeGimbalFrameBurst(frame, 5);
    addLog('GIMBAL', `Point track command 3AH dx=${delta.x} dy=${delta.y} holdMs=${clickHoldMs}`);
    return { ...delta, controlMode: 'point_track', command: 0x3a, holdMs: clickHoldMs };
  }

  const ray = gimbalAnglesFromPixelDelta(delta.x, delta.y);
  const feedback = gimbalState.feedback;
  const feedbackFresh = feedback && feedback.checksumValid && Date.now() - feedback.updatedAt <= 500;
  const currentYawDeg = feedbackFresh ? asFiniteNumber(feedback.yawDeg, 0) : 0;
  const currentPitchDeg = feedbackFresh ? asFiniteNumber(feedback.pitchDeg, 0) : 0;
  const targetYawDeg = clamp(currentYawDeg + ray.yawDeg, GIMBAL_YAW_MIN_DEG, GIMBAL_YAW_MAX_DEG);
  const targetPitchDeg = clamp(currentPitchDeg + ray.pitchDeg, GIMBAL_PITCH_MIN_DEG, GIMBAL_PITCH_MAX_DEG);
  const angleYaw = Math.round(targetYawDeg * 100);
  const anglePitch = Math.round(targetPitchDeg * 100);
  const maxAngle = Math.max(Math.abs(ray.yawDeg), Math.abs(ray.pitchDeg));
  const clickHoldMs = holdMs === null
    ? clamp(Math.round(maxAngle * 55 + 480), 480, 2200)
    : Math.max(120, Number(holdMs || 900));
  const frame = buildGimbalFrame({ command: 0x72, param1: angleYaw, param2: anglePitch });
  const target = {
    ...delta,
    controlMode: 'guided_angle',
    command: 0x72,
    offsetYawDeg: ray.yawDeg,
    offsetPitchDeg: ray.pitchDeg,
    currentYawDeg,
    currentPitchDeg,
    targetYawDeg,
    targetPitchDeg,
    angleYaw,
    anglePitch,
    feedbackFresh: Boolean(feedbackFresh),
    limited: targetYawDeg !== currentYawDeg + ray.yawDeg || targetPitchDeg !== currentPitchDeg + ray.pitchDeg,
    holdMs: clickHoldMs
  };
  setGimbalFrame(frame, 'guided-angle', 'click', clickHoldMs, target);
  writeGimbalFrameBurst(frame, 5);
  addLog('GIMBAL', `Guided angle 72H dx=${delta.x} dy=${delta.y} current=${currentYawDeg.toFixed(2)},${currentPitchDeg.toFixed(2)} target=${targetYawDeg.toFixed(2)},${targetPitchDeg.toFixed(2)} holdMs=${clickHoldMs}`);
  return target;
}

function planGimbalClickMove(dx, dy) {
  const { yawDeg, pitchDeg } = gimbalAnglesFromPixelDelta(dx, dy);
  const maxAngle = Math.max(Math.abs(yawDeg), Math.abs(pitchDeg));
  if (maxAngle < 0.01) {
    return { rateX: 0, rateY: 0, holdMs: GIMBAL_CLICK_HOLD_MIN_MS, yawDeg, pitchDeg };
  }
  const baseRate = Math.min(GIMBAL_CLICK_RATE_DPS, GIMBAL_MAX_RATE_DPS);
  const rateX = clamp(Math.round((yawDeg / maxAngle) * baseRate), -GIMBAL_MAX_RATE_DPS, GIMBAL_MAX_RATE_DPS);
  const rateY = clamp(Math.round((pitchDeg / maxAngle) * baseRate), -GIMBAL_MAX_RATE_DPS, GIMBAL_MAX_RATE_DPS);
  const holdMs = clamp(
    Math.round((maxAngle / Math.max(baseRate, 1)) * 1000 * GIMBAL_CLICK_DURATION_SCALE + GIMBAL_CLICK_EXTRA_MS),
    GIMBAL_CLICK_HOLD_MIN_MS,
    GIMBAL_CLICK_HOLD_MAX_MS
  );
  return { rateX, rateY, holdMs, yawDeg, pitchDeg };
}

function undistortNormalizedPoint(xd, yd) {
  const [k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0] = GIMBAL_CALIB_DIST;
  if (!k1 && !k2 && !p1 && !p2 && !k3) {
    return { x: xd, y: yd };
  }
  let x = xd;
  let y = yd;
  for (let index = 0; index < 6; index += 1) {
    const r2 = x * x + y * y;
    const radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    const tx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
    const ty = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
    x = (xd - tx) / Math.max(radial, 1e-6);
    y = (yd - ty) / Math.max(radial, 1e-6);
  }
  return { x, y };
}

function gimbalAnglesFromPixelDelta(dx, dy) {
  const centerU = GIMBAL_CALIB_CX;
  const centerV = GIMBAL_CALIB_CY;
  const u = centerU + asFiniteNumber(dx, 0);
  const v = centerV + asFiniteNumber(dy, 0);
  const xd = (u - GIMBAL_CALIB_CX) / Math.max(GIMBAL_CALIB_FX, 1e-6);
  const yd = (v - GIMBAL_CALIB_CY) / Math.max(GIMBAL_CALIB_FY, 1e-6);
  const ray = undistortNormalizedPoint(xd, yd);
  const yawDeg = Math.atan2(ray.x, 1) * 180 / Math.PI;
  const pitchDeg = Math.atan2(ray.y, Math.sqrt(ray.x * ray.x + 1)) * 180 / Math.PI;
  return { yawDeg, pitchDeg, rayX: ray.x, rayY: ray.y, fx: GIMBAL_CALIB_FX, fy: GIMBAL_CALIB_FY };
}

function sendGimbalZoomReset(source = 'web') {
  const command = 0x45;
  const param1 = 0x0100; // Byte4=0, Byte5=1: restore all visible-light zoom to 1.0x.
  const param2 = 0;
  const burstFrames = clamp(Math.round(asFiniteNumber(gimbalFocusConfig.burst_frames, 8)), 1, 30);
  const holdMs = clamp(Math.round(asFiniteNumber(gimbalFocusConfig.hold_ms, 650)), 80, 3000);
  const disableFrame = buildGimbalFrame({ joystickCommand: 0x00, joystickX: 0, joystickY: 0 });
  writeGimbalFrameBurst(disableFrame, 3);
  const frame = buildGimbalFrame({ command, param1, param2 });
  setGimbalFrame(frame, `zoom-reset:${source}`, 'camera', holdMs, { command, param1, param2 });
  writeGimbalFrameBurst(frame, burstFrames);
  addLog('GIMBAL', `Visible zoom reset sent command=0x${command.toString(16)} param1=0x${param1.toString(16)}`);
  return { command, param1, param2, burstFrames, holdMs };
}

function sendGimbalOsd(mode = 0, source = 'web') {
  const osdMode = clamp(Math.round(asFiniteNumber(mode, 0)), 0, 2);
  const frame = buildGimbalFrame({ command: 0x37, param1: osdMode, param2: 0 });
  setGimbalFrame(frame, `osd:${osdMode}:${source}`, 'osd', 240, { command: 0x37, osdMode });
  writeGimbalFrameBurst(frame, 5);
  addLog('GIMBAL', `OSD mode sent mode=${osdMode} (0=hide, 1=track-only, 2=all)`);
  return { command: 0x37, osdMode, holdMs: 240 };
}

function sendGimbalRecordCommand(action = 'start', source = 'web') {
  const mode = action === 'stop' || Number(action) === 2 ? 2 : 1;
  const frame = buildGimbalFrame({ command: 0x33, param1: mode, param2: 0 });
  setGimbalFrame(frame, `record:${mode}:${source}`, 'record', 240, { command: 0x33, recordMode: mode });
  writeGimbalFrameBurst(frame, 5);
  addLog('GIMBAL', `Record command sent mode=${mode} (1=start, 2=stop)`);
  return { command: 0x33, recordMode: mode, holdMs: 240 };
}

function sendGimbalStreamProfile(streamIndex = GIMBAL_RECORD_STREAM_INDEX, quality = GIMBAL_RECORD_STREAM_QUALITY, source = 'web') {
  const stream = clamp(Math.round(asFiniteNumber(streamIndex, GIMBAL_RECORD_STREAM_INDEX)), 0, 3);
  const compression = clamp(Math.round(asFiniteNumber(quality, GIMBAL_RECORD_STREAM_QUALITY)), 0, 5);
  const param1 = (stream << 8) | compression;
  const frame = buildGimbalFrame({ command: 0x2f, param1, param2: 0 });
  setGimbalFrame(frame, `stream-profile:${stream}:${compression}:${source}`, 'camera', 260, {
    command: 0x2f,
    compression,
    streamIndex: stream,
    streamName: stream === 1 ? 'live/0' : (stream === 2 ? 'live/1' : (stream === 3 ? 'live/2' : 'default'))
  });
  writeGimbalFrameBurst(frame, 5);
  addLog('GIMBAL', `Stream profile sent command=0x2f compression=${compression} stream=${stream}`);
  return { command: 0x2f, compression, streamIndex: stream, param1, holdMs: 260 };
}

function sendGimbalLaserRange(mode = 'single', source = 'web') {
  const normalized = String(mode || 'single').trim().toLowerCase();
  const command = normalized === 'continuous' || normalized === 'start'
    ? 0x3e
    : (normalized === 'stop' || normalized === 'end' ? 0x3f : 0x3d);
  const label = command === 0x3e ? 'laser-range-continuous' : (command === 0x3f ? 'laser-range-stop' : 'laser-range-single');
  const frame = buildGimbalFrame({ command, param1: 0, param2: 0 });
  setGimbalFrame(frame, `${label}:${source}`, 'laser', 260, { command, mode: normalized });
  writeGimbalFrameBurst(frame, 5);
  addLog('GIMBAL', `Laser range command sent command=0x${command.toString(16)} mode=${normalized}`);
  return { command, mode: normalized, holdMs: 260 };
}

function sendGimbalJsonCommand(payloadText, source = 'web') {
  const { frame, payloadLength } = buildGimbalJsonFrame(payloadText);
  setGimbalFrame(frame, `json:${source}`, 'json', 240, { command: 0x90, payload: payloadText, payloadLength });
  writeGimbalFrameBurst(frame, 3);
  addLog('GIMBAL', `JSON command sent payload=${payloadText}`);
  return { command: 0x90, payload: payloadText, payloadLength, holdMs: 240 };
}

function waitForGimbalRx(previousLength, timeoutMs = 900) {
  return new Promise((resolve) => {
    const startedAt = Date.now();
    let lastLength = gimbalRxBuffer.length;
    let lastGrowthAt = 0;
    const timer = setInterval(() => {
      const now = Date.now();
      if (gimbalRxBuffer.length > lastLength) {
        lastLength = gimbalRxBuffer.length;
        lastGrowthAt = now;
      }
      const hasData = gimbalRxBuffer.length > previousLength;
      const quietEnough = hasData && lastGrowthAt && now - lastGrowthAt >= 250;
      if (quietEnough || now - startedAt >= timeoutMs) {
        clearInterval(timer);
        const rx = gimbalRxBuffer.subarray(previousLength);
        resolve({
          ascii: rx.toString('utf8').replace(/\0/g, ''),
          hex: rx.toString('hex'),
          bytes: rx.length,
          timeout: rx.length === 0
        });
      }
    }, 50);
  });
}

function sendGimbalDetectorPaused(paused, source = 'web') {
  const payload = `{"pausedetecor":"${paused ? '1' : '0'}"}`;
  const result = sendGimbalJsonCommand(payload, source);
  return { ...result, paused: Boolean(paused) };
}

function slewGimbalRate(nextX, nextY) {
  const x = gimbalLastRateX + clamp(nextX - gimbalLastRateX, -GIMBAL_RATE_SLEW_DPS, GIMBAL_RATE_SLEW_DPS);
  const y = gimbalLastRateY + clamp(nextY - gimbalLastRateY, -GIMBAL_RATE_SLEW_DPS, GIMBAL_RATE_SLEW_DPS);
  gimbalLastRateX = x;
  gimbalLastRateY = y;
  return { x, y };
}

function gimbalRateFromDelta(dx, dy, gain, useSlew = true) {
  function activeError(value) {
    const abs = Math.abs(value);
    if (abs <= GIMBAL_TRACK_HOLD_ZONE_PX) return 0;
    const trimmed = abs - GIMBAL_TRACK_HOLD_ZONE_PX;
    const normalized = Math.min(1, trimmed / Math.max(1, GIMBAL_TRACK_FAST_ZONE_PX - GIMBAL_TRACK_HOLD_ZONE_PX));
    const boost = 0.65 + normalized * normalized * 0.75;
    return Math.sign(value) * trimmed * boost;
  }
  const activeDx = activeError(dx);
  const activeDy = activeError(dy);
  const angles = gimbalAnglesFromPixelDelta(activeDx, activeDy);
  const rawX = activeDx === 0 ? 0 : angles.yawDeg * GIMBAL_TRACK_ANGLE_GAIN;
  const rawY = activeDy === 0 ? 0 : angles.pitchDeg * GIMBAL_TRACK_ANGLE_GAIN;
  const rate = {
    x: clamp(Math.round(rawX), -GIMBAL_MAX_RATE_DPS, GIMBAL_MAX_RATE_DPS),
    y: clamp(Math.round(rawY), -GIMBAL_MAX_RATE_DPS, GIMBAL_MAX_RATE_DPS)
  };
  return useSlew ? slewGimbalRate(rate.x, rate.y) : rate;
}

function sendGimbalTrackDelta(dx, dy, vx = 0, vy = 0, metadata = {}) {
  const delta = normalizeGimbalDelta(dx, dy);
  const now = Date.now();
  let measuredVx = asFiniteNumber(vx, 0) * GIMBAL_YAW_SIGN;
  let measuredVy = asFiniteNumber(vy, 0) * GIMBAL_PITCH_SIGN;
  if (Math.abs(measuredVx) < GIMBAL_TRACK_VELOCITY_DEADBAND_PX_S) measuredVx = 0;
  if (Math.abs(measuredVy) < GIMBAL_TRACK_VELOCITY_DEADBAND_PX_S) measuredVy = 0;
  const velocityAlpha = Boolean(metadata.coasting)
    ? Math.min(GIMBAL_TRACK_VELOCITY_ALPHA, 0.22)
    : GIMBAL_TRACK_VELOCITY_ALPHA;
  gimbalTrackFilteredVx = gimbalTrackFilteredVx * (1 - velocityAlpha) + measuredVx * velocityAlpha;
  gimbalTrackFilteredVy = gimbalTrackFilteredVy * (1 - velocityAlpha) + measuredVy * velocityAlpha;
  if (Boolean(metadata.coasting)) {
    gimbalTrackFilteredVx *= GIMBAL_TRACK_VELOCITY_DECAY;
    gimbalTrackFilteredVy *= GIMBAL_TRACK_VELOCITY_DECAY;
  }
  gimbalTrackTarget = {
    x: delta.x,
    y: delta.y,
    vx: gimbalTrackFilteredVx,
    vy: gimbalTrackFilteredVy,
    rawVx: measuredVx,
    rawVy: measuredVy,
    flowQuality: Number(metadata.flow_quality),
    coasting: Boolean(metadata.coasting),
    predictionAgeMs: Math.max(0, asFiniteNumber(metadata.prediction_age_ms, 0)),
    detectorAgeMs: Math.max(0, asFiniteNumber(metadata.detector_age_ms, 0))
  };
  gimbalTrackLastTargetAt = now;
  const desired = computeGimbalTrackDesiredRate(now);
  gimbalTrackDesiredRateX = desired.x;
  gimbalTrackDesiredRateY = desired.y;
  gimbalState.mode = 'track';
  gimbalState.lastCommand = `${gimbalState.trackMode || 'target'}-track`;
  startGimbalLoop();
  gimbalTxEnabled = true;
  return {
    ...delta,
    rateX: Math.round(gimbalLastRateX),
    rateY: Math.round(gimbalLastRateY),
    desiredRateX: Math.round(desired.x * 10) / 10,
    desiredRateY: Math.round(desired.y * 10) / 10,
    predictedX: Math.round(asFiniteNumber(desired.predictedX, delta.x)),
    predictedY: Math.round(asFiniteNumber(desired.predictedY, delta.y)),
    targetSpeed: Math.round(asFiniteNumber(desired.targetSpeed, 0)),
    rawVx: Math.round(asFiniteNumber(gimbalTrackTarget.rawVx, 0) * 10) / 10,
    rawVy: Math.round(asFiniteNumber(gimbalTrackTarget.rawVy, 0) * 10) / 10,
    filteredVx: Math.round(asFiniteNumber(gimbalTrackTarget.vx, 0) * 10) / 10,
    filteredVy: Math.round(asFiniteNumber(gimbalTrackTarget.vy, 0) * 10) / 10,
    leadMs: Math.round(asFiniteNumber(desired.leadMs, 0)),
    detectorAgeMs: Math.round(asFiniteNumber(desired.detectorAgeMs, 0)),
    angleGain: Math.round(asFiniteNumber(desired.angleGain, 0) * 100) / 100,
    limited: Boolean(desired.limited),
    smooth: true,
    gated: Boolean(desired.gated),
    axis: Math.abs(delta.x) >= Math.abs(delta.y) ? 'x' : 'y'
  };
}

function isGimbalTrackingActive() {
  return Boolean(gimbalTrackProcess && gimbalTrackProcess.exitCode === null && !gimbalTrackProcess.killed);
}

function cleanupStaleGimbalTrackWorkers(reason = 'cleanup') {
  const scriptMarkers = [
    path.join(PROJECT_ROOT, 'scripts', 'infer_video.py'),
    path.join(PROJECT_ROOT, 'scripts', 'face_track.py')
  ];
  const protectedPid = gimbalTrackProcess && gimbalTrackProcess.pid ? Number(gimbalTrackProcess.pid) : 0;
  for (const marker of scriptMarkers) {
    const result = spawnSync('pgrep', ['-f', marker], { encoding: 'utf8' });
    const pids = String(result.stdout || '')
      .split(/\s+/)
      .map((value) => Number(value))
      .filter((pid) => Number.isInteger(pid) && pid > 1 && pid !== process.pid && pid !== protectedPid);
    for (const pid of pids) {
      try {
        process.kill(pid, 'SIGTERM');
        addLog('GIMBAL_TRACK', `Killed stale tracker pid=${pid} reason=${reason}`);
      } catch (error) {
        addLog('GIMBAL_ERR', `Failed to kill stale tracker pid=${pid}: ${error.message}`);
      }
    }
  }
}

function normalizeGimbalTrackMode(value) {
  const mode = String(value || '').trim().toLowerCase();
  return mode === 'swimmer' ? 'swimmer' : 'face';
}

function gimbalTrackLostMessage(mode = gimbalState.trackMode) {
  return normalizeGimbalTrackMode(mode) === 'swimmer' ? 'can not find swimmer' : 'CAN NOT FIND FACE';
}

function startGimbalTracking(options = {}) {
  const requestedMode = normalizeGimbalTrackMode(options.mode || options.detectorMode || options.target);
  if (isGimbalTrackingActive()) return { ok: true, alreadyRunning: true, mode: gimbalState.trackMode };
  if (gimbalTrackRestartTimer) {
    clearTimeout(gimbalTrackRestartTimer);
    gimbalTrackRestartTimer = null;
  }
  gimbalTrackStopRequested = false;
  gimbalState.trackMode = requestedMode;
  cleanupStaleGimbalTrackWorkers(`start-${requestedMode}`);
  const sharedSource = 'http://127.0.0.1:8091/stream.mjpg';
  let detector = requestedMode;
  let source = sharedSource;
  let trackerThreadLimit = 2;
  let script;
  let args;
  if (requestedMode === 'swimmer') {
    const swimmer = gimbalConfig.swimmer || {};
    detector = String(swimmer.swimmer_detector || swimmer.detector || 'swimmer').trim().toLowerCase();
    source = String(swimmer.source || sharedSource);
    script = path.join(PROJECT_ROOT, 'scripts', 'infer_video.py');
    if (!fs.existsSync(script)) return { ok: false, error: 'infer_video swimmer tracker script not found' };
    const swimmerLoopHz = Math.max(1, Number(swimmer.loop_hz ?? 10) || 10);
    const swimmerMaxCoastFrames = Number.isFinite(Number(swimmer.max_coast_seconds))
      ? Math.max(1, Math.round(Number(swimmer.max_coast_seconds) * swimmerLoopHz))
      : Math.max(1, Math.round(Number(swimmer.max_coast ?? 45) || 45));
    trackerThreadLimit = Math.max(1, Math.round(Number(swimmer.detector_threads ?? 2) || 2));
    args = ['-u', script,
      '--source', source,
      '--weights', String(swimmer.weights || 'scripts/best.pt'),
      '--tracker', String(swimmer.tracker || 'scripts/bytetrack_swimmer.yaml'),
      '--conf', String(swimmer.conf ?? 0.1),
      '--iou', String(swimmer.iou ?? 0.5),
      '--imgsz', String(swimmer.imgsz ?? 640),
      '--device', String(swimmer.device ?? '0'),
      '--loop-hz', String(swimmerLoopHz),
      '--q', String(swimmer.q ?? 1.0),
      '--r', String(swimmer.r ?? 50.0),
      '--max-coast', String(swimmerMaxCoastFrames),
      '--reid-sim', String(swimmer.reid_sim ?? 0.5),
      '--gate-dist', String(swimmer.gate_dist ?? 140.0),
      '--gate-scale', String(swimmer.gate_scale ?? 2.2),
      '--smooth-alpha', String(swimmer.smooth_alpha ?? 0.3),
      '--max-center-speed', String(swimmer.max_center_speed ?? 800.0),
      '--max-size-rate', String(swimmer.max_size_rate ?? 1.0),
      '--hold-x-px', String(swimmer.hold_x_px ?? 360.0),
      '--hold-y-px', String(swimmer.hold_y_px ?? 600.0),
      '--hold-release', String(swimmer.hold_release ?? 150.0),
      '--conf-lock', String(swimmer.conf_lock ?? 0.35),
      '--size-tol', String(swimmer.size_tol ?? 0.35),
      '--vft-alpha', String(swimmer.vft_alpha ?? 0.35),
      '--deadzone-beta', String(swimmer.deadzone_beta ?? 0.15),
      '--center-median-window', String(swimmer.center_median_window ?? 11)
    ];
  } else {
    const face = gimbalConfig.face || {};
    detector = String(face.detector || 'yolo_face').trim().toLowerCase();
    source = String(face.source || sharedSource);
    script = path.join(PROJECT_ROOT, 'scripts', 'face_track.py');
    if (!fs.existsSync(script)) return { ok: false, error: 'face tracker script not found' };
    trackerThreadLimit = Math.max(1, Math.round(Number(face.detector_threads ?? 2) || 2));
    args = ['-u', script,
      '--source', source,
      '--model', String(face.model || 'scripts/models/yolov8n-face-lindevs.pt'),
      '--detector', detector,
      '--conf', String(face.conf ?? 0.45),
      '--iou', String(face.iou ?? 0.35),
      '--imgsz', String(face.imgsz ?? 416),
      '--loop-hz', String(face.loop_hz ?? 25),
      '--input-width', String(face.input_width ?? 320),
      '--input-height', String(face.input_height ?? 240),
      '--smooth-alpha', String(face.smooth_alpha ?? 0.82),
      '--max-center-speed', String(face.max_center_speed ?? 3300),
      '--static-jitter-px', String(face.static_jitter_px ?? 6),
      '--fast-move-px', String(face.fast_move_px ?? 55),
      '--prediction-seconds', String(face.prediction_seconds ?? 0.07),
      '--flow-scale', String(face.flow_scale ?? 0.35),
      '--flow-threads', String(face.flow_threads ?? 2),
      '--detector-threads', String(face.detector_threads ?? 2),
      '--max-coast-seconds', String(face.max_coast_seconds ?? 2)
    ];
    if (face.tracker) {
      args.push('--tracker', String(face.tracker));
    }
  }
  try {
    const trackerThreadLimitText = String(trackerThreadLimit);
    const childEnv = {
      ...process.env,
      MANTA_TRACK_THREADS: trackerThreadLimitText,
      OMP_NUM_THREADS: trackerThreadLimitText,
      OPENBLAS_NUM_THREADS: trackerThreadLimitText,
      MKL_NUM_THREADS: trackerThreadLimitText,
      VECLIB_MAXIMUM_THREADS: trackerThreadLimitText,
      NUMEXPR_NUM_THREADS: trackerThreadLimitText,
      TORCH_NUM_THREADS: trackerThreadLimitText,
      OPENCV_OPENCL_RUNTIME: 'disabled'
    };
    const child = spawn(PYTHON_EXEC, args, { cwd: PROJECT_ROOT, stdio: ['ignore', 'pipe', 'pipe'], env: childEnv });
    gimbalTrackProcess = child;
    gimbalTrackTarget = null;
    gimbalTrackHolding = true;
    gimbalTrackLastTargetAt = 0;
    gimbalTrackCommandPauseUntil = 0;
    resetGimbalTrackMotionState();
    gimbalState.trackingActive = true;
    gimbalState.trackWorkerActive = true;
    gimbalState.mode = 'track';
    updateGimbalTrackStatus({ status: 'starting', mode: requestedMode, message: gimbalTrackLostMessage(), workerActive: true });
    let stdoutBuf = '';
    child.stdout.on('data', (data) => {
      stdoutBuf += String(data);
      let index;
      while ((index = stdoutBuf.indexOf('\n')) !== -1) {
        const line = stdoutBuf.slice(0, index).trim();
        stdoutBuf = stdoutBuf.slice(index + 1);
        if (!line) continue;
        if (child !== gimbalTrackProcess || gimbalTrackStopRequested || !gimbalState.trackingActive) {
          continue;
        }
        if (line.startsWith('TARGET:')) {
          try {
            const target = JSON.parse(line.slice(7));
            const sent = sendGimbalTrackDelta(target.dx, target.dy, target.vx, target.vy, target);
            const lockedLabel = requestedMode === 'swimmer' ? 'SWIMMER LOCKED' : 'FACE LOCKED';
            const message = { ...target, mode: requestedMode, commandDx: sent.x, commandDy: sent.y, predictedDx: sent.predictedX, predictedDy: sent.predictedY, targetSpeed: sent.targetSpeed, rawVx: sent.rawVx, rawVy: sent.rawVy, filteredVx: sent.filteredVx, filteredVy: sent.filteredVy, leadMs: sent.leadMs, detectorAgeMs: sent.detectorAgeMs, angleGain: sent.angleGain, rateX: sent.rateX, rateY: sent.rateY, desiredRateX: sent.desiredRateX ?? sent.rateX, desiredRateY: sent.desiredRateY ?? sent.rateY, holdMs: sent.holdMs || 0, axis: sent.axis || '', pulse: Boolean(sent.pulse), smooth: Boolean(sent.smooth), gated: Boolean(sent.gated), limited: Boolean(sent.limited), locked: true, message: target.message || lockedLabel, workerActive: true, timestamp: Date.now() };
            gimbalState.lastTarget = message;
            updateGimbalTrackStatus({ ...message, status: target.status || 'track', detections: Number(target.detections || 0) });
            io.emit('gimbal_target', message);
          } catch (error) {
            addLog('GIMBAL_ERR', `Invalid target output: ${error.message}`);
          }
        } else if (line.startsWith('STATUS:')) {
          try {
            const status = JSON.parse(line.slice(7));
            holdGimbalTrackIdle(`${requestedMode}-not-found`);
            updateGimbalTrackStatus({ ...status, mode: requestedMode, locked: false, message: status.message || gimbalTrackLostMessage(), workerActive: true });
          } catch (error) {
            addLog('GIMBAL_ERR', `Invalid tracker status: ${error.message}`);
          }
        } else {
          addLog('GIMBAL_TRACK', line);
        }
      }
    });
    child.stderr.on('data', (data) => addLog('GIMBAL_TRACK_ERR', String(data).trimEnd()));
    child.on('exit', (code, signalName) => {
      addLog('GIMBAL_TRACK', `Tracker exited code=${code} signal=${signalName || ''}`);
      gimbalTrackProcess = null;
      gimbalState.trackWorkerActive = false;
      if (!gimbalTrackStopRequested && gimbalState.trackingActive) {
        holdGimbalTrackIdle(`${requestedMode}-worker-exit`);
        updateGimbalTrackStatus({ status: 'worker_exit', mode: requestedMode, message: gimbalTrackLostMessage(), code, signal: signalName || '', workerActive: false });
        gimbalTrackRestartTimer = setTimeout(() => {
          gimbalTrackRestartTimer = null;
          if (!gimbalTrackStopRequested && gimbalState.trackingActive) {
            startGimbalTracking({ mode: gimbalState.trackMode });
          }
        }, 1000);
        return;
      }
      gimbalState.trackingActive = false;
      if (gimbalState.mode === 'track') {
        gimbalState.mode = 'idle';
        gimbalLastFrame = buildGimbalFrame();
      }
      updateGimbalTrackStatus({ status: 'stopped', message: 'idle', workerActive: false });
    });
    addLog('GIMBAL', `Tracking started mode=${requestedMode} detector=${detector} source=${source}`);
    return { ok: true, mode: requestedMode, detector };
  } catch (error) {
    return { ok: false, error: error.message };
  }
}

function readHostBoardTemperature() {
  try {
    const entries = fs.readdirSync(THERMAL_CLASS_DIR, { withFileTypes: true });
    const temperatures = entries
      .filter((entry) => entry.name.startsWith('thermal_zone'))
      .map((entry) => path.join(THERMAL_CLASS_DIR, entry.name, 'temp'))
      .filter((tempPath) => fs.existsSync(tempPath))
      .map((tempPath) => Number(fs.readFileSync(tempPath, 'utf8').trim()))
      .map((rawValue) => (rawValue > 1000 ? rawValue / 1000 : rawValue))
      .filter((value) => Number.isFinite(value) && value >= -40 && value <= 150);

    if (temperatures.length === 0) {
      return null;
    }

    return Math.max(...temperatures);
  } catch (_error) {
    return null;
  }
}

function refreshHostBoardTemperature() {
  const nextTemperature = readHostBoardTemperature();
  const previousTemperature = systemState.telemetry.temperature.hostBoard;

  if (!Number.isFinite(nextTemperature)) {
    return;
  }

  if (previousTemperature === null || Math.abs(nextTemperature - previousTemperature) >= 0.2) {
    systemState.telemetry.temperature.hostBoard = nextTemperature;
    emitTelemetryUpdate();
  }
}

let gimbalRecordingProcess = null;
let gimbalRecordingState = null;

function recordingTimestamp(date = new Date()) {
  const pad = (value) => String(value).padStart(2, '0');
  return `${date.getFullYear()}${pad(date.getMonth() + 1)}${pad(date.getDate())}_${pad(date.getHours())}${pad(date.getMinutes())}${pad(date.getSeconds())}`;
}

function sanitizeRecordingName(value, fallback = recordingTimestamp()) {
  const raw = String(value || fallback).trim();
  const cleaned = raw
    .replace(/\.[a-z0-9]+$/i, '')
    .replace(/[\\/:*?"<>|]+/g, '-')
    .replace(/\s+/g, '_')
    .replace(/[^a-zA-Z0-9._\-\u4e00-\u9fff]/g, '')
    .replace(/_+/g, '_')
    .replace(/-+/g, '-')
    .replace(/^[._-]+|[._-]+$/g, '');
  return cleaned || fallback;
}

function uniqueRecordingPath(baseName) {
  const safe = sanitizeRecordingName(baseName);
  let candidate = path.join(RECORDINGS_DIR, `${safe}.mp4`);
  let index = 2;
  while (fs.existsSync(candidate)) {
    candidate = path.join(RECORDINGS_DIR, `${safe}_${index}.mp4`);
    index += 1;
  }
  return candidate;
}

function resolveRecordingPath(fileName) {
  const base = path.basename(String(fileName || ''));
  if (!base || !base.toLowerCase().endsWith('.mp4')) return null;
  const resolved = path.resolve(RECORDINGS_DIR, base);
  if (!resolved.startsWith(RECORDINGS_DIR + path.sep)) return null;
  return resolved;
}

function publicBaseUrl(req) {
  if (!req || !req.headers) return '';
  const proto = String(req.headers['x-forwarded-proto'] || req.protocol || 'http').split(',')[0].trim() || 'http';
  const host = String(req.headers['x-forwarded-host'] || req.headers.host || '').split(',')[0].trim();
  return host ? `${proto}://${host}` : '';
}

function recordingFileInfo(filePath, baseUrl = '') {
  const stat = fs.statSync(filePath);
  const name = path.basename(filePath);
  const streamPath = `/api/gimbal/recordings/${encodeURIComponent(name)}/stream`;
  const downloadPath = `/api/gimbal/recordings/${encodeURIComponent(name)}/download`;
  return {
    name,
    title: name.replace(/\.mp4$/i, ''),
    size: stat.size,
    modifiedAt: stat.mtime.toISOString(),
    createdAt: stat.birthtime.toISOString(),
    url: `${baseUrl}${streamPath}`,
    downloadUrl: `${baseUrl}${downloadPath}`,
    relativeUrl: streamPath,
    relativeDownloadUrl: downloadPath
  };
}

function listGimbalRecordings(baseUrl = '') {
  if (!fs.existsSync(RECORDINGS_DIR)) return [];
  return fs.readdirSync(RECORDINGS_DIR)
    .filter((name) => name.toLowerCase().endsWith('.mp4'))
    .map((name) => path.join(RECORDINGS_DIR, name))
    .filter((filePath) => fs.existsSync(filePath))
    .map((filePath) => recordingFileInfo(filePath, baseUrl))
    .sort((a, b) => String(b.modifiedAt).localeCompare(String(a.modifiedAt)));
}

function isGimbalRecordingActive() {
  return Boolean(gimbalRecordingProcess && gimbalRecordingProcess.exitCode === null && !gimbalRecordingProcess.killed);
}

function buildRecordingState() {
  return {
    active: isGimbalRecordingActive(),
    current: gimbalRecordingState ? { ...gimbalRecordingState, path: undefined } : null,
    directory: RECORDINGS_DIR,
    input: GIMBAL_RECORD_INPUT
  };
}

function startGimbalRecording(options = {}) {
  if (isGimbalRecordingActive()) return { ok: true, alreadyRunning: true, recording: buildRecordingState().current };
  const outputPath = uniqueRecordingPath(options.name || recordingTimestamp());
  const input = String(options.input || GIMBAL_RECORD_INPUT || GIMBAL_VIDEO_INPUT);
  const useRtsp = input.startsWith('rtsp://');
  const args = ['-hide_banner', '-loglevel', 'warning', '-y'];
  if (useRtsp) args.push('-rtsp_transport', String(gimbalVideoConfig.rtsp_transport || 'tcp'));
  args.push('-i', input, '-map', '0:v:0', '-an', '-c:v', GIMBAL_RECORD_CODEC);
  if (GIMBAL_RECORD_CODEC !== 'copy' && GIMBAL_RECORD_BITRATE) {
    args.push('-b:v', GIMBAL_RECORD_BITRATE);
  }
  args.push('-movflags', '+faststart', outputPath);
  try {
    const child = spawn(FFMPEG_BIN, args, { cwd: PROJECT_ROOT, stdio: ['ignore', 'ignore', 'pipe'] });
    gimbalRecordingProcess = child;
    gimbalRecordingState = {
      name: path.basename(outputPath),
      title: path.basename(outputPath, '.mp4'),
      path: outputPath,
      input,
      startedAt: new Date().toISOString()
    };
    child.stderr.on('data', (data) => addLog('GIMBAL_REC_ERR', String(data).trimEnd()));
    child.on('exit', (code, signalName) => {
      addLog('GIMBAL_REC', `Recording exited code=${code} signal=${signalName || ''}`);
      gimbalRecordingProcess = null;
      gimbalRecordingState = null;
      io.emit('gimbal_recording_state', buildRecordingState());
    });
    addLog('GIMBAL_REC', `Recording started ${outputPath}`);
    io.emit('gimbal_recording_state', buildRecordingState());
    return { ok: true, recording: buildRecordingState().current };
  } catch (error) {
    return { ok: false, error: error.message };
  }
}

function stopGimbalRecording() {
  if (isGimbalRecordingActive()) {
    try { gimbalRecordingProcess.kill('SIGINT'); } catch (_) {}
  }
  const stopped = buildRecordingState().current;
  gimbalRecordingProcess = null;
  gimbalRecordingState = null;
  io.emit('gimbal_recording_state', buildRecordingState());
  return { ok: true, recording: stopped };
}

app.get('/api/camera/snapshot', async (req, res) => {
  const localSnapshotUrl = String(cameraConfig.local_source_url || 'http://127.0.0.1:8090/snapshot.jpg');

  try {
    const response = await fetch(localSnapshotUrl, { cache: 'no-store' });
    if (!response.ok) {
      res.status(response.status).json({ success: false, message: `Camera snapshot unavailable: HTTP ${response.status}` });
      return;
    }

    const buffer = Buffer.from(await response.arrayBuffer());
    res.setHeader('Content-Type', response.headers.get('content-type') || 'image/jpeg');
    res.setHeader('Cache-Control', 'no-store, no-cache, must-revalidate');
    res.send(buffer);
  } catch (error) {
    res.status(503).json({ success: false, message: `Camera snapshot unavailable: ${error.message}` });
  }
});

app.get('/api/camera/stream', async (req, res) => {
  const localStreamUrl = String(cameraConfig.local_stream_url || 'http://127.0.0.1:8090/stream.mjpg');
  let upstreamUrl;
  try {
    upstreamUrl = new URL(localStreamUrl);
  } catch (error) {
    res.status(500).json({ success: false, message: `Invalid camera stream URL: ${error.message}` });
    return;
  }

  const client = upstreamUrl.protocol === 'https:' ? https : http;
  const upstreamReq = client.request(
    upstreamUrl,
    {
      method: 'GET',
      headers: {
        accept: 'multipart/x-mixed-replace'
      }
    },
    (upstreamRes) => {
      if (upstreamRes.statusCode && upstreamRes.statusCode >= 400) {
        res.status(upstreamRes.statusCode).json({ success: false, message: `Camera stream unavailable: HTTP ${upstreamRes.statusCode}` });
        upstreamRes.resume();
        return;
      }

      res.status(200);
      res.setHeader('Content-Type', upstreamRes.headers['content-type'] || 'multipart/x-mixed-replace; boundary=ffmpeg');
      res.setHeader('Cache-Control', 'no-store, no-cache, must-revalidate');
      res.setHeader('Connection', 'keep-alive');
      if (!res.headersSent) {
        res.flushHeaders();
      }

      upstreamRes.pipe(res);
      upstreamRes.on('error', () => {
        if (!res.writableEnded) {
          res.end();
        }
      });
    }
  );

  upstreamReq.on('error', (error) => {
    if (!res.headersSent) {
      res.status(503).json({ success: false, message: `Camera stream unavailable: ${error.message}` });
      return;
    }
    if (!res.writableEnded) {
      res.end();
    }
  });

  req.on('close', () => {
    upstreamReq.destroy();
  });

  upstreamReq.end();
});

app.get('/api/gimbal/video/stream', (_req, res) => {
  res.redirect(307, '/api/gimbal/stream');
});

app.get('/api/gimbal/stream', async (req, res) => {
  let upstreamUrl;
  try {
    upstreamUrl = new URL(GIMBAL_LOCAL_STREAM_URL);
  } catch (error) {
    res.status(500).json({ success: false, message: `Invalid gimbal stream URL: ${error.message}` });
    return;
  }

  const client = upstreamUrl.protocol === 'https:' ? https : http;
  const upstreamReq = client.request(
    upstreamUrl,
    {
      method: 'GET',
      headers: {
        accept: 'multipart/x-mixed-replace'
      }
    },
    (upstreamRes) => {
      if (upstreamRes.statusCode && upstreamRes.statusCode >= 400) {
        res.status(upstreamRes.statusCode).json({ success: false, message: `Gimbal stream unavailable: HTTP ${upstreamRes.statusCode}` });
        upstreamRes.resume();
        return;
      }

      res.status(200);
      res.setHeader('Content-Type', upstreamRes.headers['content-type'] || 'multipart/x-mixed-replace; boundary=ffmpeg');
      res.setHeader('Cache-Control', 'no-store, no-cache, must-revalidate');
      res.setHeader('Connection', 'keep-alive');
      if (!res.headersSent) {
        res.flushHeaders();
      }

      upstreamRes.pipe(res);
      upstreamRes.on('error', () => {
        if (!res.writableEnded) {
          res.end();
        }
      });
    }
  );

  upstreamReq.on('error', (error) => {
    if (!res.headersSent) {
      res.status(503).json({ success: false, message: `Gimbal stream unavailable: ${error.message}` });
      return;
    }
    if (!res.writableEnded) {
      res.end();
    }
  });

  req.on('close', () => {
    upstreamReq.destroy();
  });

  upstreamReq.end();
});

function validateChannelAndPwm(channel, pwm) {
  const normalizedChannel = Number(channel);
  const normalizedPwm = Number(pwm);

  if (!Number.isInteger(normalizedChannel) || normalizedChannel < 1 || normalizedChannel > 8) {
    return { ok: false, error: 'Invalid channel, must be an integer between 1 and 8' };
  }

  if (!enabledChannels.has(normalizedChannel)) {
    return { ok: false, error: `Channel ${normalizedChannel} is disabled in config` };
  }

  if (!Number.isFinite(normalizedPwm) || normalizedPwm < PWM_MIN || normalizedPwm > PWM_MAX) {
    return { ok: false, error: `PWM out of range (${PWM_MIN}-${PWM_MAX})` };
  }

  return {
    ok: true,
    channel: normalizedChannel,
    pwm: Math.round(normalizedPwm)
  };
}

function handleMotorControl(channel, pwm, sourceLabel = 'UNKNOWN') {
  const validation = validateChannelAndPwm(channel, pwm);
  if (!validation.ok) {
    return validation;
  }

  const { channel: validChannel, pwm: validPwm } = validation;
  refreshHardwareState();
  if (!systemState.isConnected && validPwm !== PWM_CENTER) {
    return { ok: false, error: 'Pixhawk heartbeat is offline; non-neutral motor command rejected' };
  }
  const leftKey = `ch${ROVER_LEFT_CHANNEL}`;
  const rightKey = `ch${ROVER_RIGHT_CHANNEL}`;

  systemState.motorStatus[`ch${validChannel}`] = validPwm;
  const leftPwm = toPwm(systemState.motorStatus[leftKey] ?? PWM_CENTER);
  const rightPwm = toPwm(systemState.motorStatus[rightKey] ?? PWM_CENTER);
  const throttlePwm = toPwm((leftPwm + rightPwm) / 2);
  const steeringPwm = toPwm(PWM_CENTER + (rightPwm - leftPwm) / 2);

  sendMavlinkCommand('ROVER_DRIVE', {
    throttleChannel: ROVER_THROTTLE_INPUT_CHANNEL,
    throttlePwm,
    steeringChannel: ROVER_STEERING_INPUT_CHANNEL,
    steeringPwm
  });

  io.emit('motor_update', {
    channel: validChannel,
    pwm: validPwm,
    timestamp: new Date().toISOString()
  });

  addLog(
    'MOTOR',
    `${sourceLabel} set Main${validChannel}=${validPwm}us via Pixhawk mixer (left=${leftPwm}, right=${rightPwm})`
  );
  return { ok: true };
}

function normalizeRoverControl(input = {}) {
  const throttleRaw = asFiniteNumber(input.throttle, 0);
  const steeringRaw = asFiniteNumber(input.steering, 0);

  const throttle = clamp(throttleRaw, ROVER_THROTTLE_MIN, ROVER_THROTTLE_MAX);
  const steering = clamp(steeringRaw, ROVER_STEERING_MIN, ROVER_STEERING_MAX);

  const throttleScale = (PWM_MAX - PWM_CENTER) / Math.max(Math.abs(ROVER_THROTTLE_MIN), Math.abs(ROVER_THROTTLE_MAX));
  const steeringScale = (PWM_MAX - PWM_CENTER) / Math.max(Math.abs(ROVER_STEERING_MIN), Math.abs(ROVER_STEERING_MAX));

  const throttleInputPwm = toPwm(PWM_CENTER + throttle * throttleScale);
  const steeringInputPwm = toPwm(PWM_CENTER + steering * steeringScale);
  const leftPwm = toPwm(PWM_CENTER + throttle * throttleScale - steering * steeringScale);
  const rightPwm = toPwm(PWM_CENTER + throttle * throttleScale + steering * steeringScale);

  return {
    throttle,
    steering,
    throttleInputPwm,
    steeringInputPwm,
    leftPwm,
    rightPwm,
    clamped: throttle !== throttleRaw || steering !== steeringRaw
  };
}

function applyRoverControl(controlInput = {}, sourceLabel = 'WEB') {
  const normalized = normalizeRoverControl(controlInput);
  refreshHardwareState();
  const neutralCommand = normalized.throttle === 0 && normalized.steering === 0;
  if (!systemState.isConnected && !neutralCommand) {
    addLog('ERROR', `${sourceLabel} non-neutral drive command rejected: Pixhawk heartbeat offline`);
    return { ok: false, error: 'Pixhawk heartbeat is offline; drive command rejected' };
  }
  sendMavlinkCommand('ROVER_DRIVE', {
    throttle: normalized.throttle,
    steering: normalized.steering,
    throttleChannel: ROVER_THROTTLE_INPUT_CHANNEL,
    steeringChannel: ROVER_STEERING_INPUT_CHANNEL,
    throttlePwm: normalized.throttleInputPwm,
    steeringPwm: normalized.steeringInputPwm
  });

  systemState.roverControl = {
    throttle: normalized.throttle,
    steering: normalized.steering,
    leftPwm: normalized.leftPwm,
    rightPwm: normalized.rightPwm
  };

  if (normalized.clamped) {
    addLog('SAFETY', `${sourceLabel} command clamped to throttle=${normalized.throttle}, steering=${normalized.steering}`);
  }

  systemState.motorStatus[`ch${ROVER_LEFT_CHANNEL}`] = normalized.leftPwm;
  systemState.motorStatus[`ch${ROVER_RIGHT_CHANNEL}`] = normalized.rightPwm;

  io.emit('rover_control_update', {
    ...systemState.roverControl,
    timestamp: new Date().toISOString()
  });

  return { ok: true, ...systemState.roverControl, clamped: normalized.clamped };
}

function updateTelemetry(newTelemetry = {}) {
  if (!newTelemetry || typeof newTelemetry !== 'object') {
    return;
  }

  const previous = systemState.telemetry;
  const nextTelemetry = {
    position: { ...previous.position },
    attitude: { ...previous.attitude },
    velocity: { ...previous.velocity },
    battery: { ...previous.battery },
    servoOutputs: { ...(previous.servoOutputs || {}) },
    temperature: { ...previous.temperature },
    gps: { ...previous.gps },
    uwb: { ...(previous.uwb || {}) },
    imuCalibration: { ...(previous.imuCalibration || createDefaultImuCalibrationState()) },
    flightMode: previous.flightMode,
    systemStatus: previous.systemStatus,
    armed: previous.armed
  };

  if (newTelemetry.position) {
    nextTelemetry.position.lat = asFiniteNumber(newTelemetry.position.lat, nextTelemetry.position.lat);
    nextTelemetry.position.lon = asFiniteNumber(newTelemetry.position.lon, nextTelemetry.position.lon);
    nextTelemetry.position.alt = asFiniteNumber(newTelemetry.position.alt, nextTelemetry.position.alt);
    if (typeof newTelemetry.position.source === 'string') {
      nextTelemetry.position.source = newTelemetry.position.source;
    }
    nextTelemetry.position.updatedAt = mergeNullableFiniteNumber(
      newTelemetry.position,
      'updatedAt',
      nextTelemetry.position.updatedAt
    );
  }

  if (newTelemetry.attitude) {
    nextTelemetry.attitude.roll = asFiniteNumber(newTelemetry.attitude.roll, nextTelemetry.attitude.roll);
    nextTelemetry.attitude.pitch = asFiniteNumber(newTelemetry.attitude.pitch, nextTelemetry.attitude.pitch);
    nextTelemetry.attitude.yaw = asFiniteNumber(newTelemetry.attitude.yaw, nextTelemetry.attitude.yaw);
  }

  if (newTelemetry.velocity) {
    nextTelemetry.velocity.vx = asFiniteNumber(newTelemetry.velocity.vx, nextTelemetry.velocity.vx);
    nextTelemetry.velocity.vy = asFiniteNumber(newTelemetry.velocity.vy, nextTelemetry.velocity.vy);
    nextTelemetry.velocity.vz = asFiniteNumber(newTelemetry.velocity.vz, nextTelemetry.velocity.vz);
  }

  if (newTelemetry.battery) {
    nextTelemetry.battery.voltage = asFiniteNumber(newTelemetry.battery.voltage, nextTelemetry.battery.voltage);
    nextTelemetry.battery.current = asFiniteNumber(newTelemetry.battery.current, nextTelemetry.battery.current);
    nextTelemetry.battery.percentage = asFiniteNumber(newTelemetry.battery.percentage, nextTelemetry.battery.percentage);
  }

  if (newTelemetry.servoOutputs && typeof newTelemetry.servoOutputs === 'object') {
    for (const [channel, pwm] of Object.entries(newTelemetry.servoOutputs)) {
      const normalizedPwm = asFiniteNumber(pwm, 0);
      nextTelemetry.servoOutputs[channel] = normalizedPwm;
      if (/^ch[1-8]$/i.test(channel) && normalizedPwm > 0) {
        systemState.motorStatus[channel.toLowerCase()] = normalizedPwm;
      }
    }
  }

  if (newTelemetry.temperature) {
    nextTelemetry.temperature.hostBoard = mergeOptionalFiniteNumber(newTelemetry.temperature.hostBoard, nextTelemetry.temperature.hostBoard);
    nextTelemetry.temperature.flightController = mergeOptionalFiniteNumber(newTelemetry.temperature.flightController, nextTelemetry.temperature.flightController);
    nextTelemetry.temperature.motorLeft = mergeOptionalFiniteNumber(newTelemetry.temperature.motorLeft, nextTelemetry.temperature.motorLeft);
    nextTelemetry.temperature.motorRight = mergeOptionalFiniteNumber(newTelemetry.temperature.motorRight, nextTelemetry.temperature.motorRight);
  }

  if (newTelemetry.gps) {
    nextTelemetry.gps.satellites = asFiniteNumber(newTelemetry.gps.satellites, nextTelemetry.gps.satellites);
    nextTelemetry.gps.hdop = asFiniteNumber(newTelemetry.gps.hdop, nextTelemetry.gps.hdop);
    nextTelemetry.gps.fixType = asFiniteNumber(newTelemetry.gps.fixType, nextTelemetry.gps.fixType);
    nextTelemetry.gps.latitude = asFiniteNumber(newTelemetry.gps.latitude, nextTelemetry.gps.latitude);
    nextTelemetry.gps.longitude = asFiniteNumber(newTelemetry.gps.longitude, nextTelemetry.gps.longitude);
    nextTelemetry.gps.altitude = asFiniteNumber(newTelemetry.gps.altitude, nextTelemetry.gps.altitude);
    nextTelemetry.gps.horizontalAccuracy = mergeNullableFiniteNumber(
      newTelemetry.gps,
      'horizontalAccuracy',
      nextTelemetry.gps.horizontalAccuracy
    );
    nextTelemetry.gps.verticalAccuracy = mergeNullableFiniteNumber(
      newTelemetry.gps,
      'verticalAccuracy',
      nextTelemetry.gps.verticalAccuracy
    );
    nextTelemetry.gps.groundSpeed = mergeNullableFiniteNumber(
      newTelemetry.gps,
      'groundSpeed',
      nextTelemetry.gps.groundSpeed
    );
    nextTelemetry.gps.course = mergeNullableFiniteNumber(newTelemetry.gps, 'course', nextTelemetry.gps.course);
    nextTelemetry.gps.updatedAt = mergeNullableFiniteNumber(newTelemetry.gps, 'updatedAt', nextTelemetry.gps.updatedAt);
  }

  if (newTelemetry.uwb && typeof newTelemetry.uwb === 'object') {
    const incomingUwb = newTelemetry.uwb;
    for (const key of ['distanceM', 'azimuthDeg', 'elevationDeg', 'rawDistanceM', 'rawAzimuthDeg', 'rawElevationDeg', 'updatedAt']) {
      if (Object.prototype.hasOwnProperty.call(incomingUwb, key)) {
        nextTelemetry.uwb[key] = incomingUwb[key] === null
          ? null
          : asFiniteNumber(incomingUwb[key], nextTelemetry.uwb[key]);
      }
    }
    nextTelemetry.uwb.goodCount = asFiniteNumber(incomingUwb.goodCount, nextTelemetry.uwb.goodCount || 0);
    if (typeof incomingUwb.online === 'boolean') nextTelemetry.uwb.online = incomingUwb.online;
    if (typeof incomingUwb.fresh === 'boolean') nextTelemetry.uwb.fresh = incomingUwb.fresh;
    if (typeof incomingUwb.source === 'string') nextTelemetry.uwb.source = incomingUwb.source;
    if (typeof incomingUwb.filter === 'string') nextTelemetry.uwb.filter = incomingUwb.filter;
  }

  if (newTelemetry.imuCalibration && typeof newTelemetry.imuCalibration === 'object') {
    const incomingCalibration = newTelemetry.imuCalibration;

    if (typeof incomingCalibration.active === 'boolean') {
      nextTelemetry.imuCalibration.active = incomingCalibration.active;
    }

    if (typeof incomingCalibration.mode === 'string' && incomingCalibration.mode.trim()) {
      nextTelemetry.imuCalibration.mode = incomingCalibration.mode.trim().toUpperCase();
    }

    if (typeof incomingCalibration.status === 'string' && incomingCalibration.status.trim()) {
      nextTelemetry.imuCalibration.status = incomingCalibration.status.trim().toUpperCase();
    }

    if (typeof incomingCalibration.step === 'string') {
      nextTelemetry.imuCalibration.step = incomingCalibration.step.trim().toUpperCase();
    }

    if (Object.prototype.hasOwnProperty.call(incomingCalibration, 'stepCode')) {
      if (incomingCalibration.stepCode === null || incomingCalibration.stepCode === '') {
        nextTelemetry.imuCalibration.stepCode = null;
      } else {
        const stepCode = Number.parseInt(String(incomingCalibration.stepCode), 10);
        nextTelemetry.imuCalibration.stepCode = Number.isFinite(stepCode) ? stepCode : nextTelemetry.imuCalibration.stepCode;
      }
    }

    if (typeof incomingCalibration.instructions === 'string') {
      nextTelemetry.imuCalibration.instructions = incomingCalibration.instructions.trim();
    }

    if (Object.prototype.hasOwnProperty.call(incomingCalibration, 'progress')) {
      if (incomingCalibration.progress === null || incomingCalibration.progress === '') {
        nextTelemetry.imuCalibration.progress = null;
      } else {
        nextTelemetry.imuCalibration.progress = asFiniteNumber(
          incomingCalibration.progress,
          nextTelemetry.imuCalibration.progress ?? 0
        );
      }
    }

    if (Object.prototype.hasOwnProperty.call(incomingCalibration, 'lastAckCommand')) {
      if (incomingCalibration.lastAckCommand === null || incomingCalibration.lastAckCommand === '') {
        nextTelemetry.imuCalibration.lastAckCommand = null;
      } else {
        nextTelemetry.imuCalibration.lastAckCommand = asFiniteNumber(
          incomingCalibration.lastAckCommand,
          nextTelemetry.imuCalibration.lastAckCommand ?? 0
        );
      }
    }

    if (typeof incomingCalibration.lastAckResult === 'string') {
      nextTelemetry.imuCalibration.lastAckResult = incomingCalibration.lastAckResult.trim().toUpperCase();
    }

    if (Object.prototype.hasOwnProperty.call(incomingCalibration, 'updatedAt')) {
      nextTelemetry.imuCalibration.updatedAt = asFiniteNumber(
        incomingCalibration.updatedAt,
        nextTelemetry.imuCalibration.updatedAt ?? Date.now()
      );
    }
  }

  if (typeof newTelemetry.flightMode === 'string' && newTelemetry.flightMode.trim()) {
    nextTelemetry.flightMode = newTelemetry.flightMode.trim();
  }

  if (typeof newTelemetry.systemStatus === 'string' && newTelemetry.systemStatus.trim()) {
    nextTelemetry.systemStatus = newTelemetry.systemStatus.trim();
  }

  if (typeof newTelemetry.armed === 'boolean') {
    nextTelemetry.armed = newTelemetry.armed;
  }

  if (newTelemetry.hardware && typeof newTelemetry.hardware === 'object') {
    const incomingHardware = newTelemetry.hardware;
    if (incomingHardware.pixhawk && typeof incomingHardware.pixhawk === 'object') {
      systemState.hardware.pixhawk = {
        ...systemState.hardware.pixhawk,
        ...incomingHardware.pixhawk
      };
    }
    if (incomingHardware.motors && typeof incomingHardware.motors === 'object') {
      for (const side of ['left', 'right']) {
        if (incomingHardware.motors[side] && typeof incomingHardware.motors[side] === 'object') {
          systemState.hardware.motors[side] = {
            ...systemState.hardware.motors[side],
            ...incomingHardware.motors[side]
          };
        }
      }
    }
  }

  systemState.telemetry = nextTelemetry;
  refreshHardwareState();
  emitTelemetryUpdate();
  appendTelemetryCsv(systemState.telemetry);
}

function updateConnectionStatus(isConnected) {
  const previousStatus = systemState.pixhawkStatus;
  systemState.isConnected = Boolean(isConnected);
  if (isConnected) {
    systemState.hardware.pixhawk.lastSeenAt = Date.now();
  }
  refreshHardwareState();

  const payload = {
    isConnected: systemState.isConnected,
    status: systemState.pixhawkStatus,
    timestamp: new Date().toISOString()
  };

  io.emit('connection_status', payload);
  if (previousStatus !== payload.status) {
    addLog(payload.isConnected ? 'INFO' : 'ERROR', `Pixhawk connection: ${payload.status}`);
  }
}

telemetrySocket.on('message', (rawMessage) => {
  try {
    const packet = JSON.parse(rawMessage.toString('utf8'));

    if (packet.type === 'telemetry') {
      updateTelemetry(packet.payload || {});
      if (!systemState.isConnected) {
        updateConnectionStatus(true);
      }
      return;
    }

    if (packet.type === 'connection') {
      updateConnectionStatus(Boolean(packet.payload && packet.payload.connected));
      return;
    }

    if (packet.type === 'log' && packet.payload && packet.payload.message) {
      addLog(packet.payload.level || 'INFO', `[Bridge] ${packet.payload.message}`);
    }
  } catch (error) {
    addLog('ERROR', `Invalid telemetry packet: ${error.message}`);
  }
});

telemetrySocket.on('error', (error) => {
  addLog('ERROR', `Telemetry socket error: ${error.message}`);
});

telemetrySocket.bind(BRIDGE_TELEMETRY_PORT, '127.0.0.1', () => {
  addLog('INFO', `Listening bridge telemetry on 127.0.0.1:${BRIDGE_TELEMETRY_PORT}`);
});

refreshHostBoardTemperature();
setInterval(refreshHostBoardTemperature, 5000);
setInterval(refreshPeripheralState, 5000);
setInterval(refreshHardwareState, 1000);
if (GIMBAL_AUTO_CONNECT) {
  startGimbalLoop();
  openGimbalPort();
}

app.get('/api/status', (req, res) => {
  refreshPeripheralState();
  refreshHardwareState();
  systemState.camera = readCameraState(systemState.connectivity, req);
  res.json({
    success: true,
    timestamp: new Date().toISOString(),
    data: systemState,
    limits: {
      throttle: { min: ROVER_THROTTLE_MIN, max: ROVER_THROTTLE_MAX },
      steering: { min: ROVER_STEERING_MIN, max: ROVER_STEERING_MAX },
      pwm: { min: PWM_MIN, max: PWM_MAX, center: PWM_CENTER }
    },
    roverChannels: {
      left: ROVER_LEFT_CHANNEL,
      right: ROVER_RIGHT_CHANNEL
    },
    roverInputs: {
      steering: ROVER_STEERING_INPUT_CHANNEL,
      throttle: ROVER_THROTTLE_INPUT_CHANNEL
    }
  });
});

app.get('/api/telemetry', (req, res) => {
  res.json({
    success: true,
    timestamp: new Date().toISOString(),
    telemetry: systemState.telemetry
  });
});

app.get('/api/motors', (req, res) => {
  res.json({
    success: true,
    motors: systemState.motorStatus,
    roverControl: systemState.roverControl
  });
});

app.get('/api/logs', (req, res) => {
  const requested = Number.parseInt(String(req.query.limit || '100'), 10);
  const limit = Number.isFinite(requested) && requested > 0 ? requested : 100;
  res.json({
    success: true,
    logs: systemState.logs.slice(-limit)
  });
});

app.get('/api/logs/download', (req, res) => {
  if (!fs.existsSync(FLIGHT_CSV_FILE)) {
    res.status(404).json({ success: false, message: 'No flight log available yet' });
    return;
  }
  res.download(FLIGHT_CSV_FILE, 'flight_data.csv');
});

app.post('/api/control/rover', (req, res) => {
  const { throttle, steering } = req.body || {};
  const sourceLabel = isVisionActive() ? 'VISION' : 'REST';
  const result = applyRoverControl({ throttle, steering }, sourceLabel);
  if (!result.ok) {
    res.status(400).json({ success: false, message: result.error });
    return;
  }
  res.json({ success: true, data: result });
});

app.post('/api/control/motor', (req, res) => {
  const body = req.body || {};
  const items = Array.isArray(body.motors) ? body.motors : [{ channel: body.channel, pwm: body.pwm }];
  const results = items.map((it) => handleMotorControl(it.channel, it.pwm, isVisionActive() ? 'VISION' : 'REST'));
  const bad = results.find((r) => !r.ok);
  if (bad) return res.status(400).json({ success: false, message: bad.error });
  res.json({ success: true });
});

const VISION_SCRIPT = path.join(PROJECT_ROOT, 'scripts', 'vision_face_controller.py');
const PYTHON_EXEC = process.env.PYTHON_EXEC || 'python3';
let visionProcess = null;

function isVisionActive() {
  return Boolean(visionProcess && visionProcess.exitCode === null && !visionProcess.killed);
}

function startVision() {
  if (isVisionActive()) return { ok: true, alreadyRunning: true };
  if (!fs.existsSync(VISION_SCRIPT)) return { ok: false, error: 'vision script not found' };
  try {
    const child = spawn(PYTHON_EXEC, ['-u', VISION_SCRIPT], {
      cwd: PROJECT_ROOT,
      stdio: ['ignore', 'pipe', 'pipe']
    });
    visionProcess = child;
    let stdoutBuf = '';
    child.stdout.on('data', (d) => {
      stdoutBuf += String(d);
      let idx;
      while ((idx = stdoutBuf.indexOf('\n')) !== -1) {
        const line = stdoutBuf.slice(0, idx).trimEnd();
        stdoutBuf = stdoutBuf.slice(idx + 1);
        if (!line) continue;
        if (line.startsWith('DETECT:')) {
          try {
            const det = JSON.parse(line.slice(7));
            systemState.vision.detections = det;
            io.emit('vision_detections', det);
          } catch (_) {}
        } else {
          addLog('VISION', line);
        }
      }
    });
    child.stderr.on('data', (d) => addLog('VISION_ERR', String(d).trimEnd()));
    child.on('exit', (code, signalName) => {
      addLog('VISION', `Vision controller exited code=${code} signal=${signalName || ''}`);
      visionProcess = null;
      systemState.vision.active = false;
      io.emit('vision_state', systemState.vision);
    });
    systemState.vision.active = true;
    io.emit('vision_state', systemState.vision);
    addLog('COMMAND', 'Vision tracking started');
    return { ok: true };
  } catch (error) {
    return { ok: false, error: error.message };
  }
}

function stopVision() {
  if (isVisionActive()) {
    try { visionProcess.kill('SIGTERM'); } catch (_) {}
  }
  systemState.vision.active = false;
  applyRoverControl({ throttle: 0, steering: 0 }, 'VISION_STOP');
  io.emit('vision_state', systemState.vision);
  addLog('COMMAND', 'Vision tracking stopped');
  return { ok: true };
}

app.post('/api/vision/start', (_req, res) => {
  stopVision();
  res.status(410).json({
    success: false,
    active: false,
    message: 'Deprecated rover vision controller is disabled. Use /api/gimbal/track/start with mode face or swimmer.'
  });
});

app.post('/api/vision/stop', (_req, res) => {
  stopVision();
  res.json({ success: true, active: false });
});

app.get('/api/vision/state', (_req, res) => {
  res.json({ success: true, active: isVisionActive(), detections: systemState.vision.detections });
});

app.get('/api/gimbal/state', (_req, res) => {
  updateGimbalDiagnostics();
  res.json({ success: true, state: { ...gimbalState, trackingActive: Boolean(gimbalState.trackingActive), trackWorkerActive: isGimbalTrackingActive() } });
});

app.get('/api/gimbal/recording/state', (_req, res) => {
  res.json({ success: true, state: buildRecordingState() });
});

app.post('/api/gimbal/recording/start', (req, res) => {
  if (!gimbalStream || !gimbalState.connected) {
    openGimbalPort('recording');
  }
  let protocol = null;
  let streamProfile = null;
  if (gimbalStream) {
    sendGimbalOsd(0, 'recording');
    streamProfile = sendGimbalStreamProfile(
      req.body && Number.isFinite(Number(req.body.streamIndex)) ? Number(req.body.streamIndex) : GIMBAL_RECORD_STREAM_INDEX,
      req.body && Number.isFinite(Number(req.body.quality)) ? Number(req.body.quality) : GIMBAL_RECORD_STREAM_QUALITY,
      'recording'
    );
    protocol = sendGimbalRecordCommand('start', 'recording');
  }
  const result = startGimbalRecording(req.body || {});
  if (!result.ok) {
    res.status(500).json({ success: false, message: result.error, state: buildRecordingState() });
    return;
  }
  res.json({ success: true, active: true, alreadyRunning: Boolean(result.alreadyRunning), protocol, streamProfile, recording: result.recording, state: buildRecordingState() });
});

app.post('/api/gimbal/recording/stop', (req, res) => {
  let protocol = null;
  if (gimbalStream) {
    protocol = sendGimbalRecordCommand('stop', 'recording');
  }
  const result = stopGimbalRecording();
  res.json({ success: true, active: false, protocol, recording: result.recording, state: buildRecordingState(), recordings: listGimbalRecordings(publicBaseUrl(req)) });
});

app.get('/api/gimbal/recordings', (req, res) => {
  res.json({ success: true, recordings: listGimbalRecordings(publicBaseUrl(req)), recording: buildRecordingState() });
});

app.get('/api/gimbal/recordings/:name/stream', (req, res) => {
  const filePath = resolveRecordingPath(req.params.name);
  if (!filePath || !fs.existsSync(filePath)) {
    res.status(404).json({ success: false, message: 'Recording not found' });
    return;
  }
  res.sendFile(filePath);
});

app.get('/api/gimbal/recordings/:name/download', (req, res) => {
  const filePath = resolveRecordingPath(req.params.name);
  if (!filePath || !fs.existsSync(filePath)) {
    res.status(404).json({ success: false, message: 'Recording not found' });
    return;
  }
  res.download(filePath, path.basename(filePath));
});

app.patch('/api/gimbal/recordings/:name', (req, res) => {
  const filePath = resolveRecordingPath(req.params.name);
  if (!filePath || !fs.existsSync(filePath)) {
    res.status(404).json({ success: false, message: 'Recording not found' });
    return;
  }
  const targetPath = uniqueRecordingPath((req.body && req.body.name) || path.basename(filePath, '.mp4'));
  fs.renameSync(filePath, targetPath);
  res.json({ success: true, recording: recordingFileInfo(targetPath, publicBaseUrl(req)), recordings: listGimbalRecordings(publicBaseUrl(req)) });
});

app.delete('/api/gimbal/recordings/:name', (req, res) => {
  const filePath = resolveRecordingPath(req.params.name);
  if (!filePath || !fs.existsSync(filePath)) {
    res.status(404).json({ success: false, message: 'Recording not found' });
    return;
  }
  fs.unlinkSync(filePath);
  res.json({ success: true, recordings: listGimbalRecordings(publicBaseUrl(req)) });
});

app.post('/api/gimbal/connect', (_req, res) => {
  const ok = openGimbalPort('web');
  if (ok) {
    gimbalTxEnabled = false;
    gimbalHoldUntil = 0;
    gimbalLastFrame = buildGimbalFrame();
    gimbalState.mode = 'idle';
    gimbalState.lastCommand = 'connected';
  }
  res.status(ok ? 200 : 500).json({ success: ok, state: gimbalState, message: ok ? 'connecting' : gimbalState.lastError });
});

app.post('/api/gimbal/disconnect', (_req, res) => {
  disconnectGimbalPort();
  res.json({ success: true, state: gimbalState });
});

app.post('/api/gimbal/home', (req, res) => {
  if (!gimbalStream || !gimbalState.connected) {
    gimbalPendingHomeSource = 'web';
    const opened = openGimbalPort('web');
    if (!opened) {
      gimbalPendingHomeSource = '';
      res.status(500).json({ success: false, message: gimbalState.lastError || 'Gimbal serial open failed.', state: gimbalState });
      return;
    }
    res.json({ success: true, pending: true, state: gimbalState });
    return;
  }
  const body = req.body || {};
  sendGimbalHome('web', { preserveTracking: body.preserveTracking !== false });
  res.json({ success: true, state: gimbalState });
});

app.post('/api/gimbal/stop', (_req, res) => {
  stopGimbalSerial('web');
  res.json({ success: true, state: gimbalState });
});

app.post('/api/gimbal/focus/auto', (_req, res) => {
  if (!gimbalStream || !gimbalState.connected) {
    const opened = openGimbalPort('web');
    if (!opened) {
      res.status(500).json({ success: false, message: gimbalState.lastError || 'Gimbal serial open failed.', state: gimbalState });
      return;
    }
  }
  const result = sendGimbalZoomReset('web');
  res.json({ success: true, result, state: gimbalState, message: 'Protocol 45H resets visible-light zoom; this protocol does not define autofocus.' });
});

app.post('/api/gimbal/osd', (req, res) => {
  if (!gimbalStream || !gimbalState.connected) {
    const opened = openGimbalPort('web');
    if (!opened) {
      res.status(500).json({ success: false, message: gimbalState.lastError || 'Gimbal serial open failed.', state: gimbalState });
      return;
    }
  }
  const mode = req.body && Number.isFinite(Number(req.body.mode)) ? Number(req.body.mode) : 0;
  const result = sendGimbalOsd(mode, 'web');
  res.json({ success: true, result, state: gimbalState });
});

app.post('/api/gimbal/stream/profile', (req, res) => {
  if (!gimbalStream || !gimbalState.connected) {
    const opened = openGimbalPort('web');
    if (!opened) {
      res.status(500).json({ success: false, message: gimbalState.lastError || 'Gimbal serial open failed.', state: gimbalState });
      return;
    }
  }
  const body = req.body || {};
  const result = sendGimbalStreamProfile(body.streamIndex, body.quality, 'web');
  res.json({ success: true, result, state: gimbalState });
});

app.post('/api/gimbal/range', (req, res) => {
  if (!gimbalStream || !gimbalState.connected) {
    const opened = openGimbalPort('web');
    if (!opened) {
      res.status(500).json({ success: false, message: gimbalState.lastError || 'Gimbal serial open failed.', state: gimbalState });
      return;
    }
  }
  const body = req.body || {};
  const result = sendGimbalLaserRange(body.mode || body.action || 'single', 'web');
  res.json({ success: true, result, rangeM: gimbalState.feedback ? gimbalState.feedback.laserRangeM : null, state: gimbalState });
});

app.post('/api/gimbal/track/cancel', (_req, res) => {
  if (!gimbalStream || !gimbalState.connected) {
    const opened = openGimbalPort('web');
    if (!opened) {
      res.status(500).json({ success: false, message: gimbalState.lastError || 'Gimbal serial open failed.', state: gimbalState });
      return;
    }
  }
  const result = sendGimbalCancelTrack('web');
  res.json({ success: true, result, state: gimbalState });
});

app.post('/api/gimbal/detector/start', (_req, res) => {
  if (!gimbalStream || !gimbalState.connected) {
    const opened = openGimbalPort('web');
    if (!opened) {
      res.status(500).json({ success: false, message: gimbalState.lastError || 'Gimbal serial open failed.', state: gimbalState });
      return;
    }
  }
  const result = sendGimbalDetectorPaused(false, 'web');
  res.json({ success: true, result, state: gimbalState });
});

app.post('/api/gimbal/detector/stop', (_req, res) => {
  if (!gimbalStream || !gimbalState.connected) {
    const opened = openGimbalPort('web');
    if (!opened) {
      res.status(500).json({ success: false, message: gimbalState.lastError || 'Gimbal serial open failed.', state: gimbalState });
      return;
    }
  }
  const result = sendGimbalDetectorPaused(true, 'web');
  res.json({ success: true, result, state: gimbalState });
});

app.post('/api/gimbal/json', async (req, res) => {
  if (!gimbalStream || !gimbalState.connected) {
    const opened = openGimbalPort('web');
    if (!opened) {
      res.status(500).json({ success: false, message: gimbalState.lastError || 'Gimbal serial open failed.', state: gimbalState });
      return;
    }
  }
  const body = req.body || {};
  const payloadObject = body.payload && typeof body.payload === 'object' && !Array.isArray(body.payload)
    ? body.payload
    : body;
  const payloadText = JSON.stringify(payloadObject);
  if (payloadText.length > 35) {
    res.status(400).json({ success: false, message: 'Gimbal JSON payload is limited to 35 ASCII bytes by the current frame format.', payloadText });
    return;
  }
  const beforeLength = gimbalRxBuffer.length;
  const result = sendGimbalJsonCommand(payloadText, 'api');
  const rx = await waitForGimbalRx(beforeLength, Number(body.timeoutMs || 1200));
  res.json({ success: true, result, rx, state: gimbalState });
});

app.post('/api/gimbal/click', (req, res) => {
  if (!gimbalStream || !gimbalState.connected) {
    const opened = openGimbalPort('web');
    if (!opened) {
      res.status(500).json({ success: false, message: gimbalState.lastError || 'Gimbal serial open failed.', state: gimbalState });
      return;
    }
  }
  if (!gimbalStream) {
    res.status(500).json({ success: false, message: 'Gimbal serial open failed.', state: gimbalState });
    return;
  }
  const body = req.body || {};
  stopGimbalTracking(false);
  sendGimbalDetectorPaused(true, 'web:preclick');
  sendGimbalCancelTrack('web:preclick');
  const delta = sendGimbalClickTarget(body.dx, body.dy);
  res.json({ success: true, delta, state: gimbalState });
});

app.post('/api/gimbal/track/start', (req, res) => {
  if (!gimbalStream || !gimbalState.connected) {
    const opened = openGimbalPort('web');
    if (!opened) {
      res.status(500).json({ success: false, message: gimbalState.lastError || 'Gimbal serial open failed.', state: gimbalState });
      return;
    }
  }
  if (!gimbalStream) {
    res.status(500).json({ success: false, message: 'Gimbal serial open failed.', state: gimbalState });
    return;
  }
  const body = req.body || {};
  const result = startGimbalTracking({ mode: body.mode || body.target || body.detectorMode });
  if (!result.ok) {
    res.status(500).json({ success: false, message: result.error, state: gimbalState });
    return;
  }
  res.json({ success: true, active: true, alreadyRunning: Boolean(result.alreadyRunning), mode: result.mode || gimbalState.trackMode, detector: result.detector || '', state: gimbalState });
});

app.post('/api/gimbal/track/stop', (_req, res) => {
  stopGimbalTracking(true);
  res.json({ success: true, active: false, state: gimbalState });
});

app.post('/api/calibration/imu/start', (req, res) => {
  const blockReason = calibrationAllowedError();
  if (blockReason) {
    res.status(409).json({ success: false, message: blockReason });
    return;
  }

  const calibrationType = normalizeCalibrationType(req.body && req.body.type);
  if (!calibrationType) {
    res.status(400).json({ success: false, message: 'Invalid IMU calibration type. Only ACCEL or LEVEL is supported.' });
    return;
  }

  sendMavlinkCommand('IMU_CALIBRATION_START', { type: calibrationType });

  systemState.telemetry.imuCalibration = {
    ...systemState.telemetry.imuCalibration,
    active: true,
    mode: calibrationType,
    status: 'STARTING',
    step: '',
    stepCode: null,
    instructions: calibrationType === 'ACCEL'
      ? '6-point IMU calibration started. Waiting for the FCU pose request.'
      : 'Level calibration command sent. Keep the rover level and still.',
    progress: calibrationType === 'ACCEL' ? 0 : null,
    lastAckCommand: 241,
    lastAckResult: '',
    updatedAt: Date.now()
  };

  emitTelemetryUpdate();
  addLog('COMMAND', calibrationType === 'ACCEL' ? 'Started IMU 6-position calibration' : 'Started IMU level calibration');

  res.json({
    success: true,
    message: calibrationType === 'ACCEL' ? '6-point IMU calibration started' : 'Level calibration started',
    telemetry: systemState.telemetry
  });
});

app.post('/api/calibration/imu/confirm', (req, res) => {
  const blockReason = calibrationAllowedError();
  if (blockReason) {
    res.status(409).json({ success: false, message: blockReason });
    return;
  }

  if (!systemState.telemetry.imuCalibration.active) {
    res.status(409).json({ success: false, message: 'There is no active 6-point IMU calibration.' });
    return;
  }

  const requestedStepCode = req.body && Object.prototype.hasOwnProperty.call(req.body, 'positionCode')
    ? req.body.positionCode
    : systemState.telemetry.imuCalibration.stepCode;
  const positionCode = Number.parseInt(String(requestedStepCode ?? ''), 10);
  const positionName = IMU_CALIBRATION_POSITIONS[positionCode];

  if (!positionName) {
    res.status(400).json({ success: false, message: 'There is no pose ready to confirm yet. Wait for the FCU prompt.' });
    return;
  }

  sendMavlinkCommand('IMU_CALIBRATION_CONFIRM', { positionCode });

  systemState.telemetry.imuCalibration = {
    ...systemState.telemetry.imuCalibration,
    active: true,
    mode: 'ACCEL',
    status: 'CONFIRMING_POSITION',
    step: positionName,
    stepCode: positionCode,
    instructions: `${positionName} confirmed. Waiting for the next FCU step.`,
    updatedAt: Date.now()
  };

  emitTelemetryUpdate();
  addLog('COMMAND', `Confirmed IMU calibration pose: ${positionName}`);

  res.json({
    success: true,
    message: `${positionName} pose confirmation sent`,
    telemetry: systemState.telemetry
  });
});

app.post('/api/system/reboot', (req, res) => {
  addLog('WARNING', 'System reboot requested from dashboard (not implemented)');
  res.status(501).json({
    success: false,
    message: 'Reboot is not implemented in this build. Please reboot host manually via SSH.'
  });
});

app.post('/api/emergency/stop', (req, res) => {
  addLog('CRITICAL', 'Emergency stop triggered');

  for (let channel = 1; channel <= 8; channel += 1) {
    systemState.motorStatus[`ch${channel}`] = PWM_CENTER;
  }

  systemState.roverControl = {
    throttle: 0,
    steering: 0,
    leftPwm: PWM_CENTER,
    rightPwm: PWM_CENTER
  };

  sendMavlinkCommand('EMERGENCY_STOP', {
    pwm: PWM_CENTER,
    channels: [...enabledChannels],
    throttleChannel: ROVER_THROTTLE_INPUT_CHANNEL,
    steeringChannel: ROVER_STEERING_INPUT_CHANNEL,
    throttlePwm: PWM_CENTER,
    steeringPwm: PWM_CENTER
  });
  sendMavlinkCommand('DISARM');
  systemState.telemetry.armed = false;

  io.emit('aircraft_disarmed');
  io.emit('motor_update', {
    channel: 0,
    pwm: PWM_CENTER,
    timestamp: new Date().toISOString()
  });

  res.json({ success: true, message: 'Emergency stop activated' });
});

app.get('/health', (req, res) => {
  res.json({
    status: 'ok',
    timestamp: Date.now(),
    bridge: {
      host: BRIDGE_HOST,
      commandPort: BRIDGE_COMMAND_PORT,
      telemetryPort: BRIDGE_TELEMETRY_PORT
    },
    runtime: {
      nodeVersion: process.version
    }
  });
});

app.use('/api', (req, res) => {
  res.status(404).json({ success: false, message: `API route not found: ${req.method} ${req.path}` });
});

app.use((error, _req, res, _next) => {
  const badJson = error && error.type === 'entity.parse.failed';
  const status = badJson ? 400 : 500;
  if (!badJson) console.error(`[HTTP] ${error && error.stack ? error.stack : error}`);
  res.status(status).json({
    success: false,
    message: badJson ? 'Invalid JSON request body' : 'Internal server error'
  });
});

io.on('connection', (socket) => {
  addLog('INFO', `Web client connected: ${socket.id}`);
  socket.emit('system_state', systemState);

  socket.on('motor_control', (data = {}) => {
    const result = handleMotorControl(data.channel, data.pwm, `WEB:${socket.id}`);
    if (!result.ok) {
      socket.emit('error_message', { message: result.error });
    }
  });

  socket.on('rover_drive', (data = {}) => {
    const result = applyRoverControl({ throttle: data.throttle, steering: data.steering }, `SOCKET:${socket.id}`);
    if (!result.ok) {
      socket.emit('error_message', { message: result.error });
      return;
    }
    socket.emit('rover_drive_ack', result);
  });

  socket.on('arm', () => {
    if (systemState.telemetry.armed) {
      socket.emit('info_message', { message: 'Rover already armed' });
      return;
    }

    sendMavlinkCommand('ARM');
    systemState.telemetry.armed = true;
    addLog('COMMAND', 'ARM command sent');
    io.emit('aircraft_armed');
  });

  socket.on('disarm', () => {
    if (!systemState.telemetry.armed) {
      socket.emit('info_message', { message: 'Rover already disarmed' });
      return;
    }

    sendMavlinkCommand('DISARM');
    systemState.telemetry.armed = false;
    addLog('COMMAND', 'DISARM command sent');
    io.emit('aircraft_disarmed');
  });

  socket.on('request_telemetry', () => {
    socket.emit('telemetry_update', systemState.telemetry);
  });

  socket.on('disconnect', () => {
    addLog('INFO', `Web client disconnected: ${socket.id}`);
  });

  socket.on('error', (error) => {
    addLog('ERROR', `Socket error (${socket.id}): ${error}`);
  });
});

httpServer.listen(WEB_PORT, WEB_HOST, () => {
  console.log(`
╔═══════════════════════════════════════════════════════╗
║       RK3588 + Pixhawk Rover Control Started         ║
╚═══════════════════════════════════════════════════════╝

Web Server: http://${WEB_HOST}:${WEB_PORT}
Socket.io: ws://${WEB_HOST}:${WEB_PORT}
Dashboard: http://localhost:${WEB_PORT}
Bridge CMD: udp://${BRIDGE_HOST}:${BRIDGE_COMMAND_PORT}
Bridge TEL: udp://127.0.0.1:${BRIDGE_TELEMETRY_PORT}

Press Ctrl+C to stop
  `);

  addLog('INFO', 'Server started successfully');
  cleanupStaleGimbalTrackWorkers('server-start');

  try {
    bonjour = new Bonjour();
    mantaBonjourService = bonjour.publish({
      name: MANTA_SERVICE_NAME,
      type: 'manta',
      protocol: 'tcp',
      port: WEB_PORT,
      txt: {
        role: 'control',
        health: '/health',
        recordings: '/api/gimbal/recordings'
      }
    });
    addLog('INFO', `Bonjour service announced: ${MANTA_SERVICE_NAME}._manta._tcp:${WEB_PORT}`);
  } catch (error) {
    addLog('WARN', `Bonjour service unavailable: ${error.message}`);
  }
});

let bonjour = null;
let mantaBonjourService = null;

let shutdownStarted = false;

function shutdown() {
  if (shutdownStarted) return;
  shutdownStarted = true;
  const forceExitTimer = setTimeout(() => {
    console.error('[Shutdown] Force exit after timeout');
    process.exit(0);
  }, 2500);
  forceExitTimer.unref();

  flushTelemetryCsv(true);
  addLog('INFO', 'Server shutting down');

  if (isVisionActive()) {
    try { visionProcess.kill('SIGTERM'); } catch (_) {}
  }

  stopGimbalTracking(false);
  if (gimbalTrackProcess && gimbalTrackProcess.exitCode === null && !gimbalTrackProcess.killed) {
    try { gimbalTrackProcess.kill('SIGKILL'); } catch (_) {}
  }

  if (gimbalCommandTimer) {
    clearInterval(gimbalCommandTimer);
    gimbalCommandTimer = null;
  }

  if (gimbalStream) {
    try { gimbalStream.destroy(); } catch (_) {}
    gimbalStream = null;
  }

  if (gimbalReadStream) {
    try { gimbalReadStream.destroy(); } catch (_) {}
    gimbalReadStream = null;
  }

  try {
    telemetrySocket.close();
  } catch (error) {
    console.error(`[Shutdown] Telemetry socket close error: ${error.message}`);
  }

  try {
    commandSocket.close();
  } catch (error) {
    console.error(`[Shutdown] Command socket close error: ${error.message}`);
  }

  if (mantaBonjourService) {
    try { mantaBonjourService.stop(); } catch (_) {}
    mantaBonjourService = null;
  }
  if (bonjour) {
    try { bonjour.destroy(); } catch (_) {}
    bonjour = null;
  }

  httpServer.close(() => {
    console.log('[Server] Closed');
    clearTimeout(forceExitTimer);
    process.exit(0);
  });
}

process.on('SIGINT', shutdown);
process.on('SIGTERM', shutdown);

export default app;
