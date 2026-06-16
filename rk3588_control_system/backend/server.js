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
import cors from 'cors';
import bodyParser from 'body-parser';
import dgram from 'dgram';
import { spawn } from 'child_process';
import net from 'net';
import os from 'os';
import http from 'http';
import https from 'https';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const PROJECT_ROOT = path.resolve(__dirname, '..');

function readJsonFile(relativePath, fallback) {
  const filePath = path.join(PROJECT_ROOT, relativePath);
  try {
    return JSON.parse(fs.readFileSync(filePath, 'utf8'));
  } catch (error) {
    console.error(`[Config] Failed to read ${relativePath}: ${error.message}`);
    return fallback;
  }
}

const config = readJsonFile('config/system.config.json', {});
const motorConfig = readJsonFile('config/motor_config.json', { motors: [] });

const LOGS_DIR = path.resolve(PROJECT_ROOT, config.logs_dir || './logs');
const SYSTEM_LOG_FILE = path.join(LOGS_DIR, 'system.log');
const FLIGHT_CSV_FILE = path.join(LOGS_DIR, 'flight_data.csv');
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
const httpServer = createServer(app);
const io = new Server(httpServer, {
  cors: {
    origin: '*',
    methods: ['GET', 'POST']
  },
  transports: ['polling', 'websocket'],
  perMessageDeflate: false
});

const commandSocket = dgram.createSocket('udp4');
const telemetrySocket = dgram.createSocket('udp4');
const FFMPEG_BIN = process.env.FFMPEG_BIN || 'ffmpeg';

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
const GIMBAL_DEADZONE_PX = Math.max(0, Number(GIMBAL_AXIS.deadzone_px || 12));
const GIMBAL_RATE_SLEW_DPS = Math.max(1, Number(GIMBAL_AXIS.rate_slew_dps || 8));
const GIMBAL_CLICK_TARGET_HOLD_MS = Math.max(40, Number(GIMBAL_AXIS.click_target_hold_ms || 120));
const GIMBAL_CLICK_CONTROL_MODE = String(GIMBAL_AXIS.click_control_mode || 'rate').trim().toLowerCase();
const GIMBAL_CLICK_HOLD_MIN_MS = Math.max(60, Number(GIMBAL_AXIS.click_hold_min_ms || 120));
const GIMBAL_CLICK_HOLD_MAX_MS = Math.max(GIMBAL_CLICK_HOLD_MIN_MS, Number(GIMBAL_AXIS.click_hold_max_ms || 520));
const gimbalVideoConfig = gimbalConfig.video || {};
const GIMBAL_STREAM_PROXY_PORT = Number(gimbalVideoConfig.proxy_port || 8091);
const GIMBAL_LOCAL_STREAM_URL = String(gimbalVideoConfig.local_stream_url || `http://127.0.0.1:${GIMBAL_STREAM_PROXY_PORT}/stream.mjpg`);
const GIMBAL_VIDEO_TRANSPORT = String(gimbalVideoConfig.transport || '').trim().toLowerCase();
const GIMBAL_RTSP_INPUT = String(gimbalVideoConfig.rtsp_input || gimbalVideoConfig.input_url || '').trim();
const GIMBAL_UDP_INPUT = String(gimbalVideoConfig.udp_input || 'udp://0.0.0.0:9554');
const GIMBAL_VIDEO_INPUT = GIMBAL_RTSP_INPUT && (GIMBAL_VIDEO_TRANSPORT === 'rtsp' || GIMBAL_RTSP_INPUT.startsWith('rtsp://')) ? GIMBAL_RTSP_INPUT : GIMBAL_UDP_INPUT;
const gimbalFocusConfig = gimbalConfig.focus || {};
let gimbalStream = null;
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
const gimbalState = {
  enabled: Boolean(gimbalConfig.enabled),
  connected: false,
  transport: String(gimbalConfig.control_transport || 'uart'),
  serialPort: GIMBAL_SERIAL_PORT,
  baudRate: GIMBAL_BAUD_RATE,
  mode: 'idle',
  lastCommand: 'idle',
  lastError: '',
  lastTarget: null,
  trackingActive: false,
  trackWorkerActive: false,
  trackStatus: { locked: false, status: 'idle', message: 'idle', detections: 0, updatedAt: null },
  videoSource: String(gimbalVideoConfig.source_url || '/api/gimbal/stream'),
  videoTransport: GIMBAL_VIDEO_INPUT.startsWith('rtsp://') ? 'rtsp' : 'udp',
  videoInput: GIMBAL_VIDEO_INPUT,
  udpVideo: GIMBAL_UDP_INPUT,
  updatedAt: Date.now()
};
const GIMBAL_AUTO_CONNECT = gimbalConfig.auto_connect === true;
const GIMBAL_AUTO_HOME_ON_CONNECT = gimbalConfig.auto_home_on_connect === true;

function updateGimbalDiagnostics() {
  if (!fs.existsSync(GIMBAL_SERIAL_PORT)) {
    gimbalState.lastError = `${GIMBAL_SERIAL_PORT} not present; reboot after enabling UART3`;
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
      cameraUrl: `http://${ip}:${SNAPSHOT_PORT}/stream.mjpg`
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

function readCameraState(connectivity, req = null) {
  const directStreamUrl = buildDirectCameraUrl(req, '/stream.mjpg');
  const directOpenUrl = buildDirectCameraUrl(req, '/stream.mjpg');
  const sourceUrl = String(directStreamUrl || cameraConfig.source_url || `http://${MANTA_HOST}:8080/stream`);
  const openUrl = String(directOpenUrl || cameraConfig.open_url || `http://${MANTA_HOST}:8080`);
  const localVideoDevices = readLocalVideoDevices();
  const localCameraDevices = localVideoDevices.filter((entry) => entry.name && !entry.name.toLowerCase().includes('hdmirx'));
  const overlay = String(cameraConfig.overlay || '');
  const sensor = String(cameraConfig.sensor || 'camera');
  const port = String(cameraConfig.port || '');
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
    online: Boolean((connectivity && connectivity.wireless && connectivity.wireless.online) || (connectivity && connectivity.ethernet && connectivity.ethernet.online)),
    host: MANTA_HOST,
    hostState,
    localVideoDevices,
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
    position: { lat: 0, lon: 0, alt: 0 },
    attitude: { roll: 0, pitch: 0, yaw: 0 },
    velocity: { vx: 0, vy: 0, vz: 0 },
    battery: { voltage: 0, current: 0, percentage: 100 },
    servoOutputs: { ch1: 0, ch2: 0, ch3: 0, ch4: 0 },
    temperature: { hostBoard: null, flightController: null, motorLeft: null, motorRight: null },
    gps: { satellites: 0, hdop: 999 },
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
  connectivity: readConnectivityState(),
  accessUrls: [],
  camera: readCameraState(readConnectivityState()),
  vision: { active: false, detections: { w: 0, h: 0, rects: [], t: 0 } },
  logs: []
};

refreshPeripheralState();

app.use(cors());
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({ extended: true }));
app.use(express.static(path.join(PROJECT_ROOT, 'frontend')));

const telemetryCsvBuffer = [];
let telemetryCsvFlushTimer = null;

function asFiniteNumber(value, fallback = 0) {
  const numeric = Number(value);
  return Number.isFinite(numeric) ? numeric : fallback;
}

function mergeOptionalFiniteNumber(value, fallback = null) {
  if (value === null || typeof value === 'undefined' || value === '') {
    return fallback;
  }

  const numeric = Number(value);
  return Number.isFinite(numeric) ? numeric : fallback;
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
  const timestamp = new Date().toISOString();
  const entry = { timestamp, level, message };

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
  console.log(`[${level}] ${message}`);
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
  gimbalState.updatedAt = Date.now();
  io.emit('gimbal_state', { ...gimbalState });
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
      gimbalState.connected = true;
      gimbalState.lastError = '';
      addLog('GIMBAL', `Serial opened ${GIMBAL_SERIAL_PORT} @ ${GIMBAL_BAUD_RATE}`);
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
      gimbalState.lastError = error.message;
      addLog('GIMBAL_ERR', error.message);
      emitGimbalState();
      gimbalStream = null;
    });
    gimbalStream.on('close', () => {
      gimbalState.connected = false;
      emitGimbalState();
      gimbalStream = null;
    });
    return true;
  } catch (error) {
    gimbalState.connected = false;
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
  gimbalStopFramesRemaining = 0;
  if (gimbalCommandTimer) {
    clearInterval(gimbalCommandTimer);
    gimbalCommandTimer = null;
  }
  if (gimbalStream) {
    try { gimbalStream.end(); } catch (_) {}
    gimbalStream = null;
  }
  gimbalState.connected = false;
  gimbalState.mode = 'idle';
  gimbalState.lastCommand = 'disconnected';
  emitGimbalState();
}

function startGimbalLoop() {
  if (gimbalCommandTimer) return;
  gimbalLastFrame = buildGimbalFrame();
  gimbalCommandTimer = setInterval(() => {
    if (gimbalHoldUntil && Date.now() > gimbalHoldUntil) {
      gimbalHoldUntil = 0;
      gimbalLastFrame = buildGimbalFrame({ joystickCommand: 0x00, joystickX: 0, joystickY: 0 });
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
  if (resetState) {
    gimbalTxEnabled = false;
    gimbalState.mode = 'idle';
    gimbalState.lastTarget = null;
    gimbalLastFrame = buildGimbalFrame();
    gimbalLastRateX = 0;
    gimbalLastRateY = 0;
    gimbalStopFramesRemaining = 0;
    emitGimbalState();
  }
  return { ok: true };
}

function updateGimbalTrackStatus(update = {}) {
  const next = {
    locked: false,
    status: 'lost',
    message: 'can not find swimmer',
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
  gimbalLastRateX = 0;
  gimbalLastRateY = 0;
  const frame = buildGimbalFrame({ joystickCommand: 0x70, joystickX: 0, joystickY: 0 });
  setGimbalFrame(frame, source, 'track', 0, { locked: false, status: 'lost', message: 'can not find swimmer' });
  return frame;
}

function sendGimbalCancelTrack(source = 'web') {
  const frame = buildGimbalFrame({ command: 0x3b, param1: 0, param2: 0 });
  setGimbalFrame(frame, `cancel-track:${source}`, 'idle', 180, { command: 0x3b });
  writeGimbalFrameBurst(frame, 5);
  addLog('GIMBAL', `Cancel track command sent (${source})`);
  return { command: 0x3b, holdMs: 180 };
}

function sendGimbalHome(source = 'web') {
  stopGimbalTracking(false);
  sendGimbalDetectorPaused(true, `${source}:prehome`);
  sendGimbalCancelTrack(`${source}:prehome`);
  gimbalHoldUntil = 0;
  gimbalStopFramesRemaining = 0;
  gimbalLastRateX = 0;
  gimbalLastRateY = 0;
  const disableFrame = buildGimbalFrame({ joystickCommand: 0x00, joystickX: 0, joystickY: 0 });
  writeGimbalFrameBurst(disableFrame, 5);
  const frame = buildGimbalFrame({ command: 0x71 });
  setGimbalFrame(frame, `home:${source}`, 'home', 900, null);
  writeGimbalFrameBurst(frame, 3);
  gimbalState.connected = Boolean(gimbalStream);
  addLog('GIMBAL', `Home command sent (${source})`);
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
  gimbalStopFramesRemaining = 5;
  gimbalState.mode = 'idle';
  gimbalState.lastCommand = `stop:${source}`;
  gimbalState.lastTarget = null;
  if (gimbalStream) {
    const stopFrame = buildGimbalFrame({ joystickCommand: 0x00, joystickX: 0, joystickY: 0 });
    writeGimbalFrameBurst(stopFrame, 5);
  }
  gimbalState.connected = Boolean(gimbalStream);
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

  const yawDeg = (delta.x / GIMBAL_MAX_PIXEL_X) * (GIMBAL_CLICK_YAW_FOV_DEG * 0.5);
  const pitchDeg = (delta.y / GIMBAL_MAX_PIXEL_Y) * (GIMBAL_CLICK_PITCH_FOV_DEG * 0.5);
  const angleYaw = clamp(Math.round(yawDeg * 100), -15000, 15000);
  const anglePitch = clamp(Math.round(pitchDeg * 100), -15000, 15000);
  const maxAngle = Math.max(Math.abs(yawDeg), Math.abs(pitchDeg));
  const clickHoldMs = holdMs === null
    ? clamp(Math.round(maxAngle * 55 + 480), 480, 2200)
    : Math.max(120, Number(holdMs || 900));
  const frame = buildGimbalFrame({ command: 0x72, param1: angleYaw, param2: anglePitch });
  setGimbalFrame(frame, 'guided-angle', 'click', clickHoldMs, { ...delta, controlMode: 'guided_angle', command: 0x72, yawDeg, pitchDeg, angleYaw, anglePitch, holdMs: clickHoldMs });
  writeGimbalFrameBurst(frame, 5);
  addLog('GIMBAL', `Guided angle 72H dx=${delta.x} dy=${delta.y} yawDeg=${yawDeg.toFixed(2)} pitchDeg=${pitchDeg.toFixed(2)} holdMs=${clickHoldMs}`);
  return { ...delta, controlMode: 'guided_angle', command: 0x72, yawDeg, pitchDeg, angleYaw, anglePitch, holdMs: clickHoldMs };
}

function planGimbalClickMove(dx, dy) {
  const yawDeg = (dx / GIMBAL_MAX_PIXEL_X) * (GIMBAL_CLICK_YAW_FOV_DEG * 0.5);
  const pitchDeg = (dy / GIMBAL_MAX_PIXEL_Y) * (GIMBAL_CLICK_PITCH_FOV_DEG * 0.5);
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

function sendGimbalJsonCommand(payloadText, source = 'web') {
  const { frame, payloadLength } = buildGimbalJsonFrame(payloadText);
  setGimbalFrame(frame, `json:${source}`, 'json', 240, { command: 0x90, payload: payloadText, payloadLength });
  writeGimbalFrameBurst(frame, 3);
  addLog('GIMBAL', `JSON command sent payload=${payloadText}`);
  return { command: 0x90, payload: payloadText, payloadLength, holdMs: 240 };
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
  const rawX = Math.abs(dx) < GIMBAL_DEADZONE_PX ? 0 : dx * gain;
  const rawY = Math.abs(dy) < GIMBAL_DEADZONE_PX ? 0 : dy * gain;
  const rate = {
    x: clamp(Math.round(rawX), -GIMBAL_MAX_RATE_DPS, GIMBAL_MAX_RATE_DPS),
    y: clamp(Math.round(rawY), -GIMBAL_MAX_RATE_DPS, GIMBAL_MAX_RATE_DPS)
  };
  return useSlew ? slewGimbalRate(rate.x, rate.y) : rate;
}

function sendGimbalTrackDelta(dx, dy, source = 'track') {
  const delta = normalizeGimbalDelta(dx, dy);
  const rate = gimbalRateFromDelta(delta.x, delta.y, GIMBAL_TRACK_RATE_GAIN);
  const frame = buildGimbalFrame({ joystickCommand: 0x70, joystickX: rate.x, joystickY: rate.y });
  setGimbalFrame(frame, source, 'track', 0, { ...delta, rateX: rate.x, rateY: rate.y });
  return { ...delta, rateX: rate.x, rateY: rate.y };
}

function isGimbalTrackingActive() {
  return Boolean(gimbalTrackProcess && gimbalTrackProcess.exitCode === null && !gimbalTrackProcess.killed);
}

function startGimbalTracking() {
  if (isGimbalTrackingActive()) return { ok: true, alreadyRunning: true };
  const script = path.join(PROJECT_ROOT, 'scripts', 'infer_video.py');
  if (!fs.existsSync(script)) return { ok: false, error: 'infer_video swimmer tracker script not found' };
  if (gimbalTrackRestartTimer) {
    clearTimeout(gimbalTrackRestartTimer);
    gimbalTrackRestartTimer = null;
  }
  gimbalTrackStopRequested = false;
  const swimmer = gimbalConfig.swimmer || {};
  const args = ['-u', script,
    '--source', String(swimmer.source || 'http://127.0.0.1:8090/stream.mjpg'),
    '--weights', String(swimmer.weights || 'scripts/best.pt'),
    '--tracker', String(swimmer.tracker || 'scripts/bytetrack_swimmer.yaml'),
    '--conf', String(swimmer.conf ?? 0.1),
    '--iou', String(swimmer.iou ?? 0.5),
    '--imgsz', String(swimmer.imgsz ?? 640),
    '--device', String(swimmer.device ?? '0'),
    '--loop-hz', String(swimmer.loop_hz ?? 10),
    '--q', String(swimmer.q ?? 1.0),
    '--r', String(swimmer.r ?? 50.0),
    '--max-coast', String(swimmer.max_coast ?? 45),
    '--reid-sim', String(swimmer.reid_sim ?? 0.5),
    '--gate-dist', String(swimmer.gate_dist ?? 140.0),
    '--gate-scale', String(swimmer.gate_scale ?? 2.2),
    '--smooth-alpha', String(swimmer.smooth_alpha ?? 0.3),
    '--max-center-speed', String(swimmer.max_center_speed ?? 800.0),
    '--max-size-rate', String(swimmer.max_size_rate ?? 1.0),
    '--hold-x-px', String(swimmer.hold_x_px ?? 500.0),
    '--hold-y-px', String(swimmer.hold_y_px ?? 500.0),
    '--hold-release', String(swimmer.hold_release ?? 300.0),
    '--conf-lock', String(swimmer.conf_lock ?? 0.35),
    '--size-tol', String(swimmer.size_tol ?? 0.35),
    '--vft-alpha', String(swimmer.vft_alpha ?? 0.35),
    '--deadzone-beta', String(swimmer.deadzone_beta ?? 0.15),
    '--center-median-window', String(swimmer.center_median_window ?? 11)
  ];
  try {
    const child = spawn(PYTHON_EXEC, args, { cwd: PROJECT_ROOT, stdio: ['ignore', 'pipe', 'pipe'] });
    gimbalTrackProcess = child;
    gimbalState.trackingActive = true;
    gimbalState.trackWorkerActive = true;
    gimbalState.mode = 'track';
    updateGimbalTrackStatus({ status: 'starting', message: 'can not find swimmer', workerActive: true });
    let stdoutBuf = '';
    child.stdout.on('data', (data) => {
      stdoutBuf += String(data);
      let index;
      while ((index = stdoutBuf.indexOf('\n')) !== -1) {
        const line = stdoutBuf.slice(0, index).trim();
        stdoutBuf = stdoutBuf.slice(index + 1);
        if (!line) continue;
        if (line.startsWith('TARGET:')) {
          try {
            const target = JSON.parse(line.slice(7));
            const sent = sendGimbalTrackDelta(target.dx, target.dy, 'swimmer-track');
            const message = { ...target, commandDx: sent.x, commandDy: sent.y, rateX: sent.rateX, rateY: sent.rateY, locked: true, message: 'SWIMMER LOCKED', workerActive: true, timestamp: Date.now() };
            gimbalState.lastTarget = message;
            updateGimbalTrackStatus({ ...message, status: target.status || 'track', detections: Number(target.detections || 0) });
            io.emit('gimbal_target', message);
          } catch (error) {
            addLog('GIMBAL_ERR', `Invalid target output: ${error.message}`);
          }
        } else if (line.startsWith('STATUS:')) {
          try {
            const status = JSON.parse(line.slice(7));
            holdGimbalTrackIdle('swimmer-not-found');
            updateGimbalTrackStatus({ ...status, locked: false, message: status.message || 'can not find swimmer', workerActive: true });
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
        holdGimbalTrackIdle('swimmer-worker-exit');
        updateGimbalTrackStatus({ status: 'worker_exit', message: 'can not find swimmer', code, signal: signalName || '', workerActive: false });
        gimbalTrackRestartTimer = setTimeout(() => {
          gimbalTrackRestartTimer = null;
          if (!gimbalTrackStopRequested && gimbalState.trackingActive) {
            startGimbalTracking();
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
    addLog('GIMBAL', 'Swimmer tracking started');
    return { ok: true };
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
  systemState.motorStatus[`ch${validChannel}`] = validPwm;
  sendMavlinkCommand('MOTOR_CONTROL', { channel: validChannel, pwm: validPwm });

  io.emit('motor_update', {
    channel: validChannel,
    pwm: validPwm,
    timestamp: new Date().toISOString()
  });

  addLog('MOTOR', `${sourceLabel} set channel ${validChannel} => ${validPwm}us`);
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
    imuCalibration: { ...(previous.imuCalibration || createDefaultImuCalibrationState()) },
    flightMode: previous.flightMode,
    systemStatus: previous.systemStatus,
    armed: previous.armed
  };

  if (newTelemetry.position) {
    nextTelemetry.position.lat = asFiniteNumber(newTelemetry.position.lat, nextTelemetry.position.lat);
    nextTelemetry.position.lon = asFiniteNumber(newTelemetry.position.lon, nextTelemetry.position.lon);
    nextTelemetry.position.alt = asFiniteNumber(newTelemetry.position.alt, nextTelemetry.position.alt);
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

  systemState.telemetry = nextTelemetry;
  emitTelemetryUpdate();
  appendTelemetryCsv(systemState.telemetry);
}

function updateConnectionStatus(isConnected) {
  systemState.isConnected = isConnected;
  systemState.pixhawkStatus = isConnected ? 'connected' : 'disconnected';

  const payload = {
    isConnected,
    status: systemState.pixhawkStatus,
    timestamp: new Date().toISOString()
  };

  io.emit('connection_status', payload);
  addLog('INFO', `Pixhawk connection: ${payload.status}`);
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
if (GIMBAL_AUTO_CONNECT) {
  startGimbalLoop();
  openGimbalPort();
}

app.get('/api/status', (req, res) => {
  refreshPeripheralState();
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
  const r = startVision();
  if (!r.ok) return res.status(500).json({ success: false, message: r.error });
  res.json({ success: true, active: true, alreadyRunning: Boolean(r.alreadyRunning) });
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

app.post('/api/gimbal/home', (_req, res) => {
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
  sendGimbalHome('web');
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

app.post('/api/gimbal/track/start', (_req, res) => {
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
  const result = startGimbalTracking();
  if (!result.ok) {
    res.status(500).json({ success: false, message: result.error, state: gimbalState });
    return;
  }
  res.json({ success: true, active: true, alreadyRunning: Boolean(result.alreadyRunning), state: gimbalState });
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
    systemState.motorStatus[`ch${channel}`] = PWM_MIN;
  }

  systemState.roverControl = {
    throttle: 0,
    steering: 0,
    leftPwm: PWM_MIN,
    rightPwm: PWM_MIN
  };

  sendMavlinkCommand('EMERGENCY_STOP', {
    pwm: PWM_MIN,
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
    pwm: PWM_MIN,
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
});

function shutdown() {
  flushTelemetryCsv(true);
  addLog('INFO', 'Server shutting down');

  if (isVisionActive()) {
    try { visionProcess.kill('SIGTERM'); } catch (_) {}
  }

  stopGimbalTracking(false);

  if (gimbalCommandTimer) {
    clearInterval(gimbalCommandTimer);
    gimbalCommandTimer = null;
  }

  if (gimbalStream) {
    try { gimbalStream.end(); } catch (_) {}
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

  httpServer.close(() => {
    console.log('[Server] Closed');
    process.exit(0);
  });
}

process.on('SIGINT', shutdown);
process.on('SIGTERM', shutdown);

export default app;
