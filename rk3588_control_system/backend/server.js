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