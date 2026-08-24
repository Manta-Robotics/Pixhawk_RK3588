#!/usr/bin/env node

import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const SCRIPT_PATH = fileURLToPath(import.meta.url);
const PROJECT_ROOT = path.resolve(path.dirname(SCRIPT_PATH), '..');
const DEFAULT_CONFIG_PATH = path.join(PROJECT_ROOT, 'config', 'system.config.json');

function isObject(value) {
  return Boolean(value) && typeof value === 'object' && !Array.isArray(value);
}

function validPort(value) {
  return Number.isInteger(Number(value)) && Number(value) >= 1 && Number(value) <= 65535;
}

function validHttpUrl(value) {
  try {
    const url = new URL(String(value));
    return url.protocol === 'http:' || url.protocol === 'https:';
  } catch (_) {
    return false;
  }
}

export function validateSystemConfig(config) {
  const errors = [];
  if (!isObject(config)) return ['root must be a JSON object'];

  for (const key of ['web_port', 'bridge_command_port', 'bridge_telemetry_port']) {
    if (!validPort(config[key])) errors.push(`${key} must be an integer from 1 to 65535`);
  }

  if (!String(config.serial_port || '').startsWith('/dev/')) {
    errors.push('serial_port must be an absolute /dev path');
  }
  if (!Number.isFinite(Number(config.baud_rate)) || Number(config.baud_rate) <= 0) {
    errors.push('baud_rate must be a positive number');
  }

  const map = isObject(config.map) ? config.map : {};
  if (!['amap', 'local'].includes(String(map.provider || '').toLowerCase())) {
    errors.push('map.provider must be amap or local');
  }
  if (String(map.coordinate_system || '').toLowerCase() !== 'wgs84') {
    errors.push('map.coordinate_system must be wgs84 for Pixhawk GPS data');
  }

  const hotspot = isObject(config.hotspot) ? config.hotspot : {};
  if (hotspot.enabled !== false) {
    if (!String(hotspot.ssid || '').trim()) errors.push('hotspot.ssid is required');
    if (String(hotspot.password || '').length < 8) errors.push('hotspot.password must contain at least 8 characters');
    if (!validPort(hotspot.portal_port || 80)) errors.push('hotspot.portal_port must be a valid port');
  }

  const gimbal = isObject(config.gimbal) ? config.gimbal : {};
  const controlTransport = String(gimbal.control_transport || '').toLowerCase();
  if (!['uart', 'udp'].includes(controlTransport)) errors.push('gimbal.control_transport must be uart or udp');
  if (controlTransport === 'uart') {
    if (!String(gimbal.boot_config || '').startsWith('/')) errors.push('gimbal.boot_config must be an absolute path');
    if (!String(gimbal.uart_overlay || '').trim()) errors.push('gimbal.uart_overlay is required for UART control');
  }
  const video = isObject(gimbal.video) ? gimbal.video : {};
  for (const key of ['recognition_input', 'record_input']) {
    if (!String(video[key] || '').startsWith('rtsp://')) errors.push(`gimbal.video.${key} must be an RTSP URL`);
  }
  if (!validHttpUrl(video.local_stream_url || '')) errors.push('gimbal.video.local_stream_url must be an HTTP URL');
  if (!validPort(video.proxy_port || 8091)) errors.push('gimbal.video.proxy_port must be a valid port');
  if (Number(video.mobile_width) < 160 || Number(video.mobile_height) < 90) {
    errors.push('gimbal.video mobile preview dimensions are too small');
  }
  if (!['copy', 'h264_rkmpp', 'h264_v4l2m2m'].includes(String(video.record_codec || ''))) {
    errors.push('gimbal.video.record_codec must be copy, h264_rkmpp, or h264_v4l2m2m');
  }

  const face = isObject(gimbal.face) ? gimbal.face : {};
  if (face.enabled !== false && !String(face.source || '').includes('/stream.mjpg')) {
    errors.push('gimbal.face.source must use the full-resolution internal stream');
  }
  if (Number(face.track_hold_enter_x_px) >= Number(face.track_hold_exit_x_px)) {
    errors.push('gimbal.face horizontal hold enter threshold must be smaller than exit threshold');
  }
  if (Number(face.track_hold_enter_y_px) >= Number(face.track_hold_exit_y_px)) {
    errors.push('gimbal.face vertical hold enter threshold must be smaller than exit threshold');
  }

  const recordings = isObject(config.recordings) ? config.recordings : {};
  if (!String(recordings.dir || '').trim()) errors.push('recordings.dir is required');
  return errors;
}

export function loadAndValidateConfig(configPath = DEFAULT_CONFIG_PATH) {
  const resolved = path.resolve(configPath);
  const config = JSON.parse(fs.readFileSync(resolved, 'utf8'));
  const errors = validateSystemConfig(config);
  return { config, errors, path: resolved };
}

if (process.argv[1] && path.resolve(process.argv[1]) === SCRIPT_PATH) {
  try {
    const result = loadAndValidateConfig(process.argv[2] || DEFAULT_CONFIG_PATH);
    if (result.errors.length) {
      console.error(`[config] ${result.path}`);
      for (const error of result.errors) console.error(`  - ${error}`);
      process.exitCode = 1;
    } else {
      console.log(`[config] OK ${result.path}`);
    }
  } catch (error) {
    console.error(`[config] ${error.message}`);
    process.exitCode = 1;
  }
}
