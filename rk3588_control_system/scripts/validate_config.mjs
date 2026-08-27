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
  const pixhawk = isObject(config.pixhawk) ? config.pixhawk : {};
  if (!String(pixhawk.boot_config || '').startsWith('/')) {
    errors.push('pixhawk.boot_config must be an absolute path');
  }
  if (!String(pixhawk.uart_overlay || '').trim()) {
    errors.push('pixhawk.uart_overlay is required');
  }

  if (String(config.rover_control_protocol || '').toLowerCase() !== 'manual_control') {
    errors.push('rover_control_protocol must be manual_control for ArduRover throttle outputs');
  }
  const leftChannel = Number(config.rover_left_channel);
  const rightChannel = Number(config.rover_right_channel);
  if (!Number.isInteger(leftChannel) || leftChannel < 1 || leftChannel > 32) {
    errors.push('rover_left_channel must be an integer from 1 to 32');
  }
  if (!Number.isInteger(rightChannel) || rightChannel < 1 || rightChannel > 32) {
    errors.push('rover_right_channel must be an integer from 1 to 32');
  }
  if (leftChannel === rightChannel) errors.push('rover output channels must be different');
  for (const [label, minimum, maximum] of [
    ['rover throttle', Number(config.rover_throttle_min), Number(config.rover_throttle_max)],
    ['rover steering', Number(config.rover_steering_min), Number(config.rover_steering_max)],
  ]) {
    if (!Number.isFinite(minimum) || !Number.isFinite(maximum) || minimum >= 0 || maximum <= 0) {
      errors.push(`${label} range must span zero`);
    } else if (Math.abs(minimum) !== Math.abs(maximum)) {
      errors.push(`${label} range must be symmetric so zero maps to MANUAL_CONTROL neutral`);
    }
  }
  for (const key of ['rover_manual_throttle_limit_percent', 'rover_manual_steering_limit_percent']) {
    const value = Number(config[key]);
    if (!Number.isFinite(value) || value < 0 || value > 100) {
      errors.push(`${key} must be between 0 and 100`);
    }
  }
  const roverCommandTimeoutMs = Number(config.rover_command_timeout_ms);
  if (!Number.isFinite(roverCommandTimeoutMs) || roverCommandTimeoutMs < 200 || roverCommandTimeoutMs > 2000) {
    errors.push('rover_command_timeout_ms must be between 200 and 2000');
  }

  const map = isObject(config.map) ? config.map : {};
  if (!['offline_satellite', 'local'].includes(String(map.provider || '').toLowerCase())) {
    errors.push('map.provider must be offline_satellite or local');
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
  if (controlTransport === 'udp') {
    if (!String(gimbal.udp_host || '').trim()) errors.push('gimbal.udp_host is required for UDP control');
    if (!validPort(gimbal.udp_port || 9554)) errors.push('gimbal.udp_port must be a valid port');
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
  if (face.enabled !== false && !/^http:\/\/127\.0\.0\.1:\d+\/(stream|mobile)\.mjpg$/.test(String(face.source || ''))) {
    errors.push('gimbal.face.source must use a loopback internal MJPEG stream');
  }
  if (!['rknn_face', 'yolo_face', 'ultra_face', 'haar_face'].includes(String(face.detector || ''))) {
    errors.push('gimbal.face.detector must be rknn_face, yolo_face, ultra_face, or haar_face');
  }
  if (String(face.detector || '') === 'rknn_face' && !String(face.model || '').endsWith('.rknn')) {
    errors.push('gimbal.face.model must be an .rknn model when using rknn_face');
  }
  if (!String(face.fallback_model || '').endsWith('.pt')) {
    errors.push('gimbal.face.fallback_model must be a .pt model');
  }
  const faceProfiles = isObject(face.profiles) ? face.profiles : {};
  if (!['quality', 'balanced', 'fast'].includes(String(face.active_profile || ''))) {
    errors.push('gimbal.face.active_profile must be quality, balanced, or fast');
  }
  for (const [name, expectedSize] of Object.entries({ quality: 512, balanced: 384, fast: 320 })) {
    if (!isObject(faceProfiles[name]) || Number(faceProfiles[name].imgsz) !== expectedSize) {
      errors.push(`gimbal.face.profiles.${name}.imgsz must be ${expectedSize}`);
    }
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
