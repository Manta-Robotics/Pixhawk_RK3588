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

  const pwmMin = Number(config.min_motor_pwm);
  const pwmCenter = Number(config.default_motor_pwm);
  const pwmMax = Number(config.max_motor_pwm);
  if (![pwmMin, pwmCenter, pwmMax].every(Number.isFinite) || !(pwmMin < pwmCenter && pwmCenter < pwmMax)) {
    errors.push('motor PWM values must satisfy min_motor_pwm < default_motor_pwm < max_motor_pwm');
  }
  if (![1, -1].includes(Number(config.rover_throttle_sign))) {
    errors.push('rover_throttle_sign must be 1 or -1');
  }
  if (Number(config.rover_left_channel) === Number(config.rover_right_channel)) {
    errors.push('rover left and right output channels must be different');
  }
  if (Number(config.rover_left_input_channel) !== 1 || Number(config.rover_right_input_channel) !== 3) {
    errors.push('tank steering requires rover_left_input_channel=1 and rover_right_input_channel=3');
  }

  const hotspot = isObject(config.hotspot) ? config.hotspot : {};
  if (hotspot.enabled !== false) {
    if (!String(hotspot.ssid || '').trim()) errors.push('hotspot.ssid is required');
    if (String(hotspot.password || '').length < 8) errors.push('hotspot.password must contain at least 8 characters');
    if (!validPort(hotspot.portal_port || 80)) errors.push('hotspot.portal_port must be a valid port');
  }

  const gimbal = isObject(config.gimbal) ? config.gimbal : {};
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

export function validateMotorConfig(config, systemConfig = {}) {
  const errors = [];
  if (!isObject(config)) return ['root must be a JSON object'];
  if (!Array.isArray(config.motors) || config.motors.length === 0) return ['motors must be a non-empty array'];

  const seenChannels = new Set();
  for (const motor of config.motors) {
    if (!isObject(motor)) {
      errors.push('each motor must be an object');
      continue;
    }
    const channel = Number(motor.channel);
    if (!Number.isInteger(channel) || channel < 1 || channel > 32) {
      errors.push(`invalid motor channel: ${motor.channel}`);
      continue;
    }
    if (seenChannels.has(channel)) errors.push(`duplicate motor channel: ${channel}`);
    seenChannels.add(channel);
    if (typeof motor.servo_reversed !== 'boolean') errors.push(`motor channel ${channel} servo_reversed must be boolean`);

    const min = Number(motor.min_pwm);
    const center = Number(motor.center_pwm);
    const max = Number(motor.max_pwm);
    if (![min, center, max].every(Number.isFinite) || !(min < center && center < max)) {
      errors.push(`motor channel ${channel} PWM values must satisfy min < center < max`);
    }
  }

  for (const key of ['rover_left_channel', 'rover_right_channel']) {
    const channel = Number(systemConfig[key]);
    const motor = config.motors.find((item) => Number(item && item.channel) === channel);
    if (!motor || motor.enabled === false) errors.push(`${key}=${channel} must reference an enabled motor`);
  }

  const brushless = isObject(config.brushless_config) ? config.brushless_config : {};
  if (brushless.bidirectional === true && Number(brushless.idle_pwm) !== Number(brushless.center_pwm)) {
    errors.push('bidirectional brushless_config idle_pwm must equal center_pwm');
  }
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
