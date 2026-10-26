#!/usr/bin/env node

import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';
import { PREDICTION_DEFAULTS } from '../backend/uwb_target_prediction.js';

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

  if (!String(config.serial_port || '').startsWith('/dev/') && !/^COM[1-9]\d*$/i.test(String(config.serial_port || ''))) {
    errors.push('serial_port must be an absolute /dev path or a Windows COM port');
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
  const protocol = String(gimbal.protocol || 'ztd095').toLowerCase();
  if (!['ztd095', 'siyi'].includes(protocol)) errors.push('gimbal.protocol must be ztd095 or siyi');
  if (protocol === 'siyi') {
    if (gimbal.control_transport !== 'udp') errors.push('SIYI integration requires UDP control');
    if (gimbal.axis?.click_control_mode !== 'guided_angle') errors.push('SIYI requires guided_angle click control');
    if (Number(gimbal.axis?.yaw_min_deg) < -135 || Number(gimbal.axis?.yaw_max_deg) > 135
      || Number(gimbal.axis?.pitch_min_deg) < -25 || Number(gimbal.axis?.pitch_max_deg) > 90)
      errors.push('SIYI axis limits exceed the SDK control range');
    if (!Number.isFinite(Number(gimbal.siyi?.rate_units_per_dps)) || Number(gimbal.siyi?.rate_units_per_dps) <= 0
      || !Number.isFinite(Number(gimbal.siyi?.max_rate_command)) || Number(gimbal.siyi?.max_rate_command) <= 0
      || Number(gimbal.siyi?.max_rate_command) > 100) errors.push('SIYI rate gain and command cap must be valid');
  }
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
  const executionMode = config.uwb_follow?.execution_mode;
  if (config.uwb_follow?.auto_resume_after_uwb_timeout !== undefined
    && typeof config.uwb_follow.auto_resume_after_uwb_timeout !== 'boolean') errors.push('uwb_follow.auto_resume_after_uwb_timeout must be boolean');
  if (executionMode !== undefined && !['manual', 'acro', 'relative_imu'].includes(executionMode)) errors.push('uwb_follow.execution_mode must be manual, acro or relative_imu');
  const gpsSpeed = config.uwb_follow?.gps_speed_closed_loop;
  const relativeNmpc = config.uwb_follow?.relative_nmpc;
  if (relativeNmpc !== undefined) {
    if (!isObject(relativeNmpc)) errors.push('uwb_follow.relative_nmpc must be an object');
    else {
      if (typeof relativeNmpc.enabled !== 'boolean') errors.push('relative_nmpc.enabled must be boolean');
      if (relativeNmpc.enabled && executionMode !== 'relative_imu') errors.push('Relative NMPC requires relative_imu execution');
      const bounds = { horizon_s:[.3,3], solve_budget_ms:[1,20], surge_time_constant_s:[.2,3],
        yaw_time_constant_s:[.1,2], surge_map_exponent:[.3,1.5], model_cruise_speed_mps:[.1,2], model_position_error_mps:[.05,1], steering_share:[.1,.8] };
      for (const [key,[low,high]] of Object.entries(bounds)) {
        if (relativeNmpc[key] !== undefined && (!Number.isFinite(relativeNmpc[key]) || relativeNmpc[key]<low || relativeNmpc[key]>high)) errors.push(`relative_nmpc.${key} must be finite and between ${low} and ${high}`);
      }
    }
  }
  if (gpsSpeed !== undefined) {
    if (!isObject(gpsSpeed)) errors.push('uwb_follow.gps_speed_closed_loop must be an object');
    else {
      if (typeof gpsSpeed.enabled !== 'boolean') errors.push('gps_speed_closed_loop.enabled must be boolean');
      if (gpsSpeed.enabled && executionMode !== 'relative_imu') errors.push('GPS speed feedback requires relative_imu execution');
      const bounds = { acquire_ms: [1000, 30000], kp: [0, 100], ki: [0, 100], maximum_observed_speed_mps: [0.1, 30] };
      for (const [key, [low, high]] of Object.entries(bounds)) {
        if (gpsSpeed[key] !== undefined && (!Number.isFinite(gpsSpeed[key]) || gpsSpeed[key] < low || gpsSpeed[key] > high)) errors.push(`gps_speed_closed_loop.${key} must be finite and between ${low} and ${high}`);
      }
    }
  }
  if (executionMode === 'acro' && (Number(config.uwb_follow.max_speed_mps) > 2
      || Number(config.uwb_follow.commissioning_speed_mps) > 2)) errors.push('ACRO follow speed limit must not exceed 2 m/s');
  const measurementFilter = config.uwb_measurement_filter;
  if (measurementFilter !== undefined) {
    if (!isObject(measurementFilter)) errors.push('uwb_measurement_filter must be an object');
    else {
      if (typeof measurementFilter.enabled !== 'boolean') errors.push('uwb_measurement_filter.enabled must be boolean');
      if (measurementFilter.occlusion_enabled !== undefined && typeof measurementFilter.occlusion_enabled !== 'boolean')
        errors.push('uwb_measurement_filter.occlusion_enabled must be boolean');
      if (measurementFilter.adaptive_enabled !== undefined && typeof measurementFilter.adaptive_enabled !== 'boolean')
        errors.push('uwb_measurement_filter.adaptive_enabled must be boolean');
      for (const [key, fallback, min, max] of [['stationary_time_constant_s', .4, .1, 2], ['moving_time_constant_s', .01, .01, .3]]) {
        const value = measurementFilter[key] === undefined ? fallback : measurementFilter[key];
        if (typeof value !== 'number' || !Number.isFinite(value) || value < min || value > max)
          errors.push(`uwb_measurement_filter.${key} must be between ${min} and ${max}`);
      }
      if ((measurementFilter.moving_time_constant_s ?? .01) > (measurementFilter.stationary_time_constant_s ?? .4))
        errors.push('uwb_measurement_filter.moving_time_constant_s must not exceed stationary_time_constant_s');
      for (const [key, min, max] of [
        ['distance_std_m', 0.02, 1], ['angle_std_deg', 0.2, 15], ['acceleration_std_mps2', 0.1, 10],
        ['initial_velocity_std_mps', 0.1, 6], ['nis_threshold', 4, 25], ['absolute_gate_m', 0.1, 2],
        ['max_relative_speed_mps', 0.5, 10], ['rejection_covariance_scale', 1, 2]
      ]) {
        const value = measurementFilter[key];
        if (typeof value !== 'number' || !Number.isFinite(value) || value < min || value > max)
          errors.push(`uwb_measurement_filter.${key} must be between ${min} and ${max}`);
      }
    }
  }
  const prediction = config.uwb_follow?.prediction;
  if (prediction !== undefined) {
    if (!isObject(prediction)) errors.push('uwb_follow.prediction must be an object');
    else {
      const p = { ...PREDICTION_DEFAULTS, ...prediction };
      if (typeof p.enabled !== 'boolean') errors.push('uwb_follow.prediction.enabled must be boolean');
      for (const [key, min, max] of [
        ['horizon_ms', 100, 2000], ['wait_timeout_ms', 500, 10000], ['pose_timeout_ms', 100, 1000],
        ['min_samples', 3, 10], ['measurement_gap_ms', 50, 1000], ['measurement_noise_m', 0.01, 0.5],
        ['velocity_uncertainty_mps', 0.05, 3], ['acceleration_bound_mps2', 0.1, 5],
        ['max_uncertainty_m', 0.05, 1], ['max_target_speed_mps', 0.1, 5], ['innovation_gate_m', 0.1, 2],
        ['max_speed_mps', 0.05, 1], ['max_throttle_percent', 1, 25], ['max_steering_deg', 1, 15],
        ['clearance_m', 0.5, 5], ['braking_deceleration_mps2', 0.05, 1], ['recovery_blend_ms', 300, 3000]
      ]) {
        if (typeof p[key] !== 'number' || !Number.isFinite(p[key]) || p[key] < min || p[key] > max)
          errors.push(`uwb_follow.prediction.${key} must be between ${min} and ${max}`);
      }
      if (!Number.isInteger(p.min_samples)) errors.push('uwb_follow.prediction.min_samples must be integer');
      if (p.horizon_ms >= p.wait_timeout_ms) errors.push('prediction horizon must be shorter than wait timeout');
      if (p.measurement_gap_ms > Number(config.uwb_follow?.uwb_timeout_ms || 700))
        errors.push('prediction measurement gap must not exceed UWB timeout');
      if (p.measurement_noise_m >= p.max_uncertainty_m) errors.push('prediction measurement noise must be below uncertainty limit');
    }
  }
  if (config.uwb_follow?.control_input !== undefined && !['raw', 'filtered'].includes(config.uwb_follow.control_input))
    errors.push('uwb_follow.control_input must be raw or filtered');
  if (config.uwb_follow?.waiting_drift_hold !== undefined && typeof config.uwb_follow.waiting_drift_hold !== 'boolean') errors.push('uwb_follow.waiting_drift_hold must be boolean');
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
