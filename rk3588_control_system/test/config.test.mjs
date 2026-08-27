import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import { fileURLToPath } from 'node:url';
import { validateSystemConfig } from '../scripts/validate_config.mjs';

const PROJECT_ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const config = JSON.parse(fs.readFileSync(path.join(PROJECT_ROOT, 'config', 'system.config.json'), 'utf8'));
const motorConfig = JSON.parse(fs.readFileSync(path.join(PROJECT_ROOT, 'config', 'motor_config.json'), 'utf8'));

test('repository system configuration is valid', () => {
  assert.deepEqual(validateSystemConfig(config), []);
});

test('unsafe hotspot credentials are rejected', () => {
  const candidate = structuredClone(config);
  candidate.hotspot.password = 'short';
  assert.ok(validateSystemConfig(candidate).some((error) => error.includes('hotspot.password')));
});

test('invalid face hold hysteresis is rejected', () => {
  const candidate = structuredClone(config);
  candidate.gimbal.face.track_hold_enter_x_px = candidate.gimbal.face.track_hold_exit_x_px;
  assert.ok(validateSystemConfig(candidate).some((error) => error.includes('horizontal hold')));
});

test('face tracking uses the independent low-resolution loopback stream', () => {
  assert.match(config.gimbal.face.source, /\/mobile\.mjpg$/);
  assert.equal(config.gimbal.face.keep_warm, true);
  assert.equal(config.gimbal.face.prewarm, true);
});

test('face tracking uses RK3588 NPU with a PyTorch fallback', () => {
  assert.equal(config.gimbal.face.detector, 'rknn_face');
  assert.match(config.gimbal.face.model, /-rk3588-fp16\.rknn$/);
  assert.match(config.gimbal.face.fallback_model, /\.pt$/);
  const candidate = structuredClone(config);
  candidate.gimbal.face.model = candidate.gimbal.face.fallback_model;
  assert.ok(validateSystemConfig(candidate).some((error) => error.includes('.rknn model')));
});

test('face recognition profiles retain the three supported inference sizes', () => {
  assert.deepEqual(
    Object.fromEntries(Object.entries(config.gimbal.face.profiles).map(([name, value]) => [name, value.imgsz])),
    { quality: 512, balanced: 384, fast: 320 }
  );
  const candidate = structuredClone(config);
  candidate.gimbal.face.profiles.fast.imgsz = 300;
  assert.ok(validateSystemConfig(candidate).some((error) => error.includes('profiles.fast.imgsz')));
});

test('gimbal control selects exactly one supported transport', () => {
  const candidate = structuredClone(config);
  candidate.gimbal.control_transport = 'uart+udp';
  assert.ok(validateSystemConfig(candidate).some((error) => error.includes('control_transport')));
});

test('gimbal pitch lower limit is the home angle', () => {
  assert.equal(config.gimbal.axis.pitch_min_deg, 0);
  assert.equal(config.gimbal.axis.pitch_home_feedback_deg, 2.5);
  assert.ok(config.gimbal.axis.pitch_max_deg > config.gimbal.axis.pitch_min_deg);
});

test('UDP gimbal control does not require a UART overlay', () => {
  const candidate = structuredClone(config);
  candidate.gimbal.control_transport = 'udp';
  delete candidate.gimbal.uart_overlay;
  delete candidate.gimbal.boot_config;
  assert.deepEqual(validateSystemConfig(candidate), []);
});

test('UDP gimbal control requires a destination and valid port', () => {
  const missingHost = structuredClone(config);
  delete missingHost.gimbal.udp_host;
  assert.ok(validateSystemConfig(missingHost).some((error) => error.includes('udp_host')));

  const invalidPort = structuredClone(config);
  invalidPort.gimbal.udp_port = 70000;
  assert.ok(validateSystemConfig(invalidPort).some((error) => error.includes('udp_port')));
});

test('Pixhawk map input remains WGS84 before provider conversion', () => {
  const candidate = structuredClone(config);
  candidate.map.coordinate_system = 'gcj02';
  assert.ok(validateSystemConfig(candidate).some((error) => error.includes('coordinate_system')));
});

test('Pixhawk UART overlay is required for portable board installation', () => {
  const candidate = structuredClone(config);
  delete candidate.pixhawk.uart_overlay;
  assert.ok(validateSystemConfig(candidate).some((error) => error.includes('pixhawk.uart_overlay')));
});

test('rover control uses MANUAL_CONTROL with a bounded command lease', () => {
  const enabledMotors = motorConfig.motors.filter((motor) => motor.enabled);
  const leftMotor = enabledMotors.find((motor) => motor.servo_function === 73);
  const rightMotor = enabledMotors.find((motor) => motor.servo_function === 74);

  assert.equal(config.rover_control_protocol, 'manual_control');
  assert.ok(config.rover_command_timeout_ms >= 200 && config.rover_command_timeout_ms <= 2000);
  assert.ok(leftMotor, 'motor_config must define one enabled ThrottleLeft output');
  assert.ok(rightMotor, 'motor_config must define one enabled ThrottleRight output');
  assert.equal(config.rover_left_channel, leftMotor.channel);
  assert.equal(config.rover_right_channel, rightMotor.channel);
  assert.equal('rover_steering_input_channel' in config, false);
  assert.equal('rover_throttle_input_channel' in config, false);
});

test('asymmetric rover ranges are rejected because zero must remain neutral', () => {
  const candidate = structuredClone(config);
  candidate.rover_throttle_min = -80;
  candidate.rover_throttle_max = 100;
  assert.ok(validateSystemConfig(candidate).some((error) => error.includes('symmetric')));
});
