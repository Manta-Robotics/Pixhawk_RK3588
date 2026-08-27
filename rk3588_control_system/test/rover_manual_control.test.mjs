import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import { fileURLToPath } from 'node:url';

const projectRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const bridge = fs.readFileSync(path.join(projectRoot, 'backend', 'mavlink_bridge.py'), 'utf8');
const server = fs.readFileSync(path.join(projectRoot, 'backend', 'server.js'), 'utf8');
const transport = fs.readFileSync(path.join(projectRoot, 'frontend', 'js', 'manta-app-transport.js'), 'utf8');
const motorConfig = JSON.parse(fs.readFileSync(path.join(projectRoot, 'config', 'motor_config.json'), 'utf8'));

test('Pixhawk mixer owns left and right throttle outputs', () => {
  const enabled = motorConfig.motors.filter((motor) => motor.enabled !== false);
  assert.deepEqual(enabled.map((motor) => [motor.channel, motor.servo_function]), [[1, 73], [3, 74]]);
  assert.equal(motorConfig.brushless_config.mavlink_input, 'MANUAL_CONTROL');
  for (const motor of enabled) {
    assert.deepEqual([motor.min_pwm, motor.center_pwm, motor.max_pwm], [1000, 1500, 2000]);
  }
});

test('bridge uses MANUAL_CONTROL and does not bypass Rover output functions', () => {
  assert.match(bridge, /manual_control_send\(/);
  assert.match(bridge, /throttle_axis = round\(-1000\.0 \+/);
  assert.match(bridge, /min\(1000, throttle_axis\)/);
  assert.doesNotMatch(bridge, /rc_channels_override_send\(/);
  assert.doesNotMatch(bridge, /MAV_CMD_DO_SET_SERVO/);
  assert.match(bridge, /self\._send_rover_drive\(0\.0, 0\.0\)[\s\S]*self\._arm_disarm\(False\)/);
});

test('Mission Planner heartbeats cannot overwrite FCU arming state', () => {
  assert.match(bridge, /is_target_fcu_heartbeat\(msg, self\.target_system, self\.target_component\)/);
  assert.match(bridge, /vehicle_type != mavutil\.mavlink\.MAV_TYPE_GCS/);
});

test('Pixhawk output-only motor channels remain usable without claiming ESC feedback', () => {
  assert.match(transport, /leftMotor\.online \|\| leftMotor\.outputOnline/);
  assert.match(transport, /rightMotor\.online \|\| rightMotor\.outputOnline/);
});

test('server and client maintain a bounded MANUAL_CONTROL lease', () => {
  assert.match(server, /ROVER_COMMAND_TIMEOUT_MS/);
  assert.match(server, /Rover command lease expired/);
  assert.match(server, /protocol: ROVER_CONTROL_PROTOCOL/);
  assert.match(transport, /startDriveKeepalive/);
  assert.match(transport, /this\.drive\(vector\);[\s\S]*100\);/);
});

test('manual commissioning caps throttle and steering before they reach the bridge', () => {
  assert.match(server, /ROVER_MANUAL_THROTTLE_LIMIT/);
  assert.match(server, /ROVER_MANUAL_STEERING_LIMIT/);
  assert.match(server, /Math\.max\(ROVER_THROTTLE_MIN, -ROVER_MANUAL_THROTTLE_LIMIT\)/);
  assert.match(server, /Math\.min\(ROVER_THROTTLE_MAX, ROVER_MANUAL_THROTTLE_LIMIT\)/);
  assert.match(server, /Math\.max\(ROVER_STEERING_MIN, -ROVER_MANUAL_STEERING_LIMIT\)/);
  assert.match(server, /Math\.min\(ROVER_STEERING_MAX, ROVER_MANUAL_STEERING_LIMIT\)/);
  assert.match(server, /leftPwm = toPwm\(PWM_CENTER \+ throttle \* throttleScale \+ steering \* steeringScale\)/);
  assert.match(server, /rightPwm = toPwm\(PWM_CENTER \+ throttle \* throttleScale - steering \* steeringScale\)/);
});

test('arming UI waits for FCU telemetry instead of optimistic state', () => {
  assert.match(server, /Boolean\(previous\.armed\) !== Boolean\(nextTelemetry\.armed\)/);
  const socketArmHandlers = server.match(/socket\.on\('arm',[\s\S]*?socket\.on\('request_telemetry'/)?.[0] || '';
  assert.doesNotMatch(socketArmHandlers, /systemState\.telemetry\.armed\s*=/);
  assert.doesNotMatch(socketArmHandlers, /io\.emit\('aircraft_(?:armed|disarmed)'/);
});
