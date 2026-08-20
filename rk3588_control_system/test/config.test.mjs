import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import { fileURLToPath } from 'node:url';
import { validateMotorConfig, validateSystemConfig } from '../scripts/validate_config.mjs';

const PROJECT_ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const config = JSON.parse(fs.readFileSync(path.join(PROJECT_ROOT, 'config', 'system.config.json'), 'utf8'));
const motorConfig = JSON.parse(fs.readFileSync(path.join(PROJECT_ROOT, 'config', 'motor_config.json'), 'utf8'));

test('repository system configuration is valid', () => {
  assert.deepEqual(validateSystemConfig(config), []);
  assert.deepEqual(validateMotorConfig(motorConfig, config), []);
});

test('tank steering input mapping is fixed to RC1 left and RC3 right', () => {
  const candidate = structuredClone(config);
  candidate.rover_left_input_channel = 3;
  candidate.rover_right_input_channel = 1;
  assert.ok(validateSystemConfig(candidate).some((error) => error.includes('tank steering')));
});

test('bidirectional motor neutral and Pixhawk servo reversal are explicit', () => {
  const candidate = structuredClone(motorConfig);
  candidate.brushless_config.idle_pwm = 1000;
  delete candidate.motors[0].servo_reversed;
  const errors = validateMotorConfig(candidate, config);
  assert.ok(errors.some((error) => error.includes('idle_pwm')));
  assert.ok(errors.some((error) => error.includes('servo_reversed')));
});

test('upward commands remain positive with no reversal in software, RC, or Pixhawk SERVO', () => {
  assert.equal(config.rover_throttle_sign, 1);
  for (const channel of [config.rover_left_channel, config.rover_right_channel]) {
    const motor = motorConfig.motors.find((item) => item.channel === channel);
    assert.equal(motor.servo_reversed, false, `motor channel ${channel} must not be reversed`);
  }
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
