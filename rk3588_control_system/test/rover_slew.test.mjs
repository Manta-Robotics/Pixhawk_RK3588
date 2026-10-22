import test from 'node:test';
import assert from 'node:assert/strict';

import { RoverSlewLimiter, shapeRoverAxis } from '../backend/rover_slew.js';

test('manual input shaping adds a deadband and progressive response', () => {
  assert.equal(shapeRoverAxis(3, { limit: 100, deadband: 4, expo: 1.4 }), 0);
  const middle = shapeRoverAxis(50, { limit: 100, deadband: 4, expo: 1.4 });
  assert.ok(middle > 0 && middle < 50);
  assert.equal(shapeRoverAxis(100, { limit: 100, deadband: 4, expo: 1.4 }), 100);
  assert.equal(shapeRoverAxis(-100, { limit: 100, deadband: 4, expo: 1.4 }), -100);
});

test('throttle and steering ramp instead of jumping to their targets', () => {
  const limiter = new RoverSlewLimiter({
    throttle_accel_percent_s: 40,
    throttle_decel_percent_s: 80,
    steering_slew_deg_s: 40,
    maximum_dt_s: 1
  });
  limiter.reset(1000);
  limiter.setTarget({ throttle: 100, steering: 45 });
  assert.deepEqual(limiter.step(1100), { throttle: 4, steering: 4 });
  assert.deepEqual(limiter.step(1200), { throttle: 8, steering: 8 });
});

test('releasing the joystick decelerates faster but still smoothly', () => {
  const limiter = new RoverSlewLimiter({
    throttle_accel_percent_s: 40,
    throttle_decel_percent_s: 80,
    steering_slew_deg_s: 40,
    maximum_dt_s: 1
  });
  limiter.reset(1000);
  limiter.setTarget({ throttle: 100, steering: 0 });
  limiter.step(2000);
  limiter.setTarget({ throttle: 0, steering: 0 });
  assert.equal(limiter.step(2100).throttle, 32);
  assert.equal(limiter.step(2500).throttle, 0);
});

test('direction reversal must pass through neutral and dwell before reversing', () => {
  const limiter = new RoverSlewLimiter({
    throttle_accel_percent_s: 100,
    throttle_decel_percent_s: 100,
    steering_slew_deg_s: 100,
    reverse_neutral_ms: 300,
    maximum_dt_s: 1
  });
  limiter.reset(1000);
  limiter.setTarget({ throttle: 100, steering: 0 });
  assert.equal(limiter.step(1500).throttle, 50);
  limiter.setTarget({ throttle: -100, steering: 0 });
  assert.equal(limiter.step(2000).throttle, 0);
  assert.equal(limiter.step(2200).throttle, 0);
  assert.equal(limiter.step(2300).throttle, -10);
});

test('reset bypasses the ramp and returns an immediate neutral output', () => {
  const limiter = new RoverSlewLimiter({ maximum_dt_s: 1 });
  limiter.reset(1000);
  limiter.setTarget({ throttle: 100, steering: 45 });
  limiter.step(1500);
  assert.deepEqual(limiter.reset(1600), { throttle: 0, steering: 0 });
});
