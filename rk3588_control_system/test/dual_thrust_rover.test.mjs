import assert from 'node:assert/strict';
import test from 'node:test';
import { DualThrustRoverLimiter, dualThrustTargets } from '../backend/dual_thrust_rover.js';
import { SingleSideRoverLimiter } from '../backend/single_side_rover.js';

test('straight targets are equal; turns retain both forward propellers with the correct yaw sign', () => {
  assert.deepEqual(dualThrustTargets({ throttle: 60 }), { left: 60, right: 60, side: 'both', forwardTurn: false });
  assert.deepEqual(dualThrustTargets({ throttle: -60 }), { left: -60, right: -60, side: 'both', forwardTurn: false });
  for (const throttle of [-100, -30, 0, 30, 100]) {
    for (const steering of [-45, -10, 10, 45]) {
      const targets = dualThrustTargets({ throttle, steering });
      assert.ok(targets.left > 0 && targets.right > 0);
      assert.equal(Math.sign(targets.left - targets.right), Math.sign(steering));
      assert.ok(Math.min(targets.left, targets.right) >= Math.max(targets.left, targets.right) / 9 - 1e-9);
      assert.ok(Math.max(targets.left, targets.right) <= 100);
    }
  }
});

test('independent commissioning throttle and steering limits bound the mixed output', () => {
  for (const throttle of [0, 10, 100]) {
    for (const steering of [-45, 0, 45]) {
      const target = dualThrustTargets({ throttle, steering }, 30, 5);
      assert.ok(Math.max(Math.abs(target.left), Math.abs(target.right)) <= 30);
      assert.ok(Math.abs((target.left - target.right) / 2 * 45 / 100) <= 5 + 1e-9);
    }
  }
});

test('left/right turn changes ramp both propellers rather than cutting one', () => {
  const p = new DualThrustRoverLimiter(); p.reset(1000);
  p.setTarget({ throttle: 70, steering: 30 });
  for (let now = 1100; now <= 3000; now += 100) p.step(now);
  assert.ok(p.motors.left > p.motors.right && p.motors.right > 0);
  const previous = { ...p.motors };
  p.setTarget({ throttle: 70, steering: -30 }); p.step(3050);
  assert.ok(p.motors.left > 0 && p.motors.right > 0);
  assert.ok(Math.abs(p.motors.left - previous.left) <= 4 + 1e-9);
  assert.ok(Math.abs(p.motors.right - previous.right) <= 4 + 1e-9);
  for (let now = 3150; now <= 5000; now += 100) p.step(now);
  assert.ok(p.motors.right > p.motors.left && p.motors.left > 0);
});

test('reverse-to-turn transitions neutralize both and dwell before restarting forward', () => {
  const p = new DualThrustRoverLimiter(); p.reset(1000);
  p.setTarget({ throttle: -60 }); p.step(1100);
  assert.ok(p.motors.left < 0 && p.motors.right < 0);
  p.setTarget({ throttle: -60, steering: 30 });
  assert.deepEqual(p.step(1150), { throttle: 0, steering: 0 });
  assert.deepEqual(p.step(1400), { throttle: 0, steering: 0 });
  p.step(1450); assert.ok(p.motors.left > 0 && p.motors.right > 0);
});

test('a raw -3/-5 Y deadband trace cannot flip horizontal thrust direction', () => {
  const p = new DualThrustRoverLimiter(); p.reset(1000);
  for (const [index, throttle] of [0, -0.17, 0, -0.17].entries()) {
    p.setTarget({ throttle, steering: 16, rawSteering: 22.5 }); p.step(1100 + index * 100);
    assert.ok(p.motors.left > 0 && p.motors.right > 0);
  }
});

test('traditional single-side limiter behavior remains available and unchanged', () => {
  const p = new SingleSideRoverLimiter(); p.reset(1000);
  p.setTarget({ throttle: 70, steering: 30 }); p.step(1100);
  assert.ok(p.motors.left > 0); assert.equal(p.motors.right, 0);
});

test('rapid targets never exceed limits or drive propellers in opposite directions', () => {
  const p = new DualThrustRoverLimiter(); p.reset(1000);
  let seed = 23;
  for (let i = 1; i <= 2000; i++) {
    seed = (seed * 1664525 + 1013904223) >>> 0;
    const throttle = seed % 201 - 100;
    seed = (seed * 1664525 + 1013904223) >>> 0;
    p.setTarget({ throttle, steering: seed % 91 - 45 }); p.step(1000 + i * 50);
    assert.ok(p.motors.left * p.motors.right >= 0);
    assert.ok(Math.abs(p.motors.left) <= 100 && Math.abs(p.motors.right) <= 100);
  }
});
