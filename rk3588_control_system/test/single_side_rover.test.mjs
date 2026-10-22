import test from 'node:test';
import assert from 'node:assert/strict';
import { singleSideTargets, motorTargetsToManual, SingleSideRoverLimiter } from '../backend/single_side_rover.js';
import { shapeRoverAxis } from '../backend/rover_slew.js';

function reconstructed(limiter, now) {
  const { throttle, steering } = limiter.step(now);
  return { left: throttle + steering / 45 * 100, right: throttle - steering / 45 * 100 };
}

test('forward/backward use both motors; horizontal input selects exactly one motor', () => {
  assert.deepEqual(singleSideTargets({ throttle: 100 }), { left: 100, right: 100, side: 'both' });
  assert.deepEqual(singleSideTargets({ throttle: -100 }), { left: -100, right: -100, side: 'both' });
  assert.deepEqual(singleSideTargets({ steering: 45 }), { left: 100, right: 0, side: 'left' });
  assert.deepEqual(singleSideTargets({ steering: -45 }), { left: 0, right: 100, side: 'right' });
  assert.deepEqual(singleSideTargets({ throttle: -80, steering: -27 }), { left: 0, right: 100, side: 'right' });
  assert.deepEqual(singleSideTargets({}), { left: 0, right: 0, side: 'neutral' });
});

test('all requested turns contain one neutral motor and stay within commissioning limits', () => {
  for (let throttle = -100; throttle <= 100; throttle += 5) {
    for (let steering = -45; steering <= 45; steering += 3) {
      const output = singleSideTargets({ throttle, steering }, 30, 15);
      assert.ok(Math.abs(output.left) <= 30 && Math.abs(output.right) <= 30);
      if (steering) assert.ok(output.left === 0 || output.right === 0);
      const mapped = motorTargetsToManual(output.left, output.right);
      assert.ok(Math.abs(mapped.throttle + mapped.steering / 45 * 100 - output.left) < 1e-9);
      assert.ok(Math.abs(mapped.throttle - mapped.steering / 45 * 100 - output.right) < 1e-9);
    }
  }
});

test('a low steering commissioning cap also limits single-motor thrust without waking the other motor', () => {
  const output = singleSideTargets({ throttle: 100, steering: 4.5 }, 100, 4.5);
  assert.deepEqual(output, { left: 20, right: 0, side: 'left' });
  assert.deepEqual(motorTargetsToManual(output.left, output.right), { throttle: 10, steering: 4.5 });
});

test('single-sided acceleration, side switches, and release never introduce an opposing motor', () => {
  const limiter = new SingleSideRoverLimiter({ maximum_dt_s: 1 });
  limiter.reset(1000);
  limiter.setTarget({ steering: 45 });
  assert.deepEqual(reconstructed(limiter, 1500), { left: 20, right: 0 });
  limiter.setTarget({ steering: -45 });
  assert.deepEqual(reconstructed(limiter, 1600), { left: 0, right: 0 });
  const switched = reconstructed(limiter, 1750);
  assert.ok(Math.abs(switched.left) < 1e-9);
  assert.equal(switched.right, 6);
  limiter.setTarget({ throttle: 0, steering: 0 });
  assert.deepEqual(reconstructed(limiter, 1850), { left: 0, right: 0 });
});

test('reverse transitions stop both motors and dwell before starting either in reverse', () => {
  const limiter = new SingleSideRoverLimiter({ maximum_dt_s: 1, reverse_neutral_ms: 300 });
  limiter.reset(1000);
  limiter.setTarget({ throttle: 100 });
  reconstructed(limiter, 2000);
  limiter.setTarget({ throttle: -100 });
  assert.deepEqual(reconstructed(limiter, 2500), { left: 0, right: 0 });
  assert.deepEqual(reconstructed(limiter, 2700), { left: 0, right: 0 });
  const output = reconstructed(limiter, 2800);
  assert.ok(output.left < 0);
  assert.equal(output.left, output.right);
  assert.ok(output.right < 0);
});

test('entering a forward-only single turn from double reverse stops both and observes neutral dwell', () => {
  const limiter = new SingleSideRoverLimiter({ maximum_dt_s: 1, reverse_neutral_ms: 300 });
  limiter.reset(1000);
  limiter.setTarget({ throttle: -100 });
  reconstructed(limiter, 2000);
  limiter.setTarget({ throttle: -100, steering: -45 });
  assert.deepEqual(reconstructed(limiter, 2100), { left: 0, right: 0 });
  assert.deepEqual(reconstructed(limiter, 2300), { left: 0, right: 0 });
  assert.ok(reconstructed(limiter, 2400).right > 0);
});

test('watchdog/emergency reset bypasses the ramp and clears pending motion', () => {
  const limiter = new SingleSideRoverLimiter({ maximum_dt_s: 1 });
  limiter.reset(1000);
  limiter.setTarget({ steering: 45 });
  reconstructed(limiter, 2000);
  assert.deepEqual(limiter.reset(2010), { throttle: 0, steering: 0 });
  assert.deepEqual(reconstructed(limiter, 2500), { left: 0, right: 0 });
});

test('rapid changing joystick targets never command opposite motor directions', () => {
  const limiter = new SingleSideRoverLimiter({ maximum_dt_s: 0.1 });
  limiter.reset(1000);
  let random = 19;
  for (let i = 1; i <= 2000; i++) {
    random = (random * 1664525 + 1013904223) >>> 0;
    const throttle = (random % 201) - 100;
    random = (random * 1664525 + 1013904223) >>> 0;
    const steering = (random % 91) - 45;
    limiter.setTarget({ throttle, steering });
    const { left, right } = reconstructed(limiter, 1000 + i * 50);
    assert.ok(left * right >= -1e-9);
    assert.ok(Math.abs(left) <= 100 && Math.abs(right) <= 100);
    if (left < -1e-9 || right < -1e-9) assert.ok(Math.abs(left - right) < 1e-9, 'reverse always uses both motors equally');
    if (limiter.intentSide === 'left') assert.ok(Math.abs(right) < 1e-9);
    if (limiter.intentSide === 'right') assert.ok(Math.abs(left) < 1e-9);
  }
});

function joystick(limiter, throttle, steering) {
  limiter.setTarget({
    throttle: shapeRoverAxis(throttle, { limit: 100, deadband: 4, expo: 1.4 }),
    steering: shapeRoverAxis(steering, { limit: 45, deadband: 2, expo: 1.4 }),
    rawThrottle: throttle, rawSteering: steering
  });
}

test('the reported -3/-5 deadband trace never flips forward/reverse', () => {
  const limiter = new SingleSideRoverLimiter();
  limiter.reset(1000);
  for (const [index, y] of [-3, -5, -3, 0, 3, -5].entries()) {
    joystick(limiter, y, 22.5);
    assert.equal(limiter.intentDirection, 1);
    assert.ok(reconstructed(limiter, 1050 + 50 * index).left >= 0);
  }
});

test('horizontal turns are forward regardless of previous reverse, side switch or release', () => {
  const limiter = new SingleSideRoverLimiter();
  limiter.reset(1000);
  joystick(limiter, -80, 0);
  assert.ok(reconstructed(limiter, 1100).left < 0);
  for (const y of [-100, -30, -5, -3, 0, 3, 9, 100]) {
    joystick(limiter, y, 22.5);
    assert.equal(limiter.intentDirection, 1);
  }
  assert.deepEqual(reconstructed(limiter, 1200), { left: 0, right: 0 });
  assert.deepEqual(reconstructed(limiter, 1400), { left: 0, right: 0 });
  assert.ok(reconstructed(limiter, 1500).left > 0);
  joystick(limiter, 0, 0);
  assert.equal(limiter.intentDirection, 0);
  joystick(limiter, 0, -22.5);
  assert.equal(limiter.intentDirection, 1);
});

test('every single-propeller target is forward, including backward diagonals and stale direction fields', () => {
  for (const side of ['left', 'right']) {
    for (let throttle = -100; throttle <= 100; throttle += 2) {
      for (const direction of [-1, 0, 1]) {
        const target = singleSideTargets({ throttle, steering: side === 'left' ? 22.5 : -22.5, side, direction });
        assert.ok(target[side] > 0);
        assert.equal(target[side === 'left' ? 'right' : 'left'], 0);
      }
    }
  }
});

test('straight backward motion remains double reverse after either turn history', () => {
  for (const steering of [-25, 25]) {
    const limiter = new SingleSideRoverLimiter();
    limiter.reset(1000);
    joystick(limiter, -80, steering);
    reconstructed(limiter, 1100);
    joystick(limiter, -80, 0);
    assert.equal(limiter.intentDirection, -1);
    assert.deepEqual(reconstructed(limiter, 1200), { left: 0, right: 0 });
    assert.deepEqual(reconstructed(limiter, 1400), { left: 0, right: 0 });
    const motors = reconstructed(limiter, 1500);
    assert.ok(motors.left < 0);
    assert.equal(motors.left, motors.right);
  }
});

test('straight/turn hysteresis ignores small jitter and does not latch the opposite side', () => {
  const limiter = new SingleSideRoverLimiter();
  for (const x of [0, 2.1, 3.5, 5.3]) {
    joystick(limiter, 50, x);
    assert.equal(limiter.intentSide, 'both');
  }
  joystick(limiter, 50, 5.5);
  assert.equal(limiter.intentSide, 'left');
  for (const x of [5.3, 3.5, 2.8]) {
    joystick(limiter, 50, x);
    assert.equal(limiter.intentSide, 'left');
  }
  joystick(limiter, 50, 2.6);
  assert.equal(limiter.intentSide, 'both');
  joystick(limiter, 50, -5.5);
  assert.equal(limiter.intentSide, 'right');
});

test('side changes hold both neutral for 150ms before ramping the new motor', () => {
  const limiter = new SingleSideRoverLimiter();
  limiter.reset(1000);
  joystick(limiter, 50, 25);
  assert.ok(reconstructed(limiter, 1100).left > 0);
  joystick(limiter, 50, -25);
  assert.deepEqual(reconstructed(limiter, 1150), { left: 0, right: 0 });
  assert.deepEqual(reconstructed(limiter, 1250), { left: 0, right: 0 });
  assert.ok(reconstructed(limiter, 1300).right > 0);
});
