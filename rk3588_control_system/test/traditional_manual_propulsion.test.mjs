// Legacy behavioral baseline; replacement is checked by check_native_*.py/mjs.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import { TraditionalManualPropulsion } from '../backend/traditional_manual_propulsion.js';
import { UwbFollowController } from '../backend/uwb_follow.js';

const ready = { flightMode: 'MANUAL', armed: true, pilotSteering: { ready: true, type: 0 } };
function propulsion() {
  const p = new TraditionalManualPropulsion({ throttle_limit: 70, steering_limit: 18, maximum_dt_s: 0.1 });
  p.reset(1000);
  return p;
}
function update(p, now, throttle, steering, telemetry = ready, extra = {}) {
  return p.update({ now, connected: true, telemetry, result: { safe: true, throttlePercent: throttle, steeringDegrees: steering }, ...extra });
}
function motors(output) {
  return { left: output.throttle + output.steering / 45 * 100, right: output.throttle - output.steering / 45 * 100 };
}
function settle(p, throttle, steering, from = 1000) {
  let output;
  for (let now = from + 100; now <= from + 3000; now += 100) output = update(p, now, throttle, steering);
  return output;
}

test('traditional turns use both forward motors, preserving mean approach thrust', () => {
  for (const [steering, expected] of [[0, { left: 20, right: 20 }], [4.5, { left: 30, right: 10 }], [-4.5, { left: 10, right: 30 }], [18, { left: 36, right: 4 }]]) {
    assert.deepEqual(motors(settle(propulsion(), 20, steering)), expected);
  }
});

test('distance recovery uses both reverse motors even with residual steering', () => {
  for (const steering of [-18, 0, 18]) {
    assert.deepEqual(motors(settle(propulsion(), -20, steering)), { left: -20, right: -20 });
  }
});

test('turn direction changes smoothly redistribute thrust between both motors', () => {
  const p = propulsion();
  settle(p, 20, 18);
  const after = motors(update(p, 4100, 20, -18));
  assert.ok(after.left > 0 && after.right > 0);
  assert.ok(after.left < 36 && after.right > 4);
  assert.deepEqual(motors(settle(p, 20, -18, 4100)), { left: 4, right: 36 });
});

test('traditional motor saturation scales both sides together and preserves their direction', () => {
  const p = propulsion();
  const m = motors(settle(p, 70, 18));
  assert.equal(m.left, 70);
  assert.ok(m.right > 0 && m.right < m.left);
  assert.ok(Math.abs(m.right - 30 * 70 / 110) < 1e-9);
  for (let i = 0; i < 500; i++) {
    const throttle = i % 110 < 80 ? 70 : -20;
    const steering = Math.sin(i * 0.3) * 18;
    const next = motors(update(p, 4100 + i * 100, throttle, steering));
    assert.ok(next.left * next.right >= -1e-9);
    assert.ok(Math.abs(next.left) <= 70 + 1e-9 && Math.abs(next.right) <= 70 + 1e-9);
  }
});

test('forward/reverse transitions dwell at neutral and never drive opposing motors', () => {
  const p = propulsion();
  settle(p, 20, 18);
  let firstNeutral = null, firstReverse = null;
  for (let now = 4100; now < 6000; now += 100) {
    const m = motors(update(p, now, -20, -18));
    assert.ok(m.left * m.right >= -1e-9);
    if (!m.left && !m.right && firstNeutral === null) firstNeutral = now;
    if (m.left < 0 && firstReverse === null) firstReverse = now;
  }
  assert.ok(firstReverse - firstNeutral >= 300);
  assert.deepEqual(motors(p.limiter.output), { left: -20, right: -20 });
});

test('holding, zero throttle and invalid targets reset propulsion immediately without residual turn', () => {
  for (const [throttle, steering, extra] of [[0, 18, {}], [20, 18, { result: { safe: true, reason: 'holding_distance', throttlePercent: 20, steeringDegrees: 18 } }], [NaN, 18, {}]]) {
    const p = propulsion();
    settle(p, 20, 18);
    assert.deepEqual(motors(update(p, 4100, throttle, steering, ready, extra)), { left: 0, right: 0 });
  }
});

test('an outer-loop zero between forward and reverse does not bypass the neutral dwell', () => {
  const p = propulsion();
  settle(p, 20, 18);
  assert.deepEqual(motors(update(p, 4100, 0, 0)), { left: 0, right: 0 });
  assert.deepEqual(motors(update(p, 4200, 0, 0)), { left: 0, right: 0 });
  assert.deepEqual(motors(update(p, 4300, -20, 0)), { left: 0, right: 0 });
  const reverse = motors(update(p, 4400, -20, 0));
  assert.ok(reverse.left < 0 && reverse.left === reverse.right);
});

test('mode, arming, link, pilot parameter and sensor faults cannot leave a live target', () => {
  for (const [telemetry, extra, reason] of [
    [{ ...ready, flightMode: 'ACRO' }, {}, 'manual_required'],
    [{ ...ready, armed: false }, {}, 'armed_required'],
    [ready, { connected: false }, 'pixhawk_offline'],
    [{ ...ready, pilotSteering: { ready: false, type: 0 } }, {}, 'pilot_steering_not_ready'],
    [{ ...ready, pilotSteering: { ready: true, type: 99 } }, {}, 'pilot_steering_not_ready'],
    [ready, { result: { safe: false, reason: 'uwb_stale', throttlePercent: 20, steeringDegrees: 18 } }, 'uwb_stale']
  ]) {
    const p = propulsion();
    settle(p, 20, 18);
    const out = update(p, 4100, 20, 18, telemetry, extra);
    assert.equal(out.safe, false);
    assert.equal(out.reason, reason);
    assert.deepEqual(motors(out), { left: 0, right: 0 });
    assert.deepEqual(p.limiter.target, { throttle: 0, steering: 0 });
  }
});

test('traditional preflight requires MANUAL with or without navigation and rejects missing pilot parameters', () => {
  const now = 10000;
  const p = new UwbFollowController({ manual_propulsion: 'single_side', bearing_sign: 1, bearing_scale: 1, bearing_offset_deg: 0 });
  const telemetry = { ...ready, attitude: { yaw: 0 }, uwb: { online: true, fresh: true, distanceM: 3, azimuthDeg: 0, updatedAt: now } };
  for (const navigation of [ {}, { gps: { fixType: 3 }, position: { updatedAt: now }, ekf: { healthy: true, updatedAt: now }, velocity: { vx: 0, vy: 0 } } ]) {
    const t = { ...telemetry, ...navigation };
    const preflight = p.preflight({ now, telemetry: t, connected: true });
    assert.equal(preflight.safe, true);
    assert.equal(preflight.controlMode, 'relative_manual');
    assert.equal(preflight.requiredFlightMode, 'MANUAL');
    const result = p.update({ now, telemetry: t, connected: true });
    assert.equal(result.safe, true);
    assert.ok(result.throttlePercent > 0);
    assert.equal(p.update({ now, telemetry: { ...t, flightMode: 'ACRO' }, connected: true }).reason, 'manual_required');
  }
  assert.equal(p.preflight({ now, telemetry: { ...telemetry, pilotSteering: null }, connected: true }).reason, 'pilot_steering_not_ready');
  assert.equal(p.update({ now: now + 1000, telemetry, connected: true }).reason, 'uwb_stale');
});

test('real server traditional sender uses the joystick sender and marks motor target semantics', () => {
  const source = fs.readFileSync(new URL('./fixtures/legacy-server-control.txt', import.meta.url), 'utf8');
  const functions = [
    source.slice(source.indexOf('function normalizeRoverControl('), source.indexOf('let lastRoverCommandAt')),
    source.slice(source.indexOf('function sendManualRoverOutput('), source.indexOf('function applyRoverControl(')),
    source.slice(source.indexOf('function sendUwbRelativeDrive('), source.indexOf('function holdOutputsAreNeutral('))
  ].join('\n');
  const calls = [];
  const p = propulsion();
  const sandbox = {
    intermittentUwbFollow: { governDrive: (output) => output },
    intermittentUwbFollow: { governDrive: (output) => output },
    traditionalManualPropulsion: p, manualRoverSource: 'WEB', lastRoverCommandAt: 0,
    PWM_CENTER: 1500, PWM_MIN: 1000, PWM_MAX: 2000,
    ROVER_THROTTLE_MIN: -100, ROVER_THROTTLE_MAX: 100, ROVER_MANUAL_THROTTLE_LIMIT: 100,
    ROVER_STEERING_MIN: -45, ROVER_STEERING_MAX: 45, ROVER_MANUAL_STEERING_LIMIT: 45,
    ROVER_CONTROL_PROTOCOL: 'manual_control', ROVER_LEFT_CHANNEL: 1, ROVER_RIGHT_CHANNEL: 3,
    systemState: { telemetry: ready, isConnected: true, motorStatus: {} },
    io: { emit() {} }, Date: class extends Date { static now() { return 5000; } },
    asFiniteNumber: (x, fallback) => Number.isFinite(Number(x)) ? Number(x) : fallback,
    clamp: (x, lo, hi) => Math.min(hi, Math.max(lo, x)),
    toPwm: (x) => Math.round(Math.min(2000, Math.max(1000, x))),
    sendMavlinkCommand: (command, params) => calls.push({ command, params }),
    forceNeutralRoverControl: () => calls.push({ command: 'neutral' })
  };
  vm.createContext(sandbox);
  vm.runInContext(functions, sandbox);
  vm.runInContext("sendUwbRelativeDrive({ controlMode: 'relative_manual', safe: true, throttlePercent: 20, steeringDegrees: 0 })", sandbox);
  assert.equal(calls.length, 1);
  assert.equal(calls[0].command, 'ROVER_DRIVE');
  assert.equal(calls[0].params.motorTargetSemantics, true);
  assert.ok(calls[0].params.throttle > 0);
  assert.equal(calls[0].params.steering, 0);
  assert.equal(sandbox.systemState.roverControl.leftPwm, sandbox.systemState.roverControl.rightPwm);
  sandbox.systemState.telemetry = { ...ready, flightMode: 'ACRO' };
  vm.runInContext("sendUwbRelativeDrive({ controlMode: 'relative_manual', safe: true, throttlePercent: 20, steeringDegrees: 18 })", sandbox);
  assert.equal(calls.at(-1).command, 'neutral');
});
