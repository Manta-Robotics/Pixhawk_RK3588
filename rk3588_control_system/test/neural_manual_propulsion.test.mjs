import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import vm from 'node:vm';
import { fileURLToPath } from 'node:url';
import { NeuralManualPropulsion } from '../backend/neural_manual_propulsion.js';
import { NeuralUwbFollowController, JsonActor } from '../backend/neural_uwb_follow.js';
import { DualThrustRoverLimiter } from '../backend/dual_thrust_rover.js';
import { shapeRoverAxis } from '../backend/rover_slew.js';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const modelPath = path.join(root, 'models/smart_usv_uwb_policy.json');
const ready = { flightMode: 'MANUAL', armed: true, pilotSteering: { type: 0, ready: true } };
function motors(output) {
  return { left: output.throttle + output.steering / 45 * 100, right: output.throttle - output.steering / 45 * 100 };
}
function result(throttle = 70, steering = 0) {
  return { safe: true, trackingMethod: 'neural', throttlePercent: throttle, steeringDegrees: steering };
}
function step(p, now, target, overrides = {}) {
  return p.update({ now, result: target, telemetry: ready, connected: true, ...overrides });
}
function telemetry(now, overrides = {}) {
  return { ...ready, attitude: { yaw: 0, yawRateDegS: 0 }, velocity: { vx: 0, vy: 0 },
    gps: { fixType: 3 }, ekf: { healthy: true, updatedAt: now }, position: { updatedAt: now },
    uwb: { online: true, fresh: true, distanceM: 3, azimuthDeg: 15, updatedAt: now }, ...overrides };
}
function controller() {
  return new NeuralUwbFollowController({ neural_model_path: modelPath,
    relative_throttle_limit_percent: 70, relative_steering_limit_deg: 18, max_yaw_rate_deg_s: 14 });
}

test('real deployed neural model passes its golden inference vectors', () => {
  const actor = new JsonActor(modelPath);
  actor.verifyGolden();
  assert.equal(actor.infer(actor.model.golden.observations[0]).length, 2);
});

test('neural propulsion produces exactly the manual joystick outputs over a changing input trace', () => {
  const p = new NeuralManualPropulsion();
  const manual = new DualThrustRoverLimiter();
  p.reset(1000); manual.reset(1000);
  const trace = [[70, 0], [70, 0], [70, 15], [70, -15], [70, -15], [0, -15],
    [-70, -15], [-70, 0], [-70, 0], [-70, 0], [-70, 0], [0, 0], [70, 0], [70, 15]];
  trace.forEach(([throttle, steering], i) => {
    const now = 1100 + i * 100;
    manual.setTarget({ throttle: shapeRoverAxis(throttle, { limit: 100, deadband: 4, expo: 1.4 }),
      steering: shapeRoverAxis(steering, { limit: 45, deadband: 2, expo: 1.4 }),
      rawThrottle: throttle, rawSteering: steering });
    const output = step(p, now, result(throttle, steering));
    assert.deepEqual({ throttle: output.throttle, steering: output.steering }, manual.step(now));
  });
});

test('turns keep both propellers forward with unequal thrust; straight backward reverses both', () => {
  for (const [throttle, steering, side] of [[70, 18, 'left'], [70, -18, 'right'], [-70, 18, 'left'], [-70, -18, 'right']]) {
    const p = new NeuralManualPropulsion(); p.reset(1000);
    const output = motors(step(p, 1100, result(throttle, steering)));
    assert.ok(output.left > 0 && output.right > 0);
    for (let now = 1200; now <= 3000; now += 100) step(p, now, result(throttle, steering));
    assert.ok(p.limiter.motors[side] > p.limiter.motors[side === 'left' ? 'right' : 'left']);
  }
  const p = new NeuralManualPropulsion(); p.reset(1000);
  const output = motors(step(p, 1100, result(-70, 0)));
  assert.ok(output.left < 0); assert.equal(output.left, output.right);
});

test('holding, stale sensors, offline FCU, unknown parameter, bad mode and disarming stop immediately', () => {
  for (const override of [{ result: { ...result(), reason: 'holding_distance' } },
    { result: { ...result(), safe: false, reason: 'uwb_stale' } }, { connected: false },
    { telemetry: { ...ready, pilotSteering: { ready: false, type: null } } },
    { telemetry: { ...ready, flightMode: 'ACRO' } }, { telemetry: { ...ready, armed: false } },
    { result: result(NaN, 5) }]) {
    const p = new NeuralManualPropulsion(); p.reset(1000);
    assert.ok(step(p, 1100, result(70, 18)).throttle > 0);
    const output = step(p, 1150, result(), override);
    assert.deepEqual(motors(output), { left: 0, right: 0 });
    assert.deepEqual(p.limiter.motors, { left: 0, right: 0 });
  }
});

test('neural preflight always chooses MANUAL with valid UWB, with or without GPS and legacy mode requests', () => {
  const c = controller();
  for (const gps of [{ fixType: 3 }, { fixType: 0 }]) {
    for (const controlMode of ['auto', 'guided_velocity', 'relative_acro', 'relative_manual']) {
      const preflight = c.preflight({ now: 10000, connected: true,
        telemetry: telemetry(10000, { gps, armed: false }), controlMode });
      assert.equal(preflight.safe, true);
      assert.equal(preflight.controlMode, 'relative_manual');
      assert.equal(preflight.requiredFlightMode, 'MANUAL');
    }
  }
});

test('neural sensor and mode checks prevent inference while unsafe', () => {
  const c = controller();
  let inferences = 0;
  c.actor.infer = () => { inferences++; return [1, 0]; };
  for (const [reason, override] of [['uwb_stale', { uwb: { online: false } }],
    ['imu_invalid', { attitude: { yaw: null } }], ['pilot_steering_not_ready', { pilotSteering: null }],
    ['armed_required', { armed: false }], ['manual_required', { flightMode: 'ACRO' }]]) {
    const output = c.update({ now: 10000, connected: true, telemetry: telemetry(10000, override) });
    assert.equal(output.safe, false); assert.equal(output.reason, reason);
  }
  assert.equal(inferences, 0);
});

test('neural heading decisions produce both forward propellers and more thrust on the correct side', () => {
  for (const [action, side] of [[-0.25, 'left'], [0.25, 'right']]) {
    const c = controller(); c.actor.infer = () => [1, action];
    const p = new NeuralManualPropulsion(); p.reset(1000);
    const target = c.update({ now: 1100, connected: true, telemetry: telemetry(1100) });
    assert.equal(target.controlMode, 'relative_manual');
    const output = motors(step(p, 1100, target));
    assert.ok(output.left > 0 && output.right > 0);
    for (let now = 1200; now <= 3000; now += 100) step(p, now, target);
    assert.ok(p.limiter.motors[side] > p.limiter.motors[side === 'left' ? 'right' : 'left']);
  }
});

test('the newest near-distance measurement stops immediately despite the neural range filter', () => {
  const c = controller(); c.actor.infer = () => [1, 0];
  const p = new NeuralManualPropulsion(); p.reset(1000);
  step(p, 1100, c.update({ now: 1100, connected: true, telemetry: telemetry(1100) }));
  const close = telemetry(1150); close.uwb.distanceM = 0.5;
  const target = c.update({ now: 1150, connected: true, telemetry: close });
  assert.equal(target.reason, 'holding_distance');
  assert.deepEqual(motors(step(p, 1150, target)), { left: 0, right: 0 });
});

test('server neural drive dispatch uses the manual output path and stop emits no velocity command', () => {
  const source = fs.readFileSync(path.join(root, 'backend/server.js'), 'utf8');
  const drive = source.slice(source.indexOf('function sendUwbRelativeDrive('), source.indexOf('function holdOutputsAreNeutral('));
  const stop = source.slice(source.indexOf('function sendUwbFollowStop('), source.indexOf('function idleUwbFollowResult('));
  const calls = [];
  const p = new NeuralManualPropulsion(); p.reset(1000);
  const context = vm.createContext({ intermittentUwbFollow: { governDrive: (output) => output }, uwbTrackingMethod: 'neural', neuralManualPropulsion: p,
    systemState: { telemetry: ready, isConnected: true }, Date, lastRoverCommandAt: 0,
    sendManualRoverOutput: (output) => { calls.push('manual'); return output; },
    sendMavlinkCommand: (command) => calls.push(command), forceNeutralRoverControl: () => calls.push('neutral') });
  vm.runInContext(drive + '\n' + stop, context);
  context.target = result(70, 18);
  vm.runInContext('sendUwbRelativeDrive(target); sendUwbFollowStop("test");', context);
  assert.deepEqual(calls, ['manual', 'neutral']);
  assert.deepEqual(p.limiter.motors, { left: 0, right: 0 });
});

test('neural propulsion uses the manual command lease and disarms on stale output commands', () => {
  const source = fs.readFileSync(path.join(root, 'backend/server.js'), 'utf8');
  const watcher = source.slice(source.indexOf('function updateManualRoverSlew('), source.indexOf('function disarmAndReturnManual('));
  const calls = [];
  const context = vm.createContext({ Date: { now: () => 2000 }, uwbFollowState: { active: true },
    uwbTrackingMethod: 'neural', neuralManualPropulsion: { limiter: { output: { throttle: 4, steering: 0 } } },
    lastRoverCommandAt: 1100, ROVER_COMMAND_TIMEOUT_MS: 500,
    stopUwbFollow: (reason, options) => calls.push([reason, options.disarm]) });
  vm.runInContext(watcher + '\nupdateManualRoverSlew();', context);
  assert.deepEqual(calls, [['command_lease_expired', true]]);
  calls.length = 0;
  context.lastRoverCommandAt = 1900;
  vm.runInContext('updateManualRoverSlew();', context);
  assert.deepEqual(calls, []);
});
