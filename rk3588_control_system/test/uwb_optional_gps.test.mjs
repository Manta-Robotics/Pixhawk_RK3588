// Legacy behavioral baseline; replacement is checked by check_native_*.py/mjs.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import { fileURLToPath } from 'node:url';
import { UwbFollowController } from '../backend/uwb_follow.js';
import { NeuralUwbFollowController } from '../backend/neural_uwb_follow.js';
import { IntermittentUwbFollow } from '../backend/uwb_target_prediction.js';
import { validateSystemConfig } from '../scripts/validate_config.mjs';

const config = {
  execution_mode: 'relative_imu', controller_profile: 'traditional_fast',
  bearing_sign: 1, bearing_scale: 1, bearing_offset_deg: 0,
  following_distance_m: .5, safety_distance_m: .3, safety_resume_distance_m: 1.2,
  hard_stop_enabled: false, max_speed_mps: 2, commissioning_speed_mps: 2,
  relative_throttle_limit_percent: 70, relative_steering_limit_deg: 18,
  max_yaw_rate_deg_s: 14, prediction: { enabled: true },
  gps_speed_closed_loop: { enabled: true, acquire_ms: 2000, kp: 20, ki: 8, maximum_observed_speed_mps: 5 }
};
function packet(now, gps = false, speed = 0, bearing = 0, distance = 3) {
  return { armed: true, flightMode: 'MANUAL', pilotSteering: { ready: true, type: 0 },
    gps: { fixType: gps ? 3 : 1, updatedAt: now },
    position: { updatedAt: now }, velocity: { vx: gps ? speed : null, vy: gps ? 0 : null },
    ekf: { healthy: gps, updatedAt: now }, attitude: { yaw: 0, yawRateDegS: 0, updatedAt: now },
    uwb: { online: true, fresh: true, distanceM: distance, azimuthDeg: bearing,
      measurementUpdatedAt: now, measurementStatus: 'trusted' } };
}
function update(c, now, telemetry) { return c.update({ now, telemetry, connected: true }); }

test('no GPS preserves UWB angular steering and the existing relative throttle law', () => {
  const hybrid = new UwbFollowController(config);
  const relative = new UwbFollowController({ ...config, gps_speed_closed_loop: { enabled: false } });
  for (let t = 1000; t <= 4000; t += 100) {
    const telemetry = packet(t, false, 0, 20);
    const a = update(hybrid, t, telemetry), b = update(relative, t, telemetry);
    assert.equal(a.safe, true); assert.equal(a.controlMode, 'relative_manual');
    assert.equal(a.throttlePercent, b.throttlePercent);
    assert.equal(a.steeringDegrees, b.steeringDegrees);
    assert.equal(a.gpsSpeedControlActive, false); assert.equal(a.forwardSpeedMps, null);
  }
});

test('GPS needs continuous confirmation and adds feedback without changing UWB steering', () => {
  const c = new UwbFollowController(config), relative = new UwbFollowController(config);
  for (let t = 1000; t <= 4000; t += 100) {
    const a = update(c, t, packet(t, true, .2, 20));
    const b = update(relative, t, packet(t, false, 0, 20));
    assert.equal(a.gpsSpeedControlActive, t >= 3000);
    assert.equal(a.steeringDegrees, b.steeringDegrees);
    assert.equal(a.requiredFlightMode, 'MANUAL');
    if (a.gpsSpeedControlActive) {
      assert.equal(a.absoluteSpeedObservable, true);
      assert.ok(Number.isFinite(a.desiredSpeedMps));
      assert.ok(a.desiredSpeedMps >= 0 && a.desiredSpeedMps <= 2);
    }
  }
  const before = c.relativeImuControl.throttle;
  const overspeed = update(c, 4100, packet(4100, true, 2));
  assert.ok(overspeed.throttlePercent < before, 'measured overspeed must reduce thrust');
});

test('GPS loss falls back immediately and reacquisition cannot toggle modes rapidly', () => {
  const c = new UwbFollowController(config);
  for (let t = 1000; t <= 4000; t += 100) update(c, t, packet(t, true, .2));
  let previous = c.relativeImuControl.throttle;
  for (let t = 4100; t <= 8000; t += 100) {
    const hasGps = t >= 4500;
    const result = update(c, t, packet(t, hasGps, .2));
    assert.equal(result.safe, true);
    assert.equal(result.gpsSpeedControlActive, t >= 6500);
    assert.equal(result.requiredFlightMode, 'MANUAL');
    assert.ok(Math.abs(result.throttlePercent - previous) <= 2.5 + 1e-8);
    previous = result.throttlePercent;
  }
});

test('stale, future, unhealthy or nonfinite navigation falls back without poisoning UWB', () => {
  const corruptions = [
    t => { t.gps.updatedAt = 0; }, t => { t.position.updatedAt += 1; },
    t => { t.ekf.healthy = false; }, t => { t.velocity.vx = NaN; },
    t => { t.velocity.vy = 900; }, t => { t.gps.fixType = 1; }
  ];
  for (const corrupt of corruptions) {
    const c = new UwbFollowController(config);
    for (let now = 1000; now <= 4000; now += 100) update(c, now, packet(now, true, .2));
    const telemetry = packet(4100, true, .2); corrupt(telemetry);
    const result = update(c, 4100, telemetry);
    assert.equal(result.safe, true); assert.equal(result.gpsSpeedControlActive, false);
    assert.ok(Number.isFinite(result.throttlePercent));
  }
});

test('GPS cannot bypass UWB loss, stale IMU, arming, mode or close-range guards', () => {
  for (const mutate of [t => { t.armed = false; }, t => { t.flightMode = 'ACRO'; },
    t => { t.attitude.updatedAt = 0; }, t => { t.uwb.fresh = false; }]) {
    const c = new UwbFollowController(config), telemetry = packet(1000, true);
    mutate(telemetry); assert.equal(update(c, 1000, telemetry).safe, false);
  }
  const c = new UwbFollowController(config);
  for (let t = 1000; t <= 4000; t += 100) update(c, t, packet(t, true));
  const stopped = update(c, 4100, packet(4100, true, 0, 0, .29));
  assert.equal(stopped.throttlePercent, 0); assert.equal(stopped.steeringDegrees, 0);
});

test('actual intermittent wrapper survives GPS disappearance and bounds the existing UWB prediction window', () => {
  const c = new UwbFollowController(config), wrapper = new IntermittentUwbFollow(config.prediction, config);
  for (let t = 1000; t <= 5000; t += 100) {
    const r = wrapper.update({ now: t, controller: c, telemetry: packet(t, t < 3500), connected: true });
    assert.equal(r.safe, true); assert.equal(r.controlMode, 'relative_manual');
  }
  const t = packet(5100); t.uwb.online = false; t.uwb.fresh = false;
  const waiting = wrapper.update({ now: 5100, controller: c, telemetry: t, connected: true });
  assert.ok(waiting.throttlePercent <= 15); assert.ok(Math.abs(waiting.steeringDegrees) <= 6);
  for (let now = 5200; now <= 6200; now += 100) {
    const missing = packet(now); missing.uwb.online = false; missing.uwb.fresh = false;
    const r = wrapper.update({ now, controller: c, telemetry: missing, connected: true });
    if (now >= 6100) { assert.equal(r.throttlePercent, 0); assert.equal(r.steeringDegrees, 0); }
  }
});

test('moving-target plant remains bounded through GPS acquisition, loss and recovery', () => {
  const c = new UwbFollowController(config);
  let distance = 4, speed = 0, minimum = distance;
  const tail = [];
  for (let i = 0; i < 1200; i++) {
    const now = 1000 + i * 100, gps = i < 300 || i >= 600;
    const r = update(c, now, packet(now, gps, speed, 0, distance));
    assert.equal(r.safe, true);
    assert.ok(r.throttlePercent >= 0 && r.throttlePercent <= 70);
    speed += Math.max(-.025, Math.min(.025, r.throttlePercent / 70 * 2 - speed));
    distance += (.8 - speed) * .1; minimum = Math.min(minimum, distance);
    if (i > 1000) tail.push(distance);
  }
  assert.ok(minimum > .3, `closest approach ${minimum}`);
  assert.ok(distance < 1, `final range ${distance}`);
  assert.ok(Math.max(...tail) - Math.min(...tail) < .15, `tail spread ${Math.max(...tail)-Math.min(...tail)}`);
});

test('neural execution shares GPS fallback without claiming GPS inputs reached the actor', () => {
  const options = { ...config, neural_model_path: fileURLToPath(new URL('../models/smart_usv_uwb_policy.json', import.meta.url)) };
  const c = new NeuralUwbFollowController(options), relative = new NeuralUwbFollowController(options);
  for (let now = 1000; now <= 4000; now += 100) {
    const r = update(c, now, packet(now, true, .2, 10));
    const baseline = update(relative, now, packet(now, false, 0, 10));
    assert.equal(r.safe, true); assert.equal(r.gpsSpeedControlActive, now >= 3000);
    assert.equal(r.steeringDegrees, baseline.steeringDegrees);
    assert.equal(r.neuralAbsoluteVelocityMissing, true);
  }
  assert.equal(update(c, 4100, packet(4100)).gpsSpeedControlActive, false);
});

test('production stopped-follow branch never dispatches propulsion or begins a follow request', () => {
  const source = fs.readFileSync(new URL('./fixtures/legacy-server-control.txt', import.meta.url), 'utf8');
  const loop = source.slice(source.indexOf('function updateUwbFollow('), source.indexOf('function updateTelemetry('));
  const c = new UwbFollowController(config);
  const sandbox = { Date: { now: () => 1000 }, uwbFollowState: { active: false },
    uwbFollowTransitionBusy: false, uwbAutoResume: { pending: false },
    requireUwbFollowStartClearance: () => c.preflight({ now: 1000, telemetry: packet(1000), connected: true }),
    publishUwbFollowState: (r, active) => { assert.equal(active, false); },
    sendUwbRelativeDrive: () => { throw Error('propulsion dispatched while stopped'); },
    maintainUwbAutoResume: () => { throw Error('unexpected follow startup'); },
    idleUwbFollowResult: reason => ({ reason }) };
  vm.runInNewContext(loop + '\nupdateUwbFollow();', sandbox);
});

test('GPS speed feedback configuration rejects incompatible modes and invalid gains', () => {
  const original = JSON.parse(fs.readFileSync(new URL('../config/system.config.json', import.meta.url)));
  assert.deepEqual(validateSystemConfig(original), []);
  for (const mutate of [c => { c.uwb_follow.execution_mode = 'acro'; },
    c => { c.uwb_follow.gps_speed_closed_loop.kp = -1; },
    c => { c.uwb_follow.gps_speed_closed_loop.maximum_observed_speed_mps = 0; }]) {
    const c = structuredClone(original); mutate(c); assert.ok(validateSystemConfig(c).length);
  }
});
