import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import { UwbSideFollowController } from '../backend/uwb_side_follow.js';
import { validateSystemConfig } from '../scripts/validate_config.mjs';

const RAD = Math.PI / 180;
const config = { execution_mode: 'relative_imu', bearing_sign: 1, bearing_scale: 1, bearing_offset_deg: 0,
  safety_distance_m: 0.3, following_distance_m: 0.5, max_speed_mps: 2, commissioning_mode: false,
  relative_throttle_limit_percent: 70, relative_steering_limit_deg: 18, max_yaw_rate_deg_s: 14,
  side_receiver_fov_half_deg: 110, gps_speed_closed_loop: { enabled: true, acquire_ms: 1000 },
  side_follow: { distance_m: 2 }, prediction: { measurement_noise_m: 0.05 } };

function packet(at, { boat = { x: 0, y: 0 }, target = { x: 4, y: 0 }, vx = 0, vy = 0, yaw = 0, rate = 0 } = {}) {
  const x = target.x - boat.x, y = target.y - boat.y;
  return { armed: true, flightMode: 'MANUAL', pilotSteering: { type: 0, ready: true },
    gps: { fixType: 3, updatedAt: at }, position: { updatedAt: at }, ekf: { healthy: true, updatedAt: at },
    velocity: { vx, vy, updatedAt: at }, attitude: { yaw, yawRateDegS: rate, updatedAt: at },
    uwb: { online: true, fresh: true, distanceM: Math.hypot(x, y),
      azimuthDeg: Math.atan2(y, x) / RAD - yaw, measurementUpdatedAt: at,
      measurementStatus: 'trusted', activeReceiverIds: [1], sensors: [{ receiverId: 1, tagId: 42, valid: true }] } };
}
function run(mode = 'left', heading = 0) {
  const c = new UwbSideFollowController(config); c.setMode(mode);
  let result;
  for (let i = 0; i <= 40; i++) {
    const at = 10000 + i * 100, s = i * 0.04;
    const t = packet(at, { target: { x: (4 + s) * Math.cos(heading), y: (4 + s) * Math.sin(heading) }, yaw: heading / RAD });
    result = c.update({ now: at, telemetry: t, connected: true });
  }
  return { c, result };
}

test('left/right reference offsets and steering mirror in the target ground-track frame', () => {
  const left = run('left').result, right = run('right').result;
  assert.equal(left.reason, 'side_tracking'); assert.equal(right.reason, 'side_tracking');
  assert.ok(Math.abs(left.desiredOffsetNedM.x) < 1e-8);
  assert.equal(left.desiredOffsetNedM.y, -2); assert.equal(right.desiredOffsetNedM.y, 2);
  assert.ok(left.steeringDegrees < 0); assert.ok(right.steeringDegrees > 0);
  assert.ok(Math.abs(left.desiredSpeedMps - right.desiredSpeedMps) < 1e-10);
  const east = run('left', Math.PI / 2).result;
  assert.ok(Math.abs(east.desiredOffsetNedM.x - 2) < 1e-8);
  assert.ok(Math.abs(east.desiredOffsetNedM.y) < 1e-8);
});

test('own translation is compensated and a rotating boat does not invent target motion', () => {
  const c = new UwbSideFollowController(config);
  let result;
  for (let i = 0; i <= 30; i++) {
    const at = 10000 + i * 100;
    result = c.update({ now: at, connected: true, telemetry: packet(at, {
      boat: { x: i * 0.04, y: 0 }, target: { x: 4 + i * 0.04, y: 0 }, vx: 0.4, yaw: i * 0.5, rate: 5,
    }) });
  }
  assert.ok(Math.abs(c.targetVelocity.x - 0.4) < 1e-8);
  assert.ok(Math.abs(c.targetVelocity.y) < 1e-8);
  assert.ok(Math.abs(result.targetHeadingDeg) < 1e-8);
});

test('a target that stops retains its established direction without sustained speed feed-forward', () => {
  const { c } = run(); const heading = c.targetHeadingDeg;
  for (let i = 1; i <= 30; i++) c.update({ now: 14000 + i * 100, connected: true,
    telemetry: packet(14000 + i * 100, { target: { x: 5.6, y: 0 } }) });
  assert.equal(c.targetHeadingDeg, heading); assert.deepEqual(c.targetVelocity, { x: 0, y: 0 });
  const unknown = new UwbSideFollowController(config);
  for (let i = 0; i < 40; i++) {
    const r = unknown.update({ now: 10000 + i * 100, connected: true, telemetry: packet(10000 + i * 100) });
    assert.equal(r.reason, 'side_direction_unavailable'); assert.equal(r.throttlePercent, 0);
  }
});

test('repeated and replayed observations do not acquire or retain a false direction', () => {
  const c = new UwbSideFollowController(config), t = packet(10000);
  for (let now = 10000; now <= 10400; now += 100) {
    t.attitude.updatedAt = now; t.gps.updatedAt = now; t.position.updatedAt = now;
    t.velocity.updatedAt = now; t.ekf.updatedAt = now;
    assert.equal(c.update({ now, connected: true, telemetry: t }).throttlePercent, 0);
  }
  assert.equal(c.history.length, 1); assert.equal(c.targetHeadingDeg, null);
  t.uwb.distanceM += 0.1;
  assert.equal(c.update({ now: 10400, connected: true, telemetry: t }).reason, 'side_direction_unavailable');
  assert.equal(c.history.length, 1);
});

test('GPS loss, stale velocity, bad attitude, UWB loss, quarantine and close range neutralize outputs', () => {
  for (const change of [t => { t.gps.fixType = 0; }, t => { t.velocity.updatedAt = 1; },
    t => { t.ekf.healthy = false; }, t => { t.attitude.updatedAt = 1; },
    t => { t.uwb.fresh = false; }, t => { t.uwb.measurementStatus = 'suspect'; },
    t => { t.uwb.sensors[0].occlusionDiagnostics = { state: 'quarantined' }; },
    t => { Object.assign(t.uwb.sensors[0], { rawDistanceM: 0.2, rawDistanceUpdatedAt: 14100 }); },
    t => { t.uwb.distanceM = 0.31; }, t => { t.uwb.azimuthDeg = 120; }]) {
    const { c } = run(); const t = packet(14100, { target: { x: 5.64, y: 0 } }); change(t);
    const r = c.update({ now: 14100, connected: true, telemetry: t });
    assert.equal(r.throttlePercent, 0, r.reason); assert.equal(r.steeringDegrees, 0, r.reason);
    assert.equal(r.pausePropulsion, true);
  }
  const narrow = new UwbSideFollowController({ ...config, side_receiver_fov_half_deg: 70 });
  assert.equal(narrow.preflight({ now: 10000, telemetry: packet(10000), connected: true }).reason, 'side_receiver_coverage_required');
});

test('target changes stop the session; measurement clock changes restart direction acquisition', () => {
  const { c } = run(), changed = packet(14100, { target: { x: 5.64, y: 0 } });
  changed.uwb.sensors[0].tagId = 43;
  assert.equal(c.update({ now: 14100, connected: true, telemetry: changed }).reason, 'target_identity_changed');
  assert.equal(c.history.length, 0);
  const { c: replay } = run(), t = packet(14100, { target: { x: 5.64, y: 0 } });
  t.uwb.measurementUpdatedAt = 13900;
  const r = replay.update({ now: 14100, connected: true, telemetry: t });
  assert.equal(r.throttlePercent, 0); assert.equal(replay.history.length, 1); assert.equal(replay.targetHeadingDeg, null);
});

test('invalid side modes, distances and configuration are rejected', () => {
  const c = new UwbSideFollowController(config);
  assert.throws(() => c.setMode('orbit'), /mode/);
  for (const d of [NaN, null, '2', -1, 0.2, 25]) assert.throws(() => c.setMode('left', d), /distance/);
  const system = JSON.parse(fs.readFileSync(new URL('../config/system.config.json', import.meta.url)));
  for (const patch of [{ distance_m: -1 }, { position_gain: null }, { motion_confirm_ms: 2000, motion_window_ms: 1000 }]) {
    const copy = structuredClone(system); Object.assign(copy.uwb_follow.side_follow, patch);
    assert.ok(validateSystemConfig(copy).some(error => error.includes('side_follow')));
  }
});

test('closed-loop left/right followers converge behind a moving target with finite surge and yaw response', () => {
  for (const mode of ['left', 'right']) {
    const c = new UwbSideFollowController(config); c.setMode(mode);
    let x = 0, y = 0, yaw = 0, speed = 0, turn = 0, minimumRange = Infinity;
    const tail = [];
    for (let i = 0; i < 1800; i++) {
      const at = 10000 + 100 * i, target = { x: 4 + 0.04 * i, y: 0 };
      const r = c.update({ now: at, connected: true, telemetry: packet(at, {
        boat: { x, y }, target, vx: speed * Math.cos(yaw * RAD), vy: speed * Math.sin(yaw * RAD), yaw, rate: turn,
      }) });
      const steering = clamp(r.steeringDegrees, -0.36 * r.throttlePercent, 0.36 * r.throttlePercent);
      c.trackApplied(r.throttlePercent, steering);
      speed += clamp(r.throttlePercent / 70 * 2 - speed, -0.03, 0.03);
      turn += clamp(steering / 18 * 14 - turn, -2, 2);
      yaw += turn * 0.1; x += speed * Math.cos(yaw * RAD) * 0.1; y += speed * Math.sin(yaw * RAD) * 0.1;
      minimumRange = Math.min(minimumRange, Math.hypot(target.x - x, target.y - y));
      if (i > 1600) tail.push(Math.hypot(x - target.x, y - (mode === 'left' ? -2 : 2)));
    }
    assert.ok(minimumRange > 0.35, `${mode}: minimum separation ${minimumRange}`);
    assert.ok(tail.reduce((a, b) => a + b, 0) / tail.length < 0.5,
      `${mode}: final error ${tail.at(-1)}, boat ${x},${y}, yaw ${yaw}`);
  }
});
function clamp(value, low, high) { return Math.max(low, Math.min(high, value)); }

test('live and preview transports preserve selected side and distance through start/stop', async () => {
  const source = fs.readFileSync(new URL('../frontend/js/manta-app-transport.js', import.meta.url), 'utf8');
  const calls = [], scope = { window: {}, console, Date, performance, AbortController,
    setTimeout, clearTimeout, setInterval, clearInterval,
    fetch: async (url, options) => { calls.push({ url, options }); return { ok: true, json: async () => ({ success: true, state: { followMode: 'left' } }) }; } };
  vm.runInNewContext(source, scope);
  const live = scope.window.MantaAppTransport.create('live');
  await live.setUwbFollowMode('left', 3);
  assert.equal(calls[0].url, '/api/uwb-follow/mode');
  assert.deepEqual(JSON.parse(calls[0].options.body), { mode: 'left', distanceM: 3 });
  assert.equal(calls[0].options.credentials, 'same-origin');
  const mock = scope.window.MantaAppTransport.create('mock');
  await mock.setUwbFollowMode('right', 2.5);
  assert.equal((await mock.startUwbFollow()).state.followMode, 'right');
  assert.equal((await mock.stopUwbFollow()).state.sideDistanceM, 2.5);
});
