import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { resolve } from 'node:path';
import { test } from 'node:test';
import { fileURLToPath } from 'node:url';

import {UwbPathFollowController,PATH_FOLLOW_DEFAULTS} from '../backend/uwb_path_follow.js';

const RAD = Math.PI / 180;
const EARTH_M = 6378137;
const origin = { lat: 30, lon: 120 };
const near = (actual, expected, tolerance = 1e-6) =>
  assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} != ${expected}`);

function controller(extra = {}) {
  return new UwbPathFollowController({
    followingDistanceM: 2,
    ...extra,
    pathFollow: { enabled: true, ...(extra.pathFollow || {}) },
  });
}

function input(now = 10000, { boatX = 0, boatY = 0, targetX = 12, targetY = 0,
  vx = 0, vy = 0, yaw = 0 } = {}) {
  const dx = targetX - boatX, dy = targetY - boatY;
  return {
    now,
    distanceM: Math.hypot(dx, dy),
    bearingDeg: Math.atan2(dy, dx) / RAD - yaw,
    telemetry: {
      gps: { fixType: 3, updatedAt: now },
      ekf: { healthy: true, updatedAt: now },
      position: { lat: origin.lat + boatX / (RAD * EARTH_M),
        lon: origin.lon + boatY / (RAD * EARTH_M * Math.cos(origin.lat * RAD)), updatedAt: now },
      attitude: { yaw, yawRateDegS: 0, updatedAt: now },
      velocity: { vx, vy, updatedAt: now },
      uwb: { online: true, fresh: true, measurementUpdatedAt: now,
        tagId: 'swimmer', measurementStatus: 'trusted',
        sensors: [{ receiverId: 1, tagId: 'swimmer', valid: true }] },
    },
  };
}

// Seed a previously acquired path so terminal control can be tested without
// conflating it with GPS acquisition, filtering or UWB position alignment.
function seeded(extra = {}) {
  const c = controller(extra);
  c.origin = { ...origin };
  c.goodSince = 1;
  c.points = [0, 4, 8, 12].map((x, key) => ({ x, y: 0, key, at: 10000 }));
  c.nextKey = c.points.length;
  c.rebuildCurve();
  c.progressKey = c.pointAt(9).key;
  c.targetSpeed = 1;
  c.targetId = 'swimmer';
  c.lastObservation = { x: 12, y: 0, at: 10000 };
  c.filtered = { x: 12, y: 0 };
  return c;
}

test('default remains disabled and bounds remain compatible', () => {
  assert.equal(PATH_FOLLOW_DEFAULTS.enabled, false);
  assert.equal(new UwbPathFollowController().update().reason, 'disabled');
  const c = controller({ pathFollow: { min_points: -1, max_points: 99999,
    lookahead_min_m: 5, lookahead_max_m: 1 } });
  assert.equal(c.options.min_points, 3);
  assert.equal(c.options.max_points, 1024);
  assert.equal(c.options.lookahead_max_m, 5);
});

test('lookahead stops at the following-distance boundary', () => {
  const c = seeded();
  const result = c.update(input(10000, { boatX: 9, vx: 2 }));
  assert.equal(result.active, true);
  near(result.lookaheadPointNedM.x, 10);
  near(result.remainingM, 1);
});

test('no reverse rejoin is requested when already inside the following distance', () => {
  const c = seeded({ pathFollow: { lookahead_min_m: 5, lookahead_max_m: 5 } });
  c.progressKey = c.pointAt(12).key;
  c.lastObservation.x = 13; // Raw target can lead the smoothed breadcrumb tail.
  const result = c.update(input(10000, { boatX: 12.2, targetX: 13, yaw: 0 }));
  assert.equal(result.active, true);
  near(result.desiredSpeedMps, 0);
  near(result.yawRateDegS, 0);
});

test('lateral error does not restore speed within the following distance', () => {
  const c = seeded();
  c.progressKey = c.pointAt(11).key;
  const sample = input(10000, { boatX: 11, boatY: 0.2 });
  sample.telemetry.attitude.yawRateDegS = 4;
  const result = c.update(sample);
  assert.equal(result.active, true);
  near(result.remainingM, 0);
  near(result.desiredSpeedMps, 0);
  near(result.yawRateDegS, 0);
});

test('a path behind the boat still requires rejoining when travel is needed', () => {
  const c = seeded();
  c.lastObservation.x = 8;
  const result = c.update(input(10000, { boatX: 9, targetX: 8, yaw: 180 }));
  assert.equal(result.active, false);
  assert.equal(result.reason, 'path_rejoin_required');
});

test('short path still allows joining when the boat is far behind its start', () => {
  const c = seeded();
  c.points = [8, 8.3, 8.6, 9].map((x, key) => ({ x, y: 0, key, at: 10000 }));
  c.rebuildCurve();
  c.progressKey = c.curve[0].key;
  const result = c.update(input(10000, { targetX: 9 }));
  assert.equal(result.active, true);
  assert.ok(result.desiredSpeedMps > 0);
  near(result.lookaheadPointNedM.x, 8);
});

test('repeating a timestamp cannot advance established progress', () => {
  const c = seeded();
  c.progressKey = c.pointAt(1).key;
  const before = c.progressKey;
  for (let i = 0; i < 10; i++) c.locate({ x: 8, y: 0 }, 2, 0);
  near(c.progressKey, before);
});

test('positive elapsed time allows forward progress but never reverses it', () => {
  const c = seeded();
  c.progressKey = c.pointAt(1).key;
  const before = c.progressKey;
  c.locate({ x: 2, y: 0 }, 1, 0.1);
  assert.ok(c.progressKey > before);
  const advanced = c.progressKey;
  c.locate({ x: 0, y: 0 }, 1, 0.1);
  near(c.progressKey, advanced);
});

test('self intersection does not select a later lap', () => {
  const c = controller();
  c.points = [[0, 0], [4, 0], [4, 4], [0, 4], [0, 0], [4, 0]]
    .map(([x, y], key) => ({ x, y, key, at: 10000 }));
  c.rebuildCurve();
  c.progressKey = c.pointAt(0.5).key;
  const located = c.locate({ x: 0.8, y: 0 }, 1, 0.1);
  assert.ok(located.s < 2);
});

test('reject stale velocity timestamps when supplied', () => {
  const c = controller();
  const sample = input();
  sample.telemetry.velocity.updatedAt = sample.now - 701;
  assert.equal(c.update(sample).reason, 'velocity_stale');
});

test('legacy velocity without a timestamp still works', () => {
  const c = controller();
  const sample = input();
  delete sample.telemetry.velocity.updatedAt;
  assert.equal(c.update(sample).reason, 'gps_confirming');
});

test('GPS acquisition and real observations produce finite bounded references', () => {
  const c = controller();
  let result;
  for (let now = 10000; now <= 14000; now += 100) {
    result = c.update(input(now, { targetX: 4 + (now - 10000) / 1000 }));
  }
  assert.equal(result.active, true);
  for (const key of ['desiredSpeedMps', 'headingErrorDeg', 'yawRateDegS', 'curvatureMInv',
    'lookaheadM', 'remainingM', 'progressM', 'crossTrackErrorM']) {
    assert.ok(Number.isFinite(result[key]), key);
  }
  assert.ok(result.desiredSpeedMps >= 0 && result.desiredSpeedMps <= 2);
  assert.ok(Math.abs(result.yawRateDegS) <= 14);
});

test('continuous straight-line following settles at the configured distance after target stops', () => {
  const c = controller();
  let boatX = 0, vx = 0, result, activeCount = 0;
  for (let now = 10000; now <= 50000; now += 100) {
    const targetX = 4 + 0.7 * Math.min((now - 10000) / 1000, 10);
    result = c.update(input(now, { boatX, targetX, vx }));
    if (result.active) {
      activeCount++;
      assert.ok(Number.isFinite(result.desiredSpeedMps));
      near(result.yawRateDegS, 0);
      vx = result.desiredSpeedMps;
    } else {
      assert.ok(['gps_confirming', 'path_warming_up'].includes(result.reason), result.reason);
      vx = 0;
    }
    boatX += vx * 0.1;
    assert.ok(boatX <= targetX - 1.95, `boat overran following distance at ${now}`);
  }
  assert.ok(activeCount > 300);
  assert.ok(11 - boatX >= 1.95 && 11 - boatX <= 2.3, `final separation ${11 - boatX}`);
  assert.ok(result.desiredSpeedMps < 0.01);
});

test('navigation and UWB faults clear existing path', () => {
  for (const [reason, mutate] of [
    ['gps_fix_invalid', sample => { sample.telemetry.gps.fixType = 2; }],
    ['gps_stale', sample => { sample.telemetry.gps.updatedAt -= 701; }],
    ['ekf_unavailable', sample => { sample.telemetry.ekf.healthy = false; }],
    ['imu_invalid', sample => { sample.telemetry.attitude.yaw = NaN; }],
    ['velocity_invalid', sample => { sample.telemetry.velocity.vx = Infinity; }],
    ['observation_unavailable', sample => { sample.predicted = true; }],
    ['observation_unavailable', sample => { sample.telemetry.uwb.sensors[0].occlusionDiagnostics = {state:"quarantined"}; }],
    ['target_identity_changed', sample => { sample.telemetry.uwb.tagId = 'other'; }],
    ['observation_clock_gap', sample => { sample.telemetry.uwb.measurementUpdatedAt -= 100; }],
  ]) {
    const c = seeded();
    const sample = input();
    mutate(sample);
    assert.equal(c.update(sample).reason, reason);
    assert.equal(c.points.length, 0);
  }
});

test('clock gaps and GPS jumps reset the path', () => {
  const c = seeded();
  c.update(input());
  assert.equal(c.update(input(11000)).reason, 'clock_gap');
  assert.equal(c.points.length, 0);
  const d = seeded();
  d.update(input());
  assert.equal(d.update(input(10100, { boatX: 10 })).reason, 'gps_position_jump');
});

test('target position jumps are rejected', () => {
  const c = seeded();
  assert.equal(c.update(input(10100, { targetX: 30 })).reason, 'target_position_jump');
  assert.equal(c.points.length, 0);
});

test('untraversed path cannot be trimmed when capacity is exhausted', () => {
  const c = seeded({ pathFollow: { max_points: 16 } });
  c.points = Array.from({ length: 16 }, (_, key) => ({ x: key, y: 0, at: 10000, key }));
  c.nextKey = 16;
  c.rebuildCurve();
  c.progressKey = c.curve[0].key;
  c.lastObservation = { x: 15, y: 0, at: 10000 };
  c.filtered = { x: 15, y: 0 };
  assert.equal(c.update(input(10400, { targetX: 16 })).reason, 'path_buffer_exhausted');
  assert.equal(c.points.length, 0);
});
