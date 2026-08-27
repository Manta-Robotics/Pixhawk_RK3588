import test from 'node:test';
import assert from 'node:assert/strict';

import { UwbFollowController } from '../backend/uwb_follow.js';

function telemetry(now, overrides = {}) {
  return {
    flightMode: 'GUIDED',
    armed: true,
    attitude: { yaw: 0 },
    velocity: { vx: 1, vy: 0, vz: 0 },
    position: { updatedAt: now },
    ekf: { healthy: true, flags: 18, updatedAt: now },
    gps: { fixType: 3, updatedAt: now },
    uwb: { online: true, fresh: true, distanceM: 3, azimuthDeg: 0, updatedAt: now },
    ...overrides
  };
}

test('stops when GPS, UWB, mode, arming, or safety distance is invalid', () => {
  const now = 10000;
  for (const [expected, input] of [
    ['pixhawk_offline', { connected: false, telemetry: telemetry(now) }],
    ['guided_required', { connected: true, telemetry: telemetry(now, { flightMode: 'MANUAL' }) }],
    ['armed_required', { connected: true, telemetry: telemetry(now, { armed: false }) }],
    ['gps_fix_invalid', { connected: true, telemetry: telemetry(now, { gps: { fixType: 1 } }) }],
    ['ekf_unhealthy', { connected: true, telemetry: telemetry(now, { ekf: { healthy: false, updatedAt: now } }) }],
    ['uwb_stale', { connected: true, telemetry: telemetry(now, { uwb: { online: false } }) }],
    ['safety_distance', { connected: true, telemetry: telemetry(now, { uwb: { online: true, fresh: true, distanceM: 1, azimuthDeg: 0, updatedAt: now } }) }]
  ]) {
    const result = new UwbFollowController().update({ now, ...input });
    assert.equal(result.safe, false);
    assert.equal(result.reason, expected);
    assert.deepEqual(result.velocityNed, { x: 0, y: 0 });
  }
});

test('sensor preflight allows a disarmed MANUAL boat before automatic mode change and arming', () => {
  const now = 10000;
  const controller = new UwbFollowController();
  const result = controller.preflight({
    now,
    connected: true,
    telemetry: telemetry(now, { flightMode: 'MANUAL', armed: false })
  });
  assert.equal(result.safe, true);
  assert.equal(result.reason, 'preflight_ok');
  assert.equal(result.distanceM, 3);
});

test('updates the UWB bearing transform when gimbal beacon calibration changes', () => {
  const controller = new UwbFollowController({ bearing_sign: 1, bearing_scale: 1, bearing_offset_deg: 0 });
  const updated = controller.setBearingCalibration({ bearing_sign: -1, bearing_scale: 0.8, bearing_offset_deg: -5 });
  assert.deepEqual(updated, { bearingSign: -1, bearingScale: 0.8, bearingOffsetDeg: -5 });
  assert.equal(controller.config.bearingSign, -1);
  assert.equal(controller.config.bearingScale, 0.8);
  assert.equal(controller.config.bearingOffsetDeg, -5);
});

test('estimates target NED velocity from relative motion plus EKF boat velocity', () => {
  const controller = new UwbFollowController({
    bearing_sign: 1,
    bearing_scale: 1,
    bearing_offset_deg: 0,
    velocity_alpha: 1,
    direction_alpha: 1,
    commissioning_mode: false
  });
  controller.update({ now: 1000, connected: true, telemetry: telemetry(1000) });
  const second = controller.update({
    now: 1100,
    connected: true,
    telemetry: telemetry(1100, {
      uwb: { online: true, fresh: true, distanceM: 3.1, azimuthDeg: 0, updatedAt: 1100 }
    })
  });
  assert.equal(second.safe, true);
  assert.ok(Math.abs(second.targetVelocityNed.x - 2) < 1e-9);
  assert.ok(Math.abs(second.targetVelocityNed.y) < 1e-9);
  assert.ok(second.velocityNed.x > 0);
});

test('commissioning mode caps speed below the configured hard maximum', () => {
  const controller = new UwbFollowController({
    bearing_sign: 1,
    bearing_scale: 1,
    bearing_offset_deg: 0,
    commissioning_mode: true,
    commissioning_speed_mps: 0.8,
    max_speed_mps: 5
  });
  const result = controller.update({
    now: 1000,
    connected: true,
    telemetry: telemetry(1000, {
      velocity: { vx: 0, vy: 0, vz: 0 },
      uwb: { online: true, fresh: true, distanceM: 20, azimuthDeg: 0, updatedAt: 1000 }
    })
  });
  assert.equal(result.safe, true);
  assert.ok(result.desiredSpeedMps <= 0.8000001);
  assert.equal(result.speedLimitMps, 0.8);
});

test('safety latch needs extra clearance before resuming', () => {
  const controller = new UwbFollowController({ bearing_sign: 1, bearing_scale: 1, bearing_offset_deg: 0 });
  const close = (distanceM, now) => controller.update({
    now,
    connected: true,
    telemetry: telemetry(now, { uwb: { online: true, fresh: true, distanceM, azimuthDeg: 0, updatedAt: now } })
  });
  assert.equal(close(0.9, 1000).reason, 'safety_distance');
  assert.equal(close(1.1, 1100).reason, 'safety_distance');
  assert.equal(close(1.21, 1200).safe, true);
});
