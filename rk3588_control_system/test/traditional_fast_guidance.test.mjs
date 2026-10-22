import test from 'node:test';
import assert from 'node:assert/strict';

import { TraditionalFastGuidance } from '../backend/traditional_fast_guidance.js';
import { UwbFollowController } from '../backend/uwb_follow.js';

test('traditional fast guidance returns the original speed and heading-rate interface', () => {
  const controller = new TraditionalFastGuidance({
    heading_gain: 1.2,
    yaw_rate_damping: 0.35,
    minimum_turn_speed_scale: 0.15
  });
  const still = controller.command({
    headingErrorDeg: 60,
    measuredYawRateDegS: 0,
    forwardSpeedMps: 1,
    maximumYawRateDegS: 35
  });
  const alreadyTurning = controller.command({
    headingErrorDeg: 60,
    measuredYawRateDegS: 30,
    forwardSpeedMps: 1,
    maximumYawRateDegS: 35
  });

  assert.ok(Math.abs(still.speedScale - 0.5) < 1e-9);
  assert.ok(Math.abs(still.forwardSpeedMps - 0.5) < 1e-9);
  assert.equal(still.desiredYawRateDegS, 35);
  assert.ok(alreadyTurning.rawYawRateDegS < still.rawYawRateDegS);
});

test('UWB traditional fast profile preserves speed and steering-angle outputs', () => {
  const now = 1000;
  const controller = new UwbFollowController({
    controller_profile: 'traditional_fast',
    bearing_sign: 1,
    bearing_scale: 1,
    bearing_offset_deg: 0,
    bearing_filter_alpha: 1,
    commissioning_mode: true,
    commissioning_speed_mps: 2,
    relative_throttle_limit_percent: 70,
    relative_steering_limit_deg: 25
  });
  const result = controller.update({
    now,
    connected: true,
    controlMode: 'relative_acro',
    telemetry: {
      flightMode: 'ACRO',
      armed: true,
      attitude: { yaw: 0, yawRateDegS: 12 },
      velocity: { vx: null, vy: null, vz: null },
      position: { updatedAt: null },
      ekf: { healthy: false, updatedAt: now },
      gps: { fixType: 1, updatedAt: now },
      uwb: { online: true, fresh: true, distanceM: 3, azimuthDeg: 30, updatedAt: now }
    }
  });

  assert.equal(result.safe, true);
  assert.equal(result.controllerProfile, 'traditional_fast');
  assert.equal(result.measuredYawRateDegS, 12);
  assert.equal(typeof result.forwardSpeedMps, 'number');
  assert.equal(typeof result.steeringDegrees, 'number');
  assert.equal(result.leftThrust, undefined);
  assert.equal(result.rightThrust, undefined);
  assert.ok(result.forwardSpeedMps > 0);
  assert.ok(result.steeringDegrees > 0);
});
