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

test('stops when required sensors, mode, or arming is invalid', () => {
  const now = 10000;
  for (const [expected, expectedFrame, input] of [
    ['pixhawk_offline', 'BODY_RELATIVE', { connected: false, telemetry: telemetry(now) }],
    ['guided_required', 'BODY_NED', { connected: true, telemetry: telemetry(now, { flightMode: 'MANUAL' }) }],
    ['armed_required', 'BODY_NED', { connected: true, telemetry: telemetry(now, { armed: false }) }],
    ['imu_invalid', 'BODY_RELATIVE', { connected: true, telemetry: telemetry(now, { attitude: { yaw: null } }) }],
    ['uwb_stale', 'BODY_RELATIVE', { connected: true, telemetry: telemetry(now, { uwb: { online: false } }) }]
  ]) {
    const result = new UwbFollowController().update({ now, ...input });
    assert.equal(result.safe, false);
    assert.equal(result.reason, expected);
    assert.equal(result.commandFrame, expectedFrame);
    assert.equal(result.forwardSpeedMps, 0);
    assert.equal(result.yawRateRadS, 0);
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
  assert.equal(result.controlMode, 'guided_velocity');
  assert.equal(result.requiredFlightMode, 'GUIDED');
});

test('GPS and EKF failure selects body-relative ACRO following', () => {
  const now = 10000;
  const controller = new UwbFollowController({
    bearing_sign: 1,
    bearing_scale: 1,
    bearing_offset_deg: 0
  });
  const noNavigation = telemetry(now, {
    flightMode: 'ACRO',
    gps: { fixType: 1, updatedAt: now },
    ekf: { healthy: false, updatedAt: now },
    velocity: { vx: null, vy: null, vz: null },
    position: { updatedAt: null },
    uwb: { online: true, fresh: true, distanceM: 3, azimuthDeg: 20, updatedAt: now }
  });

  const preflight = controller.preflight({ now, connected: true, telemetry: noNavigation });
  assert.equal(preflight.safe, true);
  assert.equal(preflight.controlMode, 'relative_acro');
  assert.equal(preflight.requiredFlightMode, 'ACRO');
  assert.equal(preflight.navigationAvailable, false);
  assert.equal(preflight.navigationReason, 'gps_fix_invalid');

  const result = controller.update({ now, connected: true, telemetry: noNavigation });
  assert.equal(result.safe, true);
  assert.equal(result.controlMode, 'relative_acro');
  assert.equal(result.commandFrame, 'BODY_RELATIVE');
  assert.ok(Math.abs(result.relativeBodyM.x - 3 * Math.cos(20 * Math.PI / 180)) < 1e-9);
  assert.ok(Math.abs(result.relativeBodyM.y - 3 * Math.sin(20 * Math.PI / 180)) < 1e-9);
  assert.ok(result.throttlePercent > 0);
  assert.ok(result.steeringDegrees > 0);
});

test('explicit guided velocity mode still rejects invalid navigation', () => {
  const now = 10000;
  const result = new UwbFollowController().preflight({
    now,
    connected: true,
    controlMode: 'guided_velocity',
    telemetry: telemetry(now, { gps: { fixType: 1, updatedAt: now } })
  });
  assert.equal(result.safe, false);
  assert.equal(result.reason, 'gps_fix_invalid');
  assert.equal(result.controlMode, 'guided_velocity');
  assert.equal(result.commandFrame, 'BODY_NED');
});

test('updates the UWB bearing transform when gimbal beacon calibration changes', () => {
  const controller = new UwbFollowController({ bearing_sign: 1, bearing_scale: 1, bearing_offset_deg: 0 });
  const updated = controller.setBearingCalibration({ bearing_sign: -1, bearing_scale: 0.8, bearing_offset_deg: -5 });
  assert.deepEqual(updated, { bearingSign: -1, bearingScale: 0.8, bearingOffsetDeg: -5 });
  assert.equal(controller.config.bearingSign, -1);
  assert.equal(controller.config.bearingScale, 0.8);
  assert.equal(controller.config.bearingOffsetDeg, -5);
});

test('estimates target radial speed and commands body-forward velocity', () => {
  const controller = new UwbFollowController({
    bearing_sign: 1,
    bearing_scale: 1,
    bearing_offset_deg: 0,
    target_speed_alpha: 1,
    bearing_rate_alpha: 1,
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
  assert.equal(second.commandFrame, 'BODY_NED');
  assert.ok(Math.abs(second.rangeRateMps - 1) < 1e-9);
  assert.ok(Math.abs(second.targetRadialSpeedMps - 2) < 1e-9);
  assert.ok(second.forwardSpeedMps > 0);
  assert.ok(Math.abs(second.yawRateRadS) < 1e-12);
  assert.ok(Math.abs(second.velocityNed.y) < 1e-12);
});

test('large bearing error follows a bounded moving arc instead of rotating in place', () => {
  const controller = new UwbFollowController({
    bearing_sign: 1,
    bearing_scale: 1,
    bearing_offset_deg: 0,
    bearing_gain: 1,
    bearing_rate_gain: 0,
    bearing_slowdown_deg: 20,
    bearing_stop_deg: 55,
    max_yaw_rate_deg_s: 35,
    commissioning_mode: false
  });
  const result = controller.update({
    now: 1000,
    connected: true,
    telemetry: telemetry(1000, {
      velocity: { vx: 0, vy: 0, vz: 0 },
      uwb: { online: true, fresh: true, distanceM: 4, azimuthDeg: 70, updatedAt: 1000 }
    })
  });
  assert.equal(result.safe, true);
  assert.ok(result.curvatureMInv > 0 && result.curvatureMInv <= controller.config.maxCurvatureMInv);
  assert.ok(result.curvatureSpeedScale >= controller.config.minimumCurveSpeedScale);
  assert.ok(result.forwardSpeedMps > 0);
  assert.ok(result.throttlePercent > 0);
  assert.ok(result.steeringDegrees > 0);
  assert.ok(Math.abs(result.steeringDegrees) <= Math.abs(result.throttlePercent) * controller.config.sameDirectionSteeringRatio + 1e-9);
  assert.ok(result.yawRateRadS > 0);
});

test('commissioning mode caps forward and reverse body speed', () => {
  const controller = new UwbFollowController({
    bearing_sign: 1,
    bearing_scale: 1,
    bearing_offset_deg: 0,
    commissioning_mode: true,
    commissioning_speed_mps: 0.35,
    max_reverse_speed_mps: 0.2,
    max_speed_mps: 5
  });
  const forward = controller.update({
    now: 1000,
    connected: true,
    telemetry: telemetry(1000, {
      velocity: { vx: 0, vy: 0, vz: 0 },
      uwb: { online: true, fresh: true, distanceM: 20, azimuthDeg: 0, updatedAt: 1000 }
    })
  });
  assert.equal(forward.safe, true);
  assert.ok(forward.forwardSpeedMps <= 0.3500001);
  assert.equal(forward.speedLimitMps, 0.35);

  controller.reset();
  const reverse = controller.update({
    now: 2000,
    connected: true,
    telemetry: telemetry(2000, {
      velocity: { vx: 0, vy: 0, vz: 0 },
      uwb: { online: true, fresh: true, distanceM: 1.25, azimuthDeg: 0, updatedAt: 2000 }
    })
  });
  assert.ok(reverse.forwardSpeedMps >= -0.2000001);
});

test('target uses low-speed approach from 1.2m to 0.5m, then hard-stops with hysteresis', () => {
  const controller = new UwbFollowController({
    bearing_sign: 1,
    bearing_scale: 1,
    bearing_offset_deg: 0,
    safety_distance_m: 0.5,
    safety_resume_distance_m: 1.2,
    hard_stop_enabled: true,
    hard_stop_resume_distance_m: 0.58
  });
  const close = (distanceM, now) => controller.update({
    now,
    connected: true,
    telemetry: telemetry(now, { uwb: { online: true, fresh: true, distanceM, azimuthDeg: 0, updatedAt: now } })
  });
  const approaching = close(1.1, 900);
  assert.equal(approaching.safe, true);
  assert.equal(approaching.reason, 'tracking');
  assert.equal(approaching.closeApproachActive, true);
  assert.equal(approaching.trackingPhase, 'close_approach');
  assert.ok(approaching.desiredForwardSpeedMps > 0);
  assert.ok(approaching.desiredThrottlePercent >= 20);
  assert.ok(approaching.desiredThrottlePercent <= 30);

  const veryClose = close(0.5, 1000);
  assert.equal(veryClose.safe, true);
  assert.equal(veryClose.status, 'holding');
  assert.equal(veryClose.reason, 'holding_distance');
  assert.equal(veryClose.forwardSpeedMps, 0);
  assert.equal(veryClose.yawRateRadS, 0);
  assert.equal(veryClose.throttlePercent, 0);
  assert.equal(veryClose.steeringDegrees, 0);

  const stillClose = close(0.54, 1100);
  assert.equal(stillClose.safe, true);
  assert.equal(stillClose.reason, 'holding_distance');
  assert.deepEqual(stillClose.velocityNed, { x: 0, y: 0 });

  const resumed = close(0.6, 1200);
  assert.equal(resumed.safe, true);
  assert.equal(resumed.reason, 'tracking');
  assert.equal(resumed.closeApproachActive, true);
  assert.equal(resumed.rangeRateMps, 0);

  const normal = close(1.21, 1300);
  assert.equal(normal.safe, true);
  assert.equal(normal.reason, 'tracking');
  assert.equal(normal.closeApproachActive, false);
});

test('three-zone distance hold approaches above 0.55m and reverses below 0.3m until 0.5m', () => {
  const controller = new UwbFollowController({
    bearing_sign: 1,
    bearing_scale: 1,
    bearing_offset_deg: 0,
    safety_distance_m: 0.3,
    safety_resume_distance_m: 1.2,
    hard_stop_enabled: false,
    close_approach_min_speed_mps: 0.12,
    close_approach_max_speed_mps: 0.3,
    close_approach_min_throttle_percent: 20,
    close_approach_max_throttle_percent: 30,
    distance_hold_upper_m: 0.55,
    distance_recovery_resume_m: 0.5
  });
  const update = (distanceM, now) => controller.update({
    now,
    connected: true,
    telemetry: telemetry(now, {
      uwb: { online: true, fresh: true, distanceM, azimuthDeg: 0, updatedAt: now }
    })
  });

  const approaching = update(0.6, 900);
  assert.equal(approaching.trackingPhase, 'close_approach');
  assert.ok(approaching.desiredThrottlePercent > 0);

  const held = update(0.55, 1000);
  assert.equal(held.safe, true);
  assert.equal(held.reason, 'tracking');
  assert.equal(held.closeApproachActive, true);
  assert.equal(held.trackingPhase, 'distance_hold');
  assert.equal(held.desiredForwardSpeedMps, 0);
  assert.equal(held.desiredThrottlePercent, 0);

  const recovering = update(0.29, 1100);
  assert.equal(recovering.safe, true);
  assert.equal(recovering.trackingPhase, 'distance_recovery');
  assert.ok(recovering.desiredForwardSpeedMps < 0);
  assert.equal(recovering.desiredThrottlePercent, -20);

  const recoveryLatched = update(0.4, 1200);
  assert.equal(recoveryLatched.trackingPhase, 'distance_recovery');
  assert.equal(recoveryLatched.desiredThrottlePercent, -20);

  const recovered = update(0.5, 1300);
  assert.equal(recovered.trackingPhase, 'distance_hold');
  assert.equal(recovered.desiredThrottlePercent, 0);
});

test('holding remains safe after the server switches ACRO to MANUAL for true neutral output', () => {
  const now = 10000;
  const controller = new UwbFollowController({ bearing_sign: 1, bearing_scale: 1, bearing_offset_deg: 0 });
  const result = controller.update({
    now,
    connected: true,
    controlMode: 'relative_acro',
    telemetry: telemetry(now, {
      flightMode: 'MANUAL',
      gps: { fixType: 1, updatedAt: now },
      ekf: { healthy: false, updatedAt: now },
      position: { updatedAt: null },
      velocity: { vx: null, vy: null, vz: null },
      uwb: { online: true, fresh: true, distanceM: 0.9, azimuthDeg: 15, updatedAt: now }
    })
  });
  assert.equal(result.safe, true);
  assert.equal(result.status, 'holding');
  assert.equal(result.reason, 'holding_distance');
  assert.equal(result.controlMode, 'relative_acro');
  assert.equal(result.requiredFlightMode, 'ACRO');
  assert.equal(result.throttlePercent, 0);
  assert.equal(result.steeringDegrees, 0);
});

test('small bearing sign changes cannot make steering chatter left and right', () => {
  const controller = new UwbFollowController({
    bearing_sign: 1,
    bearing_scale: 1,
    bearing_offset_deg: 0,
    bearing_gain: 1,
    bearing_rate_gain: 0,
    bearing_filter_alpha: 1,
    bearing_deadband_deg: 4,
    bearing_reverse_confirm_deg: 8,
    bearing_reverse_confirm_ms: 300,
    max_yaw_rate_deg_s: 30,
    relative_steering_limit_deg: 30,
    steering_slew_deg_s: 100,
    steering_return_deg_s: 100
  });
  const step = (azimuthDeg, now) => controller.update({
    now,
    connected: true,
    controlMode: 'relative_acro',
    telemetry: telemetry(now, {
      flightMode: 'ACRO',
      gps: { fixType: 1, updatedAt: now },
      ekf: { healthy: false, updatedAt: now },
      position: { updatedAt: null },
      velocity: { vx: null, vy: null, vz: null },
      uwb: { online: true, fresh: true, distanceM: 3, azimuthDeg, updatedAt: now }
    })
  });

  assert.ok(step(15, 1000).steeringDegrees > 0);
  assert.equal(step(-3, 1100).steeringDegrees, 0);
  assert.equal(step(-6, 1200).steeringDegrees, 0);
  assert.equal(step(-10, 1300).steeringDegrees, 0);
  assert.equal(step(-10, 1500).steeringDegrees, 0);
  assert.ok(step(-10, 1600).steeringDegrees < 0);
});

test('automatic steering obeys its configured slew-rate limit', () => {
  const controller = new UwbFollowController({
    bearing_sign: 1,
    bearing_scale: 1,
    bearing_offset_deg: 0,
    bearing_filter_alpha: 1,
    bearing_deadband_deg: 0,
    bearing_gain: 1,
    bearing_rate_gain: 0,
    max_yaw_rate_deg_s: 20,
    relative_steering_limit_deg: 20,
    steering_slew_deg_s: 18
  });
  const input = (now) => ({
    now,
    connected: true,
    controlMode: 'relative_acro',
    telemetry: telemetry(now, {
      flightMode: 'ACRO',
      gps: { fixType: 1, updatedAt: now },
      ekf: { healthy: false, updatedAt: now },
      position: { updatedAt: null },
      velocity: { vx: null, vy: null, vz: null },
      uwb: { online: true, fresh: true, distanceM: 3, azimuthDeg: 60, updatedAt: now }
    })
  });
  assert.ok(controller.update(input(1000)).steeringDegrees <= 1.800001);
  assert.ok(controller.update(input(1100)).steeringDegrees <= 3.600001);
});

test('ILOS integrates persistent cross-track error while curvature stays bounded', () => {
  const controller = new UwbFollowController({
    bearing_sign: 1,
    bearing_scale: 1,
    bearing_offset_deg: 0,
    bearing_filter_alpha: 1,
    bearing_deadband_deg: 0,
    ilos_integral_gain: 0.5,
    ilos_integral_decay: 0,
    ilos_max_correction_deg: 15,
    lookahead_base_m: 1,
    lookahead_speed_gain_s: 0,
    lookahead_min_m: 1,
    lookahead_max_m: 1,
    max_curvature_m_inv: 0.4
  });
  const step = (now) => controller.update({
    now,
    connected: true,
    controlMode: 'relative_acro',
    telemetry: telemetry(now, {
      flightMode: 'ACRO',
      gps: { fixType: 1, updatedAt: now },
      ekf: { healthy: false, updatedAt: now },
      position: { updatedAt: null },
      velocity: { vx: null, vy: null, vz: null },
      uwb: { online: true, fresh: true, distanceM: 4, azimuthDeg: 12, updatedAt: now }
    })
  });
  const first = step(1000);
  const second = step(1100);
  const third = step(1200);
  assert.equal(first.ilosCorrectionDeg, 0);
  assert.ok(second.ilosCorrectionDeg > 0);
  assert.ok(third.ilosCorrectionDeg >= second.ilosCorrectionDeg);
  assert.ok(Math.abs(third.curvatureMInv) <= 0.4000001);
});

test('aggressive profile clears the ESC deadband outside the near-target zone', () => {
  const controller = new UwbFollowController({
    bearing_sign: 1,
    bearing_scale: 1,
    bearing_offset_deg: 0,
    commissioning_mode: true,
    commissioning_speed_mps: 2,
    range_gain: 1.2,
    relative_throttle_limit_percent: 70,
    relative_min_effective_throttle_percent: 35,
    minimum_throttle_range_error_m: 0.4,
    relative_throttle_accel_percent_s: 100
  });
  const input = (now) => ({
    now,
    connected: true,
    controlMode: 'relative_acro',
    telemetry: telemetry(now, {
      flightMode: 'ACRO',
      gps: { fixType: 1, updatedAt: now },
      ekf: { healthy: false, updatedAt: now },
      position: { updatedAt: null },
      velocity: { vx: null, vy: null, vz: null },
      uwb: { online: true, fresh: true, distanceM: 2, azimuthDeg: 0, updatedAt: now }
    })
  });
  controller.update(input(1000));
  controller.update(input(1100));
  const result = controller.update(input(1200));
  assert.equal(result.minimumEffectiveThrottleApplied, true);
  assert.equal(result.desiredThrottlePercent, 35);
  assert.ok(result.throttlePercent > result.proportionalThrottlePercent);
});

test('preflight permits starting while target is inside the neutral hold zone', () => {
  const now = 10000;
  const result = new UwbFollowController().preflight({
    now,
    connected: true,
    telemetry: telemetry(now, {
      uwb: { online: true, fresh: true, distanceM: 0.5, azimuthDeg: 0, updatedAt: now }
    })
  });
  assert.equal(result.safe, true);
  assert.equal(result.distanceM, 0.5);
});
