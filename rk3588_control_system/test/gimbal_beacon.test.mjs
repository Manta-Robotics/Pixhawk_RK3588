import assert from 'node:assert/strict';
import test from 'node:test';
import { computeBeaconControl, normalizeAngleDeg } from '../backend/gimbal_beacon.js';

const now = 10_000;
const freshUwb = {
  online: true,
  fresh: true,
  azimuthDeg: 20,
  elevationDeg: 10,
  distanceM: 2,
  updatedAt: now - 20
};
const freshFeedback = {
  checksumValid: true,
  yawDeg: 5,
  pitchDeg: 2,
  updatedAt: now - 10
};

test('normalizes wrap-around errors to the shortest turn', () => {
  assert.equal(normalizeAngleDeg(181), -179);
  assert.equal(normalizeAngleDeg(-181), 179);
  const result = computeBeaconControl({
    now,
    uwb: { ...freshUwb, azimuthDeg: -179 },
    feedback: { ...freshFeedback, yawDeg: 179 },
    config: { yaw_deadband_deg: 0, yaw_gain: 1 }
  });
  assert.equal(result.yawErrorDeg, 2);
  assert.equal(result.rateX, 2);
});

test('applies source signs, offsets, deadbands and rate limits', () => {
  const result = computeBeaconControl({
    now,
    uwb: freshUwb,
    feedback: freshFeedback,
    config: {
      yaw_source_sign: -1,
      pitch_source_sign: -1,
      yaw_offset_deg: 3,
      pitch_offset_deg: 4,
      yaw_deadband_deg: 1,
      pitch_deadband_deg: 2,
      yaw_gain: 2,
      pitch_gain: 3,
      max_yaw_rate_dps: 12,
      max_pitch_rate_dps: 15
    }
  });
  assert.equal(result.targetYawDeg, -17);
  assert.equal(result.targetPitchDeg, -6);
  assert.equal(result.yawErrorDeg, -22);
  assert.equal(result.pitchErrorDeg, -8);
  assert.equal(result.rateX, -12);
  assert.equal(result.rateY, -15);
});

test('applies independently calibrated UWB angle scales', () => {
  const result = computeBeaconControl({
    now,
    uwb: { ...freshUwb, azimuthDeg: -42, elevationDeg: -34 },
    feedback: { ...freshFeedback, yawDeg: 24, pitchDeg: 14.3 },
    config: {
      yaw_source_sign: -1,
      pitch_source_sign: -1,
      yaw_source_scale: 0.713,
      pitch_source_scale: 0.768,
      yaw_offset_deg: -5.87,
      pitch_offset_deg: -11.776,
      yaw_deadband_deg: 0,
      pitch_deadband_deg: 0
    }
  });
  assert.ok(Math.abs(result.targetYawDeg - 24.076) < 0.001);
  assert.ok(Math.abs(result.targetPitchDeg - 14.336) < 0.001);
});

test('command signs can be calibrated independently from UWB source signs', () => {
  const result = computeBeaconControl({
    now,
    uwb: freshUwb,
    feedback: freshFeedback,
    config: { yaw_deadband_deg: 0, pitch_deadband_deg: 0, yaw_command_sign: -1, pitch_command_sign: -1 }
  });
  assert.ok(result.rateX < 0);
  assert.ok(result.rateY < 0);
});

test('gates stale UWB or stale gimbal feedback', () => {
  const staleUwb = computeBeaconControl({
    now,
    uwb: { ...freshUwb, updatedAt: now - 900 },
    feedback: freshFeedback,
    config: { timeout_ms: 700 }
  });
  assert.equal(staleUwb.valid, false);
  assert.equal(staleUwb.reason, 'uwb_stale');

  const staleFeedback = computeBeaconControl({
    now,
    uwb: freshUwb,
    feedback: { ...freshFeedback, updatedAt: now - 500 },
    config: { feedback_timeout_ms: 350 }
  });
  assert.equal(staleFeedback.valid, false);
  assert.equal(staleFeedback.reason, 'gimbal_feedback_stale');
});

test('absolute-angle Beacon control does not require gimbal feedback', () => {
  const result = computeBeaconControl({
    now,
    uwb: freshUwb,
    feedback: {},
    config: {
      control_mode: 'absolute_angle',
      yaw_source_sign: -1,
      pitch_source_sign: -1,
      yaw_source_scale: 0.8,
      pitch_source_scale: 0.5,
      yaw_offset_deg: -4,
      pitch_offset_deg: -5
    }
  });
  assert.equal(result.valid, true);
  assert.equal(result.controlMode, 'absolute_angle');
  assert.equal(result.feedbackRequired, false);
  assert.equal(result.targetYawDeg, -20);
  assert.equal(result.targetPitchDeg, -10);
});

test('distance-aware 3D extrinsic corrects antenna-to-camera translation', () => {
  const result = computeBeaconControl({
    now,
    uwb: { ...freshUwb, azimuthDeg: 0, elevationDeg: 0, distanceM: 2 },
    feedback: {},
    config: {
      control_mode: 'absolute_angle',
      extrinsic_3d: {
        enabled: true,
        matrix: [1, 0, 0, 0, 1, 0, 0, 0, 1],
        translation_m: [0, 0.1, 0]
      }
    }
  });
  assert.equal(result.valid, true);
  assert.equal(result.calibrationModel, 'rigid_3d');
  assert.ok(Math.abs(result.targetYawDeg - 2.8624) < 0.001);
  assert.equal(result.targetPitchDeg, 0);
});

test('enabled 3D extrinsic safely gates measurements without distance', () => {
  const result = computeBeaconControl({
    now,
    uwb: { ...freshUwb, distanceM: null },
    feedback: {},
    config: {
      control_mode: 'absolute_angle',
      extrinsic_3d: { enabled: true, matrix: [1, 0, 0, 0, 1, 0, 0, 0, 1], translation_m: [0, 0, 0] }
    }
  });
  assert.equal(result.valid, false);
  assert.equal(result.reason, 'uwb_distance_unavailable');
});
