import assert from 'node:assert/strict';
import test from 'node:test';
import { fitBeaconCalibration, generateBeaconCalibrationPoints } from '../backend/gimbal_beacon_calibration.js';
import { predictGimbalAnglesFromExtrinsic } from '../backend/gimbal_beacon_geometry.js';

test('generates a shuffled safe 3x3 calibration grid spanning both axes', () => {
  let value = 0;
  const points = generateBeaconCalibrationPoints(() => ((value += 0.173) % 1));
  assert.equal(points.length, 9);
  assert.deepEqual([...new Set(points.map((point) => point.yawDeg))].sort((a, b) => a - b), [-32, 0, 32]);
  assert.deepEqual([...new Set(points.map((point) => point.pitchDeg))].sort((a, b) => a - b), [0, 10, 20]);
  assert.equal(new Set(points.map((point) => `${point.yawDeg},${point.pitchDeg}`)).size, 9);
  assert.ok(Math.min(...points.map((point) => point.yawDeg)) < -25);
  assert.ok(Math.max(...points.map((point) => point.yawDeg)) > 25);
  assert.equal(Math.min(...points.map((point) => point.pitchDeg)), 0);
  assert.equal(Math.max(...points.map((point) => point.pitchDeg)), 20);
  assert.ok(points.every((point) => Math.abs(point.yawDeg) <= 45 && point.pitchDeg >= 0 && point.pitchDeg <= 20));
});

function transposeMatrix3(matrix) {
  return [matrix[0], matrix[3], matrix[6], matrix[1], matrix[4], matrix[7], matrix[2], matrix[5], matrix[8]];
}

function applyMatrix3(matrix, vector) {
  return [0, 1, 2].map((row) => matrix[row * 3] * vector[0] + matrix[row * 3 + 1] * vector[1] + matrix[row * 3 + 2] * vector[2]);
}

function measurementFromVector(vector) {
  const distanceM = Math.hypot(...vector);
  return {
    distanceM,
    uwbAzimuthDeg: Math.atan2(vector[1], vector[0]) * 180 / Math.PI,
    uwbElevationDeg: Math.atan2(vector[2], Math.hypot(vector[0], vector[1])) * 180 / Math.PI
  };
}

function syntheticSample(yawDeg, pitchDeg, rangeM, matrix, translation) {
  const yaw = yawDeg * Math.PI / 180;
  const pitch = pitchDeg * Math.PI / 180;
  const ray = [Math.cos(pitch) * Math.cos(yaw), Math.cos(pitch) * Math.sin(yaw), Math.sin(pitch)];
  const cameraPoint = ray.map((value) => value * rangeM);
  const uwbPoint = applyMatrix3(transposeMatrix3(matrix), cameraPoint.map((value, index) => value - translation[index]));
  return { ...measurementFromVector(uwbPoint), gimbalYawDeg: yawDeg, gimbalPitchDeg: pitchDeg };
}

test('fits a distance-aware rigid transform from nine centered samples', () => {
  const angle = 24 * Math.PI / 180;
  const matrix = [Math.cos(angle), -Math.sin(angle), 0, Math.sin(angle), Math.cos(angle), 0, 0, 0, 1];
  const translation = [0.18, -0.07, 0.11];
  const grid = [[-32, 0], [0, 0], [32, 0], [-32, 10], [0, 10], [32, 10], [-32, 20], [0, 20], [32, 20]];
  const samples = grid.map(([yaw, pitch], index) => syntheticSample(yaw, pitch, 1.3 + index * 0.17, matrix, translation));
  const result = fitBeaconCalibration(samples);
  assert.equal(result.model, 'rigid_3d');
  assert.equal(result.extrinsic_3d.enabled, true);
  assert.ok(result.metrics.angularRmsErrorDeg < 0.2);
  const heldOut = syntheticSample(17, -7, 2.4, matrix, translation);
  const prediction = predictGimbalAnglesFromExtrinsic({
    distanceM: heldOut.distanceM,
    azimuthDeg: heldOut.uwbAzimuthDeg,
    elevationDeg: heldOut.uwbElevationDeg
  }, result.extrinsic_3d);
  assert.ok(Math.abs(prediction.yawDeg - 17) < 0.5);
  assert.ok(Math.abs(prediction.pitchDeg + 7) < 0.5);
});

test('rejects clustered or inconsistent samples', () => {
  const samples = Array.from({ length: 9 }, (_, index) => ({
    uwbAzimuthDeg: index,
    uwbElevationDeg: index,
    distanceM: 1,
    gimbalYawDeg: index % 2 ? 30 : -30,
    gimbalPitchDeg: index % 2 ? -20 : 20
  }));
  assert.throws(() => fitBeaconCalibration(samples));
});
