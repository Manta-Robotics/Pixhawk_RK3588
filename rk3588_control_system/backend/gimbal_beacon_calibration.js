import {
  applyMatrix3,
  gimbalRayFromAngles,
  normalizeGeometryAngleDeg,
  predictGimbalAnglesFromExtrinsic,
  uwbVectorFromSpherical
} from './gimbal_beacon_geometry.js';

function finite(value) {
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

function shuffle(values, random) {
  for (let index = values.length - 1; index > 0; index -= 1) {
    const other = Math.floor(random() * (index + 1));
    [values[index], values[other]] = [values[other], values[index]];
  }
  return values;
}

export function generateBeaconCalibrationPoints(random = Math.random) {
  const anchors = [
    [-32, 0], [0, 0], [32, 0],
    [-32, 10], [0, 10], [32, 10],
    [-32, 20], [0, 20], [32, 20]
  ];
  return shuffle(anchors.map(([yaw, pitch]) => ({
    yawDeg: yaw,
    pitchDeg: pitch
  })), random);
}

function multiplyMatrices3(left, right) {
  const result = Array(9).fill(0);
  for (let row = 0; row < 3; row += 1) {
    for (let column = 0; column < 3; column += 1) {
      for (let index = 0; index < 3; index += 1) result[row * 3 + column] += left[row * 3 + index] * right[index * 3 + column];
    }
  }
  return result;
}

function rotationMatrix(roll, pitch, yaw, azimuthSign = 1, elevationSign = 1) {
  const cx = Math.cos(roll); const sx = Math.sin(roll);
  const cy = Math.cos(pitch); const sy = Math.sin(pitch);
  const cz = Math.cos(yaw); const sz = Math.sin(yaw);
  const rotation = [
    cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx,
    sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx,
    -sy, cy * sx, cy * cx
  ];
  return multiplyMatrices3(rotation, [1, 0, 0, 0, azimuthSign, 0, 0, 0, elevationSign]);
}

function solveLinear3(matrix, vector) {
  const rows = [0, 1, 2].map((row) => [
    matrix[row * 3], matrix[row * 3 + 1], matrix[row * 3 + 2], vector[row]
  ]);
  for (let column = 0; column < 3; column += 1) {
    let pivot = column;
    for (let row = column + 1; row < 3; row += 1) if (Math.abs(rows[row][column]) > Math.abs(rows[pivot][column])) pivot = row;
    if (Math.abs(rows[pivot][column]) < 1e-9) return null;
    [rows[column], rows[pivot]] = [rows[pivot], rows[column]];
    const divisor = rows[column][column];
    for (let index = column; index < 4; index += 1) rows[column][index] /= divisor;
    for (let row = 0; row < 3; row += 1) {
      if (row === column) continue;
      const factor = rows[row][column];
      for (let index = column; index < 4; index += 1) rows[row][index] -= factor * rows[column][index];
    }
  }
  return rows.map((row) => row[3]);
}

function fitTranslation(matrix, prepared) {
  const normal = Array(9).fill(0);
  const right = [0, 0, 0];
  for (const sample of prepared) {
    const [dx, dy, dz] = sample.ray;
    const projection = [
      1 - dx * dx, -dx * dy, -dx * dz,
      -dy * dx, 1 - dy * dy, -dy * dz,
      -dz * dx, -dz * dy, 1 - dz * dz
    ];
    const rotated = applyMatrix3(matrix, sample.vector);
    for (let index = 0; index < 9; index += 1) normal[index] += projection[index];
    for (let row = 0; row < 3; row += 1) {
      right[row] -= projection[row * 3] * rotated[0] + projection[row * 3 + 1] * rotated[1] + projection[row * 3 + 2] * rotated[2];
    }
  }
  return solveLinear3(normal, right);
}

function evaluate(matrix, translation, prepared, maxTranslationM) {
  if (!translation || Math.hypot(...translation) > maxTranslationM) return null;
  const errors = [];
  const yawErrors = [];
  const pitchErrors = [];
  for (const sample of prepared) {
    const rotated = applyMatrix3(matrix, sample.vector);
    const transformed = rotated.map((value, index) => value + translation[index]);
    const length = Math.hypot(...transformed);
    if (!Number.isFinite(length) || length < 0.05) return null;
    const unit = transformed.map((value) => value / length);
    const dot = Math.max(-1, Math.min(1, unit.reduce((sum, value, index) => sum + value * sample.ray[index], 0)));
    errors.push(Math.acos(dot) * 180 / Math.PI);
    const prediction = {
      yawDeg: Math.atan2(transformed[1], transformed[0]) * 180 / Math.PI,
      pitchDeg: Math.atan2(transformed[2], Math.hypot(transformed[0], transformed[1])) * 180 / Math.PI
    };
    yawErrors.push(normalizeGeometryAngleDeg(prediction.yawDeg - sample.gimbalYawDeg));
    pitchErrors.push(prediction.pitchDeg - sample.gimbalPitchDeg);
  }
  const rms = (values) => Math.sqrt(values.reduce((sum, value) => sum + value * value, 0) / values.length);
  const angularRmsErrorDeg = rms(errors);
  const angularMaxErrorDeg = Math.max(...errors);
  return {
    score: angularRmsErrorDeg + angularMaxErrorDeg * 0.05,
    angularRmsErrorDeg,
    angularMaxErrorDeg,
    yawRmsErrorDeg: rms(yawErrors),
    pitchRmsErrorDeg: rms(pitchErrors)
  };
}

function refineCandidate(candidate, prepared, medianDistance, maxTranslationM) {
  const params = [...candidate.params];
  const steps = [Math.PI / 8, Math.PI / 8, Math.PI / 8, Math.max(0.05, medianDistance * 0.25), Math.max(0.05, medianDistance * 0.25), Math.max(0.05, medianDistance * 0.25)];
  let matrix = rotationMatrix(params[0], params[1], params[2], candidate.azimuthSign, candidate.elevationSign);
  let metrics = evaluate(matrix, params.slice(3), prepared, maxTranslationM);
  for (let iteration = 0; iteration < 120; iteration += 1) {
    let improved = false;
    for (let index = 0; index < params.length; index += 1) {
      for (const direction of [-1, 1]) {
        const trial = [...params];
        trial[index] += direction * steps[index];
        const trialMatrix = rotationMatrix(trial[0], trial[1], trial[2], candidate.azimuthSign, candidate.elevationSign);
        const trialMetrics = evaluate(trialMatrix, trial.slice(3), prepared, maxTranslationM);
        if (trialMetrics && (!metrics || trialMetrics.score + 1e-9 < metrics.score)) {
          params.splice(0, params.length, ...trial);
          matrix = trialMatrix;
          metrics = trialMetrics;
          improved = true;
        }
      }
    }
    if (!improved) steps.forEach((value, index) => { steps[index] = value * 0.5; });
    if (steps[0] < 0.0005 && steps[3] < 0.0005) break;
  }
  return metrics ? { ...candidate, params, matrix, translation: params.slice(3), metrics } : null;
}

export function fitBeaconCalibration(samples) {
  if (!Array.isArray(samples) || samples.length !== 9) throw new Error('Exactly 9 calibration samples are required');
  const prepared = samples.map((sample) => {
    const distanceM = finite(sample.distanceM);
    const uwbAzimuthDeg = finite(sample.uwbAzimuthDeg);
    const uwbElevationDeg = finite(sample.uwbElevationDeg);
    const gimbalYawDeg = finite(sample.gimbalYawDeg);
    const gimbalPitchDeg = finite(sample.gimbalPitchDeg);
    const vector = uwbVectorFromSpherical(distanceM, uwbAzimuthDeg, uwbElevationDeg);
    const ray = gimbalRayFromAngles(gimbalYawDeg, gimbalPitchDeg);
    if (!vector || !ray || distanceM < 0.05 || distanceM > 100) throw new Error('UWB distance is required for 3D calibration; restart the nine-point calibration');
    return { distanceM, uwbAzimuthDeg, uwbElevationDeg, gimbalYawDeg, gimbalPitchDeg, vector, ray };
  });
  const yawSpan = Math.max(...prepared.map((sample) => sample.gimbalYawDeg)) - Math.min(...prepared.map((sample) => sample.gimbalYawDeg));
  const pitchSpan = Math.max(...prepared.map((sample) => sample.gimbalPitchDeg)) - Math.min(...prepared.map((sample) => sample.gimbalPitchDeg));
  if (yawSpan < 40 || pitchSpan < 20) throw new Error('The 3D calibration grid does not span enough yaw and pitch');
  const distances = prepared.map((sample) => sample.distanceM).sort((left, right) => left - right);
  const medianDistance = distances[Math.floor(distances.length / 2)];
  const maxTranslationM = Math.max(0.5, Math.min(3, medianDistance * 2));
  const coarseAngles = [-180, -135, -90, -45, 0, 45, 90, 135].map((value) => value * Math.PI / 180);
  const coarsePitch = [-90, -45, 0, 45, 90].map((value) => value * Math.PI / 180);
  const candidates = [];
  for (const azimuthSign of [-1, 1]) {
    for (const elevationSign of [-1, 1]) {
      for (const roll of coarseAngles) {
        for (const pitch of coarsePitch) {
          for (const yaw of coarseAngles) {
            const matrix = rotationMatrix(roll, pitch, yaw, azimuthSign, elevationSign);
            const translation = fitTranslation(matrix, prepared);
            const metrics = evaluate(matrix, translation, prepared, maxTranslationM);
            if (metrics) candidates.push({ params: [roll, pitch, yaw, ...translation], azimuthSign, elevationSign, matrix, translation, metrics });
          }
        }
      }
    }
  }
  candidates.sort((left, right) => left.metrics.score - right.metrics.score);
  const refined = candidates.slice(0, 10).map((candidate) => refineCandidate(candidate, prepared, medianDistance, maxTranslationM)).filter(Boolean);
  refined.sort((left, right) => left.metrics.score - right.metrics.score);
  const best = refined[0];
  if (!best || best.metrics.angularRmsErrorDeg > 7 || best.metrics.angularMaxErrorDeg > 16) {
    const detail = best ? ` (RMS ${best.metrics.angularRmsErrorDeg.toFixed(1)}°, max ${best.metrics.angularMaxErrorDeg.toFixed(1)}°)` : '';
    throw new Error(`3D fit rejected${detail}; keep the beacon centered and keep its antenna orientation fixed`);
  }
  const round = (value, digits) => Number(value.toFixed(digits));
  const extrinsic3d = {
    enabled: true,
    model: 'rigid_point_to_ray_v1',
    matrix: best.matrix.map((value) => round(value, 9)),
    translation_m: best.translation.map((value) => round(value, 6))
  };
  const verification = prepared.map((sample) => predictGimbalAnglesFromExtrinsic({
    distanceM: sample.distanceM,
    azimuthDeg: sample.uwbAzimuthDeg,
    elevationDeg: sample.uwbElevationDeg
  }, extrinsic3d));
  if (verification.some((prediction) => !prediction)) throw new Error('3D calibration produced an invalid transform');
  return {
    model: 'rigid_3d',
    extrinsic_3d: extrinsic3d,
    metrics: {
      angularRmsErrorDeg: round(best.metrics.angularRmsErrorDeg, 3),
      angularMaxErrorDeg: round(best.metrics.angularMaxErrorDeg, 3),
      yawRmsErrorDeg: round(best.metrics.yawRmsErrorDeg, 3),
      pitchRmsErrorDeg: round(best.metrics.pitchRmsErrorDeg, 3),
      minDistanceM: round(distances[0], 3),
      maxDistanceM: round(distances[distances.length - 1], 3),
      translationNormM: round(Math.hypot(...best.translation), 4)
    }
  };
}
