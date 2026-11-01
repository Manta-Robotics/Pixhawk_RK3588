const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;

function finite(value) {
  if ((typeof value !== 'number' && typeof value !== 'string') || (typeof value === 'string' && value.trim() === '')) return null;
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

export function normalizeGeometryAngleDeg(value) {
  const angle = finite(value) ?? 0;
  return ((angle + 180) % 360 + 360) % 360 - 180;
}

export function uwbVectorFromSpherical(distanceM, azimuthDeg, elevationDeg) {
  const distance = finite(distanceM);
  const azimuth = finite(azimuthDeg);
  const elevation = finite(elevationDeg);
  if (distance === null || distance <= 0 || azimuth === null || Math.abs(azimuth) > 180
    || elevation === null || Math.abs(elevation) > 90) return null;
  const azimuthRad = azimuth * DEG_TO_RAD;
  const elevationRad = elevation * DEG_TO_RAD;
  const horizontal = distance * Math.cos(elevationRad);
  return [
    horizontal * Math.cos(azimuthRad),
    horizontal * Math.sin(azimuthRad),
    distance * Math.sin(elevationRad)
  ];
}

export function gimbalRayFromAngles(yawDeg, pitchDeg) {
  const yaw = finite(yawDeg);
  const pitch = finite(pitchDeg);
  if (yaw === null || pitch === null) return null;
  const yawRad = yaw * DEG_TO_RAD;
  const pitchRad = pitch * DEG_TO_RAD;
  const horizontal = Math.cos(pitchRad);
  return [horizontal * Math.cos(yawRad), horizontal * Math.sin(yawRad), Math.sin(pitchRad)];
}

export function applyMatrix3(matrix, vector) {
  if (!Array.isArray(matrix) || matrix.length !== 9 || !Array.isArray(vector) || vector.length !== 3) return null;
  const values = matrix.map(finite);
  if (values.some((value) => value === null)) return null;
  return [
    values[0] * vector[0] + values[1] * vector[1] + values[2] * vector[2],
    values[3] * vector[0] + values[4] * vector[1] + values[5] * vector[2],
    values[6] * vector[0] + values[7] * vector[1] + values[8] * vector[2]
  ];
}

export function predictGimbalAnglesFromExtrinsic(uwb, extrinsic) {
  if (!extrinsic || extrinsic.enabled !== true) return null;
  const vector = uwbVectorFromSpherical(uwb && uwb.distanceM, uwb && uwb.azimuthDeg, uwb && uwb.elevationDeg);
  const rotated = applyMatrix3(extrinsic.matrix, vector);
  const translation = Array.isArray(extrinsic.translation_m) ? extrinsic.translation_m.map(finite) : [];
  if (!rotated || translation.length !== 3 || translation.some((value) => value === null)) return null;
  const transformed = rotated.map((value, index) => value + translation[index]);
  const horizontal = Math.hypot(transformed[0], transformed[1]);
  const distanceM = Math.hypot(horizontal, transformed[2]);
  if (!Number.isFinite(distanceM) || distanceM < 0.05) return null;
  return {
    yawDeg: normalizeGeometryAngleDeg(Math.atan2(transformed[1], transformed[0]) * RAD_TO_DEG),
    pitchDeg: Math.atan2(transformed[2], horizontal) * RAD_TO_DEG,
    distanceM,
    vector: transformed
  };
}

// Yaw is independent of elevation only for a transform without vertical mixing
// or horizontal translation. Never invent a zero elevation for a general fit.
export function predictGimbalYawFromPlanarExtrinsic(azimuthDeg, extrinsic) {
  const azimuth=finite(azimuthDeg), m=extrinsic?.matrix?.map(finite), t=extrinsic?.translation_m?.map(finite);
  if (extrinsic?.enabled !== true || azimuth === null || Math.abs(azimuth)>180
    || m?.length!==9 || t?.length!==3 || [...m,...t].some(v=>v===null)
    || Math.abs(m[2])>1e-8 || Math.abs(m[5])>1e-8 || Math.abs(t[0])>1e-8 || Math.abs(t[1])>1e-8) return null;
  const a=azimuth*DEG_TO_RAD, x=m[0]*Math.cos(a)+m[1]*Math.sin(a), y=m[3]*Math.cos(a)+m[4]*Math.sin(a);
  return Math.hypot(x,y)>1e-8 ? normalizeGeometryAngleDeg(Math.atan2(y,x)*RAD_TO_DEG) : null;
}
