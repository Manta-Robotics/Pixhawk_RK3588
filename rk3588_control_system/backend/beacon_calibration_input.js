const numeric = value => (typeof value === 'number' || typeof value === 'string' && value.trim() !== '')
  && Number.isFinite(Number(value)) ? Number(value) : null;
const fresh = (at, now) => numeric(at) !== null && at > 0 && now >= at && now - at <= 700;
const admitted = status => status === undefined || status === 'trusted' || status === 'legacy';

// Calibration uses the same admitted smoothed output and clock as control.
// An unverified vertical field cannot be used for a three-dimensional fit.
export function calibrationUwbObservation(packet, now, key = null) {
  if (packet?.coordinateSource !== 'smoothed' || packet.smoothing?.ready !== true
    || packet.online !== true || packet.fresh !== true || !admitted(packet.measurementStatus)
    || packet.elevationValid !== true || packet.smoothing.elevationSmoothed !== true) return null;
  const at = packet.measurementUpdatedAt;
  const sampleKey = JSON.stringify([packet.streamId ?? null, packet.activeReceiverIds || [],
    (packet.sensors || []).map(s => [s.receiverId, s.tagId])]);
  const distance = numeric(packet.distanceM), azimuth = numeric(packet.azimuthDeg), elevation = numeric(packet.elevationDeg);
  if (!fresh(at,now) || key !== null && sampleKey !== key || distance === null || distance <= 0
    || azimuth === null || Math.abs(azimuth) > 180 || elevation === null || Math.abs(elevation) > 90) return null;
  return {at, key:sampleKey, distanceM:distance, uwbAzimuthDeg:azimuth, uwbElevationDeg:elevation};
}

export function calibrationGimbalPose(feedback, point, now, required = false) {
  const valid = feedback?.checksumValid === true && feedback.attitudeValid !== false && fresh(feedback.updatedAt, now)
    && now - feedback.updatedAt <= 300 && numeric(feedback.yawDeg) !== null && numeric(feedback.pitchDeg) !== null;
  if (!valid) {
    if (required) throw new Error('Fresh gimbal feedback required for calibration');
    return { gimbalYawDeg: point.yawDeg, gimbalPitchDeg: point.pitchDeg, feedbackVerified: false };
  }
  const yaw = Number(feedback.yawDeg), pitch = Number(feedback.pitchDeg);
  // Fit measured angles, not commanded angles. A small static servo offset is
  // allowed; the capture window separately requires a settled measured pose.
  if (Math.abs(yaw - point.yawDeg) > 3 || Math.abs(pitch - point.pitchDeg) > 3) {
    throw new Error('Gimbal has not reached the calibration point');
  }
  return { gimbalYawDeg: yaw, gimbalPitchDeg: pitch, feedbackVerified: true };
}
