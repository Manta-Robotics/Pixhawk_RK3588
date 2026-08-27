import { predictGimbalAnglesFromExtrinsic } from './gimbal_beacon_geometry.js';

function finiteNumber(value, fallback = null) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

export function normalizeAngleDeg(value) {
  const angle = finiteNumber(value, 0);
  return ((angle + 180) % 360 + 360) % 360 - 180;
}

function trimDeadband(errorDeg, deadbandDeg) {
  const magnitude = Math.abs(errorDeg);
  if (magnitude <= deadbandDeg) return 0;
  return Math.sign(errorDeg) * (magnitude - deadbandDeg);
}

export function computeBeaconControl({ now = Date.now(), uwb = {}, feedback = {}, config = {} } = {}) {
  const timeoutMs = Math.max(100, finiteNumber(config.timeout_ms, 700));
  const feedbackTimeoutMs = Math.max(100, finiteNumber(config.feedback_timeout_ms, 350));
  const uwbUpdatedAt = finiteNumber(uwb.updatedAt);
  const feedbackUpdatedAt = finiteNumber(feedback.updatedAt);
  const uwbAgeMs = uwbUpdatedAt === null ? Infinity : Math.max(0, now - uwbUpdatedAt);
  const feedbackAgeMs = feedbackUpdatedAt === null ? Infinity : Math.max(0, now - feedbackUpdatedAt);
  const azimuthDeg = finiteNumber(uwb.azimuthDeg);
  const elevationDeg = finiteNumber(uwb.elevationDeg);
  const usePitch = config.use_pitch !== false;
  const controlMode = String(config.control_mode || 'feedback_rate').trim().toLowerCase();
  const absoluteAngleMode = controlMode === 'absolute_angle';

  if (uwb.online !== true || uwb.fresh !== true || azimuthDeg === null || (usePitch && elevationDeg === null) || uwbAgeMs > timeoutMs) {
    return { valid: false, reason: 'uwb_stale', uwbAgeMs, feedbackAgeMs, rateX: 0, rateY: 0 };
  }
  if (!absoluteAngleMode && (feedback.checksumValid === false || finiteNumber(feedback.yawDeg) === null || (usePitch && finiteNumber(feedback.pitchDeg) === null) || feedbackAgeMs > feedbackTimeoutMs)) {
    return { valid: false, reason: 'gimbal_feedback_stale', uwbAgeMs, feedbackAgeMs, rateX: 0, rateY: 0 };
  }

  const yawSourceSign = finiteNumber(config.yaw_source_sign, 1) < 0 ? -1 : 1;
  const pitchSourceSign = finiteNumber(config.pitch_source_sign, 1) < 0 ? -1 : 1;
  const yawSourceScale = Math.max(0, finiteNumber(config.yaw_source_scale, 1));
  const pitchSourceScale = Math.max(0, finiteNumber(config.pitch_source_scale, 1));
  const yawCommandSign = finiteNumber(config.yaw_command_sign, 1) < 0 ? -1 : 1;
  const pitchCommandSign = finiteNumber(config.pitch_command_sign, 1) < 0 ? -1 : 1;
  const yawOffsetDeg = finiteNumber(config.yaw_offset_deg, 0);
  const pitchOffsetDeg = finiteNumber(config.pitch_offset_deg, 0);
  const yawDeadbandDeg = Math.max(0, finiteNumber(config.yaw_deadband_deg, 1.5));
  const pitchDeadbandDeg = Math.max(0, finiteNumber(config.pitch_deadband_deg, 1.5));
  const yawGain = Math.max(0, finiteNumber(config.yaw_gain, 2));
  const pitchGain = Math.max(0, finiteNumber(config.pitch_gain, 2));
  const maxYawRateDps = Math.max(0.5, finiteNumber(config.max_yaw_rate_dps, 25));
  const maxPitchRateDps = Math.max(0.5, finiteNumber(config.max_pitch_rate_dps, 20));

  const extrinsic = config.extrinsic_3d;
  const useExtrinsic = Boolean(extrinsic && extrinsic.enabled === true);
  const extrinsicPrediction = useExtrinsic ? predictGimbalAnglesFromExtrinsic({
    distanceM: uwb.distanceM,
    azimuthDeg,
    elevationDeg
  }, extrinsic) : null;
  if (useExtrinsic && !extrinsicPrediction) {
    return { valid: false, reason: 'uwb_distance_unavailable', uwbAgeMs, feedbackAgeMs, rateX: 0, rateY: 0 };
  }
  const targetYawDeg = extrinsicPrediction
    ? normalizeAngleDeg(extrinsicPrediction.yawDeg)
    : normalizeAngleDeg(azimuthDeg * yawSourceSign * yawSourceScale + yawOffsetDeg);
  const targetPitchDeg = usePitch
    ? (extrinsicPrediction ? extrinsicPrediction.pitchDeg : elevationDeg * pitchSourceSign * pitchSourceScale + pitchOffsetDeg)
    : finiteNumber(feedback.pitchDeg, 0);
  const calibrationModel = extrinsicPrediction ? 'rigid_3d' : 'legacy_angular';
  if (absoluteAngleMode) {
    return {
      valid: true,
      reason: 'tracking',
      controlMode: 'absolute_angle',
      feedbackRequired: false,
      uwbAgeMs,
      feedbackAgeMs,
      azimuthDeg,
      elevationDeg,
      distanceM: finiteNumber(uwb.distanceM),
      calibrationModel,
      targetYawDeg,
      targetPitchDeg,
      yawDeadbandDeg,
      pitchDeadbandDeg,
      maxYawRateDps,
      maxPitchRateDps,
      rateX: 0,
      rateY: 0
    };
  }
  const yawErrorDeg = normalizeAngleDeg(targetYawDeg - finiteNumber(feedback.yawDeg, 0));
  const pitchErrorDeg = usePitch ? targetPitchDeg - finiteNumber(feedback.pitchDeg, 0) : 0;
  const activeYawErrorDeg = trimDeadband(yawErrorDeg, yawDeadbandDeg);
  const activePitchErrorDeg = trimDeadband(pitchErrorDeg, pitchDeadbandDeg);
  const rateX = clamp(activeYawErrorDeg * yawGain * yawCommandSign, -maxYawRateDps, maxYawRateDps);
  const rateY = clamp(activePitchErrorDeg * pitchGain * pitchCommandSign, -maxPitchRateDps, maxPitchRateDps);

  return {
    valid: true,
    reason: 'tracking',
    uwbAgeMs,
    feedbackAgeMs,
    azimuthDeg,
    elevationDeg,
    distanceM: finiteNumber(uwb.distanceM),
    calibrationModel,
    targetYawDeg,
    targetPitchDeg,
    yawErrorDeg,
    pitchErrorDeg,
    activeYawErrorDeg,
    activePitchErrorDeg,
    rateX,
    rateY
  };
}
