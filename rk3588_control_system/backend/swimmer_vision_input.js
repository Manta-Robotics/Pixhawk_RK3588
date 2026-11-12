// Only the swimmer worker may supply vision observations to UWB fusion.
// Angles remain numeric degrees; pose time is the actual feedback receive time.
const targetFields = ['mode', 'status', 'locked', 'id', 'conf', 'x', 'y', 'w', 'h',
    'frame_w', 'frame_h', 'coasting', 'prediction_age_ms', 'timestamp'];
const pick = (value, fields) => Object.fromEntries(fields.map(key => [key, value[key]]));

export function captureSwimmerVisionSample(target, feedback, { protocol, connected } = {}) {
    if (target?.mode !== 'swimmer' || target.locked !== true
        || !Number.isFinite(target.conf) || target.conf < 0 || target.conf > 1) return null;
    const validPose = connected === true && feedback?.checksumValid === true
        && Number.isFinite(feedback.yawDeg) && Number.isFinite(feedback.updatedAt)
        && (protocol !== 'siyi' || (feedback.protocol === 'siyi' && feedback.attitudeValid === true));
    const pose = validPose ? pick(feedback, ['protocol', 'yawDeg', 'updatedAt',
        'checksumValid', 'attitudeValid', 'digitalZoom']) : {};
    // Copy on TARGET receipt: a later poll must not pair an old box with a new yaw.
    return Object.freeze({ target: Object.freeze(pick(target, targetFields)), pose: Object.freeze(pose) });
}

export function swimmerVisionInputs(sample, { mode, trackingActive, workerActive, locked, connected } = {}) {
    const active = mode === 'swimmer' && trackingActive === true && workerActive === true
        && locked === true && connected === true && sample?.target.mode === 'swimmer';
    return active ? { target: sample.target, pose: sample.pose, trackerActive: true }
        : { target: {}, pose: {}, trackerActive: false };
}

// Existing vessel-follow calibration yields body-right-positive bearings (NED).
// This API uses body-left-positive bearings. Reflect calibration exactly once at
// its boundary, including the offset, without changing raw UWB or vessel control.
export function fusionBodyLeftUwb(uwb = {}, bearingCalibration = {}) {
    const calibration = uwb.coordinateCalibration || bearingCalibration;
    const sign = calibration.bearingSign ?? 1;
    const scale = calibration.bearingScale ?? 1;
    const offset = calibration.bearingOffsetDeg ?? 0;
    if (![sign, scale, offset].every(Number.isFinite)) return { ...uwb, fresh: false };
    return { ...uwb, coordinateCalibration: {
        bearingSign: -sign, bearingScale: scale, bearingOffsetDeg: -offset
    } };
}
