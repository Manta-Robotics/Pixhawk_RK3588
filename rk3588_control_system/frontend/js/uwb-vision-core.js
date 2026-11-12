// Shared by the device backend and the browser's explicitly labelled demo.
// Body frame: X forward, Y left, bearing positive to the left.
export const FUSION_DEFAULTS = Object.freeze({
    horizontalFovDeg: 70, yawSign: 1, imageSign: -1, yawOffsetDeg: 0,
    calibrated: false, minConfidence: 0.35, visionWeight: 0.65,
    maxBearingDifferenceDeg: 20, maxSkewMs: 300,
    uwbTimeoutMs: 800, visionTimeoutMs: 600, poseTimeoutMs: 350
});

const bounds = {
    horizontalFovDeg: [5, 160], yawOffsetDeg: [-180, 180],
    minConfidence: [0.05, 1], visionWeight: [0, 1],
    maxBearingDifferenceDeg: [1, 90], maxSkewMs: [20, 1000],
    uwbTimeoutMs: [100, 3000], visionTimeoutMs: [100, 2000], poseTimeoutMs: [100, 1000]
};
export function validateFusionConfig(input) {
    if (!input || typeof input !== 'object' || Array.isArray(input)) throw new Error('配置必须是对象');
    const result = {};
    for (const [key, value] of Object.entries(input)) {
        if (!Object.hasOwn(FUSION_DEFAULTS, key)) throw new Error('未知配置项：' + key);
        if (key === 'calibrated') {
            if (typeof value !== 'boolean') throw new Error('标定状态必须为布尔值');
        } else if (key === 'yawSign' || key === 'imageSign') {
            if (value !== 1 && value !== -1) throw new Error('方向必须为 1 或 -1');
        } else {
            const [min, max] = bounds[key];
            if (!Number.isFinite(value) || value < min || value > max) throw new Error(key + ' 超出范围 ' + min + '–' + max);
        }
        result[key] = value;
    }
    return result;
}
export const wrapAngle = value => ((value + 180) % 360 + 360) % 360 - 180;
const finite = Number.isFinite;
function age(now, timestamp) {
    return finite(timestamp) && timestamp > 0 && timestamp <= now ? now - timestamp : null;
}
const fresh = (value, limit) => value !== null && value <= limit;

export function evaluateUwbVision({ now = Date.now(), enabled = false, config = {}, uwb = {},
    target = {}, pose = {}, trackerActive = false, bearingCalibration = {} } = {}) {
    const c = { ...FUSION_DEFAULTS, ...config };
    const uwbAt = uwb.measurementUpdatedAt === undefined ? uwb.updatedAt : uwb.measurementUpdatedAt;
    const ua = age(now, uwbAt), va = age(now, target.timestamp), pa = age(now, pose.updatedAt);
    const admitted = (uwb.measurementStatus === undefined || ['trusted', 'legacy'].includes(uwb.measurementStatus))
        && !['suspect', 'quarantined', 'recovering'].includes(uwb.occlusionDiagnostics?.state);
    const uValid = admitted && uwb.online === true && uwb.fresh === true && fresh(ua, c.uwbTimeoutMs)
        && finite(uwb.distanceM) && uwb.distanceM > 0 && finite(uwb.azimuthDeg) && Math.abs(uwb.azimuthDeg) <= 180;
    // Use the same azimuth calibration as existing vessel-following control.
    const calibration = uwb.coordinateCalibration || bearingCalibration;
    const uBearing = uValid ? wrapAngle(uwb.azimuthDeg * (calibration.bearingSign ?? 1)
        * (calibration.bearingScale ?? 1) + (calibration.bearingOffsetDeg ?? 0)) : null;
    const poseValid = pose.checksumValid === true && pose.attitudeValid !== false
        && finite(pose.yawDeg) && fresh(pa, c.poseTimeoutMs);
    // Effective FOV must be recalibrated if the camera changes zoom.
    const zoomValid = !finite(pose.digitalZoom) || pose.digitalZoom <= 1;
    const boxValid = finite(target.frame_w) && finite(target.frame_h) && target.frame_w > 0 && target.frame_h > 0
        && finite(target.x) && finite(target.y) && target.x >= 0 && target.x <= target.frame_w
        && target.y >= 0 && target.y <= target.frame_h && finite(target.w) && finite(target.h) && target.w > 0 && target.h > 0;
    const detected = trackerActive && target.locked === true && target.mode === 'swimmer'
        && target.coasting !== true && !['coast', 'predict', 'lost'].includes(target.status)
        && finite(target.conf) && target.conf >= c.minConfidence && boxValid && fresh(va, c.visionTimeoutMs);
    const poseSkewMs = target.timestamp > 0 && pose.updatedAt > 0 ? Math.abs(target.timestamp - pose.updatedAt) : null;
    const vValid = detected && poseValid && zoomValid && poseSkewMs <= c.maxSkewMs;
    const imageAngle = vValid ? Math.atan((target.x / target.frame_w - 0.5) * 2
        * Math.tan(c.horizontalFovDeg * Math.PI / 360)) * 180 / Math.PI : null;
    const vBearing = vValid ? wrapAngle(c.yawSign * pose.yawDeg + c.yawOffsetDeg + c.imageSign * imageAngle) : null;
    const state = { enabled, status: 'disabled', reason: 'disabled', evaluatedAt: now,
        uwb: { valid: uValid, ageMs: ua, distanceM: uValid ? uwb.distanceM : null, bearingDeg: uBearing },
        vision: { valid: vValid, ageMs: va, poseAgeMs: pa, bearingDeg: vBearing, confidence: detected ? target.conf : null,
            trackId: detected ? target.id ?? null : null, box: detected ? {
                x: target.x, y: target.y, w: target.w, h: target.h, frame_w: target.frame_w, frame_h: target.frame_h
            } : null },
        skewMs: null, bearingDifferenceDeg: null, visionContribution: 0, result: null };
    if (!enabled) return state;
    let bearing = uBearing;
    state.status = uValid ? 'uwb_only' : vValid ? 'vision_only' : 'waiting';
    state.reason = !trackerActive ? 'tracker_inactive' : !detected ? 'vision_unavailable'
        : !poseValid ? 'pose_unavailable' : !zoomValid ? 'zoom_changed'
        : !vValid ? 'pose_unsynchronized' : !uValid ? 'uwb_unavailable' : 'tracking';
    if (vValid && !c.calibrated) {
        state.reason = 'calibration_required';
    } else if (uValid && vValid) {
        state.skewMs = Math.abs(uwbAt - target.timestamp);
        state.bearingDifferenceDeg = wrapAngle(vBearing - uBearing);
        if (state.skewMs > c.maxSkewMs) state.reason = 'unsynchronized';
        else if (Math.abs(state.bearingDifferenceDeg) > c.maxBearingDifferenceDeg) {
            state.status = 'conflict'; state.reason = 'bearing_conflict';
        } else {
            const weight = c.visionWeight * target.conf;
            bearing = wrapAngle(uBearing + state.bearingDifferenceDeg * weight);
            state.status = 'fused'; state.reason = 'tracking'; state.visionContribution = weight;
        }
    }
    if (uValid) {
        state.result = { source: state.status === 'fused' ? 'uwb+vision' : 'uwb', distanceM: uwb.distanceM,
            bearingDeg: bearing, xM: uwb.distanceM * Math.cos(bearing * Math.PI / 180),
            yM: uwb.distanceM * Math.sin(bearing * Math.PI / 180), measurementUpdatedAt: uwbAt };
    } else if (vValid && c.calibrated) {
        state.result = { source: 'vision_bearing', distanceM: null, bearingDeg: vBearing, xM: null, yM: null,
            measurementUpdatedAt: target.timestamp };
    }
    return state;
}
