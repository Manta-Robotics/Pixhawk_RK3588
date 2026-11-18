import assert from 'node:assert/strict';
import test from 'node:test';
import { evaluateUwbVision, validateFusionConfig, FUSION_DEFAULTS } from '../frontend/js/uwb-vision-core.js';

function sample(overrides = {}) {
    return { now: 10000, enabled: true, config: { calibrated: true },
        uwb: { online: true, fresh: true, distanceM: 6, azimuthDeg: 10, measurementUpdatedAt: 9950, measurementStatus: 'trusted' },
        target: { mode: 'swimmer', status: 'track', id: 8, locked: true, conf: 0.9, frame_w: 1920, frame_h: 1080,
            x: 960, y: 540, w: 120, h: 80, timestamp: 9940 },
        pose: { checksumValid: true, yawDeg: 14, updatedAt: 9970, digitalZoom: 1 }, trackerActive: true, ...overrides };
}
test('fusion retains UWB range, weights the synchronized bearing and returns body coordinates', () => {
    const state = evaluateUwbVision(sample());
    assert.equal(state.status, 'fused'); assert.equal(state.result.distanceM, 6);
    assert.ok(Math.abs(state.result.bearingDeg - (10 + 4 * 0.65 * 0.9)) < 1e-9);
    assert.ok(Math.abs(Math.hypot(state.result.xM, state.result.yM) - 6) < 1e-9);
    assert.equal(state.skewMs, 10); assert.equal(state.vision.trackId, 8);
});
test('camera geometry handles image edge, yaw sign, offset and UWB calibration in the same body frame', () => {
    const input = sample();
    input.config = { calibrated: true, horizontalFovDeg: 90, yawSign: -1, imageSign: -1, yawOffsetDeg: 5 };
    input.target.x = 1920; input.pose.yawDeg = -60;
    input.uwb.azimuthDeg = -10; input.bearingCalibration = { bearingSign: -1, bearingScale: 2, bearingOffsetDeg: 0 };
    const state = evaluateUwbVision(input);
    assert.equal(state.vision.bearingDeg, 20); assert.equal(state.uwb.bearingDeg, 20); assert.equal(state.status, 'fused');
});
test('angles wrap across ±180 degrees without fusing through zero', () => {
    const input = sample(); input.uwb.azimuthDeg = 179; input.pose.yawDeg = -179;
    const state = evaluateUwbVision(input);
    assert.equal(state.status, 'fused'); assert.equal(state.bearingDifferenceDeg, 2);
    assert.ok(Math.abs(state.result.bearingDeg) > 178);
});
test('disabled mode publishes no position and calibration defaults prevent fusion', () => {
    assert.equal(evaluateUwbVision(sample({ enabled: false })).result, null);
    const state = evaluateUwbVision(sample({ config: {} }));
    assert.equal(FUSION_DEFAULTS.calibrated, false); assert.equal(state.reason, 'calibration_required');
    assert.equal(state.result.source, 'uwb'); assert.equal(state.result.bearingDeg, 10);
});
test('missing vision, predictions, low confidence, faces and invalid boxes fall back to fresh UWB', () => {
    for (const fields of [{ locked: false }, { coasting: true }, { status: 'coast' }, { conf: 0.1 }, { conf: null },
        { mode: 'face' }, { frame_w: 0 }, { x: -2 }, { w: null }, { timestamp: 9000 }, { timestamp: 11000 }]) {
        const input = sample(); Object.assign(input.target, fields);
        const state = evaluateUwbVision(input);
        assert.equal(state.status, 'uwb_only', JSON.stringify(fields)); assert.equal(state.result.source, 'uwb');
    }
    assert.equal(evaluateUwbVision(sample({ trackerActive: false })).reason, 'tracker_inactive');
});
test('stale, invalid, unsynchronized pose or changed zoom cannot support visual fusion', () => {
    for (const fields of [{ checksumValid: false }, { yawDeg: null }, { updatedAt: 9000 }, { updatedAt: 11000 }, { digitalZoom: 2 }]) {
        const input = sample(); Object.assign(input.pose, fields);
        assert.equal(evaluateUwbVision(input).result.source, 'uwb', JSON.stringify(fields));
    }
    const input = sample(); input.target.timestamp = 9450;
    assert.equal(evaluateUwbVision(input).reason, 'pose_unsynchronized');
});
test('UWB loss yields bearing only, never fabricates range from a detection box', () => {
    for (const fields of [{ online: false }, { fresh: false }, { distanceM: null }, { distanceM: 0 }, { azimuthDeg: 181 },
        { measurementUpdatedAt: null }, { measurementUpdatedAt: 9000 }, { measurementUpdatedAt: 11000 },
        { measurementStatus: 'suspect' }, { occlusionDiagnostics: { state: 'quarantined' } }]) {
        const input = sample(); Object.assign(input.uwb, fields);
        const state = evaluateUwbVision(input);
        assert.equal(state.status, 'vision_only', JSON.stringify(fields));
        assert.equal(state.result.distanceM, null); assert.equal(state.result.xM, null); assert.equal(state.result.yM, null);
    }
});
test('legacy measurement timestamps are accepted, but an explicit null clock is never replaced by refresh time', () => {
    const input = sample(); delete input.uwb.measurementUpdatedAt; input.uwb.updatedAt = 9950;
    assert.equal(evaluateUwbVision(input).status, 'fused');
    input.uwb.measurementUpdatedAt = null;
    assert.equal(evaluateUwbVision(input).status, 'vision_only');
});
test('bearing conflicts and sensor time skew reject fusion and preserve the UWB solution', () => {
    const conflict = sample(); conflict.pose.yawDeg = 70;
    const state = evaluateUwbVision(conflict);
    assert.equal(state.status, 'conflict'); assert.equal(state.result.source, 'uwb'); assert.equal(state.result.bearingDeg, 10);
    const skew = sample(); skew.uwb.measurementUpdatedAt = 9300;
    const unsynced = evaluateUwbVision(skew);
    assert.equal(unsynced.reason, 'unsynchronized'); assert.equal(unsynced.result.source, 'uwb');
});
test('both sensors expiring clears the previous fix immediately', () => {
    const input = sample(); assert.equal(evaluateUwbVision(input).status, 'fused');
    input.now = 12000;
    const lost = evaluateUwbVision(input); assert.equal(lost.status, 'waiting'); assert.equal(lost.result, null);
});
test('configuration rejects invalid types, unknown fields, excessive weights and unsafe timing ranges', () => {
    for (const value of [null, [], { minConfidence: '0.5' }, { visionWeight: 1.01 }, { yawSign: 0 }, { calibrated: 1 },
        { horizontalFovDeg: 180 }, { maxSkewMs: 0 }, { unknown: true }, { yawOffsetDeg: Infinity }]) {
        assert.throws(() => validateFusionConfig(value));
    }
    assert.deepEqual(validateFusionConfig({ ...FUSION_DEFAULTS }), FUSION_DEFAULTS);
});
