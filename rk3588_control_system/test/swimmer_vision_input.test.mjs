import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import { captureSwimmerVisionSample, swimmerVisionInputs, fusionBodyLeftUwb } from '../backend/swimmer_vision_input.js';
import { evaluateUwbVision, wrapAngle } from '../frontend/js/uwb-vision-core.js';
import { installUwbVisionApi } from '../backend/uwb_vision_service.js';

const active = { mode: 'swimmer', trackingActive: true, workerActive: true, locked: true, connected: true };
function observation(yaw = 0, at = 10000) {
    const target = { mode: 'swimmer', status: 'track', locked: true, id: 8, conf: 0.9,
        x: 960, y: 540, w: 100, h: 80, frame_w: 1920, frame_h: 1080, timestamp: at,
        commandYawDeg: 135, rateX: 40, desiredRateX: 50 };
    const feedback = { protocol: 'siyi', checksumValid: true, attitudeValid: true,
        yawDeg: yaw, updatedAt: at - 20, digitalZoom: 1 };
    return { target, feedback };
}
const capture = ({ target, feedback }) => captureSwimmerVisionSample(target, feedback, { protocol: 'siyi', connected: true });
function evaluate(sample, now = 10020, uwb = {}) {
    return evaluateUwbVision({ ...swimmerVisionInputs(sample, active), now, enabled: true, uwb,
        config: { calibrated: true, yawSign: -1, imageSign: -1, horizontalFovDeg: 90 } });
}

test('SIYI actual pose becomes bow-zero/left-positive bearing; movement commands are not observations', () => {
    for (const yaw of [0, 30, -30, 179, -179]) {
        const sample = capture(observation(yaw));
        assert.equal(evaluate(sample).result.bearingDeg, wrapAngle(-yaw));
        assert.equal(sample.pose.yawDeg, yaw);
        assert.equal(sample.target.commandYawDeg, undefined);
        assert.equal(sample.target.rateX, undefined);
    }
    const input = observation(30); input.target.x = 1920;
    assert.equal(evaluate(capture(input)).result.bearingDeg, -75);
});

test('only the current locked swimmer worker can supply the fusion interface', () => {
    const sample = capture(observation());
    for (const update of [{ mode: 'face' }, { mode: 'click' }, { mode: 'beacon' },
        { trackingActive: false }, { workerActive: false }, { locked: false }, { connected: false }]) {
        assert.deepEqual(swimmerVisionInputs(sample, { ...active, ...update }), { target: {}, pose: {}, trackerActive: false });
    }
    assert.equal(swimmerVisionInputs(null, active).trackerActive, false);
    const input = observation(); input.target.mode = 'face';
    assert.equal(capture(input), null);
});

test('numeric zero is valid; invalid SIYI pose types or attitude are never silently coerced', () => {
    assert.equal(evaluate(capture(observation(0))).vision.valid, true);
    for (const fields of [{ yawDeg: '0' }, { yawDeg: null }, { yawDeg: NaN }, { yawDeg: Infinity },
        { updatedAt: '9980' }, { checksumValid: false }, { protocol: 'ztd095' },
        { attitudeValid: undefined }, { attitudeValid: false }]) {
        const input = observation(); Object.assign(input.feedback, fields);
        assert.equal(evaluate(capture(input)).result, null, JSON.stringify(fields));
    }
    for (const conf of ['0.9', null, NaN, -0.1, 1.1]) {
        const input = observation(); input.target.conf = conf;
        assert.equal(capture(input), null);
    }
});

test('TARGET capture freezes its matching actual pose and retains both source receive times', () => {
    const input = observation(12), sample = capture(input);
    input.feedback.yawDeg = 90; input.feedback.updatedAt = 10100;
    input.target.x = 1920; input.target.timestamp = 10120;
    assert.equal(sample.pose.yawDeg, 12); assert.equal(sample.pose.updatedAt, 9980);
    assert.equal(sample.target.x, 960); assert.equal(sample.target.timestamp, 10000);
    assert.equal(evaluate(sample).result.bearingDeg, -12);
    assert.equal(evaluate(sample, 10400).vision.valid, false);
    assert.equal(sample.pose.updatedAt, 9980);
});

test('25 Hz TARGET updates may share a 10 Hz feedback, without inventing feedback samples', () => {
    const input = observation();
    const times = [10000, 10040, 10080].map(at => {
        input.target.timestamp = at;
        const sample = capture(input);
        assert.equal(evaluate(sample, at + 1).vision.valid, true);
        return sample.pose.updatedAt;
    });
    assert.deepEqual(times, [9980, 9980, 9980]);
    const stale = observation(); stale.feedback.updatedAt = 9500;
    assert.equal(evaluate(capture(stale)).vision.valid, false);
});

test('UWB controller-right calibration is reflected once, including scale/offset and USB metadata precedence', () => {
    const calibration = { bearingSign: -1, bearingScale: 0.5, bearingOffsetDeg: 15 };
    const original = { online: true, fresh: true, distanceM: 6, azimuthDeg: 40,
        measurementUpdatedAt: 10000, coordinateCalibration: calibration };
    const before = structuredClone(original);
    for (let i = 0; i < 3; i++) {
        const converted = fusionBodyLeftUwb(original, { bearingSign: 1, bearingScale: 2, bearingOffsetDeg: 99 });
        const state = evaluate(capture(observation(-5)), 10020, converted);
        assert.equal(state.uwb.bearingDeg, 5); assert.equal(state.vision.bearingDeg, 5);
        assert.equal(state.status, 'fused'); assert.equal(state.result.distanceM, 6);
        assert.equal(converted.measurementUpdatedAt, 10000);
        assert.equal(converted.azimuthDeg, 40);
        assert.deepEqual(original, before);
    }
    for (const angle of [0, 90, -90, 179, -179, 180]) {
        const converted = fusionBodyLeftUwb({ ...original, azimuthDeg: angle, coordinateCalibration: undefined },
            { bearingSign: 1, bearingScale: 1, bearingOffsetDeg: 0 });
        assert.equal(evaluate(capture(observation(angle)), 10020, converted).uwb.bearingDeg, wrapAngle(-angle));
    }
});

test('SIYI defaults use the proper yaw direction while saved calibration remains explicit', async t => {
    const directory = await fs.mkdtemp(path.join(os.tmpdir(), 'manta-swimmer-input-'));
    t.after(() => fs.rm(directory, { recursive: true, force: true }));
    const filename = path.join(directory, 'fusion.json');
    const app = { get() {}, post() {} };
    const options = { filename, getInputs: () => ({}), defaults: { yawSign: -1 } };
    const initial = installUwbVisionApi(app, options).read();
    assert.equal(initial.config.yawSign, -1); assert.equal(initial.config.yawOffsetDeg, 0);
    assert.equal(initial.config.calibrated, false); assert.equal(initial.state.enabled, false);
    await fs.writeFile(filename, JSON.stringify({ yawSign: 1, yawOffsetDeg: 4, calibrated: true }));
    const saved = installUwbVisionApi(app, options).read();
    assert.equal(saved.config.yawSign, 1); assert.equal(saved.config.yawOffsetDeg, 4);
    assert.equal(saved.config.calibrated, true);
    assert.throws(() => installUwbVisionApi(app, { ...options, defaults: { yawSign: '1' } }));
});
