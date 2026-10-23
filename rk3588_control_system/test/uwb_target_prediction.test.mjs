import test from 'node:test';
import assert from 'node:assert/strict';
import { UwbTargetEstimator, IntermittentUwbFollow, usableControlEstimate } from '../backend/uwb_target_prediction.js';
import { UwbFollowController } from '../backend/uwb_follow.js';
import { NeuralUwbFollowController } from '../backend/neural_uwb_follow.js';

const prediction = { enabled: true };
const calibration = { bearing_sign: 1, bearing_scale: 1, bearing_offset_deg: 0 };
function telemetry(now, range = 3, overrides = {}) {
  return { flightMode: 'MANUAL', armed: true, pilotSteering: { ready: true, type: 0 },
    attitude: { yaw: 0, updatedAt: now }, velocity: { vx: 0, vy: 0 },
    position: { updatedAt: now }, ekf: { healthy: true, updatedAt: now }, gps: { fixType: 0 },
    uwb: { online: true, fresh: true, distanceM: range, azimuthDeg: 0, updatedAt: now, measurementUpdatedAt: now }, ...overrides };
}
function controller(neural = false) {
  const options = { ...calibration, prediction, manual_propulsion: 'single_side', following_distance_m: 0.5,
    hard_stop_enabled: false, controller_profile: 'traditional_fast', commissioning_mode: false,
    relative_throttle_limit_percent: 70, relative_min_effective_throttle_percent: 35 };
  return neural ? new NeuralUwbFollowController({ ...options,
    neural_model_path: new URL('../models/smart_usv_uwb_policy.json', import.meta.url).pathname.replace(/^\/([A-Za-z]:)/, '$1') })
    : new UwbFollowController(options);
}
function lost(now, lastAt = 1200, overrides = {}) {
  return telemetry(now, 3.2, { uwb: { online: false, fresh: false, distanceM: 3.2, azimuthDeg: 0,
    updatedAt: lastAt, measurementUpdatedAt: lastAt }, ...overrides });
}
function warm(estimator) {
  [1000, 1100, 1200].forEach((now, i) => estimator.update(now, telemetry(now, 3 + i * 0.1)));
}
function run(session, c, now, t) {
  const result = session.update({ controller: c, now, telemetry: t, connected: true, controlMode: 'relative_manual' });
  session.governDrive({ throttle: result.throttlePercent, steering: result.steeringDegrees }, result);
  return result;
}

test('constant velocity predicts target displacement without changing sensor freshness', () => {
  const e = new UwbTargetEstimator(prediction, calibration);
  for(let at=1000;at<=2300;at+=100)e.update(at,telemetry(at,3+(at-1000)*.0002));
  const t = lost(2700,2300), before = structuredClone(t);
  const result = e.update(2700, t);
  assert.equal(result.source, 'predicted'); assert.equal(result.usable, true);
  assert.ok(Math.abs(result.relativeBodyM.x - 3.34) < 1e-8);
  assert.equal(result.lastMeasuredAt, 2300); assert.equal(result.predictionAgeMs, 400);
  assert.deepEqual(t, before); assert.ok(result.uncertaintyM > 0.12);
});

test('rejected sensor covariance raises target uncertainty without accepting an outlier', () => {
  const e = new UwbTargetEstimator(prediction, calibration); warm(e);
  const t = lost(1600); t.uwb.measurementStatus = 'suspect'; t.uwb.rejectionReason = 'innovation_outlier';
  t.uwb.sensors = [{ receiverId: 1, measurementStatus: 'suspect', filterUncertaintyM: 0.8 }];
  const result = e.update(1600, t);
  assert.equal(result.lastMeasuredAt, 1200); assert.equal(result.source, 'predicted');
  assert.ok(result.uncertaintyM >= 0.8); assert.equal(result.usable, false);
  assert.equal(result.inputRejectionReason, 'innovation_outlier');
  const session = new IntermittentUwbFollow(prediction, calibration), c = controller();
  [1000, 1100, 1200].forEach((now) => run(session, c, now, telemetry(now)));
  assert.equal(run(session, c, 1600, t).status, 'waiting_target');
});

test('a rejected other receiver does not inflate a selected trusted receiver uncertainty', () => {
  const e = new UwbTargetEstimator(prediction, calibration);
  const t = telemetry(1000); t.uwb.activeReceiverIds = [1];
  t.uwb.sensors = [{ receiverId: 1, filterUncertaintyM: 0.2 }, { receiverId: 2, filterUncertaintyM: 10 }];
  assert.equal(e.update(1000, t).measurementUncertaintyM, 0.2);
});

test('a rejected close raw range vetoes propulsion without becoming a target coordinate', () => {
  const session = new IntermittentUwbFollow(prediction, calibration), c = controller();
  [1000, 1100, 1200].forEach((now) => run(session, c, now, telemetry(now)));
  const t = lost(1500); t.uwb.sensors = [{ receiverId: 1, measurementStatus: 'suspect',
    filterUncertaintyM: 0.2, rawDistanceM: 0.3, rawDistanceUpdatedAt: 1500 }];
  const result = run(session, c, 1500, t);
  assert.equal(result.status, 'waiting_target'); assert.equal(result.reason, 'prediction_clearance');
  assert.equal(result.throttlePercent, 0); assert.equal(result.targetEstimate.lastMeasuredAt, 1200);
  assert.ok(result.targetEstimate.relativeBodyM.x > 2);
});

test('prediction accounts for vessel translation and heading in a local fixed frame', () => {
  const e = new UwbTargetEstimator(prediction, calibration);
  [1000, 1100, 1200].forEach((at, i) => e.update(at, telemetry(at, 3 - 0.1 * i, { velocity: { vx: 1, vy: 0 } })));
  const result = e.update(1600, lost(1600, 1200, { velocity: { vx: 1, vy: 0 }, attitude: { yaw: 90, updatedAt: 1600 } }));
  assert.ok(Math.abs(result.relativeBodyM.x) < 1e-9);
  assert.ok(Math.abs(result.relativeBodyM.y + 2.4) < 1e-9);
});

test('duplicate measurement sequence cannot grow samples or extend the prediction window', () => {
  const e = new UwbTargetEstimator(prediction, calibration);
  for (let now = 1000; now <= 1700; now += 100) {
    const t = telemetry(now); t.uwb.sensors = [{ receiverId: 1, sequenceId: 42, tagId: 7 }]; t.uwb.activeReceiverIds = [1];
    e.update(now, t);
  }
  assert.equal(e.last.samples, 1); assert.equal(e.last.at, 1000);
  assert.equal(e.update(1800, lost(1800, 1000)).usable, false);
});

test('jumping and changed-tag measurements do not silently reacquire another target', () => {
  const e = new UwbTargetEstimator(prediction, calibration); warm(e);
  e.update(1600, lost(1600));
  const jump = e.update(1700, telemetry(1700, 15));
  assert.equal(jump.usable, false); assert.equal(jump.reason, 'measurement_jump');
  assert.equal(e.last.at, 1200);
  assert.equal(e.update(1800, telemetry(1800, 15)).reacquiring, true);
  const tags = new UwbTargetEstimator(prediction, calibration);
  const first = telemetry(1000); first.uwb.tagId = 1; tags.update(1000, first);
  const changed = telemetry(1100); changed.uwb.tagId = 2;
  assert.equal(tags.update(1100, changed).reason, 'target_identity_changed');
});

test('recovery needs three distinct consistent measurements and starts with zero propulsion', () => {
  const session = new IntermittentUwbFollow(prediction, calibration), c = controller();
  [1000, 1100, 1200].forEach((now, i) => run(session, c, now, telemetry(now, 3 + i * 0.1)));
  run(session, c, 1600, lost(1600));
  const first = run(session, c, 1700, telemetry(1700, 3.7));
  assert.equal(first.status, 'waiting_target'); assert.equal(first.throttlePercent, 0);
  const repeated = telemetry(1750, 3.7); repeated.uwb.measurementUpdatedAt = 1700;
  assert.equal(run(session, c, 1750, repeated).targetEstimate.recoverySamples, 1);
  assert.equal(run(session, c, 1800, telemetry(1800, 3.8)).status, 'waiting_target');
  const recovered = run(session, c, 1900, telemetry(1900, 3.9));
  assert.equal(recovered.status, 'recovering'); assert.equal(recovered.intermittentThrottleCap, 0);
  assert.equal(recovered.throttlePercent, 0);
  const blend = run(session, c, 2000, telemetry(2000, 4));
  assert.ok(blend.intermittentThrottleCap > 0 && blend.intermittentThrottleCap < 15);
});

test('normal real-UWB following without healthy navigation survives short loss by waiting and reacquiring', () => {
  const session = new IntermittentUwbFollow(prediction, calibration), c = controller();
  const noEkf = (now, valid = true) => telemetry(now, 3, { ekf: { healthy: false, updatedAt: now },
    ...(valid ? {} : { uwb: { online: false, fresh: false, measurementUpdatedAt: 1000 } }) });
  assert.equal(run(session, c, 1000, noEkf(1000)).safe, true);
  assert.equal(run(session, c, 1500, noEkf(1500, false)).status, 'waiting_target');
  assert.equal(run(session, c, 1600, noEkf(1600)).status, 'waiting_target');
  assert.equal(run(session, c, 1700, noEkf(1700)).status, 'waiting_target');
  assert.equal(run(session, c, 1800, noEkf(1800)).status, 'recovering');
});

for (const neural of [false, true]) test(`${neural ? 'neural' : 'traditional'} controller follows a bounded explicit prediction`, () => {
  const session = new IntermittentUwbFollow(prediction, calibration), c = controller(neural);
  [1000, 1100, 1200].forEach((now, i) => run(session, c, now, telemetry(now, 3 + i * 0.1)));
  const result = run(session, c, 1600, lost(1600));
  assert.equal(result.status, 'predicting'); assert.equal(result.safe, true);
  assert.ok(result.throttlePercent >= 0 && result.throttlePercent <= 9);
  assert.ok(Math.abs(result.steeringDegrees) <= 3.6);
  assert.equal(result.targetEstimate.lastMeasuredAt, 1200);
  assert.equal(c.preflight({ now: 1600, connected: true, telemetry: lost(1600) }).safe, false);
});

test('close clearance, high speed, stale pose, uncertainty and horizon hold neutral', () => {
  for (const [name, config, range, override] of [
    ['clearance', {}, 0.8, {}], ['speed', {}, 3, { velocity: { vx: 1, vy: 0 } }],
    ['pose', {}, 3, { ekf: { healthy: true, updatedAt: 800 } }],
    ['uncertainty', { max_uncertainty_m: 0.15 }, 3, {}], ['horizon', { horizon_ms: 200 }, 3, {}]
  ]) {
    const session = new IntermittentUwbFollow({ ...prediction, ...config }, calibration), c = controller();
    [1000, 1100, 1200].forEach((at) => run(session, c, at, telemetry(at, range, override)));
    const result = run(session, c, 1600, lost(1600, 1200, override));
    assert.equal(result.status, 'waiting_target', name); assert.equal(result.throttlePercent, 0, name);
  }
});

test('absolute waiting deadline cannot be refreshed by invalid observations', () => {
  const session = new IntermittentUwbFollow(prediction, calibration), c = controller();
  [1000, 1100, 1200].forEach((at) => run(session, c, at, telemetry(at)));
  run(session, c, 1600, lost(1600));
  const result = run(session, c, 6600, telemetry(6600));
  assert.equal(result.safe, false); assert.equal(result.reason, 'prediction_wait_timeout');
});

test('actuator governor caps existing limiter output and never increases thrust during prediction', () => {
  const s = new IntermittentUwbFollow(prediction, calibration);
  s.governDrive({ throttle: 70, steering: 10 }, { safe: true });
  const result = { safe: true, intermittentThrottleCap: 9, intermittentSteeringCap: 4, targetEstimate: { source: 'predicted' } };
  const first = s.governDrive({ throttle: 69, steering: 20 }, result);
  assert.equal(first.throttle, 9); assert.ok(first.steering <= first.throttle * 0.45);
  assert.equal(s.governDrive({ throttle: 70, steering: 0 }, { ...result, intermittentThrottleCap: 15 }).throttle, 9);
  s.governDrive(first, { safe: true, pausePropulsion: true });
  assert.equal(s.governDrive({ throttle: 70, steering: 10 }, result).throttle, 0);
});

test('reset, disarm, lost heartbeat and clock rollback cannot initiate predicted propulsion', () => {
  const s = new IntermittentUwbFollow(prediction, calibration), c = controller();
  [1000, 1100, 1200].forEach((at) => run(s, c, at, telemetry(at)));
  assert.equal(run(s, c, 1300, lost(1300, 1200, { armed: false })).safe, false);
  assert.equal(s.update({ controller: c, now: 1400, telemetry: lost(1400), connected: false }).safe, false);
  s.reset(); assert.equal(run(s, c, 1600, lost(1600)).status, 'waiting_target');
  assert.equal(run(s, c, 1500, telemetry(1500)).reason, 'prediction_clock_reset');
});

test('estimate cannot pass the original sensor gate with stale, future or disabled metadata', () => {
  const e = new UwbTargetEstimator(prediction, calibration); warm(e);
  const estimate = e.update(1600, lost(1600));
  assert.equal(usableControlEstimate(estimate, 1600, prediction), true);
  assert.equal(usableControlEstimate(estimate, 1700, prediction), false);
  assert.equal(usableControlEstimate({ ...estimate, lastMeasuredAt: 1700 }, 1600, prediction), false);
  assert.equal(usableControlEstimate(estimate, 1600, { enabled: false }), false);
});
