import {UwbGpsDriftHold} from '../backend/uwb_gps_drift_hold.js';
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import { IntermittentUwbFollow } from '../backend/uwb_target_prediction.js';
import { UwbFollowController } from '../backend/uwb_follow.js';
import { UwbAutoResume } from '../backend/uwb_auto_resume.js';

const source = fs.readFileSync(new URL('../backend/server.js', import.meta.url), 'utf8');
const update = source.slice(source.indexOf('function updateUwbFollow('), source.indexOf('function updateTelemetry('));
const calibration = { bearing_sign: 1, bearing_scale: 1, bearing_offset_deg: 0 };
function t(now, valid = true) {
  return { armed: true, flightMode: 'MANUAL', pilotSteering: { ready: true, type: 0 },
    attitude: { yaw: 0, updatedAt: now }, velocity: { vx: 0, vy: 0 }, position: { updatedAt: now },
    ekf: { healthy: true, updatedAt: now }, gps: { fixType: 0 },
    uwb: { online: valid, fresh: valid, distanceM: 0.8, azimuthDeg: 0,
      updatedAt: valid ? now : 1200, measurementUpdatedAt: valid ? now : 1200 } };
}

test('when automatic resume is disabled the server expires a loss wait without rearming', () => {
  const commands = [], samples = [];
  const c = new UwbFollowController({ ...calibration, prediction: { enabled: true }, manual_propulsion: 'single_side',
    following_distance_m: 0.5, hard_stop_enabled: false, controller_profile: 'traditional_fast' });
  const session = new IntermittentUwbFollow({ enabled: true }, calibration);
  const sandbox = { now: 1000, Date: { now: () => sandbox.now }, uwbGpsDriftHold:new UwbGpsDriftHold(),uwbAutoResume:new UwbAutoResume(), systemState: { isConnected: true, telemetry: t(1000) },
    uwbFollowState: { active: true, status: 'tracking' }, uwbFollowController: c, intermittentUwbFollow: session,
    uwbFollowControlMode: 'relative_manual', uwbFollowTransitionId: 0, uwbFollowTransitionBusy: false,
    uwbHoldingEnteredAt: 0, uwbResumeStartedAt: 0,
    sharedUwbControlTelemetry: () => sandbox.systemState.telemetry,
    requireUwbFollowStartClearance: () => c.preflight({ now: sandbox.now, telemetry: sandbox.systemState.telemetry, connected: true }),
    idleUwbFollowResult: (reason) => ({ reason, safe: false }),
    publishUwbFollowState: (result, active) => { sandbox.uwbFollowState = { ...result, active }; },
    sendUwbFollowStop: (reason) => { session.governDrive({}, { pausePropulsion: true }); commands.push(['neutral', reason]); },
    disarmAndReturnManual: (reason) => { commands.push(['disarm', reason]); sandbox.systemState.telemetry.armed = false; },
    maintainUwbHoldingMode: () => true, maintainUwbResumeTransition: () => {}, resetUwbHoldTransition: () => {},
    sendUwbRelativeDrive: (result) => ({ safe: true, ...session.governDrive({ throttle: result.throttlePercent, steering: result.steeringDegrees }, result) }),
    finishUwbTestLog: () => {}, recordUwbTestSample: (result) => samples.push(result.status), addLog: () => {} };
  const context = vm.createContext(sandbox);
  vm.runInContext(update, context);
  const step = (now, valid) => { sandbox.now = now; sandbox.systemState.telemetry = t(now, valid); vm.runInContext('updateUwbFollow()', context); };
  [1000, 1100, 1200].forEach((now) => step(now, true));
  step(1600, false);
  assert.equal(sandbox.uwbFollowState.active, true); assert.equal(sandbox.uwbFollowState.status, 'waiting_target');
  assert.equal(sandbox.systemState.telemetry.armed, true); assert.ok(samples.includes('waiting_target'));
  assert.equal(commands.filter(([type]) => type === 'disarm').length, 0);
  step(6600, false);
  assert.equal(sandbox.uwbFollowState.active, false);
  assert.equal(sandbox.systemState.telemetry.armed, false);
  assert.ok(commands.some(([type, reason]) => type === 'disarm' && reason === 'prediction_wait_timeout'));
  const count = commands.length;
  step(6700, true); step(6800, true); step(6900, true);
  assert.equal(sandbox.uwbFollowState.active, false); assert.equal(commands.length, count);
});
