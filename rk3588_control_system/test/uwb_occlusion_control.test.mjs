import test from 'node:test';
import assert from 'node:assert/strict';
import {UwbGpsDriftHold} from '../backend/uwb_gps_drift_hold.js';
import {UwbRejectNmpc} from '../backend/uwb_reject_nmpc.js';
import {RawWindowRecovery} from '../backend/uwb_raw_recovery.js';

const config = { waiting_drift_hold: true, bearing_sign: 1, bearing_scale: 1, bearing_offset_deg: 0,
  reject_nmpc: { enabled: true, solve_budget_ms: 25 }, prediction: { measurement_gap_ms: 450 } };
function packet(at, state = 'clear', angle = 0) {
  return { attitude: { yaw: 0, yawRateDegS: 0 }, position: { lat: 30, lon: 120 },
    velocity: { vx: 0, vy: 0 }, uwb: { sensors: [{ receiverId: 1, tagId: 5313,
      receivedMeasurementAt: at, rawDistanceM: 6, rawAzimuthDeg: angle, valid: true,
      measurementStatus: 'trusted', occlusionDiagnostics: { state } }] } };
}
test('quarantined raw bearing cannot bypass XY gate through waiting steering', () => {
  const hold = new UwbGpsDriftHold(config);
  for (let at = 1000; at <= 1300; at += 100) hold.observeBearing(at, packet(at));
  assert.equal(hold.observeBearing(1300, packet(1300)).trusted, true);
  for (const state of ['suspect', 'quarantined', 'recovering']) {
    for (let at = 1400; at <= 2400; at += 100) {
      const direction = hold.observeBearing(at, packet(at, state, 20));
      assert.equal(direction.trusted, false);
      assert.equal(direction.reason, 'occlusion_quarantine');
      assert.equal(hold.bearings.length, 0);
    }
  }
  assert.equal(hold.observeBearing(2500, packet(2500)).trusted, false);
  assert.equal(hold.observeBearing(2600, packet(2600)).trusted, false);
  assert.equal(hold.observeBearing(2700, packet(2700)).trusted, true);
});
test('raw-window fallback clears isolated evidence and never samples quarantined points', () => {
  const recovery = new RawWindowRecovery({ measurement_gap_ms: 450 });
  recovery.reacquisition = [{ at: 900, position: { x: 6, y: 0 } }];
  const estimator = { lastPose: {}, sample() { throw new Error('suspect data reached raw fit'); } };
  assert.equal(recovery.observe(1000, packet(1000, 'quarantined'), estimator, {}), false);
  assert.equal(recovery.reacquisitionDiagnostic.reason, 'occlusion_quarantine');
  assert.deepEqual(recovery.reacquisition, []);
});
test('NMPC keeps old admitted position and ignores raw direction during quarantine', (t) => {
  // This checks admission geometry, not host scheduling. Deadline rejection is
  // covered separately in uwb_reject_nmpc.test.mjs with an advancing test clock.
  t.mock.method(performance, 'now', () => 0);
  const controller = new UwbRejectNmpc(config);
  const accepted = (at, range) => ({ safe: true, forwardSpeedMps: 0, yawRateDegS: 0,
    targetEstimate: { admissionVersion: 1, measurementConfirmed: true, referenceFrame: 'local_ned',
      lastMeasuredAt: at, evaluatedAt: at, relativeBodyM: { x: range, y: 0 }, uncertaintyM: .12,
      targetVelocityNedMps: { x: 0, y: 0 }, velocityEstimate: { trusted: true } } });
  controller.observe(1000, packet(1000), accepted(1000, 6));
  // Even contradictory trusted flags cannot override explicit quarantine.
  controller.observe(1100, packet(1100, 'quarantined', 20), accepted(1100, 15));
  assert.equal(controller.accepted.measurementAt, 1000);
  assert.equal(controller.accepted.x, 6);
  const result = controller.solve(1200, packet(1200, 'quarantined', 20),
    { trusted: true, at: 1200, headingDeg: 20, rateDegS: 0 });
  assert.equal(result.feasible, true); // Bounded old-posterior prediction remains supported.
  assert.equal(result.liveBearing, false);
});
