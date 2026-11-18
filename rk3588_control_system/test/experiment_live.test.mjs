import test from 'node:test';
import assert from 'node:assert/strict';
import { ExperimentLive, schema } from '../backend/experiment_live.js';
function rig() {
  let now = 10_000_000_000n;
  const logs = [],
    push = [];
  const live = new ExperimentLive({
    clock: () => now,
    boot: 'board',
    journal: (kind, payload) => logs.push({ kind, payload }),
    publish: (s) => push.push(s)
  });
  return {
    live,
    logs,
    push,
    at: (ms) => {
      now = BigInt(ms) * 1_000_000n;
    },
    now: () => String(now)
  };
}
function observe(r, source, id, values, meta = {}, overrides = {}) {
  return r.live.ingest({
    source,
    sample_id: id,
    obs_ns: r.now(),
    time_kind: 'host_receive',
    quality: 'measured',
    values,
    meta,
    ...overrides
  });
}
test('UWB alert escalation and recovery use unique measured samples and exact inclusive limits', () => {
  const r = rig(),
    { live, logs } = r;
  live.setExpected('uwb', true);
  live.enter('trial', r.now(), null);
  observe(r, 'uwb', 'u1', { uwb_range_m: 4.2, uwb_bearing_deg: 0 });
  for (const [ms, state] of [
    [10350, 'ok'],
    [10351, 'warn'],
    [11050, 'warn'],
    [11051, 'error']
  ]) {
    r.at(ms);
    live.check();
    assert.equal(live.snapshot().modules.find((m) => m.id === 'uwb').state, state);
  }
  observe(r, 'uwb', 'u1', { uwb_range_m: 5 });
  observe(
    r,
    'uwb_pred',
    'p1',
    { uwb_pred_range_m: 5 },
    { based_on_sample_id: 'u1' },
    { quality: 'predicted' }
  );
  live.overview();
  assert.equal(logs.at(-1).payload.uwb_sample_id, 'u1');
  assert.equal(logs.at(-1).payload.uwb_range_m, null);
  r.at(11052);
  observe(r, 'uwb', 'u2', { uwb_range_m: 0 });
  live.check();
  assert.equal(live.snapshot().active_alerts[0].level, 'error');
  r.at(11053);
  observe(r, 'uwb', 'u3', { uwb_range_m: 0 });
  live.check();
  const e = logs.filter((e) => e.kind === 'event').map((e) => e.payload);
  assert.deepEqual(
    e.map((x) => x.type),
    ['alert_open', 'alert_level', 'alert_close']
  );
  assert.equal(new Set(e.map((x) => x.alert_id)).size, 1);
  assert.equal(live.values().row.uwb_range_m, 0);
});
test('only confirmed axes support bearing difference and retained visual results are causal', () => {
  const r = rig(),
    { live } = r;
  observe(r, 'uwb', '1', { uwb_bearing_deg: -179, uwb_range_m: 4 }, { calibration_id: 'mount-u' });
  observe(r, 'gimbal', '2', { gimbal_yaw_deg: 179 }, { calibration_id: 'mount-a8' });
  assert.equal(live.values().row.bearing_error_deg, 2);
  r.at(10008);
  observe(r, 'uwb', '3', { uwb_bearing_deg: 0, uwb_range_m: 4 }, {});
  assert.equal(live.values().row.bearing_error_deg, null);
  live.visionWorker('start', { worker_epoch: 'w1', mode_generation: 1 });
  r.at(10050);
  assert.equal(live.values().row.vision_state, undefined);
  r.at(10180);
  live.visionResult(
    { frame_id: 'f42', state: 'no_target', targets: [], worker_epoch: 'w1', mode_generation: 1 },
    { obs_ns: '10000000000', time_kind: 'host_decode' }
  );
  assert.equal(live.values().row.detected, false);
  assert.equal(live.values().row.target_count, 0);
  live.visionResult(
    { frame_id: 'f43', state: 'detected', targets: [], worker_epoch: 'old', mode_generation: 0 },
    { obs_ns: '10180000000', time_kind: 'host_decode' }
  );
  assert.equal(live.values().row.vision_frame_id, 'f42');
  const s = live.snapshot();
  assert.deepEqual(
    s.modules.map((m) => m.id),
    ['fc', 'mode', 'imu', 'uwb', 'gimbal', 'vision', 'motors', 'video', 'logger']
  );
});
test('rejected and applied mode transitions between overview rows keep distinct request IDs', () => {
  const r = rig(),
    { live, logs } = r;
  live.enter('session', r.now(), null);
  live.modes({ follow_active: false, follow_side: 'rear' });
  const a = live.modeRequest('follow_side', 'other');
  live.modeOutcome('follow_side', a, 'rejected', 'invalid');
  const b = live.modeRequest('follow_side', 'left');
  live.modes({ follow_side: 'left' });
  live.overview();
  const row = logs.at(-1).payload;
  assert.deepEqual(Object.keys(row), schema.overview);
  assert.equal(row.follow_active, false);
  assert.equal(row.follow_side, 'left');
  const events = logs.filter((x) => x.kind === 'event').map((x) => x.payload);
  assert.ok(
    events.some((x) => x.type === 'mode_rejected' && x.detail_json.actual === 'rear' && x.request_id === a)
  );
  assert.ok(
    events.some((x) => x.type === 'mode_applied' && x.detail_json.actual === 'left' && x.request_id === b)
  );
  assert.equal(live.snapshot().active_alerts.length, 0);
});
