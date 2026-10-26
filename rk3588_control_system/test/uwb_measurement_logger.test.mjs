import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import test from 'node:test';
import { UwbMeasurementLogger, MEASUREMENT_COLUMNS } from '../backend/uwb_measurement_logger.js';

function setup(t, options) {
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'uwb-measurements-'));
  const logger = new UwbMeasurementLogger(directory, options);
  t.after(async () => { await logger.stop(); fs.rmSync(directory, { recursive: true, force: true }); });
  return { logger, directory };
}
const observation = (id, extra = {}) => ({ recordId: id, sourceInstance: 'fcu-1', sourceSession: 1,
  receiverId: 1, status: 'accepted', rawDistanceM: 3.1, filteredDistanceM: 3.02,
  rawBearingDeg: 179, filteredBearingDeg: -179, receivedAt: 1000,
  sampleAt: 900 + id * 20, sourceTimeKind: 'lua_processing', ...extra });

test('same-batch measurements are distinct rows with raw, filtered, timing and reference errors', async t => {
  const { logger, directory } = setup(t);
  const started = await logger.start({ referenceDistanceM: 3, referenceBearingDeg: 180 }, 1000);
  for (let i = 1; i <= 4; i++) assert.equal(logger.record(observation(i), 1000), true);
  const stopped = await logger.stop();
  assert.equal(stopped.rows, 4);
  assert.equal(stopped.droppedRows, 0);
  const lines = fs.readFileSync(path.join(directory, started.fileName), 'utf8').trim().split('\n');
  assert.equal(lines.length, 5);
  assert.equal(lines[0], MEASUREMENT_COLUMNS.join(','));
  const rows = lines.slice(1).map(line => Object.fromEntries(MEASUREMENT_COLUMNS.map((key, i) => [key, line.split(',')[i]])));
  assert.deepEqual(rows.map(r => r.sampleAt), ['920', '940', '960', '980']);
  assert.equal(Number(rows[0].bearingErrorDeg), 1);
  assert.equal(Number(rows[0].rawBearingErrorDeg), -1);
  assert.ok(Math.abs(Number(rows[0].distanceErrorM) - .02) < 1e-8);
  assert.equal(JSON.parse(fs.readFileSync(path.join(directory, started.fileName + '.json'))).sourceTimeKind,
               'lua_processing_not_radio_sampling');
});

test('missing measurements remain blank; duplicates, sequence gaps and producer restarts are visible', async t => {
  const { logger, directory } = setup(t);
  const { fileName } = await logger.start({}, 1000);
  logger.record(observation(1));
  assert.equal(logger.record(observation(1)), false);
  logger.record(observation(4, { status: 'rejected', filteredDistanceM: null, reason: 'heading_unavailable' }));
  logger.record(observation(1, { sourceInstance: 'fcu-2' }));
  const state = await logger.stop();
  assert.equal(state.rows, 3);
  assert.equal(state.duplicates, 1);
  assert.equal(state.observedSequenceGaps, 2);
  const cells = fs.readFileSync(path.join(directory, fileName), 'utf8').trim().split('\n')[2].split(',');
  assert.equal(cells[MEASUREMENT_COLUMNS.indexOf('filteredDistanceM')], '');
  assert.equal(cells[MEASUREMENT_COLUMNS.indexOf('distanceErrorM')], '');
});

test('backpressure is bounded and reports dropped rows without blocking reception', async t => {
  const { logger } = setup(t, { maxPendingBytes: 1 });
  await logger.start();
  assert.equal(logger.record(observation(1)), false);
  assert.equal((await logger.stop()).droppedRows, 1);
});

test('overlapping start/stop operations serialize and both recordings remain readable', async t => {
  const { logger, directory } = setup(t);
  const [first, second] = await Promise.all([logger.start({ label: 'first' }), logger.start({ label: 'second' })]);
  logger.record(observation(1, { frameKey: [[1, 1], 20], reason: 'a,"quoted"\nvalue' }));
  await logger.stop();
  assert.notEqual(first.fileName, second.fileName);
  assert.ok(fs.existsSync(path.join(directory, first.fileName + '.summary.json')));
  const text = fs.readFileSync(path.join(directory, second.fileName), 'utf8');
  assert.match(text, /"\[\[1,1\],20\]"/);
  assert.match(text, /"a,""quoted""\nvalue"/);
});
