import assert from 'node:assert/strict';
import test from 'node:test';
import {loadTypeScript} from './helpers/load-typescript.mjs';
import '../frontend/js/uwb-raw-display.js';
const migrated = loadTypeScript('web/src/uwb-raw-display.ts').RawDisplay;
for (const Raw of [globalThis.MantaUwbRawDisplay, migrated]) {
const sensor = (at, overrides = {}) => ({ receiverId: 1, tagId: 5313, valid: true,
  rawDistanceM: 1.11, rawAzimuthDeg: 11, rawDistanceUpdatedAt: at,
  receivedMeasurementAt: at, ...overrides });
const packet = (...sensors) => ({ online: false, fresh: false, measurementStatus: 'suspect', sensors });

test('raw diagnostic frames continue at 40 Hz while every frame is rejected', () => {
  const display = new Raw();
  for (let i = 0; i < 100; i++) {
    const at = 1000 + i * 25, p = packet(sensor(at, { rawAzimuthDeg: i / 2 }));
    const before = structuredClone(p), result = display.view(p, at, true);
    assert.equal(result.valid, true); assert.equal(result.at, at);
    assert.equal(result.azimuthDeg, i / 2); assert.deepEqual(p, before);
  }
});

test('raw expiry, invalid geometry, radio invalidity and disconnect never show a stale target', () => {
  for (const fields of [{ valid: false }, { rawDistanceM: null }, { rawDistanceM: 0 },
    { rawAzimuthDeg: NaN }, { rawAzimuthDeg: 181 }, { receivedMeasurementAt: null },
    { receivedMeasurementAt: 2000 }]) {
    assert.equal(new Raw().view(packet(sensor(1000, fields)), 1000, true).valid, false);
  }
  const display = new Raw(), p = packet(sensor(1000));
  assert.equal(display.view(p, 1000, true).valid, true);
  assert.equal(display.view(p, 1701, true).valid, false);
  assert.equal(display.view(p, 1000, false).valid, false);
  assert.equal(display.samples.size, 0);
});

test('metadata and partial DIST do not retarget or freshen a complete raw frame', () => {
  const display = new Raw();
  display.view(packet(sensor(1000)), 1000, true);
  let result = display.view(packet(sensor(1000, { rawDistanceM: 8, rawAzimuthDeg: 80 })), 1025, true);
  assert.equal(result.distanceM, 1.11); assert.equal(result.azimuthDeg, 11); assert.equal(result.at, 1000);
  const partial = packet(sensor(1000, { rawDistanceUpdatedAt: 1025, rawDistanceM: 9 }));
  result = display.view(partial, 1025, true);
  assert.equal(result.distanceM, 1.11); assert.equal(result.at, 1000);
  assert.equal(new Raw().view(partial, 1025, true).valid, false);
  assert.equal(display.view(partial, 1701, true).valid, false);
  result = display.view(packet(sensor(1725, { rawDistanceM: 2 })), 1725, true);
  assert.equal(result.distanceM, 2); assert.equal(result.at, 1725);
});

test('raw values keep module angles, apply mount geometry only to plot and identify the receiver', () => {
  const display = new Raw();
  const result = display.view(packet(sensor(1000), sensor(1025, { receiverId: 2, tagId: 12,
    rawDistanceM: 2, rawAzimuthDeg: 0, mountYawDeg: 90, positionXM: 1, positionYM: .2 })), 1025, true);
  assert.equal(result.receiverId, 2); assert.equal(result.tagId, 12);
  assert.equal(result.azimuthDeg, 0); assert.equal(result.distanceM, 2);
  assert.ok(Math.abs(result.forward - 1) < 1e-10); assert.ok(Math.abs(result.left - 2.2) < 1e-10);
  const changed = display.view(packet(sensor(1025, { tagId: 22, rawDistanceM: 3 })), 1025, true);
  assert.equal(changed.distanceM, 3); assert.equal(changed.tagId, 22);
  assert.equal(display.samples.size, 1);
});

}
