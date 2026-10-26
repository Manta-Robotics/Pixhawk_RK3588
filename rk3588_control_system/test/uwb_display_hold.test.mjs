import assert from 'node:assert/strict';
import test from 'node:test';
import '../frontend/js/uwb-display-smoother.js';

test('last-position memory never turns a missing observation into a fresh one', () => {
  const hold = new MantaUwbDisplayHold();
  assert.equal(hold.view({ valid: false }, true, '1:a').valid, false);
  const sample = { valid: true, distanceM: 2, azimuthDeg: 30, measurementUpdatedAt: 1000 };
  assert.equal(hold.view(sample, true, '1:a').stale, false);
  for (let i = 0; i < 100; i++) {
    const view = hold.view({ valid: false }, true, '1:a');
    assert.equal(view.stale, true); assert.equal(view.measurementUpdatedAt, 1000);
    assert.equal(view.distanceM, 2);
  }
  assert.equal(sample.stale, undefined);
});

test('disconnect and receiver or tag identity changes clear all old position memory', () => {
  const hold = new MantaUwbDisplayHold(), sample = { valid: true, distanceM: 2 };
  hold.view(sample, true, '1:a');
  assert.equal(hold.view({ valid: false }, true, '1:b').valid, false);
  hold.view(sample, true, '1:b');
  assert.equal(hold.view({ valid: false }, true, '2:b').valid, false);
  hold.view(sample, true, '2:b');
  assert.equal(hold.view(sample, false, '2:b').valid, false);
  assert.equal(hold.view({ valid: false }, true, '2:b').valid, false);
});

test('recovery replaces the retained point without mutating the new measurement', () => {
  const hold = new MantaUwbDisplayHold();
  hold.view({ valid: true, distanceM: 2 }, true, '1:a');
  hold.view({ valid: false }, true, '1:a');
  const sample = { valid: true, distanceM: 3 }, before = structuredClone(sample);
  assert.equal(hold.view(sample, true, '1:a').stale, false);
  assert.equal(hold.view({ valid: false }, true, '1:a').distanceM, 3);
  assert.deepEqual(sample, before);
});
