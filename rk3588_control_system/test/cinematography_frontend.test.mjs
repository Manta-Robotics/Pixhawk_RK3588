import assert from 'node:assert/strict';
import test from 'node:test';
import { canRequestCinematography, normalizeCinematography } from '../frontend/js/cinematography-panel.js';

const control = { mode: 'live', connected: true, canControl: true };

test('unavailable strategy and foreign schema leave every action disabled', () => {
  for (const data of [null, { schema_version: 2, available: true, can_start: true },
    { schema_version: 1, available: false, can_start: true, can_stop: true }]) {
    const state = normalizeCinematography(data);
    assert.equal(state.available, false);
    assert.equal(state.mode, null);
    assert.equal(canRequestCinematography(state, control, 'start'), false);
    assert.equal(canRequestCinematography(state, control, 'stop'), false);
  }
});

test('future strategy enables only the advertised action for the live owner', () => {
  const state = normalizeCinematography({ schema_version: 1, available: true, state: 'idle', modes: ['follow', 'orbit', 'follow', null], can_start: true, can_stop: false });
  assert.deepEqual(state.modes, ['follow', 'orbit']);
  assert.equal(canRequestCinematography(state, control, 'start'), true);
  assert.equal(canRequestCinematography(state, control, 'stop'), false);
  for (const viewer of [{ ...control, canControl: false }, { ...control, mode: 'mock' }, { ...control, connected: false }]) {
    assert.equal(canRequestCinematography(state, viewer, 'start'), false);
  }
});
