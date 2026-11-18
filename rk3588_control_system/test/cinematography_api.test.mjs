import test from 'node:test';
import assert from 'node:assert/strict';
import express from 'express';
import { installCinematographyApi } from '../backend/cinematography_api.js';

async function mount(t, provider) {
  const app = express();
  app.use(express.json());
  installCinematographyApi(app, provider);
  const server = await new Promise((resolve) => {
    const listener = app.listen(0, '127.0.0.1', () => resolve(listener));
  });
  t.after(() => server.close());
  return (name, body) =>
    fetch(
      `http://127.0.0.1:${server.address().port}/api/cinematography/${name}`,
      body == null
        ? {}
        : { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) }
    );
}
test('reserved cinematography capability is explicit; neither control command can activate an absent policy', async (t) => {
  const api = await mount(t, null);
  const { data } = await (await api('state')).json();
  assert.deepEqual(
    [data.available, data.state, data.can_start, data.can_stop, data.modes],
    [false, 'unavailable', false, false, []]
  );
  for (const action of ['start', 'stop']) {
    const response = await api(action, {});
    assert.equal(response.status, 409);
    assert.equal((await response.json()).code, 'CINEMATOGRAPHY_UNAVAILABLE');
  }
});
test('a provider advertises its modes and controls transition state from authoritative feedback', async (t) => {
  let mode = null;
  const api = await mount(t, {
    state: () => ({
      available: true,
      state: mode ? 'running' : 'idle',
      mode,
      modes: ['auto'],
      can_start: !mode,
      can_stop: !!mode,
      reason: null
    }),
    start: async (req) => {
      mode = req.mode;
    },
    stop: async () => {
      mode = null;
    }
  });
  assert.equal((await api('start', { mode: 'unknown' })).status, 400);
  assert.equal(mode, null);
  assert.equal((await (await api('start', { mode: 'auto' })).json()).data.state, 'running');
  assert.equal((await api('start', {})).status, 409);
  assert.equal((await (await api('stop', {})).json()).data.state, 'idle');
});
