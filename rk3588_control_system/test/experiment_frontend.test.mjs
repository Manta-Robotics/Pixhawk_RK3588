import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import vm from 'node:vm';

const apiCode = fs.readFileSync(new URL('../frontend/js/experiment-api.js', import.meta.url), 'utf8');
const transportCode = fs.readFileSync(new URL('../frontend/js/manta-app-transport.js', import.meta.url), 'utf8');
const tick = () => new Promise(resolve => setImmediate(resolve));
const wireSnapshot = (server_epoch, seq) => ({ schema_version: 2, server_epoch, seq, modules: [], active_alerts: [] });
const response = (payload, status = 200) => ({ ok: status < 400, status, json: async () => payload });

function loadClient(fetch) {
  const scope = { window: {}, console, Date, performance, AbortController, fetch, URLSearchParams,
    setTimeout, clearTimeout, setInterval, clearInterval };
  vm.runInNewContext(apiCode, scope);
  vm.runInNewContext(transportCode, scope);
  return scope.window;
}

function fixture() {
  const calls = [];
  const results = [];
  const api = { snapshot: signal => new Promise(resolve => calls.push({ resolve, signal })) };
  const { MantaExperimentApi } = loadClient();
  let time = 0;
  const feed = new MantaExperimentApi.SnapshotFeed(api, (event, data) => results.push({ event, data }), () => time);
  return { calls, results, feed, clock: value => { time = value; } };
}

function applied(f) {
  return f.results.filter(item => item.event === 'experimentSnapshot').map(item => [item.data.server_epoch, item.data.seq]);
}

test('GET establishes the epoch and only the newest buffered snapshot is applied', async () => {
  const f = fixture();
  f.feed.open();
  f.feed.receive(wireSnapshot('older', 500));
  f.feed.receive(wireSnapshot('first', 9));
  f.feed.receive(wireSnapshot('first', 8));
  await tick();
  assert.equal(f.calls.length, 1);
  f.calls[0].resolve(wireSnapshot('first', 6));
  await tick();
  f.feed.receive(wireSnapshot('first', 7));
  f.feed.receive(wireSnapshot('first', 10));
  assert.deepEqual(applied(f), [['first', 9], ['first', 10]]);
  f.feed.close();
});

test('reconnect rejects an aborted socket callback, confirms the new epoch and retires the old epoch', async () => {
  const f = fixture();
  f.feed.open(); await tick();
  f.calls[0].resolve(wireSnapshot('first', 12)); await tick();
  f.clock(2000); f.feed.poll(); await tick();
  f.feed.close();
  assert.equal(f.calls[1].signal.aborted, true);
  f.feed.open(); f.feed.receive(wireSnapshot('second', 3)); await tick();
  assert.equal(f.calls.length, 2);
  f.calls[1].resolve(wireSnapshot('first', 14)); await tick();
  assert.equal(f.calls.length, 3);
  f.calls[2].resolve(wireSnapshot('second', 1)); await tick();
  f.feed.receive(wireSnapshot('first', 900));
  f.feed.receive(wireSnapshot('second', 2));
  assert.deepEqual(applied(f), [['first', 12], ['second', 3]]);
  assert.equal(f.calls.length, 3);
  f.feed.close();
});

test('read-only pairing requests authorized reads without claiming control or opening the gimbal link', async () => {
  const requests = [];
  const { MantaAppTransport } = loadClient(async (url, options) => {
    requests.push({ url, options });
    return response({ success: true, data: url === '/api/experiments/live' ? wireSnapshot('first', 1) : {} });
  });
  const live = MantaAppTransport.create('live');
  live.connectSocket = () => {
    live.socket = { connected: true, emit() {}, disconnect() {} };
    live.experimentFeed.open();
  };
  live.startPolling = live.startDriveKeepalive = () => {};
  let opens = 0;
  live.openGimbalLink = () => { opens++; };
  await live.pair('123456');
  await live.connect({ readOnly: true });
  await tick();
  assert.deepEqual(requests.map(call => call.url), ['/api/auth/pair', '/api/status', '/api/experiments/live']);
  assert.ok(requests.every(call => call.options.credentials === 'same-origin' && call.options.headers['X-Manta-Client']));
  assert.equal(opens, 0);
  assert.equal(live.canControl, false);
  await live.disconnect();
});

test('the historical client sends the query cursor, zero time, and distinct locate modes', async () => {
  const paths = [];
  const { MantaExperimentApi } = loadClient();
  const api = MantaExperimentApi.liveApi(async path => { paths.push(path); return response({ success: true, data: {} }); });
  await api.events('s/a', { after: 10, module: 'uwb', level: 'warn' });
  await api.locate('s/a', { t_ms: 0 });
  await api.locate('s/a', { clip_id: 'clip', play_us: 0 });
  await api.locate('s/a', { clip_id: 'clip', display_no: 0 });
  assert.deepEqual(paths, ['/api/experiments/sessions/s%2Fa/events?after=10&module=uwb&level=warn',
    '/api/experiments/sessions/s%2Fa/locate?t_ms=0', '/api/experiments/sessions/s%2Fa/locate?clip_id=clip&play_us=0',
    '/api/experiments/sessions/s%2Fa/locate?clip_id=clip&display_no=0']);
});
