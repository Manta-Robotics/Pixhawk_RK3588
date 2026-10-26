import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import vm from 'node:vm';

test('burst of telemetry and hardware events paints once with the latest state', () => {
  const html = fs.readFileSync(new URL('../frontend/mobile-preview-kimi-k26.html', import.meta.url), 'utf8');
  const code = html.slice(html.indexOf('var telemetryPaintTimer='), html.indexOf('function ensureTransport(){'));
  let callback, scheduled = 0;
  const calls = [];
  const scope = { setTimeout: (fn, ms) => { assert.equal(ms, 50); callback = fn; scheduled++; return scheduled; },
    renderConnection: () => calls.push('hardware'), renderUwbFollow: () => calls.push('follow'),
    renderTelemetry: () => calls.push('telemetry'), renderGpsStatus: () => calls.push('gps') };
  vm.createContext(scope); vm.runInContext(code, scope);
  for (let i = 0; i < 100; i++) scope.scheduleTelemetryPaint(i === 10, true);
  assert.equal(scheduled, 1); assert.deepEqual(calls, []);
  callback(); assert.deepEqual(calls, ['hardware', 'telemetry', 'gps']);
  scope.scheduleTelemetryPaint(false, true); assert.equal(scheduled, 2);
});

test('slow polling has one in-flight request and aborts on deadline', async () => {
  let tick, deadline, fetches = 0;
  const scope = { window: {}, console, Date, performance: { now: () => 100 }, AbortController,
    setInterval: fn => { tick = fn; return 1; }, clearInterval() {},
    setTimeout: fn => { deadline = fn; return 1; }, clearTimeout() {},
    fetch: (url, options) => { fetches++; return new Promise((resolve, reject) => {
      options.signal.addEventListener('abort', () => reject(new Error('aborted')));
    }); } };
  vm.runInNewContext(fs.readFileSync(new URL('../frontend/js/manta-app-transport.js', import.meta.url), 'utf8'), scope);
  const transport = scope.window.MantaAppTransport.create('live'); transport.connected = true;
  transport.startPolling(); tick(); tick(); tick();
  assert.equal(fetches, 1); assert.equal(transport.pollInFlight, true);
  deadline(); await new Promise(resolve => setImmediate(resolve));
  assert.equal(transport.pollInFlight, false); assert.equal(transport.pollFailures, 1);
  tick(); assert.equal(fetches, 2);
  deadline(); await new Promise(resolve => setImmediate(resolve));
});
