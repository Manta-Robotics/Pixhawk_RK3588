import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import vm from 'node:vm';

const source = fs.readFileSync(new URL('../frontend/js/manta-app-transport.js', import.meta.url), 'utf8');
const tick = () => new Promise(resolve => setImmediate(resolve));
function transport(fetch) {
  const scope = { window: {}, console, Date, performance, AbortController, fetch,
    setTimeout, clearTimeout, setInterval, clearInterval };
  vm.runInNewContext(source, scope);
  return scope.window.MantaAppTransport.create('live');
}
const response = (body, status = 200) => ({ ok: status < 400, status, json: async () => body });

test('live pairing submits the PIN to the server and propagates invalid-code responses', async () => {
  const calls = [], live = transport(async (url, options) => {
    calls.push({ url, options }); return response({ success: false, message: 'Incorrect pairing code' }, 401);
  });
  await assert.rejects(live.pair('123'), /六位/); assert.equal(calls.length, 0);
  await assert.rejects(live.pair('123456'), /Incorrect pairing code/);
  assert.equal(calls[0].url, '/api/auth/pair');
  assert.equal(JSON.parse(calls[0].options.body).pin, '123456');
  assert.equal(calls[0].options.credentials, 'same-origin');
  assert.match(calls[0].options.headers['X-Manta-Client'], /^[a-zA-Z0-9_-]{16,80}$/);
});

test('cancelled initial status response cannot resurrect a connection or request control', async () => {
  let finish, sockets = 0, claims = 0;
  const live = transport(() => new Promise(resolve => { finish = resolve; }));
  live.connectSocket = () => { sockets++; }; live.claimControl = () => { claims++; };
  const pending = live.connect(); const rejected = assert.rejects(pending, /连接已取消/);
  await tick(); await live.disconnect();
  finish(response({ success: true, data: {} })); await rejected;
  assert.equal(live.connected, false); assert.equal(sockets, 0); assert.equal(claims, 0);
});

test('connection completes while gimbal open is pending; another owner produces read-only state', async () => {
  for (const readonly of [false, true]) {
    const live = transport(async url => url === '/api/status' ? response({ success: true, data: {} })
      : readonly ? response({ success: false, message: 'Another page already owns control' }, 409)
        : response({ success: true, controlSession: { revision: 1, canControl: true } }));
    live.connectSocket = () => { live.socket = { connected: true, emit() {}, disconnect() {} }; };
    live.controlRevision = 99; // Reusing a transport after backend restart must accept new ownership revisions.
    live.startPolling = live.startDriveKeepalive = () => {};
    let opens = 0; live.openGimbalLink = () => { opens++; return new Promise(() => {}); };
    await live.connect();
    assert.equal(live.connected, true); assert.equal(live.canControl, !readonly); assert.equal(opens, readonly ? 0 : 1);
    await live.disconnect();
  }
});

test('a delayed control-claim response cannot restore authority after disconnect', async () => {
  let finish;
  const live = transport(async url => url === '/api/status' ? response({ success: true, data: {} })
    : new Promise(resolve => { finish = resolve; }));
  live.connectSocket = () => { live.socket = { connected: true, emit() {}, disconnect() {} }; };
  const pending = live.connect(), rejected = assert.rejects(pending, /连接已取消/);
  await tick(); await live.disconnect();
  finish(response({ success: true, controlSession: { revision: 2, canControl: true } }));
  await rejected;
  assert.equal(live.connected, false); assert.equal(live.canControl, false);
});

test('late session events cannot restore an old owner; read-only pages send no movement', () => {
  const live = transport(() => { throw new Error('Read-only page must not send HTTP commands'); });
  const sent = [];
  live.connected = true; live.socket = { connected: true, emit: (...event) => sent.push(event) };
  live.applyControlSession({ revision: 2, canControl: false });
  live.applyControlSession({ revision: 1, canControl: true }); live.drive({ x: 1, y: -1 });
  assert.equal(live.canControl, false); assert.deepEqual(sent, []);
  live.applyControlSession({ revision: 3, canControl: true }); live.drive({ x: 1, y: 0 });
  assert.equal(sent.length, 1);
  live.applyControlSession({ revision: 4, canControl: false });
  assert.equal(live.driveVector.x, 0); assert.equal(live.driveVector.y, 0);
});
