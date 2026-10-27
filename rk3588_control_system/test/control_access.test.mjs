import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { createServer } from 'node:http';
import express from 'express';
import { Server } from 'socket.io';
import { createControlAccess, loadControlPin } from '../backend/control_access.js';
import { api, channel, pair, until } from './helpers/control_client.mjs';

async function fixture(t, { retain = false } = {}) {
  let clock = Date.now();
  const lost = [], commands = [], app = express(), server = createServer(app), io = new Server(server);
  const access = createControlAccess({ pin: '123456', now: () => clock, lifetimeMs: 60000,
    onControlLost: reason => { lost.push(reason); return retain; } });
  app.use(express.json()); access.installRoutes(app); app.use('/api', access.middleware);
  app.get('/api/status', (req, res) => res.json({ controlSession: access.state(req.controlSession, req.headers['x-manta-client']) }));
  app.post('/api/control/rover', (_req, res) => { commands.push('rover'); res.json({ success: true }); });
  app.post('/api/emergency/stop', (_req, res) => {
    commands.push('stop'); access.clearDisconnectedOwner('emergency_stop'); res.json({ success: true });
  });
  io.use(access.authenticateSocket);
  io.on('connection', socket => {
    socket.on('arm', () => { if (access.authorizeSocket(socket)) commands.push('arm'); });
    socket.on('disarm', () => { if (access.authorizeSocket(socket, false)) commands.push('disarm'); });
    socket.on('disconnect', () => access.disconnectSocket(socket));
  });
  await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
  t.after(async () => { access.close(); io.disconnectSockets(true); server.closeAllConnections(); await new Promise(resolve => io.close(resolve)); });
  return { base: `http://127.0.0.1:${server.address().port}`, lost, commands, advance: ms => { clock += ms; } };
}

test('private control PIN persists, stays out of configuration and rejects invalid overrides', t => {
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'manta-pin-'));
  t.after(() => fs.rmSync(directory, { recursive: true, force: true }));
  assert.equal(loadControlPin(directory, '012345'), '012345');
  assert.equal(fs.existsSync(path.join(directory, 'data/control-pin')), false);
  for (const value of ['', '12345', 'abcdef', '1234567']) assert.throws(() => loadControlPin(directory, value));
  const generated = loadControlPin(directory, undefined);
  assert.match(generated, /^\d{6}$/);
  assert.equal(loadControlPin(directory, undefined), generated);
  if (process.platform !== 'win32') assert.equal(fs.statSync(path.join(directory, 'data/control-pin')).mode & 0o777, 0o600);
});

test('PIN pairing is enforced, rate limited, and uses a private expiring cookie', async t => {
  const f = await fixture(t), request = api(f.base, '', 'test-page-0000000001');
  assert.equal((await request('/api/status')).status, 401);
  assert.equal((await request('/api/control/rover', { throttle: 50 })).status, 401);
  assert.equal((await request('/api/emergency/stop', {})).status, 401);
  const unauth = channel(t, f.base);
  await assert.rejects(unauth.ready, /Pair with the device/);
  for (let i = 0; i < 5; i++) assert.equal((await request('/api/auth/pair', { pin: '000000' })).status, 401);
  const locked = await request('/api/auth/pair', { pin: '123456' });
  assert.equal(locked.status, 429); assert.ok(Number(locked.headers.get('retry-after')) > 0);
  f.advance(15 * 60000);
  const success = await request('/api/auth/pair', { pin: '123456' });
  assert.equal(success.status, 200);
  assert.match(success.headers.get('set-cookie'), /^manta_session=[a-f0-9]{64}; Path=\/; HttpOnly; SameSite=Strict; Max-Age=60$/);
  assert.equal(success.headers.get('cache-control'), 'no-store');
  const payload = await success.text();
  assert.ok(!payload.includes('123456')); assert.ok(!payload.includes('manta_session='));
  assert.deepEqual(f.commands, []);
});

test('one page owns control; paired spectators and another tab sharing its cookie cannot control', async t => {
  const f = await fixture(t), ownerId = 'test-owner-0000000001', viewerId = 'test-viewer-000000001';
  const cookie = await pair(f.base, ownerId), ownerApi = api(f.base, cookie, ownerId), viewerApi = api(f.base, cookie, viewerId);
  assert.equal((await ownerApi('/api/control/claim', {})).status, 409, 'claim requires a live authenticated channel');
  const owner = channel(t, f.base, cookie, ownerId), spectator = channel(t, f.base, cookie, viewerId);
  await Promise.all([owner.ready, spectator.ready]);
  assert.equal((await ownerApi('/api/control/claim', {})).status, 200);
  assert.equal((await ownerApi('/api/control/rover', { throttle: 20 })).status, 200);
  assert.equal((await viewerApi('/api/status')).status, 200);
  assert.equal((await viewerApi('/api/control/claim', {})).status, 409);
  assert.equal((await viewerApi('/api/control/rover', { throttle: 90 })).status, 403);
  assert.equal((await viewerApi('/api/control/release', {})).status, 403);
  spectator.send('arm');
  await until(() => spectator.events.some(([name]) => name === 'error_message'));
  assert.deepEqual(f.commands, ['rover']);
  owner.send('arm'); spectator.send('disarm');
  await until(() => f.commands.includes('arm') && f.commands.includes('disarm'));
  assert.equal((await viewerApi('/api/emergency/stop', {})).status, 200);
  spectator.socket.close();
  await until(() => spectator.socket.readyState === 3);
  assert.deepEqual(f.lost, [], 'spectator disconnect must not stop the owner');
  const duplicate = channel(t, f.base, cookie, ownerId); await duplicate.ready;
  owner.socket.close(); await until(() => owner.socket.readyState === 3);
  assert.deepEqual(f.lost, [], 'reconnection overlap must not release the same page');
  duplicate.socket.close(); await until(() => f.lost.length === 1);
  assert.deepEqual(f.lost, ['transport_lost']);
  assert.equal((await ownerApi('/api/control/claim', {})).status, 409, 'late claim after cancellation cannot leave an owner behind');
  const newOwner = channel(t, f.base, cookie, viewerId); await newOwner.ready;
  assert.equal((await viewerApi('/api/control/claim', {})).status, 200);
});

test('session expiry and logout revoke channels and control; release stops a retained autonomous run', async t => {
  const f = await fixture(t, { retain: true }), client = 'test-owner-0000000001';
  const cookie = await pair(f.base, client), request = api(f.base, cookie, client);
  const owner = channel(t, f.base, cookie, client); await owner.ready;
  await request('/api/control/claim', {});
  owner.socket.close(); await until(() => f.lost.includes('transport_lost'));
  assert.equal((await (await request('/api/status')).json()).controlSession.canControl, true);
  assert.equal((await request('/api/control/rover', { throttle: 20 })).status, 403, 'transport loss revokes manual commands even for a retained autonomous owner');
  assert.equal((await request('/api/control/release', {})).status, 200);
  assert.ok(f.lost.includes('operator_release'));
  const reconnect = channel(t, f.base, cookie, client); await reconnect.ready;
  await request('/api/control/claim', {});
  f.advance(60001);
  assert.equal((await request('/api/status')).status, 401);
  assert.ok(f.lost.includes('session_expired'));
  await until(() => reconnect.socket.readyState === 3);
  const expired = channel(t, f.base, cookie, client); await assert.rejects(expired.ready, /Pair with the device/);
  const cookie2 = await pair(f.base, client), request2 = api(f.base, cookie2, client);
  const second = channel(t, f.base, cookie2, client); await second.ready;
  await request2('/api/control/claim', {});
  const logout = await request2('/api/auth/logout', {});
  assert.equal(logout.status, 200); assert.match(logout.headers.get('set-cookie'), /Max-Age=0/);
  assert.ok(f.lost.includes('operator_logout'));
  assert.equal((await request2('/api/status')).status, 401);
  await until(() => second.socket.readyState === 3);
});

test('paired emergency stop frees a disconnected autonomous owner so another page can take over', async t => {
  const f = await fixture(t, { retain: true }), ownerId = 'test-owner-0000000001', viewerId = 'test-viewer-000000001';
  const cookie = await pair(f.base, ownerId), ownerApi = api(f.base, cookie, ownerId), viewerApi = api(f.base, cookie, viewerId);
  const owner = channel(t, f.base, cookie, ownerId), viewer = channel(t, f.base, cookie, viewerId);
  await Promise.all([owner.ready, viewer.ready]); await ownerApi('/api/control/claim', {});
  assert.equal((await viewerApi('/api/emergency/stop', {})).status, 200);
  assert.equal((await viewerApi('/api/control/claim', {})).status, 409, 'online owner keeps control after emergency stop');
  owner.socket.close(); await until(() => f.lost.includes('transport_lost'));
  assert.equal((await viewerApi('/api/control/claim', {})).status, 409, 'autonomous owner is retained on transport loss');
  assert.equal((await viewerApi('/api/emergency/stop', {})).status, 200);
  assert.ok(f.lost.includes('emergency_stop'));
  assert.equal((await viewerApi('/api/control/claim', {})).status, 200);
});
