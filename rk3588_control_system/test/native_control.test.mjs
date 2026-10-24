import test from 'node:test';
import assert from 'node:assert/strict';
import dgram from 'node:dgram';
import fs from 'node:fs';
import { NativeControlClient } from '../backend/native_control.js';

async function fixture(t, respond, timeoutMs = 100) {
  const server = dgram.createSocket('udp4');
  await new Promise(resolve => server.bind(0, '127.0.0.1', resolve));
  const client = new NativeControlClient({ port: server.address().port, timeoutMs });
  server.on('message', (data, peer) => {
    const request = JSON.parse(data);
    respond(request, result => server.send(Buffer.from(JSON.stringify({ type: 'command_result', payload: { requestId: request.requestId, result } })), peer.port, peer.address));
  });
  t.after(() => { client.close(); server.close(); });
  return client;
}
test('native IPC correlates concurrent replies and preserves operation errors', async t => {
  const client = await fixture(t, (request, reply) => setTimeout(() => reply(request.command === 'ARM' ? { ok: false, error: 'denied' } : { ok: true, value: request.params.value }), request.command === 'ARM' ? 15 : 1));
  const [arm, status] = await Promise.all([client.command('ARM'), client.command('CONTROL_STATUS', { value: 12 })]);
  assert.deepEqual(arm, { ok: false, error: 'denied' }); assert.equal(status.value, 12); assert.equal(client.pending.size, 0);
});
test('lost IPC replies expire without retransmitting arm or start', async t => {
  let received = 0;
  const client = await fixture(t, () => received++, 30);
  assert.equal((await client.command('ARM')).error, 'native_control_timeout');
  assert.equal(received, 1); assert.equal(client.pending.size, 0);
});
test('start waits for native telemetry-confirmed completion and detects cancellation', async t => {
  let busy = true, cancelled = false;
  const client = await fixture(t, (request, reply) => {
    if (request.command === 'FOLLOW_START') return reply({ ok: true, pending: true, transitionId: 7 });
    reply({ ok: true, transitionId: cancelled ? 8 : 7, busy, state: { active: !busy, reason: cancelled ? 'manual_stop' : 'tracking' } });
    busy = false;
  });
  assert.equal((await client.start()).state.active, true);
  busy = true; cancelled = true;
  assert.equal((await client.start()).error, 'manual_stop');
});
test('closing Web IPC resolves pending callers without issuing vehicle commands', async t => {
  const requests = [];
  const client = await fixture(t, request => requests.push(request.command), 1000);
  const response = client.command('CONTROL_STATUS');
  await new Promise(resolve => setTimeout(resolve, 20)); client.close();
  assert.equal((await response).error, 'native_control_unavailable');
  assert.deepEqual(requests, ['CONTROL_STATUS']);
});
test('deployed Web owns no actuator/controller timer and systemd owns one native serial process', () => {
  const server = fs.readFileSync(new URL('../backend/server.js', import.meta.url), 'utf8');
  assert.doesNotMatch(server, /setInterval\((?:updateUwbFollow|updateManualRoverSlew)/);
  assert.doesNotMatch(server, /uwbFollowController\.(?:update|preflight)\(/);
  assert.doesNotMatch(server, /stopUwbFollow\('server_shutdown'/);
  assert.match(server, /nativeControl\.command\('MANUAL_TARGET'/);
  assert.match(server, /nativeControl\.start\(/);
  const unit = fs.readFileSync(new URL('../systemd/manta-bridge.service.template', import.meta.url), 'utf8');
  assert.match(unit, /native_control_service\.sh/); assert.doesNotMatch(unit, /mavlink_bridge\.py|PartOf=.*backend|BindsTo=.*backend/);
});
