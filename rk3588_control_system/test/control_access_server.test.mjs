// Only random loopback ports and a fake UDP bridge are used in this test.
import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import http from 'node:http';
import dgram from 'node:dgram';
import { spawn } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { api, channel, delay, pair, until } from './helpers/control_client.mjs';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
test('production backend rejects unpaired and spectator commands, preserves owner motion, and stops on owner loss', { timeout: 14000 }, async t => {
  const directory = await fs.mkdtemp(path.join(os.tmpdir(), 'manta-control-access-'));
  t.after(() => fs.rm(directory, { recursive: true, force: true }));
  const receiver = dgram.createSocket('udp4');
  await new Promise(resolve => receiver.bind(0, '127.0.0.1', resolve));
  t.after(() => receiver.close());
  const commands = []; receiver.on('message', data => commands.push(JSON.parse(data)));
  const telemetry = dgram.createSocket('udp4');
  await new Promise(resolve => telemetry.bind(0, '127.0.0.1', resolve));
  const telemetryPort = telemetry.address().port;
  await new Promise(resolve => telemetry.close(resolve));
  const web = http.createServer(); await new Promise(resolve => web.listen(0, '127.0.0.1', resolve));
  const port = web.address().port; await new Promise(resolve => web.close(resolve));
  const config = JSON.parse(await fs.readFile(path.join(root, 'config/system.config.json'), 'utf8'));
  Object.assign(config, { web_host: '127.0.0.1', web_port: port, bridge_host: '127.0.0.1',
    bridge_command_port: receiver.address().port, bridge_telemetry_port: telemetryPort,
    logs_dir: path.join(directory, 'logs'), rover_command_timeout_ms: 2000 });
  config.gimbal.enabled = false; config.gimbal.auto_connect = false;
  config.gimbal.face.keep_warm = false; config.gimbal.face.prewarm = false;
  config.camera.enabled = false; config.hotspot.managed_externally = true;
  config.recordings.dir = path.join(directory, 'recordings');
  const configFile = path.join(directory, 'config.json'); await fs.writeFile(configFile, JSON.stringify(config));
  const env = { ...process.env, MANTA_CONFIG_FILE: configFile, MANTA_CONTROL_PIN: '123456' }; delete env.MANTA_SIMULATION;
  const child = spawn(process.execPath, ['backend/server.js'], { cwd: root, env, windowsHide: true, stdio: ['ignore', 'pipe', 'pipe'] });
  let output = ''; child.stdout.on('data', chunk => { output += chunk; }); child.stderr.on('data', chunk => { output += chunk; });
  const exited = new Promise(resolve => child.once('exit', resolve));
  t.after(async () => { if (child.exitCode === null) child.kill('SIGKILL'); await exited; });
  const base = `http://127.0.0.1:${port}`, deadline = Date.now() + 6000;
  for (;;) {
    try { if ((await fetch(base + '/health')).ok) break; } catch {}
    assert.equal(child.exitCode, null, output); assert.ok(Date.now() < deadline, output); await delay(20);
  }
  const sendTelemetry = payload => new Promise((resolve, reject) => receiver.send(Buffer.from(JSON.stringify({ type: 'telemetry', payload })),
    telemetryPort, '127.0.0.1', error => error ? reject(error) : resolve()));
  await sendTelemetry({ armed: true, flightMode: 'MANUAL', pilotSteering: { type: 0, ready: true } });
  const unauthApi = api(base, '', 'test-unpaired-0000001');
  for (const route of ['/api/control/rover', '/api/control/motor', '/api/uwb-follow/start', '/api/system/reboot', '/api/emergency/stop']) {
    assert.equal((await unauthApi(route, { throttle: 70, channel: 1, pwm: 1700 })).status, 401, route);
  }
  const unauth = channel(t, base); await assert.rejects(unauth.ready, /Pair with the device/);
  const compressed = await fetch(base + '/api/auth/pair', { method: 'POST',
    headers: { 'Content-Type': 'application/json', 'Content-Encoding': 'gzip' }, body: 'compressed input is unsupported' });
  assert.equal(compressed.status, 415, 'compressed control requests are rejected before decompression');
  const oversized = await fetch(base + '/api/auth/pair', { method: 'POST',
    headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ pin: 'x'.repeat(1024 * 1024) }) });
  assert.equal(oversized.status, 413, 'oversized control requests are rejected');
  assert.ok(!commands.some(packet => packet.command === 'ARM'));
  const ownerId = 'test-owner-0000000001', viewerId = 'test-viewer-000000001';
  const ownerCookie = await pair(base, ownerId), viewerCookie = await pair(base, viewerId);
  const ownerApi = api(base, ownerCookie, ownerId), viewerApi = api(base, viewerCookie, viewerId);
  const owner = channel(t, base, ownerCookie, ownerId), viewer = channel(t, base, viewerCookie, viewerId);
  await Promise.all([owner.ready, viewer.ready]);
  assert.equal((await ownerApi('/api/control/claim', {})).status, 200);
  assert.equal((await viewerApi('/api/control/claim', {})).status, 409);
  for (const route of ['/api/control/rover', '/api/control/motor', '/api/uwb-follow/start', '/api/system/reboot']) {
    assert.equal((await viewerApi(route, { throttle: 70, channel: 1, pwm: 1700 })).status, 403, route);
  }
  for (const name of ['arm', 'rover_drive', 'motor_control']) viewer.send(name, { throttle: 70, channel: 1, pwm: 1700 });
  await until(() => viewer.events.filter(([name]) => name === 'error_message').length === 3);
  assert.ok(!commands.some(packet => packet.command === 'ARM'));
  // An owner can send ARM through the same production path (fake telemetry is disarmed first).
  await sendTelemetry({ armed: false }); await delay(20);
  owner.send('arm'); await until(() => commands.some(packet => packet.command === 'ARM'));
  await sendTelemetry({ armed: true }); await delay(20);
  owner.send('rover_drive', { throttle: 25, steering: 0 });
  await until(() => commands.some(packet => packet.command === 'ROVER_DRIVE' && packet.params?.throttle > 0));
  viewer.socket.close(); await until(() => viewer.socket.readyState === 3); await delay(40);
  assert.ok(!commands.some(packet => packet.params?.reason === 'control_owner_disconnected'), 'spectator disconnect must not stop motion');
  assert.ok((await (await ownerApi('/api/status')).json()).data.roverControl.throttle > 0);
  owner.socket.close(); await until(() => commands.some(packet => packet.command === 'ROVER_DRIVE' && packet.params?.reason === 'control_owner_disconnected'));
  const stopped = commands.findLast(packet => packet.params?.reason === 'control_owner_disconnected');
  assert.equal(stopped.params.throttle, 0); assert.equal(stopped.params.steering, 0);
  // Any paired observer keeps access to emergency stop after the owner is gone.
  assert.equal((await viewerApi('/api/emergency/stop', {})).status, 200);
  await until(() => commands.some(packet => packet.command === 'DISARM'));
  assert.ok(!output.includes('123456'), 'backend logs must not disclose the control PIN');
});
