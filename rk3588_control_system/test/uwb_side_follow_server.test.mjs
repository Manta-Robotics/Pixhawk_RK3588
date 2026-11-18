import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import http from 'node:http';
import dgram from 'node:dgram';
import { spawn } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { api, channel, delay, pair } from './helpers/control_client.mjs';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
test('production side-position API enforces ownership, validates values and never starts or arms on selection', { timeout: 20000 }, async t => {
  const directory = await fs.mkdtemp(path.join(os.tmpdir(), 'manta-side-follow-'));
  t.after(() => fs.rm(directory, { recursive: true, force: true }));
  const bridge = dgram.createSocket('udp4'); await new Promise(resolve => bridge.bind(0, '127.0.0.1', resolve));
  t.after(() => bridge.close());
  const commands = []; bridge.on('message', b => commands.push(JSON.parse(b)));
  const telemetry = dgram.createSocket('udp4'); await new Promise(resolve => telemetry.bind(0, '127.0.0.1', resolve));
  const telemetryPort = telemetry.address().port; await new Promise(resolve => telemetry.close(resolve));
  const web = http.createServer(); await new Promise(resolve => web.listen(0, '127.0.0.1', resolve));
  const port = web.address().port; await new Promise(resolve => web.close(resolve));
  const config = JSON.parse(await fs.readFile(path.join(root, 'config/system.config.json'), 'utf8'));
  Object.assign(config, { web_host: '127.0.0.1', web_port: port, bridge_host: '127.0.0.1',
    bridge_command_port: bridge.address().port, bridge_telemetry_port: telemetryPort, logs_dir: path.join(directory, 'logs') });
  config.gimbal.enabled = false; config.gimbal.auto_connect = false;
  config.gimbal.face.keep_warm = false; config.gimbal.face.prewarm = false;
  config.camera.enabled = false; config.hotspot.managed_externally = true;
  config.uwb_usb.enabled = false; config.recordings.dir = path.join(directory, 'recordings');
  const configFile = path.join(directory, 'config.json'); await fs.writeFile(configFile, JSON.stringify(config));
  const env = { ...process.env, MANTA_CONFIG_FILE: configFile, MANTA_CONTROL_PIN: '123456' }; delete env.MANTA_SIMULATION;
  const child = spawn(process.execPath, ['backend/server.js'], { cwd: root, env, windowsHide: true, stdio: ['ignore', 'pipe', 'pipe'] });
  let output = ''; child.stdout.on('data', b => { output += b; }); child.stderr.on('data', b => { output += b; });
  const exited = new Promise(resolve => child.once('exit', resolve));
  t.after(async () => { if (child.exitCode === null) child.kill('SIGKILL'); await exited; });
  const base = `http://127.0.0.1:${port}`, deadline = Date.now() + 7000;
  for (;;) {
    try { if ((await fetch(base + '/health')).ok) break; } catch {}
    assert.equal(child.exitCode, null, output); assert.ok(Date.now() < deadline, output); await delay(20);
  }
  const unpaired = api(base, '', 'unpaired-side-00001');
  assert.equal((await unpaired('/api/uwb-follow/mode', { mode: 'left' })).status, 401);
  const ownerId = 'side-owner-000000001', viewerId = 'side-viewer-00000001';
  const ownerCookie = await pair(base, ownerId), viewerCookie = await pair(base, viewerId);
  const owner = channel(t, base, ownerCookie, ownerId), viewer = channel(t, base, viewerCookie, viewerId);
  await Promise.all([owner.ready, viewer.ready]);
  const ownerApi = api(base, ownerCookie, ownerId), viewerApi = api(base, viewerCookie, viewerId);
  assert.equal((await ownerApi('/api/control/claim', {})).status, 200);
  assert.equal((await viewerApi('/api/uwb-follow/mode', { mode: 'right' })).status, 403);
  const left = await (await ownerApi('/api/uwb-follow/mode', { mode: 'left', distanceM: 3 })).json();
  assert.equal(left.success, true); assert.equal(left.state.followMode, 'left');
  assert.equal(left.state.sideDistanceM, 3); assert.equal(left.state.active, false);
  assert.equal(left.state.trackingMethod, 'traditional');
  for (const body of [{ mode: 'orbit' }, { mode: 'right', distanceM: '2' }, { mode: 'right', distanceM: 30 }])
    assert.equal((await ownerApi('/api/uwb-follow/mode', body)).status, 409);
  assert.equal((await ownerApi('/api/uwb-follow/method', { method: 'neural' })).status, 409);
  const state = await (await ownerApi('/api/uwb-follow/state')).json();
  assert.equal(state.followMode, 'left'); assert.deepEqual(state.followModes, ['rear', 'left', 'right']);
  assert.equal(state.sideFollow.requiresGps, true);
  assert.equal((await ownerApi('/api/uwb-follow/mode', { mode: 'right', distanceM: 2.5 })).status, 200);
  await new Promise((resolve, reject) => bridge.send(Buffer.from(JSON.stringify({ type: 'connection', payload: { connected: true } })),
    telemetryPort, '127.0.0.1', error => error ? reject(error) : resolve()));
  await delay(30);
  const start = await ownerApi('/api/uwb-follow/start', {});
  assert.equal(start.status, 202, JSON.stringify(await start.json()));
  assert.equal((await ownerApi('/api/uwb-follow/mode', { mode: 'rear' })).status, 409);
  assert.equal((await ownerApi('/api/uwb-follow/stop', {})).status, 200);
  assert.equal((await ownerApi('/api/uwb-follow/mode', { mode: 'rear' })).status, 200);
  assert.ok(!commands.some(packet => packet.command === 'ARM'));
  const movement = commands.filter(p => ['ROVER_DRIVE', 'ROVER_BODY_VELOCITY', 'ROVER_ACRO'].includes(p.command));
  assert.ok(movement.every(p => !p.params?.throttle && !p.params?.forwardSpeedMps));
});
