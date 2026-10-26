import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import http from 'node:http';
import dgram from 'node:dgram';
import { spawn } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import WebSocket from 'ws';
import { createAsyncHealth } from '../backend/async_health.js';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const delay = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
async function freeUdpPort() {
  const socket = dgram.createSocket('udp4');
  await new Promise((resolve) => socket.bind(0, '127.0.0.1', resolve));
  const port = socket.address().port;
  await new Promise((resolve) => socket.close(resolve));
  return port;
}

test('camera requests remain active while standalone UWB and full telemetry keep arriving', { timeout: 12000 }, async (t) => {
  const directory = await fs.mkdtemp(path.join(os.tmpdir(), 'manta-uwb-async-'));
  t.after(() => fs.rm(directory, { recursive: true, force: true }));
  const requests = [];
  const camera = http.createServer((req, res) => {
    requests.push(req.url);
    setTimeout(() => {
      if (res.destroyed) return;
      res.setHeader('Content-Type', req.url === '/snapshot.jpg' ? 'image/jpeg' : 'application/json');
      res.end(req.url === '/snapshot.jpg' ? 'image' : JSON.stringify({ ok: true }));
    }, 1000);
  });
  await new Promise((resolve) => camera.listen(0, '127.0.0.1', resolve));
  t.after(() => { camera.closeAllConnections(); camera.close(); });
  const config = JSON.parse(await fs.readFile(path.join(root, 'config/system.config.json'), 'utf8'));
  const listener = http.createServer();
  await new Promise((resolve) => listener.listen(0, '127.0.0.1', resolve));
  config.web_port = listener.address().port;
  await new Promise((resolve) => listener.close(resolve));
  config.web_host = '127.0.0.1';
  config.bridge_command_port = await freeUdpPort();
  config.bridge_telemetry_port = await freeUdpPort();
  config.hotspot.camera_port = camera.address().port;
  config.camera.enabled = true;
  config.camera.transport = 'local';
  config.camera.local_source_url = `http://127.0.0.1:${camera.address().port}/snapshot.jpg`;
  config.gimbal.enabled = true;
  config.gimbal.auto_connect = false;
  config.gimbal.face.keep_warm = false;
  config.logs_dir = path.join(directory, 'logs');
  config.recordings.dir = path.join(directory, 'recordings');
  const configFile = path.join(directory, 'config.json');
  await fs.writeFile(configFile, JSON.stringify(config));
  const child = spawn(process.execPath, ['backend/server.js'], {
    cwd: root, env: { ...process.env, MANTA_CONFIG_PATH: configFile }, windowsHide: true,
    stdio: ['ignore', 'pipe', 'pipe']
  });
  let output = '';
  child.stdout.on('data', (chunk) => { output += chunk; });
  child.stderr.on('data', (chunk) => { output += chunk; });
  const exited = new Promise((resolve) => child.once('exit', resolve));
  t.after(async () => { child.kill('SIGKILL'); await exited; });
  const base = `http://127.0.0.1:${config.web_port}`;
  const deadline = Date.now() + 6000;
  for (;;) {
    try { if ((await fetch(`${base}/health`)).ok) break; } catch {}
    assert.equal(child.exitCode, null, output);
    assert.ok(Date.now() < deadline, output);
    await delay(20);
  }
  const socket = new WebSocket(`${base.replace('http:', 'ws:')}/socket.io/?EIO=4&transport=websocket`);
  const packets = [];
  let ready;
  const connected = new Promise((resolve) => { ready = resolve; });
  socket.on('message', (buffer) => {
    const raw = buffer.toString();
    if (raw.startsWith('0')) socket.send('40');
    else if (raw === '2') socket.send('3');
    else if (raw.startsWith('40')) ready();
    else if (raw.startsWith('42')) packets.push(JSON.parse(raw.slice(2)));
  });
  t.after(() => socket.close());
  await connected;
  const snapshot = fetch(`${base}/api/camera/snapshot`);
  const sender = dgram.createSocket('udp4');
  t.after(() => sender.close());
  const send = (packet) => new Promise((resolve, reject) => sender.send(Buffer.from(JSON.stringify(packet)),
    config.bridge_telemetry_port, '127.0.0.1', (error) => error ? reject(error) : resolve()));
  const started = performance.now();
  await send({ type: 'uwb', payload: { distanceM: 1.23, azimuthDeg: 4, fresh: true, online: true,
    measurementStatus: 'trusted', measurementUpdatedAt: 1000 } });
  while (!packets.some(([name]) => name === 'uwb_update') && performance.now() - started < 1000) await delay(10);
  assert.ok(packets.some(([name, data]) => name === 'uwb_update' && data.distanceM === 1.23));
  assert.ok(performance.now() - started < 400, 'slow camera requests must not delay UWB publication');
  const statusStarted = performance.now();
  const status = await (await fetch(`${base}/api/status`)).json();
  assert.ok(performance.now() - statusStarted < 400, 'status reads return without waiting for camera health');
  assert.equal(status.data.camera.enabled, true);
  assert.equal(status.data.telemetry.uwb.distanceM, 1.23);
  assert.equal((await fetch(`${base}/api/gimbal/state`)).status, 200, 'gimbal requests remain available');
  await send({ type: 'telemetry', payload: { servoOutputs: { ch1: 1500, ch3: 1500 } } });
  const fullDeadline = Date.now() + 1000;
  while (!packets.some(([name]) => name === 'telemetry_update') && Date.now() < fullDeadline) await delay(10);
  assert.ok(packets.some(([name]) => name === 'motor_output_update'));
  assert.ok(packets.some(([name]) => name === 'telemetry_update'));
  assert.equal((await snapshot).status, 200);
  assert.ok(requests.includes('/healthz'));
  assert.ok(requests.includes('/snapshot.jpg'));
});

test('a camera that never returns a body times out without blocking callers', async (t) => {
  const server = http.createServer((_req, res) => { res.writeHead(200); res.flushHeaders(); });
  await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
  t.after(() => { server.closeAllConnections(); server.close(); });
  const read = createAsyncHealth(`http://127.0.0.1:${server.address().port}/healthz`, { timeoutMs: 30 });
  read();
  for (let i = 0; i < 100; i++) read();
  await delay(60);
  assert.match(read().message, /timed out/);
});
