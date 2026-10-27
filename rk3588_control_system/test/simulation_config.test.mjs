import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import http from 'node:http';
import { spawn } from 'node:child_process';
import { once } from 'node:events';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const delay = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

async function startServer(t, { legacy = false, invalid = false } = {}) {
  const directory = await fs.mkdtemp(path.join(os.tmpdir(), 'manta-simulation-'));
  t.after(() => fs.rm(directory, { recursive: true, force: true }));
  const listener = http.createServer();
  await new Promise((resolve) => listener.listen(0, '127.0.0.1', resolve));
  const port = listener.address().port;
  await new Promise((resolve) => listener.close(resolve));
  const config = JSON.parse(await fs.readFile(path.join(root, 'config/system.config.json'), 'utf8'));
  Object.assign(config, {
    web_host: invalid ? '0.0.0.0' : '127.0.0.1', web_port: port,
    bridge_host: '127.0.0.1', bridge_command_port: 15551, bridge_telemetry_port: 15552,
    serial_port: '/dev/manta-sitl-virtual', logs_dir: path.join(directory, 'logs'),
  });
  config.camera.enabled = false;
  config.gimbal.enabled = false;
  config.gimbal.auto_connect = false;
  config.recordings.dir = path.join(directory, 'recordings');
  const configFile = path.join(directory, 'config.json');
  await fs.writeFile(configFile, JSON.stringify(config));
  const environment = { ...process.env, MANTA_SIMULATION: '1', MANTA_CONTROL_PIN: '123456' };
  delete environment.MANTA_CONFIG_FILE;
  delete environment.MANTA_CONFIG_PATH;
  if (legacy) environment.MANTA_CONFIG_PATH = configFile;
  else {
    environment.MANTA_CONFIG_FILE = configFile;
    environment.MANTA_CONFIG_PATH = path.join(directory, 'must-not-be-read.json');
  }
  const child = spawn(process.execPath, ['backend/server.js'], {
    cwd: root, env: environment, windowsHide: true, stdio: ['ignore', 'pipe', 'pipe'],
  });
  let output = '';
  child.stdout.on('data', (chunk) => { output += chunk; });
  child.stderr.on('data', (chunk) => { output += chunk; });
  const exited = once(child, 'exit');
  t.after(async () => {
    if (child.exitCode === null && child.signalCode === null) child.kill('SIGKILL');
    await exited;
  });
  return { child, exited, port, output: () => output };
}

for (const legacy of [false, true]) {
  test(`isolated simulation starts with ${legacy ? 'legacy config path' : 'config file taking precedence'}`,
    { timeout: 12000 }, async (t) => {
      const server = await startServer(t, { legacy });
      const base = `http://127.0.0.1:${server.port}`;
      const deadline = Date.now() + 6000;
      for (;;) {
        let response;
        try { response = await fetch(`${base}/health`); } catch {}
        if (response?.ok) {
          assert.equal(response.headers.get('x-manta-mode'), 'Local-PC-SITL');
          break;
        }
        assert.equal(server.child.exitCode, null, server.output());
        assert.ok(Date.now() < deadline, server.output());
        await delay(20);
      }
      for (const route of ['/api/gimbal/state', '/api/camera/snapshot', '/api/system/status']) {
        const response = await fetch(base + route);
        assert.equal(response.status, 501, route);
        assert.match((await response.json()).message, /Hardware API disabled/);
      }
    });
}

test('simulation refuses a configuration exposing the web server outside loopback',
  { timeout: 12000 }, async (t) => {
    const server = await startServer(t, { invalid: true });
    const [code] = await server.exited;
    assert.notEqual(code, 0);
    assert.match(server.output(), /Simulation requires the isolated local transport configuration/);
  });
