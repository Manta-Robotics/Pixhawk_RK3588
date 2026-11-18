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
import express from 'express';
import { installUwbVisionApi } from '../backend/uwb_vision_service.js';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
test('device fusion API requires ownership; saving/enabling never starts propulsion or a tracker', { timeout: 20000 }, async t => {
    const directory = await fs.mkdtemp(path.join(os.tmpdir(), 'manta-uwb-vision-'));
    t.after(() => fs.rm(directory, { recursive: true, force: true }));
    const bridge = dgram.createSocket('udp4'); await new Promise(resolve => bridge.bind(0, '127.0.0.1', resolve)); t.after(() => bridge.close());
    const commands = []; bridge.on('message', bytes => commands.push(JSON.parse(bytes)));
    const telemetry = dgram.createSocket('udp4'); await new Promise(resolve => telemetry.bind(0, '127.0.0.1', resolve));
    const telemetryPort = telemetry.address().port; await new Promise(resolve => telemetry.close(resolve));
    const probe = http.createServer(); await new Promise(resolve => probe.listen(0, '127.0.0.1', resolve));
    const port = probe.address().port; await new Promise(resolve => probe.close(resolve));
    const config = JSON.parse(await fs.readFile(path.join(root, 'config/system.config.json'), 'utf8'));
    Object.assign(config, { web_host: '127.0.0.1', web_port: port, bridge_host: '127.0.0.1', bridge_command_port: bridge.address().port,
        bridge_telemetry_port: telemetryPort, logs_dir: path.join(directory, 'logs') });
    config.gimbal.enabled = false; config.gimbal.auto_connect = false;
    config.gimbal.face.keep_warm = false; config.gimbal.face.prewarm = false;
    config.camera.enabled = false; config.hotspot.managed_externally = true;
    config.uwb_usb.enabled = false; config.recordings.dir = path.join(directory, 'recordings');
    const configFile = path.join(directory, 'config.json'), fusionFile = path.join(directory, 'fusion.json');
    await fs.writeFile(configFile, JSON.stringify(config));
    const env = { ...process.env, MANTA_CONFIG_FILE: configFile, MANTA_UWB_VISION_CONFIG: fusionFile, MANTA_CONTROL_PIN: '123456' };
    delete env.MANTA_SIMULATION;
    const child = spawn(process.execPath, ['backend/server.js'], { cwd: root, env, windowsHide: true, stdio: ['ignore', 'pipe', 'pipe'] });
    let output = ''; child.stdout.on('data', bytes => { output += bytes; }); child.stderr.on('data', bytes => { output += bytes; });
    const exited = new Promise(resolve => child.once('exit', resolve));
    t.after(async () => { if (child.exitCode === null) child.kill('SIGKILL'); await exited; });
    const base = `http://127.0.0.1:${port}`, deadline = Date.now() + 7000;
    for (;;) {
        try { if ((await fetch(base + '/health')).ok) break; } catch {}
        assert.equal(child.exitCode, null, output); assert.ok(Date.now() < deadline, output); await delay(30);
    }
    assert.equal((await fetch(base + '/api/uwb-vision/state')).status, 401);
    const ownerId = 'vision-owner-00000001', viewerId = 'vision-viewer-0000001';
    const ownerCookie = await pair(base, ownerId), viewerCookie = await pair(base, viewerId);
    const owner = channel(t, base, ownerCookie, ownerId), viewer = channel(t, base, viewerCookie, viewerId);
    await Promise.all([owner.ready, viewer.ready]);
    const ownerApi = api(base, ownerCookie, ownerId), viewerApi = api(base, viewerCookie, viewerId);
    assert.equal((await ownerApi('/api/control/claim', {})).status, 200);
    const initial = await (await viewerApi('/api/uwb-vision/state')).json();
    assert.equal(initial.config.calibrated, false); assert.equal(initial.state.enabled, false);
    assert.equal((await viewerApi('/api/uwb-vision/enabled', { enabled: true })).status, 403);
    assert.equal((await viewerApi('/api/uwb-vision/config', { calibrated: true })).status, 403);
    assert.equal((await ownerApi('/api/uwb-vision/config', { horizontalFovDeg: 190 })).status, 400);
    assert.equal((await ownerApi('/api/uwb-vision/enabled', { enabled: 'true' })).status, 400);
    assert.equal((await ownerApi('/api/uwb-vision/config', { horizontalFovDeg: 75, calibrated: true })).status, 200);
    assert.equal(JSON.parse(await fs.readFile(fusionFile, 'utf8')).calibrated, true);
    const changed = await (await ownerApi('/api/uwb-vision/config', { yawOffsetDeg: 2 })).json();
    assert.equal(changed.config.calibrated, false);
    const enabled = await (await ownerApi('/api/uwb-vision/enabled', { enabled: true })).json();
    assert.equal(enabled.state.enabled, true); assert.equal(enabled.state.result, null);
    const gimbal = await (await ownerApi('/api/gimbal/state')).json(); assert.equal(gimbal.state.trackingActive, false);
    assert.ok(!commands.some(packet => packet.command === 'ARM'));
    assert.ok(commands.filter(packet => ['ROVER_DRIVE', 'ROVER_BODY_VELOCITY', 'ROVER_ACRO'].includes(packet.command))
        .every(packet => !packet.params?.throttle && !packet.params?.forwardSpeedMps));
    assert.equal((await ownerApi('/api/uwb-vision/enabled', { enabled: false })).status, 200);
});

test('persisted fusion settings survive service recreation while enable state resets', async t => {
    const directory = await fs.mkdtemp(path.join(os.tmpdir(), 'manta-vision-persist-'));
    t.after(() => fs.rm(directory, { recursive: true, force: true }));
    const filename = path.join(directory, 'fusion.json');
    await fs.writeFile(filename, JSON.stringify({ horizontalFovDeg: 80, calibrated: true }));
    const app = express(); const service = installUwbVisionApi(app, { filename, getInputs: () => ({}) });
    assert.equal(service.read().config.horizontalFovDeg, 80); assert.equal(service.read().config.calibrated, true);
    assert.equal(service.read().state.enabled, false);
    await fs.writeFile(filename, '{invalid');
    const invalid = installUwbVisionApi(express(), { filename, getInputs: () => ({}) });
    assert.equal(invalid.read().config.calibrated, false);
});
