import assert from 'node:assert/strict';
import { spawn, spawnSync } from 'node:child_process';
import dgram from 'node:dgram';
import fs from 'node:fs/promises';
import http from 'node:http';
import os from 'node:os';
import path from 'node:path';
import test from 'node:test';
import { fileURLToPath } from 'node:url';
import { api, channel, delay, pair, until } from './helpers/control_client.mjs';

const root = fileURLToPath(new URL('..', import.meta.url));
const endpoint = '/api/experiments';
const addresses = '127.0.0.1';
const node = process.execPath;
const python = process.env.MANTA_LOGGER_PYTHON || 'python3';

async function bindUdp() {
  const socket = dgram.createSocket('udp4');
  await new Promise((resolve, reject) => { socket.once('error', reject); socket.bind(0, addresses, resolve); });
  return socket;
}

async function freeHttpPort() {
  const web = http.createServer();
  await new Promise((resolve, reject) => { web.once('error', reject); web.listen(0, addresses, resolve); });
  const { port } = web.address();
  await new Promise(resolve => web.close(resolve));
  return port;
}

async function waitFor(read, description, timeout = 7000) {
  const deadline = Date.now() + timeout;
  let last;
  do {
    try { const value = await read(); if (value) return value; } catch (error) { last = error; }
    await delay(20);
  } while (Date.now() < deadline);
  throw new Error(description + (last ? ': ' + last.message : ''));
}

async function json(response, status = 200) {
  const value = await response.json();
  assert.equal(response.status, status, JSON.stringify(value));
  return value;
}

async function download(call, path, column, expectedName) {
  const response = await call(path);
  assert.equal(response.status, 200, response.ok ? '' : await response.text());
  assert.match(response.headers.get('content-type'), /^text\/csv;.*utf-8/i);
  assert.ok(response.headers.get('content-disposition').includes(expectedName));
  const text = await response.text();
  assert.ok(text.split(/\r?\n/, 1)[0].split(',').includes(column));
  const parsed = spawnSync(python, ['-c', 'import csv, json, sys; print(json.dumps(list(csv.DictReader(sys.stdin))))'], { input: text, encoding: 'utf8' });
  assert.equal(parsed.status, 0, parsed.stderr);
  return JSON.parse(parsed.stdout);
}

function latest(socket, id, predicate = () => true) {
  return socket.events.findLast(([name, snapshot]) => name === 'experiment_snapshot'
    && snapshot.session_id === id && predicate(snapshot))?.[1];
}

const moduleData = (snapshot, id) => snapshot?.modules.find(module => module.id === id);
const reading = (snapshot, id, key) => moduleData(snapshot, id)?.readings.find(item => item.key === key);
const uwbAlert = snapshot => snapshot?.active_alerts.find(alert => alert.code === 'UWB_STALE');

test('loopback experiment API secures ownership and preserves UWB and mode evidence through the writer and three downloads', { timeout: 25000 }, async t => {
  const directory = await fs.mkdtemp(path.join(os.tmpdir(), 'manta-exp-http-'));
  const bridge = await bindUdp();
  const telemetryBind = await bindUdp();
  const webPort = await freeHttpPort();
  const telemetryPort = telemetryBind.address().port;
  const bridgePort = bridge.address().port;
  await new Promise(resolve => telemetryBind.close(resolve));
  const rawBind = await bindUdp();
  const rawPort = rawBind.address().port;
  await new Promise(resolve => rawBind.close(resolve));
  const base = `http://${addresses}:${webPort}`;
  const commands = [];
  const connectedSockets = [];
  bridge.on('message', message => { try { commands.push(JSON.parse(message)); } catch {} });
  const settings = JSON.parse(await fs.readFile(path.join(root, 'config/system.config.json'), 'utf8'));
  Object.assign(settings, { web_host: addresses, web_port: webPort, bridge_host: addresses, bridge_command_port: bridgePort,
    bridge_telemetry_port: telemetryPort, logs_dir: path.join(directory, 'device-logs'), manta_service_name: `Manta test ${webPort}` });
  settings.gimbal.enabled = false;
  settings.gimbal.auto_connect = false;
  settings.gimbal.face.keep_warm = false;
  settings.gimbal.face.prewarm = false;
  settings.gimbal.video.shared_frame_alignment = false;
  settings.camera.enabled = false;
  settings.hotspot.managed_externally = true;
  settings.uwb_usb.enabled = false;
  settings.uwb_dual.enabled = true;
  settings.recordings.dir = path.join(directory, 'recordings');
  settings.experiment = { diagnostic: { uwb: 350 } };
  const config = path.join(directory, 'config.json');
  const logger = path.join(directory, 'logger.json');
  const logs = path.join(directory, 'sessions');
  await fs.writeFile(config, JSON.stringify(settings));
  await fs.writeFile(logger, JSON.stringify({ directory: logs, rate_hz: 10, rotate_seconds: 900, min_free_gb: 0.01, max_total_gb: 1 }));
  const env = { ...process.env, MANTA_CONFIG_FILE: config, MANTA_CONTROL_PIN: '123456',
    MANTA_LOGGER_CONFIG: logger, MANTA_JOURNAL_SOCKET: path.join(directory, 'run', 'exp.sock'),
    MANTA_EXPERIMENT_RAW_PORT: String(rawPort), MANTA_LOGGER_PYTHON: python,
    MANTA_ROS_SETUP: path.join(directory, 'no-ros'), MANTA_ROS_WORKSPACE: path.join(directory, 'workspace') };
  delete env.MANTA_SIMULATION;
  const child = spawn(node, ['backend/server.js'], { cwd: root, env, stdio: ['ignore', 'pipe', 'pipe'] });
  let output = '';
  let closed = false;
  child.stdout.on('data', chunk => { output = (output + chunk).slice(-8000); });
  child.stderr.on('data', chunk => { output = (output + chunk).slice(-8000); });
  const exited = new Promise(resolve => child.once('exit', () => { closed = true; resolve(); }));
  t.after(async () => {
    bridge.close();
    for (const socket of connectedSockets) socket.terminate();
    if (!closed) child.kill('SIGTERM');
    const deadline = setTimeout(() => { if (!closed) child.kill('SIGKILL'); }, 6000);
    await exited;
    clearTimeout(deadline);
    await fs.rm(directory, { recursive: true, force: true });
  });
  const send = payload => new Promise((resolve, reject) => bridge.send(Buffer.from(JSON.stringify(payload)), telemetryPort, addresses, error => error ? reject(error) : resolve()));
  const push = (sample_id, values, source = 'uwb', quality = 'measured', meta = {}) => send({ kind: 'experiment_sample', v: 2,
    payload: { source, sample_id, obs_ns: process.hrtime.bigint().toString(), avail_ns: null, time_kind: 'host_receive', quality, values, meta } });
  const actual = range => ({ uwb_range_m: range, uwb_bearing_deg: 0, uwb_elevation_deg: null });
  try {
    await waitFor(async () => (await fetch(base + '/health')).ok, 'HTTP backend did not start');
    assert.equal((await fetch(base + endpoint + '/live')).status, 401);
    assert.equal((await fetch(base + endpoint + '/sessions')).status, 401);
    const ownerId = 'experiment-owner-0001', viewerId = 'experiment-viewer-0001';
    const ownerCookie = await pair(base, ownerId), viewerCookie = await pair(base, viewerId);
    const ownerSocket = channel(t, base, ownerCookie, ownerId), viewerSocket = channel(t, base, viewerCookie, viewerId);
    connectedSockets.push(ownerSocket.socket, viewerSocket.socket);
    await Promise.all([ownerSocket.ready, viewerSocket.ready]);
    const owner = api(base, ownerCookie, ownerId), viewer = api(base, viewerCookie, viewerId);
    assert.equal((await owner('/api/control/claim', {})).status, 200);
    assert.equal((await viewer(endpoint + '/start', {})).status, 403);
    assert.equal((await viewer(endpoint + '/stop', {})).status, 403);
    const initial = (await json(await viewer(endpoint + '/live'))).data;
    assert.equal(initial.schema_version, 2);
    assert.equal(initial.session_id, null);
    assert.deepEqual(initial.modules.map(item => item.id), ['fc', 'mode', 'imu', 'uwb', 'gimbal', 'vision', 'motors', 'video', 'logger']);
    assert.deepEqual((await json(await viewer(endpoint + '/sessions'))).data.items, []);
    assert.ok((await fetch(base + '/')).headers.get('content-type').includes('text/html'));
    assert.match(await (await fetch(base + '/')).text(), /experiment-panel\.js/);
    assert.equal((await json(await owner(endpoint + '/start', {}))).data.state, 'starting');
    const writing = await waitFor(async () => {
      const snapshot = (await json(await viewer(endpoint + '/live'))).data;
      if (reading(snapshot, 'logger', 'writer_state')?.value === 'failed') throw new Error('logger reported failed');
      return snapshot.session_id && reading(snapshot, 'logger', 'writer_state')?.value === 'writing' ? snapshot : null;
    }, 'writer did not open a session');
    const id = writing.session_id;
    const atSession = endpoint + '/sessions/' + encodeURIComponent(id);
    await push('fixture/uwb/bootstrap', actual(4.1));
    await push('fixture/uwb/1', actual(4.2));
    await until(() => latest(viewerSocket, id, snapshot => moduleData(snapshot, 'uwb').sample_id === 'fixture/uwb/1'
      && reading(snapshot, 'uwb', 'uwb_bearing_deg').value === 0), 'fresh UWB and zero bearing did not reach the viewer', 3000);
    const warning = await waitFor(() => latest(viewerSocket, id, snapshot => uwbAlert(snapshot)?.level === 'warn'), 'missing UWB warning', 3000);
    const alertId = uwbAlert(warning).alert_id;
    assert.equal(reading(warning, 'uwb', 'uwb_range_m').quality, 'last_valid');
    assert.equal(uwbAlert(warning).threshold, 350);
    await push('fixture/uwb/1', actual(9.9)); // Same producer ID cannot renew or replace evidence.
    await push('fixture/predicted/1', { uwb_pred_range_m: 5, uwb_pred_bearing_deg: 0 }, 'uwb_pred', 'predicted', { based_on_sample_id: 'fixture/uwb/1' });
    const errorSnapshot = await waitFor(() => latest(viewerSocket, id, snapshot => uwbAlert(snapshot)?.level === 'error'), 'missing UWB error', 3000);
    assert.equal(uwbAlert(errorSnapshot).alert_id, alertId);
    assert.equal(uwbAlert(errorSnapshot).threshold, 1050);
    assert.equal(reading(errorSnapshot, 'uwb', 'uwb_range_m').value, 4.2);
    await push('fixture/uwb/2', actual(4.3));
    const one = await waitFor(async () => {
      const data = (await json(await viewer(endpoint + '/live'))).data;
      return moduleData(data, 'uwb').sample_id === 'fixture/uwb/2' ? data : null;
    }, 'first recovery sample was not admitted');
    assert.equal(uwbAlert(one).alert_id, alertId, 'one new measured sample must not clear the alert');
    await push('fixture/uwb/3', actual(4.4));
    await until(() => latest(viewerSocket, id, snapshot => !uwbAlert(snapshot) && moduleData(snapshot, 'uwb').sample_id === 'fixture/uwb/3'),
      'two distinct measured samples did not clear the same alert', 3000);
    await push('fixture/fc/1', { fc_mode: 'MANUAL', armed: false }, 'fc');
    const fusion = await json(await owner('/api/uwb-vision/follow-source', { source: 'uwb+vision' }));
    assert.equal(fusion.followSource, 'uwb+vision');
    const observed = (await json(await viewer(endpoint + '/live'))).data;
    assert.equal(reading(observed, 'mode', 'follow_source').value, 'uwb+vision');
    const selected = await json(await owner('/api/uwb-follow/mode', { mode: 'left', distanceM: 3 }));
    assert.equal(selected.state.followMode, 'left');
    assert.equal(selected.state.active, false);
    await until(() => latest(viewerSocket, id, snapshot => reading(snapshot, 'mode', 'follow_side')?.value === 'left'),
      'viewer did not see the selected left-side mode', 3000);
    const mode = latest(viewerSocket, id, snapshot => reading(snapshot, 'mode', 'follow_side')?.value === 'left');
    assert.equal(reading(mode, 'mode', 'follow_active').value, false);
    assert.equal(reading(mode, 'fc', 'fc_mode').value, 'MANUAL');
    const snapshots = viewerSocket.events.filter(([event, snapshot]) => event === 'experiment_snapshot' && snapshot.session_id === id).map(([, snapshot]) => snapshot);
    assert.ok(snapshots.length >= 5, 'Socket.IO did not deliver the regular 4 Hz snapshot flow');
    assert.ok(snapshots.every((value, i) => i === 0 || value.seq > snapshots[i - 1].seq));
    const intervals = snapshots.slice(1).map((value, i) => Number(BigInt(value.mono_ns) - BigInt(snapshots[i].mono_ns)) / 1e6);
    assert.ok(intervals.filter(interval => interval >= 140 && interval <= 410).length >= 2, String(intervals));
    assert.equal((await viewer(endpoint + '/stop', {})).status, 403);
    // The mode event is immediate. A 20 Hz row containing it exists only after
    // the next sample tick and writer commit, which may follow the HTTP reply.
    await waitFor(async () => {
      const rows = await download(viewer, atSession + '/files/overview', 'follow_side', 'state_overview.csv');
      return rows.some(row => row.follow_side === 'left' && row.fc_mode === 'MANUAL');
    }, 'committed overview did not include the selected side and confirmed flight mode');
    assert.equal((await json(await owner(endpoint + '/stop', {}))).data.state, 'finalizing');
    await waitFor(async () => {
      const data = (await json(await viewer(endpoint + '/live'))).data;
      return data.session_id === null;
    }, 'writer did not commit the logical session on stop');
    const listed = (await json(await viewer(endpoint + '/sessions'))).data;
    assert.equal(listed.items[0].session_id, id);
    const manifest = (await json(await viewer(atSession))).data;
    assert.equal(manifest.session_id, id);
    assert.deepEqual(manifest.files.map(item => item.kind).sort(), ['events', 'overview', 'video']);
    const chunk = (await json(await viewer(atSession + '/events?module=uwb&limit=1'))).data;
    assert.equal(chunk.items.length, 1);
    const later = (await json(await viewer(atSession + '/events?module=uwb&after=' + chunk.next_after))).data;
    assert.ok(later.items.every(event => event.event_no > chunk.next_after));
    const eventPage = (await json(await viewer(atSession + '/events'))).data;
    const events = eventPage.items;
    const sourceRequest = events.find(event => event.type === 'mode_request' && event.detail_json.axis === 'follow_source');
    assert.ok(sourceRequest?.request_id);
    const appliedSource = events.find(event => event.type === 'mode_applied' && event.request_id === sourceRequest.request_id);
    assert.equal(appliedSource?.detail_json.actual, 'uwb+vision');
    assert.deepEqual(events.filter(event => event.alert_id === alertId).map(event => event.type), ['alert_open', 'alert_level', 'alert_close']);
    const modeRequest = events.find(event => event.type === 'mode_request' && event.detail_json.requested === 'left');
    assert.ok(modeRequest?.request_id, 'mode request is missing its stable request ID');
    const modeApplied = events.find(event => event.type === 'mode_applied' && event.request_id === modeRequest.request_id);
    assert.equal(modeApplied?.detail_json.actual, 'left');
    const gap = (await json(await viewer(atSession + '/locate?t_ms=' + modeApplied.t_ms))).data;
    assert.equal(gap.match, 'no_video');
    assert.equal(gap.frame, null);
    assert.equal(gap.state.follow_side, 'left');
    const overview = await download(viewer, atSession + '/files/overview', 'uwb_sample_id', 'state_overview.csv');
    const eventCsv = await download(viewer, atSession + '/files/events', 'alert_id', 'events.csv');
    const video = await download(viewer, atSession + '/files/video', 'display_no', 'video_aligned.csv');
    assert.equal(video.length, 0);
    assert.ok(overview.some(row => row.uwb_sample_id === 'fixture/uwb/1' && row.uwb_range_m === '4.2' && row.uwb_bearing_deg === '0'));
    assert.ok(overview.some(row => row.uwb_sample_id === 'fixture/uwb/1' && row.uwb_range_m === '' && Number(row.uwb_age_ms) > 350));
    assert.ok(overview.some(row => row.follow_side === 'left' && row.fc_mode === 'MANUAL'));
    assert.ok(overview.every((row, i) => i === 0 || Number(row.row_no) > Number(overview[i - 1].row_no)));
    assert.deepEqual(eventCsv.filter(row => row.alert_id === alertId).map(row => row.event_id), events.filter(event => event.alert_id === alertId).map(event => event.event_id));
    assert.ok(!commands.some(packet => packet.command === 'ARM'));
  } catch (error) {
    t.diagnostic(output);
    throw error;
  }
});
