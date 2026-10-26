import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import dgram from 'node:dgram';
import fs from 'node:fs/promises';
import http from 'node:http';
import os from 'node:os';
import path from 'node:path';
import test from 'node:test';
import { fileURLToPath } from 'node:url';
import { decodeSiyiPackets, encodeSiyiPacket, siyiCrc16 } from '../backend/siyi_gimbal.js';

const root = fileURLToPath(new URL('../', import.meta.url));
const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
async function waitUntil(predicate, detail) {
  const end = Date.now() + 8000;
  while (Date.now() < end) {
    if (await predicate()) return;
    await delay(30);
  }
  assert.fail(detail());
}
async function udpPort() {
  const socket = dgram.createSocket('udp4');
  await new Promise(resolve => socket.bind(0, '127.0.0.1', resolve));
  const port = socket.address().port;
  await new Promise(resolve => socket.close(resolve));
  return port;
}

test('four gimbal modes use SIYI feedback, surface tracker startup failure, and gate missing UWB', { timeout: 25000 }, async t => {
  const directory = await fs.mkdtemp(path.join(os.tmpdir(), 'manta-gimbal-modes-'));
  t.after(() => fs.rm(directory, { recursive: true, force: true }));
  const wire = dgram.createSocket('udp4'), packets = [];
  await new Promise(resolve => wire.bind(0, '127.0.0.1', resolve));
  t.after(() => wire.close());
  wire.on('message', (data, remote) => {
    for (const packet of decodeSiyiPackets(data).packets) {
      packets.push(packet);
      let payload;
      if (packet.command === 0x0a) { payload = Buffer.alloc(7); payload[5] = 2; }
      if (packet.command === 0x0d) { payload = Buffer.alloc(12); payload.writeInt16LE(1800, 2); }
      if (payload) {
        const response = encodeSiyiPacket(packet.command, payload, packet.sequence);
        response[2] = 2;
        // The response control bit is covered by the CRC.
        response.writeUInt16LE(siyiCrc16(response.subarray(0, -2)), response.length - 2);
        wire.send(response, remote.port, remote.address);
      }
    }
  });
  const listener = http.createServer();
  await new Promise(resolve => listener.listen(0, '127.0.0.1', resolve));
  const webPort = listener.address().port;
  await new Promise(resolve => listener.close(resolve));
  const config = JSON.parse(await fs.readFile(path.join(root, 'config/system.config.json'), 'utf8'));
  Object.assign(config, {web_host:'127.0.0.1', web_port:webPort, bridge_host:'127.0.0.1',
    bridge_command_port:await udpPort(), bridge_telemetry_port:await udpPort(),
    logs_dir:path.join(directory,'logs')});
  config.mission_planner_tcp.enabled = false;
  config.camera.enabled = false;
  Object.assign(config.gimbal, {enabled:true, auto_connect:true, auto_home_on_connect:false,
    control_transport:'udp', udp_host:'127.0.0.1', udp_port:wire.address().port});
  Object.assign(config.gimbal.face, {keep_warm:false, prewarm:false});
  config.recordings.dir = path.join(directory,'recordings');
  const filename = path.join(directory,'config.json');
  await fs.writeFile(filename, JSON.stringify(config));
  const environment = {...process.env, MANTA_CONFIG_FILE:filename,
    PYTHON_EXEC:path.join(directory,'missing-tracker-python')};
  delete environment.MANTA_SIMULATION;
  const child = spawn(process.execPath, ['backend/server.js'], {cwd:root, env:environment, windowsHide:true});
  let output = '';
  child.stdout.on('data', chunk => {output += chunk;});
  child.stderr.on('data', chunk => {output += chunk;});
  const exited = new Promise(resolve => child.once('exit',resolve));
  t.after(async () => {if (child.exitCode === null) child.kill('SIGTERM'); await exited;});
  const base = `http://127.0.0.1:${webPort}`;
  const state = async () => (await (await fetch(base+'/api/gimbal/state')).json()).state;
  await waitUntil(async () => {try {return (await state()).feedback?.checksumValid;} catch {return false;}}, () => output);
  const post = (route, body) => fetch(base+route, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
  assert.equal((await post('/api/gimbal/click',{dx:80,dy:80})).status,200);
  await waitUntil(() => packets.some(p=>p.command===0x0e), ()=>'No real SIYI angle packet emitted');
  await post('/api/gimbal/stop',{});
  for (const mode of ['face','swimmer']) {
    const response = await post('/api/gimbal/track/start',{mode});
    const result = await response.json();
    assert.equal(response.status,503,JSON.stringify(result));
    assert.equal(result.success,false);
    assert.match(result.message,/could not start/);
    assert.equal(result.state.trackingActive,false);
  }
  const beacon = await (await post('/api/gimbal/track/start',{mode:'beacon'})).json();
  assert.equal(beacon.success,true);
  await waitUntil(async () => (await state()).trackStatus?.status==='uwb_stale', ()=>'Beacon did not expose missing UWB');
  const angleCount = packets.filter(p=>p.command===0x0e).length;
  await delay(250);
  assert.equal(packets.filter(p=>p.command===0x0e).length,angleCount,'Untrusted UWB must not issue angle targets');
  await post('/api/gimbal/track/stop',{});
  assert.equal((await fetch(base+'/health')).status,200,'Tracker spawn failure must not crash the backend');
});
