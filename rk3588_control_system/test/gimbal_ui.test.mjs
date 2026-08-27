import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import { fileURLToPath } from 'node:url';

const PROJECT_ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const html = fs.readFileSync(path.join(PROJECT_ROOT, 'frontend', 'gimbal.html'), 'utf8');
const script = fs.readFileSync(path.join(PROJECT_ROOT, 'frontend', 'js', 'page_gimbal.js'), 'utf8');
const mobileHtml = fs.readFileSync(path.join(PROJECT_ROOT, 'frontend', 'mobile-preview-kimi-k26.html'), 'utf8');
const mobileTransport = fs.readFileSync(path.join(PROJECT_ROOT, 'frontend', 'js', 'manta-app-transport.js'), 'utf8');
const server = fs.readFileSync(path.join(PROJECT_ROOT, 'backend', 'server.js'), 'utf8');

test('gimbal page exposes separate control and video states', () => {
  assert.match(html, /id="gimbalControlCard"/);
  assert.match(html, /id="gimbalVideoCard"/);
  assert.match(html, /id="gimbalVideoBadge"/);
});

test('video refresh button only rebuilds the browser stream source', () => {
  assert.match(html, /id="gimbalRefreshVideo"/);
  const body = script.match(/function refreshCameraStream\(\) \{([\s\S]*?)\n    \}/)?.[1] || '';
  assert.match(body, /removeAttribute\("src"\)/);
  assert.match(body, /startCamera\(cameraSource, "manual"\)/);
  assert.doesNotMatch(body, /postJson|fetch|gimbal\/connect|systemctl|restart/);
});

test('interrupted mobile video refreshes from an anywhere click without moving the gimbal', () => {
  assert.match(mobileHtml, /点击画面任意位置刷新，控制链路保持在线/);
  const body = mobileHtml.match(/function handleGimbalVideoClick\(event\)\{([\s\S]*?)\n\}/)?.[1] || '';
  const refreshGuard = body.indexOf('isGimbalVideoInterrupted()');
  const clickToCenterGuard = body.indexOf('state.gimbalMode!=="click"');
  assert.ok(refreshGuard >= 0 && refreshGuard < clickToCenterGuard);
  assert.match(body.slice(refreshGuard, clickToCenterGuard), /refreshGimbalVideoStream\(\)/);
});

test('live frames clear only a stale video-interruption overlay', () => {
  const interrupted = mobileHtml.match(/function isGimbalVideoInterrupted\(\)\{([\s\S]*?)\n\}/)?.[1] || '';
  assert.match(interrupted, /state\.videoLost/);
  assert.match(interrupted, /alert\.dataset\.reason==="video"/);
  const stateBody = mobileHtml.match(/function setGimbalVideoState\(status,transport\)\{([\s\S]*?)\n\}/)?.[1] || '';
  assert.match(stateBody, /status==="live"[\s\S]*alert\.dataset\.reason==="video"/);
  assert.match(stateBody, /state\.videoLost=false;alert\.hidden=true/);
  assert.match(mobileHtml, /liveGimbalWebrtc"\)\.addEventListener\("timeupdate"/);
  assert.match(mobileHtml, /videoAlert"\)\.addEventListener\("click"[\s\S]*refreshGimbalVideoStream\(\)/);
});

test('missing gimbal control feedback never covers a working video stream', () => {
  const body = mobileHtml.match(/function renderHardwareStatus\(\)\{([\s\S]*?)\n\}/)?.[1] || '';
  assert.doesNotMatch(body, /showGimbalOffline|等待云台串口与影像链路恢复/);
  assert.match(body, /videoAlert"\)\.hidden=true/);
  assert.match(body, /button\.disabled=state\.connected&&!state\.gimbalOutputAvailable/);
});

test('an open UART keeps output-capable gimbal controls enabled without feedback', () => {
  assert.match(mobileHtml, /gimbalOutputAvailable:false/);
  assert.match(mobileHtml, /state\.gimbalOutputAvailable=Boolean\(payload\.portOpen\|\|payload\.connected\)/);
  const hardwareBody = mobileHtml.match(/function renderHardwareStatus\(\)\{([\s\S]*?)\n\}/)?.[1] || '';
  assert.match(hardwareBody, /button\.disabled=state\.connected&&!state\.gimbalOutputAvailable/);
  assert.match(mobileTransport, /gimbalOutputAvailable: Boolean\(state\.connected \|\| state\.portOpen\)/);
});

test('the UI reports only a command-response timeout, not passive silence', () => {
  assert.doesNotMatch(mobileHtml, /No valid gimbal feedback/);
  assert.match(mobileHtml, /gimbalLinkStatus==="command_timeout"\?"ERR"/);
  assert.match(mobileHtml, /state\.gimbalLinkStatus==="command_timeout"[\s\S]*云台命令无响应/);
  assert.doesNotMatch(server, /No valid gimbal feedback within/);
  assert.match(server, /beginGimbalResponseCheck\(`home:/);
  assert.match(server, /acceptGimbalFeedback\(gimbalLinkHealth, updatedAt\)/);
});

test('home command writes immediately when UART is already open without feedback', () => {
  const start = server.indexOf("app.post('/api/gimbal/home'");
  const end = server.indexOf("app.post('/api/gimbal/stop'", start);
  const body = server.slice(start, end);
  assert.match(body, /if \(!gimbalStream\)/);
  assert.doesNotMatch(body, /if \(!gimbalStream \|\| !gimbalState\.connected\)/);
  assert.match(body, /sendGimbalHome\('web'/);
});

test('Beacon calibration and tracking use commanded absolute angles without feedback', () => {
  const start = server.indexOf('function startBeaconCalibrationSession()');
  const collect = server.indexOf('async function collectBeaconCalibrationReading()', start);
  const capture = server.indexOf('async function captureBeaconCalibrationPoint()', collect);
  const end = server.indexOf('function planGimbalClickMove(', capture);
  assert.doesNotMatch(server.slice(start, end), /Gimbal feedback|gimbalState\.feedback|feedback\.checksumValid/);
  assert.match(server.slice(collect, capture), /gimbalYawDeg: Number\(point\.yawDeg\)/);
  assert.match(server, /rawDesired\.absoluteAngle[\s\S]*command: 0x72/);
  assert.match(mobileHtml, /state\.gimbalMode!=="beacon"\|\|!state\.gimbalOutputAvailable/);
  assert.doesNotMatch(mobileHtml, /九点校准需要云台姿态反馈|feedback wire/);
});
