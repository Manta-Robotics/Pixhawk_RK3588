import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
const html = fs.readFileSync(new URL('../frontend/mobile-preview-kimi-k26.html', import.meta.url), 'utf8');

test('all inline scripts in the control page parse after prediction UI changes', () => {
  for (const match of html.matchAll(/<script\b[^>]*>([\s\S]*?)<\/script>/gi)) {
    if (match[1].trim()) new vm.Script(match[1]);
  }
});

test('lost raw UWB still renders an explicitly labelled dashed predicted target', () => {
  const elements = new Map();
  const $ = (id) => {
    if (!elements.has(id)) elements.set(id, { style: {}, dataset: {}, attributes: {},
      setAttribute(k, v) { this.attributes[k] = v; } });
    return elements.get(id);
  };
  const state = { connected: true, uwb: { online: false, fresh: false, distanceM: null, azimuthDeg: null },
    uwbFollow: { active: true, relativeBodyM: { x: 3, y: 1 }, targetEstimate: {
      usable: true, source: 'predicted', relativeBodyM: { x: 3, y: 1 } } } };
  const context = vm.createContext({ state, $, localize: (zh) => zh, clamp: (x, lo, hi) => Math.min(hi, Math.max(lo, x)) });
  vm.runInContext('var window=globalThis;\n' + fs.readFileSync(new URL('../frontend/js/uwb-display-smoother.js', import.meta.url), 'utf8'), context);
  vm.runInContext(fs.readFileSync(new URL('../frontend/js/uwb-raw-display.js', import.meta.url), 'utf8'), context);
  vm.runInContext(html.slice(html.indexOf('var uwbDisplaySmoother='), html.indexOf('function renderRelativePosition(){')), context);
  const fn = html.slice(html.indexOf('function renderRelativePosition(){'), html.indexOf('function currentMapLocation(){'));
  vm.runInContext(fn + '\nrenderRelativePosition();', context);
  assert.equal($('#relativePositionTitle').textContent, '目标预测位置');
  assert.equal($('#relativeFusionMode').textContent, '预测，非实测');
  assert.equal($('#relativeTargetVector').attributes['stroke-dasharray'], '3 2');
  assert.ok(Number.isFinite(Number($('#relativeTargetPoint').attributes.cx)));
  state.uwbFollow.targetEstimate.usable = false;
  vm.runInContext('renderRelativePosition()', context);
  assert.equal($('#relativeFusionMode').textContent, '等待 UWB');
  state.uwb = { online: true, fresh: true, distanceM: 3, azimuthDeg: 20, measurementUpdatedAt: Date.now() };
  const before = structuredClone(state.uwb);
  vm.runInContext('renderRelativePosition()', context);
  assert.equal($('#relativePositionPanel').dataset.state, 'online');
  assert.deepEqual(state.uwb, before);
  state.uwb = { online: true, fresh: false, measurementStatus: 'suspect', sensors: [
    { receiverId: 1, occlusionDiagnostics: { state: 'quarantined' } } ] };
  vm.runInContext('renderRelativePosition()', context);
  assert.equal($('#relativePositionPanel').dataset.state, 'offline');
  assert.equal($('#relativeFusionMode').textContent, '定位异常，等待恢复');
  state.connected = false;
  vm.runInContext('renderRelativePosition()', context);
  assert.equal($('#relativePositionPanel').dataset.state, 'offline');
  assert.equal(vm.runInContext('uwbDisplaySmoother.angle', context), null);
});

test('rejected fresh raw position is visible alongside an independently admitted position', () => {
  const elements = new Map(), $ = id => {
    if (!elements.has(id)) elements.set(id, { style: {}, dataset: {}, attributes: {},
      setAttribute(k, v) { this.attributes[k] = v; } });
    return elements.get(id);
  };
  const now = 100000, state = { connected: true, uwbFollow: { active: false }, uwb: {
    online: false, fresh: false, measurementStatus: 'suspect', rejectionReason: 'occlusion_suspected',
    measurementUpdatedAt: now - 86000, distanceM: 2.58, azimuthDeg: 2.26,
    sensors: [{ receiverId: 1, tagId: 5313, valid: true, rawDistanceM: 1.11, rawAzimuthDeg: 11,
      rawDistanceUpdatedAt: now, receivedMeasurementAt: now, measurementUpdatedAt: now - 86000,
      occlusionDiagnostics: { state: 'quarantined' } }] } };
  const context = vm.createContext({ state, $, Date: { now: () => now }, localize: zh => zh,
    clamp: (x, lo, hi) => Math.min(hi, Math.max(lo, x)) });
  for (const name of ['uwb-display-smoother', 'uwb-raw-display']) {
    vm.runInContext('var window=globalThis;\n' + fs.readFileSync(new URL('../frontend/js/' + name + '.js', import.meta.url), 'utf8'), context);
  }
  vm.runInContext(html.slice(html.indexOf('var uwbDisplaySmoother='), html.indexOf('function currentMapLocation(){')), context);
  const before = structuredClone(state);
  vm.runInContext('renderRelativePosition()', context);
  assert.equal($('#relativePositionPanel').dataset.state, 'raw-only');
  assert.equal($('#relativeRawPoint').attributes.visibility, 'visible');
  assert.equal($('#relativeTargetPoint').attributes.visibility, 'hidden');
  assert.equal($('#relativeRawValue').textContent, '1.11 m / 11.0°');
  assert.equal($('#relativeRawLabel').textContent, '○ 原始·基站1');
  assert.match($('#relativeDecision').textContent, /未接纳：突跳隔离/);
  assert.match($('#relativeRangeValue').textContent, /拒收/);
  assert.deepEqual(state, before); // Display cannot change admission or control.
  Object.assign(state.uwb, { online: true, fresh: true, measurementStatus: 'trusted',
    measurementUpdatedAt: now, distanceM: 1.2, azimuthDeg: 10 });
  vm.runInContext('renderRelativePosition()', context);
  assert.equal($('#relativePositionPanel').dataset.state, 'online');
  assert.equal($('#relativeRawPoint').attributes.visibility, 'visible');
  assert.equal($('#relativeTargetPoint').attributes.visibility, 'visible');
  assert.notEqual($('#relativeRawPoint').attributes.cx, $('#relativeTargetPoint').attributes.cx);
  assert.equal($('#relativeRangeValue').textContent, '1.20 m / 10.0°');
  state.uwb.trustChecksEnabled = false;
  vm.runInContext('renderRelativePosition()', context);
  assert.equal($('#relativeJudgedLabel').textContent, '● 平滑');
  assert.match($('#relativeDecision').textContent, /可信度仅作诊断；使用平滑测量/);
  state.uwb.sensors[0].trustDiagnostics = { status: 'suspect', controlGating: false };
  vm.runInContext('renderRelativePosition()', context);
  assert.equal($('#relativeTargetPoint').attributes.visibility, 'visible');
  assert.match($('#relativeDecision').textContent, /旁路：可疑（不停推）/);
  state.connected = false;
  vm.runInContext('renderRelativePosition()', context);
  assert.equal($('#relativeRawPoint').attributes.visibility, 'hidden');
  assert.equal($('#relativeTargetPoint').attributes.visibility, 'visible');
  assert.equal($('#relativeTargetPoint').dataset.missing, 'true');
  assert.equal($('#relativeTargetPoint').attributes.cx, '90.00');
  assert.equal($('#relativeRangeValue').textContent, '-- m / --°');
  assert.equal($('#relativeDecision').textContent, '连接已断开');
});
