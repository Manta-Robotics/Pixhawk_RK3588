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
  const fn = html.slice(html.indexOf('function renderRelativePosition(){'), html.indexOf('function currentMapLocation(){'));
  vm.runInContext(fn + '\nrenderRelativePosition();', context);
  assert.equal($('#relativePositionTitle').textContent, '目标预测位置');
  assert.equal($('#relativeFusionMode').textContent, '预测，非实测');
  assert.equal($('#relativeTargetVector').attributes['stroke-dasharray'], '3 2');
  assert.ok(Number.isFinite(Number($('#relativeTargetPoint').attributes.cx)));
  state.uwbFollow.targetEstimate.usable = false;
  vm.runInContext('renderRelativePosition()', context);
  assert.equal($('#relativeFusionMode').textContent, '等待 UWB');
});
