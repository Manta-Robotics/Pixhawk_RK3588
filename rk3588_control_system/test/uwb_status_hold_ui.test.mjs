import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import test from 'node:test';

test('UWB instrument retains last readings, blinks only its point and clears on disconnect', () => {
  const html = fs.readFileSync(new URL('../frontend/mobile-preview-kimi-k26.html', import.meta.url), 'utf8');
  const nodes = new Map(), $ = id => {
    if (!nodes.has(id)) nodes.set(id, { dataset: {}, style: { setProperty() {} },
      classList: { values: new Set(), toggle(name, value) { value ? this.values.add(name) : this.values.delete(name); } } });
    return nodes.get(id);
  };
  let now = 1000;
  const state = { connected: true, uwb: { online: true, fresh: true, distanceM: 2,
    azimuthDeg: 30, elevationDeg: 5, measurementUpdatedAt: 1000, sensors: [{ receiverId: 1, tagId: 5 }] } };
  const scope = vm.createContext({ window: {}, $, state, Date: { now: () => now }, transport: null,
    localize: zh => zh, renderRelativePosition() {} });
  vm.runInContext(fs.readFileSync(new URL('../frontend/js/uwb-display-smoother.js', import.meta.url), 'utf8'), scope);
  vm.runInContext('var uwbDisplaySmoother=new window.MantaUwbDisplaySmoother(),uwbStatusHold=new window.MantaUwbDisplayHold();', scope);
  const start = html.indexOf('function uwbDisplayIdentity('), end = html.indexOf('var uwbDisplaySmoother=', start);
  vm.runInContext(html.slice(start, end), scope);
  vm.runInContext('renderUwbStatus()', scope);
  assert.equal($('#uwbDistance').textContent, '2.00 m');
  now = 1800; state.uwb.fresh = false; state.uwb.online = false;
  const before = structuredClone(state.uwb);
  vm.runInContext('renderUwbStatus()', scope);
  assert.equal($('#uwbDistance').textContent, '2.00 m');
  assert.equal($('#uwbDistanceLabel').textContent, '上次距离');
  assert.equal($('#uwbPanel').dataset.stale, 'true');
  assert.equal($('#uwbDot').classList.values.has('is-last-position'), true);
  assert.equal($('#uwbPanel').classList.values.has('is-last-position'), false);
  assert.deepEqual(state.uwb, before);
  state.connected = false; vm.runInContext('renderUwbStatus()', scope);
  assert.equal($('#uwbDistance').textContent, '-- m');
  assert.equal($('#uwbDot').classList.values.has('is-last-position'), false);
});
