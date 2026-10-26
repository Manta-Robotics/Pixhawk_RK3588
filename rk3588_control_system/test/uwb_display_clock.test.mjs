import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import vm from 'node:vm';
import '../frontend/js/uwb-display-smoother.js';
import '../frontend/js/uwb-raw-display.js';

function setup(wall) {
  let tick = 100;
  const scope = { window: {}, performance: { now: () => tick }, Date: class extends Date { static now() { return wall; } }, console };
  vm.runInNewContext(fs.readFileSync(new URL('../frontend/js/manta-app-transport.js', import.meta.url), 'utf8'), scope);
  const transport = scope.window.MantaAppTransport.create('live');
  transport.lastLatency = 40;
  return { transport, advance: ms => { tick += ms; } };
}
const at = 1793000000000;
const packet = { online: true, fresh: true, measurementStatus: 'trusted', distanceM: 2, azimuthDeg: 20,
  measurementUpdatedAt: at, activeReceiverIds: [1], sensors: [{ receiverId: 1, tagId: 12, valid: true,
    rawDistanceM: 2, rawAzimuthDeg: 20, rawDistanceUpdatedAt: at, receivedMeasurementAt: at }] };

for (const skew of [-86400000, 1500, 86400000]) {
  test(`fresh raw and trusted targets render with client clock skew ${skew}`, () => {
    const { transport } = setup(at + skew);
    transport.applyStatus({ timestamp: new Date(at).toISOString(), data: {} });
    assert.equal(transport.boardNow(), at + 40);
    assert.equal(new MantaUwbDisplaySmoother().view(packet, transport.boardNow()).valid, true);
    assert.equal(new MantaUwbRawDisplay().view(packet, transport.boardNow(), true).valid, true);
  });
}
test('repeated status packets cannot refresh a frozen measurement; server-only events preserve clock', () => {
  const { transport, advance } = setup(at + 5000);
  transport.applyStatus({ timestamp: at, data: {} });
  const trusted = new MantaUwbDisplaySmoother(), raw = new MantaUwbRawDisplay();
  assert.equal(trusted.view(packet, transport.boardNow()).valid, true);
  assert.equal(raw.view(packet, transport.boardNow(), true).valid, true);
  advance(701);
  transport.applyStatus({ data: {} });
  assert.equal(transport.boardNow(), at + 741);
  transport.applyStatus({ timestamp: at + 701, data: {} });
  assert.equal(trusted.view(packet, transport.boardNow()).valid, false);
  assert.equal(raw.view(packet, transport.boardNow(), true).valid, false);
});
test('rejected measurements remain rejected with synchronized display clock', () => {
  const { transport } = setup(at - 5000);
  transport.applyStatus({ timestamp: at, data: {} });
  assert.equal(new MantaUwbDisplaySmoother().view({ ...packet, measurementStatus: 'suspect' }, transport.boardNow()).valid, false);
});

test('asymmetric request/response timing does not hide a fresh sample as future-dated', () => {
  const { transport } = setup(at + 5000);
  transport.lastLatency = 100;
  transport.applyStatus({ timestamp: at, data: {} });
  const fresh = { ...packet, measurementUpdatedAt: at + 85 };
  assert.equal(transport.boardNow(), at + 100);
  assert.equal(new MantaUwbDisplaySmoother().view(fresh, transport.boardNow()).valid, true);
});
