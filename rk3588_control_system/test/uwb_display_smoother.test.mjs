import assert from 'node:assert/strict';
import test from 'node:test';
import '../frontend/js/uwb-display-smoother.js';
const Smoother = globalThis.MantaUwbDisplaySmoother;
const packet = (at, azimuthDeg = 0) => ({ online: true, fresh: true, distanceM: 3, azimuthDeg,
  measurementUpdatedAt: at, activeReceiverIds: [1], sensors: [{ receiverId: 1, tagId: 'a' }] });

test('display smoothing preserves packets, range and 40 Hz measurement clocks', () => {
  const f = new Smoother();
  for (let i = 0; i < 100; i++) {
    const p = packet(1000 + i * 25, i % 2 ? 2 : -2), before = structuredClone(p);
    const view = f.view(p, p.measurementUpdatedAt);
    assert.equal(view.valid, true); assert.equal(view.distanceM, 3);
    assert.equal(f.clock, p.measurementUpdatedAt); assert.deepEqual(p, before);
  }
});
test('duplicates cannot retarget and invalid or stale positions disappear immediately', () => {
  const f = new Smoother(); f.view(packet(1000), 1000);
  assert.equal(f.view(packet(1000, 60), 1025).azimuthDeg, 0);
  assert.equal(f.target, 0);
  for (const p of [{ ...packet(1050), fresh: false }, { ...packet(1050), online: false },
    { ...packet(1050), distanceM: null }, packet(100)]) {
    assert.equal(f.view(p, 1050).valid, false); assert.equal(f.angle, null);
  }
});
test('short circular path, source changes and gap reset', () => {
  const f = new Smoother(); f.view(packet(1000, 179), 1000); f.view(packet(1025, -179), 1025);
  assert.ok(Math.abs(f.view(packet(1050, -179), 1050).azimuthDeg) > 170);
  const p = packet(1075, 30); p.sensors[0].tagId = 'b';
  assert.equal(f.view(p, 1075).azimuthDeg, 30);
  assert.equal(f.view(packet(1500, -20), 1500).azimuthDeg, -20);
});
test('display step reaches 90 percent within 325 ms without synthetic source updates', () => {
  const f = new Smoother(120); f.view(packet(1000), 1000);
  f.view(packet(1025, 20), 1025);
  let view;
  for (let at = 1050; at <= 1325; at += 25) view = f.view(packet(1025, 99), at);
  assert.ok(view.azimuthDeg >= 18 && view.azimuthDeg < 20);
  assert.equal(f.clock, 1025); assert.equal(f.target, 20);
});

test('declared suspect position cannot be displayed even if fresh flag is inconsistent', () => {
  const f = new Smoother(); f.view(packet(1000), 1000);
  const p = { ...packet(1025, 60), measurementStatus: 'suspect' }, before = structuredClone(p);
  assert.equal(f.view(p, 1025).valid, false);
  assert.equal(f.angle, null);
  assert.deepEqual(p, before);
});

test('moving posterior follows each new 40 Hz clock without a second display delay', () => {
  const f = new Smoother(); f.view(packet(1000), 1000);
  for (let i = 1; i <= 80; i++) {
    const p = packet(1000 + i * 25, i * .75), before = structuredClone(p);
    p.sensors[0].angularFilterMode = 'moving';
    before.sensors[0].angularFilterMode = 'moving';
    const v = f.view(p, p.measurementUpdatedAt);
    assert.equal(v.azimuthDeg, p.azimuthDeg);
    assert.equal(f.clock, p.measurementUpdatedAt);
    assert.deepEqual(p, before);
  }
  const duplicate = packet(3000, 99); duplicate.sensors[0].angularFilterMode = 'moving';
  assert.equal(f.view(duplicate, 3025).azimuthDeg, 60);
  assert.equal(f.clock, 3000);
});

test('an unselected moving receiver cannot disable stationary display smoothing', () => {
  const f = new Smoother(), initial = packet(1000);
  initial.sensors.push({ receiverId: 2, tagId: 'b', angularFilterMode: 'stationary' });
  f.view(initial, 1000);
  const p = packet(1025, 20);
  p.sensors.push({ receiverId: 2, tagId: 'b', angularFilterMode: 'moving' });
  assert.equal(f.view(p, 1025).azimuthDeg, 0);
  assert.equal(f.moving, false);
  assert.equal(f.view({...p, fresh:false}, 1050).valid, false);
});
