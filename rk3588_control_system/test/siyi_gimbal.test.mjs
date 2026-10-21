import assert from 'node:assert/strict';
import test from 'node:test';
import { SiyiGimbal, siyiCrc16, encodeSiyiPacket, decodeSiyiPackets } from '../backend/siyi_gimbal.js';

function envelope(command = 0, a = 0, b = 0, joystick = 0) {
  const f = Buffer.alloc(44);
  f[0] = 0xfb; f[1] = 0x2c; f[2] = command; f[37] = joystick; f[43] = 0xf0;
  f.writeInt16LE(a, joystick ? 38 : 3); f.writeInt16LE(b, joystick ? 40 : 5);
  for (let i = 2; i <= 41; i += 1) f[42] ^= f[i];
  return f;
}
function packetFromHex(hex) { return decodeSiyiPackets(Buffer.from(hex, 'hex')).packets[0]; }
const fresh = { checksumValid: true, updatedAt: 1000, yawDeg: 0, pitchDeg: 0 };
const dataOf = (b) => decodeSiyiPackets(b).packets[0];

test('CRC matches the published SDK hardware-ID request', () => {
  assert.equal(encodeSiyiPacket(2).toString('hex'), '556601000000000207f4');
  assert.equal(siyiCrc16(Buffer.from('123456789')), 0x31c3);
});

test('decoder validates a real A8 response and rejects CRC corruption', () => {
  const b = Buffer.from('5566020c0003000d0300070700000000050002000c9e', 'hex');
  assert.equal(dataOf(b).command, 13);
  const bad = Buffer.from(b); bad[9] ^= 1;
  assert.equal(decodeSiyiPackets(bad).packets.length, 0);
  const prefix = decodeSiyiPackets(Buffer.concat([Buffer.from([0, 4]), b.subarray(0, 9)]));
  assert.equal(prefix.packets.length, 0);
  assert.equal(decodeSiyiPackets(Buffer.concat([prefix.remainder, b.subarray(9), b])).packets.length, 2);
  assert.equal(decodeSiyiPackets(Buffer.concat([Buffer.from('556601ffff0000000000', 'hex'), b])).packets.length, 1);
});

test('only acknowledged attitude updates feedback; upside-down center normalizes to zero', () => {
  const g = new SiyiGimbal();
  const p = packetFromHex('5566020c0003000d0300070700000000050002000c9e');
  assert.equal(g.feedback(p), null, 'wait for mounting configuration');
  assert.equal(g.feedback(packetFromHex('556602070001000a00000002010200542c')), null);
  const f = g.feedback(p, {}, 1234);
  assert.ok(Math.abs(f.pitchDeg - 0.1) < 0.001);
  assert.ok(Math.abs(f.yawDeg - 0.3) < 0.001); assert.equal(f.gyroPitchDps, -0.5);
  assert.equal(f.tfCardInserted, false); assert.equal(f.updatedAt, 1234);
  assert.equal(g.feedback({ ...p, control: 1 }), null);
  const normal = new SiyiGimbal(); normal.mounting = 1;
  const b = Buffer.alloc(12); b.writeInt16LE(-50, 2);
  assert.equal(normal.feedback({ command: 13, control: 2, payload: b }).pitchDeg, 5);
});

test('absolute angle conversion enforces the ship horizon and stops on stale feedback', () => {
  const g = new SiyiGimbal();
  const p = dataOf(g.translate(envelope(0x72, 16000, 15000), fresh, 1001)[0]);
  assert.equal(p.command, 0x0e);
  assert.equal(p.payload.readInt16LE(0), 1350);
  assert.equal(p.payload.readInt16LE(2), -900);
  assert.equal(dataOf(g.translate(envelope(0x72, 0, -5000), fresh, 1001)[0]).payload.readInt16LE(2), 0);
  const stale = dataOf(g.translate(envelope(0x72, 500, 500), fresh, 1400)[0]);
  assert.equal(stale.command, 7); assert.deepEqual([...stale.payload], [0, 0]);
});

test('rate conversion enforces direction, command cap, limit guard and release stop', () => {
  const g = new SiyiGimbal({ rate_units_per_dps: 1, max_rate_command: 40 });
  const f = envelope(0, 100, 100, 0x70);
  assert.deepEqual([...dataOf(g.translate(f, fresh, 1001)[0]).payload], [40, 216]);
  const limited = dataOf(g.translate(f, { ...fresh, yawDeg: 135, pitchDeg: 90 }, 1001)[0]);
  assert.deepEqual([...limited.payload], [0, 0]);
  assert.deepEqual([...dataOf(g.translate(envelope(), fresh, 1001)[0]).payload], [0, 0]);
});

test('repeated legacy home bursts send one SIYI home and unsupported commands never pass through', () => {
  const g = new SiyiGimbal();
  const home = envelope(0x71);
  assert.equal(dataOf(g.translate(home, fresh, 1001)[0]).command, 8);
  assert.equal(g.translate(home, fresh, 1001).length, 0);
  g.translate(envelope(), fresh, 1001);
  assert.equal(g.translate(home, fresh, 1001).length, 1);
  for (const cmd of [0x33, 0x37, 0x90, 0x3e]) assert.throws(() => g.translate(envelope(cmd), fresh, 1001), /unavailable/);
  assert.equal(dataOf(g.translate(envelope(0x45), fresh, 1001)[0]).payload.toString('hex'), '0100');
});


test('ship camera never accepts an upward target at the SIYI wire boundary', () => {
  const g = new SiyiGimbal();
  for (const requested of [-2500, -1, 0, 1, 4500, 9000, 12000]) {
    const p = dataOf(g.translate(envelope(0x72, 0, requested), fresh, 1001)[0]);
    const nativePitch = p.payload.readInt16LE(2);
    assert.ok(nativePitch <= 0 && nativePitch >= -900, `unsafe native pitch ${nativePitch}`);
    assert.equal(nativePitch, Math.round(-Math.max(0, Math.min(90, requested / 100)) * 10) || 0);
  }
});

test('ship camera upward rate stops at the horizon; downward recovery remains allowed', () => {
  const g = new SiyiGimbal();
  for (const pitchDeg of [-10, 0]) {
    const feedback = { ...fresh, pitchDeg };
    const upward = dataOf(g.translate(envelope(0, 10, -20, 0x70), feedback, 1001)[0]);
    assert.equal(upward.payload.readInt8(0), 10, 'yaw remains available');
    assert.equal(upward.payload.readInt8(1), 0, 'upward pitch is stopped');
    const downward = dataOf(g.translate(envelope(0, 0, 20, 0x70), feedback, 1001)[0]);
    assert.equal(downward.payload.readInt8(1), -20, 'can recover toward the allowed range');
  }
  const towardHorizon = dataOf(g.translate(envelope(0, 0, -20, 0x70), { ...fresh, pitchDeg: 30 }, 1001)[0]);
  assert.equal(towardHorizon.payload.readInt8(1), 20, 'can look up from below the horizon');
  const stale = dataOf(g.translate(envelope(0, 0, -20, 0x70), fresh, 1400)[0]);
  assert.equal(stale.payload.readInt8(1), 0);
});
