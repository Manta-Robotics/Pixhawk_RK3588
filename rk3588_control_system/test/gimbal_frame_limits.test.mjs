import assert from 'node:assert/strict';
import test from 'node:test';
import { limitOutgoingGimbalFrame } from '../backend/gimbal_frame_limits.js';

function checksum(frame) {
  let value = 0;
  for (let index = 2; index <= 41; index += 1) value ^= frame[index];
  return value & 0xff;
}

function frame({ command = 0, pitchDeg = 0, pitchRate = 0 } = {}) {
  const output = Buffer.alloc(44, 0);
  output[0] = 0xfb;
  output[1] = 0x2c;
  output[2] = command;
  output.writeInt16LE(Math.round(pitchDeg * 100), 5);
  if (pitchRate !== 0) {
    output[37] = 0x70;
    output.writeInt16LE(pitchRate, 40);
  }
  output[42] = checksum(output);
  output[43] = 0xf0;
  return output;
}

const limits = { pitchMinDeg: 0, pitchMaxDeg: 150, ratePitchMinDeg: 2.5, softLimitBrakeDeg: 5 };

test('absolute pitch commands cannot move below the home angle', () => {
  const result = limitOutgoingGimbalFrame(frame({ command: 0x72, pitchDeg: -12 }), limits);
  assert.equal(result.frame.readInt16LE(5), 0);
  assert.equal(result.reason, 'absolute_pitch_limit');
  assert.equal(result.frame[42], checksum(result.frame));
});

test('upward absolute pitch commands remain unchanged', () => {
  const input = frame({ command: 0x72, pitchDeg: 12 });
  const result = limitOutgoingGimbalFrame(input, limits);
  assert.equal(result.limited, false);
  assert.equal(result.frame.readInt16LE(5), 1200);
});

test('downward rate is blocked until pitch feedback is available', () => {
  const result = limitOutgoingGimbalFrame(frame({ pitchRate: -20 }), limits);
  assert.equal(result.frame.readInt16LE(40), 0);
  assert.equal(result.reason, 'rate_pitch_feedback_guard');
});

test('downward rate stops at zero and slows near the lower limit', () => {
  const now = 10000;
  const stopped = limitOutgoingGimbalFrame(frame({ pitchRate: -20 }), {
    ...limits,
    now,
    feedback: { pitchDeg: 2.5, updatedAt: now, checksumValid: true }
  });
  const slowed = limitOutgoingGimbalFrame(frame({ pitchRate: -20 }), {
    ...limits,
    now,
    feedback: { pitchDeg: 4.5, updatedAt: now, checksumValid: true }
  });
  const allowed = limitOutgoingGimbalFrame(frame({ pitchRate: -20 }), {
    ...limits,
    now,
    feedback: { pitchDeg: 10, updatedAt: now, checksumValid: true }
  });
  assert.equal(stopped.frame.readInt16LE(40), 0);
  assert.ok(slowed.frame.readInt16LE(40) < 0 && slowed.frame.readInt16LE(40) > -20);
  assert.equal(allowed.frame.readInt16LE(40), -20);
});
