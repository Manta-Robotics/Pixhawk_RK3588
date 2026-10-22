// SIYI A8 mini SDK: https://siyi.biz/siyi_file/A8%20mini/SDK%20-%20V0.1.1.pdf
// Application coordinates: yaw right positive, pitch down positive.
// Ship camera policy: horizontal (0 degrees) through straight down (90 degrees).
// Enforce the horizon at the wire boundary as well as in the control planner.
// The existing control planner uses 44-byte internal command envelopes. Only
// SIYI packets produced here may reach an A8 socket; envelopes are never sent.
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const wrap = (v) => ((v + 180) % 360 + 360) % 360 - 180;

export function siyiCrc16(bytes) {
  let crc = 0;
  for (const byte of bytes) {
    crc ^= byte << 8;
    for (let bit = 0; bit < 8; bit += 1) crc = ((crc << 1) ^ (crc & 0x8000 ? 0x1021 : 0)) & 0xffff;
  }
  return crc;
}

export function encodeSiyiPacket(command, payload = Buffer.alloc(0), sequence = 0) {
  if (payload.length > 1024) throw new Error('SIYI payload too long');
  const frame = Buffer.alloc(10 + payload.length);
  frame[0] = 0x55; frame[1] = 0x66; frame[2] = 1;
  frame.writeUInt16LE(payload.length, 3);
  frame.writeUInt16LE(sequence & 0xffff, 5);
  frame[7] = command;
  payload.copy(frame, 8);
  frame.writeUInt16LE(siyiCrc16(frame.subarray(0, -2)), frame.length - 2);
  return frame;
}

export function decodeSiyiPackets(input) {
  let buffer = Buffer.from(input);
  const packets = [];
  while (buffer.length >= 2) {
    const offset = buffer.indexOf(Buffer.from([0x55, 0x66]));
    if (offset < 0) return { packets, remainder: buffer.subarray(-1) };
    buffer = buffer.subarray(offset);
    if (buffer.length < 10) break;
    const length = buffer.readUInt16LE(3);
    if (length > 1024) { buffer = buffer.subarray(1); continue; }
    if (buffer.length < length + 10) break;
    const frame = buffer.subarray(0, length + 10);
    if (siyiCrc16(frame.subarray(0, -2)) !== frame.readUInt16LE(frame.length - 2)) {
      buffer = buffer.subarray(1); continue;
    }
    packets.push({ command: frame[7], control: frame[2], sequence: frame.readUInt16LE(5), payload: frame.subarray(8, -2) });
    buffer = buffer.subarray(frame.length);
  }
  return { packets, remainder: buffer };
}

export class SiyiGimbal {
  constructor(options = {}) {
    this.sequence = 0;
    this.options = options;
    this.mounting = null;
    this.lastOneShot = '';
    this.configuration = {};
  }

  packet(command, payload) { return encodeSiyiPacket(command, payload, this.sequence++); }

  feedback(packet, previous = {}, now = Date.now()) {
    const p = packet.payload;
    // Only response packets may update telemetry. Bad CRC frames never arrive here.
    if (!(packet.control & 2)) return null;
    if (packet.command === 0x0a && p.length >= 6) {
      this.mounting = p[5];
      this.configuration = { cameraRecording: p[3] === 1, recordStatus: p[3], tfCardInserted: p[3] !== 2, mounting: p[5] };
      return null;
    }
    if (packet.command !== 0x0d || p.length !== 12 || ![1, 2].includes(this.mounting)) return null;
    const rawPitchDeg = p.readInt16LE(2) / 10;
    // A8 upside-down mounting reports ~180 degrees at center (verified on hardware).
    const nativePitch = wrap(rawPitchDeg - (this.mounting === 2 ? 180 : 0));
    return {
      ...previous, ...this.configuration, protocol: 'siyi',
      yawDeg: wrap(p.readInt16LE(0) / 10), pitchDeg: -nativePitch,
      rollDeg: p.readInt16LE(4) / 10,
      gyroYawDps: p.readInt16LE(6) / 10, gyroPitchDps: -p.readInt16LE(8) / 10,
      gyroRollDps: p.readInt16LE(10) / 10,
      rawPitchDeg, checksumValid: true, updatedAt: now
    };
  }

  translate(frame, feedback, now = Date.now()) {
    if (!Buffer.isBuffer(frame) || frame.length !== 44 || frame[0] !== 0xfb || frame[1] !== 0x2c || frame[43] !== 0xf0) {
      throw new Error('Invalid internal gimbal command');
    }
    let check = 0;
    for (let i = 2; i <= 41; i += 1) check ^= frame[i];
    if (check !== frame[42]) throw new Error('Invalid internal gimbal checksum');
    const command = frame[2];
    let id; let payload;
    const fresh = feedback?.checksumValid === true && now - Number(feedback.updatedAt) <= 350;
    if (command === 0) {
      if (![0, 0x70].includes(frame[37])) throw new Error('SIYI requires guided_angle click control');
      this.lastOneShot = '';
      payload = Buffer.alloc(2);
      if (fresh && frame[37] === 0x70) {
        let yaw = frame.readInt16LE(38);
        let pitch = frame.readInt16LE(40);
        if ((yaw < 0 && feedback.yawDeg <= -135) || (yaw > 0 && feedback.yawDeg >= 135)) yaw = 0;
        if ((pitch < 0 && feedback.pitchDeg <= 0) || (pitch > 0 && feedback.pitchDeg >= 90)) pitch = 0;
        // SDK 0x07 uses signed percentage, not degrees/second. Keep an explicit
        // commissioning gain and cap instead of treating them as equal units.
        const gain = Number(this.options.rate_units_per_dps ?? 1);
        const cap = Number(this.options.max_rate_command ?? 40);
        payload.writeInt8(Math.round(clamp(yaw * gain, -cap, cap)), 0);
        payload.writeInt8(Math.round(clamp(-pitch * gain, -cap, cap)), 1);
      }
      return [this.packet(0x07, payload)];
    }
    if (command === 0x72) {
      if (!fresh) return [this.packet(0x07, Buffer.alloc(2))];
      payload = Buffer.alloc(4);
      payload.writeInt16LE(Math.round(clamp(frame.readInt16LE(3) / 100, -135, 135) * 10), 0);
      payload.writeInt16LE(Math.round(-clamp(frame.readInt16LE(5) / 100, 0, 90) * 10), 2);
      this.lastOneShot = '';
      return [this.packet(0x0e, payload)];
    }
    if (command === 0x71) { id = 0x08; payload = Buffer.from([1]); }
    else if (command === 0x45) { id = 0x0f; payload = Buffer.from([1, 0]); }
    else throw new Error(`Command 0x${command.toString(16)} is unavailable on SIYI A8 mini`);
    const key = `${id}:${payload.toString('hex')}`;
    if (this.lastOneShot === key) return [];
    this.lastOneShot = key;
    return [this.packet(id, payload)];
  }
}
