function clamp(value, minimum, maximum) {
  return Math.min(maximum, Math.max(minimum, value));
}

function smoothStep01(value) {
  const x = clamp(value, 0, 1);
  return x * x * (3 - 2 * x);
}

function rewriteChecksum(frame) {
  let checksum = 0;
  for (let index = 2; index <= 41; index += 1) checksum ^= frame[index];
  frame[42] = checksum & 0xff;
}

function limitRateAtAngle(rate, angleDeg, minimumDeg, maximumDeg, brakeDeg) {
  if (rate === 0) return 0;
  if (rate < 0) {
    const remaining = angleDeg - minimumDeg;
    if (remaining <= 0) return 0;
    if (remaining < brakeDeg) return Math.round(rate * smoothStep01(remaining / brakeDeg));
    return rate;
  }
  const remaining = maximumDeg - angleDeg;
  if (remaining <= 0) return 0;
  if (remaining < brakeDeg) return Math.round(rate * smoothStep01(remaining / brakeDeg));
  return rate;
}

export function limitOutgoingGimbalFrame(frame, options = {}) {
  if (!Buffer.isBuffer(frame) || frame.length !== 44 || frame[0] !== 0xfb || frame[1] !== 0x2c || frame[43] !== 0xf0) {
    return { frame, limited: false, reason: '' };
  }

  const pitchMinDeg = Number(options.pitchMinDeg ?? -150);
  const pitchMaxDeg = Number(options.pitchMaxDeg ?? 150);
  const yawMinDeg = Number(options.yawMinDeg ?? -135);
  const yawMaxDeg = Number(options.yawMaxDeg ?? 135);
  const ratePitchMinDeg = Number(options.ratePitchMinDeg ?? pitchMinDeg);
  const brakeDeg = Math.max(0.5, Number(options.softLimitBrakeDeg || 5));
  const now = Number(options.now ?? Date.now());
  const feedbackMaxAgeMs = Math.max(50, Number(options.feedbackMaxAgeMs || 300));
  const feedback = options.feedback || {};
  const age = now - feedback.updatedAt;
  const feedbackFresh = feedback.checksumValid === true
    && Number.isFinite(feedback.pitchDeg)
    && Number.isFinite(feedback.yawDeg)
    && Number.isFinite(feedback.updatedAt) && feedback.updatedAt > 0
    && age >= 0 && age <= feedbackMaxAgeMs;
  const output = Buffer.from(frame);
  let limited = false;
  let reason = '';

  if (output[2] === 0x72 || output[2] === 0x80) {
    const yaw = output.readInt16LE(3) / 100;
    const safeYaw = clamp(yaw, yawMinDeg, yawMaxDeg);
    if (safeYaw !== yaw) {
      output.writeInt16LE(Math.round(safeYaw * 100), 3);
      limited = true;
      reason = 'absolute_yaw_limit';
    }
    const requested = output.readInt16LE(5) / 100;
    const safe = clamp(requested, pitchMinDeg, pitchMaxDeg);
    if (safe !== requested) {
      output.writeInt16LE(Math.round(safe * 100), 5);
      limited = true;
      reason = 'absolute_pitch_limit';
    }
  }

  if (output[37] === 0x70) {
    for (const [offset, angle, min, max, axis] of [
      [38, feedback.yawDeg, yawMinDeg, yawMaxDeg, 'yaw'],
      [40, feedback.pitchDeg, ratePitchMinDeg, pitchMaxDeg, 'pitch']
    ]) {
      const requested = output.readInt16LE(offset);
      const safe = feedbackFresh ? limitRateAtAngle(requested, angle, min, max, brakeDeg) : 0;
      if (safe !== requested) {
        output.writeInt16LE(safe, offset);
        limited = true;
        reason = feedbackFresh ? `rate_${axis}_limit` : `rate_${axis}_feedback_guard`;
      }
    }
  }

  if (limited) rewriteChecksum(output);
  return { frame: output, limited, reason };
}
