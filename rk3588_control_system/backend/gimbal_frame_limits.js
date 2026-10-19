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
  const ratePitchMinDeg = Number(options.ratePitchMinDeg ?? pitchMinDeg);
  const brakeDeg = Math.max(0.5, Number(options.softLimitBrakeDeg || 5));
  const now = Number(options.now ?? Date.now());
  const feedbackMaxAgeMs = Math.max(50, Number(options.feedbackMaxAgeMs || 300));
  const feedback = options.feedback || {};
  const feedbackFresh = feedback.checksumValid !== false
    && Number.isFinite(Number(feedback.pitchDeg))
    && Number(feedback.updatedAt) > 0
    && now - Number(feedback.updatedAt) <= feedbackMaxAgeMs;
  const output = Buffer.from(frame);
  let limited = false;
  let reason = '';

  if (output[2] === 0x72 || output[2] === 0x80) {
    const requested = output.readInt16LE(5) / 100;
    const safe = clamp(requested, pitchMinDeg, pitchMaxDeg);
    if (safe !== requested) {
      output.writeInt16LE(Math.round(safe * 100), 5);
      limited = true;
      reason = 'absolute_pitch_limit';
    }
  }

  if (output[37] === 0x70) {
    const requested = output.readInt16LE(40);
    const safe = feedbackFresh
      ? limitRateAtAngle(requested, Number(feedback.pitchDeg), ratePitchMinDeg, pitchMaxDeg, brakeDeg)
      : (requested < 0 ? 0 : requested);
    if (safe !== requested) {
      output.writeInt16LE(safe, 40);
      limited = true;
      reason = feedbackFresh ? 'rate_pitch_limit' : 'rate_pitch_feedback_guard';
    }
  }

  if (limited) rewriteChecksum(output);
  return { frame: output, limited, reason };
}
