function finite(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function clamp(value, minimum, maximum) {
  return Math.min(maximum, Math.max(minimum, value));
}

function moveToward(current, target, maximumDelta) {
  if (target > current) return Math.min(target, current + maximumDelta);
  if (target < current) return Math.max(target, current - maximumDelta);
  return current;
}

export function shapeRoverAxis(value, { limit, deadband = 0, expo = 1 } = {}) {
  const safeLimit = Math.max(0.001, Math.abs(finite(limit, 1)));
  const safeDeadband = clamp(Math.abs(finite(deadband, 0)), 0, safeLimit * 0.95);
  const magnitude = clamp(Math.abs(finite(value, 0)), 0, safeLimit);
  if (magnitude <= safeDeadband) return 0;
  const normalized = (magnitude - safeDeadband) / (safeLimit - safeDeadband);
  return Math.sign(value) * safeLimit * normalized ** Math.max(1, finite(expo, 1));
}

export class RoverSlewLimiter {
  constructor(config = {}) {
    this.config = {
      throttleAccelPerS: Math.max(1, finite(config.throttle_accel_percent_s, 40)),
      throttleDecelPerS: Math.max(1, finite(config.throttle_decel_percent_s, 80)),
      steeringRatePerS: Math.max(1, finite(config.steering_slew_deg_s, 40)),
      reverseNeutralMs: Math.max(0, finite(config.reverse_neutral_ms, 300)),
      maximumDtS: clamp(finite(config.maximum_dt_s, 0.2), 0.02, 1)
    };
    this.reset();
  }

  reset(now = Date.now()) {
    this.target = { throttle: 0, steering: 0 };
    this.output = { throttle: 0, steering: 0 };
    this.updatedAt = finite(now, Date.now());
    this.lastThrottleSign = 0;
    this.reverseReadyAt = 0;
    return { ...this.output };
  }

  setTarget(target = {}) {
    this.target = {
      throttle: finite(target.throttle, 0),
      steering: finite(target.steering, 0)
    };
    return { ...this.target };
  }

  step(now = Date.now()) {
    const currentAt = finite(now, Date.now());
    const dt = clamp((currentAt - this.updatedAt) / 1000, 0, this.config.maximumDtS);
    this.updatedAt = currentAt;
    if (dt <= 0) return { ...this.output };

    const steeringDelta = this.config.steeringRatePerS * dt;
    this.output.steering = moveToward(this.output.steering, this.target.steering, steeringDelta);

    const currentThrottle = this.output.throttle;
    const targetThrottle = this.target.throttle;
    const currentSign = Math.sign(currentThrottle);
    const targetSign = Math.sign(targetThrottle);

    if (currentSign !== 0 && targetSign !== 0 && currentSign !== targetSign) {
      this.output.throttle = moveToward(currentThrottle, 0, this.config.throttleDecelPerS * dt);
      if (this.output.throttle === 0) {
        this.reverseReadyAt = currentAt + this.config.reverseNeutralMs;
      }
      return { ...this.output };
    }

    if (currentThrottle === 0 && targetSign !== 0 && this.lastThrottleSign !== 0 && targetSign !== this.lastThrottleSign && currentAt < this.reverseReadyAt) {
      return { ...this.output };
    }

    const slowing = targetSign === 0 || (currentSign === targetSign && Math.abs(targetThrottle) < Math.abs(currentThrottle));
    const throttleRate = slowing ? this.config.throttleDecelPerS : this.config.throttleAccelPerS;
    const nextThrottle = moveToward(currentThrottle, targetThrottle, throttleRate * dt);
    if (currentThrottle !== 0 && nextThrottle === 0) {
      this.reverseReadyAt = currentAt + this.config.reverseNeutralMs;
    }
    this.output.throttle = nextThrottle;
    if (nextThrottle !== 0) this.lastThrottleSign = Math.sign(nextThrottle);
    return { ...this.output };
  }
}
