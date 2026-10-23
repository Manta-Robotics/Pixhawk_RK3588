import { SingleSideRoverLimiter } from './single_side_rover.js';

// Keep both propellers in the same direction. At maximum requested yaw the
// weaker propeller retains 20% of the base thrust instead of being turned off.
export function dualThrustTargets(input = {}, throttleLimit = 100, steeringLimit = 45, throttleSpan = 100, steeringSpan = 45) {
  const throttle = Math.max(-throttleLimit, Math.min(throttleLimit, Number(input.throttle) || 0));
  const steering = Math.max(-steeringLimit, Math.min(steeringLimit, Number(input.steering) || 0));
  const turning = input.turning === undefined ? steering !== 0 : input.turning;
  const throttleRatio = throttleLimit > 0 ? throttle / throttleLimit : 0;
  const turnRatio = turning && steeringLimit > 0 ? steering / steeringLimit : 0;
  const base = Math.min(1, Math.hypot(throttleRatio, turnRatio)) * Math.max(0, throttleLimit);
  if (!base) return { left: 0, right: 0, side: 'neutral', forwardTurn: false };
  const direction = turning ? 1 : Math.sign(throttle);
  const difference = turning && steeringSpan > 0
    ? Math.sign(steering) * Math.min(Math.abs(steering / steeringSpan * throttleSpan), base * 0.8) : 0;
  const scale = Math.min(1, throttleLimit / (base + Math.abs(difference)));
  return { left: (direction * base + difference) * scale,
    right: (direction * base - difference) * scale, side: 'both', forwardTurn: turning };
}

// Only the target mixer changes. Deadband hysteresis, acceleration, deceleration
// and reversal dwell continue to use the existing, tested per-propeller limiter.
export class DualThrustRoverLimiter extends SingleSideRoverLimiter {
  motorTargets() {
    return dualThrustTargets({ ...this.target, turning: this.intentSide !== 'both' },
      this.throttleLimit, this.steeringLimit, this.throttleSpan, this.steeringSpan);
  }
}
