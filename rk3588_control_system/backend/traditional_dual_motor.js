import { motorTargetsToManual } from './single_side_rover.js';

const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));
const finite = (x, fallback) => Number.isFinite(Number(x)) ? Number(x) : fallback;
const toward = (x, target, delta) => x < target ? Math.min(target, x + delta) : Math.max(target, x - delta);

// Used only by traditional following. Both propellers carry forward thrust;
// steering changes their relative thrust without introducing counter-rotation.
export class TraditionalDualMotorLimiter {
  constructor(config = {}) {
    this.throttleLimit = Math.max(0, finite(config.throttle_limit, 100));
    this.steeringLimit = Math.max(0, finite(config.steering_limit, 45));
    this.throttleSpan = Math.max(1, finite(config.throttle_span, 100));
    this.steeringSpan = Math.max(1, finite(config.steering_span, 45));
    this.config = {
      throttleAccelPerS: Math.max(1, finite(config.throttle_accel_percent_s, 40)),
      throttleDecelPerS: Math.max(1, finite(config.throttle_decel_percent_s, 80)),
      reverseNeutralMs: Math.max(0, finite(config.reverse_neutral_ms, 300)),
      maximumDtS: clamp(finite(config.maximum_dt_s, 0.1), 0.02, 1),
      steeringShare: 0.8
    };
    this.reset();
  }

  reset(now = Date.now()) {
    this.target = { throttle: 0, steering: 0 };
    this.motors = { left: 0, right: 0 };
    this.output = { throttle: 0, steering: 0 };
    this.updatedAt = now;
    this.lastDirection = 0;
    this.reverseReadyAt = 0;
    return { ...this.output };
  }

  setTarget(input = {}) {
    this.target = {
      throttle: clamp(finite(input.throttle, 0), -this.throttleLimit, this.throttleLimit),
      steering: clamp(finite(input.steering, 0), -this.steeringLimit, this.steeringLimit)
    };
    return { ...this.target };
  }

  step(now = Date.now()) {
    const dt = clamp((now - this.updatedAt) / 1000, 0, this.config.maximumDtS);
    this.updatedAt = now;
    if (!dt) return { ...this.output };
    const throttle = this.target.throttle;
    const direction = Math.sign(throttle);
    const turn = clamp(this.target.steering / this.steeringSpan * this.throttleSpan,
      -Math.abs(throttle) * this.config.steeringShare, Math.abs(throttle) * this.config.steeringShare);
    const requested = { left: throttle + turn, right: throttle - turn };
    const maximum = Math.max(Math.abs(requested.left), Math.abs(requested.right));
    const scale = maximum > this.throttleLimit ? this.throttleLimit / maximum : 1;
    requested.left *= scale;
    requested.right *= scale;
    const moving = this.motors.left !== 0 || this.motors.right !== 0;
    const reversing = direction !== 0 && this.lastDirection !== 0 && direction !== this.lastDirection;
    const hold = reversing && (moving || now < this.reverseReadyAt);
    for (const side of ['left', 'right']) {
      const target = hold ? 0 : requested[side];
      const slowing = Math.abs(target) < Math.abs(this.motors[side]);
      this.motors[side] = toward(this.motors[side], target,
        (slowing ? this.config.throttleDecelPerS : this.config.throttleAccelPerS) * dt);
    }
    const nowMoving = this.motors.left !== 0 || this.motors.right !== 0;
    if (moving && !nowMoving) this.reverseReadyAt = now + this.config.reverseNeutralMs;
    if (nowMoving && !hold) this.lastDirection = direction;
    this.output = motorTargetsToManual(this.motors.left, this.motors.right, this.throttleSpan, this.steeringSpan);
    return { ...this.output };
  }
}
