function finite(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function toward(value, target, delta) {
  return value < target ? Math.min(target, value + delta) : Math.max(target, value - delta);
}

// Inputs have already passed the server's deadband/expo shaping.
// A horizontal input selects one FORWARD propeller; vertical-only input selects both.
export function singleSideTargets(input = {}, throttleLimit = 100, steeringLimit = 45, throttleScale = 100, steeringScale = 45) {
  const throttle = finite(input.throttle);
  const steering = finite(input.steering);
  const throttleSpan = Math.max(0, finite(throttleLimit, 100));
  const steeringSpan = Math.max(0, finite(steeringLimit, 45));
  if (!throttleSpan) return { left: 0, right: 0, side: 'neutral' };
  const selectedSide = input.side;
  const turn = selectedSide === 'both' ? 0 : steeringSpan > 0 ? Math.max(-1, Math.min(1, steering / steeringSpan)) : 0;
  const forward = Math.max(-1, Math.min(1, throttle / throttleSpan));
  const motorLimit = turn && steeringScale > 0
    ? Math.min(throttleSpan, 2 * steeringSpan / steeringScale * throttleScale)
    : throttleSpan;
  const singleMotor = selectedSide === 'left' || selectedSide === 'right' || (!selectedSide && turn !== 0);
  const direction = singleMotor ? 1 : Math.sign(forward);
  const requestedPower = input.powerPercent === undefined
    ? Math.min(1, Math.hypot(forward, turn)) * throttleSpan
    : Math.abs(finite(input.powerPercent));
  const power = Math.min(motorLimit, requestedPower) * direction;
  if (!power) return { left: 0, right: 0, side: 'neutral' };
  if (selectedSide === 'left' || (!selectedSide && turn > 0)) return { left: power, right: 0, side: 'left' };
  if (selectedSide === 'right' || (!selectedSide && turn < 0)) return { left: 0, right: power, side: 'right' };
  return { left: power, right: power, side: 'both' };
}

export function motorTargetsToManual(left, right, throttleSpan = 100, steeringSpan = 45) {
  return {
    throttle: (left + right) / 2,
    steering: (left - right) / 2 * steeringSpan / throttleSpan
  };
}

export class SingleSideRoverLimiter {
  constructor(config = {}) {
    this.config = {
      throttleAccelPerS: Math.max(1, finite(config.throttle_accel_percent_s, 40)),
      throttleDecelPerS: Math.max(1, finite(config.throttle_decel_percent_s, 80)),
      steeringRatePerS: 0,
      reverseNeutralMs: Math.max(0, finite(config.reverse_neutral_ms, 300)),
      sideSwitchNeutralMs: Math.max(0, finite(config.side_switch_neutral_ms, 150)),
      powerMode: config.power_mode === 'throttle' ? 'throttle' : 'vector',
      turnEnter: 0.12,
      turnExit: 0.06,
      maximumDtS: Math.max(0.02, Math.min(1, finite(config.maximum_dt_s, 0.1)))
    };
    this.throttleLimit = Math.max(0, finite(config.throttle_limit, 100));
    this.steeringLimit = Math.max(0, finite(config.steering_limit, 45));
    this.throttleSpan = Math.max(1, finite(config.throttle_span, 100));
    this.steeringSpan = Math.max(0, finite(config.steering_span, 45));
    this.reset();
  }

  reset(now = Date.now()) {
    this.target = { throttle: 0, steering: 0 };
    this.motors = { left: 0, right: 0 };
    this.output = { throttle: 0, steering: 0 };
    this.updatedAt = finite(now, Date.now());
    this.lastDirection = 0;
    this.reverseReadyAt = 0;
    this.intentDirection = 0;
    this.intentSide = 'both';
    this.activeSide = 'neutral';
    this.sideReadyAt = 0;
    return { ...this.output };
  }

  setTarget(target = {}) {
    this.target = { throttle: finite(target.throttle), steering: finite(target.steering) };
    // Raw X decides straight/turn mode; propulsion direction has no gesture memory.
    const horizontal = finite(target.rawSteering, this.target.steering) / Math.max(1, this.steeringLimit);
    if (!this.target.throttle && !this.target.steering) {
      this.intentDirection = 0;
      this.intentSide = 'both';
      return { ...this.target };
    }
    if (horizontal >= this.config.turnEnter) this.intentSide = 'left';
    else if (horizontal <= -this.config.turnEnter) this.intentSide = 'right';
    else if (Math.abs(horizontal) <= this.config.turnExit) this.intentSide = 'both';
    this.intentDirection = this.intentSide === 'both' ? Math.sign(this.target.throttle) : 1;
    return { ...this.target };
  }

  motorTargets() {
    return singleSideTargets({ ...this.target, side: this.intentSide,
      powerPercent: this.config.powerMode === 'throttle' ? Math.abs(this.target.throttle) : undefined
    }, this.throttleLimit, this.steeringLimit, this.throttleSpan, this.steeringSpan);
  }

  step(now = Date.now()) {
    const at = finite(now, Date.now());
    const dt = Math.max(0, Math.min(this.config.maximumDtS, (at - this.updatedAt) / 1000));
    this.updatedAt = at;
    if (!dt) return { ...this.output };
    const targets = this.motorTargets();
    const direction = Math.sign(targets.left || targets.right);
    const previouslyMoving = this.motors.left !== 0 || this.motors.right !== 0;
    const reversing = direction !== 0 && this.lastDirection !== 0 && direction !== this.lastDirection;
    // Entering a forward-only turn from reverse must not leave a single reverse
    // propeller running while its peer is cut. Stop both, then observe the dwell.
    if (reversing && (targets.side !== 'both' || targets.forwardTurn) && previouslyMoving) {
      this.motors.left = this.motors.right = 0;
      this.reverseReadyAt = at + this.config.reverseNeutralMs;
    }
    const changesSide = (targets.side === 'left' && this.activeSide === 'right') ||
      (targets.side === 'right' && this.activeSide === 'left');
    if (changesSide) {
      this.motors.left = this.motors.right = 0;
      this.sideReadyAt = at + this.config.sideSwitchNeutralMs;
      if (previouslyMoving) this.reverseReadyAt = at + this.config.reverseNeutralMs;
    }
    if (targets.side !== 'neutral') this.activeSide = targets.side;
    // The inactive motor returns directly to neutral, even during a side switch.
    if (targets.side === 'left') this.motors.right = 0;
    if (targets.side === 'right') this.motors.left = 0;
    const moving = this.motors.left !== 0 || this.motors.right !== 0;
    if (reversing && previouslyMoving && !moving) this.reverseReadyAt = at + this.config.reverseNeutralMs;
    const holdReverse = reversing && (moving || at < this.reverseReadyAt);
    for (const side of ['left', 'right']) {
      const target = holdReverse || at < this.sideReadyAt ? 0 : targets[side];
      const current = this.motors[side];
      const slowing = target === 0 || Math.abs(target) < Math.abs(current);
      this.motors[side] = toward(current, target,
        (slowing ? this.config.throttleDecelPerS : this.config.throttleAccelPerS) * dt);
    }
    const nowMoving = this.motors.left !== 0 || this.motors.right !== 0;
    if (previouslyMoving && !nowMoving) this.reverseReadyAt = at + this.config.reverseNeutralMs;
    if (nowMoving && !holdReverse) this.lastDirection = direction;
    this.output = motorTargetsToManual(this.motors.left, this.motors.right, this.throttleSpan, this.steeringSpan);
    return { ...this.output };
  }
}
