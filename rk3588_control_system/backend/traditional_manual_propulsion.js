import { TraditionalDualMotorLimiter } from './traditional_dual_motor.js';

// Traditional guidance requests thrust and yaw, not physical PWM. Use the same
// pilot-input conversion as the joystick, with its own dual-motor limiter.
export class TraditionalManualPropulsion {
  constructor(config = {}) {
    this.limiter = new TraditionalDualMotorLimiter(config);
  }

  reset(now = Date.now()) {
    return this.limiter.reset(now);
  }

  update({ now = Date.now(), result = {}, telemetry = {}, connected = false } = {}) {
    let reason = null;
    if (!connected) reason = 'pixhawk_offline';
    else if (String(telemetry.flightMode || '').toUpperCase() !== 'MANUAL') reason = 'manual_required';
    else if (telemetry.armed !== true) reason = 'armed_required';
    else if (telemetry.pilotSteering?.ready !== true ||
      ![0, 1, 2, 3].includes(telemetry.pilotSteering.type)) reason = 'pilot_steering_not_ready';
    else if (result.safe !== true) reason = result.reason || 'follow_unsafe';
    const throttle = Number(result.throttlePercent);
    const steering = Number(result.steeringDegrees);
    if (!Number.isFinite(throttle) || !Number.isFinite(steering)) reason ||= 'invalid_propulsion_target';
    if (reason || result.reason === 'holding_distance') {
      return { ...this.reset(now), safe: !reason, reason: reason || 'neutral' };
    }
    if (throttle === 0) {
      // The outer loop also passes through zero when reversing. Clear thrust
      // immediately but retain the shared limiter's direction and neutral dwell.
      const lastDirection = this.limiter.lastDirection;
      const moving = this.limiter.motors.left !== 0 || this.limiter.motors.right !== 0;
      const reverseReadyAt = moving ? now + this.limiter.config.reverseNeutralMs : this.limiter.reverseReadyAt;
      const output = this.reset(now);
      this.limiter.lastDirection = lastDirection;
      this.limiter.reverseReadyAt = reverseReadyAt;
      return { ...output, safe: true, reason: 'neutral' };
    }
    // Distance recovery remains straight reverse with both motors.
    this.limiter.setTarget({ throttle, steering: throttle < 0 ? 0 : steering });
    return { ...this.limiter.step(now), safe: true, reason: 'tracking' };
  }
}
