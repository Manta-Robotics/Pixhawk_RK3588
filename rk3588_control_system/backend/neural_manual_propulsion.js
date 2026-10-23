import { shapeRoverAxis } from './rover_slew.js';
import { DualThrustRoverLimiter } from './dual_thrust_rover.js';

// The neural outer loop supplies virtual stick axes. Shape and execute them
// as the manual joystick, with a separate state for follow sessions.
export class NeuralManualPropulsion {
  constructor(config = {}) {
    this.limiter = new DualThrustRoverLimiter(config);
    this.throttleDeadband = config.throttle_deadband ?? 4;
    this.steeringDeadband = config.steering_deadband ?? 2;
    this.expo = config.input_expo ?? 1.4;
  }

  reset(now = Date.now()) {
    return this.limiter.reset(now);
  }

  update({ now = Date.now(), result = {}, telemetry = {}, connected = false } = {}) {
    let reason = null;
    if (!connected) reason = 'pixhawk_offline';
    else if (String(telemetry.flightMode || '').toUpperCase() !== 'MANUAL') reason = 'manual_required';
    else if (telemetry.armed !== true) reason = 'armed_required';
    else if (telemetry.pilotSteering?.ready !== true || ![0, 1, 2, 3].includes(telemetry.pilotSteering.type)) {
      reason = 'pilot_steering_not_ready';
    } else if (result.safe !== true) reason = result.reason || 'follow_unsafe';
    const rawThrottle = result.throttlePercent;
    const rawSteering = result.steeringDegrees;
    if (!Number.isFinite(rawThrottle) || !Number.isFinite(rawSteering)) reason ||= 'invalid_propulsion_target';
    if (reason || result.reason === 'holding_distance') {
      return { ...this.reset(now), safe: !reason, reason: reason || 'neutral' };
    }
    const throttle = Math.max(-this.limiter.throttleLimit, Math.min(this.limiter.throttleLimit, rawThrottle));
    const steering = Math.max(-this.limiter.steeringLimit, Math.min(this.limiter.steeringLimit, rawSteering));
    this.limiter.setTarget({
      throttle: shapeRoverAxis(throttle, { limit: this.limiter.throttleLimit, deadband: this.throttleDeadband, expo: this.expo }),
      steering: shapeRoverAxis(steering, { limit: this.limiter.steeringLimit, deadband: this.steeringDeadband, expo: this.expo }),
      rawThrottle: throttle, rawSteering: steering
    });
    return { ...this.limiter.step(now), safe: true, reason: 'tracking', propulsion: 'dual_thrust' };
  }
}
