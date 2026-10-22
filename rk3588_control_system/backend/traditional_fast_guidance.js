const DEG_TO_RAD = Math.PI / 180;

function finite(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function clamp(value, minimum, maximum) {
  return Math.min(maximum, Math.max(minimum, value));
}

function normalizeAngleDeg(value) {
  return ((finite(value) + 180) % 360 + 360) % 360 - 180;
}

/**
 * Real-boat adapter for the Traditional Fast LOS outer loop.
 *
 * The source berthing controller returns forward speed and yaw rate. This
 * adapter deliberately keeps that interface and does not import the WAM-V
 * force/propeller model, ROS topics, or terminal NMPC actuator output.
 */
export class TraditionalFastGuidance {
  constructor(config = {}) {
    this.headingGain = Math.max(0, finite(config.heading_gain, 1.2));
    this.yawRateDamping = Math.max(0, finite(config.yaw_rate_damping, 0.35));
    this.minimumTurnSpeedScale = clamp(finite(config.minimum_turn_speed_scale, 0.15), 0, 1);
  }

  command({
    headingErrorDeg = 0,
    measuredYawRateDegS = 0,
    forwardSpeedMps = 0,
    maximumYawRateDegS = 35
  } = {}) {
    const headingError = normalizeAngleDeg(headingErrorDeg);
    const headingMagnitudeRad = Math.min(Math.abs(headingError) * DEG_TO_RAD, Math.PI / 2);
    const speedScale = Math.max(this.minimumTurnSpeedScale, Math.cos(headingMagnitudeRad));
    const maximumYawRate = Math.max(0, finite(maximumYawRateDegS, 35));
    const rawYawRateDegS = this.headingGain * headingError
      - this.yawRateDamping * finite(measuredYawRateDegS, 0);

    return {
      headingErrorDeg: headingError,
      measuredYawRateDegS: finite(measuredYawRateDegS, 0),
      speedScale,
      forwardSpeedMps: finite(forwardSpeedMps, 0) * speedScale,
      rawYawRateDegS,
      desiredYawRateDegS: clamp(rawYawRateDegS, -maximumYawRate, maximumYawRate)
    };
  }
}
