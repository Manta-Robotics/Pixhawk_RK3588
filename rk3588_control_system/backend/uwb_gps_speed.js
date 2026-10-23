// Optional GPS surge-speed feedback. UWB/IMU continue to own bearing guidance,
// range protection and the GPS-free fallback. This class sends no commands.
const finite = value => typeof value === 'number' && Number.isFinite(value);
const clamp = (value, low, high) => Math.max(low, Math.min(high, value));

export class UwbGpsSpeedLoop {
  constructor(config) {
    this.c = config;
    this.options = config.gpsSpeedClosedLoop || {};
    this.reset();
  }

  reset() {
    this.goodSince = null;
    this.at = null;
    this.active = false;
    this.integral = 0;
    this.targetRadial = null;
    this.measurementAt = null;
    this.request = 0;
  }

  quality(now, telemetry) {
    const c = this.c, t = telemetry;
    const fresh = at => finite(at) && at <= now && now - at <= c.navigationTimeoutMs;
    if (!this.options.enabled) return 'disabled';
    if (!finite(t.gps?.fixType) || t.gps.fixType < c.minimumGpsFixType) return 'gps_fix_invalid';
    if (!fresh(t.gps?.updatedAt)) return 'gps_stale';
    if (t.ekf?.healthy !== true || !fresh(t.ekf.updatedAt)) return 'ekf_unavailable';
    if (!fresh(t.position?.updatedAt)) return 'velocity_stale';
    if (!fresh(t.attitude?.updatedAt) || !finite(t.attitude.yaw)) return 'imu_stale';
    if (!finite(t.velocity?.vx) || !finite(t.velocity?.vy)
        || Math.hypot(t.velocity.vx, t.velocity.vy) > this.options.maximumObservedSpeedMps) return 'velocity_invalid';
    return null;
  }

  update({ now, telemetry, bearingDeg, rangeRateMps, desiredClosingMps, appliedThrottle,
    policySpeedMps = null, stop = false }) {
    const reason = this.quality(now, telemetry);
    if (reason || (this.at !== null && now < this.at)) {
      this.reset();
      return { active: false, reason: reason || 'clock_reset' };
    }
    const c = this.c, o = this.options;
    const dt = this.at === null ? 0.1 : clamp((now - this.at) / 1000, 0.001, 0.25);
    this.at = now;
    if (this.goodSince === null) this.goodSince = now;
    const yaw = telemetry.attitude.yaw * Math.PI / 180;
    const bearing = bearingDeg * Math.PI / 180;
    const { vx, vy } = telemetry.velocity;
    const measured = vx * Math.cos(yaw) + vy * Math.sin(yaw);
    const at = telemetry.uwb?.measurementUpdatedAt;
    if (finite(at) && at !== this.measurementAt) {
      const target = vx * Math.cos(yaw + bearing) + vy * Math.sin(yaw + bearing) + rangeRateMps;
      const elapsed = this.measurementAt === null ? 0.1 : Math.max(0, (at - this.measurementAt) / 1000);
      const alpha = 1 - Math.exp(-Math.min(1, elapsed) / 1.0);
      this.targetRadial = this.targetRadial === null ? target : this.targetRadial + alpha * (target - this.targetRadial);
      this.measurementAt = at;
    }
    const limit = Math.min(c.maxSpeedMps, c.commissioningMode ? c.commissioningSpeedMps : c.maxSpeedMps);
    const desired = stop ? 0 : clamp(finite(policySpeedMps) ? policySpeedMps
      : ((this.targetRadial ?? 0) + desiredClosingMps) / Math.max(0.3, Math.cos(bearing)), 0, limit);
    const error = desired - measured;
    if (now - this.goodSince < o.acquireMs) return { active: false, reason: 'gps_confirming' };
    if (!this.active) {
      // Start at the applied thrust, then let the common actuator slew/braking
      // limits govern subsequent changes. No mode switch or arming is needed.
      this.integral = clamp(appliedThrottle - o.kp * error, -c.relativeThrottleLimitPercent, c.relativeThrottleLimitPercent);
      this.active = true;
    } else {
      const candidate = clamp(this.integral + o.ki * error * dt, -c.relativeThrottleLimitPercent, c.relativeThrottleLimitPercent);
      const output = o.kp * error + candidate;
      if (!((output > c.relativeThrottleLimitPercent && error > 0) || (output < 0 && error < 0))) this.integral = candidate;
    }
    this.request = clamp(o.kp * error + this.integral, 0, c.relativeThrottleLimitPercent);
    return { active: true, reason: 'gps_speed_feedback', throttlePercent: this.request,
      measuredForwardSpeedMps: measured, desiredSpeedMps: desired, speedErrorMps: error };
  }

  trackApplied(throttle, dt) {
    if (this.active) this.integral = clamp(this.integral + (throttle - this.request) * dt,
      -this.c.relativeThrottleLimitPercent, this.c.relativeThrottleLimitPercent);
  }
}
