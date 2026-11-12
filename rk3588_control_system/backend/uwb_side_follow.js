// Side positions are defined in the target's ground-track frame, never by
// adding 90 degrees to a pursuit steering command. No actuator is sent here.
import { UwbFollowController, normalizeAngleDeg } from './uwb_follow.js';
import { UwbGpsSpeedLoop } from './uwb_gps_speed.js';
import { uwbObservationQuarantined } from './uwb_quality.js';

const RAD = Math.PI / 180;
const finite = value => typeof value === 'number' && Number.isFinite(value);
const clamp = (value, low, high) => Math.max(low, Math.min(high, value));
export const FOLLOW_MODES = Object.freeze(['rear', 'left', 'right']);
export const SIDE_FOLLOW_DEFAULTS = Object.freeze({
  distance_m: 2, position_gain: 0.4, minimum_target_speed_mps: 0.2,
  motion_confirm_ms: 800, motion_window_ms: 1600, max_speed_mps: 0.8,
});
export const SIDE_FOLLOW_BOUNDS = Object.freeze({
  distance_m: [0.5, 20], position_gain: [0.05, 1], minimum_target_speed_mps: [0.1, 1],
  motion_confirm_ms: [400, 3000], motion_window_ms: [800, 5000], max_speed_mps: [0.1, 2],
});

export class UwbSideFollowController extends UwbFollowController {
  constructor(config = {}) {
    super({ ...config, execution_mode: 'relative_imu', minimum_gps_fix_type: 3 });
    this.options = { ...SIDE_FOLLOW_DEFAULTS, ...config.side_follow };
    this.config.sideFollow = this.options;
    this.receiverFovHalfDeg = config.side_receiver_fov_half_deg ?? 45;
    this.followMode = 'left';
    this.speedLoop = new UwbGpsSpeedLoop(this.config);
    this.reset();
  }

  reset() {
    super.reset();
    this.speedLoop?.reset();
    this.history = [];
    this.integratedBoat = { x: 0, y: 0 };
    this.lastSample = null;
    this.updatedAt = null;
    this.targetHeadingDeg = null;
    this.targetVelocity = { x: 0, y: 0 };
    this.targetKey = null;
    this.appliedThrottle = 0;
  }

  setMode(mode, distanceM = this.options.distance_m) {
    if (!['left', 'right'].includes(mode)) throw new Error('invalid_side_follow_mode');
    const [low, high] = SIDE_FOLLOW_BOUNDS.distance_m;
    if (!finite(distanceM) || distanceM < low || distanceM > high
      || distanceM <= this.config.safetyDistanceM + 0.2) throw new Error('invalid_side_distance');
    this.followMode = mode;
    this.options.distance_m = distanceM;
    this.reset();
  }

  preflight(input = {}) {
    const base = super.preflight({ ...input, targetEstimate: null });
    const result = { ...base, followMode: this.followMode, sideDistanceM: this.options.distance_m,
      controlMode: 'relative_manual', requiredFlightMode: 'MANUAL', relativeImuControl: true };
    if (!base.safe) return result;
    const now = input.now ?? Date.now(), t = input.telemetry || {};
    const velocityAt = t.velocity?.updatedAt;
    const reason = this.receiverFovHalfDeg < 90 ? 'side_receiver_coverage_required'
      : this.speedLoop.quality(now, t)
        || (!finite(velocityAt) || velocityAt > now || now - velocityAt > this.config.navigationTimeoutMs ? 'side_navigation_stale' : null);
    return { ...result, safe: !reason, reason: reason || 'preflight_ok',
      navigationAvailable: !reason, navigationReason: reason };
  }

  neutral(reason, extra = {}, safe = true) {
    if (!safe) this.reset();
    this.appliedThrottle = 0;
    if (reason !== 'side_gps_confirming') this.speedLoop.reset();
    return { ...extra, safe, status: extra.status || (safe ? 'waiting_target' : 'stopped'), reason,
      followMode: this.followMode, sideDistanceM: this.options.distance_m,
      trackingMethod: 'traditional', controlMode: 'relative_manual', requiredFlightMode: 'MANUAL',
      relativeImuControl: true, commandFrame: 'BODY_RELATIVE', pausePropulsion: true,
      throttlePercent: 0, steeringDegrees: 0, forwardSpeedMps: 0, desiredSpeedMps: 0,
      yawRateDegS: 0, yawRateRadS: 0, velocityNed: { x: 0, y: 0 },
      targetHeadingDeg: this.targetHeadingDeg };
  }

  observe(t, relative) {
    const at = t.uwb.measurementUpdatedAt;
    let previous = this.lastSample;
    if (at === previous?.at) return;
    if (previous && (at < previous.at || at - previous.at > this.config.uwbTimeoutMs)) { this.reset(); previous = null; }
    const velocity = { x: t.velocity.vx, y: t.velocity.vy };
    if (previous) {
      const dt = (at - previous.at) / 1000;
      this.integratedBoat.x += (velocity.x + previous.velocity.x) * 0.5 * dt;
      this.integratedBoat.y += (velocity.y + previous.velocity.y) * 0.5 * dt;
    }
    this.lastSample = { at, relative, velocity };
    this.history.push({ at, x: relative.x + this.integratedBoat.x, y: relative.y + this.integratedBoat.y });
    this.history = this.history.filter(point => at - point.at <= this.options.motion_window_ms);
    const points = this.history;
    if (points.length < 4 || at - points[0].at < this.options.motion_confirm_ms) return;
    const mean = points.reduce((sum, p) => ({ x: sum.x + p.x / points.length,
      y: sum.y + p.y / points.length, time: sum.time + (p.at - at) / 1000 / points.length }), { x: 0, y: 0, time: 0 });
    let denominator = 0, vx = 0, vy = 0;
    for (const p of points) {
      const dt = (p.at - at) / 1000 - mean.time;
      denominator += dt * dt; vx += dt * (p.x - mean.x); vy += dt * (p.y - mean.y);
    }
    const estimate = { x: vx / denominator, y: vy / denominator };
    const speed = Math.hypot(estimate.x, estimate.y);
    this.targetVelocity = speed >= this.options.minimum_target_speed_mps ? estimate : { x: 0, y: 0 };
    if (speed >= this.options.minimum_target_speed_mps) {
      const heading = Math.atan2(estimate.y, estimate.x) / RAD;
      const delta = previous ? 90 * (at - previous.at) / 1000 : 180;
      this.targetHeadingDeg = this.targetHeadingDeg === null ? heading
        : normalizeAngleDeg(this.targetHeadingDeg + clamp(normalizeAngleDeg(heading - this.targetHeadingDeg), -delta, delta));
    }
  }

  trackApplied(throttle) {
    this.appliedThrottle = throttle;
    this.speedLoop.trackApplied(throttle, this.stepS ?? 0.1);
  }

  update({ now = Date.now(), telemetry: t = {}, connected = false } = {}) {
    const preflight = this.preflight({ now, telemetry: t, connected });
    if (!preflight.safe) return this.neutral(preflight.reason, preflight, false);
    this.stepS = this.updatedAt === null ? 0.1 : clamp((now - this.updatedAt) / 1000, 0, 0.25);
    this.updatedAt = now;
    const u = t.uwb;
    const selected = (u.sensors || []).filter(sensor => !u.activeReceiverIds?.length || u.activeReceiverIds.includes(sensor.receiverId));
    if (['suspect', 'quarantined', 'recovering'].includes(u.measurementStatus)
      || selected.some(sensor => sensor.valid === false || uwbObservationQuarantined(sensor)))
      return this.neutral('side_measurement_untrusted');
    const key = JSON.stringify([u.streamId ?? null, selected.map(s => [s.receiverId, s.tagId])]);
    if (this.targetKey !== null && key !== this.targetKey) return this.neutral('target_identity_changed', {}, false);
    this.targetKey = key;
    const c = this.config, distanceM = preflight.distanceM;
    const bearingDeg = normalizeAngleDeg(preflight.azimuthDeg * c.bearingSign * c.bearingScale + c.bearingOffsetDeg);
    const relativeBodyM = { x: distanceM * Math.cos(bearingDeg * RAD), y: distanceM * Math.sin(bearingDeg * RAD) };
    const uncertaintyM = Math.max(c.prediction.measurement_noise_m ?? 0.12,
      ...selected.map(sensor => finite(sensor.filterUncertaintyM) ? sensor.filterUncertaintyM : 0));
    const base = { ...preflight, distanceM, bearingDeg, relativeBodyM, uncertaintyM };
    const rawNear = (u.sensors || []).some(sensor => finite(sensor.rawDistanceM) && sensor.rawDistanceM >= 0
      && sensor.rawDistanceM <= c.safetyDistanceM && finite(sensor.rawDistanceUpdatedAt)
      && sensor.rawDistanceUpdatedAt <= now && now - sensor.rawDistanceUpdatedAt <= c.uwbTimeoutMs);
    if (rawNear) return this.neutral('holding_distance', { ...base, status: 'holding', holdingDistanceM: c.safetyDistanceM });
    if (distanceM <= c.safetyDistanceM + uncertaintyM)
      return this.neutral('holding_distance', { ...base, status: 'holding', holdingDistanceM: c.safetyDistanceM + uncertaintyM });
    if (Math.abs(bearingDeg) > this.receiverFovHalfDeg) return this.neutral('side_target_outside_coverage', base);
    const ageS = (now - u.measurementUpdatedAt) / 1000;
    const acquisitionYaw = (t.attitude.yaw + t.attitude.yawRateDegS * (u.measurementUpdatedAt - t.attitude.updatedAt) / 1000) * RAD;
    const relative = { x: relativeBodyM.x * Math.cos(acquisitionYaw) - relativeBodyM.y * Math.sin(acquisitionYaw),
      y: relativeBodyM.x * Math.sin(acquisitionYaw) + relativeBodyM.y * Math.cos(acquisitionYaw) };
    this.observe(t, relative);
    if (this.targetHeadingDeg === null) return this.neutral('side_direction_unavailable', base);

    const heading = this.targetHeadingDeg * RAD, side = this.followMode === 'left' ? -1 : 1;
    const offset = { x: -Math.sin(heading) * side * this.options.distance_m,
      y: Math.cos(heading) * side * this.options.distance_m };
    const targetVelocity = this.targetVelocity;
    // Compensate observation age without changing its measurement timestamp.
    const r = { x: relative.x + (targetVelocity.x - t.velocity.vx) * ageS,
      y: relative.y + (targetVelocity.y - t.velocity.vy) * ageS };
    const error = { x: r.x + offset.x, y: r.y + offset.y };
    const v = { x: targetVelocity.x + this.options.position_gain * error.x,
      y: targetVelocity.y + this.options.position_gain * error.y };
    const desiredHeadingDeg = Math.atan2(v.y, v.x) / RAD;
    const headingErrorDeg = normalizeAngleDeg(desiredHeadingDeg - t.attitude.yaw);
    const speedLimitMps = Math.min(this.options.max_speed_mps, c.maxSpeedMps,
      c.commissioningMode ? c.commissioningSpeedMps : c.maxSpeedMps);
    const rangeSpeedLimit = Math.sqrt(2 * (c.prediction.braking_deceleration_mps2 ?? 0.3)
      * Math.max(0, distanceM - c.safetyDistanceM - uncertaintyM));
    const desiredSpeedMps = Math.min(speedLimitMps, rangeSpeedLimit, Math.hypot(v.x, v.y)) * Math.max(0, Math.cos(headingErrorDeg * RAD));
    const speed = this.speedLoop.update({ now, telemetry: t, bearingDeg, rangeRateMps: 0,
      desiredClosingMps: 0, appliedThrottle: this.appliedThrottle, policySpeedMps: desiredSpeedMps });
    if (!speed.active) return this.neutral(speed.reason === 'gps_confirming' ? 'side_gps_confirming' : speed.reason, base);
    const throttle = desiredSpeedMps < 0.01 ? 0 : speed.throttlePercent;
    const yawRateDegS = this.traditionalFastGuidance.command({ headingErrorDeg,
      measuredYawRateDegS: t.attitude.yawRateDegS, maximumYawRateDegS: c.maxYawRateDegS }).desiredYawRateDegS;
    const steering = yawRateDegS / c.maxYawRateDegS * c.relativeSteeringLimitDeg;
    return { ...base, safe: true, status: 'tracking', reason: 'side_tracking', pausePropulsion: false,
      trackingMethod: 'traditional', followMode: this.followMode, sideDistanceM: this.options.distance_m,
      commandFrame: 'BODY_RELATIVE', relativeImuControl: true, gpsSpeedControlActive: true,
      absoluteSpeedObservable: true, targetHeadingDeg: this.targetHeadingDeg, targetVelocityNedMps: { ...targetVelocity },
      desiredOffsetNedM: offset, positionErrorNedM: error, positionErrorM: Math.hypot(error.x, error.y),
      desiredHeadingDeg, headingErrorDeg, desiredSpeedMps, forwardSpeedMps: desiredSpeedMps,
      measuredForwardSpeedMps: speed.measuredForwardSpeedMps, speedLimitMps,
      throttlePercent: throttle, steeringDegrees: steering, yawRateDegS, yawRateRadS: yawRateDegS * RAD,
      velocityNed: { x: desiredSpeedMps * Math.cos(t.attitude.yaw * RAD), y: desiredSpeedMps * Math.sin(t.attitude.yaw * RAD) } };
  }
}
