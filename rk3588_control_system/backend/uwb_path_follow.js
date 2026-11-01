// GPS-assisted breadcrumb following. Coordinates are local NED metres:
// x north, y east; bearing is calibrated body-forward/right, in degrees.
// This module builds a causal cubic B-spline and returns references only.
// It never sends MAVLink commands or treats a predicted point as an observation.
import { uwbObservationQuarantined } from './uwb_quality.js';

const RAD = Math.PI / 180;
const EARTH_M = 6378137;
const finite = value => typeof value === 'number' && Number.isFinite(value);
const clamp = (value, low, high) => Math.min(high, Math.max(low, value));
const wrap = value => ((value + 180) % 360 + 360) % 360 - 180;
const distance = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
const fresh = (at, now, age) => finite(at) && at > 0 && at <= now && now - at <= age;
const mix = (a, b, t) => ({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t });

export const PATH_FOLLOW_DEFAULTS = Object.freeze({
  enabled: false, gps_acquire_ms: 2000, history_s: 30, max_points: 256, min_points: 4,
  min_point_spacing_m: .2, min_path_length_m: .6, smoothing_time_s: .25,
  lookahead_min_m: .6, lookahead_max_m: 2, lookahead_time_s: 1,
  max_lateral_accel_mps2: .25, progress_gain: .6, gps_jump_tolerance_m: 1,
});

export const PATH_FOLLOW_BOUNDS = Object.freeze({
  gps_acquire_ms: [1000, 30000], history_s: [5, 120], max_points: [16, 1024], min_points: [3, 16],
  min_point_spacing_m: [.05, 2], min_path_length_m: [.1, 5], smoothing_time_s: [.02, 2],
  lookahead_min_m: [.2, 5], lookahead_max_m: [.2, 10], lookahead_time_s: [0, 5],
  max_lateral_accel_mps2: [.05, 2], progress_gain: [.05, 2], gps_jump_tolerance_m: [.2, 10],
});

export class UwbPathFollowController {
  constructor(config = {}) {
    this.c = config;
    const supplied = config.pathFollow || {};
    this.options = { ...PATH_FOLLOW_DEFAULTS, enabled: supplied.enabled === true };
    for (const [key, [low, high]] of Object.entries(PATH_FOLLOW_BOUNDS)) {
      this.options[key] = finite(supplied[key]) ? clamp(supplied[key], low, high) : PATH_FOLLOW_DEFAULTS[key];
    }
    this.options.max_points = Math.floor(this.options.max_points);
    this.options.min_points = Math.floor(this.options.min_points);
    this.options.lookahead_max_m = Math.max(this.options.lookahead_min_m, this.options.lookahead_max_m);
    this.segmentId = 0;
    this.reset();
  }

  reset() {
    this.goodSince = null;
    this.lastNow = null;
    this.origin = null;
    this.navigation = null;
    this.clearTrack();
  }

  clearTrack() {
    this.segmentId++;
    this.points = [];
    this.curve = [];
    this.lastObservation = null;
    this.filtered = null;
    this.targetSpeed = 0;
    this.progressKey = null;
    this.targetId = null;
    this.nextKey = 0;
  }

  inactive(reason) {
    return { active: false, reason, segmentId: this.segmentId, pointCount: this.points.length };
  }

  navigationQuality(now, t) {
    const limit = this.c.navigationTimeoutMs ?? 700;
    if (!finite(t.gps?.fixType) || t.gps.fixType < Math.max(3, this.c.minimumGpsFixType ?? 3)) return 'gps_fix_invalid';
    if (!fresh(t.gps?.updatedAt, now, limit)) return 'gps_stale';
    if (!t.ekf?.healthy || !fresh(t.ekf.updatedAt, now, limit)) return 'ekf_unavailable';
    if (!fresh(t.position?.updatedAt, now, limit) || !finite(t.position.lat) || !finite(t.position.lon)
      || Math.abs(t.position.lat) >= 90 || Math.abs(t.position.lon) > 180) return 'position_invalid';
    if (!fresh(t.attitude?.updatedAt, now, limit) || !finite(t.attitude.yaw)
      || !finite(t.attitude.yawRateDegS)) return 'imu_invalid';
    if (!finite(t.velocity?.vx) || !finite(t.velocity?.vy)
      || Math.hypot(t.velocity.vx, t.velocity.vy) > (this.c.gpsSpeedClosedLoop?.maximumObservedSpeedMps ?? 5)) return 'velocity_invalid';
    // Older telemetry producers omit this timestamp. Validate it whenever
    // supplied, since stale velocity also corrupts observation alignment.
    if (t.velocity.updatedAt !== undefined && !fresh(t.velocity.updatedAt, now, limit)) return 'velocity_stale';
    return null;
  }

  localPosition(position) {
    return { x: (position.lat - this.origin.lat) * RAD * EARTH_M,
      y: wrap(position.lon - this.origin.lon) * RAD * EARTH_M * Math.cos(this.origin.lat * RAD) };
  }

  // Only the last three spline spans can change when a new control point is
  // appended. Endpoint replication avoids extrapolating past the newest point.
  rebuildCurve() {
    const points = this.points;
    if (points.length < 2) { this.curve = []; return; }
    const padded = [points[0], points[0], ...points, points.at(-1), points.at(-1)];
    const curve = [];
    for (let i = 0; i < padded.length - 3; i++) {
      for (let j = 0; j <= 8; j++) {
        const u = j / 8;
        const weights = [(1 - u) ** 3, 3 * u ** 3 - 6 * u * u + 4,
          -3 * u ** 3 + 3 * u * u + 3 * u + 1, u ** 3];
        const point = { x: 0, y: 0, key: points[0].key - 2 + i + u, s: 0 };
        for (let k = 0; k < 4; k++) {
          point.x += padded[i + k].x * weights[k] / 6;
          point.y += padded[i + k].y * weights[k] / 6;
        }
        const previous = curve.at(-1);
        if (previous && distance(previous, point) < 1e-6) continue;
        point.s = previous ? previous.s + distance(previous, point) : 0;
        curve.push(point);
      }
    }
    this.curve = curve;
  }

  pointAt(s) {
    const curve = this.curve;
    for (let i = 1; i < curve.length; i++) {
      const a = curve[i - 1], b = curve[i];
      if (b.s >= s) {
        const fraction = clamp((s - a.s) / (b.s - a.s), 0, 1);
        return { ...mix(a, b, fraction), key: a.key + fraction * (b.key - a.key), s };
      }
    }
    return { ...curve.at(-1) };
  }

  locate(boat, speed, dt) {
    const curve = this.curve;
    let start = curve[0].s;
    if (this.progressKey !== null) {
      for (let i = 1; i < curve.length; i++) {
        const a = curve[i - 1], b = curve[i];
        if (b.key >= this.progressKey) {
          start = a.s + clamp((this.progressKey - a.key) / (b.key - a.key), 0, 1) * (b.s - a.s);
          break;
        }
        start = b.s;
      }
    }
    // Search only near the previous progress. A global nearest-point search
    // could jump to a later lap where a swimmer's route crosses itself.
    // Reprocessing the same control timestamp must not creep along the route.
    const searchM = this.progressKey !== null && dt <= 0 ? 0
      : Math.max(this.options.lookahead_min_m, 2 * speed * dt + .2);
    const end = start + searchM;
    let best = this.pointAt(start), error = distance(best, boat);
    for (let i = 1; i < curve.length; i++) {
      const a = curve[i - 1], b = curve[i], span = b.s - a.s;
      if (b.s < start || a.s > end) continue;
      const dx = b.x - a.x, dy = b.y - a.y;
      const fraction = clamp(((boat.x - a.x) * dx + (boat.y - a.y) * dy) / (span * span),
        Math.max(0, (start - a.s) / span), Math.min(1, (end - a.s) / span));
      const point = { ...mix(a, b, fraction), key: a.key + fraction * (b.key - a.key), s: a.s + fraction * span };
      const candidateError = distance(point, boat);
      if (candidateError < error - 1e-8) { best = point; error = candidateError; }
    }
    this.progressKey = Math.max(this.progressKey ?? best.key, best.key);
    return { ...best, error };
  }

  update({ now, telemetry: t = {}, distanceM, bearingDeg, predicted = false } = {}) {
    const o = this.options, c = this.c;
    if (!o.enabled) return this.inactive('disabled');
    if (!finite(now) || (this.lastNow !== null && (now < this.lastNow
      || now - this.lastNow > (c.navigationTimeoutMs ?? 700)))) {
      this.reset(); return this.inactive('clock_gap');
    }
    const dt = this.lastNow === null ? .1 : clamp((now - this.lastNow) / 1000, 0, .25);
    this.lastNow = now;
    const fault = this.navigationQuality(now, t);
    if (fault) { this.reset(); return this.inactive(fault); }
    if (!this.origin) this.origin = { lat: t.position.lat, lon: t.position.lon };
    const position = this.localPosition(t.position), at = t.position.updatedAt;
    const speed = Math.hypot(t.velocity.vx, t.velocity.vy);
    if (this.navigation) {
      const gap = (at - this.navigation.at) / 1000;
      if (gap < 0 || (gap === 0 && distance(position, this.navigation) > 1e-5)
        || distance(position, this.navigation) > o.gps_jump_tolerance_m + Math.max(speed, this.navigation.speed) * Math.max(0, gap)) {
        this.reset(); return this.inactive('gps_position_jump');
      }
    }
    this.navigation = { ...position, at, speed };
    this.goodSince ??= now;

    const u = t.uwb || {}, observedAt = u.measurementUpdatedAt;
    const selected = (u.sensors || []).filter(s => !u.activeReceiverIds?.length || u.activeReceiverIds.includes(s.receiverId));
    if (predicted || !u.online || !u.fresh || !fresh(observedAt, now, c.uwbTimeoutMs ?? 450)
      || !finite(distanceM) || distanceM <= 0 || !finite(bearingDeg)
      || Math.abs(bearingDeg) > (c.fieldOfViewHalfDeg ?? 90)
      || selected.some(s => s.valid === false || uwbObservationQuarantined(s))
      || (u.controlSource !== 'raw' && u.measurementStatus && u.measurementStatus !== 'trusted')) {
      this.clearTrack(); return this.inactive('observation_unavailable');
    }
    const ids = [...new Set(selected.map(s => s.tagId).filter(id => id !== null && id !== undefined))].sort();
    const id = u.tagId !== null && u.tagId !== undefined ? String(u.tagId) : ids.length ? ids.join(',') : null;
    if (this.targetId !== null && id !== this.targetId) {
      this.clearTrack(); return this.inactive('target_identity_changed');
    }
    this.targetId = id;
    if (this.lastObservation && (observedAt < this.lastObservation.at
      || observedAt - this.lastObservation.at > (c.uwbTimeoutMs ?? 450))) {
      this.clearTrack(); return this.inactive('observation_clock_gap');
    }

    if (!this.lastObservation || observedAt > this.lastObservation.at) {
      // Bridge timestamps are host observation clocks, not synchronized radio
      // acquisition times. Bound the constant-velocity/yaw-rate compensation.
      const positionDt = (observedAt - at) / 1000;
      const attitudeDt = (observedAt - t.attitude.updatedAt) / 1000;
      if (Math.max(Math.abs(positionDt), Math.abs(attitudeDt)) > (c.navigationTimeoutMs ?? 700) / 1000) {
        this.clearTrack(); return this.inactive('pose_alignment_gap');
      }
      const heading = (t.attitude.yaw + t.attitude.yawRateDegS * attitudeDt + bearingDeg) * RAD;
      const target = { x: position.x + t.velocity.vx * positionDt + distanceM * Math.cos(heading),
        y: position.y + t.velocity.vy * positionDt + distanceM * Math.sin(heading), at: observedAt };
      const previous = this.lastObservation;
      const sampleDt = previous ? (observedAt - previous.at) / 1000 : 0;
      const maximumTargetSpeed = c.prediction?.max_target_speed_mps ?? 3;
      if (previous && distance(previous, target) > (c.prediction?.innovation_gate_m ?? .6) + maximumTargetSpeed * sampleDt) {
        this.clearTrack(); return this.inactive('target_position_jump');
      }
      const alpha = previous ? 1 - Math.exp(-sampleDt / o.smoothing_time_s) : 1;
      const filtered = this.filtered ? mix(this.filtered, target, alpha) : target;
      if (this.filtered && sampleDt > 0) {
        const measuredSpeed = Math.min(maximumTargetSpeed, distance(this.filtered, filtered) / sampleDt);
        this.targetSpeed += (1 - Math.exp(-sampleDt / .7)) * (measuredSpeed - this.targetSpeed);
      }
      this.filtered = filtered;
      this.lastObservation = target;
      if (!this.points.length || distance(this.points.at(-1), filtered) >= o.min_point_spacing_m) {
        this.points.push({ ...filtered, at: observedAt, key: this.nextKey++ });
        while (this.points.length > o.min_points && (this.points.length > o.max_points
          || observedAt - this.points[0].at > o.history_s * 1000)) {
          // Never silently discard an untraversed turn and connect across it.
          if (this.progressKey !== null && this.points[0].key + 1 >= this.progressKey) {
            this.clearTrack(); return this.inactive('path_buffer_exhausted');
          }
          this.points.shift();
        }
        this.rebuildCurve();
      }
    }
    if (now - this.goodSince < o.gps_acquire_ms) return this.inactive('gps_confirming');
    if (this.points.length < o.min_points || !this.curve.length
      || this.curve.at(-1).s < o.min_path_length_m) return this.inactive('path_warming_up');

    const poseAge = (now - at) / 1000;
    const boat = { x: position.x + t.velocity.vx * poseAge, y: position.y + t.velocity.vy * poseAge };
    const yaw = (t.attitude.yaw + t.attitude.yawRateDegS * (now - t.attitude.updatedAt) / 1000) * RAD;
    const progress = this.locate(boat, speed, dt);
    const lookaheadM = clamp(o.lookahead_min_m + speed * o.lookahead_time_s, o.lookahead_min_m, o.lookahead_max_m);
    const pathLengthM = this.curve.at(-1).s;
    const followingDistanceM = finite(c.followingDistanceM) ? Math.max(0, c.followingDistanceM) : 0;
    const signedRemainingM = pathLengthM - progress.s - followingDistanceM;
    const remainingM = Math.max(0, signedRemainingM);
    // Subtract the following distance before clamping. Otherwise lateral
    // error can demand forward travel even after the stopping boundary.
    // Retain joining distance when the boat is still behind a short path.
    const travelM = Math.max(0, signedRemainingM + progress.error);
    const holding = travelM <= 1e-6;
    // Do not steer past the stopping boundary or ask for reverse progress
    // when the boat has already reached/passed that boundary.
    const goalS = Math.max(progress.s, Math.min(pathLengthM - followingDistanceM, progress.s + lookaheadM));
    const goal = this.pointAt(goalS);
    const dx = goal.x - boat.x, dy = goal.y - boat.y;
    const forward = dx * Math.cos(yaw) + dy * Math.sin(yaw);
    const right = -dx * Math.sin(yaw) + dy * Math.cos(yaw);
    const goalDistance = Math.hypot(dx, dy);
    const headingErrorDeg = holding || goalDistance < .02 ? 0 : Math.atan2(right, forward) / RAD;
    if (!holding && forward < -.05) return this.inactive('path_rejoin_required');
    const curvatureMInv = holding ? 0 : clamp(2 * right / Math.max(.01, goalDistance ** 2),
      -(c.maxCurvatureMInv ?? .55), c.maxCurvatureMInv ?? .55);
    const deceleration = c.prediction?.braking_deceleration_mps2 ?? .3;
    const brakingCap = Math.sqrt((.6 * deceleration) ** 2 + 2 * deceleration * travelM) - .6 * deceleration;
    const curvature = Math.max(1e-6, Math.abs(curvatureMInv));
    const maximumSpeed = Math.min(c.maxSpeedMps ?? 2, c.commissioningMode ? c.commissioningSpeedMps ?? 2 : Infinity);
    const desiredSpeedMps = holding ? 0 : Math.max(0, Math.min(maximumSpeed, this.targetSpeed + o.progress_gain * travelM,
      brakingCap, Math.sqrt(o.max_lateral_accel_mps2 / curvature), (c.maxYawRateDegS ?? 14) * RAD / curvature));
    const yawRateDegS = holding ? 0 : clamp(desiredSpeedMps * curvatureMInv / RAD
      - (c.traditionalYawRateDamping ?? .6) * t.attitude.yawRateDegS, -(c.maxYawRateDegS ?? 14), c.maxYawRateDegS ?? 14);
    return { active: true, reason: 'gps_path_follow', segmentId: this.segmentId, pointCount: this.points.length,
      desiredSpeedMps, headingErrorDeg, yawRateDegS, curvatureMInv, lookaheadM,
      pathLengthM, remainingM, progressM: progress.s, progressKey: this.progressKey,
      crossTrackErrorM: progress.error, targetSpeedMps: this.targetSpeed, lastObservationAt: this.lastObservation.at,
      origin: { ...this.origin }, boatPositionNedM: boat, lookaheadPointNedM: { x: goal.x, y: goal.y } };
  }
}
