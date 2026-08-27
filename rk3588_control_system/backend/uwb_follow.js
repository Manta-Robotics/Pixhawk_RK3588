const DEG_TO_RAD = Math.PI / 180;

function finite(value, fallback = null) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function clamp(value, minimum, maximum) {
  return Math.min(maximum, Math.max(minimum, value));
}

export function normalizeAngleDeg(value) {
  return ((finite(value, 0) + 180) % 360 + 360) % 360 - 180;
}

function magnitude(vector) {
  return Math.hypot(vector.x, vector.y);
}

function unit(vector, fallback = { x: 1, y: 0 }) {
  const length = magnitude(vector);
  return length > 1e-6
    ? { x: vector.x / length, y: vector.y / length }
    : { ...fallback };
}

function zero(reason, extra = {}) {
  return {
    safe: false,
    reason,
    velocityNed: { x: 0, y: 0 },
    desiredSpeedMps: 0,
    ...extra
  };
}

export class UwbFollowController {
  constructor(config = {}) {
    this.config = {
      followingDistanceM: Math.max(1.05, finite(config.following_distance_m, 1.5)),
      safetyDistanceM: Math.max(0.1, finite(config.safety_distance_m, 1.0)),
      safetyResumeDistanceM: Math.max(0.2, finite(config.safety_resume_distance_m, 1.2)),
      maxSpeedMps: clamp(finite(config.max_speed_mps, 5.0), 0.1, 5.0),
      commissioningSpeedMps: clamp(finite(config.commissioning_speed_mps, 0.8), 0.1, 5.0),
      commissioningMode: config.commissioning_mode !== false,
      positionGain: Math.max(0.05, finite(config.position_gain, 0.8)),
      velocityAlpha: clamp(finite(config.velocity_alpha, 0.25), 0.01, 1.0),
      directionAlpha: clamp(finite(config.direction_alpha, 0.2), 0.01, 1.0),
      minimumDirectionSpeedMps: Math.max(0.02, finite(config.minimum_direction_speed_mps, 0.15)),
      uwbTimeoutMs: Math.max(100, finite(config.uwb_timeout_ms, 700)),
      navigationTimeoutMs: Math.max(100, finite(config.navigation_timeout_ms, 700)),
      minimumGpsFixType: Math.max(2, Math.round(finite(config.minimum_gps_fix_type, 2))),
      bearingSign: finite(config.bearing_sign, -1) < 0 ? -1 : 1,
      bearingScale: Math.max(0.01, finite(config.bearing_scale, 0.8148)),
      bearingOffsetDeg: finite(config.bearing_offset_deg, -4.816)
    };
    this.reset();
  }

  reset() {
    this.previousRelativeNed = null;
    this.previousAt = null;
    this.targetVelocityNed = { x: 0, y: 0 };
    this.targetDirectionNed = null;
    this.safetyLatched = false;
  }

  setBearingCalibration(config = {}) {
    this.config.bearingSign = finite(config.bearing_sign, this.config.bearingSign) < 0 ? -1 : 1;
    this.config.bearingScale = Math.max(0.01, finite(config.bearing_scale, this.config.bearingScale));
    this.config.bearingOffsetDeg = finite(config.bearing_offset_deg, this.config.bearingOffsetDeg);
    this.reset();
    return {
      bearingSign: this.config.bearingSign,
      bearingScale: this.config.bearingScale,
      bearingOffsetDeg: this.config.bearingOffsetDeg
    };
  }

  preflight({ now = Date.now(), telemetry = {}, connected = false } = {}) {
    const gps = telemetry.gps || {};
    const uwb = telemetry.uwb || {};
    const attitude = telemetry.attitude || {};
    const boatVelocity = telemetry.velocity || {};
    const position = telemetry.position || {};
    const ekf = telemetry.ekf || {};
    const distanceM = finite(uwb.distanceM);
    const azimuthDeg = finite(uwb.azimuthDeg);
    const yawDeg = finite(attitude.yaw);
    const boatVx = finite(boatVelocity.vx);
    const boatVy = finite(boatVelocity.vy);
    const uwbUpdatedAt = finite(uwb.updatedAt);
    const navigationUpdatedAt = finite(position.updatedAt);
    const ekfUpdatedAt = finite(ekf.updatedAt);

    if (!connected) return zero('pixhawk_offline');
    if (finite(gps.fixType, 0) < this.config.minimumGpsFixType) return zero('gps_fix_invalid');
    if (ekf.healthy !== true || ekfUpdatedAt === null || now - ekfUpdatedAt > this.config.navigationTimeoutMs) {
      return zero('ekf_unhealthy');
    }
    if (navigationUpdatedAt === null || now - navigationUpdatedAt > this.config.navigationTimeoutMs) return zero('navigation_stale');
    if (uwb.online !== true || uwb.fresh !== true || distanceM === null || azimuthDeg === null || uwbUpdatedAt === null || now - uwbUpdatedAt > this.config.uwbTimeoutMs) {
      return zero('uwb_stale');
    }
    if (yawDeg === null || boatVx === null || boatVy === null) return zero('navigation_invalid');
    if (distanceM <= this.config.safetyDistanceM) {
      this.safetyLatched = true;
      return zero('safety_distance', { distanceM, safetyLatched: true });
    }

    return {
      safe: true,
      reason: 'preflight_ok',
      distanceM,
      azimuthDeg,
      yawDeg,
      boatVx,
      boatVy
    };
  }

  update({ now = Date.now(), telemetry = {}, connected = false } = {}) {
    const preflight = this.preflight({ now, telemetry, connected });
    if (!preflight.safe) return preflight;
    if (String(telemetry.flightMode || '').toUpperCase() !== 'GUIDED') return zero('guided_required');
    if (telemetry.armed !== true) return zero('armed_required');
    const { distanceM, azimuthDeg, yawDeg, boatVx, boatVy } = preflight;

    if (distanceM <= this.config.safetyDistanceM) this.safetyLatched = true;
    if (this.safetyLatched && distanceM < this.config.safetyResumeDistanceM) {
      return zero('safety_distance', { distanceM, safetyLatched: true });
    }
    this.safetyLatched = false;

    const bearingDeg = normalizeAngleDeg(
      azimuthDeg * this.config.bearingSign * this.config.bearingScale
        + this.config.bearingOffsetDeg
    );
    const absoluteBearingRad = (yawDeg + bearingDeg) * DEG_TO_RAD;
    const relativeNed = {
      x: distanceM * Math.cos(absoluteBearingRad),
      y: distanceM * Math.sin(absoluteBearingRad)
    };

    if (this.previousRelativeNed && this.previousAt !== null) {
      const dt = (now - this.previousAt) / 1000;
      if (dt >= 0.03 && dt <= 0.6) {
        const measuredTargetVelocity = {
          x: (relativeNed.x - this.previousRelativeNed.x) / dt + boatVx,
          y: (relativeNed.y - this.previousRelativeNed.y) / dt + boatVy
        };
        const alpha = this.config.velocityAlpha;
        this.targetVelocityNed = {
          x: this.targetVelocityNed.x * (1 - alpha) + measuredTargetVelocity.x * alpha,
          y: this.targetVelocityNed.y * (1 - alpha) + measuredTargetVelocity.y * alpha
        };
      } else if (dt > 0.6) {
        this.targetVelocityNed = { x: 0, y: 0 };
      }
    }
    this.previousRelativeNed = relativeNed;
    this.previousAt = now;

    const targetSpeedMps = magnitude(this.targetVelocityNed);
    if (targetSpeedMps >= this.config.minimumDirectionSpeedMps) {
      const measuredDirection = unit(this.targetVelocityNed);
      if (!this.targetDirectionNed) {
        this.targetDirectionNed = measuredDirection;
      } else {
        const alpha = this.config.directionAlpha;
        this.targetDirectionNed = unit({
          x: this.targetDirectionNed.x * (1 - alpha) + measuredDirection.x * alpha,
          y: this.targetDirectionNed.y * (1 - alpha) + measuredDirection.y * alpha
        });
      }
    }

    // Before enough target motion is observed, line-of-sight is the safest
    // temporary definition of "behind" and converges once motion is detected.
    const targetDirection = this.targetDirectionNed || unit(relativeNed);
    const trailingPointRelativeNed = {
      x: relativeNed.x - targetDirection.x * this.config.followingDistanceM,
      y: relativeNed.y - targetDirection.y * this.config.followingDistanceM
    };
    const desiredVelocity = {
      x: this.targetVelocityNed.x + this.config.positionGain * trailingPointRelativeNed.x,
      y: this.targetVelocityNed.y + this.config.positionGain * trailingPointRelativeNed.y
    };
    const requestedSpeed = magnitude(desiredVelocity);
    const effectiveSpeedLimit = Math.min(
      this.config.maxSpeedMps,
      this.config.commissioningMode ? this.config.commissioningSpeedMps : this.config.maxSpeedMps
    );
    const scale = requestedSpeed > effectiveSpeedLimit ? effectiveSpeedLimit / requestedSpeed : 1;
    const velocityNed = { x: desiredVelocity.x * scale, y: desiredVelocity.y * scale };

    return {
      safe: true,
      reason: 'tracking',
      distanceM,
      bearingDeg,
      targetSpeedMps,
      targetVelocityNed: { ...this.targetVelocityNed },
      targetDirectionNed: { ...targetDirection },
      trailingPointRelativeNed,
      velocityNed,
      desiredSpeedMps: magnitude(velocityNed),
      speedLimitMps: effectiveSpeedLimit,
      commissioningMode: this.config.commissioningMode,
      safetyLatched: false
    };
  }
}
