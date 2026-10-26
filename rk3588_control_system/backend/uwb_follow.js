import { TraditionalFastGuidance } from './traditional_fast_guidance.js';
import { usableControlEstimate } from './uwb_target_prediction.js';
import { RelativeImuControl } from './uwb_relative_imu.js';
import { RELATIVE_NMPC_DEFAULTS } from './uwb_relative_nmpc.js';

const DEG_TO_RAD = Math.PI / 180;

function finite(value, fallback = null) {
  if (value === null || value === undefined || value === '') return fallback;
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function clamp(value, minimum, maximum) {
  return Math.min(maximum, Math.max(minimum, value));
}

function moveToward(current, target, maximumDelta) {
  if (target > current) return Math.min(target, current + maximumDelta);
  if (target < current) return Math.max(target, current - maximumDelta);
  return current;
}

export function normalizeAngleDeg(value) {
  return ((finite(value, 0) + 180) % 360 + 360) % 360 - 180;
}

function zero(reason, extra = {}) {
  const controlMode = extra.controlMode || 'auto';
  const guided = controlMode === 'guided_velocity';
  return {
    safe: false,
    reason,
    controlMode,
    requiredFlightMode: extra.requiredFlightMode || (guided ? 'GUIDED' : controlMode === 'relative_manual' ? 'MANUAL' : controlMode === 'relative_acro' ? 'ACRO' : null),
    navigationAvailable: false,
    commandFrame: guided ? 'BODY_NED' : 'BODY_RELATIVE',
    relativeBodyM: { x: 0, y: 0 },
    forwardSpeedMps: 0,
    yawRateDegS: 0,
    yawRateRadS: 0,
    throttlePercent: 0,
    steeringDegrees: 0,
    velocityNed: { x: 0, y: 0 },
    desiredSpeedMps: 0,
    ...extra
  };
}

export class UwbFollowController {
  constructor(config = {}) {
    this.config = {
      prediction: config.prediction || {},
      manualPropulsion: config.manual_propulsion === 'single_side',
      executionMode: ['acro','relative_imu'].includes(config.execution_mode) ? config.execution_mode : null,
      relativeNmpc: {
        ...RELATIVE_NMPC_DEFAULTS,
        ...config.relative_nmpc,
        enabled: config.relative_nmpc?.enabled === true
      },
      gpsSpeedClosedLoop: {
        enabled: config.gps_speed_closed_loop?.enabled === true,
        acquireMs: Math.max(0, finite(config.gps_speed_closed_loop?.acquire_ms, 2000)),
        kp: Math.max(0, finite(config.gps_speed_closed_loop?.kp, 20)),
        ki: Math.max(0, finite(config.gps_speed_closed_loop?.ki, 8)),
        maximumObservedSpeedMps: Math.max(0.1, finite(config.gps_speed_closed_loop?.maximum_observed_speed_mps, 5))
      },
      followingDistanceM: Math.max(0.1, finite(config.following_distance_m, 1.5)),
      controllerProfile: String(config.controller_profile || 'ilos_curvature').trim().toLowerCase() === 'traditional_fast'
        ? 'traditional_fast'
        : 'ilos_curvature',
      traditionalHeadingGain: Math.max(0, finite(config.traditional_heading_gain, 1.2)),
      traditionalYawRateDamping: Math.max(0, finite(config.traditional_yaw_rate_damping, 0.35)),
      traditionalMinimumTurnSpeedScale: clamp(
        finite(config.traditional_minimum_turn_speed_scale, 0.15),
        0,
        1
      ),
      safetyDistanceM: Math.max(0.1, finite(config.safety_distance_m, 1.0)),
      safetyResumeDistanceM: Math.max(0.2, finite(config.safety_resume_distance_m, 1.2)),
      hardStopEnabled: config.hard_stop_enabled !== false,
      hardStopResumeDistanceM: Math.max(0.1, finite(config.hard_stop_resume_distance_m, 1.08)),
      closeApproachEnabled: config.close_approach_enabled !== false,
      closeApproachMinSpeedMps: clamp(finite(config.close_approach_min_speed_mps, 0.12), 0.02, 1),
      closeApproachMaxSpeedMps: clamp(finite(config.close_approach_max_speed_mps, 0.3), 0.02, 1.5),
      closeApproachMinThrottlePercent: clamp(finite(config.close_approach_min_throttle_percent, 20), 1, 100),
      closeApproachMaxThrottlePercent: clamp(finite(config.close_approach_max_throttle_percent, 30), 1, 100),
      distanceHoldUpperM: Math.max(0.1, finite(config.distance_hold_upper_m, 0.55)),
      distanceRecoveryResumeM: Math.max(0.1, finite(config.distance_recovery_resume_m, 0.5)),
      maxSpeedMps: clamp(finite(config.max_speed_mps, 5.0), 0.1, 5.0),
      maxReverseSpeedMps: clamp(finite(config.max_reverse_speed_mps, 0.2), 0, 1.5),
      commissioningSpeedMps: clamp(finite(config.commissioning_speed_mps, 0.35), 0.1, 5.0),
      commissioningMode: config.commissioning_mode !== false,
      rangeGain: Math.max(0.01, finite(config.range_gain, finite(config.position_gain, 0.65))),
      rangeRateGain: Math.max(0, finite(config.range_rate_gain, 0.25)),
      rangeIntegralGain: Math.max(0, finite(config.range_integral_gain, 0.08)),
      rangeIntegralLimitMps: clamp(finite(config.range_integral_limit_mps, 0.25), 0, 2),
      targetSpeedAlpha: clamp(finite(config.target_speed_alpha, finite(config.velocity_alpha, 0.2)), 0.01, 1.0),
      bearingGain: Math.max(0.01, finite(config.bearing_gain, 1.2)),
      bearingRateGain: Math.max(0, finite(config.bearing_rate_gain, 0.18)),
      bearingRateAlpha: clamp(finite(config.bearing_rate_alpha, 0.3), 0.01, 1.0),
      bearingFilterAlpha: clamp(finite(config.bearing_filter_alpha, 0.2), 0.01, 1.0),
      bearingDeadbandDeg: clamp(finite(config.bearing_deadband_deg, 4), 0, 30),
      bearingReverseConfirmDeg: clamp(finite(config.bearing_reverse_confirm_deg, 8), 0, 60),
      bearingReverseConfirmMs: clamp(finite(config.bearing_reverse_confirm_ms, 300), 0, 2000),
      ilosIntegralGain: clamp(finite(config.ilos_integral_gain, 0.12), 0, 2),
      ilosIntegralDecay: clamp(finite(config.ilos_integral_decay, 0.25), 0, 5),
      ilosMaxCorrectionDeg: clamp(finite(config.ilos_max_correction_deg, 12), 0, 45),
      lookaheadBaseM: clamp(finite(config.lookahead_base_m, 1.2), 0.2, 10),
      lookaheadSpeedGainS: clamp(finite(config.lookahead_speed_gain_s, 0.8), 0, 5),
      lookaheadMinM: clamp(finite(config.lookahead_min_m, 1.0), 0.2, 10),
      lookaheadMaxM: clamp(finite(config.lookahead_max_m, 3.0), 0.3, 20),
      maxCurvatureMInv: clamp(finite(config.max_curvature_m_inv, 0.8), 0.05, 5),
      curvatureSpeedGain: clamp(finite(config.curvature_speed_gain, 1.5), 0, 10),
      minimumCurveSpeedScale: clamp(finite(config.minimum_curve_speed_scale, 0.3), 0.05, 1),
      sameDirectionSteeringRatio: clamp(finite(config.same_direction_steering_ratio, 0.45), 0.01, 2),
      bearingSlowdownDeg: clamp(finite(config.bearing_slowdown_deg, 20), 0, 170),
      bearingStopDeg: clamp(finite(config.bearing_stop_deg, 55), 1, 179),
      fieldOfViewHalfDeg: clamp(finite(config.field_of_view_half_deg, 45), 30, 90),
      maxYawRateDegS: clamp(finite(config.max_yaw_rate_deg_s, 35), 1, 120),
      relativeThrottleLimitPercent: clamp(finite(config.relative_throttle_limit_percent, 25), 1, 100),
      minimumEffectiveThrottlePercent: clamp(finite(config.relative_min_effective_throttle_percent, 0), 0, 100),
      minimumThrottleRangeErrorM: clamp(finite(config.minimum_throttle_range_error_m, 0.35), 0, 5),
      relativeSteeringLimitDeg: clamp(finite(config.relative_steering_limit_deg, 30), 1, 45),
      steeringSlewDegS: clamp(finite(config.steering_slew_deg_s, 18), 1, 180),
      steeringReturnDegS: clamp(finite(config.steering_return_deg_s, 30), 1, 180),
      throttleAccelPercentS: clamp(finite(config.relative_throttle_accel_percent_s, 12), 1, 100),
      throttleDecelPercentS: clamp(finite(config.relative_throttle_decel_percent_s, 25), 1, 100),
      turnDecelPercentS: clamp(finite(config.relative_turn_decel_percent_s, finite(config.relative_throttle_decel_percent_s, 25)), 1, 100),
      turnMinimumThrottlePercent: clamp(finite(config.relative_turn_min_throttle_percent, 0), 0, 100),
      rangeDeadbandM: clamp(finite(config.range_deadband_m, 0.12), 0, 2),
      uwbTimeoutMs: Math.max(100, finite(config.uwb_timeout_ms, 700)),
      navigationTimeoutMs: Math.max(100, finite(config.navigation_timeout_ms, 700)),
      minimumGpsFixType: Math.max(2, Math.round(finite(config.minimum_gps_fix_type, 2))),
      bearingSign: finite(config.bearing_sign, -1) < 0 ? -1 : 1,
      bearingScale: Math.max(0.01, finite(config.bearing_scale, 0.8148)),
      bearingOffsetDeg: finite(config.bearing_offset_deg, -4.816)
    };
    if (this.config.bearingStopDeg <= this.config.bearingSlowdownDeg) {
      this.config.bearingStopDeg = Math.min(179, this.config.bearingSlowdownDeg + 1);
    }
    this.config.bearingReverseConfirmDeg = Math.max(
      this.config.bearingDeadbandDeg,
      this.config.bearingReverseConfirmDeg
    );
    this.config.lookaheadMaxM = Math.max(this.config.lookaheadMinM, this.config.lookaheadMaxM);
    this.config.hardStopResumeDistanceM = clamp(
      this.config.hardStopResumeDistanceM,
      this.config.safetyDistanceM,
      this.config.safetyResumeDistanceM
    );
    this.config.closeApproachMaxSpeedMps = Math.max(
      this.config.closeApproachMinSpeedMps,
      this.config.closeApproachMaxSpeedMps
    );
    this.config.closeApproachMinThrottlePercent = Math.min(
      this.config.closeApproachMinThrottlePercent,
      this.config.relativeThrottleLimitPercent
    );
    this.config.closeApproachMaxThrottlePercent = clamp(
      this.config.closeApproachMaxThrottlePercent,
      this.config.closeApproachMinThrottlePercent,
      this.config.relativeThrottleLimitPercent
    );
    this.config.distanceHoldUpperM = clamp(
      this.config.distanceHoldUpperM,
      this.config.safetyDistanceM,
      this.config.safetyResumeDistanceM
    );
    this.config.distanceRecoveryResumeM = clamp(
      this.config.distanceRecoveryResumeM,
      this.config.safetyDistanceM,
      this.config.distanceHoldUpperM
    );
    this.config.minimumEffectiveThrottlePercent = Math.min(
      this.config.minimumEffectiveThrottlePercent,
      this.config.relativeThrottleLimitPercent
    );
    this.traditionalFastGuidance = new TraditionalFastGuidance({
      heading_gain: this.config.traditionalHeadingGain,
      yaw_rate_damping: this.config.traditionalYawRateDamping,
      minimum_turn_speed_scale: this.config.traditionalMinimumTurnSpeedScale
    });
    this.reset();
  }

  reset() {
    this.relativeImuControl = new RelativeImuControl(this.config);
    this.previousDistanceM = null;
    this.previousBearingDeg = null;
    this.previousAt = null;
    this.rangeRateMps = 0;
    this.bearingRateDegS = 0;
    this.targetRadialSpeedMps = 0;
    this.rangeIntegralMps = 0;
    this.filteredBearingDeg = null;
    this.steeringCommandDeg = 0;
    this.steeringUpdatedAt = null;
    this.lastSteeringSign = 0;
    this.reverseSteeringCandidateSign = 0;
    this.reverseSteeringCandidateAt = 0;
    this.crossTrackIntegralMs = 0;
    this.throttleCommandPercent = 0;
    this.throttleUpdatedAt = null;
    this.hardStopHolding = false;
    this.distanceRecoveryHolding = false;
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

  preflight({ now = Date.now(), telemetry = {}, connected = false, controlMode = 'auto', targetEstimate = null } = {}) {
    const gps = telemetry.gps || {};
    const uwb = telemetry.uwb || {};
    const attitude = telemetry.attitude || {};
    const boatVelocity = telemetry.velocity || {};
    const position = telemetry.position || {};
    const ekf = telemetry.ekf || {};
    const estimated = usableControlEstimate(targetEstimate, now, this.config.prediction);
    const distanceM = estimated ? Math.hypot(targetEstimate.relativeBodyM.x, targetEstimate.relativeBodyM.y) : finite(uwb.distanceM);
    const estimatedBearingDeg = estimated ? Math.atan2(targetEstimate.relativeBodyM.y, targetEstimate.relativeBodyM.x) / DEG_TO_RAD : null;
    const azimuthDeg = estimated ? normalizeAngleDeg(estimatedBearingDeg - this.config.bearingOffsetDeg)
      / (this.config.bearingSign * this.config.bearingScale) : finite(uwb.azimuthDeg);
    const yawDeg = finite(attitude.yaw);
    const measuredYawRateDegS = finite(attitude.yawRateDegS, 0);
    const boatVx = finite(boatVelocity.vx);
    const boatVy = finite(boatVelocity.vy);
    const uwbUpdatedAt = Object.hasOwn(uwb, 'measurementUpdatedAt') ? finite(uwb.measurementUpdatedAt) : finite(uwb.updatedAt);
    const navigationUpdatedAt = finite(position.updatedAt);
    const ekfUpdatedAt = finite(ekf.updatedAt);

    if (!connected) return zero('pixhawk_offline');
    if (!estimated && (uwb.online !== true || uwb.fresh !== true || distanceM === null || azimuthDeg === null || uwbUpdatedAt === null || uwbUpdatedAt > now || now - uwbUpdatedAt > this.config.uwbTimeoutMs)) {
      return zero('uwb_stale');
    }
    if (yawDeg === null) return zero('imu_invalid');
    let navigationReason = null;
    if (finite(gps.fixType, 0) < this.config.minimumGpsFixType) {
      navigationReason = 'gps_fix_invalid';
    } else if (ekf.healthy !== true || ekfUpdatedAt === null || now - ekfUpdatedAt > this.config.navigationTimeoutMs) {
      navigationReason = 'ekf_unhealthy';
    } else if (navigationUpdatedAt === null || now - navigationUpdatedAt > this.config.navigationTimeoutMs) {
      navigationReason = 'navigation_stale';
    } else if (boatVx === null || boatVy === null) {
      navigationReason = 'navigation_invalid';
    }
    if(this.config.executionMode==='relative_imu') {
      if(!Number.isFinite(attitude.updatedAt) || attitude.updatedAt>now || now-attitude.updatedAt>this.config.navigationTimeoutMs) return zero('imu_stale');
      navigationReason='uwb_imu_only';
    }
    const navigationAvailable = navigationReason === null;
    const requestedControlMode = String(controlMode || 'auto').trim().toLowerCase();
    const selectedControlMode = this.config.executionMode === 'relative_imu' ? 'relative_manual'
      : this.config.executionMode === 'acro' ? 'relative_acro' : this.config.manualPropulsion
      ? 'relative_manual'
      : requestedControlMode === 'guided_velocity'
      ? 'guided_velocity'
      : requestedControlMode === 'relative_acro'
        ? 'relative_acro'
        : navigationAvailable ? 'guided_velocity' : 'relative_acro';
    if (this.config.executionMode === 'acro') {
      if (!navigationAvailable) return zero('acro_velocity_unavailable', {
        controlMode: 'relative_acro', requiredFlightMode: 'ACRO', navigationReason });
      if (telemetry.acroControl?.ready !== true || !Number.isFinite(telemetry.acroControl?.updatedAt)
          || now - telemetry.acroControl.updatedAt > this.config.navigationTimeoutMs
          || telemetry.acroControl.updatedAt > now) {
        return zero('acro_parameters_not_ready', { controlMode: 'relative_acro', requiredFlightMode: 'ACRO' });
      }
    }
    if (selectedControlMode === 'relative_manual' && (telemetry.pilotSteering?.ready !== true ||
      ![0, 1, 2, 3].includes(telemetry.pilotSteering.type))) {
      return zero('pilot_steering_not_ready', { controlMode: selectedControlMode, requiredFlightMode: 'MANUAL' });
    }
    if (selectedControlMode === 'guided_velocity' && !navigationAvailable) {
      return zero(navigationReason || 'navigation_invalid', {
        controlMode: selectedControlMode,
        requiredFlightMode: 'GUIDED',
        navigationReason
      });
    }

    return {
      safe: true,
      reason: 'preflight_ok',
      controlMode: selectedControlMode,
      requiredFlightMode: selectedControlMode === 'relative_manual' ? 'MANUAL' : selectedControlMode === 'guided_velocity' ? 'GUIDED' : 'ACRO',
      navigationAvailable,
      navigationReason,
      distanceM,
      azimuthDeg,
      yawDeg,
      measuredYawRateDegS,
      boatVx,
      boatVy
    };
  }

  update({ now = Date.now(), telemetry = {}, connected = false, controlMode = 'auto', targetEstimate = null, targetMotionEstimate = null } = {}) {
    const preflight = this.preflight({ now, telemetry, connected, controlMode, targetEstimate });
    if (!preflight.safe) return preflight;
    if(this.config.executionMode==='relative_imu') {
      if(String(telemetry.flightMode||'').toUpperCase()!=='MANUAL') return zero('manual_required',{controlMode:'relative_manual',requiredFlightMode:'MANUAL'});
      if(telemetry.armed!==true)return zero('armed_required',{controlMode:'relative_manual',requiredFlightMode:'MANUAL'});
      return this.relativeImuControl.update({now,preflight,telemetry,estimate:targetMotionEstimate,guidance:this.traditionalFastGuidance});
    }
    const {
      distanceM,
      azimuthDeg,
      yawDeg,
      measuredYawRateDegS,
      boatVx,
      boatVy,
      navigationAvailable
    } = preflight;

    const bearingDeg = normalizeAngleDeg(
      azimuthDeg * this.config.bearingSign * this.config.bearingScale
        + this.config.bearingOffsetDeg
    );
    if (this.filteredBearingDeg === null) {
      this.filteredBearingDeg = bearingDeg;
    } else {
      const bearingResidualDeg = normalizeAngleDeg(bearingDeg - this.filteredBearingDeg);
      this.filteredBearingDeg = normalizeAngleDeg(
        this.filteredBearingDeg + this.config.bearingFilterAlpha * bearingResidualDeg
      );
    }
    const controlBearingDeg = this.filteredBearingDeg;
    const bearingRad = bearingDeg * DEG_TO_RAD;
    const relativeBodyM = {
      x: distanceM * Math.cos(bearingRad),
      y: distanceM * Math.sin(bearingRad)
    };
    const effectiveSpeedLimit = Math.min(
      this.config.maxSpeedMps,
      this.config.commissioningMode ? this.config.commissioningSpeedMps : this.config.maxSpeedMps
    );
    if (this.config.hardStopEnabled) {
      if (distanceM <= this.config.safetyDistanceM) {
        this.hardStopHolding = true;
      } else if (this.hardStopHolding && distanceM >= this.config.hardStopResumeDistanceM) {
        this.hardStopHolding = false;
      }
    } else {
      this.hardStopHolding = false;
    }
    const closeApproachActive = this.config.closeApproachEnabled
      && !this.hardStopHolding
      && distanceM < this.config.safetyResumeDistanceM;
    if (!this.config.hardStopEnabled) {
      if (distanceM < this.config.safetyDistanceM) {
        this.distanceRecoveryHolding = true;
      } else if (
        this.distanceRecoveryHolding
        && distanceM >= this.config.distanceRecoveryResumeM
      ) {
        this.distanceRecoveryHolding = false;
      }
    } else {
      this.distanceRecoveryHolding = false;
    }
    const distanceHoldErrorM = distanceM - this.config.distanceRecoveryResumeM;
    const distanceRecoveryActive = closeApproachActive && this.distanceRecoveryHolding;
    const distanceHoldActive = closeApproachActive
      && !distanceRecoveryActive
      && distanceM <= this.config.distanceHoldUpperM;
    const closeApproachSpanM = Math.max(
      0.01,
      this.config.safetyResumeDistanceM - this.config.distanceHoldUpperM
    );
    const closeApproachProgress = clamp(
      (distanceM - this.config.distanceHoldUpperM) / closeApproachSpanM,
      0,
      1
    );
    const closeApproachSpeedMps = this.config.closeApproachMinSpeedMps
      + (this.config.closeApproachMaxSpeedMps - this.config.closeApproachMinSpeedMps)
        * closeApproachProgress;
    const closeApproachThrottlePercent = this.config.closeApproachMinThrottlePercent
      + (this.config.closeApproachMaxThrottlePercent - this.config.closeApproachMinThrottlePercent)
        * closeApproachProgress;
    if (this.hardStopHolding) {
      this.previousDistanceM = null;
      this.previousBearingDeg = null;
      this.previousAt = null;
      this.rangeRateMps = 0;
      this.bearingRateDegS = 0;
      this.targetRadialSpeedMps = 0;
      this.rangeIntegralMps = 0;
      this.filteredBearingDeg = null;
      this.steeringCommandDeg = 0;
      this.steeringUpdatedAt = null;
      this.lastSteeringSign = 0;
      this.reverseSteeringCandidateSign = 0;
      this.reverseSteeringCandidateAt = 0;
      this.crossTrackIntegralMs = 0;
      this.throttleCommandPercent = 0;
      this.throttleUpdatedAt = null;
      return {
        safe: true,
        status: 'holding',
        reason: 'holding_distance',
        controlMode: preflight.controlMode,
        requiredFlightMode: preflight.requiredFlightMode,
        navigationAvailable,
        navigationReason: preflight.navigationReason,
        commandFrame: preflight.controlMode === 'guided_velocity' ? 'BODY_NED' : 'BODY_RELATIVE',
        relativeBodyM,
        distanceM,
        bearingDeg,
        controlBearingDeg: bearingDeg,
        rangeErrorM: distanceM - this.config.followingDistanceM,
        rangeRateMps: 0,
        bearingRateDegS: 0,
        targetRadialSpeedMps: 0,
        rangeIntegralMps: 0,
        boatTowardTargetMps: 0,
        turnSlowdownScale: 0,
        closeApproachActive: false,
        trackingPhase: 'hard_stop',
        forwardSpeedMps: 0,
        yawRateDegS: 0,
        yawRateRadS: 0,
        throttlePercent: 0,
        steeringDegrees: 0,
        velocityNed: { x: 0, y: 0 },
        desiredSpeedMps: 0,
        speedLimitMps: effectiveSpeedLimit,
        commissioningMode: this.config.commissioningMode,
        holdingDistanceM: this.config.safetyDistanceM,
        hardStopResumeDistanceM: this.config.hardStopResumeDistanceM
      };
    }
    if (String(telemetry.flightMode || '').toUpperCase() !== preflight.requiredFlightMode) {
      return zero(preflight.requiredFlightMode === 'MANUAL' ? 'manual_required' : preflight.requiredFlightMode === 'GUIDED' ? 'guided_required' : 'acro_required', {
        controlMode: preflight.controlMode,
        requiredFlightMode: preflight.requiredFlightMode,
        navigationAvailable: preflight.navigationAvailable,
        navigationReason: preflight.navigationReason
      });
    }
    if (telemetry.armed !== true) return zero('armed_required', {
      controlMode: preflight.controlMode,
      requiredFlightMode: preflight.requiredFlightMode,
      navigationAvailable: preflight.navigationAvailable,
      navigationReason: preflight.navigationReason
    });
    const absoluteBearingRad = (yawDeg + bearingDeg) * DEG_TO_RAD;
    const boatTowardTargetMps = navigationAvailable
      ? boatVx * Math.cos(absoluteBearingRad) + boatVy * Math.sin(absoluteBearingRad)
      : 0;

    let sampleDt = 0;
    if (this.previousAt !== null && this.previousDistanceM !== null && this.previousBearingDeg !== null) {
      const dt = (now - this.previousAt) / 1000;
      if (dt >= 0.03 && dt <= 0.6) {
        sampleDt = dt;
        const measuredRangeRate = (distanceM - this.previousDistanceM) / dt;
        const measuredBearingRate = normalizeAngleDeg(controlBearingDeg - this.previousBearingDeg) / dt;
        const bearingAlpha = this.config.bearingRateAlpha;
        this.rangeRateMps = measuredRangeRate;
        this.bearingRateDegS = this.bearingRateDegS * (1 - bearingAlpha) + measuredBearingRate * bearingAlpha;

        // Relative range rate plus boat velocity projected along line-of-sight
        // estimates target radial speed, providing steady-state feed-forward.
        if (navigationAvailable) {
          const measuredTargetRadialSpeed = measuredRangeRate + boatTowardTargetMps;
          const speedAlpha = this.config.targetSpeedAlpha;
          this.targetRadialSpeedMps = this.targetRadialSpeedMps * (1 - speedAlpha)
            + measuredTargetRadialSpeed * speedAlpha;
        } else {
          this.targetRadialSpeedMps = 0;
        }
      } else if (dt > 0.6) {
        this.rangeRateMps = 0;
        this.bearingRateDegS = 0;
        this.targetRadialSpeedMps = 0;
        this.rangeIntegralMps = 0;
      }
    }
    this.previousDistanceM = distanceM;
    this.previousBearingDeg = controlBearingDeg;
    this.previousAt = now;

    let speedFeedForwardSource = 'range_difference';
    if (this.config.executionMode === 'acro' && navigationAvailable
      && targetMotionEstimate?.evaluatedAt === now && targetMotionEstimate.velocityEstimate) {
      const velocity = targetMotionEstimate.targetVelocityNedMps;
      const trusted = targetMotionEstimate.velocityEstimate.trusted === true
        && Number.isFinite(velocity?.x) && Number.isFinite(velocity?.y);
      this.targetRadialSpeedMps = trusted
        ? velocity.x * Math.cos(absoluteBearingRad) + velocity.y * Math.sin(absoluteBearingRad) : 0;
      // The longer, acceleration-bounded motion track supplies feed-forward.
      // Do not differentiate noisy positions again after confirming velocity.
      if (trusted) this.rangeRateMps = this.targetRadialSpeedMps - boatTowardTargetMps;
      speedFeedForwardSource = trusted ? 'confirmed_motion' : 'position_only';
    }

    const rangeErrorM = distanceM - this.config.followingDistanceM;
    const controlledRangeErrorM = Math.abs(rangeErrorM) <= this.config.rangeDeadbandM ? 0 : rangeErrorM;
    if (preflight.controlMode === 'relative_acro' || preflight.controlMode === 'relative_manual') {
      if (sampleDt > 0) {
        this.rangeIntegralMps += this.config.rangeIntegralGain * controlledRangeErrorM * sampleDt;
        this.rangeIntegralMps = clamp(
          this.rangeIntegralMps,
          -Math.min(this.config.rangeIntegralLimitMps, this.config.maxReverseSpeedMps),
          Math.min(this.config.rangeIntegralLimitMps, effectiveSpeedLimit)
        );
      }
    } else {
      this.rangeIntegralMps = 0;
    }
    const acroVelocity = this.config.executionMode === 'acro';
    // ACRO requests a velocity, so target velocity is a physical feed-forward.
    // MANUAL retains its existing throttle/integral control law.
    const speedFeedForwardMps = preflight.controlMode === 'guided_velocity' || acroVelocity
      ? this.targetRadialSpeedMps
      : this.rangeIntegralMps;
    const relativeSpeedTracking = acroVelocity && navigationAvailable
      && speedFeedForwardSource === 'confirmed_motion';
    let relativeSpeedCorrectionMps = this.config.rangeGain * controlledRangeErrorM
      + this.config.rangeRateGain * this.rangeRateMps;
    let relativeApproachLimitMps = null;
    if (relativeSpeedTracking && this.config.closeApproachEnabled) {
      // A moving target's speed is the base, not part of the approach budget.
      // Use one continuous bound on both sides of the old 1.2 m mode boundary.
      // Near the requested spacing the position error naturally reduces the
      // correction to zero; there is no positive minimum pursuit velocity.
      relativeApproachLimitMps = distanceHoldActive || distanceRecoveryActive ? 0
        : this.config.closeApproachMaxSpeedMps * closeApproachProgress + this.config.rangeGain
          * Math.max(0, distanceM - this.config.safetyResumeDistanceM);
      relativeSpeedCorrectionMps = Math.min(relativeSpeedCorrectionMps, relativeApproachLimitMps);
    }
    let forwardSpeedMps = speedFeedForwardMps + relativeSpeedCorrectionMps;
    forwardSpeedMps = clamp(forwardSpeedMps, -this.config.maxReverseSpeedMps, effectiveSpeedLimit);

    const previousCommandSpeedMps = Math.abs(this.throttleCommandPercent)
      / this.config.relativeThrottleLimitPercent * effectiveSpeedLimit;
    const lookaheadM = clamp(
      this.config.lookaheadBaseM
        + this.config.lookaheadSpeedGainS * Math.max(previousCommandSpeedMps, Math.abs(forwardSpeedMps)),
      this.config.lookaheadMinM,
      this.config.lookaheadMaxM
    );
    if (sampleDt > 0) {
      const crossTrackErrorM = relativeBodyM.y;
      const signChanged = this.crossTrackIntegralMs !== 0
        && crossTrackErrorM !== 0
        && Math.sign(crossTrackErrorM) !== Math.sign(this.crossTrackIntegralMs);
      const decayRate = this.config.ilosIntegralDecay + (signChanged ? 2 : 0);
      this.crossTrackIntegralMs *= Math.exp(-decayRate * sampleDt);
      this.crossTrackIntegralMs += crossTrackErrorM * sampleDt;
      const integralLimitMs = this.config.ilosIntegralGain > 0
        ? lookaheadM * Math.tan(this.config.ilosMaxCorrectionDeg * DEG_TO_RAD) / this.config.ilosIntegralGain
        : 0;
      this.crossTrackIntegralMs = clamp(this.crossTrackIntegralMs, -integralLimitMs, integralLimitMs);
    }
    const ilosCorrectionDeg = clamp(
      Math.atan2(this.config.ilosIntegralGain * this.crossTrackIntegralMs, lookaheadM) / DEG_TO_RAD,
      -this.config.ilosMaxCorrectionDeg,
      this.config.ilosMaxCorrectionDeg
    );
    const guidanceBearingDeg = normalizeAngleDeg(controlBearingDeg + ilosCorrectionDeg);
    const absGuidanceBearingDeg = Math.abs(guidanceBearingDeg);
    const deadbandedBearingDeg = absGuidanceBearingDeg <= this.config.bearingDeadbandDeg
      ? 0
      : Math.sign(guidanceBearingDeg) * (absGuidanceBearingDeg - this.config.bearingDeadbandDeg);
    const curvatureMInv = clamp(
      2 * Math.sin(deadbandedBearingDeg * DEG_TO_RAD) / lookaheadM,
      -this.config.maxCurvatureMInv,
      this.config.maxCurvatureMInv
    );
    const curvatureSpeedScale = Math.max(
      this.config.minimumCurveSpeedScale,
      1 / (1 + this.config.curvatureSpeedGain * Math.abs(curvatureMInv))
    );
    // Keep the physical UWB target within the receiver's forward cone. The
    // LOS rate comes from the board's confirmed target track and own GPS/EKF,
    // never simulator truth. Gyro removes our turn from the bearing forecast.
    let lineOfSightRateDegS = 0;
    if (acroVelocity && navigationAvailable && targetMotionEstimate?.evaluatedAt === now
      && targetMotionEstimate.velocityEstimate?.trusted === true) {
      const v = targetMotionEstimate.targetVelocityNedMps;
      if (Number.isFinite(v?.x) && Number.isFinite(v?.y)) {
        const psi = yawDeg * DEG_TO_RAD;
        const dx = v.x - boatVx, dy = v.y - boatVy;
        const vf = dx * Math.cos(psi) + dy * Math.sin(psi);
        const vr = -dx * Math.sin(psi) + dy * Math.cos(psi);
        lineOfSightRateDegS = clamp((relativeBodyM.x * vr - relativeBodyM.y * vf)
          / Math.max(.09, distanceM ** 2) / DEG_TO_RAD,
          -this.config.maxYawRateDegS, this.config.maxYawRateDegS);
      }
    }
    const fovSlowDeg = this.config.fieldOfViewHalfDeg - 25;
    const fovStopDeg = this.config.fieldOfViewHalfDeg - 10;
    const outwardRateDegS = Math.max(0, Math.sign(bearingDeg || lineOfSightRateDegS)
      * (lineOfSightRateDegS - measuredYawRateDegS));
    const fovProjectedBearingDeg = Math.max(Math.abs(bearingDeg), Math.abs(controlBearingDeg))
      + .5 * outwardRateDegS;
    const fovSpeedScale = acroVelocity
      ? clamp((fovStopDeg - fovProjectedBearingDeg) / (fovStopDeg - fovSlowDeg), 0, 1) : 1;
    const fovYawPriority = acroVelocity && Math.abs(bearingDeg) >= fovSlowDeg;
    const traditionalGuidance = this.traditionalFastGuidance.command({
      headingErrorDeg: acroVelocity ? bearingDeg : guidanceBearingDeg,
      measuredYawRateDegS: measuredYawRateDegS - lineOfSightRateDegS,
      forwardSpeedMps,
      maximumYawRateDegS: this.config.maxYawRateDegS
    });
    if (acroVelocity) traditionalGuidance.desiredYawRateDegS = clamp(
      traditionalGuidance.desiredYawRateDegS + lineOfSightRateDegS,
      -this.config.maxYawRateDegS, this.config.maxYawRateDegS);
    const turnSpeedScale = this.config.controllerProfile === 'traditional_fast'
      ? traditionalGuidance.speedScale
      : curvatureSpeedScale;
    forwardSpeedMps *= turnSpeedScale;
    const turnAlignmentScale = acroVelocity ? Math.min(fovSpeedScale, clamp(
      (this.config.bearingStopDeg - absGuidanceBearingDeg)
        / (this.config.bearingStopDeg - this.config.bearingSlowdownDeg), 0, 1)) : 1;
    // Preserve yaw authority while slowing translation to face a turning target.
    forwardSpeedMps *= turnAlignmentScale;
    if (closeApproachActive) {
      this.rangeIntegralMps = 0;
      const closeSpeed = distanceHoldActive
        ? 0
        : distanceRecoveryActive
          ? -Math.min(this.config.maxReverseSpeedMps, this.config.closeApproachMinSpeedMps)
          : closeApproachSpeedMps * Math.max(0.15, turnSpeedScale);
      if (relativeSpeedTracking && !distanceRecoveryActive) {
        // At the desired separation a moving pair should keep the same speed.
        // Preserve heading slowdown, braking and the physical speed limit.
        const relativeCap = (speedFeedForwardMps + relativeApproachLimitMps)
          * turnSpeedScale * turnAlignmentScale;
        forwardSpeedMps = Math.min(forwardSpeedMps, relativeCap);
      } else {
        // An unconfirmed velocity cannot raise the legacy cautious approach.
        forwardSpeedMps = acroVelocity ? Math.min(forwardSpeedMps, closeSpeed) : closeSpeed;
      }
    }
    forwardSpeedMps = clamp(forwardSpeedMps, -this.config.maxReverseSpeedMps, effectiveSpeedLimit);

    let brakingSpeedLimitMps = effectiveSpeedLimit;
    let brakingDistanceM = 0;
    let brakingClearanceM = null;
    let brakingActive = false;
    if (acroVelocity && navigationAvailable) {
      const deceleration = Math.max(0.05, finite(this.config.prediction.braking_deceleration_mps2, 0.3));
      const ageS = Math.max(0, (now - finite(telemetry.uwb?.measurementUpdatedAt, now)) / 1000);
      // Include sensing/FCU response delay before the conservative braking model.
      const reactionS = 0.6 + Math.min(ageS, this.config.uwbTimeoutMs / 1000);
      const marginM = Math.max(0.05, finite(this.config.prediction.measurement_noise_m, 0.12));
      const targetSpeed = distanceRecoveryActive ? 0 : clamp(Math.min(this.targetRadialSpeedMps,
        boatTowardTargetMps + this.rangeRateMps), 0, effectiveSpeedLimit);
      const closingSpeed = Math.max(0, boatTowardTargetMps - targetSpeed);
      brakingClearanceM = Math.max(0, distanceM - this.config.followingDistanceM - marginM);
      brakingDistanceM = closingSpeed * reactionS + closingSpeed ** 2 / (2 * deceleration);
      const closingLimit = Math.sqrt((deceleration * reactionS) ** 2
        + 2 * deceleration * brakingClearanceM) - deceleration * reactionS;
      brakingSpeedLimitMps = Math.min(effectiveSpeedLimit, targetSpeed + Math.max(0, closingLimit));
      brakingActive = forwardSpeedMps > brakingSpeedLimitMps
        || brakingDistanceM >= brakingClearanceM;
      if (brakingActive) this.rangeIntegralMps = Math.min(0, this.rangeIntegralMps);
      forwardSpeedMps = Math.min(forwardSpeedMps, brakingSpeedLimitMps);
    }

    const proportionalThrottlePercent = clamp(
      forwardSpeedMps / effectiveSpeedLimit * this.config.relativeThrottleLimitPercent,
      -this.config.relativeThrottleLimitPercent,
      this.config.relativeThrottleLimitPercent
    );
    const applyMinimumEffectiveThrottle = !acroVelocity && rangeErrorM >= this.config.minimumThrottleRangeErrorM
      && proportionalThrottlePercent > 0.01;
    const desiredThrottlePercent = acroVelocity ? proportionalThrottlePercent : closeApproachActive
      ? distanceHoldActive
        ? 0
        : distanceRecoveryActive
          ? -this.config.closeApproachMinThrottlePercent
          : closeApproachThrottlePercent
      : applyMinimumEffectiveThrottle
        ? Math.sign(proportionalThrottlePercent) * Math.max(
          Math.abs(proportionalThrottlePercent),
          this.config.minimumEffectiveThrottlePercent
        )
        : proportionalThrottlePercent;
    const throttleDt = this.throttleUpdatedAt === null
      ? 0.1
      : clamp((now - this.throttleUpdatedAt) / 1000, 0, 0.25);
    this.throttleUpdatedAt = now;
    const throttleSlowing = Math.sign(desiredThrottlePercent) !== Math.sign(this.throttleCommandPercent)
      || Math.abs(desiredThrottlePercent) < Math.abs(this.throttleCommandPercent);
    const throttleRate = throttleSlowing
      ? this.config.throttleDecelPercentS
      : this.config.throttleAccelPercentS;
    const throttleTarget = this.throttleCommandPercent !== 0
      && desiredThrottlePercent !== 0
      && Math.sign(desiredThrottlePercent) !== Math.sign(this.throttleCommandPercent)
      ? 0
      : desiredThrottlePercent;
    this.throttleCommandPercent = moveToward(
      this.throttleCommandPercent,
      throttleTarget,
      throttleRate * throttleDt
    );
    // The comfort slew may delay braking by more than a metre. Safety bounds
    // apply to the actual FCU request, not only the unslewed desired velocity.
    if (acroVelocity) this.throttleCommandPercent = Math.min(this.throttleCommandPercent,
      brakingSpeedLimitMps / effectiveSpeedLimit * this.config.relativeThrottleLimitPercent);
    if (acroVelocity) this.throttleCommandPercent = clamp(this.throttleCommandPercent,
      -fovSpeedScale * this.config.relativeThrottleLimitPercent,
      fovSpeedScale * this.config.relativeThrottleLimitPercent);
    if (Math.abs(this.throttleCommandPercent) < 0.01) this.throttleCommandPercent = 0;
    const throttlePercent = this.throttleCommandPercent;
    const commandedForwardSpeedMps = throttlePercent
      / this.config.relativeThrottleLimitPercent * effectiveSpeedLimit;
    const desiredYawRateDegS = this.config.controllerProfile === 'traditional_fast'
      ? traditionalGuidance.desiredYawRateDegS
      : clamp(
        commandedForwardSpeedMps * curvatureMInv / DEG_TO_RAD,
        -this.config.maxYawRateDegS,
        this.config.maxYawRateDegS
      );
    const sameDirectionSteeringLimitDeg = acroVelocity ? this.config.relativeSteeringLimitDeg : Math.abs(throttlePercent) * this.config.sameDirectionSteeringRatio;
    const unconstrainedSteeringDeg = desiredYawRateDegS
      / this.config.maxYawRateDegS * this.config.relativeSteeringLimitDeg;
    const sameDirectionLimited = Math.abs(unconstrainedSteeringDeg) > sameDirectionSteeringLimitDeg + 1e-9;
    let desiredSteeringDeg = clamp(
      unconstrainedSteeringDeg,
      -Math.min(this.config.relativeSteeringLimitDeg, sameDirectionSteeringLimitDeg),
      Math.min(this.config.relativeSteeringLimitDeg, sameDirectionSteeringLimitDeg)
    );
    const desiredSteeringSign = Math.sign(desiredSteeringDeg);
    if (acroVelocity) {
      // Keep actuator slew, but do not add reversal confirmation delay while
      // a trusted target is approaching the edge of the physical field of view.
      this.reverseSteeringCandidateSign = 0;
      this.reverseSteeringCandidateAt = 0;
    } else if (desiredSteeringSign !== 0 && this.lastSteeringSign !== 0 && desiredSteeringSign !== this.lastSteeringSign) {
      if (absGuidanceBearingDeg < this.config.bearingReverseConfirmDeg) {
        desiredSteeringDeg = 0;
        this.reverseSteeringCandidateSign = 0;
        this.reverseSteeringCandidateAt = 0;
      } else {
        if (this.reverseSteeringCandidateSign !== desiredSteeringSign) {
          this.reverseSteeringCandidateSign = desiredSteeringSign;
          this.reverseSteeringCandidateAt = now;
        }
        const reversalConfirmed = now - this.reverseSteeringCandidateAt >= this.config.bearingReverseConfirmMs;
        const steeringAtNeutral = Math.abs(this.steeringCommandDeg) < 0.05;
        if (!reversalConfirmed || !steeringAtNeutral) {
          desiredSteeringDeg = 0;
        } else {
          this.lastSteeringSign = desiredSteeringSign;
          this.reverseSteeringCandidateSign = 0;
          this.reverseSteeringCandidateAt = 0;
        }
      }
    } else if (desiredSteeringSign === this.lastSteeringSign) {
      this.reverseSteeringCandidateSign = 0;
      this.reverseSteeringCandidateAt = 0;
    }

    const steeringDt = this.steeringUpdatedAt === null
      ? 0.1
      : clamp((now - this.steeringUpdatedAt) / 1000, 0, 0.25);
    this.steeringUpdatedAt = now;
    const steeringReturning = Math.abs(desiredSteeringDeg) < Math.abs(this.steeringCommandDeg);
    const steeringRate = steeringReturning ? this.config.steeringReturnDegS : this.config.steeringSlewDegS;
    // In ACRO these degrees encode a bounded yaw-rate request, not a rudder
    // position. At the cone edge, waiting for the extra comfort ramp after
    // reacquisition can lose the target again. The FCU retains its inner loop.
    this.steeringCommandDeg = acroVelocity ? desiredSteeringDeg : moveToward(
      this.steeringCommandDeg,
      desiredSteeringDeg,
      steeringRate * steeringDt
    );
    if (Math.abs(this.steeringCommandDeg) < 0.01) this.steeringCommandDeg = 0;
    if (this.steeringCommandDeg !== 0 && desiredSteeringDeg !== 0) {
      this.lastSteeringSign = Math.sign(this.steeringCommandDeg);
    }
    const finalSameDirectionLimitDeg = acroVelocity ? this.config.relativeSteeringLimitDeg : Math.abs(throttlePercent) * this.config.sameDirectionSteeringRatio;
    this.steeringCommandDeg = clamp(
      this.steeringCommandDeg,
      -Math.min(this.config.relativeSteeringLimitDeg, finalSameDirectionLimitDeg),
      Math.min(this.config.relativeSteeringLimitDeg, finalSameDirectionLimitDeg)
    );
    const steeringDegrees = this.steeringCommandDeg;
    const yawRateDegS = steeringDegrees / this.config.relativeSteeringLimitDeg * this.config.maxYawRateDegS;
    const yawRateRadS = yawRateDegS * DEG_TO_RAD;
    const yawRad = yawDeg * DEG_TO_RAD;
    const velocityNed = {
      x: commandedForwardSpeedMps * Math.cos(yawRad),
      y: commandedForwardSpeedMps * Math.sin(yawRad)
    };
    return {
      safe: true,
      reason: 'tracking',
      controlMode: preflight.controlMode,
      requiredFlightMode: preflight.requiredFlightMode,
      navigationAvailable,
      navigationReason: preflight.navigationReason,
      fieldOfViewHalfDeg: this.config.fieldOfViewHalfDeg,
      fovProjectedBearingDeg, fovSpeedScale, fovYawPriority, lineOfSightRateDegS,
      commandFrame: preflight.controlMode === 'guided_velocity' ? 'BODY_NED' : 'BODY_RELATIVE',
      relativeBodyM,
      distanceM,
      bearingDeg,
      controlBearingDeg,
      guidanceBearingDeg,
      deadbandedBearingDeg,
      crossTrackErrorM: relativeBodyM.y,
      crossTrackIntegralMs: this.crossTrackIntegralMs,
      ilosCorrectionDeg,
      lookaheadM,
      curvatureMInv,
      rangeErrorM,
      rangeRateMps: this.rangeRateMps,
      bearingRateDegS: this.bearingRateDegS,
      targetRadialSpeedMps: this.targetRadialSpeedMps,
      speedFeedForwardSource,
      relativeSpeedTracking,
      relativeSpeedCorrectionMps,
      relativeApproachLimitMps,
      rangeIntegralMps: this.rangeIntegralMps,
      boatTowardTargetMps,
      controllerProfile: this.config.controllerProfile,
      measuredYawRateDegS,
      traditionalHeadingGain: this.config.traditionalHeadingGain,
      traditionalYawRateDamping: this.config.traditionalYawRateDamping,
      traditionalRawYawRateDegS: traditionalGuidance.rawYawRateDegS,
      traditionalTurnSpeedScale: traditionalGuidance.speedScale,
      turnSlowdownScale: turnSpeedScale,
      curvatureSpeedScale,
      turnAlignmentScale,
      sameDirectionSteeringLimitDeg,
      sameDirectionLimited,
      closeApproachActive,
      closeApproachProgress,
      closeApproachSpeedMps,
      closeApproachThrottlePercent,
      distanceHoldErrorM,
      distanceHoldActive,
      distanceRecoveryActive,
      brakingActive,
      brakingSpeedLimitMps,
      brakingDistanceM,
      brakingClearanceM,
      trackingPhase: closeApproachActive
        ? distanceHoldActive
          ? 'distance_hold'
          : distanceRecoveryActive ? 'distance_recovery' : 'close_approach'
        : this.config.controllerProfile === 'traditional_fast' && absGuidanceBearingDeg > this.config.bearingDeadbandDeg
          ? 'traditional_fast_turn'
          : Math.abs(curvatureMInv) > 0.05 ? 'arc_turn' : 'approaching',
      forwardSpeedMps: commandedForwardSpeedMps,
      desiredForwardSpeedMps: forwardSpeedMps,
      proportionalThrottlePercent,
      minimumEffectiveThrottleApplied: applyMinimumEffectiveThrottle,
      commandedForwardSpeedMps,
      desiredThrottlePercent,
      desiredYawRateDegS,
      yawRateDegS,
      yawRateRadS,
      throttlePercent,
      steeringDegrees,
      velocityNed,
      desiredSpeedMps: Math.abs(forwardSpeedMps),
      speedLimitMps: effectiveSpeedLimit,
      commissioningMode: this.config.commissioningMode,
      holdingDistanceM: this.config.safetyDistanceM,
      hardStopResumeDistanceM: this.config.hardStopResumeDistanceM
    };
  }
}
