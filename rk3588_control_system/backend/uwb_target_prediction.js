// All coordinates here are local NED metres (+X north, +Y east).
// Prediction never rewrites a sensor value, validity bit or measurement time.
import { RawWindowRecovery } from './uwb_raw_recovery.js';
const rad = Math.PI / 180;
const number = (x) => x === null || x === undefined || x === '' ? null : Number.isFinite(Number(x)) ? Number(x) : null;
const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));
const wrap = (x) => ((x + 180) % 360 + 360) % 360 - 180;
const rotate = (p, angle) => ({ x: p.x * Math.cos(angle) - p.y * Math.sin(angle), y: p.x * Math.sin(angle) + p.y * Math.cos(angle) });
const distance = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
const fresh = (at, now, timeout) => number(at) !== null && at > 0 && now >= at && now - at <= timeout;

export const PREDICTION_DEFAULTS = Object.freeze({
  enabled: false, horizon_ms: 1000, wait_timeout_ms: 5000, pose_timeout_ms: 700,
  min_samples: 3, measurement_gap_ms: 450, measurement_noise_m: 0.12,
  velocity_uncertainty_mps: 0.2, acceleration_bound_mps2: 0.6,
  max_uncertainty_m: 0.5, max_target_speed_mps: 3, innovation_gate_m: 0.6,
  max_speed_mps: 0.35, max_throttle_percent: 15, max_steering_deg: 6,
  clearance_m: 1.2, braking_deceleration_mps2: 0.3, recovery_blend_ms: 800
});

function identity(uwb) {
  const active = uwb.activeReceiverIds || [];
  const tags = (uwb.sensors || []).filter((s) => active.includes(s.receiverId))
    .map((s) => number(s.tagId)).filter((id) => id !== null);
  if (number(uwb.tagId) !== null) tags.push(number(uwb.tagId));
  return tags.length ? [...new Set(tags)].sort().join(',') : null;
}

function measurementKey(uwb) {
  const sequences = (uwb.sensors || []).filter((s) => (uwb.activeReceiverIds || []).includes(s.receiverId)
    && number(s.sequenceId) !== null).map((s) => `${s.receiverId}:${s.sequenceId}`).sort();
  return sequences.length ? sequences.join('|') : `time:${uwb.measurementUpdatedAt}`;
}

function sensorUncertainty(uwb) {
  const ids = uwb?.activeReceiverIds || [];
  const values = (uwb?.sensors || []).filter((s) => !ids.length || ids.includes(s.receiverId))
    .map((s) => number(s.filterUncertaintyM)).filter((value) => value !== null && value >= 0);
  return values.length ? Math.max(...values) : 0;
}

// Versioned contract: measurement filtering owns outlier admission and raw
// reacquisition. Navigation transforms an admitted posterior, not raw candidates.
function admissionContract(uwb, now, gap) {
  const sensors = uwb?.sensors || [];
  const versioned = sensors.some(s => s.filterAdmissionVersion === 1);
  const selected = sensors.filter(s => (uwb.activeReceiverIds || []).includes(s.receiverId));
  const confirmed = versioned && uwb.measurementStatus === 'trusted' && selected.length > 0
    && selected.every(s => s.filterAdmissionVersion === 1 && s.measurementStatus === 'trusted'
      && s.valid !== false && fresh(s.measurementUpdatedAt, now, gap)
      && s.receivedMeasurementAt === s.measurementUpdatedAt
      && number(s.filterUncertaintyM) !== null && s.filterUncertaintyM >= 0)
    && Math.max(...selected.map(s => s.measurementUpdatedAt)) === uwb.measurementUpdatedAt;
  return { versioned, confirmed };
}

export function usableControlEstimate(estimate, now, prediction) {
  const config = { ...PREDICTION_DEFAULTS, ...prediction };
  return config.enabled === true && estimate?.usable === true
    && ['predicted', 'recovering'].includes(estimate.source)
    && estimate.evaluatedAt === now && fresh(estimate.lastMeasuredAt, now, config.horizon_ms)
    && number(estimate.relativeBodyM?.x) !== null && number(estimate.relativeBodyM?.y) !== null
    && number(estimate.uncertaintyM) !== null && estimate.uncertaintyM >= 0
    && (estimate.source === 'recovering' && estimate.measurementConfirmed === true
      ? fresh(estimate.lastMeasuredAt, now, config.measurement_gap_ms)
      : estimate.uncertaintyM <= config.max_uncertainty_m);
}

export class UwbTargetEstimator {
  constructor(config = {}, calibration = {}) {
    this.config = { ...PREDICTION_DEFAULTS, ...config };
    this.calibration = calibration;
    this.reset();
  }

  reset() {
    this.last = null;
    this.origin = null;
    this.lastPose = null;
    this.lastSeenMeasurementAt = null;
    this.lastSeenToken = null;
    this.lossAt = null;
    this.recovery = null;
    this.recoveredAt = null;
    this.rawWindow = new RawWindowRecovery(this.config);
    this.recoveryAnchor = null;
    this.motionHistory = [];
    this.motionVelocity = { x: 0, y: 0 };
    this.motionAt = null;
    this.motionTrustedAt = null;
    this.motionDiagnostic = { trusted: false, reason: 'warming_up', samples: 0, spanMs: 0 };
  }

  pose(now, telemetry) {
    const yaw = number(telemetry.attitude?.yaw);
    if(this.calibration.execution_mode==='relative_imu') {
      if(yaw===null || !fresh(telemetry.attitude?.updatedAt,now,this.config.pose_timeout_ms))return null;
      // Yaw-stabilized relative frame, with no vessel translation or ground
      // velocity. Common translation of the two boats is unobservable here.
      const pose={yaw,vx:0,vy:0,position:{x:0,y:0},at:now,relativeOnly:true};
      this.lastPose=pose;return pose;
    }
    const vx = number(telemetry.velocity?.vx), vy = number(telemetry.velocity?.vy);
    const c = this.config;
    if (yaw === null || vx === null || vy === null || telemetry.ekf?.healthy !== true
      || !fresh(telemetry.attitude?.updatedAt, now, c.pose_timeout_ms)
      || !fresh(telemetry.position?.updatedAt, now, c.pose_timeout_ms)
      || !fresh(telemetry.ekf?.updatedAt, now, c.pose_timeout_ms)) return null;
    // Short-window local odometry also works with a healthy non-GPS EKF.
    // Reject a gap rather than integrating an unobserved vessel trajectory.
    const dt = this.lastPose ? (now - this.lastPose.at) / 1000 : 0;
    if (dt < 0 || dt * 1000 > c.pose_timeout_ms) return null;
    const position = this.lastPose ? {
      x: this.lastPose.position.x + (vx + this.lastPose.vx) * 0.5 * dt,
      y: this.lastPose.position.y + (vy + this.lastPose.vy) * 0.5 * dt
    } : { x: 0, y: 0 };
    const pose = { yaw, vx, vy, position, at: now };
    this.lastPose = pose;
    return pose;
  }

  sample(uwb, pose, now) {
    // Only a complete DIST+AZ observation from the bridge advances this clock.
    const at = number(uwb.measurementUpdatedAt);
    const range = number(uwb.distanceM), azimuth = number(uwb.azimuthDeg);
    if (!pose || uwb.online !== true || uwb.fresh !== true || range === null || range <= 0 || azimuth === null
      || !fresh(at, now, this.config.measurement_gap_ms)) return null;
    const sign = number(this.calibration.bearing_sign) ?? -1;
    const scale = number(this.calibration.bearing_scale) ?? 0.8148;
    const offset = number(this.calibration.bearing_offset_deg) ?? -4.816;
    const bearing = wrap(azimuth * sign * scale + offset);
    const body = { x: range * Math.cos(bearing * rad), y: range * Math.sin(bearing * rad) };
    const relative = rotate(body, pose.yaw * rad);
    const age = (now - at) / 1000;
    // Back-project own position to the observation time instead of attributing
    // network latency / own motion to the swimmer.
    return { at, token: measurementKey(uwb), id: identity(uwb), measurementUncertaintyM: sensorUncertainty(uwb), position: {
      x: pose.position.x - pose.vx * age + relative.x,
      y: pose.position.y - pose.vy * age + relative.y
    } };
  }

  projected(sample, now) {
    const dt = Math.max(0, (now - sample.at) / 1000);
    return { x: sample.position.x + sample.velocity.x * dt, y: sample.position.y + sample.velocity.y * dt };
  }

  observeMotion(sample) {
    const c = this.config, minimumSpan = Math.max(c.measurement_gap_ms, c.recovery_blend_ms);
    const previous = this.motionHistory.at(-1);
    const dt = this.motionAt === null ? 0 : Math.max(0, (sample.at - this.motionAt) / 1000);
    this.motionAt = sample.at;
    if (previous && (sample.at - previous.at > c.measurement_gap_ms
      || distance(sample.position, previous.position) > c.innovation_gate_m
        + c.max_target_speed_mps * (sample.at - previous.at) / 1000)) {
      this.motionHistory = []; this.motionVelocity = { x: 0, y: 0 }; this.motionTrustedAt = null;
    }
    this.motionHistory = this.motionHistory.filter(p => sample.at - p.at <= 2 * minimumSpan);
    this.motionHistory.push(sample);
    const points = this.motionHistory, spanMs = points.at(-1).at - points[0].at;
    const minimumSamples = 2 * c.min_samples;
    const median = values => { const a = [...values].sort((x,y) => x-y); const i = Math.floor(a.length/2);
      return a.length % 2 ? a[i] : (a[i-1]+a[i])/2; };
    let reason = 'velocity_warming_up', inliers = [];
    if (points.length >= minimumSamples && spanMs >= minimumSpan) {
      const slopes = { x: [], y: [] };
      for (let i=0;i<points.length-1;i++) for (let j=i+1;j<points.length;j++) {
        const gap = (points[j].at-points[i].at)/1000;
        if (gap*1000 < minimumSpan*.75) continue;
        for (const axis of ['x','y']) slopes[axis].push((points[j].position[axis]-points[i].position[axis])/gap);
      }
      if (slopes.x.length) {
        const velocity = { x: median(slopes.x), y: median(slopes.y) };
        const position = {};
        for (const axis of ['x','y']) position[axis] = median(points.map(p => p.position[axis]
          - velocity[axis]*(p.at-sample.at)/1000));
        inliers = points.filter(p => distance(p.position, {
          x: position.x+velocity.x*(p.at-sample.at)/1000,
          y: position.y+velocity.y*(p.at-sample.at)/1000 }) <= 2*c.measurement_noise_m);
        if (inliers.length >= minimumSamples && inliers.at(-1).at === sample.at
          && inliers.at(-1).at-inliers[0].at >= minimumSpan
          && Math.hypot(velocity.x,velocity.y) <= c.max_target_speed_mps) {
          // A short position reacquisition cannot reset the velocity. Even a
          // longer, robust track changes velocity only within the existing
          // configured acceleration bound.
          const delta = { x: velocity.x-this.motionVelocity.x, y: velocity.y-this.motionVelocity.y };
          const magnitude = Math.hypot(delta.x,delta.y);
          const limit = c.acceleration_bound_mps2*Math.min(dt,c.measurement_gap_ms/1000);
          const scale = magnitude > limit ? limit/magnitude : 1;
          this.motionVelocity.x += delta.x*scale; this.motionVelocity.y += delta.y*scale;
          this.motionTrustedAt = sample.at; reason = 'velocity_confirmed';
        } else reason = 'velocity_inconsistent';
      }
    }
    const trusted = fresh(this.motionTrustedAt,sample.at,c.measurement_gap_ms);
    this.motionDiagnostic = { trusted, reason, samples: points.length, inliers: inliers.length, spanMs };
    return trusted ? { ...this.motionVelocity } : { x: 0, y: 0 };
  }

  accept(sample, previous = this.last, recoveredSamples = null) {
    const dt = previous ? (sample.at - previous.at) / 1000 : 0;
    const velocity = this.observeMotion(sample);
    this.last = { ...sample, velocity, velocityTrusted: this.motionDiagnostic.trusted,
      samples: recoveredSamples ?? (previous && dt > 0 && dt * 1000 <= this.config.measurement_gap_ms
        ? previous.samples + 1 : 1) };
  }

  update(now, telemetry) {
    const c = this.config;
    if (this.lossAt !== null && now - this.lossAt >= c.wait_timeout_ms)
      return { terminal: true, reason: 'prediction_wait_timeout', source: 'unavailable', evaluatedAt: now };
    const pose = this.pose(now, telemetry);
    const sample = this.sample(telemetry.uwb || {}, pose, now);
    const admission = admissionContract(telemetry.uwb || {}, now, c.measurement_gap_ms);
    const rawSensors=(telemetry.uwb?.sensors || []).filter(s=>number(s.receivedMeasurementAt)!==null || number(s.rawRejectedAt)!==null);
    // The production vessel path owns reacquisition. No simulation adapter or
    // simulator truth is involved. Legacy telemetry without raw provenance
    // retains its existing compatibility path.
    const rawRecovery=!admission.versioned && this.lossAt!==null && rawSensors.length===1 && pose && this.last;
    if (sample && this.last?.id!==null && this.last?.id!==undefined && sample.id!==null && this.last.id!==sample.id)
      return {terminal:true,reason:'target_identity_changed',source:'unavailable',evaluatedAt:now};
    if (!admission.versioned && rawSensors.length===1 && pose && this.last)
      this.rawWindow.observe(now,telemetry,this,this.recoveryAnchor || this.last,Boolean(rawRecovery));
    const newSample = !rawRecovery && (!admission.versioned || admission.confirmed)
      && sample && sample.token !== this.lastSeenToken && (this.lastSeenMeasurementAt === null || sample.at > this.lastSeenMeasurementAt);
    let rejection = null;
    if (newSample) {
      this.lastSeenMeasurementAt = sample.at;
      this.lastSeenToken = sample.token;
      if (this.last?.id !== null && this.last?.id !== undefined && sample.id !== null && this.last.id !== sample.id) {
        return { terminal: true, reason: 'target_identity_changed', source: 'unavailable', evaluatedAt: now };
      }
      const reference = this.recovery || this.last;
      const dt = reference ? (sample.at - reference.at) / 1000 : 0;
      const gap = dt * 1000;
      const expected = reference ? this.projected(reference, sample.at) : sample.position;
      const allowed = c.innovation_gate_m + c.velocity_uncertainty_mps * Math.max(0, dt)
        + 0.5 * c.acceleration_bound_mps2 * Math.max(0, dt) ** 2;
      if (admission.confirmed) {
        const recovering = this.lossAt !== null;
        sample.admissionVersion = 1;
        this.accept(sample);
        this.lossAt = null; this.recovery = null; this.recoveryAnchor = null;
        if (recovering) this.recoveredAt = now;
      } else if (reference && (gap <= 0 || distance(sample.position, expected) > allowed
        || (gap <= c.measurement_gap_ms && distance(sample.position, reference.position) > c.measurement_noise_m * 2 + c.max_target_speed_mps * dt))) {
        rejection = 'measurement_jump';
        this.recovery = null;
      } else if (this.lossAt !== null) {
        const previous = this.recovery;
        const samples = previous && gap <= c.measurement_gap_ms ? previous.samples + 1 : 1;
        const velocity = previous && dt > 0 ? { x: (sample.position.x - previous.position.x) / dt,
          y: (sample.position.y - previous.position.y) / dt } : this.last?.velocity || { x: 0, y: 0 };
        const speed = Math.hypot(velocity.x, velocity.y);
        if (speed > c.max_target_speed_mps) {
          velocity.x *= c.max_target_speed_mps / speed;
          velocity.y *= c.max_target_speed_mps / speed;
        }
        this.recovery = { ...sample, velocity, samples };
        if (samples >= c.min_samples) {
          this.accept(this.recovery,this.last,samples);
          this.recovery = null;
          this.lossAt = null;
          this.recoveryAnchor = null;
          this.recoveredAt = now;
        }
      } else {
        this.accept(sample);
      }
    }
    const observationFresh = (!admission.versioned || admission.confirmed) && sample && !rejection && this.last && sample.token === this.last.token
      && fresh(this.last.at, now, c.measurement_gap_ms);
    if (!observationFresh && this.lossAt === null) {
      this.lossAt = now;
      this.recoveryAnchor=this.last?structuredClone(this.last):null;
      // Keep the bounded raw evidence window across a single bad frame. It
      // still expires on observation timestamps and needs a new trusted handoff.
    }
    if (this.lossAt !== null && now - this.lossAt >= c.wait_timeout_ms) {
      return { terminal: true, reason: 'prediction_wait_timeout', source: 'unavailable', evaluatedAt: now };
    }
    if (!this.last || !pose) return { usable: false, source: 'unavailable', evaluatedAt: now,
      reason: pose ? 'prediction_warming_up' : 'prediction_pose_unavailable', lastMeasuredAt: this.last?.at ?? null };
    const ageMs = now - this.last.at;
    const measurementUncertaintyM = Math.max(c.measurement_noise_m, this.last.measurementUncertaintyM || 0, sensorUncertainty(telemetry.uwb));
    const uncertaintyM = measurementUncertaintyM + c.velocity_uncertainty_mps * ageMs / 1000
      + 0.5 * c.acceleration_bound_mps2 * (ageMs / 1000) ** 2;
    const target = this.projected(this.last, now);
    const body = rotate({ x: target.x - pose.position.x, y: target.y - pose.position.y }, -pose.yaw * rad);
    const recovering = observationFresh && this.recoveredAt !== null && now - this.recoveredAt < c.recovery_blend_ms;
    const source = observationFresh ? recovering ? 'recovering' : 'measured' : 'predicted';
    const measurementConfirmed = Boolean(observationFresh
      && (!rawSensors.length || telemetry.uwb?.measurementStatus === 'trusted')
      && Number.isFinite(measurementUncertaintyM) && measurementUncertaintyM >= 0);
    const usable = !rejection && (measurementConfirmed
      ? ageMs >= 0 && ageMs <= c.measurement_gap_ms
      : !observationFresh && ageMs >= 0 && ageMs <= c.horizon_ms
        && uncertaintyM <= c.max_uncertainty_m && this.last.samples >= c.min_samples);
    return { usable, source, evaluatedAt: now, lastMeasuredAt: this.last.at, predictionAgeMs: ageMs,
      uncertaintyM, measurementUncertaintyM, measurementConfirmed,
      admissionVersion: this.last.admissionVersion || 0,
      positionAdmission: admission.versioned ? (measurementConfirmed ? 'filter_admitted' : 'await_filter') : 'legacy_window',
      inputMeasurementStatus: telemetry.uwb?.measurementStatus || 'unknown',
      inputRejectionReason: telemetry.uwb?.rejectionReason || null,
      relativeBodyM: body, targetPositionNedM: pose.relativeOnly?null:target,
      targetVelocityNedMps: pose.relativeOnly?null:{ ...this.last.velocity },
      relativeVelocityHeadingMps:pose.relativeOnly?{...this.last.velocity}:null,
      referenceFrame:pose.relativeOnly?'yaw_stabilized_relative':'local_ned',
      relativeClosingSpeedMps:pose.relativeOnly?Math.max(0,-(target.x*this.last.velocity.x+target.y*this.last.velocity.y)/Math.max(.01,Math.hypot(target.x,target.y))):null,
      boatSpeedMps: pose.relativeOnly?null:Math.hypot(pose.vx, pose.vy), samples: this.last.samples,
      velocityEstimate: { ...this.motionDiagnostic },
      recoverySamples: this.recovery?.samples ?? 0,
      independentRecovery: {...this.rawWindow.reacquisitionDiagnostic},
      reacquisitionFit: this.rawWindow.reacquisitionDiagnostic.reason==='confirmed'?this.rawWindow.reacquisitionFit:null,
      reacquiring: this.lossAt !== null && this.lastSeenMeasurementAt > this.last.at,
      recoveryScale: recovering ? clamp((now - this.recoveredAt) / c.recovery_blend_ms, 0, 1) : 1,
      reason: rejection || (!usable ? 'prediction_limit' : source) };
  }
}

export class IntermittentUwbFollow {
  constructor(prediction = {}, calibration = {}) {
    this.config = { ...PREDICTION_DEFAULTS, ...prediction };
    this.rejectNmpcEnabled = calibration.reject_nmpc?.enabled === true;
    this.estimator = new UwbTargetEstimator(this.config, calibration);
    this.reset();
  }
  reset() { this.estimator.reset(); this.lastThrottle = 0; this.lastSteering = 0; this.lastForwardSpeedMps = 0; this.controlMode = this.controlMode || 'relative_manual'; this.rawRecovery = null; this.rawRecoveredAt = null; this.targetId = null; this.lastNow = null;
    this.recoveryCycleAt = null; this.recoveryGoodMs = 0; this.recoveryStepAt = null;
    this.recoveryStepGood = false; this.lastPositionGoodAt = null;
    this.liveCommandAt = null; this.liveCommandSpeed = 0; this.admittedDrive = null; }

  coastAcceptedPosition(now, estimate, telemetry, controller) {
    const c = this.config, drive = this.admittedDrive;
    if (this.controlMode !== 'relative_acro' || estimate.admissionVersion !== 1
      || estimate.source !== 'predicted' || !drive
      || !fresh(estimate.lastMeasuredAt,now,c.measurement_gap_ms)
      || !fresh(drive.at,now,c.measurement_gap_ms)
      || !Number.isFinite(estimate.uncertaintyM) || !estimate.relativeBodyM) return null;
    const body = estimate.relativeBodyM, range = Math.hypot(body.x,body.y);
    const bearing = Math.atan2(body.y,body.x)/rad;
    const slow = controller.config.fieldOfViewHalfDeg-25;
    const stop = controller.config.fieldOfViewHalfDeg-10;
    if (Math.abs(bearing)>=stop) return null;
    // An unresolved close raw return must veto continuation as before.
    if ((telemetry.uwb?.sensors || []).some(s =>
      fresh(s.rawDistanceUpdatedAt ?? s.receivedMeasurementAt,now,c.measurement_gap_ms)
      && number(s.rawDistanceM)!==null && s.rawDistanceM>=0 && s.rawDistanceM<=c.clearance_m)) return null;
    const ownSpeed = Math.hypot(telemetry.velocity.vx,telemetry.velocity.vy);
    const clearance = range-estimate.uncertaintyM-Math.max(c.clearance_m,controller.config.safetyDistanceM);
    const stopDistance = ownSpeed*.6+ownSpeed**2/(2*c.braking_deceleration_mps2);
    if (!(clearance>stopDistance)) return null;
    const alignment = clamp((stop-Math.abs(bearing))/(stop-slow),0,1);
    const speed = Math.max(0,Math.min(drive.speed*alignment,
      Math.sqrt(2*c.braking_deceleration_mps2*Math.max(0,clearance-ownSpeed*.6))));
    const yaw = controller.traditionalFastGuidance.command({headingErrorDeg:bearing,
      measuredYawRateDegS:Number(telemetry.attitude.yawRateDegS)||0,
      forwardSpeedMps:speed,maximumYawRateDegS:controller.config.maxYawRateDegS}).desiredYawRateDegS;
    // Never renew the admitted measurement or drive clock during continuation.
    // Preserve a bounded outer-loop starting point; fresh evidence is required
    // before it may request an increase again.
    controller.throttleCommandPercent=speed/Math.min(2,controller.config.maxSpeedMps)*controller.config.relativeThrottleLimitPercent;
    return {safe:true,status:'predicting',reason:'accepted_position_coast',controlMode:'relative_acro',
      requiredFlightMode:'ACRO',commandFrame:'BODY_RELATIVE',navigationAvailable:true,
      targetEstimate:estimate,relativeBodyM:body,bearingDeg:bearing,distanceM:range,
      forwardSpeedMps:speed,desiredSpeedMps:speed,yawRateDegS:yaw,yawRateRadS:yaw*rad,
      throttlePercent:controller.throttleCommandPercent,steeringDegrees:yaw/controller.config.maxYawRateDegS*controller.config.relativeSteeringLimitDeg,
      speedLimitMps:Math.min(2,controller.config.maxSpeedMps),shortGapHold:true,
      coastMeasurementAgeMs:now-estimate.lastMeasuredAt,coastClearanceM:clearance,
      coastStoppingDistanceM:stopDistance,coastPreviousCommandMps:drive.speed};
  }

  positionRecoveryScale(now, estimate) {
    const good = estimate.usable && estimate.measurementConfirmed
      && ['measured','recovering'].includes(estimate.source);
    const cycle = this.estimator.recoveredAt;
    if (estimate.source === 'recovering' && cycle !== this.recoveryCycleAt) {
      if (this.recoveryCycleAt === null || !fresh(this.lastPositionGoodAt,now,this.config.measurement_gap_ms))
        this.recoveryGoodMs = 0;
      this.recoveryCycleAt = cycle;
      this.recoveryStepGood = false;
    }
    if (good && this.recoveryCycleAt !== null && this.recoveryStepGood && this.recoveryStepAt !== null)
      this.recoveryGoodMs = Math.min(this.config.recovery_blend_ms,
        this.recoveryGoodMs+Math.max(0,now-this.recoveryStepAt));
    if (good) this.lastPositionGoodAt = now;
    this.recoveryStepGood = Boolean(good); this.recoveryStepAt = now;
    return this.recoveryCycleAt === null ? null : clamp(this.recoveryGoodMs/this.config.recovery_blend_ms,0,1);
  }

  update({ controller, now, telemetry, connected, controlMode }) {
    this.controlMode = controller.config.executionMode === 'acro' || controlMode === 'relative_acro' ? 'relative_acro' : 'relative_manual';
    if (!this.config.enabled) return controller.update({ now, telemetry, connected, controlMode });
    if (this.lastNow !== null && now < this.lastNow) return this.neutral('prediction_clock_reset', null, false);
    this.lastNow = now;
    // These failures always exit; prediction never initiates/rearms a session.
    let fault = null;
    if (!connected) fault = 'pixhawk_offline';
    else if (telemetry.armed !== true) fault = 'armed_required';
    else if (String(telemetry.flightMode).toUpperCase() !== (this.controlMode === 'relative_acro' ? 'ACRO' : 'MANUAL')) fault = this.controlMode === 'relative_acro' ? 'acro_required' : 'manual_required';
    else if (this.controlMode === 'relative_acro' && (telemetry.acroControl?.ready !== true
      || !fresh(telemetry.acroControl?.updatedAt, now, this.config.pose_timeout_ms)
      || !telemetry.ekf?.healthy || !fresh(telemetry.ekf?.updatedAt, now, this.config.pose_timeout_ms)
      || !fresh(telemetry.position?.updatedAt, now, this.config.pose_timeout_ms)
      || Number(telemetry.gps?.fixType || 0) < 2)) fault = 'acro_velocity_unavailable';
    else if (telemetry.pilotSteering?.ready !== true || ![0, 1, 2, 3].includes(telemetry.pilotSteering.type)) fault = 'pilot_steering_not_ready';
    else if (number(telemetry.attitude?.yaw) === null) fault = 'imu_invalid';
    else if (!fresh(telemetry.attitude?.updatedAt, now, this.config.pose_timeout_ms)) fault = 'imu_stale';
    else if (telemetry.uwb?.rejectionReason === 'target_identity_changed') fault = 'target_identity_changed';
    if (fault) return this.neutral(fault, null, false);
    const calibration = { bearing_sign: controller.config.bearingSign, bearing_scale: controller.config.bearingScale,
      bearing_offset_deg: controller.config.bearingOffsetDeg,
      ...(controller.config.executionMode==='relative_imu'?{execution_mode:'relative_imu'}:{}) };
    if (Object.keys(calibration).some((key) => calibration[key] !== this.estimator.calibration[key])) {
      this.reset();
      this.estimator.calibration = calibration;
    }
    const wasLost = this.estimator.lossAt !== null;
    const estimate = this.estimator.update(now, telemetry);
    if (estimate.terminal) return this.neutral(estimate.reason, estimate, false);
    const positionRecoveryScale = this.positionRecoveryScale(now,estimate);
    // Real measurements still work without GPS; prediction requires fresh pose.
    const uwb = telemetry.uwb || {};
    const rawFresh = uwb.online === true && uwb.fresh === true && number(uwb.distanceM) !== null && uwb.distanceM > 0
      && number(uwb.azimuthDeg) !== null && fresh(uwb.measurementUpdatedAt, now, controller.config.uwbTimeoutMs);
    const id = rawFresh ? identity(uwb) : null;
    if (id !== null && this.targetId !== null && id !== this.targetId) return this.neutral('target_identity_changed', estimate, false);
    if (id !== null) this.targetId = id;
    // Without a trustworthy velocity estimate, hold during loss but allow
    // recovery from consecutive real UWB observations (never from prediction).
    if (estimate.reason === 'prediction_pose_unavailable' && rawFresh) {
      const bearing = (uwb.azimuthDeg * controller.config.bearingSign * controller.config.bearingScale + controller.config.bearingOffsetDeg) * rad;
      const body = { x: uwb.distanceM * Math.cos(bearing), y: uwb.distanceM * Math.sin(bearing) };
      if (wasLost) {
        const previous = this.rawRecovery;
        const token = measurementKey(uwb);
        if (!previous || (uwb.measurementUpdatedAt > previous.at && token !== previous.token)) {
          const dt = previous ? (uwb.measurementUpdatedAt - previous.at) / 1000 : 0;
          const consistent = previous && dt > 0 && dt * 1000 <= this.config.measurement_gap_ms
            && distance(body, previous.body) <= this.config.innovation_gate_m + 2 * this.config.max_target_speed_mps * dt;
          this.rawRecovery = { at: uwb.measurementUpdatedAt, token, body, samples: consistent ? previous.samples + 1 : 1 };
        }
        if (this.rawRecovery.samples < this.config.min_samples) return this.neutral('prediction_reacquiring', {
          ...estimate, recoverySamples: this.rawRecovery.samples }, true);
        this.estimator.reset();
        this.rawRecovery = null;
        this.rawRecoveredAt = now;
      } else this.estimator.lossAt = null;
      const rawResult = controller.update({ now, telemetry, connected, controlMode });
      const recovering = this.rawRecoveredAt !== null && now - this.rawRecoveredAt < this.config.recovery_blend_ms;
      if (!recovering) return { ...rawResult, targetEstimate: estimate };
      const scale = clamp((now - this.rawRecoveredAt) / this.config.recovery_blend_ms, 0, 1);
      return { ...rawResult, status: 'recovering', reason: 'prediction_recovering',
        targetEstimate: { ...estimate, source: 'recovering', lastMeasuredAt: uwb.measurementUpdatedAt, relativeBodyM: body },
        intermittentThrottleCap: this.config.max_throttle_percent * scale, intermittentSteeringCap: this.config.max_steering_deg * scale,
        throttlePercent: clamp(rawResult.throttlePercent, 0, this.config.max_throttle_percent * scale) };
    }
    const measured = (estimate.source === 'measured' && estimate.usable)
      || (estimate.source === 'recovering' && estimate.usable && positionRecoveryScale === 1)
      || (!wasLost && estimate.reason === 'prediction_pose_unavailable'
      && uwb.online === true && uwb.fresh === true && fresh(uwb.measurementUpdatedAt, now, controller.config.uwbTimeoutMs));
    if (measured && estimate.source === 'unavailable') this.estimator.lossAt = null;
    const predicted = estimate.source === 'predicted';
    if (predicted && this.rejectNmpcEnabled && this.controlMode === 'relative_acro') {
      // Route the FIRST rejected/missing observation to the NMPC assistance
      // branch. Do not first coast on the old command for measurement_gap_ms.
      // The estimator owns admission and the unchanged 5 s terminal deadline.
      controller.reset();
      return { ...this.neutral('reject_nmpc_planning', estimate, true), nmpcRequested: true };
    }
    const coast = predicted ? this.coastAcceptedPosition(now,estimate,telemetry,controller) : null;
    if (coast) return coast;
    let allowed = estimate.usable;
    let pauseReason = estimate.reason;
    if (predicted) {
      const speed = estimate.referenceFrame==='yaw_stabilized_relative'?estimate.relativeClosingSpeedMps:estimate.boatSpeedMps;
      const stopDistance = speed ** 2 / (2 * this.config.braking_deceleration_mps2);
      const clearance = Math.hypot(estimate.relativeBodyM.x, estimate.relativeBodyM.y) - estimate.uncertaintyM;
      if (speed > this.config.max_speed_mps) { allowed = false; pauseReason = 'prediction_speed_limit'; }
      if (clearance <= this.config.clearance_m + stopDistance) { allowed = false; pauseReason = 'prediction_clearance'; }
      if (estimate.reacquiring) { allowed = false; pauseReason = 'prediction_reacquiring'; }
      const suspectClose = (telemetry.uwb?.sensors || []).some((s) => s.measurementStatus === 'suspect'
        && fresh(s.rawDistanceUpdatedAt, now, this.config.measurement_gap_ms)
        && number(s.rawDistanceM) !== null && s.rawDistanceM >= 0 && s.rawDistanceM <= this.config.clearance_m);
      if (suspectClose) { allowed = false; pauseReason = 'prediction_clearance'; }
    }
    if (!measured && !allowed) {
      // The FCU is commanded to zero while waiting. Forget the outer-loop
      // throttle, derivative and integrals as well; recovery starts from zero.
      if(controller.config.executionMode==='relative_imu')controller.relativeImuControl.pause();
      else controller.reset();
      return this.neutral(pauseReason || 'prediction_limit', estimate, true);
    }
    // Live uncertainty limits physical clearance and speed, independently of
    // the 0.5 m / 1 s blind-prediction budget. Do not discard the covariance.
    const measurementClearance = estimate.measurementConfirmed
      ? Math.hypot(estimate.relativeBodyM.x,estimate.relativeBodyM.y)-estimate.uncertaintyM
        -controller.config.safetyDistanceM : null;
    if (measurementClearance !== null && measurementClearance <= 0) {
      controller.reset();
      return this.neutral('measurement_clearance',estimate,true);
    }
    if (estimate.admissionVersion===1 && estimate.measurementConfirmed && this.controlMode==='relative_acro'
      && controller.previousAt===null) {
      const psi=telemetry.attitude.yaw*rad;
      const own=Math.max(0,telemetry.velocity.vx*Math.cos(psi)+telemetry.velocity.vy*Math.sin(psi));
      controller.throttleCommandPercent=Math.min(own,controller.config.maxSpeedMps)
        /controller.config.maxSpeedMps*controller.config.relativeThrottleLimitPercent;
    }
    const result = controller.update({ now, telemetry, connected, controlMode,
      targetEstimate: measured ? null : estimate, targetMotionEstimate: estimate });
    if (!result.safe) return { ...result, targetEstimate: estimate };
    if (measurementClearance !== null) {
      const speedCap=Math.sqrt(2*this.config.braking_deceleration_mps2*measurementClearance);
      result.measurementClearanceM=measurementClearance;
      result.measurementSpeedLimitMps=speedCap;
      result.forwardSpeedMps=Math.min(result.forwardSpeedMps,speedCap);
      result.desiredSpeedMps=Math.min(result.desiredSpeedMps??result.forwardSpeedMps,speedCap);
    }
    if (result.reason === 'holding_distance') return { ...result, targetEstimate: estimate };
    if (estimate.admissionVersion === 1 && estimate.measurementConfirmed && this.controlMode === 'relative_acro') {
      // Confirmed position can drive while velocity warms up. Preserve the
      // controller's distance/braking/FOV limits; do not apply blind 0.35 m/s
      // limits to live evidence. Accelerate from measured own speed, never from
      // an imagined target speed or a stale command after a long gap.
      const yaw = telemetry.attitude.yaw * rad;
      const ownForward = telemetry.velocity.vx*Math.cos(yaw)+telemetry.velocity.vy*Math.sin(yaw);
      const dt = this.liveCommandAt === null ? 0 : clamp((now-this.liveCommandAt)/1000,0,this.config.measurement_gap_ms/1000);
      const recent = this.liveCommandAt !== null && now-this.liveCommandAt <= this.config.measurement_gap_ms;
      // Fresh measurements authorize a bounded command ramp. Tracking lag in
      // the FCU/current is not evidence of an unsafe target position: an extra
      // own-speed +0.2 cap previously prevented the speed loop from catching up.
      // A long gap still re-seeds from actual own motion; braking and clearance
      // above, plus the controller's relative-closure/FOV limits, stay active.
      const start = recent ? this.liveCommandSpeed : Math.max(0,ownForward);
      // Reuse the existing propulsion command slew, converted to m/s per s.
      // acceleration_bound_mps2 describes TARGET motion uncertainty; it is
      // not an additional own-vessel actuator limit. Controller safety caps
      // still apply before this recovery governor.
      const speedLimit = Math.min(controller.config.maxSpeedMps,
        controller.config.commissioningMode ? controller.config.commissioningSpeedMps : controller.config.maxSpeedMps);
      const acceleration = speedLimit * controller.config.throttleAccelPercentS
        / controller.config.relativeThrottleLimitPercent;
      const cap = start + acceleration*dt;
      result.forwardSpeedMps = Math.min(result.forwardSpeedMps,cap);
      result.desiredSpeedMps = Math.min(result.desiredSpeedMps ?? result.forwardSpeedMps,cap);
      this.liveCommandSpeed = result.forwardSpeedMps; this.liveCommandAt = now;
      return { ...result, targetEstimate: estimate, liveRecoverySpeedCapMps: cap,
        liveRecoveryReferenceMps:start,liveRecoveryStepS:dt,
        liveRecoveryAccelerationMps2:acceleration,
        status: estimate.source === 'recovering' ? 'recovering' : result.status,
        reason: estimate.source === 'recovering' ? 'measurement_recovering' : result.reason };
    }
    if (measured) {
      if ((positionRecoveryScale !== null && positionRecoveryScale < 1)
        || (this.rawRecoveredAt !== null && now - this.rawRecoveredAt < this.config.recovery_blend_ms)) {
        const scale = positionRecoveryScale ?? clamp((now - this.rawRecoveredAt) / this.config.recovery_blend_ms, 0, 1);
        return { ...result, status: 'recovering', reason: 'prediction_recovering',
          targetEstimate: { ...estimate, source: 'recovering' },
          intermittentThrottleCap: this.config.max_throttle_percent * scale,
          intermittentSteeringCap: this.controlMode === 'relative_acro'
            ? controller.config.relativeSteeringLimitDeg : this.config.max_steering_deg * scale,
          throttlePercent: clamp(result.throttlePercent, 0, this.config.max_throttle_percent * scale) };
      }
      return { ...result, targetEstimate: estimate };
    }
    const decay = predicted ? Math.max(0, 1 - estimate.predictionAgeMs / this.config.horizon_ms)
      : positionRecoveryScale ?? estimate.recoveryScale;
    const throttleCap = this.config.max_throttle_percent * decay;
    // A confirmed, fresh reacquisition is a measured turn. Applying the blind
    // prediction steering cap here limits ACRO yaw to 4.7 deg/s, less than a
    // turning target can require. Keep the propulsion recovery ramp separate.
    const confirmedAcroTurn = this.controlMode === 'relative_acro' && !predicted
      && estimate.source === 'recovering' && estimate.usable;
    const steeringCap = confirmedAcroTurn ? controller.config.relativeSteeringLimitDeg
      : this.config.max_steering_deg * decay;
    return { ...result, status: predicted ? 'predicting' : 'recovering', reason: predicted ? 'prediction_tracking' : 'prediction_recovering',
      targetEstimate: estimate, intermittentThrottleCap: throttleCap, intermittentSteeringCap: steeringCap,
      throttlePercent: clamp(result.throttlePercent, 0, throttleCap), steeringDegrees: clamp(result.steeringDegrees, -steeringCap, steeringCap),
      forwardSpeedMps: clamp(result.forwardSpeedMps, 0, this.config.max_speed_mps * decay),
      desiredSpeedMps: clamp(result.desiredSpeedMps ?? result.forwardSpeedMps, 0, this.config.max_speed_mps * decay) };
  }

  neutral(reason, estimate, waiting) {
    this.lastThrottle = 0; this.lastSteering = 0;
    this.admittedDrive = null;
    this.lastForwardSpeedMps = 0;
    return { safe: waiting, status: waiting ? 'waiting_target' : 'stopped', reason,
      ...(this.estimator.calibration.execution_mode==='relative_imu'?{relativeImuControl:true,absoluteSpeedObservable:false}:{}),
      pausePropulsion: waiting, targetEstimate: estimate, relativeBodyM: estimate?.relativeBodyM || { x: 0, y: 0 },
      controlMode: this.controlMode, requiredFlightMode: this.controlMode === 'relative_acro' ? 'ACRO' : 'MANUAL', commandFrame: 'BODY_RELATIVE',
      throttlePercent: 0, steeringDegrees: 0, forwardSpeedMps: 0, yawRateDegS: 0, yawRateRadS: 0,
      desiredSpeedMps: 0, velocityNed: { x: 0, y: 0 } };
  }

  governAcro(result, speedLimitMps, yawLimitDegS, steeringSpan, throttleSpan) {
    if (!result.safe || result.pausePropulsion) { this.lastForwardSpeedMps = 0; return { forwardSpeedMps: 0, yawRateDegS: 0 }; }
    let speed = clamp(Number(result.forwardSpeedMps) || 0, -0.2, Math.min(2, speedLimitMps));
    let yaw = clamp(Number(result.yawRateDegS) || 0, -yawLimitDegS, yawLimitDegS);
    if (result.intermittentThrottleCap !== undefined) {
      let cap = Math.min(this.config.max_speed_mps, speedLimitMps * Math.max(0, result.intermittentThrottleCap) / Math.max(throttleSpan, 1));
      if (result.targetEstimate?.source === 'predicted') cap = Math.min(cap, Math.max(0, this.lastForwardSpeedMps));
      speed = clamp(speed, 0, cap);
      const turnCap = yawLimitDegS * Math.max(0, result.intermittentSteeringCap || 0) / Math.max(steeringSpan, 1);
      yaw = clamp(yaw, -turnCap, turnCap);
    }
    this.lastForwardSpeedMps = speed;
    if (result.targetEstimate?.admissionVersion===1 && result.targetEstimate.measurementConfirmed)
      this.admittedDrive={at:result.targetEstimate.evaluatedAt,speed};
    else if (result.shortGapHold && this.admittedDrive)
      this.admittedDrive.speed=Math.min(this.admittedDrive.speed,speed);
    return { forwardSpeedMps: speed, yawRateDegS: yaw };
  }

  governDrive(output, result) {
    if (result.pausePropulsion || !result.safe) { this.lastThrottle = 0; this.lastSteering = 0; return { ...output, throttle: 0, steering: 0 }; }
    let throttle = output.throttle, steering = output.steering;
    if (result.intermittentThrottleCap !== undefined) {
      const cap = result.targetEstimate?.source === 'predicted'
        ? Math.min(result.intermittentThrottleCap, Math.max(0, this.lastThrottle)) : result.intermittentThrottleCap;
      throttle = clamp(throttle, 0, cap);
      steering = clamp(steering, -Math.min(result.intermittentSteeringCap, throttle * 0.45), Math.min(result.intermittentSteeringCap, throttle * 0.45));
    }
    this.lastThrottle = throttle; this.lastSteering = steering;
    return { ...output, throttle, steering };
  }
}
