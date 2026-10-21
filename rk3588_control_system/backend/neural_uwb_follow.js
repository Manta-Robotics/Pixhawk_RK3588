import fs from 'fs';
import path from 'path';
import { UwbFollowController, normalizeAngleDeg } from './uwb_follow.js';

const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;

function finite(value, fallback = null) {
  if (value === null || value === undefined || value === '') return fallback;
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function clamp(value, minimum, maximum) {
  return Math.min(maximum, Math.max(minimum, value));
}

function rotate(x, y, angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return { x: c * x - s * y, y: s * x + c * y };
}

function wrapRad(value) {
  return ((value + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
}

function dense(input, layer) {
  return layer.weight.map((row, outputIndex) => {
    let value = layer.bias[outputIndex];
    for (let inputIndex = 0; inputIndex < row.length; inputIndex += 1) {
      value += row[inputIndex] * input[inputIndex];
    }
    return layer.activation === 'elu' && value < 0 ? Math.expm1(value) : value;
  });
}

export class JsonActor {
  constructor(modelPath) {
    this.modelPath = path.resolve(modelPath);
    this.model = JSON.parse(fs.readFileSync(this.modelPath, 'utf8'));
    if (this.model.format !== 'smart_usv_rsl_rl_actor_v1') throw new Error('unsupported_neural_policy_format');
    if (this.model.observationNames?.length !== 20 || this.model.layers?.length !== 4) {
      throw new Error('invalid_neural_policy_dimensions');
    }
    this.verifyGolden();
  }

  infer(observation) {
    if (!Array.isArray(observation) || observation.length !== 20 || !observation.every(Number.isFinite)) {
      throw new Error('invalid_neural_observation');
    }
    const { mean, std, epsilon } = this.model.normalizer;
    let values = observation.map((value, index) => (value - mean[index]) / (std[index] + epsilon));
    for (const layer of this.model.layers) values = dense(values, layer);
    return values.map((value) => clamp(value, -1, 1));
  }

  verifyGolden(tolerance = 2e-5) {
    const golden = this.model.golden;
    if (!golden?.observations?.length || golden.observations.length !== golden.actions?.length) {
      throw new Error('neural_policy_golden_vectors_missing');
    }
    golden.observations.forEach((observation, sampleIndex) => {
      const actual = this.infer(observation);
      actual.forEach((value, actionIndex) => {
        if (Math.abs(value - golden.actions[sampleIndex][actionIndex]) > tolerance) {
          throw new Error(`neural_policy_self_test_failed_${sampleIndex}_${actionIndex}`);
        }
      });
    });
  }
}

/**
 * SmartUSV PPO outer loop. The actor selects speed and absolute compass
 * heading; virtual stick targets use the shared manual propulsion layer.
 */
export class NeuralUwbFollowController extends UwbFollowController {
  constructor(config = {}) {
    super(config);
    this.policyConfig = {
      modelPath: config.neural_model_path,
      maxSpeedMps: clamp(finite(config.neural_max_speed_mps, 3), 0.1, 3),
      stopDistanceM: Math.max(0.1, finite(config.neural_safety_stop_distance_m, 0.65)),
      resumeDistanceM: Math.max(0.2, finite(config.neural_safety_resume_distance_m, 0.8)),
      targetVelocityAlpha: clamp(finite(config.neural_target_velocity_alpha, 0.25), 0.01, 1),
    };
    if (this.policyConfig.resumeDistanceM <= this.policyConfig.stopDistanceM) {
      throw new Error('neural_safety_resume_must_exceed_stop');
    }
    this.actor = new JsonActor(this.policyConfig.modelPath);
    this.reset();
  }

  reset() {
    super.reset();
    this.policyInitialized = false;
    this.filter = [0, 0, 0];
    this.uncertainty = [1, 1, 1];
    this.lastMeasurementAt = null;
    this.lastRelativeEnu = { x: 0, y: 0 };
    this.targetVelocityEnu = { x: 0, y: 0 };
    this.previousCommandSpeedMps = 0;
    this.previousCommandHeadingRad = null;
    this.neuralSafetyLatched = false;
    this.neuralThrottlePercent = 0;
    this.neuralSteeringDeg = 0;
  }

  preflight(input = {}) {
    // Keep the existing relative-UWB sensor checks, even when GPS is available.
    // Execution mode selects the physical adapter; the trained actor is unchanged.
    const result = super.preflight({ ...input, controlMode: 'relative_acro' });
    const pilot = input.telemetry?.pilotSteering;
    const pilotReady = pilot?.ready === true && [0, 1, 2, 3].includes(pilot.type);
    return { ...result, ...(result.safe && !pilotReady ? { safe: false, reason: 'pilot_steering_not_ready' } : {}),
      trackingMethod: 'neural', controlMode: this.config.executionMode === 'acro' ? 'relative_acro' : 'relative_manual',
      requiredFlightMode: this.config.executionMode === 'acro' ? 'ACRO' : 'MANUAL',
      commandFrame: 'BODY_RELATIVE', propulsion: this.config.executionMode === 'acro' ? 'acro_velocity' : 'dual_thrust' };
  }

  ingestMeasurement(telemetry, preflight) {
    const uwb = telemetry.uwb || {};
    const measurementAt = Object.hasOwn(uwb, 'measurementUpdatedAt') ? finite(uwb.measurementUpdatedAt) : finite(uwb.updatedAt);
    if (measurementAt === null || measurementAt === this.lastMeasurementAt) return;
    const raw = [
      preflight.distanceM,
      preflight.azimuthDeg,
      finite(uwb.elevationDeg, 0),
    ];
    if (!this.policyInitialized) {
      this.filter = raw.slice();
      this.policyInitialized = true;
    } else {
      const q = [0.01, 0.8, 0.8];
      const r = [0.16, 9, 9];
      for (let index = 0; index < 3; index += 1) {
        const predicted = this.uncertainty[index] + q[index];
        const gain = predicted / (predicted + r[index]);
        let residual = raw[index] - this.filter[index];
        if (index > 0) residual = normalizeAngleDeg(residual);
        this.filter[index] += gain * residual;
        if (index > 0) this.filter[index] = normalizeAngleDeg(this.filter[index]);
        this.uncertainty[index] = (1 - gain) * predicted;
      }
    }

    const bearingDeg = normalizeAngleDeg(
      this.filter[1] * this.config.bearingSign * this.config.bearingScale + this.config.bearingOffsetDeg
    );
    const bearingRad = bearingDeg * DEG_TO_RAD;
    // Training uses FLU body coordinates: right-positive bearing means negative body Y.
    const relativeBody = {
      x: this.filter[0] * Math.cos(bearingRad),
      y: -this.filter[0] * Math.sin(bearingRad),
    };
    const yawEnuRad = wrapRad(Math.PI / 2 - preflight.yawDeg * DEG_TO_RAD);
    const relativeEnu = rotate(relativeBody.x, relativeBody.y, yawEnuRad);
    if (this.lastMeasurementAt !== null) {
      const interval = (measurementAt - this.lastMeasurementAt) / 1000;
      if (interval >= 0.03 && interval <= 0.6) {
        const velocity = telemetry.velocity || {};
        const measured = {
          x: (relativeEnu.x - this.lastRelativeEnu.x) / interval + finite(velocity.vy, 0),
          y: (relativeEnu.y - this.lastRelativeEnu.y) / interval + finite(velocity.vx, 0),
        };
        const alpha = this.policyConfig.targetVelocityAlpha;
        this.targetVelocityEnu.x = (1 - alpha) * this.targetVelocityEnu.x + alpha * measured.x;
        this.targetVelocityEnu.y = (1 - alpha) * this.targetVelocityEnu.y + alpha * measured.y;
      } else if (interval > 0.6) {
        this.targetVelocityEnu = { x: 0, y: 0 };
      }
    }
    this.lastRelativeEnu = relativeEnu;
    this.lastMeasurementAt = measurementAt;
  }

  buildObservation(now, telemetry, preflight, targetEstimate = null) {
    if (!targetEstimate) this.ingestMeasurement(telemetry, preflight);
    const yawEnuRad = wrapRad(Math.PI / 2 - preflight.yawDeg * DEG_TO_RAD);
    const bearingDeg = normalizeAngleDeg(
      this.filter[1] * this.config.bearingSign * this.config.bearingScale + this.config.bearingOffsetDeg
    );
    const targetVelocityBody = rotate(this.targetVelocityEnu.x, this.targetVelocityEnu.y, -yawEnuRad);
    const boatVelocityBody = rotate(finite(preflight.boatVy, 0), finite(preflight.boatVx, 0), -yawEnuRad);
    const relativeBody = rotate(this.lastRelativeEnu.x, this.lastRelativeEnu.y, -yawEnuRad);
    const ageSeconds = Math.max(0, (now - (targetEstimate?.lastMeasuredAt ?? this.lastMeasurementAt)) / 1000);
    const timeoutSeconds = this.config.uwbTimeoutMs / 1000;
    const heading = this.previousCommandHeadingRad ?? (preflight.yawDeg * DEG_TO_RAD);
    return [
      this.filter[0] / 10,
      Math.sin(bearingDeg * DEG_TO_RAD),
      Math.cos(bearingDeg * DEG_TO_RAD),
      Math.sin(this.filter[2] * DEG_TO_RAD),
      Math.min(ageSeconds, 2) / timeoutSeconds,
      targetEstimate?.source === 'predicted' ? 0 : 1,
      relativeBody.x / 10,
      relativeBody.y / 10,
      targetVelocityBody.x,
      targetVelocityBody.y,
      boatVelocityBody.x,
      boatVelocityBody.y,
      Math.sin(yawEnuRad),
      Math.cos(yawEnuRad),
      -preflight.measuredYawRateDegS * DEG_TO_RAD,
      this.previousCommandSpeedMps / this.policyConfig.maxSpeedMps,
      Math.sin(heading),
      Math.cos(heading),
      this.config.followingDistanceM / 10,
      this.neuralSafetyLatched ? 1 : 0,
    ];
  }

  updateRelativeOnly({now,telemetry,preflight,estimate}) {
    if(String(telemetry.flightMode||'').toUpperCase()!=='MANUAL')return {...preflight,safe:false,reason:'manual_required'};
    if(telemetry.armed!==true)return {...preflight,safe:false,reason:'armed_required'};
    const bearing=normalizeAngleDeg(preflight.azimuthDeg*this.config.bearingSign*this.config.bearingScale+this.config.bearingOffsetDeg)*DEG_TO_RAD;
    const yawEnu=wrapRad(Math.PI/2-preflight.yawDeg*DEG_TO_RAD);
    const body={x:preflight.distanceM*Math.cos(bearing),y:-preflight.distanceM*Math.sin(bearing)};
    const relative=rotate(body.x,body.y,yawEnu),at=finite(telemetry.uwb.measurementUpdatedAt,estimate?.lastMeasuredAt??this.lastMeasurementAt??now);
    const real=!estimate||['measured','recovering'].includes(estimate.source);
    if(real && Number.isFinite(at) && (this.lastMeasurementAt===null||at>this.lastMeasurementAt)){
      const dt=this.lastMeasurementAt===null?0:(at-this.lastMeasurementAt)/1000;
      if(dt>=.03 && dt<=.45){const alpha=this.policyConfig.targetVelocityAlpha;
        this.targetVelocityEnu.x=(1-alpha)*this.targetVelocityEnu.x+alpha*(relative.x-this.lastRelativeEnu.x)/dt;
        this.targetVelocityEnu.y=(1-alpha)*this.targetVelocityEnu.y+alpha*(relative.y-this.lastRelativeEnu.y)/dt;
      }else this.targetVelocityEnu={x:0,y:0};
      this.lastRelativeEnu=relative;this.lastMeasurementAt=at;
    }
    const velocity=rotate(this.targetVelocityEnu.x,this.targetVelocityEnu.y,-yawEnu);
    const previous=this.previousCommandHeadingRad??preflight.yawDeg*DEG_TO_RAD;
    // Explicit missing-input experiment: the original actor expects two
    // absolute velocities. Zero own velocity is an imputation, not a sensor
    // observation; target slots contain relative motion in a co-moving gauge.
    const observation=[preflight.distanceM/10,Math.sin(bearing),Math.cos(bearing),0,
      Math.min(2,Math.max(0,(now-at)/1000))/(this.config.uwbTimeoutMs/1000),estimate?.source==='predicted'?0:1,
      body.x/10,body.y/10,velocity.x,velocity.y,0,0,Math.sin(yawEnu),Math.cos(yawEnu),
      -preflight.measuredYawRateDegS*DEG_TO_RAD,this.previousCommandSpeedMps/this.policyConfig.maxSpeedMps,
      Math.sin(previous),Math.cos(previous),this.config.followingDistanceM/10,preflight.distanceM<=this.config.safetyDistanceM?1:0];
    const action=this.actor.infer(observation);
    const nominalSpeed=(action[0]+1)*.5*this.policyConfig.maxSpeedMps;
    const heading=wrapRad(preflight.yawDeg*DEG_TO_RAD-Math.PI*action[1]);
    const error=normalizeAngleDeg(heading*RAD_TO_DEG-preflight.yawDeg);
    const torque=clamp(.5*(-error*DEG_TO_RAD)-.25*(-preflight.measuredYawRateDegS*DEG_TO_RAD),-.5,.5);
    const result=this.relativeImuControl.update({now,preflight,telemetry,estimate,guidance:this.traditionalFastGuidance,
      policy:{nominalSpeedMps:nominalSpeed,nominalThrottlePercent:Math.min(this.config.maxSpeedMps,nominalSpeed)/this.config.maxSpeedMps*this.config.relativeThrottleLimitPercent,
        headingErrorDeg:error,yawRateDegS:-torque/.5*this.config.maxYawRateDegS}});
    this.previousCommandSpeedMps=nominalSpeed;this.previousCommandHeadingRad=heading;
    return {...result,trackingMethod:'neural',controllerProfile:'ppo_missing_absolute_velocity_experiment',
      neuralModel:path.basename(this.actor.modelPath),neuralAction:action,neuralObservation:observation,
      neuralAbsoluteVelocityMissing:true,neuralObservationAdaptation:'own absolute velocity imputed zero; target slots use relative velocity proxy',
        neuralRole:result.planner==='nmpc'?'nominal_throttle_reference':'direct_relative_guidance',
        speedFeedForwardSource:result.planner==='nmpc'?result.speedFeedForwardSource
          :result.gpsSpeedControlActive?'gps_speed_pi':'experimental_ppo_nominal_throttle'};
  }

  update({ now = Date.now(), telemetry = {}, connected = false, controlMode = 'auto', targetEstimate = null, targetMotionEstimate = null } = {}) {
    const preflight = this.preflight({ now, telemetry, connected, controlMode, targetEstimate });
    if (!preflight.safe) return { ...preflight, trackingMethod: 'neural' };
    if(this.config.executionMode==='relative_imu')return this.updateRelativeOnly({now,telemetry,preflight,estimate:targetMotionEstimate||targetEstimate});
    if (targetEstimate) {
      this.filter = [preflight.distanceM, preflight.azimuthDeg, finite(telemetry.uwb?.elevationDeg, 0)];
      const rightBody = targetEstimate.relativeBodyM;
      const yawEnu = Math.PI / 2 - preflight.yawDeg * DEG_TO_RAD;
      this.lastRelativeEnu = rotate(rightBody.x, -rightBody.y, yawEnu);
      this.targetVelocityEnu = { x: targetEstimate.targetVelocityNedMps.y, y: targetEstimate.targetVelocityNedMps.x };
    } else {
      this.ingestMeasurement(telemetry, preflight);
    }
    const distanceM = this.filter[0];
    // The actor sees the filtered range, while the collision stop uses the
    // newest raw range so filtering can never delay a stop command.
    if (preflight.distanceM <= this.policyConfig.stopDistanceM) this.neuralSafetyLatched = true;
    if (this.neuralSafetyLatched && preflight.distanceM >= this.policyConfig.resumeDistanceM) this.neuralSafetyLatched = false;

    const bearingDeg = normalizeAngleDeg(
      this.filter[1] * this.config.bearingSign * this.config.bearingScale + this.config.bearingOffsetDeg
    );
    const relativeBodyM = rotate(this.lastRelativeEnu.x, this.lastRelativeEnu.y,
      -(Math.PI / 2 - preflight.yawDeg * DEG_TO_RAD));
    const effectiveSpeedLimit = Math.min(
      this.policyConfig.maxSpeedMps,
      this.config.commissioningMode ? this.config.commissioningSpeedMps : this.policyConfig.maxSpeedMps
    );
    if (this.neuralSafetyLatched) {
      this.neuralThrottlePercent = 0;
      this.neuralSteeringDeg = 0;
      this.previousCommandSpeedMps = 0;
      return {
        safe: true,
        status: 'holding',
        reason: 'holding_distance',
        trackingMethod: 'neural',
        controllerProfile: 'smart_usv_ppo',
        controlMode: preflight.controlMode,
        requiredFlightMode: preflight.requiredFlightMode,
        navigationAvailable: preflight.navigationAvailable,
        navigationReason: preflight.navigationReason,
        commandFrame: 'BODY_RELATIVE',
        relativeBodyM,
        distanceM,
        bearingDeg,
        controlBearingDeg: bearingDeg,
        forwardSpeedMps: 0,
        desiredForwardSpeedMps: 0,
        yawRateDegS: 0,
        yawRateRadS: 0,
        throttlePercent: 0,
        steeringDegrees: 0,
        velocityNed: { x: 0, y: 0 },
        desiredSpeedMps: 0,
        desiredHeadingDeg: this.previousCommandHeadingRad === null ? preflight.yawDeg : this.previousCommandHeadingRad * RAD_TO_DEG,
        speedLimitMps: effectiveSpeedLimit,
        commissioningMode: this.config.commissioningMode,
        holdingDistanceM: this.policyConfig.stopDistanceM,
        hardStopResumeDistanceM: this.policyConfig.resumeDistanceM,
        trackingPhase: 'neural_safety_stop',
      };
    }
    if (String(telemetry.flightMode || '').toUpperCase() !== preflight.requiredFlightMode) {
      return { ...preflight, safe: false, reason: preflight.requiredFlightMode === 'ACRO' ? 'acro_required' : 'manual_required', trackingMethod: 'neural' };
    }
    if (telemetry.armed !== true) return { ...preflight, safe: false, reason: 'armed_required', trackingMethod: 'neural' };

    const observation = this.buildObservation(now, telemetry, preflight, targetEstimate);
    const action = this.actor.infer(observation);
    const desiredSpeedMps = (action[0] + 1) * 0.5 * this.policyConfig.maxSpeedMps;
    const yawEnuRad = wrapRad(Math.PI / 2 - preflight.yawDeg * DEG_TO_RAD);
    const desiredYawEnuRad = wrapRad(yawEnuRad + Math.PI * action[1]);
    const desiredHeadingRad = wrapRad(Math.PI / 2 - desiredYawEnuRad);
    const desiredHeadingDeg = ((desiredHeadingRad * RAD_TO_DEG) % 360 + 360) % 360;
    const headingErrorDeg = normalizeAngleDeg(desiredHeadingDeg - preflight.yawDeg);
    const headingErrorRad = headingErrorDeg * DEG_TO_RAD;
    const forwardSpeedMps = Math.min(effectiveSpeedLimit, desiredSpeedMps) * Math.max(0, Math.cos(headingErrorRad));
    // Same heading servo used in training; map its requested yaw to a virtual stick.
    const trainingTorque = clamp(0.5 * (-headingErrorRad) - 0.25 * (-preflight.measuredYawRateDegS * DEG_TO_RAD), -0.5, 0.5);
    const desiredYawRateDegS = -trainingTorque / 0.5 * this.config.maxYawRateDegS;
    const desiredThrottle = forwardSpeedMps / Math.max(effectiveSpeedLimit, 0.1) * this.config.relativeThrottleLimitPercent;
    const desiredSteering = desiredYawRateDegS / this.config.maxYawRateDegS * this.config.relativeSteeringLimitDeg;
    // Ramping, deadband, turn hysteresis and reversal dwell belong to the same
    // per-propeller limiter used by the joystick, not a second yaw/throttle ramp.
    this.neuralThrottlePercent = desiredThrottle;
    this.neuralSteeringDeg = desiredSteering;
    const commandedForwardSpeedMps = this.neuralThrottlePercent / this.config.relativeThrottleLimitPercent * effectiveSpeedLimit;
    const yawRateDegS = this.neuralSteeringDeg / this.config.relativeSteeringLimitDeg * this.config.maxYawRateDegS;
    const headingRad = preflight.yawDeg * DEG_TO_RAD;
    this.previousCommandSpeedMps = desiredSpeedMps;
    this.previousCommandHeadingRad = desiredHeadingRad;

    return {
      safe: true,
      reason: 'tracking',
      trackingMethod: 'neural',
      controllerProfile: 'smart_usv_ppo',
      neuralModel: path.basename(this.actor.modelPath),
      neuralAction: action,
      neuralObservation: observation,
      controlMode: preflight.controlMode,
      requiredFlightMode: preflight.requiredFlightMode,
      navigationAvailable: preflight.navigationAvailable,
      navigationReason: preflight.navigationReason,
      commandFrame: 'BODY_RELATIVE',
      relativeBodyM,
      distanceM,
      bearingDeg,
      controlBearingDeg: bearingDeg,
      guidanceBearingDeg: headingErrorDeg,
      headingErrorDeg,
      forwardSpeedMps: commandedForwardSpeedMps,
      desiredForwardSpeedMps: forwardSpeedMps,
      commandedForwardSpeedMps,
      desiredYawRateDegS,
      yawRateDegS,
      yawRateRadS: yawRateDegS * DEG_TO_RAD,
      throttlePercent: this.neuralThrottlePercent,
      steeringDegrees: this.neuralSteeringDeg,
      velocityNed: {
        x: commandedForwardSpeedMps * Math.cos(headingRad),
        y: commandedForwardSpeedMps * Math.sin(headingRad),
      },
      desiredSpeedMps: this.config.executionMode === 'acro' ? Math.min(effectiveSpeedLimit, desiredSpeedMps) : desiredSpeedMps,
      policyDesiredSpeedMps: desiredSpeedMps,
      desiredHeadingDeg,
      speedLimitMps: effectiveSpeedLimit,
      commissioningMode: this.config.commissioningMode,
      holdingDistanceM: this.policyConfig.stopDistanceM,
      hardStopResumeDistanceM: this.policyConfig.resumeDistanceM,
      trackingPhase: 'neural_policy',
    };
  }
}
