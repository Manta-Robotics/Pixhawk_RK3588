// GPS-free relative following. Only UWB polar observations and IMU attitude
// enter this controller; learned cruise thrust is NOT an absolute speed.
const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
const wrap=x=>((x+180)%360+360)%360-180;
export class RelativeImuControl {
 constructor(config){this.c=config;this.reset();}
 reset(){this.at=null;this.measurementAt=null;this.distance=null;this.rangeRate=0;this.cruise=0;this.throttle=0;this.steering=0;this.bearing=null;}
 pause(){const cruise=this.cruise;this.reset();this.cruise=cruise;}
 update({now,preflight,telemetry,estimate,guidance,policy=null}){
  const c=this.c,d=preflight.distanceM;
  const dt=this.at===null?.1:clamp((now-this.at)/1000,.001,.25);
  const bearing=wrap(preflight.azimuthDeg*c.bearingSign*c.bearingScale+c.bearingOffsetDeg);
  this.bearing=this.bearing===null?bearing:wrap(this.bearing+c.bearingFilterAlpha*wrap(bearing-this.bearing));
  const sampleAt=telemetry.uwb.measurementUpdatedAt;
  const observed=!estimate || ['measured','recovering'].includes(estimate.source);
  if(observed && Number.isFinite(sampleAt) && (this.measurementAt===null || sampleAt>this.measurementAt)){
   if(this.distance!==null && this.measurementAt!==null && sampleAt-this.measurementAt<=c.uwbTimeoutMs){
    const sampleDt=(sampleAt-this.measurementAt)/1000;
    const measured=clamp((d-this.distance)/sampleDt,-3,3);
    const alpha=1-Math.pow(1-c.targetSpeedAlpha,sampleDt/.1);
    this.rangeRate+=alpha*(measured-this.rangeRate);
   }
   this.distance=d;this.measurementAt=sampleAt;
  }
  // Relative velocity responds to OUR acceleration too. A long target-motion
  // window is useful for prediction but introduces destabilizing feedback lag.
  this.at=now;
  const error=d-c.followingDistanceM;
  const controlled=Math.abs(error)<=c.rangeDeadbandM?0:error;
  const progress=clamp((d-c.distanceHoldUpperM)/Math.max(.01,c.safetyResumeDistanceM-c.distanceHoldUpperM),0,1);
  const approachLimit=c.closeApproachMaxSpeedMps*progress+c.rangeGain*Math.max(0,d-c.safetyResumeDistanceM);
  const deceleration=Math.max(.05,c.prediction.braking_deceleration_mps2??.3);
  const age=Math.max(0,(now-telemetry.uwb.measurementUpdatedAt)/1000);
  const reaction=.6+Math.min(age,c.uwbTimeoutMs/1000);
  const margin=Math.max(.05,c.prediction.measurement_noise_m??.12);
  const clearance=Math.max(0,d-c.followingDistanceM-margin);
  const brakingLimit=Math.max(0,Math.sqrt((deceleration*reaction)**2+2*deceleration*clearance)-deceleration*reaction);
  const desiredClosing=clamp(c.rangeGain*controlled,-c.maxReverseSpeedMps,Math.min(approachLimit,.5*brakingLimit,c.maxSpeedMps));
  const rateError=desiredClosing+this.rangeRate;
  // PI on relative velocity learns thrust needed to match the other boat,
  // including a steady current load, without inventing either boat's speed.
  const nominalScale=c.relativeThrottleLimitPercent/c.maxSpeedMps;
  const kp=2*nominalScale,ki=.4*nominalScale;
  const previousCruise=this.cruise;
  this.cruise=clamp(this.cruise+ki*rateError*dt,0,c.relativeThrottleLimitPercent);
  let requested=this.cruise+kp*rateError;
  if((requested>c.relativeThrottleLimitPercent && rateError>0)||(requested<0 && rateError<0))this.cruise=previousCruise;
  requested=clamp(this.cruise+kp*rateError,0,c.relativeThrottleLimitPercent);
  if(policy)requested=clamp(policy.nominalThrottlePercent,0,c.relativeThrottleLimitPercent);
  const closing=Math.max(0,-this.rangeRate);
  const braking=closing>brakingLimit+.02;
  if(braking)requested=Math.min(requested,Math.max(0,this.throttle-c.throttleDecelPercentS*dt-kp*(closing-brakingLimit)));
  const turn=policy?{speedScale:Math.max(0,Math.cos(policy.headingErrorDeg*Math.PI/180)),
   desiredYawRateDegS:clamp(policy.yawRateDegS,-c.maxYawRateDegS,c.maxYawRateDegS)}
   :guidance.command({headingErrorDeg:this.bearing,measuredYawRateDegS:preflight.measuredYawRateDegS,
    forwardSpeedMps:0,maximumYawRateDegS:c.maxYawRateDegS});
  const alignment=clamp((c.bearingStopDeg-Math.abs(this.bearing))/(c.bearingStopDeg-c.bearingSlowdownDeg),0,1);
  requested*=turn.speedScale*alignment;
  const safety=d<=c.safetyDistanceM;
  if(safety){requested=0;this.cruise=Math.min(this.cruise,this.throttle);}
  const step=(requested<this.throttle?c.throttleDecelPercentS:c.throttleAccelPercentS)*dt;
  this.throttle+=clamp(requested-this.throttle,-step,step);
  if(safety || braking || alignment===0)this.throttle=Math.min(this.throttle,requested);
  // Track the applied actuator, including slew/braking/turn saturation. A
  // cruise-thrust integrator must not wind up behind a constrained output.
  this.cruise=policy?0:clamp(this.cruise+(this.throttle-(this.cruise+kp*rateError))*dt,0,c.relativeThrottleLimitPercent);
  const steering=clamp(turn.desiredYawRateDegS/c.maxYawRateDegS*c.relativeSteeringLimitDeg,-c.relativeSteeringLimitDeg,c.relativeSteeringLimitDeg);
  this.steering+=clamp(steering-this.steering,-c.steeringSlewDegS*dt,c.steeringSlewDegS*dt);
  if(safety)this.steering=0;
  return {safe:true,reason:safety?'holding_distance':'tracking',status:safety?'holding':'tracking',
   controlMode:'relative_manual',requiredFlightMode:'MANUAL',commandFrame:'BODY_RELATIVE',
   relativeImuControl:true,navigationAvailable:false,navigationReason:'uwb_imu_only',
   absoluteSpeedObservable:false,forwardSpeedMps:null,desiredSpeedMps:null,velocityNed:null,
   targetRadialSpeedMps:null,speedFeedForwardSource:'learned_cruise_thrust',
   distanceM:d,bearingDeg:bearing,controlBearingDeg:this.bearing,rangeErrorM:error,rangeRateMps:this.rangeRate,
   desiredClosingSpeedMps:desiredClosing,relativeApproachLimitMps:approachLimit,brakingClosingLimitMps:brakingLimit,
   brakingActive:braking,learnedCruiseThrottlePercent:policy?null:this.cruise,desiredThrottlePercent:requested,
   throttlePercent:this.throttle,steeringDegrees:this.steering,
   holdingDistanceM:c.safetyDistanceM,hardStopResumeDistanceM:c.hardStopResumeDistanceM,
   yawRateDegS:turn.desiredYawRateDegS,yawRateRadS:turn.desiredYawRateDegS*Math.PI/180,
   relativeBodyM:{x:d*Math.cos(bearing*Math.PI/180),y:d*Math.sin(bearing*Math.PI/180)}};
 }
}
