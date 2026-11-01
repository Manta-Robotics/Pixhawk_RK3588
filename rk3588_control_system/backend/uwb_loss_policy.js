// Keep an already armed relative-follow session alive through missing UWB.
// Missing coordinates authorize neutral output only, never a held drive vector.
export function holdForMissingUwb({enabled=false,result,telemetry={},connected=false,
  now=Date.now(),controlMode='auto',imuTimeoutMs=700}={}) {
  if(!enabled||result?.safe!==false||result.reason!=='uwb_stale')return result;
  const fault=reason=>({...result,safe:false,reason,positionWaiting:false});
  if(!connected)return fault('pixhawk_offline');
  if(telemetry.armed!==true)return fault('armed_required');
  if(controlMode!=='relative_manual'||String(telemetry.flightMode).toUpperCase()!=='MANUAL')
    return fault(controlMode==='relative_manual'?'manual_required':'uwb_stale');
  if(telemetry.imuCalibration?.active)return fault('imu_calibration_active');
  const a=telemetry.attitude||{};
  if(typeof a.yaw!=='number'||!Number.isFinite(a.yaw))return fault('imu_invalid');
  if(typeof a.updatedAt!=='number'||!Number.isFinite(a.updatedAt)||a.updatedAt>now
    ||now-a.updatedAt>imuTimeoutMs)return fault('imu_stale');
  if(telemetry.pilotSteering?.ready!==true||![0,1,2,3].includes(telemetry.pilotSteering.type))
    return fault('pilot_steering_not_ready');
  return {...result,safe:true,status:'waiting_target',reason:'uwb_waiting_position',
    positionWaiting:true,pausePropulsion:true,relativeImuControl:true,
    controlMode:'relative_manual',requiredFlightMode:'MANUAL',commandFrame:'BODY_RELATIVE',
    throttlePercent:0,steeringDegrees:0,forwardSpeedMps:0,desiredSpeedMps:0,
    yawRateDegS:0,yawRateRadS:0,velocityNed:null,relativeBodyM:null,
    distanceM:null,bearingDeg:null,leftPwm:1500,rightPwm:1500};
}

const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
const wrap=x=>((x+180)%360+360)%360-180;
const rad=Math.PI/180;
const finite=x=>typeof x==='number'&&Number.isFinite(x);
const value=(x,fallback)=>finite(x)?x:fallback;
const identity=u=>JSON.stringify([u.streamId,(u.sensors||[]).map(s=>[s.receiverId,s.anchorId,s.tagId])]);

// This is bounded dead reckoning from applied thrust, not a position sensor.
// It must never renew UWB timestamps or turn model estimates into measurements.
export class LastUwbPointApproach {
  constructor(config={}) {this.c=config;this.reset();}
  reset(){this.at=null;this.yaw=null;this.speed=0;this.x=0;this.y=0;this.travel=0;this.last=null;this.lossThrottle=null;}
  update(input={}) {
    const {enabled=false,result,telemetry={},connected=false,now=Date.now(),controlMode='auto',
      imuTimeoutMs=700,appliedCommand={},calibration={}}=input;
    if(!enabled){this.reset();return result;}
    const guard=holdForMissingUwb({...input,result:{safe:false,reason:'uwb_stale'}});
    if(!guard.safe){this.reset();return guard;}
    const c=this.c,p=c.prediction||{},model=c.relative_nmpc||{};
    const yaw=telemetry.attitude.yaw;
    const powerLimit=Math.max(1,value(c.relative_throttle_limit_percent,70));
    const cruise=Math.max(.01,value(model.model_cruise_speed_mps,.8));
    const exponent=Math.max(.1,value(model.surge_map_exponent,.5));
    const tau=Math.max(.1,value(model.surge_time_constant_s,1));
    const throttle=clamp(value(appliedCommand.throttle,0),0,powerLimit);
    const dt=this.at===null?0:(now-this.at)/1000;
    if(dt<0||dt*1000>imuTimeoutMs){
      this.reset();
      if(!result.safe)return {...guard,lastPointApproach:{active:false,reason:'model_clock_gap',estimated:true}};
    } else if(dt>0) {
      const desired=cruise*Math.pow(throttle/powerLimit,exponent);
      const decay=Math.exp(-dt/tau);
      const distance=desired*dt+(this.speed-desired)*tau*(1-decay);
      const heading=(this.yaw+wrap(yaw-this.yaw)/2)*rad;
      this.x+=distance*Math.cos(heading);this.y+=distance*Math.sin(heading);this.travel+=distance;
      this.speed=desired+(this.speed-desired)*decay;
    }
    this.at=now;this.yaw=yaw;
    const u=telemetry.uwb||{},stamp=u.measurementUpdatedAt;
    const fresh=result.safe&&u.online===true&&u.fresh===true
      &&u.measurementStatus==='trusted'&&finite(stamp)&&stamp<=now
      &&now-stamp<=value(c.uwb_timeout_ms,450)&&finite(u.distanceM)&&u.distanceM>0&&finite(u.azimuthDeg);
    if(fresh) {
      const id=identity(u);
      if(!this.last||this.last.at!==stamp||this.last.id!==id){
        const bearing=wrap(u.azimuthDeg*value(calibration.bearingSign,-1)*value(calibration.bearingScale,.8148)
          +value(calibration.bearingOffsetDeg,-4.816));
        const age=(now-stamp)/1000;
        const measuredX=this.x-this.speed*age*Math.cos(yaw*rad);
        const measuredY=this.y-this.speed*age*Math.sin(yaw*rad);
        this.last={id,at:stamp,x:measuredX+u.distanceM*Math.cos((yaw+bearing)*rad),
          y:measuredY+u.distanceM*Math.sin((yaw+bearing)*rad),travel:this.travel-this.speed*age};
      }
      this.lossThrottle=null;
      return result;
    }
    if(result.safe||result.reason!=='uwb_stale')return result;
    const wait=reason=>({...guard,lastPointApproach:{active:false,reason,estimated:true,lastMeasuredAt:this.last?.at??null}});
    if(u.transport?.connected===false)return {...result,reason:'uwb_transport_offline'};
    if(['distance_out_of_range','invalid_measurement','invalid_worker_measurement','stale_worker_delivery',
      'uwb_worker_error','uwb_worker_exited'].includes(u.rejectionReason))return wait('invalid_current_measurement');
    if(!this.last)return wait('no_last_position');
    if(identity(u)!==this.last.id){this.reset();return {...result,reason:'target_identity_changed'};}
    const ageMs=now-this.last.at;
    const horizonMs=clamp(value(p.horizon_ms,1000),100,2000);
    if(ageMs<0||ageMs>horizonMs)return wait('model_time_limit');
    const maxSpeed=clamp(value(p.max_speed_mps,.35),.01,.35);
    if(this.travel-this.last.travel>maxSpeed*horizonMs/1000)return wait('model_distance_limit');
    const dx=this.last.x-this.x,dy=this.last.y-this.y;
    const distance=Math.hypot(dx,dy),standOff=Math.max(.1,value(c.following_distance_m,.5));
    if(distance<=standOff+.05)return wait('estimated_arrival');
    const bearing=wrap(Math.atan2(dy,dx)/rad-yaw);
    if(Math.abs(bearing)>45)return wait('last_position_outside_forward_sector');
    if(this.lossThrottle===null)this.lossThrottle=throttle;
    const desiredSpeed=Math.min(maxSpeed,Math.sqrt(2*.3*Math.max(0,distance-standOff-.05)));
    const modelThrottle=powerLimit*Math.pow(desiredSpeed/cruise,1/exponent);
    const alignment=clamp((45-Math.abs(bearing))/25,0,1);
    const targetThrottle=Math.min(this.lossThrottle,clamp(value(p.max_throttle_percent,15),0,15),modelThrottle)*alignment;
    const maxYaw=Math.max(1,value(c.max_yaw_rate_deg_s,14));
    const yawRate=clamp(value(c.traditional_heading_gain,.85)*bearing
      -value(c.traditional_yaw_rate_damping,.6)*value(telemetry.attitude.yawRateDegS,0),-maxYaw,maxYaw);
    const steeringCap=Math.min(6,value(p.max_steering_deg,6));
    const desiredSteering=clamp(yawRate/maxYaw*value(c.relative_steering_limit_deg,18),-steeringCap,steeringCap);
    const steering=value(appliedCommand.steering,0)+clamp(desiredSteering-value(appliedCommand.steering,0),
      -value(c.steering_slew_deg_s,9)*Math.max(0,dt),value(c.steering_slew_deg_s,9)*Math.max(0,dt));
    const body={x:dx*Math.cos(yaw*rad)+dy*Math.sin(yaw*rad),y:-dx*Math.sin(yaw*rad)+dy*Math.cos(yaw*rad)};
    return {...guard,status:'predicting',reason:'last_point_approach',positionWaiting:false,pausePropulsion:false,
      throttlePercent:targetThrottle,steeringDegrees:clamp(steering,-steeringCap,steeringCap),
      relativeBodyM:body,distanceM:distance,bearingDeg:bearing,yawRateDegS:yawRate,yawRateRadS:yawRate*rad,
      forwardSpeedMps:null,desiredSpeedMps:null,absoluteSpeedObservable:false,estimatedForwardSpeedMps:this.speed,
      lastPointApproach:{active:true,estimated:true,uncalibrated:true,lastMeasuredAt:this.last.at,ageMs,
        horizonMs,standOffM:standOff,estimatedRemainingM:Math.max(0,distance-standOff)},
      targetEstimate:{source:'predicted',usable:true,modelBased:true,relativeBodyM:body,
        lastMeasuredAt:this.last.at,predictionAgeMs:ageMs,measurementConfirmed:false}};
  }
}
