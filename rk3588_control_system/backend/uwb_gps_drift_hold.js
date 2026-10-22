// GPS/IMU waiting assistance, with independently confirmed live UWB heading.
// No simulator truth, flight-controller parameter or motor PWM access.
import {UwbRejectNmpc} from './uwb_reject_nmpc.js';
const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
const wrap=x=>((x+180)%360+360)%360-180;
const rad=Math.PI/180;
const finite=x=>typeof x==='number'&&Number.isFinite(x);
const fresh=(at,now,limit)=>finite(at)&&at>0&&at<=now&&now-at<=limit;
export class UwbGpsDriftHold {
 constructor(config={}){this.c=config;this.enabled=config.waiting_drift_hold===true;this.reset();}
 reset(){this.anchor=null;this.lastTracking=null;this.bearings=[];this.rangeCertificate=null;this.creepSpeed=0;this.creepAt=null;this.lastProgressAt=null;this.targetId=null;this.farPursuit=false;this.nmpc=new UwbRejectNmpc(this.c);}
 track(now,telemetry,result){
  this.nmpc.observe(now,telemetry,result);
  this.anchor=null;this.farPursuit=false;
  if(result.safe&&finite(result.bearingDeg)&&finite(telemetry.attitude?.yaw))
   this.lastTracking={at:now,heading:wrap(telemetry.attitude.yaw+result.bearingDeg),range:result.distanceM};
  const psi=(telemetry.attitude?.yaw||0)*rad,velocity=telemetry.velocity||{};
  const ownForward=finite(velocity.vx)&&finite(velocity.vy)?velocity.vx*Math.cos(psi)+velocity.vy*Math.sin(psi):0;
  this.creepSpeed=clamp(ownForward,0,Math.min(this.c.prediction?.max_speed_mps??.35,this.c.max_speed_mps??2));
  this.creepAt=now;this.lastProgressAt=now;
  const sensor=telemetry.uwb?.sensors?.[0];
  if(sensor)this.targetId=`${sensor.receiverId}:${sensor.tagId??''}`;
  const e=result.targetEstimate;
  if(e?.measurementConfirmed&&finite(result.distanceM)&&finite(e.uncertaintyM)
    &&this.rangeCertificate?.measurementAt!==e.lastMeasuredAt
    &&finite(telemetry.position?.lat)&&finite(telemetry.position?.lon)){
   this.rangeCertificate={at:now,measurementAt:e.lastMeasuredAt,range:result.distanceM,
    uncertainty:e.uncertaintyM,lat:telemetry.position.lat,lon:telemetry.position.lon};
  }
 }
 observeBearing(now,telemetry){
  const c=this.c,p=c.prediction||{},u=telemetry.uwb||{},a=telemetry.attitude||{},gap=p.measurement_gap_ms??450;
  const no=reason=>({trusted:false,reason,hits:0});
  const sensors=(u.sensors||[]).filter(s=>finite(s.receivedMeasurementAt)||finite(s.rawRejectedAt));
  if(sensors.length!==1)return no('one_raw_receiver_required');
  const s=sensors[0],at=Math.max(s.receivedMeasurementAt||0,s.rawRejectedAt||0);
  if(!fresh(at,now,gap)||s.valid===false||at!==s.receivedMeasurementAt)return no('raw_invalid_or_stale');
  // A position-KF innovation rejection is not evidence of a bad bearing.
  // Evaluate raw heading in an IMU-stabilized frame, independently of range.
  const az=finite(s.rawAzimuthDeg)?s.rawAzimuthDeg:u.measurementStatus==='trusted'?u.azimuthDeg:null;
  if(!finite(az)||Math.abs(az)>180)return no('angle_invalid');
  const bearing=wrap((az+(s.mountYawDeg||0))*(c.bearing_sign??-1)*(c.bearing_scale??.8148)+(c.bearing_offset_deg??-4.816));
  if(Math.abs(bearing)>45)return no('outside_fov');
  const heading=wrap(a.yaw+bearing-(finite(a.yawRateDegS)?a.yawRateDegS:0)*(now-at)/1000);
  const id=`${s.receiverId}:${s.tagId??''}`;
  if(this.targetId!==null&&this.targetId!==id){this.bearings=[];this.rangeCertificate=null;return no('identity_changed');}
  this.bearings=this.bearings.filter(x=>now-x.at<=700);
  const previous=this.bearings.at(-1);
  if(previous&&previous.id!==id){this.bearings=[];this.rangeCertificate=null;return no('identity_changed');}
  if(!previous||at>previous.at){
   this.bearings.push({at,id,heading,range:s.rawDistanceM});this.bearings=this.bearings.slice(-7);
  }
  const points=this.bearings;
  if(points.length<3||points.at(-1).at!==at)return no('bearing_warming_up');
  const required=Math.max(3,Math.ceil(points.length*.6)),origin=points[0].heading;
  const fit=ps=>{
   const ts=ps.map(q=>(q.at-at)/1000),mt=ts.reduce((x,y)=>x+y,0)/ps.length;
   const hs=ps.map(q=>origin+wrap(q.heading-origin)),mh=hs.reduce((x,y)=>x+y,0)/ps.length;
   const den=ts.reduce((x,t)=>x+(t-mt)**2,0);
   if(!den)return null;
   const rate=ts.reduce((x,t,i)=>x+(t-mt)*(hs[i]-mh),0)/den,intercept=mh-rate*mt;
   return {rate,heading:wrap(intercept),error:q=>Math.abs(wrap(q.heading-intercept-rate*(q.at-at)/1000))};
  };
  const models=[];
  for(let i=0;i<points.length-2;i++)for(let j=i+1;j<points.length-1;j++)for(let k=j+1;k<points.length;k++){
   const initial=fit([points[i],points[j],points[k]]);if(!initial||Math.abs(initial.rate)>30)continue;
   const inliers=points.filter(q=>initial.error(q)<=3);
   if(inliers.length<required||inliers.at(-1).at!==at||at-inliers[0].at<180)continue;
   const model=fit(inliers);
   if(!model||Math.abs(model.rate)>30||inliers.some(q=>model.error(q)>3))continue;
   models.push({...model,hits:inliers.length});
  }
  models.sort((a,b)=>b.hits-a.hits||a.error(points.at(-1))-b.error(points.at(-1)));
  const model=models[0];if(!model)return no('bearing_inconsistent');
  this.targetId=id;
  return {trusted:true,reason:'raw_heading_confirmed',at,bearingDeg:bearing,
   headingDeg:wrap(model.heading+model.rate*(now-at)/1000),rateDegS:model.rate,hits:model.hits};
 }
 refreshRangeCertificate(now,telemetry){
  const u=telemetry.uwb||{},s=(u.sensors||[])[0],at=u.measurementUpdatedAt,p=this.c.prediction||{};
  if(!s||u.measurementStatus!=='trusted'||s.measurementStatus!=='trusted'||!u.fresh
    ||!fresh(at,now,p.measurement_gap_ms??450)||s.receivedMeasurementAt!==at
    ||!finite(u.distanceM)||!finite(s.filterUncertaintyM)||s.filterUncertaintyM<0
    ||this.rangeCertificate?.measurementAt===at)return;
  const points=this.bearings.filter(q=>finite(q.range)&&q.range>=.15&&q.range<=20&&at-q.at<=500);
  if(points.length<3||points.at(-1).at!==at||at-points[0].at<180)return;
  const ts=points.map(q=>(q.at-at)/1000),mt=ts.reduce((a,b)=>a+b,0)/points.length;
  const mean=points.reduce((sum,q)=>sum+q.range,0)/points.length,den=ts.reduce((sum,t)=>sum+(t-mt)**2,0);
  if(!den)return;
  const slope=ts.reduce((sum,t,i)=>sum+(t-mt)*(points[i].range-mean),0)/den;
  const predicted=mean-slope*mt,gate=2*(p.measurement_noise_m??.12);
  const own=Math.hypot(telemetry.velocity.vx,telemetry.velocity.vy);
  if(Math.abs(slope)>(p.max_target_speed_mps??3)+own
    ||points.some((q,i)=>Math.abs(q.range-predicted-slope*ts[i])>gate)
    ||Math.abs(u.distanceM-predicted)>gate)return;
  const previous=this.rangeCertificate;
  const range=Math.min(u.distanceM,points.at(-1).range);
  if(previous&&Math.abs(range-previous.range)>(p.innovation_gate_m??.6)
    +((p.max_target_speed_mps??3)+own)*(now-previous.at)/1000)return;
  this.rangeCertificate={at:now,measurementAt:at,range,uncertainty:Math.max(p.measurement_noise_m??.12,s.filterUncertaintyM),
   lat:telemetry.position.lat,lon:telemetry.position.lon};
 }
 acceptedPrediction(now,result,forwardVelocity,ownSpeed){
  const e=result.targetEstimate,certificate=this.rangeCertificate,p=this.c.prediction||{};
  // The navigation estimator projects the last admitted UWB posterior using
  // own GPS/IMU. Never seed this path from a raw observation or reset its age.
  if(!certificate||e?.admissionVersion!==1||e.source!=='predicted'
   ||!fresh(e.evaluatedAt,now,100)||e.lastMeasuredAt!==certificate.measurementAt
   ||!fresh(e.lastMeasuredAt,now,Math.min(p.horizon_ms??1000,p.wait_timeout_ms??5000))
   ||!finite(e.relativeBodyM?.x)||!finite(e.relativeBodyM?.y)
   ||!finite(e.uncertaintyM)||e.uncertaintyM<certificate.uncertainty)return null;
  const range=Math.hypot(e.relativeBodyM.x,e.relativeBodyM.y);
  const bearing=Math.atan2(e.relativeBodyM.y,e.relativeBodyM.x)/rad;
  const deceleration=p.braking_deceleration_mps2??.3,closing=Math.max(0,forwardVelocity);
  const stopping=.6*closing+closing*closing/(2*deceleration);
  const schedulingAge=(now-e.evaluatedAt)/1000;
  const uncertainty=e.uncertaintyM+((p.max_target_speed_mps??3)+ownSpeed)*schedulingAge;
  const lower=range-uncertainty;
  const clearance=lower-Math.max(this.c.safety_distance_m??.3,p.clearance_m??1.2)-stopping;
  if(!finite(clearance)||clearance<=0||Math.abs(bearing)>=35)return null;
  return {bearingDeg:bearing,rangeLowerBoundM:lower,clearanceM:clearance,
   stoppingDistanceM:stopping,uncertaintyM:uncertainty,measurementAt:e.lastMeasuredAt,ageMs:now-e.lastMeasuredAt};
 }
 update(now,telemetry,result){
  const c=this.c,p=c.prediction||{},a=telemetry.attitude||{},v=telemetry.velocity||{},pos=telemetry.position||{};
  const zero=reason=>{this.creepSpeed=0;this.creepAt=now;this.lastProgressAt=null;this.farPursuit=false;return {active:false,reason,forwardSpeedMps:0,yawRateDegS:0};};
  if(!this.enabled||!result.pausePropulsion||result.status!=='waiting_target')return zero('not_waiting');
  const ageLimit=c.navigation_timeout_ms??700;
  if(!telemetry.armed||String(telemetry.flightMode).toUpperCase()!=='ACRO'||Number(telemetry.gps?.fixType||0)<3
    ||!telemetry.ekf?.healthy||!fresh(telemetry.ekf?.updatedAt,now,ageLimit)
    ||!fresh(pos.updatedAt,now,ageLimit)||!finite(pos.lat)||!finite(pos.lon)
    ||!fresh(a.updatedAt,now,ageLimit)||!finite(a.yaw)||!finite(v.vx)||!finite(v.vy)
    ||!telemetry.acroControl?.ready||!fresh(telemetry.acroControl?.updatedAt,now,ageLimit))return zero('hold_navigation_unavailable');
  const uwb=telemetry.uwb||{};
  if(uwb.fresh&&finite(uwb.distanceM)&&uwb.distanceM>0&&uwb.distanceM<=(c.safety_distance_m??.3))return zero('safety_distance');
  if(this.lastTracking&&finite(this.lastTracking.range)&&this.lastTracking.range<=(c.safety_distance_m??.3)+(p.measurement_noise_m??.12))return zero('last_clearance_too_small');
  if(!this.anchor)this.anchor={lat:pos.lat,lon:pos.lon,heading:this.lastTracking&&now-this.lastTracking.at<=(p.horizon_ms??1000)
    ?this.lastTracking.heading:a.yaw,enteredAt:now};
  // The existing five-second loss timer remains authoritative in the wrapper.
  // This additional bound prevents a direct caller from extending assistance.
  if(now-this.anchor.enteredAt>=(p.wait_timeout_ms??5000))return zero('hold_timeout');
  const north=(pos.lat-this.anchor.lat)*rad*6378137;
  const east=wrap(pos.lon-this.anchor.lon)*rad*6378137*Math.cos(this.anchor.lat*rad);
  const yaw=a.yaw*rad;
  const forwardDisplacement=north*Math.cos(yaw)+east*Math.sin(yaw);
  const crossDisplacement=-north*Math.sin(yaw)+east*Math.cos(yaw);
  const forwardVelocity=v.vx*Math.cos(yaw)+v.vy*Math.sin(yaw);
  const maximum=Math.max(0,Math.min(p.max_speed_mps??.35,c.max_speed_mps??2));
  const reverse=Math.max(0,Math.min(maximum,c.max_reverse_speed_mps??.2));
  let speed=clamp(-.6*forwardDisplacement-.25*forwardVelocity,-reverse,maximum);
  // Exactly zero selects Rover's stop controller, which can coast. A tiny
  // retreat-biased reference keeps the normal speed loop active. The existing
  // calibrated RC encoder compensates dead zones without changing parameters.
  const bias=Math.min(.03,maximum,reverse);
  if(Math.abs(speed)<bias)speed=-bias;
  if(forwardDisplacement>.08)speed=Math.min(speed,-bias);
  let yawLimit=Math.min(c.max_yaw_rate_deg_s??14,
    (c.max_yaw_rate_deg_s??14)*(p.max_steering_deg??6)/Math.max(1,c.relative_steering_limit_deg??18));
  const direction=this.observeBearing(now,telemetry),liveBearing=direction.trusted;
  if(liveBearing){this.anchor.heading=direction.headingDeg;yawLimit=c.max_yaw_rate_deg_s??14;}
  this.refreshRangeCertificate(now,telemetry);
  const certificate=this.rangeCertificate;
  let rangeLowerBound=null,creepClearance=null;
  if(certificate){
   const movedNorth=(pos.lat-certificate.lat)*rad*6378137;
   const movedEast=wrap(pos.lon-certificate.lon)*rad*6378137*Math.cos(certificate.lat*rad);
   rangeLowerBound=certificate.range-certificate.uncertainty-Math.hypot(movedNorth,movedEast)
    -(p.max_target_speed_mps??3)*Math.max(0,now-certificate.at)/1000;
   creepClearance=rangeLowerBound-Math.max(c.safety_distance_m??.3,p.clearance_m??1.2)
    -.6*Math.max(0,forwardVelocity);
  }
  const nearRaw=(uwb.sensors||[]).some(s=>fresh(s.rawDistanceUpdatedAt||s.receivedMeasurementAt,now,p.measurement_gap_ms??450)
    &&finite(s.rawDistanceM)&&s.rawDistanceM>=0&&s.rawDistanceM<=(p.clearance_m??1.2));
  if(c.reject_nmpc?.enabled===true){
   const evidence=c.reject_nmpc.clearance_policy==='adaptive'?this.nmpc.rangeEvidence(now,telemetry):null;
   const plan=this.nmpc.solve(now,telemetry,direction,evidence??nearRaw);
   const headingError=wrap(this.anchor.heading-a.yaw);
   let turn=clamp((c.traditional_heading_gain??.85)*headingError
    -(c.traditional_yaw_rate_damping??.6)*(a.yawRateDegS||0),-yawLimit,yawLimit);
   if(plan.feasible){
    speed=plan.forwardSpeedMps;turn=plan.yawRateDegS;
    this.anchor.lat=pos.lat;this.anchor.lon=pos.lon;
   }else {
    // A credible near-range veto must also prevent the holding anchor from
    // commanding forward motion. Unknown zero packets do not assert proximity.
    if(evidence?.rangeM!==null&&evidence?.rangeM!==undefined)speed=Math.min(0,speed);
    this.nmpc.command={at:now,speed,yaw:turn};
   }
   this.farPursuit=false;this.creepSpeed=Math.max(0,speed);this.creepAt=now;
   return {active:true,reason:plan.feasible?'reject_nmpc':'gps_drift_hold',forwardSpeedMps:speed,yawRateDegS:turn,
    forwardDisplacementM:forwardDisplacement,crossDisplacementM:crossDisplacement,forwardVelocityMps:forwardVelocity,
    anchorHeadingDeg:this.anchor.heading,headingErrorDeg:headingError,elapsedMs:now-this.anchor.enteredAt,
    positionHoldIsLongitudinal:!plan.feasible,liveBearingConfirmed:liveBearing,bearingConfidence:direction,
    bearingProgress:false,predictedProgress:false,farProgress:false,nmpcProgress:plan.feasible,nmpc:plan};
  }
  const prediction=this.acceptedPrediction(now,result,forwardVelocity,Math.hypot(v.vx,v.vy));
  // Coarse distant pursuit needs live angular evidence, not a new precise XY
  // fix. Its range bound comes ONLY from an admitted certificate or its bounded
  // GPS/IMU prediction. A large raw outlier cannot switch on this mode.
  const certificateLower=certificate&&fresh(certificate.measurementAt,now,p.wait_timeout_ms??5000)
   ?rangeLowerBound-(p.max_target_speed_mps??3)*Math.max(0,certificate.at-certificate.measurementAt)/1000:null;
  const farLower=Math.max(finite(certificateLower)?certificateLower:-Infinity,prediction?.rangeLowerBoundM??-Infinity);
  const deceleration=p.braking_deceleration_mps2??.3,closing=Math.max(0,forwardVelocity);
  const stoppingDistance=.6*closing+closing*closing/(2*deceleration);
  const farClearance=farLower-Math.max(c.safety_distance_m??.3,p.clearance_m??1.2)-stoppingDistance;
  const outward=liveBearing?Math.max(0,Math.sign(direction.bearingDeg)*(direction.rateDegS-(a.yawRateDegS||0))):0;
  const projectedBearing=liveBearing?Math.abs(direction.bearingDeg)+.5*outward:180;
  const farAligned=clamp((35-projectedBearing)/15,0,1);
  this.farPursuit=c.far_pursuit_enabled===true&&liveBearing&&!nearRaw&&farAligned>0
   &&farLower>=(this.farPursuit?2.5:3)&&farClearance>0;
  const farProgress=this.farPursuit;
  const farSpeedLimit=Math.min(c.max_speed_mps??2,c.commissioning_mode?c.commissioning_speed_mps??2:2,
   maximum+(1-maximum)*clamp((farLower-2.5)/2.5,0,1));
  const liveAligned=liveBearing?clamp((35-Math.abs(direction.bearingDeg))/15,0,1):0;
  const bearingProgress=!farProgress&&liveBearing&&liveAligned>0&&!nearRaw&&finite(creepClearance)&&creepClearance>0;
  const predictedAligned=prediction?clamp((35-Math.abs(liveBearing?direction.bearingDeg:prediction.bearingDeg))/15,0,1):0;
  const predictedProgress=!farProgress&&!bearingProgress&&!nearRaw&&prediction!==null&&predictedAligned>0;
  const progress=farProgress||bearingProgress||predictedProgress;
  if(predictedProgress&&!liveBearing)this.anchor.heading=wrap(a.yaw+prediction.bearingDeg);
  const dt=this.creepAt===null?0.1:clamp((now-this.creepAt)/1000,0,.25);this.creepAt=now;
  if(progress){
   const available=predictedProgress?prediction.clearanceM:creepClearance;
   const aligned=predictedProgress?predictedAligned:liveAligned;
   const cap=farProgress?Math.min(farSpeedLimit*farAligned,
    Math.sqrt((.6*deceleration)**2+2*deceleration*Math.max(0,farLower-Math.max(c.safety_distance_m??.3,p.clearance_m??1.2)))-.6*deceleration)
    :Math.min(maximum*aligned,Math.sqrt(2*deceleration*available));
   this.creepSpeed=Math.min(cap,this.creepSpeed+(p.braking_deceleration_mps2??.3)*dt);speed=this.creepSpeed;
   this.lastProgressAt=now;
   // Progress replaces the old stationary anchor. A later bad angle must not
   // command us back over distance legitimately gained during this mode.
   this.anchor.lat=pos.lat;this.anchor.lon=pos.lon;
  }else if(nearRaw||!finite(creepClearance)||creepClearance<=0||this.lastProgressAt===null
    ||now-this.lastProgressAt>(p.measurement_gap_ms??450))this.creepSpeed=0;
  // Either independent live bearing plus certified range, or a strictly
  // bounded admitted-position prediction authorizes progress. Both expire on
  // their original clocks and fall back to the same GPS holding anchor.
  const headingError=wrap(this.anchor.heading-a.yaw);
  const yawRate=clamp((c.traditional_heading_gain??.85)*headingError
    -(c.traditional_yaw_rate_damping??.6)*(finite(a.yawRateDegS)?a.yawRateDegS:0),-yawLimit,yawLimit);
  return {active:true,reason:farProgress?'far_bearing_pursuit':predictedProgress?'accepted_prediction_progress':bearingProgress?'bearing_progress':'gps_drift_hold',forwardSpeedMps:speed,yawRateDegS:yawRate,
   forwardDisplacementM:forwardDisplacement,crossDisplacementM:crossDisplacement,
   forwardVelocityMps:forwardVelocity,anchorHeadingDeg:this.anchor.heading,headingErrorDeg:headingError,
   elapsedMs:now-this.anchor.enteredAt,positionHoldIsLongitudinal:!progress,
   liveBearingConfirmed:liveBearing,bearingSamples:this.bearings.length,yawLimitDegS:yawLimit,
   bearingProgress,bearingConfidence:direction,rangeLowerBoundM:rangeLowerBound,creepClearanceM:creepClearance,
   rangeCertificateAt:certificate?.at??null,rangeCertificateMeasurementAt:certificate?.measurementAt??null,
   predictedProgress,acceptedPrediction:prediction,farProgress,
   farRangeLowerBoundM:finite(farLower)?farLower:null,farClearanceM:finite(farClearance)?farClearance:null,
   farBoundSource:prediction&&prediction.rangeLowerBoundM>(certificateLower??-Infinity)?'accepted_prediction':'accepted_certificate',
   farRangeCertificateAgeMs:certificate?now-certificate.measurementAt:null,
   farStoppingDistanceM:stoppingDistance,farSpeedLimitMps:farSpeedLimit,farProjectedBearingDeg:projectedBearing};
 }
}
