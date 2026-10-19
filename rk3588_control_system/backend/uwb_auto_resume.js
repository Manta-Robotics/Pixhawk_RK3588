// Retain operator follow intent across UWB timeouts; never across manual stops.
const wrap=x=>((x+180)%360+360)%360-180;
const finite=x=>typeof x==='number'&&Number.isFinite(x);
function targetId(uwb={}) {
 const tags=(uwb.sensors||[]).filter(s=>!(uwb.activeReceiverIds||[]).length||uwb.activeReceiverIds.includes(s.receiverId))
  .map(s=>s.tagId).filter(x=>x!==null&&x!==undefined);
 if(uwb.tagId!==null&&uwb.tagId!==undefined)tags.push(uwb.tagId);
 return tags.length?[...new Set(tags)].sort().join(','):null;
}
export class UwbAutoResume {
 constructor(config={}) {
  this.enabled=config.auto_resume_after_uwb_timeout===true;
  this.c=config;this.p=config.prediction||{};this.cancel();
 }
 clearWindow(){this.samples=0;this.firstAt=null;this.lastAt=null;this.point=null;}
 cancel(){this.requested=false;this.pending=false;this.attempting=false;this.target=null;this.lastStopReason=null;this.resumeAt=null;this.clearWindow();}
 started(telemetry={},now=Date.now(),automatic=false){
  this.requested=this.enabled;this.pending=false;this.attempting=false;
  this.target=targetId(telemetry.uwb)||this.target;this.resumeAt=automatic?now:null;this.clearWindow();
 }
 stopped(reason){
  this.resumeAt=null;this.clearWindow();this.attempting=false;this.lastStopReason=reason;
  this.pending=this.enabled&&this.requested&&reason==='prediction_wait_timeout';
  if(!this.pending)this.requested=false;
 }
 attemptFailed(reason){
  const transient=['uwb_stale','auto_resume_signal_timeout','recovered_uwb_timeout','imu_stale','imu_invalid','acro_velocity_unavailable','gps_fix_invalid','ekf_unhealthy','navigation_stale'];
  if(!transient.includes(reason)){this.cancel();return;}
  this.attempting=false;this.clearWindow();
 }
 transitionFailure(now,telemetry,preflight){
  // A stale UWB packet must not hide a simultaneous navigation failure.
  const a=telemetry.attitude||{},v=telemetry.velocity||{},u=telemetry.uwb||{};
  const fresh=at=>finite(at)&&at<=now&&now-at<=(this.c.navigation_timeout_ms??700);
  if(!finite(a.yaw)||!fresh(a.updatedAt))return 'imu_stale';
  if(this.c.execution_mode==='acro'){
   if(!fresh(telemetry.position?.updatedAt)||!fresh(telemetry.ekf?.updatedAt)
     ||telemetry.ekf?.healthy!==true||!finite(v.vx)||!finite(v.vy)
     ||(telemetry.gps?.fixType??0)<(this.c.minimum_gps_fix_type??2))return 'navigation_stale';
   if(telemetry.acroControl?.ready!==true||!fresh(telemetry.acroControl.updatedAt))return 'acro_parameters_not_ready';
  }
  if(this.target!==null&&targetId(u)!==this.target)return 'target_identity_changed';
  const ranges=[u.distanceM,...(u.sensors||[]).filter(s=>{
   const at=s.rawDistanceUpdatedAt||s.receivedMeasurementAt;
   return finite(at)&&at<=now&&now-at<=450;
  }).map(s=>s.rawDistanceM)];
  if(ranges.some(r=>finite(r)&&r>=0&&r<=(this.c.safety_distance_m??.3)))return 'safety_distance';
  return !preflight.safe&&preflight.reason!=='uwb_stale'?preflight.reason:null;
 }
 observe(now,telemetry,preflight,{duringAttempt=false}={}){
  if(!this.pending||(this.attempting&&!duringAttempt))return false;
  const u=telemetry.uwb||{},a=telemetry.attitude||{};
  const sensors=(u.sensors||[]).filter(s=>!(u.activeReceiverIds||[]).length||u.activeReceiverIds.includes(s.receiverId));
  const trusted=u.measurementStatus==='trusted'||(sensors.length>0&&sensors.every(s=>s.measurementStatus==='trusted'));
  const at=u.measurementUpdatedAt,range=u.distanceM;
  const bearing=wrap(u.azimuthDeg*(this.c.bearing_sign??1)*(this.c.bearing_scale??1)+(this.c.bearing_offset_deg??0));
  const id=targetId(u),gap=this.p.measurement_gap_ms??450;
  // max_uncertainty_m limits blind prediction. A new KF-accepted observation
  // is confirmed through this independent temporal/kinematic window instead;
  // inflated historical measurement covariance must not lock out reacquisition.
  if(this.transitionFailure(now,telemetry,preflight)) {this.clearWindow();return false;}
  if(!preflight.safe||!trusted||!u.online||!u.fresh||!finite(at)||at>now||now-at>gap
    ||!finite(range)||range<=(this.c.safety_distance_m??.3)||!finite(u.azimuthDeg)
    ||Math.abs(bearing)>(this.c.field_of_view_half_deg??45)
    ||!finite(a.yaw)||!finite(a.updatedAt)||a.updatedAt>now||now-a.updatedAt>(this.c.navigation_timeout_ms??700)
    ||(this.target!==null&&id!==this.target)) {
    // No authorization on an invalid frame. Preserve the preceding good
    // samples only across a short radio gap; repeated polling is not a sample.
    if(this.lastAt===null||now-this.lastAt>gap||Math.abs(bearing)>(this.c.field_of_view_half_deg??45))this.clearWindow();
    return false;
  }
  if(this.lastAt!==null&&at<=this.lastAt)return false;
  const theta=(a.yaw+bearing)*Math.PI/180,point={x:range*Math.cos(theta),y:range*Math.sin(theta)};
  const dt=this.lastAt===null?0:(at-this.lastAt)/1000;
  const own=telemetry.velocity||{};
  const ownSpeed=finite(own.vx)&&finite(own.vy)?Math.hypot(own.vx,own.vy):3;
  if(this.lastAt!==null&&(at-this.lastAt>gap||Math.hypot(point.x-this.point.x,point.y-this.point.y)
    >(this.p.innovation_gate_m??.6)+((this.p.max_target_speed_mps??3)+ownSpeed)*dt))this.clearWindow();
  if(this.firstAt===null)this.firstAt=at;
  this.lastAt=at;this.point=point;this.samples++;
  return this.samples>=(this.p.min_samples??3)&&at-this.firstAt>=180;
 }
 beginAttempt(){if(!this.pending||this.attempting)return false;this.attempting=true;return true;}
 ramp(result,now){
  if(this.resumeAt===null)return result;
  const scale=Math.min(1,Math.max(0,(now-this.resumeAt)/Math.max(1,this.p.recovery_blend_ms??800)));
  if(scale>=1){this.resumeAt=null;return result;}
  return {...result,status:'recovering',reason:'auto_resume_recovering',autoResumeScale:scale,
   forwardSpeedMps:finite(result.forwardSpeedMps)?result.forwardSpeedMps*scale:result.forwardSpeedMps,throttlePercent:(result.throttlePercent||0)*scale,
   desiredSpeedMps:finite(result.desiredSpeedMps)?result.desiredSpeedMps*scale:result.desiredSpeedMps,
   velocityNed:result.velocityNed?{x:result.velocityNed.x*scale,y:result.velocityNed.y*scale}:result.velocityNed};
 }
 snapshot(){return {followRequested:this.requested,autoResumePending:this.pending,
  autoResumeSamples:this.samples,autoResumeAttempting:this.attempting,lastStopReason:this.lastStopReason};}
}
