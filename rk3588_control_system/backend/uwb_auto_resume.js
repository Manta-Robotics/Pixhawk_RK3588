// Retain operator follow intent across UWB timeouts; never across manual stops.
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
 clearWindow(){this.samples=0;this.firstAt=null;this.lastAt=null;this.point=null;this.windowTarget=null;}
 cancel(){this.requested=false;this.pending=false;this.attempting=false;this.target=null;this.lastStopReason=null;this.resumeAt=null;this.clearWindow();}
 // An operator may request acquisition before the first valid radio packet.
 // This authorizes waiting only; observe() still requires the complete resume gate.
 search(){this.cancel();this.requested=true;this.pending=true;this.lastStopReason='uwb_stale';}
 started(telemetry={},now=Date.now(),automatic=false){
  this.requested=this.enabled;this.pending=false;this.attempting=false;
  this.target=targetId(telemetry.uwb)||this.target;this.resumeAt=automatic?now:null;this.clearWindow();
 }
 stopped(reason){
  this.resumeAt=null;this.clearWindow();this.attempting=false;this.lastStopReason=reason;
  this.pending=this.enabled&&this.requested&&['prediction_wait_timeout','uwb_stale'].includes(reason);
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
  const u=telemetry.uwb||{},at=u.measurementSampleAt??u.measurementUpdatedAt,range=u.distanceM;
  // The bridge admits complete valid frames. Recovery requires one fresh
  // observation, with no frame-count, position-continuity or angle gate.
  if(this.transitionFailure(now,telemetry,preflight)||!preflight.safe||!u.online||!u.fresh
    ||!finite(at)||at>now||now-at>(this.c.uwb_timeout_ms??700)
    ||!finite(range)||range<=(this.c.safety_distance_m??.3)||!finite(u.azimuthDeg)){
    this.clearWindow();return false;
  }
  const id=targetId(u);
  if(this.target===null&&id!==null)this.target=id;
  this.firstAt=this.lastAt=at;this.windowTarget=id;this.samples=1;
  return true;
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
