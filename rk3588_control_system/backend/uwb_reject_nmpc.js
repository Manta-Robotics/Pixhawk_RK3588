// Receding-horizon nonlinear shooting controller for rejected UWB intervals.
// Own GPS/IMU + last admitted posterior only; no simulator target/state access.
// This is a bounded approximate optimizer, not a global optimum or a certified
// hydrodynamic model. Every applied solution is checked against the same tube.
import {uwbObservationQuarantined} from './uwb_quality.js';
const R=Math.PI/180, earth=6378137;
const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
const wrap=x=>((x+180)%360+360)%360-180;
const finite=Number.isFinite;
const fresh=(at,now,ms)=>finite(at)&&at>0&&now>=at&&now-at<ms;
export class UwbRejectNmpc {
 constructor(config={}){this.c=config;this.reset();}
 reset(){this.accepted=null;this.command=null;this.plan=null;}
 rangeEvidence(now,t){
  const c=this.c,p=c.prediction||{},gap=p.measurement_gap_ms??450;
  let near=null,invalidZeros=0,invalidPackets=0;
  for(const s of t.uwb?.sensors||[]){
   const at=s.rawDistanceUpdatedAt??s.receivedMeasurementAt,r=s.rawDistanceM;
   if(!fresh(at,now,gap))continue;
   // Zero is the receiver/replay's invalid sentinel, never a physical range.
   // An invalid radio packet cannot prove proximity or refresh a certificate.
   if(r===0){invalidZeros++;continue;}
   if(s.valid===false||!finite(r)||r<.15||r>20){invalidPackets++;continue;}
   if(r<=1.2&&(near===null||r<near.rangeM))near={rangeM:r,at};
  }
  const noise=Math.max(.12,p.measurement_noise_m??.12),safety=c.safety_distance_m??.3;
  const sampleAge=near?(now-near.at)/1000:0;
  const ageAllowance=sampleAge*((p.max_target_speed_mps??3)+Math.hypot(t.velocity?.vx??0,t.velocity?.vy??0));
  return {kind:near?'positive_near_range':invalidZeros?'invalid_zero_unknown':'no_near_range',
   veto:near!==null&&near.rangeM-noise<=safety,rangeM:near?.rangeM??null,
   lowerBoundM:near?near.rangeM-noise-ageAllowance:null,at:near?.at??null,
   noiseM:noise,ageAllowanceM:ageAllowance,invalidZeros,invalidPackets};
 }
 observe(now,t,result){
  if(result.safe&&finite(result.forwardSpeedMps)&&finite(result.yawRateDegS))
   this.command={at:now,speed:result.forwardSpeedMps,yaw:result.yawRateDegS};
  const e=result.targetEstimate,p=this.c.prediction||{},s=t.uwb?.sensors?.[0];
  if(e?.admissionVersion!==1||e.measurementConfirmed!==true||e.referenceFrame!=='local_ned'
   ||!fresh(e.lastMeasuredAt,now,p.measurement_gap_ms??450)||e.evaluatedAt!==now
   ||!finite(e.relativeBodyM?.x)||!finite(e.relativeBodyM?.y)||!finite(e.uncertaintyM)||e.uncertaintyM<0
   ||!finite(t.position?.lat)||!finite(t.position?.lon)||!finite(t.attitude?.yaw)
   ||!s||uwbObservationQuarantined(s)||s.valid===false||s.measurementStatus!=='trusted'||s.receivedMeasurementAt!==e.lastMeasuredAt
   ||this.accepted?.measurementAt===e.lastMeasuredAt)return;
  const yaw=t.attitude.yaw*R,b=e.relativeBodyM;
  const trusted=e.velocityEstimate?.trusted===true&&finite(e.targetVelocityNedMps?.x)&&finite(e.targetVelocityNedMps?.y);
  const velocity=trusted?{...e.targetVelocityNedMps}:{x:0,y:0};
  if(Math.hypot(velocity.x,velocity.y)>(p.max_target_speed_mps??3))return;
  this.accepted={at:now,measurementAt:e.lastMeasuredAt,id:`${s.receiverId}:${s.tagId??''}`,
   lat:t.position.lat,lon:t.position.lon,x:b.x*Math.cos(yaw)-b.y*Math.sin(yaw),
   y:b.x*Math.sin(yaw)+b.y*Math.cos(yaw),velocity,velocityTrusted:trusted,
   // Preserve the admitted covariance; a later rejected sample cannot inflate
   // or replace this posterior. Time propagation is added below exactly once.
   uncertainty:e.uncertaintyM};
 }
 solve(now,t,direction,nearRaw=false){
  const start=performance.now(),c=this.c,p=c.prediction||{},m=c.reject_nmpc||{},a=this.accepted;
  const fail=reason=>({feasible:false,reason,solveMs:performance.now()-start});
  if(m.enabled!==true)return fail('disabled');
  const lossLimit=p.wait_timeout_ms??5000;
  if(!a||now<a.at||!fresh(a.measurementAt,now,lossLimit))return fail('accepted_state_expired');
  const sensors=t.uwb?.sensors||[];
  if(sensors.some(s=>`${s.receiverId}:${s.tagId??''}`!==a.id)){this.reset();return fail('identity_changed');}
  const evidence=typeof nearRaw==='object'?nearRaw:null;
  if(evidence?evidence.veto:nearRaw)return {...fail('near_raw_veto'),rangeEvidence:evidence};
  if(!finite(t.velocity?.vx)||!finite(t.velocity?.vy)||!finite(t.attitude?.yaw)||!finite(t.attitude?.yawRateDegS)
   ||!finite(t.position?.lat)||!finite(t.position?.lon))return fail('navigation_invalid');
  const yaw=t.attitude.yaw*R,own={x:t.velocity.vx,y:t.velocity.vy};
  const u=own.x*Math.cos(yaw)+own.y*Math.sin(yaw);
  // Local cross-track disturbance inferred from own GPS; longitudinal ground
  // speed is regulated by Rover's native ACRO speed loop, with a lag model.
  const drift={x:own.x-u*Math.cos(yaw),y:own.y-u*Math.sin(yaw)};
  const elapsed=(now-a.at)/1000,measurementAge=(now-a.measurementAt)/1000;
  const moved={x:(t.position.lat-a.lat)*R*earth,
   y:wrap(t.position.lon-a.lon)*R*earth*Math.cos(a.lat*R)};
  const target={x:a.x+a.velocity.x*elapsed-moved.x,y:a.y+a.velocity.y*elapsed-moved.y};
  const targetSpeedError=a.velocityTrusted?(p.velocity_uncertainty_mps??.2):(p.max_target_speed_mps??3);
  const acceleration=p.acceleration_bound_mps2??.6;
  const ageAtAcceptance=(a.at-a.measurementAt)/1000;
  const radius=time=>a.uncertainty+targetSpeedError*(elapsed+time)
   +.5*acceleration*((measurementAge+time)**2-ageAtAcceptance**2)
   +.10*time+.10*time*time; // own short-horizon model error allowance
  const adaptive=m.clearance_policy==='adaptive';
  const safety=c.safety_distance_m??.3;
  const live=!sensors.some(uwbObservationQuarantined)&&direction?.trusted===true&&fresh(direction.at,now,p.measurement_gap_ms??450);
  const initialAngle=live?wrap(direction.headingDeg-t.attitude.yaw):wrap(Math.atan2(target.y,target.x)/R-t.attitude.yaw);
  // Covariance, future target motion and stopping distance are already explicit
  // constraints. Replace the legacy blanket 1.2 m by physical safety + a small
  // speed/turn dependent modelling reserve, without reducing those other terms.
  const reserve=.10+.10*clamp(Math.abs(u),0,1)+.15*clamp(Math.abs(initialAngle)/45,0,1);
  const buffer=adaptive?safety+reserve:Math.max(safety,p.clearance_m??1.2);
  const brake=p.braking_deceleration_mps2??.3;
  const stop=v=>.6*Math.max(0,v)+Math.max(0,v)**2/(2*brake);
  const initialRange=Math.hypot(target.x,target.y),initialRadius=radius(0);
  const initialLower=Math.min(initialRange-initialRadius,evidence?.lowerBoundM??Infinity);
  const diagnostic={initialRangeLowerBoundM:initialLower,initialStoppingDistanceM:stop(u),bufferM:buffer,
   initialRangeM:initialRange,initialUncertaintyM:initialRadius,initialBearingDeg:initialAngle,initialForwardVelocityMps:u,
   safetyDistanceM:safety,modellingReserveM:adaptive?reserve:buffer-safety,clearancePolicy:adaptive?'adaptive':'legacy',
   rangeEvidence:evidence,measurementAt:a.measurementAt,measurementAgeMs:now-a.measurementAt};
  if(initialLower-buffer-stop(u)<=0)return {...fail('no_initial_braking_clearance'),...diagnostic};
  const maxSpeed=Math.max(0,Math.min(1,c.max_speed_mps??2,c.commissioning_mode?c.commissioning_speed_mps??2:2));
  const yawMax=Math.min(14,c.max_yaw_rate_deg_s??14);
  const commandDt=this.command?clamp((now-this.command.at)/1000,0,.25):.1;
  const recent=this.command&&fresh(this.command.at,now,p.measurement_gap_ms??450);
  const previous={speed:recent?this.command.speed:Math.max(0,u),yaw:recent?this.command.yaw:t.attitude.yawRateDegS};
  const slew=brake,yawSlew=40;
  let evaluations=0,best=null,timedOut=false;
  const budget=Math.min(25,Math.max(1,m.solve_budget_ms??20));
  const project=(controls,blockDt)=>{
   let v=previous.speed,w=previous.yaw;
   return controls.map((q,i)=>{
    const dt=i===0?commandDt:blockDt;
    // Immediate speed reductions remain possible for safety. Acceleration and
    // both yaw changes are bounded; no stale command can authorize a surge.
    v=clamp(q[0],0,Math.min(maxSpeed,Math.max(0,v)+slew*dt));
    w=clamp(q[1],Math.max(-yawMax,w-yawSlew*dt),Math.min(yawMax,w+yawSlew*dt));
    return [v,w];
   });
  };
  const rollout=(controls,horizon,record=false)=>{
   evaluations++;
   const blockDt=horizon/4,dt=blockDt/3;
   let x=0,y=0,psi=yaw,speed=u,turn=t.attitude.yawRateDegS,cost=0,minClearance=Infinity,maxFov=0;
   let prior=previous,path=[];
   for(let k=0;k<4;k++){
    const [v,w]=controls[k];
    cost+=3*(v-prior.speed)**2+.004*(w-prior.yaw)**2;prior={speed:v,yaw:w};
    for(let j=0;j<3;j++){
     const time=(k*3+j+1)*dt;
     speed+=clamp((v-speed)/.7,-brake,.8)*dt;
     turn+=clamp((w-turn)/.35,-40,40)*dt;
     psi+=turn*R*dt;x+=(speed*Math.cos(psi)+drift.x)*dt;y+=(speed*Math.sin(psi)+drift.y)*dt;
     const dx=target.x+a.velocity.x*time-x,dy=target.y+a.velocity.y*time-y;
     const range=Math.hypot(dx,dy),uncertainty=radius(time);
     // A plausible positive close return can only LOWER the planning envelope.
     // It is never accepted as target position or used to refresh its clock.
     const rawLower=evidence?.lowerBoundM===null||!evidence?Infinity:
      evidence.lowerBoundM+(range-initialRange)-(uncertainty-initialRadius);
     const lower=Math.min(range-uncertainty,rawLower);
     const clearance=lower-buffer-stop(Math.max(speed,v));
     const heading=live?direction.headingDeg+direction.rateDegS*time:Math.atan2(dy,dx)/R;
     const angle=wrap(heading-psi/R);
     // Live bearing supplies independent angular evidence, not a new range
     // certificate. Blind angular uncertainty comes from the position tube.
     const angleError=live?6+3*time:Math.asin(clamp(uncertainty/Math.max(range,.001),0,1))/R;
     const fov=Math.abs(angle)+angleError;
     if(!finite(clearance)||clearance<=0||!finite(fov)||fov>=45)return {feasible:false,cost:Infinity};
     minClearance=Math.min(minClearance,clearance);maxFov=Math.max(maxFov,fov);
     const desired=Math.max(buffer+.4,c.following_distance_m??.5);
     cost+=dt*(2*(range-desired)**2+.012*angle**2+.12*turn**2+.05*v*v);
     if(record)path.push({timeS:time,ownNorthM:x,ownEastM:y,targetNorthM:target.x+a.velocity.x*time,
      targetEastM:target.y+a.velocity.y*time,rangeM:range,uncertaintyM:uncertainty,speedMps:speed,
      commandSpeedMps:v,brakingDistanceM:stop(Math.max(speed,v)),clearanceM:clearance,
      rangeLowerBoundM:lower,rawRangeLowerBoundM:finite(rawLower)?rawLower:null,bearingBoundDeg:fov});
    }
   }
   return {feasible:true,cost,minClearanceM:minClearance,maxBearingBoundDeg:maxFov,path};
  };
  // Direct shooting with warm starts and bounded coordinate search over eight
  // control variables. Nonlinear sine/cosine dynamics and tube constraints are
  // re-evaluated for every candidate. Apply only the first control, then replan.
  for(const horizon of [1.2,.6,.3]){
   if(measurementAge+horizon>=lossLimit/1000)continue;
   const heading=live?direction.headingDeg:Math.atan2(target.y,target.x)/R;
   const aim=clamp(.85*wrap(heading-t.attitude.yaw)-.3*t.attitude.yawRateDegS,-yawMax,yawMax);
   const seeds=[this.plan?.controls,Array.from({length:4},()=>[previous.speed,aim]),
    Array.from({length:4},()=>[0,aim]),Array.from({length:4},()=>[maxSpeed,aim])].filter(Boolean);
   let winner=null;
   for(const seed of seeds){const controls=project(seed,horizon/4),r=rollout(controls,horizon);
    if(r.feasible&&(!winner||r.cost<winner.cost))winner={...r,controls,horizon};}
   if(!winner)continue;
   for(const scale of [1,.5,.25]){
    for(let k=0;k<4;k++)for(let axis=0;axis<2;axis++)for(const sign of [-1,1]){
     if(performance.now()-start>budget){timedOut=true;break;}
     const trial=winner.controls.map(q=>[...q]);trial[k][axis]+=sign*scale*(axis===0?.3:6);
     const controls=project(trial,horizon/4),r=rollout(controls,horizon);
     if(r.feasible&&r.cost<winner.cost)winner={...r,controls,horizon};
    }
    if(timedOut)break;
   }
   best=winner;break;
  }
  // Never reuse an old trajectory after infeasibility or solver deadline.
  timedOut=timedOut||performance.now()-start>budget;
  if(!best||timedOut){this.plan=null;return {...fail(timedOut?'solver_deadline':'no_feasible_trajectory'),...diagnostic};}
  const audit=rollout(best.controls,best.horizon,true);
  if(!audit.feasible)return fail('solution_validation_failed');
  if(performance.now()-start>budget){this.plan=null;return fail('solver_deadline');}
  this.plan=best;
  const [speed,yawRate]=best.controls[0];this.command={at:now,speed,yaw:yawRate};
  return {feasible:true,reason:'reject_nmpc',forwardSpeedMps:speed,yawRateDegS:yawRate,
   measurementAt:a.measurementAt,measurementAgeMs:now-a.measurementAt,velocityTrusted:a.velocityTrusted,
   horizonS:best.horizon,evaluations,solveMs:performance.now()-start,...diagnostic,maxSpeedMps:maxSpeed,liveBearing:live,
   minClearanceM:audit.minClearanceM,maxBearingBoundDeg:audit.maxBearingBoundDeg,
   path:audit.path,controls:best.controls,cost:best.cost};
 }
}
