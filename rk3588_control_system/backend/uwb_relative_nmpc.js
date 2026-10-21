// Nonlinear receding-horizon control for the relative MANUAL execution path.
// GPS supplies measured surge when available. Without GPS, surge is only an
// internal actuator-response state: measured relative motion supplies the
// disturbance, and no absolute vessel/target velocity is claimed observable.
const R=Math.PI/180;
const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
const wrap=x=>((x+180)%360+360)%360-180;
const finite=Number.isFinite;
export const RELATIVE_NMPC_DEFAULTS=Object.freeze({enabled:false,horizon_s:2.4,
  solve_budget_ms:15,surge_time_constant_s:1,yaw_time_constant_s:.35,
  surge_map_exponent:.5,model_cruise_speed_mps:.8,model_position_error_mps:.1,steering_share:.8});

export class UwbRelativeNmpc {
  constructor(config){this.c=config;this.o={...RELATIVE_NMPC_DEFAULTS,...config.relativeNmpc};this.reset();}
  reset(){this.at=null;this.measurementAt=null;this.id=null;this.history=[];
    this.modelSurge=0;this.applied={throttle:0,steering:0};this.plan=null;}
  trackApplied(throttle,steering){
    this.applied={throttle:finite(throttle)?Math.max(0,throttle):0,steering:finite(steering)?steering:0};
  }
  project(controls,dt){
    const c=this.c,step=this.o.horizon_s/4;let q={...this.applied};
    return controls.map((input,i)=>{
      const h=i===0?dt:step;
      const throttle=clamp(input[0],Math.max(0,q.throttle-c.throttleDecelPercentS*h),
        Math.min(c.relativeThrottleLimitPercent,q.throttle+c.throttleAccelPercentS*h));
      // Same geometry and forward-only constraint as the production sender.
      const limit=Math.max(0,Math.min(c.relativeSteeringLimitDeg,throttle*.45*this.o.steering_share,(100-throttle)*.45));
      const steering=clamp(clamp(input[1],q.steering-c.steeringSlewDegS*h,q.steering+c.steeringSlewDegS*h),-limit,limit);
      q={throttle,steering};return [throttle,steering];
    });
  }
  observe(now,t,range,bearing,estimate){
    const at=t.uwb?.measurementUpdatedAt;
    const ids=t.uwb?.activeReceiverIds||[];
    const selected=(t.uwb?.sensors||[]).filter(s=>ids.includes(s.receiverId));
    const id=selected.map(s=>`${s.receiverId}:${s.tagId??''}`).sort().join('|')||null;
    if(this.id!==null&&id!==null&&id!==this.id){this.reset();return 'target_identity_changed';}
    if(id!==null)this.id=id;
    // Predictions/rejected candidates may be evaluated, but never enter the
    // observed-motion history or refresh the accepted measurement clock.
    const admitted=t.uwb?.online===true&&t.uwb?.fresh===true
      &&(estimate?estimate.measurementConfirmed===true:t.uwb.measurementStatus==='trusted')
      &&selected.every(s=>s.filterAdmissionVersion!==1||(s.measurementStatus==='trusted'
        &&s.valid!==false&&s.receivedMeasurementAt===s.measurementUpdatedAt))
      &&finite(at)&&at<=now&&now-at<=this.c.uwbTimeoutMs;
    if(admitted&&(this.measurementAt===null||at>this.measurementAt)){
      const yaw=(t.attitude.yaw-t.attitude.yawRateDegS*(now-at)/1000)*R;
      const angle=yaw+bearing*R;
      this.history=this.history.filter(p=>at-p.at<=900);
      this.history.push({at,x:range*Math.cos(angle),y:range*Math.sin(angle)});
      this.measurementAt=at;
    }
    return null;
  }
  relativeVelocity(yaw){
    const points=this.history;
    if(points.length<3||points.at(-1).at-points[0].at<200)return {x:0,y:0,trusted:false};
    // Robust pairwise slope rejects a single noisy range derivative. These are
    // relative velocities in the IMU-stabilized frame, not target ground speed.
    const sx=[],sy=[];
    for(let i=0;i<points.length-1;i++)for(let j=i+1;j<points.length;j++){
      const dt=(points[j].at-points[i].at)/1000;if(dt<.18)continue;
      sx.push((points[j].x-points[i].x)/dt);sy.push((points[j].y-points[i].y)/dt);
    }
    if(!sx.length)return {x:0,y:0,trusted:false};
    const median=a=>{a.sort((x,y)=>x-y);const n=a.length;return n%2?a[n>>1]:(a[n/2-1]+a[n/2])/2;};
    const x=median(sx),y=median(sy),limit=this.c.prediction.max_target_speed_mps??3;
    const scale=Math.min(1,limit/Math.max(.001,Math.hypot(x,y)));
    return {x:scale*(x*Math.cos(yaw)+y*Math.sin(yaw)),
      y:scale*(-x*Math.sin(yaw)+y*Math.cos(yaw)),trusted:true};
  }
  solve({now,telemetry:t,range,bearing,estimate,referenceThrottle,gpsAvailable=false}){
    const started=performance.now(),c=this.c,o=this.o;
    const mode=gpsAvailable?'gps_imu':'uwb_imu_relative';
    const fail=reason=>{this.plan=null;return {feasible:false,reason,mode,
      solveMs:performance.now()-started,path:[],absoluteSpeedObservable:gpsAvailable};};
    if(!o.enabled)return fail('disabled');
    if(this.at!==null&&(now<=this.at||now-this.at>c.uwbTimeoutMs)){this.reset();return fail('control_clock_gap');}
    const dt=this.at===null?.1:clamp((now-this.at)/1000,.001,.25);this.at=now;
    if(!finite(range)||range<=c.safetyDistanceM||!finite(bearing)
      ||!finite(t.attitude?.yaw)||!finite(t.attitude?.yawRateDegS)
      ||!finite(t.attitude?.updatedAt)||t.attitude.updatedAt>now||now-t.attitude.updatedAt>c.navigationTimeoutMs)return fail('invalid_relative_state');
    const fault=this.observe(now,t,range,bearing,estimate);if(fault)return fail(fault);
    const at=estimate?.lastMeasuredAt??t.uwb?.measurementUpdatedAt;
    const maximumAge=estimate?.source==='predicted'?(c.prediction.horizon_ms??1000):c.uwbTimeoutMs;
    if(!finite(at)||at>now||now-at>maximumAge||this.measurementAt===null)return fail('accepted_state_stale');
    const yaw=t.attitude.yaw*R,rv=this.relativeVelocity(yaw);
    if(!rv.trusted&&!gpsAvailable)return fail('relative_motion_warming_up');
    const deceleration=c.prediction.braking_deceleration_mps2??.3;
    const speedLimit=Math.min(c.maxSpeedMps,c.commissioningMode?c.commissioningSpeedMps:c.maxSpeedMps);
    const throttleLimit=Math.max(1,c.relativeThrottleLimitPercent);
    // A nonlinear thrust/drag surrogate (quadratic drag by default), not a
    // linear assertion that a throttle percentage measures ground speed.
    const surgeTarget=q=>Math.min(speedLimit,
      Math.pow(clamp(q/throttleLimit,0,1),o.surge_map_exponent)*o.model_cruise_speed_mps);
    const surgeAcceleration=(u,q)=>{
      const v=o.model_cruise_speed_mps,desired=surgeTarget(q);
      // Quadratic hull drag retains coasting momentum at neutral thrust.
      // Model speed is an internal surrogate, never a GPS-free measurement.
      return clamp(v/(2*o.surge_time_constant_s)*((desired/v)**2-u*Math.abs(u)/(v*v)),
        -deceleration,deceleration);
    };
    this.modelSurge+=surgeAcceleration(this.modelSurge,this.applied.throttle)*dt;
    if(gpsAvailable){
      if(!finite(t.velocity?.vx)||!finite(t.velocity?.vy))return fail('gps_velocity_invalid');
      this.modelSurge=t.velocity.vx*Math.cos(yaw)+t.velocity.vy*Math.sin(yaw);
    }
    const u0=this.modelSurge,r0=t.attitude.yawRateDegS;
    const target={x:rv.x+u0,y:rv.y};
    const x0=range*Math.cos(bearing*R),y0=range*Math.sin(bearing*R);
    const covariance=estimate?.uncertaintyM??Math.max(c.prediction.measurement_noise_m??.12,
      ...(t.uwb?.sensors||[]).map(s=>finite(s.filterUncertaintyM)?s.filterUncertaintyM:0));
    if(!finite(covariance)||covariance<0)return fail('invalid_uncertainty');
    // The requested separation is a lower goal, not permission to consume the
    // uncertainty/braking reserve. At higher surge keep additional stand-off
    // so ordinary target turns do not repeatedly reach the infeasible boundary.
    const horizonError=covariance+o.model_position_error_mps*o.horizon_s+.05*o.horizon_s**2;
    const approachSpeed=.35+.5*(c.prediction.velocity_uncertainty_mps??.2);
    const approachReserve=.6*approachSpeed+approachSpeed**2/(2*deceleration);
    const desired=Math.max(c.followingDistanceM,c.safetyDistanceM+horizonError+approachReserve+.3);
    const steps=12,h=o.horizon_s/steps;
    let evaluations=0;
    const rollout=controls=>{
      evaluations++;let x=0,y=0,psi=0,u=u0,r=r0,cost=0,minClearance=Infinity;
      let prior={...this.applied};const path=[];
      for(let i=0;i<steps;i++){
        const q=controls[Math.floor(i/3)],time=(i+1)*h;
        const turn=q[1]/c.relativeSteeringLimitDeg*c.maxYawRateDegS;
        u+=surgeAcceleration(u,q[0])*h;
        r+=clamp((turn-r)/o.yaw_time_constant_s,-40,40)*h;
        psi+=r*R*h;x+=u*Math.cos(psi)*h;y+=u*Math.sin(psi)*h;
        const px=x0+target.x*time-x,py=y0+target.y*time-y,d=Math.hypot(px,py);
        const angle=wrap(Math.atan2(py,px)/R-psi/R);
        const closing=Math.max(0,-((target.x-u*Math.cos(psi))*px+(target.y-u*Math.sin(psi))*py)/Math.max(.01,d));
        const error=covariance+o.model_position_error_mps*time+.05*time*time;
        const closingUpper=closing+.5*(c.prediction.velocity_uncertainty_mps??.2);
        const stopping=.6*closingUpper+closingUpper*closingUpper/(2*deceleration);
        const clearance=d-error-c.safetyDistanceM-stopping;
        if(!finite(clearance)||clearance<=0||Math.abs(angle)>c.fieldOfViewHalfDeg)return null;
        minClearance=Math.min(minClearance,clearance);
        const desiredClosing=clamp(.5*(d-desired),0,.35);
        cost+=h*((d<desired?12:1.2)*(d-desired)**2+25*(angle*R)**2+4*(closing-desiredClosing)**2
          +.3*((q[0]-referenceThrottle)/throttleLimit)**2+.01*(r/c.maxYawRateDegS)**2);
        if(i%3===0){cost+=.2*((q[0]-prior.throttle)/throttleLimit)**2
          +.2*((q[1]-prior.steering)/c.relativeSteeringLimitDeg)**2;prior={throttle:q[0],steering:q[1]};}
        path.push({timeS:time,relativeXM:px*Math.cos(psi)+py*Math.sin(psi),
          relativeYM:-px*Math.sin(psi)+py*Math.cos(psi),rangeM:d,bearingDeg:angle,
          uncertaintyM:error,closingSpeedMps:closing,stoppingDistanceM:stopping,clearanceM:clearance});
      }
      cost+=40*(path.at(-1).bearingDeg*R)**2;
      return {controls,path,cost,minClearanceM:minClearance};
    };
    const aim=clamp(.85*bearing-.6*r0,-c.maxYawRateDegS,c.maxYawRateDegS)/c.maxYawRateDegS*c.relativeSteeringLimitDeg;
    const constant=(a,b)=>Array.from({length:4},()=>[a,b]);
    const seeds=[this.plan?[...this.plan.controls.slice(1),this.plan.controls.at(-1)]:null,
      constant(referenceThrottle,aim),constant(this.applied.throttle,aim),
      // Coupled low-thrust arc seeds avoid the neutral coordinate-search trap.
      ...[.05,.15,.3,.5,1].map(f=>constant(throttleLimit*f,aim)),constant(0,0)].filter(Boolean);
    let best=null,deadlineReached=false;
    const consider=seed=>{const q=this.project(seed,dt),v=rollout(q);if(v&&(!best||v.cost<best.cost))best=v;};
    // Bounded direct shooting, nonlinear rollout at every candidate, warm
    // start, and first-control-only application. Keep an already-audited best
    // candidate if the search budget expires; never reuse an old invalid plan.
    for(const seed of seeds){consider(seed);if(performance.now()-started>=o.solve_budget_ms){deadlineReached=true;break;}}
    if(!best)return fail('no_feasible_trajectory');
    search:for(const scale of [1,.5,.25])for(let k=0;k<4;k++)for(let axis=0;axis<2;axis++)for(const sign of [-1,1]){
      if(performance.now()-started>=o.solve_budget_ms){deadlineReached=true;break search;}
      const q=best.controls.map(x=>[...x]);q[k][axis]+=sign*scale*(axis===0?throttleLimit*.2:6);consider(q);
    }
    const solveMs=performance.now()-started;
    if(solveMs>Math.max(30,o.solve_budget_ms*2))return fail('solver_overrun');
    this.plan=best;
    const [throttlePercent,steeringDegrees]=best.controls[0];
    return {feasible:true,reason:'relative_nmpc',mode,throttlePercent,steeringDegrees,
      nominalSpeedMps:surgeTarget(throttlePercent),
      yawRateDegS:steeringDegrees/c.relativeSteeringLimitDeg*c.maxYawRateDegS,
      horizonS:o.horizon_s,solveMs,evaluations,deadlineReached,path:best.path,cost:best.cost,
      minClearanceM:best.minClearanceM,measurementAt:at,relativeMotionConfirmed:rv.trusted,
      effectiveFollowingDistanceM:desired,
      absoluteSpeedObservable:gpsAvailable,modelSurgeMps:u0,modelSurgeMeasured:gpsAvailable};
  }
}
