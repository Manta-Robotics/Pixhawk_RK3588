import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import {fileURLToPath} from 'node:url';
import {UwbFollowController} from '../backend/uwb_follow.js';
import {NeuralUwbFollowController} from '../backend/neural_uwb_follow.js';
import {IntermittentUwbFollow} from '../backend/uwb_target_prediction.js';
import {validateSystemConfig} from '../scripts/validate_config.mjs';

const config={execution_mode:'relative_imu',relative_nmpc:{enabled:true},prediction:{enabled:true},
  gps_speed_closed_loop:{enabled:true,acquire_ms:2000,kp:20,ki:8,maximum_observed_speed_mps:5},
  bearing_sign:1,bearing_scale:1,bearing_offset_deg:0,following_distance_m:.5,safety_distance_m:.3,
  hard_stop_enabled:false,max_speed_mps:2,commissioning_speed_mps:2,
  relative_throttle_limit_percent:70,relative_steering_limit_deg:18,
  max_yaw_rate_deg_s:14,relative_throttle_accel_percent_s:30,relative_throttle_decel_percent_s:45,
  steering_slew_deg_s:9,controller_profile:'traditional_fast'};
function frame(t,{range=3,bearing=20,yaw=0,rate=0,gps=false,vx=0,vy=0,at=t}={}) {
  return {armed:true,flightMode:'MANUAL',pilotSteering:{ready:true,type:0},
    attitude:{yaw,yawRateDegS:rate,updatedAt:t},gps:{fixType:gps?3:1,updatedAt:t},
    ekf:{healthy:gps,updatedAt:t},position:{updatedAt:t},velocity:{vx:gps?vx:null,vy:gps?vy:null},
    uwb:{online:true,fresh:true,distanceM:range,azimuthDeg:bearing,measurementStatus:'trusted',
      measurementUpdatedAt:at,activeReceiverIds:[1],sensors:[{receiverId:1,tagId:5313,sequenceId:42,
        filterAdmissionVersion:1,valid:true,measurementStatus:'trusted',measurementUpdatedAt:at,
        receivedMeasurementAt:at,rawDistanceM:range,rawAzimuthDeg:bearing,filterUncertaintyM:.2}]}};
}
const update=(c,t,options)=>c.update({now:t,telemetry:frame(t,options),connected:true});
function forward(r) {
  const turn=r.steeringDegrees/45*100;
  assert.ok(r.throttlePercent+turn>=-1e-9);assert.ok(r.throttlePercent-turn>=-1e-9);
}

test('relative NMPC optimizes a nonlinear horizon and retains forward thrust through ordinary left/right arcs',()=>{
  const c=new UwbFollowController(config);let moving=0;
  for(let i=0;i<110;i++) {
    const t=1000+i*100,bearing=25*Math.cos(i/35),r=update(c,t,{bearing});forward(r);
    assert.equal(r.planner,'nmpc');assert.equal(r.absoluteSpeedObservable,false);
    if(r.nmpcActive) {
      moving++;assert.equal(r.nmpc.path.length,12);assert.ok(r.nmpc.evaluations>=1);
      assert.ok(r.nmpc.path.every(p=>p.clearanceM>0&&Math.abs(p.bearingDeg)<=45));
      assert.ok(r.throttlePercent>0,`ordinary arc at sample ${i} stopped`);
    }
  }
  assert.ok(moving>=100,`feasible arc samples ${moving}`);
});

test('GPS NMPC confirms navigation, uses measured surge, and immediately falls back to relative NMPC',()=>{
  const c=new UwbFollowController(config);let active=false;
  for(let i=0;i<60;i++) {
    const t=1000+i*100,r=update(c,t,{gps:i<40,vx:.2,bearing:10});forward(r);
    if(i>=22&&i<40) {
      assert.equal(r.nmpcMode,'gps_imu');assert.equal(r.gpsSpeedControlActive,true);
      assert.equal(r.nmpc.modelSurgeMeasured,true);assert.equal(r.nmpc.modelSurgeMps,.2);active=true;
    }
    if(i>=40) {
      assert.equal(r.nmpcMode,'uwb_imu_relative');assert.equal(r.gpsSpeedControlActive,false);
      assert.equal(r.absoluteSpeedObservable,false);assert.equal(r.nmpc.modelSurgeMeasured,false);
      assert.equal(r.measuredForwardSpeedMps,null);
    }
  }
  assert.ok(active);
});

test('new admission clocks advance despite frozen diagnostic sequence; stale/duplicate clocks do not',()=>{
  const c=new UwbFollowController(config),p=new IntermittentUwbFollow(config.prediction,config);let r;
  for(let t=1000;t<=3000;t+=100) {
    r=p.update({controller:c,now:t,telemetry:frame(t),connected:true});
    assert.equal(r.targetEstimate.lastMeasuredAt,t);
  }
  assert.equal(r.nmpcActive,true);assert.ok(r.throttlePercent>0);
  const at=c.relativeImuControl.nmpc.measurementAt;
  update(c,3100,{at:3000});assert.equal(c.relativeImuControl.nmpc.measurementAt,at);
  const stale=update(c,4100,{at:3000});assert.equal(stale.safe,false);assert.equal(stale.throttlePercent,0);
});

test('close separation, rejected navigation, target changes and invalid observations cannot authorize an NMPC surge',()=>{
  for(const mutate of [t=>{t.armed=false;},t=>{t.attitude.updatedAt=0;},
    t=>{t.uwb.online=false;},t=>{t.uwb.distanceM=.29;}]) {
    const c=new UwbFollowController(config);for(let t=1000;t<=2000;t+=100)update(c,t,{});
    const m=frame(2100);mutate(m);const r=c.update({now:2100,telemetry:m,connected:true});
    assert.equal(r.throttlePercent,0);assert.equal(r.steeringDegrees,0);
  }
  const c=new UwbFollowController(config);for(let t=1000;t<=2000;t+=100)update(c,t,{});
  const m=frame(2100);m.uwb.sensors[0].tagId=999;
  const r=c.update({now:2100,telemetry:m,connected:true});
  assert.equal(r.throttlePercent,0);assert.equal(r.nmpc.reason,'target_identity_changed');
});

test('actuator caps feed back into the next plan instead of retaining an imaginary high thrust',()=>{
  const c=new UwbFollowController(config);for(let t=1000;t<=4000;t+=100)update(c,t,{bearing:0});
  c.relativeImuControl.trackApplied(2,.2);
  const r=update(c,4100,{bearing:10});assert.ok(r.throttlePercent<=5+1e-9);forward(r);
  c.relativeImuControl.pause();assert.equal(c.relativeImuControl.nmpc.applied.throttle,0);
});

test('NMPC configuration rejects incompatible execution and invalid model/search bounds',()=>{
  const original=JSON.parse(fs.readFileSync(new URL('../config/system.config.json',import.meta.url)));
  for(const options of [{enabled:true,horizon_s:0},{enabled:true,solve_budget_ms:100},
    {enabled:true,steering_share:1.1},{enabled:true,model_cruise_speed_mps:0},{enabled:'yes'}]) {
    const cfg=structuredClone(original);cfg.uwb_follow.relative_nmpc=options;
    assert.ok(validateSystemConfig(cfg).length);
  }
  const cfg=structuredClone(original);cfg.uwb_follow.relative_nmpc={enabled:true};
  assert.deepEqual(validateSystemConfig(cfg),[]);
});

test('closed-loop arcs stay bounded with a different nonlinear plant and intermittent GPS',()=>{
  for(const gpsMode of ['absent','present','intermittent']) {
    const c=new UwbFollowController({...config,max_speed_mps:.8,commissioning_speed_mps:.8});
    let x=0,y=0,psi=0,u=0,r=0,minRange=Infinity,maxRange=0,opposed=0,plans=0,neutral=0,turnNeutral=0,maximumSolveMs=0;
    const reasons={},failures=[];
    for(let i=0;i<700;i++) {
      const now=1000+i*100,time=i*.1;
      // Independent nonlinear thrust/drag plant, mismatched gains/lags and
      // weak variable current. Only simulated sensor observations enter c.
      const target={x:3+.22*time,y:1.2*Math.sin(time/10)};
      const dx=target.x-x,dy=target.y-y,range=Math.hypot(dx,dy);
      const bearing=((Math.atan2(dy,dx)-psi)*180/Math.PI+540)%360-180;
      const gps=gpsMode==='present'||(gpsMode==='intermittent'&&i>=200&&i<450);
      const vx=u*Math.cos(psi)+.025*Math.sin(time/7),vy=u*Math.sin(psi);
      const q=update(c,now,{range:range+.008*Math.sin(i*.7),bearing:bearing+.15*Math.cos(i*.3),
        yaw:psi*180/Math.PI,rate:r,gps,vx,vy});
      const left=q.throttlePercent+q.steeringDegrees/45*100,right=q.throttlePercent-q.steeringDegrees/45*100;
      if(left*right<-.00001)opposed++;
      if(q.nmpcActive)plans++;
      if(i>10&&q.throttlePercent===0)neutral++;
      // Distance/braking stops remain legitimate. Detect the reported turn
      // stop specifically when separation exceeds the prediction reserve.
      if(i>10&&Math.abs(bearing)>10&&range>(q.nmpc?.effectiveFollowingDistanceM??Infinity)+.2&&q.throttlePercent===0)turnNeutral++;
      maximumSolveMs=Math.max(maximumSolveMs,q.nmpc?.solveMs??0);
      reasons[q.reason]=(reasons[q.reason]||0)+1;
      if(!q.nmpcActive&&i>10&&(failures.length<4||i%100===0))failures.push({i,range,bearing,u,r,reason:q.reason});
      const force=(Math.max(0,left)+Math.max(0,right))/140;
      u+=clamp(.45*force-.7*u*Math.abs(u),-.25,.25)*.1;
      const turn=(left-right)*.16;
      r+=(turn-r)*(.1/.6);psi+=r*Math.PI/180*.1;x+=vx*.1;y+=vy*.1;
      minRange=Math.min(minRange,range);maxRange=Math.max(maxRange,range);
    }
    console.log(JSON.stringify({gpsMode,plans,minRange,maxRange,neutral,turnNeutral,maximumSolveMs,reasons,failures}));
    assert.equal(opposed,0);assert.ok(plans>=690,JSON.stringify({gpsMode,plans,minRange,maxRange,reasons,failures}));
    assert.equal(turnNeutral,0,'a turn with sufficient reserve must not use stop/turn/restart');
    assert.ok(minRange>.3,`${gpsMode}: minimum ${minRange}`);
    assert.ok(maxRange<7,`${gpsMode}: maximum ${maxRange}`);
  }
});
const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));

test('NMPC navigation quality failures immediately remove measured-speed authority',()=>{
  for(const mutate of [t=>{t.gps.fixType=1;},t=>{t.gps.updatedAt=0;},
    t=>{t.ekf.healthy=false;},t=>{t.position.updatedAt=0;},
    t=>{t.velocity.vx=NaN;},t=>{t.velocity.vx=6;}]) {
    const c=new UwbFollowController(config);
    for(let t=1000;t<=4000;t+=100)update(c,t,{gps:true,vx:.2});
    const packet=frame(4100,{gps:true,vx:.2});mutate(packet);
    const r=c.update({now:4100,telemetry:packet,connected:true});
    assert.equal(r.gpsSpeedControlActive,false);assert.equal(r.nmpcMode,'uwb_imu_relative');
    assert.equal(r.measuredForwardSpeedMps,null);forward(r);
  }
});

test('budget expiry uses only an already validated plan; an overrun clears it and requests neutral',()=>{
  const source=fs.readFileSync(new URL('../backend/uwb_relative_nmpc.js',import.meta.url),'utf8')
    .replaceAll('export ','')+'\nglobalThis.Nmpc=UwbRelativeNmpc;';
  let tick=0,step=0;
  const box={performance:{now:()=>{tick+=step;return tick;}}};
  vm.runInNewContext(source,box);
  const c=new UwbFollowController(config).config,n=new box.Nmpc(c);
  const solve=t=>n.solve({now:t,telemetry:frame(t,{bearing:10}),range:3,bearing:10,referenceThrottle:15});
  solve(1000);solve(1100);assert.equal(solve(1200).feasible,true);
  step=1;const bounded=solve(1300);
  assert.equal(bounded.feasible,true);assert.equal(bounded.deadlineReached,true);
  assert.ok(bounded.path.every(p=>p.clearanceM>0));
  step=50;const late=solve(1400);assert.equal(late.feasible,false);
  assert.equal(late.reason,'solver_overrun');assert.equal(n.plan,null);
});

test('neural mode shares both NMPC branches and reports the actor as a reference, not the optimizer',()=>{
  const c=new NeuralUwbFollowController({...config,neural_model_path:fileURLToPath(new URL('../models/smart_usv_uwb_policy.json',import.meta.url))});
  for(let i=0;i<60;i++) {
    const t=1000+i*100,r=update(c,t,{gps:i<40,vx:.2,bearing:15});
    assert.equal(r.planner,'nmpc');assert.equal(r.neuralRole,'nominal_throttle_reference');
    assert.equal(r.speedFeedForwardSource,i>=20&&i<40?'gps_state_feedback_nmpc':'relative_state_nmpc');
    forward(r);
  }
});
