// Legacy behavioral baseline; replacement is checked by check_native_*.py/mjs.
import {UwbGpsDriftHold} from '../backend/uwb_gps_drift_hold.js';
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import {UwbFollowController} from '../backend/uwb_follow.js';
import {IntermittentUwbFollow} from '../backend/uwb_target_prediction.js';
import {UwbAutoResume} from '../backend/uwb_auto_resume.js';
const calibration={bearing_sign:1,bearing_scale:1,bearing_offset_deg:0};
test('ACRO turns toward a small new opposite bearing without manual-mode reversal delay',()=>{
 const c=new UwbFollowController({...config,max_yaw_rate_deg_s:14});
 for(const [now,bearing] of [[1000,12],[1100,-12],[1200,8]]){
  const m=packet(now);m.uwb.azimuthDeg=bearing;
  const r=c.update({now,telemetry:m,connected:true});
  assert.ok(r.yawRateDegS*bearing>0);assert.ok(Math.abs(r.yawRateDegS)<=14);
 }
});
const config={...calibration,execution_mode:'acro',controller_profile:'traditional_fast',manual_propulsion:'single_side',
 following_distance_m:.5,safety_distance_m:.3,safety_resume_distance_m:1.2,hard_stop_enabled:false,
 max_speed_mps:2,commissioning_speed_mps:2,relative_throttle_limit_percent:70,
 relative_throttle_accel_percent_s:30,relative_throttle_decel_percent_s:45,range_gain:1.2,
 prediction:{enabled:true,braking_deceleration_mps2:.3}};
function packet(t,d=4,v=0) {return {armed:true,flightMode:'ACRO',pilotSteering:{ready:true,type:0},
 acroControl:{ready:true,updatedAt:t},attitude:{yaw:0,updatedAt:t},position:{updatedAt:t},
 ekf:{healthy:true,updatedAt:t},gps:{fixType:6},velocity:{vx:v,vy:0},
 uwb:{online:true,fresh:true,distanceM:d,azimuthDeg:0,measurementUpdatedAt:t,measurementStatus:'trusted'}};}

test('physical FOV guard stops translation before 45 degrees and retains both turn directions',()=>{
 for(const bearing of [-35,35]) {
  const c=new UwbFollowController(config);c.throttleCommandPercent=70;c.throttleUpdatedAt=1000;
  const m=packet(1100);m.uwb.azimuthDeg=bearing;
  const r=c.update({now:1100,telemetry:m,connected:true});
  assert.equal(r.forwardSpeedMps,0);assert.equal(r.fovSpeedScale,0);
  assert.ok(r.yawRateDegS*bearing>0);
 }
});

test('new edge observation overrides bearing-filter and stored propulsion lag',()=>{
 const c=new UwbFollowController(config);
 c.update({now:1000,telemetry:packet(1000),connected:true});
 c.throttleCommandPercent=70;
 const m=packet(1100);m.uwb.azimuthDeg=38;
 const r=c.update({now:1100,telemetry:m,connected:true});
 assert.ok(r.controlBearingDeg<20);assert.equal(r.forwardSpeedMps,0);assert.ok(r.yawRateDegS>0);
});

test('confirmed inertial LOS rotation is followed without a persistent bearing error',()=>{
 const c=new UwbFollowController(config);let r;
 const rate=.25/2*180/Math.PI;
 for(let now=1000;now<=5000;now+=100) {
  const m=packet(now,2,.8);m.attitude.yawRateDegS=rate;
  r=c.update({now,telemetry:m,connected:true,targetMotionEstimate:{evaluatedAt:now,
   velocityEstimate:{trusted:true},targetVelocityNedMps:{x:.8,y:.25}}});
 }
 assert.ok(Math.abs(r.lineOfSightRateDegS-rate)<1e-9);
 assert.ok(Math.abs(r.yawRateDegS-rate)<1e-9);assert.equal(r.fovSpeedScale,1);
});

test('FOV reacquisition immediately restores bounded ACRO yaw authority without a second comfort ramp',()=>{
 const c=new UwbFollowController({...config,max_yaw_rate_deg_s:14,relative_steering_limit_deg:18});
 c.lastSteeringSign=-1;c.steeringCommandDeg=0;c.steeringUpdatedAt=1000;
 const m=packet(1100);m.uwb.azimuthDeg=30;
 const r=c.update({now:1100,telemetry:m,connected:true});
 assert.equal(r.steeringDegrees,18);assert.equal(r.yawRateDegS,14);
});

test('confirmed target motion gives steady same-speed feed-forward despite small UWB range jitter',()=>{
 const c=new UwbFollowController(config);let r;
 for(let i=0;i<40;i++) {
  const now=1000+i*100;
  r=c.update({now,telemetry:packet(now,1.3+(i%2?.01:-.01),.25),connected:true,
   targetMotionEstimate:{evaluatedAt:now,velocityEstimate:{trusted:true},targetVelocityNedMps:{x:.25,y:0}}});
  assert.equal(r.speedFeedForwardSource,'confirmed_motion');
  assert.equal(r.targetRadialSpeedMps,.25);assert.equal(r.rangeRateMps,0);
 }
});

test('unconfirmed short motion cannot inject target speed into ACRO feed-forward',()=>{
 const c=new UwbFollowController(config);
 c.update({now:1000,telemetry:packet(1000,2),connected:true});
 const r=c.update({now:1100,telemetry:packet(1100,2.4),connected:true,
  targetMotionEstimate:{evaluatedAt:1100,velocityEstimate:{trusted:false},targetVelocityNedMps:{x:3,y:0}}});
 assert.equal(r.targetRadialSpeedMps,0);assert.equal(r.speedFeedForwardSource,'position_only');
});

function movingUpdate(c,t,d,speed=1.2,own=speed) {
 return c.update({now:t,telemetry:packet(t,d,own),connected:true,
  targetMotionEstimate:{evaluatedAt:t,velocityEstimate:{trusted:true},targetVelocityNedMps:{x:speed,y:0}}});
}

test('at requested separation a confirmed moving target is matched instead of forced to stop',()=>{
 const c=new UwbFollowController(config);let r;
 for(let t=1000;t<=5000;t+=100)r=movingUpdate(c,t,.5);
 assert.equal(r.relativeSpeedTracking,true);assert.equal(r.distanceHoldActive,true);
 assert.equal(r.relativeSpeedCorrectionMps,0);assert.ok(Math.abs(r.forwardSpeedMps-1.2)<1e-9);
});

test('crossing 1.2 m has continuous pursuit speed and caps closing speed rather than target speed',()=>{
 const values=[1.1999,1.2,1.2001].map(d=>{
  const c=new UwbFollowController(config);return movingUpdate(c,1000,d,.8,.8);
 });
 assert.ok(values.every(r=>r.desiredForwardSpeedMps>=.8));
 assert.ok(Math.abs(values[2].desiredForwardSpeedMps-values[0].desiredForwardSpeedMps)<.001);
 assert.ok(values.every(r=>r.relativeSpeedCorrectionMps<=r.relativeApproachLimitMps+1e-9));
 for(const boundary of [.55,.62,1.2]) {
  const limits=[boundary-1e-5,boundary+1e-5].map(d=>movingUpdate(new UwbFollowController(config),1000,d,.8,.4).relativeApproachLimitMps);
  assert.ok(Math.abs(limits[1]-limits[0])<.0001,`approach bound discontinuity at ${boundary}`);
 }
});

test('stationary target, safety crossing and large bearing retain braking authority',()=>{
 const stopped=new UwbFollowController(config);let r;
 for(let t=1000;t<=3000;t+=100)r=movingUpdate(stopped,t,.5,0,0);
 assert.equal(r.forwardSpeedMps,0);
 const close=new UwbFollowController(config);close.throttleCommandPercent=70;close.throttleUpdatedAt=1000;
 r=movingUpdate(close,1100,.29,1.2,1.3);
 assert.ok(r.desiredForwardSpeedMps<0);assert.ok(r.forwardSpeedMps<=0);
 const turn=new UwbFollowController(config);const m=packet(1000,1,1.2);m.uwb.azimuthDeg=60;
 r=turn.update({now:1000,telemetry:m,connected:true,
  targetMotionEstimate:{evaluatedAt:1000,velocityEstimate:{trusted:true},targetVelocityNedMps:{x:1.2,y:0}}});
 assert.equal(r.turnAlignmentScale,0);assert.equal(r.forwardSpeedMps,0);
});

test('moving target remains at steady separation with delayed physical velocity response',()=>{
 const c=new UwbFollowController(config);let d=1.05,v=.8;const commands=[];const tail=[];let minimum=d;
 for(let i=0;i<500;i++) {
  const r=movingUpdate(c,1000+i*100,d,.8,v);commands.push(r.forwardSpeedMps);
  const requested=commands[Math.max(0,commands.length-4)];
  v+=Math.max(-.03,Math.min(.03,requested-v));d+=(.8-v)*.1;minimum=Math.min(minimum,d);
  if(i>=300)tail.push(d);
 }
 assert.ok(minimum>.3,`minimum ${minimum}`);
 assert.ok(Math.max(...tail)-Math.min(...tail)<.05,`steady oscillation ${Math.min(...tail)}..${Math.max(...tail)}`);
 assert.ok(Math.abs(d-.5)<.2,`final separation ${d}`);
});
test('ACRO braking limits the final request before close approach despite a high stored command',()=>{
 const c=new UwbFollowController(config);c.throttleCommandPercent=70;c.throttleUpdatedAt=1000;
 const r=c.update({now:1100,telemetry:packet(1100,2.5,1.58),connected:true});
 assert.equal(r.closeApproachActive,false);assert.equal(r.brakingActive,true);
 assert.ok(r.forwardSpeedMps<1);assert.ok(r.forwardSpeedMps<=r.brakingSpeedLimitMps+1e-9);
 assert.ok(r.brakingDistanceM>r.brakingClearanceM);
});
test('near target cannot force a positive minimum approach velocity over a stop/reverse request',()=>{
 const c=new UwbFollowController(config);
 const r=c.update({now:1000,telemetry:packet(1000,.56,.6),connected:true});
 assert.ok(r.forwardSpeedMps<=.01);assert.ok(r.desiredForwardSpeedMps<=.01);
});
test('braking envelope keeps separation with slow physical deceleration and delayed FCU response',()=>{
 const c=new UwbFollowController(config);let d=4,v=0,min=4;const commands=[];
 for(let i=0;i<400;i++) {
  const t=1000+i*100,r=c.update({now:t,telemetry:packet(t,d,v),connected:true});
  commands.push(r.forwardSpeedMps);const requested=commands[Math.max(0,commands.length-6)];
  v+=Math.max(-.03,Math.min(.03,requested-v));d+=(.22-v)*.1;min=Math.min(min,d);
 }
 assert.ok(min>.3,`minimum range ${min}`);assert.ok(d<1.5,`final range ${d}`);
});
test('UWB waiting clears stored propulsion, then ramps up after independent recovery',()=>{
 const c=new UwbFollowController(config),p=new IntermittentUwbFollow(config.prediction,calibration);
 for(let t=1000;t<=1500;t+=100)p.update({controller:c,now:t,telemetry:packet(t),connected:true});
 assert.ok(c.throttleCommandPercent>0);
 const missing=packet(1700,4,.6);missing.uwb.online=false;missing.uwb.fresh=false;missing.uwb.measurementUpdatedAt=1500;
 const r=p.update({controller:c,now:1700,telemetry:missing,connected:true});
 assert.equal(r.status,'waiting_target');assert.equal(c.throttleCommandPercent,0);assert.equal(c.rangeIntegralMps,0);
 for(const t of [1800,1900,2000])p.update({controller:c,now:t,telemetry:packet(t),connected:true});
 assert.ok(c.throttleCommandPercent<=6);assert.ok(p.estimator.lossAt===null);
});

test('confirmed ACRO recovery turns independently of the cautious propulsion ramp',()=>{
 const c=new UwbFollowController({...config,max_yaw_rate_deg_s:14,relative_steering_limit_deg:18,
   steering_slew_deg_s:90}),p=new IntermittentUwbFollow(config.prediction,calibration);
 for(let t=1000;t<=1400;t+=100){const m=packet(t);m.uwb.azimuthDeg=25;
   p.update({controller:c,now:t,telemetry:m,connected:true});}
 const missing=packet(1500,4,.6);missing.uwb.online=false;missing.uwb.fresh=false;
 missing.uwb.measurementUpdatedAt=1400;p.update({controller:c,now:1500,telemetry:missing,connected:true});
 let r;
 for(let t=1600;t<=2000;t+=100){const m=packet(t);m.uwb.azimuthDeg=25;
   r=p.update({controller:c,now:t,telemetry:m,connected:true});}
 assert.equal(r.status,'recovering');assert.equal(r.targetEstimate.usable,true);
 const output=p.governAcro(r,2,14,18,70);
 assert.ok(output.forwardSpeedMps<=.35);assert.ok(output.yawRateDegS>14/3);
 const lost=p.neutral('prediction_limit',null,true);
 assert.deepEqual(p.governAcro(lost,2,14,18,70),{forwardSpeedMps:0,yawRateDegS:0});
});

test('ACRO faces a target near the edge of view before accelerating forward',()=>{
 const c=new UwbFollowController({...config,max_yaw_rate_deg_s:14,relative_steering_limit_deg:18});
 let r;
 for(let t=1000;t<3000;t+=100){const m=packet(t);m.uwb.azimuthDeg=60;
  r=c.update({now:t,telemetry:m,connected:true});}
 assert.equal(r.turnAlignmentScale,0);assert.equal(r.forwardSpeedMps,0);
 assert.ok(r.yawRateDegS>0);
});

test('production start establishes a neutral ACRO command before exposing an active watchdog lease',async()=>{
 const source=fs.readFileSync(new URL('./fixtures/legacy-server-control.txt',import.meta.url),'utf8');
 const route=source.slice(source.indexOf("app.post('/api/uwb-follow/start'"),source.indexOf("app.post('/api/uwb-follow/stop'"));
 const events=[];let handler;
 const box={app:{post:(_path,fn)=>{handler=fn;}},UWB_FOLLOW_ENABLED:true,uwbGpsDriftHold:new UwbGpsDriftHold(),uwbAutoResume:new UwbAutoResume(),
  uwbFollowTransitionBusy:false,uwbFollowState:{active:false},uwbFollowTransitionId:0,uwbFollowControlMode:'auto',
  uwbTrackingMethod:'traditional',Date,systemState:{isConnected:true,telemetry:{armed:false}},
  intermittentUwbFollow:{reset:()=>{}},uwbFollowController:{reset:()=>{},update:()=>({safe:true,controlMode:'relative_acro',speedLimitMps:2})},
  requireUwbFollowStartClearance:()=>({safe:true,controlMode:'relative_acro',requiredFlightMode:'ACRO'}),
  sharedUwbControlTelemetry:()=>({}),idleUwbFollowResult:()=>({}),sendUwbFollowStop:()=>{},
  publishUwbFollowState:(_r,active)=>{if(active)events.push('active');},sendMavlinkCommand:()=>{},
  waitForFollowTelemetry:async()=>{},startUwbTestLog:()=>{},addLog:()=>{},
  sendUwbRelativeDrive:r=>{assert.equal(r.forwardSpeedMps,0);assert.equal(r.yawRateDegS,0);events.push('neutral');}};
 vm.runInContext(route,vm.createContext(box));let response;
 await handler({body:{}},{json:r=>response=r,status:()=>{throw Error('Unexpected startup failure');}});
 assert.equal(response.success,true);assert.deepEqual(events,['neutral','active']);
});
