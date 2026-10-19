import {UwbGpsDriftHold} from '../backend/uwb_gps_drift_hold.js';
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import {UwbAutoResume} from '../backend/uwb_auto_resume.js';
import {UwbFollowController} from '../backend/uwb_follow.js';
import {IntermittentUwbFollow} from '../backend/uwb_target_prediction.js';
const config={auto_resume_after_uwb_timeout:true,execution_mode:'acro',bearing_sign:1,bearing_scale:1,bearing_offset_deg:0,
 controller_profile:'traditional_fast',manual_propulsion:'single_side',following_distance_m:.5,safety_distance_m:.3,
 hard_stop_enabled:false,max_speed_mps:2,commissioning_speed_mps:2,max_yaw_rate_deg_s:14,relative_steering_limit_deg:18,
 prediction:{enabled:true,min_samples:3,recovery_blend_ms:800,measurement_gap_ms:450}};
function packet(now,valid=true){return {armed:true,flightMode:'ACRO',pilotSteering:{ready:true,type:0},
 acroControl:{ready:true,updatedAt:now},attitude:{yaw:0,updatedAt:now},position:{updatedAt:now},
 velocity:{vx:0,vy:0},gps:{fixType:3},ekf:{healthy:true,updatedAt:now},
 uwb:{online:valid,fresh:valid,distanceM:2,azimuthDeg:0,measurementUpdatedAt:valid?now:1200,
 measurementStatus:valid?'trusted':'suspect',tagId:7}};}
function pending(){const p=new UwbAutoResume(config);p.started(packet(1000),1000);p.stopped('prediction_wait_timeout');return p;}
test('an authorized UWB timeout retains intent and needs sustained distinct trusted frames before one restart',()=>{
 const p=pending();assert.equal(p.pending,true);
 for(let now=2000;now<2200;now+=100)assert.equal(p.observe(now,packet(now),{safe:true}),false);
 assert.equal(p.observe(2200,packet(2200),{safe:true}),true);
 assert.equal(p.beginAttempt(),true);assert.equal(p.beginAttempt(),false);
});
test('duplicates, a single trusted point, bad navigation, out-of-view target and changed tag do not rearm',()=>{
 const p=pending();const one=packet(2000);
 assert.equal(p.observe(2000,one,{safe:true}),false);
 for(let now=2100;now<=2400;now+=100)assert.equal(p.observe(now,one,{safe:true}),false);
 for(const modify of [m=>m.uwb.azimuthDeg=46,m=>m.uwb.tagId=8,
  m=>m.uwb.measurementUpdatedAt=5000,m=>m.attitude.updatedAt=0,m=>m.uwb.distanceM=.29]) {
  const m=packet(3000);modify(m);assert.equal(p.observe(3000,m,{safe:true}),false);assert.equal(p.samples,0);
 }
 assert.equal(p.observe(3000,packet(3000),{safe:false}),false);
});
test('manual stop, emergency, operator disarm and non-UWB failures revoke the follow request',()=>{
 for(const reason of ['manual_stop','emergency_stop','operator_disarm','command_lease_expired','pixhawk_offline']) {
  const p=pending();p.stopped(reason);assert.equal(p.pending,false);assert.equal(p.requested,false);
  assert.equal(p.observe(4000,packet(4000),{safe:true}),false);
 }
 const p=pending();p.cancel();p.stopped('prediction_wait_timeout');assert.equal(p.pending,false);
});
test('automatic resume defaults off and a failed arming attempt is not retried indefinitely',()=>{
 const off=new UwbAutoResume();off.started(packet(1000),1000);off.stopped('prediction_wait_timeout');assert.equal(off.pending,false);
 const p=pending();p.beginAttempt();p.attemptFailed('uwb_stale');assert.equal(p.pending,true);assert.equal(p.attempting,false);
 p.beginAttempt();p.attemptFailed('arming_timeout');assert.equal(p.pending,false);
});
test('rearming starts with zero propulsion then ramps it while keeping FOV yaw authority',()=>{
 const p=pending();p.started(packet(3000),3000,true);
 const r={forwardSpeedMps:1,throttlePercent:50,yawRateDegS:14,velocityNed:{x:1,y:0}};
 assert.equal(p.ramp(r,3000).forwardSpeedMps,0);assert.equal(p.ramp(r,3400).forwardSpeedMps,.5);
 assert.equal(p.ramp(r,3400).yawRateDegS,14);assert.equal(p.ramp(r,3800).forwardSpeedMps,1);
});
test('trusted returned measurements with inflated KF covariance use an independent window, not the blind-prediction limit',()=>{
 const p=new UwbAutoResume({...config,prediction:{...config.prediction,max_uncertainty_m:.5}});
 p.started(packet(1000),1000);p.stopped('prediction_wait_timeout');
 for(let now=2000;now<=2200;now+=100){
  const m=packet(now);m.uwb.sensors=[{measurementStatus:'trusted',filterUncertaintyM:.95,tagId:7}];
  assert.equal(p.observe(now,m,{safe:true}),now===2200);
 }
 assert.equal(p.p.max_uncertainty_m,.5);
});

const source=fs.readFileSync(new URL('../backend/server.js',import.meta.url),'utf8');
function server(){
 const gate=new UwbAutoResume(config),controller=new UwbFollowController(config);
 const session=new IntermittentUwbFollow(config.prediction,config),commands=[];
 gate.started(packet(1000),1000);
 const box={now:1000,Date:{now:()=>box.now},uwbGpsDriftHold:new UwbGpsDriftHold(),uwbAutoResume:gate,uwbFollowController:controller,
  intermittentUwbFollow:session,uwbFollowState:{active:true,status:'tracking'},uwbFollowControlMode:'relative_acro',
  uwbFollowTransitionId:0,uwbFollowTransitionBusy:false,uwbHoldingEnteredAt:0,uwbResumeStartedAt:0,
  uwbTrackingMethod:'traditional',UWB_FOLLOW_ENABLED:true,systemState:{isConnected:true,telemetry:packet(1000)},
  app:{post:()=>{}},sharedUwbControlTelemetry:()=>box.systemState.telemetry,
  requireUwbFollowStartClearance:()=>controller.preflight({now:box.now,telemetry:box.systemState.telemetry,connected:true}),
  idleUwbFollowResult:(reason,status='stopped',preflight={})=>({...preflight,safe:false,reason,status}),
  publishUwbFollowState:(r,active)=>box.uwbFollowState={...r,...gate.snapshot(),active},
  sendUwbFollowStop:reason=>commands.push(['STOP',reason]),
  disarmAndReturnManual:reason=>{commands.push(['DISARM',reason]);Object.assign(box.systemState.telemetry,{armed:false,flightMode:'MANUAL'});},
  sendMavlinkCommand:(name,payload)=>{commands.push([name,payload]);if(name==='ARM')box.systemState.telemetry.armed=true;
   if(name==='DISARM')box.systemState.telemetry.armed=false;if(name==='SET_MODE')box.systemState.telemetry.flightMode=payload.mode;},
  waitForFollowTelemetry:async(_description,predicate,id)=>{if(id!==box.uwbFollowTransitionId)throw Error('start_cancelled');
   if(_description==='recovered_uwb')for(let i=0;i<3;i++){box.now+=100;box.systemState.telemetry=packet(box.now);if(predicate(box.systemState.telemetry))return;}
   assert.equal(predicate(box.systemState.telemetry),true);},
  sendUwbRelativeDrive:r=>{commands.push(['DRIVE',r.forwardSpeedMps,r.yawRateDegS]);return {safe:true,throttle:r.throttlePercent,steering:r.steeringDegrees};},
  maintainUwbHoldingMode:()=>true,maintainUwbResumeTransition:()=>{},resetUwbHoldTransition:()=>{},
  startUwbTestLog:()=>{},finishUwbTestLog:()=>{},recordUwbTestSample:()=>{},addLog:()=>{}};
 const context=vm.createContext(box);
 const stop=source.slice(source.indexOf('function stopUwbFollow('),source.indexOf('function selectUwbTrackingMethod('));
 const loop=source.slice(source.indexOf('function maintainUwbAutoResume('),source.indexOf('function updateTelemetry('));
 const route=source.slice(source.indexOf("app.post('/api/uwb-follow/start'"),source.indexOf("app.post('/api/uwb-follow/stop'"));
 vm.runInContext(stop+'\n'+loop+'\n'+route,context);
 async function step(now,valid=true){box.now=now;const m=packet(now,valid);
  m.armed=box.systemState.telemetry.armed;m.flightMode=box.systemState.telemetry.flightMode;box.systemState.telemetry=m;
  vm.runInContext('updateUwbFollow()',context);await new Promise(resolve=>setImmediate(resolve));return box.uwbFollowState;}
 return {box,context,commands,step};
}
test('production timeout disarms, collects recovered UWB and reuses the real startup route to rearm once',async()=>{
 const {box,commands,step}=server();
 for(const now of [1000,1100,1200])await step(now);
 await step(1700,false);await step(6700,false);
 assert.equal(box.uwbFollowState.active,false);assert.equal(box.systemState.telemetry.armed,false);
 assert.equal(box.uwbAutoResume.pending,true);assert.equal(box.uwbFollowState.lastStopReason,'prediction_wait_timeout');
 for(let now=6800;now<=7000;now+=100)await step(now);
 assert.equal(box.systemState.telemetry.armed,true);assert.equal(box.uwbFollowState.active,true);
 assert.equal(box.uwbFollowState.status,'recovering');assert.equal(box.uwbFollowState.forwardSpeedMps,0);
 assert.equal(commands.filter(([name])=>name==='ARM').length,1);
 assert.ok(commands.some(([name,v,yaw])=>name==='DRIVE'&&v===0&&yaw===0));
});
test('operator stop while waiting cancels automatic rearming even when good UWB returns',async()=>{
 const {box,context,commands,step}=server();
 for(const now of [1000,1100,1200])await step(now);
 await step(1700,false);await step(6700,false);
 vm.runInContext("stopUwbFollow('manual_stop',{disarm:true})",context);
 for(let now=6800;now<=8500;now+=100)await step(now);
 assert.equal(box.uwbFollowState.active,false);assert.equal(box.uwbAutoResume.pending,false);
 assert.equal(commands.filter(([name])=>name==='ARM').length,0);
});
test('explicit disarm while already disarmed still revokes a pending automatic resume',()=>{
 const {box,context}=server();box.uwbAutoResume.stopped('prediction_wait_timeout');box.systemState.telemetry.armed=false;
 let disarm;box.socket={on:(_name,fn)=>disarm=fn,emit:()=>{}};
 const handler=source.slice(source.indexOf("socket.on('disarm'"),source.indexOf("socket.on('request_telemetry'"));
 vm.runInContext(handler,context);disarm();assert.equal(box.uwbAutoResume.pending,false);
});
test('operator stop during an automatic mode transition cancels the in-flight arm sequence',async()=>{
 const {box,context,commands,step}=server();
 for(const now of [1000,1100,1200])await step(now);
 await step(1700,false);await step(6700,false);
 let release;const blocked=new Promise(resolve=>release=resolve);
 box.waitForFollowTelemetry=async(_name,predicate,id)=>{await blocked;
  if(id!==box.uwbFollowTransitionId)throw Error('start_cancelled');assert.ok(predicate(box.systemState.telemetry));};
 for(let now=6800;now<=7600;now+=100)await step(now);
 assert.equal(box.uwbFollowTransitionBusy,true);
 vm.runInContext("stopUwbFollow('manual_stop',{disarm:true})",context);release();
 await new Promise(resolve=>setImmediate(resolve));
 assert.equal(commands.filter(([name])=>name==='ARM').length,0);
 assert.equal(box.uwbAutoResume.pending,false);assert.equal(box.systemState.telemetry.armed,false);
});

test('one rejected frame does not erase fresh good samples or itself authorize rearm',()=>{
 const p=pending();p.observe(2000,packet(2000),{safe:true});p.observe(2100,packet(2100),{safe:true});
 assert.equal(p.observe(2200,packet(2200,false),{safe:false,reason:'uwb_stale'}),false);
 assert.equal(p.samples,2);
 assert.equal(p.observe(2300,packet(2300),{safe:true}),true);
 p.observe(2800,packet(2800,false),{safe:false,reason:'uwb_stale'});assert.equal(p.samples,0);
});

function useRealWait(s, refresh){
 const {box,context}=s;
 box.setTimeout=(fn,ms)=>{box.now+=ms;refresh(box);setImmediate(fn);};
 const wait=source.slice(source.indexOf('async function waitForFollowTelemetry('),source.indexOf('function sendUwbRelativeDrive('));
 vm.runInContext(wait,context);
}
test('real startup survives UWB loss after ARM ACK, stays neutral, and confirms new samples before activation',async()=>{
 const s=server(),{box,commands,step}=s;
 for(const now of [1000,1100,1200])await step(now);
 await step(1700,false);await step(6700,false);
 let armAt=null;
 const send=box.sendMavlinkCommand;
 box.sendMavlinkCommand=(name,payload)=>{send(name,payload);if(name==='ARM'){
  armAt=box.now;box.systemState.telemetry=packet(box.now,false);
 }};
 useRealWait(s,b=>{const valid=armAt===null||b.now-armAt>=600;
  const m=packet(b.now,valid);m.armed=b.systemState.telemetry.armed;m.flightMode=b.systemState.telemetry.flightMode;b.systemState.telemetry=m;});
 for(const now of [6800,6900,7000])await step(now);
 for(let i=0;i<30&&box.uwbFollowTransitionBusy;i++)await new Promise(r=>setImmediate(r));
 assert.equal(box.uwbFollowState.active,true);assert.equal(box.systemState.telemetry.armed,true);
 assert.ok(box.now-armAt>=800&&box.now-armAt<5000);
 assert.equal(commands.filter(([n])=>n==='ARM').length,1);
 assert.equal(commands.filter(([n])=>n==='DISARM').length,1); // original five-second timeout only
 assert.ok(commands.filter(([n,reason])=>n==='STOP'&&reason==='auto_resume_waiting_fresh_measurement').length>=6);
 assert.ok(commands.slice(commands.findIndex(([n])=>n==='ARM')).filter(([n])=>n==='DRIVE').every(([,v,yaw])=>v===0&&yaw===0));
});
test('real startup with persistent signal loss disarms at the shared five-second deadline and retains authorized intent',async()=>{
 const s=server(),{box,step}=s;
 for(const now of [1000,1100,1200])await step(now);
 await step(1700,false);await step(6700,false);
 const send=box.sendMavlinkCommand;let armAt;
 box.sendMavlinkCommand=(name,payload)=>{send(name,payload);if(name==='ARM'){
  armAt=box.now;box.systemState.telemetry=packet(box.now,false);
 }};
 useRealWait(s,b=>{const m=packet(b.now,false);m.armed=b.systemState.telemetry.armed;m.flightMode=b.systemState.telemetry.flightMode;b.systemState.telemetry=m;});
 for(const now of [6800,6900,7000])await step(now);
 for(let i=0;i<65&&box.uwbFollowTransitionBusy;i++)await new Promise(r=>setImmediate(r));
 assert.equal(box.now-armAt,5000);assert.equal(box.systemState.telemetry.armed,false);
 assert.equal(box.uwbFollowState.active,false);assert.equal(box.uwbAutoResume.pending,true);
 assert.equal(box.uwbFollowState.reason,'auto_resume_signal_timeout');
});
test('neutral transition still rejects close raw range, changed target and stale navigation even when UWB is stale',()=>{
 const p=pending();
 for(const change of [m=>m.position.updatedAt=1,m=>m.uwb.tagId=8,m=>m.uwb.sensors=[{rawDistanceM:.2,rawDistanceUpdatedAt:3000}]]){
  const m=packet(3000,false);change(m);assert.ok(p.transitionFailure(3000,m,{safe:false,reason:'uwb_stale'}));
 }
});
