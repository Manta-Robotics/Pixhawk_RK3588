import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import {UwbFollowController} from '../backend/uwb_follow.js';
import {IntermittentUwbFollow} from '../backend/uwb_target_prediction.js';
import {UwbAutoResume} from '../backend/uwb_auto_resume.js';
import {LastUwbPointApproach} from '../backend/uwb_loss_policy.js';

const source=fs.readFileSync(new URL('../backend/server.js',import.meta.url),'utf8');
const config={execution_mode:'relative_imu',bearing_sign:1,bearing_scale:1,bearing_offset_deg:0,
 following_distance_m:.5,safety_distance_m:.3,hard_stop_enabled:false,controller_profile:'traditional_fast',
 relative_throttle_limit_percent:70,relative_steering_limit_deg:18,max_speed_mps:2,uwb_timeout_ms:450,
 navigation_timeout_ms:700,auto_resume_after_uwb_timeout:false,
 prediction:{enabled:true,horizon_ms:1000,max_speed_mps:.35,max_throttle_percent:15},
 relative_nmpc:{enabled:true,model_cruise_speed_mps:.8,surge_time_constant_s:1,surge_map_exponent:.5}};
function functionSource(name){const a=source.indexOf(`function ${name}(`);assert.ok(a>=0,name);
 const b=source.indexOf('\nfunction ',a+1);return source.slice(a,b<0?undefined:b);}
function fixture(){
 let now=10000;const commands=[],logs=[];
 const t={armed:true,flightMode:'MANUAL',pilotSteering:{ready:true,type:3},servoOutputs:{ch1:1500,ch3:1500},
  attitude:{yaw:0,yawRateDegS:0,updatedAt:now},gps:{fixType:0},velocity:{vx:0,vy:0},
  uwb:{online:true,fresh:true,measurementStatus:'trusted',measurementUpdatedAt:now,streamId:'one',
   directControl:true,controlSource:'smoothed',coordinateSource:'smoothed',distanceM:2,azimuthDeg:0,
   transport:{connected:true},activeReceiverIds:[1],sensors:[{receiverId:1,anchorId:5565,tagId:5565}]}};
 const c={Date:{now:()=>now},Number,Math,Boolean,String,console,clamp:(x,a,b)=>Math.max(a,Math.min(b,x)),
  systemState:{isConnected:true,telemetry:t,roverControl:{throttle:0,steering:0}},
  uwbFollowState:{active:true,controlMode:'relative_manual',relativeImuControl:true,status:'tracking'},
  uwbFollowTransitionBusy:false,uwbFollowTransitionId:0,uwbFollowControlMode:'relative_manual',
  uwbTrackingMethod:'traditional',UWB_HOLD_ON_POSITION_LOSS:true,ROVER_COMMAND_TIMEOUT_MS:500,
  ROVER_THROTTLE_MIN:-100,ROVER_THROTTLE_MAX:100,ROVER_STEERING_MIN:-45,ROVER_STEERING_MAX:45,
  ROVER_LEFT_CHANNEL:1,ROVER_RIGHT_CHANNEL:3,PWM_CENTER:1500,UWB_HOLD_PWM_TOLERANCE:25,
  UWB_HOLD_MODE_RETRY_MS:500,UWB_HOLD_NEUTRAL_TIMEOUT_MS:1500,
  uwbHoldingEnteredAt:0,uwbHoldingManualConfirmedAt:0,uwbHoldingModeRequestedAt:0,
  uwbResumeStartedAt:0,uwbResumeModeRequestedAt:0,lastRoverCommandAt:now,
  uwbFollowController:new UwbFollowController(config),intermittentUwbFollow:new IntermittentUwbFollow(config.prediction,config),
  uwbAutoResume:new UwbAutoResume(config),lastUwbPointApproach:new LastUwbPointApproach(config),
  uwbGpsDriftHold:{track(){},reset(){}},traditionalManualPropulsion:{limiter:{config:{steeringShare:.8}}},
  manualRoverSlew:{target:{throttle:0,steering:0},output:{throttle:0,steering:0},step:()=>({throttle:0,steering:0})},
  normalizeRoverControl:x=>({...x,leftPwm:1500+5*x.throttle,rightPwm:1500+5*x.throttle}),
  sharedUwbControlTelemetry:()=>t,addLog:(...x)=>logs.push(x),recordUwbTestSample(){},finishUwbTestLog(){},
  maintainUwbResumeTransition(){throw Error('unexpected mode transition');},
  sendMavlinkCommand:(cmd,p)=>commands.push({cmd,...p}),
  sendUwbFollowStop(){c.systemState.roverControl={throttle:0,steering:0};},
  disarmAndReturnManual:()=>commands.push({cmd:'DISARM'}),
  stopUwbFollow:(reason)=>{commands.push({cmd:'STOP',reason});c.uwbFollowState.active=false;},
  sendManualRoverOutput:x=>{commands.push({cmd:'ROVER_DRIVE',...x});c.systemState.roverControl={...x};return {...x};},
  publishUwbFollowState:(r,active)=>{c.uwbFollowState={...r,active};}};
 vm.createContext(c);
 for(const name of ['holdOutputsAreNeutral','maintainUwbHoldingMode','resetUwbHoldTransition','sendUwbRelativeDrive',
   'updateUwbFollow','updateManualRoverSlew'])vm.runInContext(functionSource(name),c);
 return {c,t,commands,logs,step(fresh){now+=100;t.attitude.updatedAt=now;
  Object.assign(t.uwb,{online:fresh,fresh,rejectionReason:fresh?null:'no_radio_position'});
  if(fresh)t.uwb.measurementUpdatedAt=now;
  c.updateUwbFollow();c.updateManualRoverSlew();},time:()=>now};
}

test('real server loss path estimates briefly, then renews neutral leases without disarming or rearming',()=>{
 const f=fixture();for(let i=0;i<10;i++)f.step(true);
 const lastAt=f.t.uwb.measurementUpdatedAt;let estimated=false,waited=false;
 for(let i=0;i<35;i++){
  f.step(false);const r=f.c.uwbFollowState;assert.equal(r.active,true);assert.equal(f.t.armed,true);
  assert.equal(f.c.lastRoverCommandAt,f.time());assert.equal(f.t.uwb.measurementUpdatedAt,lastAt);
  if(r.lastPointApproach?.active){estimated=true;assert.ok(r.throttlePercent<=15);}
  if(i>=10){waited=true;assert.equal(r.pausePropulsion,true);assert.equal(f.c.systemState.roverControl.throttle,0);}
 }
 assert.ok(estimated&&waited);f.step(true);assert.equal(f.c.uwbFollowState.active,true);
 assert.equal(f.c.uwbFollowState.reason,'tracking');
 assert.equal(f.commands.filter(x=>['ARM','DISARM','STOP'].includes(x.cmd)).length,0);
});

test('a genuine Pixhawk disconnect still stops the session',()=>{
 const f=fixture();f.step(true);f.c.systemState.isConnected=false;f.step(false);
 assert.equal(f.c.uwbFollowState.active,false);assert.equal(f.c.uwbFollowState.reason,'pixhawk_offline');
 assert.equal(f.commands.filter(x=>x.cmd==='DISARM').length,1);
});

function startFixture(){
 const commands=[];let handler;const auto=new UwbAutoResume(config);
 const c={app:{post(_path,fn){handler=fn;}},UWB_FOLLOW_ENABLED:true,UWB_HOLD_ON_POSITION_LOSS:true,
  uwbAutoResume:auto,uwbFollowTransitionBusy:false,uwbFollowState:{active:false},
  intermittentUwbFollow:{reset(){}},uwbFollowController:{reset(){}},uwbGpsDriftHold:{reset(){}},
  lastUwbPointApproach:new LastUwbPointApproach(config),
  requireUwbFollowStartClearance:()=>({safe:false,reason:'uwb_stale'}),
  sendUwbFollowStop(){},disarmAndReturnManual:()=>commands.push('DISARM'),publishUwbFollowState(){}};
 vm.createContext(c);const a=source.indexOf("app.post('/api/uwb-follow/start'");
 const b=source.indexOf("app.post('/api/uwb-follow/stop'",a);vm.runInContext(source.slice(a,b),c);
 const response={code:200,status(n){this.code=n;return this;},json(body){this.body=body;return body;}};
 return {c,auto,commands,response,run:req=>handler(req,response)};
}
test('starting without current position does not queue an automatic arming attempt',async()=>{
 const f=startFixture();await f.run({body:{}});assert.equal(f.response.code,409);
 assert.equal(f.auto.pending,false);assert.equal(f.auto.requested,false);assert.ok(!f.commands.includes('ARM'));
});
test('disabled auto-resume cannot enter an automatic arming transaction',async()=>{
 const f=startFixture();f.auto.pending=f.auto.attempting=true;
 await f.run({body:{},uwbAutoResume:true});assert.equal(f.response.code,409);
 assert.equal(f.response.body.message,'auto_resume_cancelled');assert.equal(f.commands.length,0);
});
