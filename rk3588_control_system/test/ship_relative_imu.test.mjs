// Legacy behavioral baseline; replacement is checked by check_native_*.py/mjs.
import {UwbGpsDriftHold} from '../backend/uwb_gps_drift_hold.js';
import test from 'node:test';
import {UwbAutoResume} from '../backend/uwb_auto_resume.js';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import {fileURLToPath} from 'node:url';
import {NeuralUwbFollowController} from '../backend/neural_uwb_follow.js';
import {UwbFollowController} from '../backend/uwb_follow.js';
import {IntermittentUwbFollow} from '../backend/uwb_target_prediction.js';
const config={execution_mode:'relative_imu',bearing_sign:1,bearing_scale:1,bearing_offset_deg:0,
 following_distance_m:.5,safety_distance_m:.3,safety_resume_distance_m:1.2,hard_stop_enabled:false,
 max_speed_mps:2,commissioning_speed_mps:2,relative_throttle_limit_percent:70,
 relative_steering_limit_deg:18,max_yaw_rate_deg_s:14,controller_profile:'traditional_fast',
 prediction:{enabled:true}};
function packet(t,d=2,yaw=0,bearing=0){return {armed:true,flightMode:'MANUAL',gps:{fixType:0},
 position:{updatedAt:0},velocity:{vx:null,vy:null},ekf:{healthy:false},pilotSteering:{ready:true,type:0},
 attitude:{yaw,yawRateDegS:0,updatedAt:t},
 uwb:{online:true,fresh:true,distanceM:d,azimuthDeg:bearing,measurementUpdatedAt:t,measurementStatus:'trusted'}};}

test('PPO missing-input experiment has no GPS dependence and labels absolute velocity imputation',()=>{
 const neuralConfig={...config,neural_model_path:fileURLToPath(new URL('../models/smart_usv_uwb_policy.json',import.meta.url))};
 const controllers=[new NeuralUwbFollowController(neuralConfig),new NeuralUwbFollowController(neuralConfig)];
 for(let t=1000;t<=2000;t+=100){const a=packet(t,2),b=structuredClone(a);
  b.velocity={vx:100,vy:-100};b.gps={fixType:6};b.position={updatedAt:t};b.ekf={healthy:true,updatedAt:t};
  const result=[a,b].map((telemetry,i)=>controllers[i].update({now:t,telemetry,connected:true}));
  assert.deepEqual(result[0],result[1]);assert.equal(result[0].neuralAbsoluteVelocityMissing,true);
  assert.equal(result[0].neuralObservation[10],0);assert.equal(result[0].neuralObservation[11],0);
  assert.equal(result[0].absoluteSpeedObservable,false);
 }
 controllers[0].actor.infer=()=>[1,1];
 const stopped=controllers[0].update({now:2100,telemetry:packet(2100,.29),connected:true});
 assert.equal(stopped.throttlePercent,0);assert.equal(stopped.steeringDegrees,0);
});

test('GPS-free controller accepts only UWB/IMU and never presents throttle as absolute velocity',()=>{
 const c=new UwbFollowController(config),m=packet(1000);
 const r=c.update({now:1000,telemetry:m,connected:true});
 assert.equal(r.safe,true);assert.equal(r.controlMode,'relative_manual');
 assert.equal(r.absoluteSpeedObservable,false);assert.equal(r.forwardSpeedMps,null);
 assert.equal(r.navigationAvailable,false);assert.ok(r.throttlePercent>0);
 m.attitude.updatedAt=0;assert.equal(c.preflight({now:1000,telemetry:m,connected:true}).reason,'imu_stale');
});

test('GPS position and ground velocity cannot influence relative estimation or control',()=>{
 const controllers=[new UwbFollowController(config),new UwbFollowController(config)];
 const wrappers=[new IntermittentUwbFollow(config.prediction,config),new IntermittentUwbFollow(config.prediction,config)];
 for(let t=1000;t<=4000;t+=100){
  const a=packet(t,2+.0001*(t-1000)),b=structuredClone(a);
  Object.assign(b,{gps:{fixType:6},velocity:{vx:900,vy:-800},position:{lat:10,lon:20,updatedAt:t},ekf:{healthy:true,updatedAt:t}});
  const results=[a,b].map((telemetry,i)=>wrappers[i].update({controller:controllers[i],now:t,telemetry,connected:true}));
  assert.deepEqual(results[0],results[1]);
  assert.equal(results[0].targetEstimate.referenceFrame,'yaw_stabilized_relative');
  assert.equal(results[0].targetEstimate.targetVelocityNedMps,null);
 }
});

test('IMU rotation compensation does not invent translation while own boat turns',()=>{
 const c=new UwbFollowController(config),p=new IntermittentUwbFollow(config.prediction,config);let r;
 for(let t=1000;t<=4000;t+=100){const yaw=(t-1000)*.005;
  r=p.update({controller:c,now:t,telemetry:packet(t,3,yaw,-yaw),connected:true});}
 assert.equal(r.targetEstimate.velocityEstimate.trusted,true);
 const v=r.targetEstimate.relativeVelocityHeadingMps;
 assert.ok(Math.hypot(v.x,v.y)<1e-8);
});

test('relative velocity loop learns matching thrust with no GPS or speed feedback',()=>{
 const c=new UwbFollowController(config);let d=4,v=0,min=d;const tail=[];
 for(let i=0;i<800;i++){
  const t=1000+100*i,r=c.update({now:t,telemetry:packet(t,d),connected:true});
  v+=Math.max(-.025,Math.min(.025,r.throttlePercent/70*2-v));d+=(.8-v)*.1;min=Math.min(min,d);
  if(i>600)tail.push(d);
 }
 assert.ok(min>.3,`minimum distance ${min}`);assert.ok(d<1,`final distance ${d}`);
 assert.ok(Math.max(...tail)-Math.min(...tail)<.1,`tail range ${Math.min(...tail)}..${Math.max(...tail)}; final ${d}; speed ${v}`);
});

test('unchanged 0.3 m safety guard neutralizes actual throttle without waiting for slew',()=>{
 const c=new UwbFollowController(config);
 for(let t=1000;t<=3000;t+=100)c.update({now:t,telemetry:packet(t,4),connected:true});
 const r=c.update({now:3100,telemetry:packet(3100,.29),connected:true});
 assert.equal(r.throttlePercent,0);assert.equal(r.steeringDegrees,0);
});

test('real wrapper and repeated UWB frames retain stable relative feedback without a long-window velocity lag',()=>{
 const c=new UwbFollowController(config),p=new IntermittentUwbFollow(config.prediction,config);
 let d=4,v=0,measured=4;let at=1000;const tail=[];
 for(let i=0;i<1000;i++){
  const t=1000+i*100;if(i%2===0){measured=d;at=t;}
  const m=packet(t,measured);m.uwb.measurementUpdatedAt=at;
  const r=p.update({controller:c,now:t,telemetry:m,connected:true});assert.equal(r.safe,true);
  v+=Math.max(-.025,Math.min(.025,r.throttlePercent/70*2-v));d+=(.8-v)*.1;
  assert.ok(d>.3);if(i>800)tail.push(d);
 }
 assert.ok(Math.max(...tail)-Math.min(...tail)<.1);assert.ok(d<1);
});

test('GPS-free wait remains neutral and exits at the original 5 second deadline',()=>{
 const c=new UwbFollowController(config),p=new IntermittentUwbFollow(config.prediction,config);
 for(let t=1000;t<=3000;t+=100)p.update({controller:c,now:t,telemetry:packet(t,1),connected:true});
 let r;
 for(let t=3100;t<=8000;t+=100){const m=packet(t,1);m.uwb.online=false;m.uwb.fresh=false;
  r=p.update({controller:c,now:t,telemetry:m,connected:true});
  assert.equal(r.safe,true);assert.equal(r.throttlePercent,0);assert.equal(r.steeringDegrees,0);
 }
 const m=packet(8100);m.uwb.online=false;m.uwb.fresh=false;
 r=p.update({controller:c,now:8100,telemetry:m,connected:true});
 assert.equal(r.safe,false);assert.equal(r.reason,'prediction_wait_timeout');
});

test('production server resumes GPS-free throttle control without velocity-unit assumptions',()=>{
 const source=fs.readFileSync(new URL('./fixtures/legacy-server-control.txt',import.meta.url),'utf8');
 const loop=source.slice(source.indexOf('function updateUwbFollow('),source.indexOf('function updateTelemetry('));
 const c=new UwbFollowController(config),p=new IntermittentUwbFollow(config.prediction,config),logs=[];
 const box={now:1000,Date:{now:()=>box.now},uwbGpsDriftHold:new UwbGpsDriftHold(),uwbAutoResume:new UwbAutoResume(),uwbFollowState:{active:true,status:'tracking',reason:'starting'},
  uwbFollowController:c,intermittentUwbFollow:p,uwbFollowControlMode:'auto',uwbFollowTransitionBusy:false,
  systemState:{isConnected:true,telemetry:packet(1000)},uwbHoldingEnteredAt:0,uwbResumeStartedAt:0,
  sharedUwbControlTelemetry:()=>box.systemState.telemetry,
  publishUwbFollowState:(r,active)=>{box.uwbFollowState={...r,active};},recordUwbTestSample:()=>{},
  sendUwbRelativeDrive:r=>({safe:true,throttle:r.throttlePercent,steering:r.steeringDegrees}),
  sendUwbFollowStop:()=>{},maintainUwbHoldingMode:()=>true,resetUwbHoldTransition:()=>{},addLog:(_tag,s)=>logs.push(s),
  disarmAndReturnManual:()=>{throw Error('Unexpected disarm');}};
 const context=vm.createContext(box);vm.runInContext(loop,context);
 for(let t=1000;t<=5000;t+=100){box.now=t;box.systemState.telemetry=packet(t,2);
  if(t>=3100&&t<=3300){box.systemState.telemetry.uwb.online=false;box.systemState.telemetry.uwb.fresh=false;}
  vm.runInContext('updateUwbFollow()',context);assert.equal(box.uwbFollowState.active,true);
 }
 assert.equal(box.uwbFollowState.status,'tracking');assert.ok(logs.some(s=>s.includes('Relative UWB/IMU tracking; throttle')));
 assert.ok(logs.every(s=>!s.includes('m/s')));
});
