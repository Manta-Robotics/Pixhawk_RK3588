import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import fs from 'node:fs';
import { UwbFollowController } from '../backend/uwb_follow.js';
import { RelativeImuControl } from '../backend/uwb_relative_imu.js';
import { UwbRelativeNmpc } from '../backend/uwb_relative_nmpc.js';
import { UwbGpsSpeedLoop } from '../backend/uwb_gps_speed.js';
import { NeuralUwbFollowController } from '../backend/neural_uwb_follow.js';
import { UwbTargetEstimator, IntermittentUwbFollow } from '../backend/uwb_target_prediction.js';
import { DualThrustRoverLimiter } from '../backend/dual_thrust_rover.js';
import { TraditionalDualMotorLimiter } from '../backend/traditional_dual_motor.js';
import { TraditionalManualPropulsion } from '../backend/traditional_manual_propulsion.js';
import { NeuralManualPropulsion } from '../backend/neural_manual_propulsion.js';
import { UwbAutoResume } from '../backend/uwb_auto_resume.js';
import { UwbGpsDriftHold } from '../backend/uwb_gps_drift_hold.js';
import { UwbRejectNmpc } from '../backend/uwb_reject_nmpc.js';

// The old controllers run only in this differential test. The deployed C++
// executable has no JS/Python engine and loads no old controller modules.
const requests=[], expected=[];
let config, controller, relative, nmpc, gps;
const add=(request,value)=>{requests.push(request);expected.push(JSON.parse(JSON.stringify(value)));};
function reset(raw){
  controller=new UwbFollowController(raw);config=controller.config;
  relative=new RelativeImuControl(config);nmpc=new UwbRelativeNmpc(config);gps=new UwbGpsSpeedLoop(config);
  add({op:'reset',config:raw},config);
}
function telemetry(now,i,gps=true){
  const d=4+.5*Math.sin(i/35), b=12*Math.sin(i/25), yaw=20+3*Math.sin(i/40);
  return {flightMode:'MANUAL',armed:true,pilotSteering:{ready:true,type:0},
    uwb:{online:true,fresh:true,measurementStatus:'trusted',distanceM:d,azimuthDeg:b,measurementUpdatedAt:now,
      activeReceiverIds:[1],sensors:[{receiverId:1,tagId:1,filterAdmissionVersion:1,valid:true,
        measurementStatus:'trusted',measurementUpdatedAt:now,receivedMeasurementAt:now,filterUncertaintyM:.12}]},
    attitude:{yaw,yawRateDegS:.3*Math.cos(i/40),updatedAt:now},gps:{fixType:gps?3:0,updatedAt:now},
    ekf:{healthy:gps,updatedAt:now},position:{updatedAt:now},velocity:{vx:.2,vy:.02}};
}
const production=JSON.parse(fs.readFileSync(new URL('../config/system.config.json',import.meta.url))).uwb_follow;
for(const raw of [{}, production,{...production,relative_nmpc:{enabled:false}}]){
  reset(raw);
  for(let i=0;i<300;i++){
    const now=100000+i*100,t=telemetry(now,i,i<80||i>140),p={distanceM:t.uwb.distanceM,azimuthDeg:t.uwb.azimuthDeg,measuredYawRateDegS:t.attitude.yawRateDegS};
    if(i===100){relative.pause();add({op:'pause'},null);}
    // Complete direct-shooting search for numerical parity. Production keeps
    // its configured 15 ms deadline; deadline fallback is tested separately.
    if(config.relativeNmpc.enabled)continue;
    const input={now,preflight:p,telemetry:t};
    add({op:'relative',...input},relative.update({...input,guidance:controller.traditionalFastGuidance}));
  }
}
reset({...production,relative_nmpc:{...production.relative_nmpc,enabled:true,solve_budget_ms:1000}});
for(let i=0;i<100;i++){
  const now=200000+i*100,t=telemetry(now,i,false);
  const input={now,telemetry:t,range:t.uwb.distanceM,bearing:t.uwb.azimuthDeg,referenceThrottle:20,gpsAvailable:i>70};
  const result=nmpc.solve(input);add({op:'nmpc',...input},result);
  const throttle=result.feasible?result.throttlePercent:0,steering=result.feasible?result.steeringDegrees:0;
  nmpc.trackApplied(throttle,steering);add({op:'nmpc_applied',throttle,steering},null);
}
reset(production);
for(let i=0;i<250;i++){
  const now=300000+i*100,t=telemetry(now,i,i<80||i>130);
  const input={now,telemetry:t,bearingDeg:t.uwb.azimuthDeg,rangeRateMps:.15*Math.sin(i/25),desiredClosingMps:.2,appliedThrottle:20};
  add({op:'gps',...input},gps.update(input));
  gps.trackApplied(19,.1);add({op:'gps_applied',throttle:19,dt:.1},null);
}
for(const raw of [{},{...production,execution_mode:'acro'},{...production,execution_mode:null,manual_propulsion:'single_side'},
    {...production,execution_mode:null,manual_propulsion:'single_side',hard_stop_enabled:true}]){
  reset(raw);
  for(let i=0;i<500;i++){
    const now=400000+i*100,t=telemetry(now,i,i%100<80);
    t.uwb.distanceM=Math.max(.15,2+1.9*Math.sin(i/30));t.uwb.azimuthDeg=65*Math.sin(i/20);
    t.acroControl={ready:true,updatedAt:now};
    t.flightMode=raw.execution_mode==='acro'?'ACRO':raw.manual_propulsion==='single_side'?'MANUAL':t.gps.fixType>=2?'GUIDED':'ACRO';
    if(i%70===20)t.uwb.fresh=false;if(i%100===30)t.armed=false;
    const input={now,telemetry:t,connected:i%125!==20};
    add({op:'preflight',...input},controller.preflight(input));
    add({op:'controller',...input},controller.update(input));
  }
}
for(const execution_mode of ['acro','relative_imu',null]){
  const raw={...production,execution_mode,relative_nmpc:{enabled:false}};
  const neural=new NeuralUwbFollowController(raw);
  add({op:'reset',config:raw,neural:true},neural.config);
  for(let i=0;i<300;i++){
    const now=500000+i*100,t=telemetry(now,i,true);
    t.uwb.distanceM=Math.max(.2,2+1.8*Math.sin(i/30));
    t.flightMode=execution_mode==='acro'?'ACRO':'MANUAL';t.acroControl={ready:true,updatedAt:now};
    const input={now,telemetry:t,connected:true};add({op:'neural',...input},neural.update(input));
  }
}
for(const execution_mode of ['relative_imu','acro'])for(const versioned of [true,false]){
  const config=production.prediction,calibration={...production,execution_mode};
  const estimator=new UwbTargetEstimator(config,calibration);add({op:'estimator_reset',config,calibration},null);
  for(let i=0;i<400;i++){
    const now=600000+i*100,t=telemetry(now,i,true);
    t.uwb.sensors[0].filterAdmissionVersion=versioned?1:0;
    t.uwb.sensors[0].rawDistanceM=t.uwb.distanceM;t.uwb.sensors[0].rawAzimuthDeg=t.uwb.azimuthDeg;
    t.uwb.sensors[0].sequenceId=i;t.uwb.sensors[0].rawDistanceUpdatedAt=now;
    if((i>=80&&i<88)||(i>=160&&i<185)||(i>=250&&i<260)){
      t.uwb.online=t.uwb.fresh=false;t.uwb.measurementStatus='suspect';
      t.uwb.sensors[0].measurementStatus='suspect';t.uwb.measurementUpdatedAt=now-1000;
      t.uwb.sensors[0].measurementUpdatedAt=now-1000;
    }
    add({op:'estimate',now,telemetry:t},estimator.update(now,t));
  }
}
for(const execution_mode of ['relative_imu','acro']){
  const raw={...production,execution_mode,relative_nmpc:{enabled:false}},intermittent=new IntermittentUwbFollow(raw.prediction,raw);
  reset(raw);
  for(let i=0;i<400;i++){
    const now=700000+i*100,t=telemetry(now,i,true);
    t.flightMode=execution_mode==='acro'?'ACRO':'MANUAL';t.acroControl={ready:true,updatedAt:now};
    if(i>=80&&i<88){t.uwb.online=t.uwb.fresh=false;t.uwb.measurementStatus='suspect';t.uwb.measurementUpdatedAt=now-1000;}
    const input={now,telemetry:t,connected:true,controlMode:execution_mode==='acro'?'relative_acro':'relative_manual'};
    const result=intermittent.update({controller,...input});add({op:'intermittent',...input},result);
    if(execution_mode==='acro')add({op:'govern_acro',result,speedLimitMps:2,yawLimitDegS:14,steeringSpan:18,throttleSpan:70},intermittent.governAcro(result,2,14,18,70));
  }
}
for(const traditional of [false,true]){
  const config={throttle_limit:70,steering_limit:18,throttle_span:100,steering_span:45};
  const limiter=traditional?new TraditionalDualMotorLimiter(config):new DualThrustRoverLimiter(config);
  add({op:'motor_reset',config,traditional,now:800000},limiter.reset(800000));
  for(let i=0;i<500;i++){
    const target={throttle:Math.sin(i/15)*90,steering:i%60<30?0:Math.sin(i/17)*30};
    add({op:'motor_target',target},limiter.setTarget(target));add({op:'motor_step',now:800000+i*50},limiter.step(800000+i*50));
  }
  const propulsion=traditional?new TraditionalManualPropulsion(config):new NeuralManualPropulsion(config);
  add({op:'motor_reset',config,traditional,now:900000},propulsion.reset(900000));
  for(let i=0;i<300;i++){
    const now=900000+i*100,t=telemetry(now,i),result={safe:i%80!==20,reason:i%40===0?'holding_distance':'tracking',throttlePercent:Math.sin(i/15)*80,steeringDegrees:Math.sin(i/18)*30};
    if(i%50===10)t.flightMode='ACRO';if(i%60===15)t.pilotSteering.ready=false;
    const input={now,result,telemetry:t,connected:i%70!==30};add({op:'motor_propulsion',...input},propulsion.update(input));
  }
}
for(const execution_mode of ['acro','relative_imu']){
  const config={...production,execution_mode},resume=new UwbAutoResume(config);
  add({op:'resume_reset',config},resume.snapshot());resume.search();add({op:'resume_search'},resume.snapshot());
  for(let i=0;i<100;i++){
    const now=1000000+i*100,t=telemetry(now,i);t.acroControl={ready:true,updatedAt:now};
    if(i%20===7)t.uwb.sensors[0].tagId=2;if(i%20===8)t.uwb.fresh=false;
    const preflight={safe:true,reason:'ready'};
    add({op:'resume_observe',now,telemetry:t,preflight,duringAttempt:true},resume.observe(now,t,preflight,{duringAttempt:true}));
    if(i===10){add({op:'resume_begin'},resume.beginAttempt());resume.started(t,now,true);add({op:'resume_started',telemetry:t,now,automatic:true},resume.snapshot());}
    if(i>=10&&i<=20){const result={forwardSpeedMps:1,throttlePercent:20,desiredSpeedMps:1,velocityNed:{x:.8,y:.2}};add({op:'resume_ramp',now,result},resume.ramp(result,now));}
    if(i===20){resume.stopped('uwb_stale');add({op:'resume_stopped',reason:'uwb_stale'},resume.snapshot());}
    if(i===60){resume.attemptFailed('navigation_stale');add({op:'resume_failed',reason:'navigation_stale'},resume.snapshot());}
  }
}
for(const enabled of [false,true])for(const adaptive of [false,true]){
  const config={...production,execution_mode:'acro',waiting_drift_hold:true,reject_nmpc:{enabled,clearance_policy:adaptive?'adaptive':'legacy',solve_budget_ms:25}};
  const reject=new UwbRejectNmpc(config),drift=new UwbGpsDriftHold(config);
  add({op:'reject_reset',config},null);add({op:'drift_reset',config},null);
  for(let i=0;i<100;i++){
    const now=1100000+i*100,t=telemetry(now,i);t.flightMode='ACRO';t.position={lat:30+i*.00000001,lon:120,updatedAt:now};t.acroControl={ready:true,updatedAt:now};
    t.uwb.sensors[0].rawAzimuthDeg=5;t.uwb.sensors[0].rawDistanceM=i%30===12?0:i%30===13?.25:5;t.uwb.sensors[0].rawDistanceUpdatedAt=now;
    const result={safe:true,distanceM:5,bearingDeg:5,forwardSpeedMps:.2,yawRateDegS:0,targetEstimate:{admissionVersion:1,measurementConfirmed:true,referenceFrame:'local_ned',lastMeasuredAt:now,evaluatedAt:now,relativeBodyM:{x:5,y:.1},uncertaintyM:.12,velocityEstimate:{trusted:true},targetVelocityNedMps:{x:.1,y:0}}};
    if(i%20===0){reject.observe(now,t,result);add({op:'reject_observe',now,telemetry:t,result},null);drift.track(now,t,result);add({op:'drift_track',now,telemetry:t,result},null);}
    const evidence=reject.rangeEvidence(now,t);add({op:'reject_evidence',now,telemetry:t},evidence);
    const direction={trusted:true,at:now,headingDeg:t.attitude.yaw+5,rateDegS:0};
    add({op:'reject_solve',now,telemetry:t,direction,nearRaw:evidence},reject.solve(now,t,direction,evidence));
    const waiting={...result,status:'waiting_target',pausePropulsion:true};
    add({op:'drift_update',now,telemetry:t,result:waiting},drift.update(now,t,waiting));
  }
}
const run=spawnSync(process.argv[2],{input:requests.map(r=>JSON.stringify(r)).join('\n')+'\n',encoding:'utf8',maxBuffer:128*1024*1024,timeout:120000});
assert.equal(run.status,0,run.stderr||String(run.error));
const actual=run.stdout.trim().split('\n').map(JSON.parse);assert.equal(actual.length,expected.length);
function compare(want,got,path=''){
  if(path.endsWith('.solveMs')){assert.ok(Number.isFinite(got)&&got>=0);return;}
  if(typeof want==='number')assert.ok(typeof got==='number'&&Math.abs(want-got)<=1e-8*Math.max(1,Math.abs(want)),`${path}: ${want} != ${got}`);
  else if(want&&typeof want==='object'){
    assert.deepEqual(Object.keys(got).sort(),Object.keys(want).sort(),`${path}: fields`);
    for(const key of Object.keys(want))compare(want[key],got[key],`${path}.${key}`);
  }else assert.equal(got,want,path);
}
for(let i=0;i<actual.length;i++){
  try{compare(expected[i],actual[i]);}
  catch(error){
    fs.writeFileSync(new URL('../control/build/follow-failure.json',import.meta.url),JSON.stringify({i,request:requests[i],expected:expected[i],actual:actual[i],previous:actual.slice(Math.max(0,i-2),i)},null,2));
    throw new Error(`Native follow case ${i} ${requests[i].op}: ${error.message}`,{cause:error});
  }
}
console.log(`Native follow: ${actual.length} configuration, relative control, GPS and NMPC cases passed`);
