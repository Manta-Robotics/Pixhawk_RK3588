import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import { UwbFollowController } from '../backend/uwb_follow.js';
import { NeuralUwbFollowController } from '../backend/neural_uwb_follow.js';
import { IntermittentUwbFollow } from '../backend/uwb_target_prediction.js';
const config={execution_mode:'acro',manual_propulsion:'single_side',prediction:{enabled:true},max_speed_mps:2,commissioning_speed_mps:2,
  bearing_sign:1,bearing_scale:1,bearing_offset_deg:0,controller_profile:'traditional_fast',
  max_yaw_rate_deg_s:14,neural_model_path:new URL('../models/smart_usv_uwb_policy.json',import.meta.url).pathname};
const telemetry=(now=1000)=>({armed:true,flightMode:'ACRO',pilotSteering:{ready:true,type:0},
  acroControl:{ready:true,updatedAt:now},attitude:{yaw:0,yawRateDegS:0,updatedAt:now},
  position:{updatedAt:now},ekf:{healthy:true,updatedAt:now},gps:{fixType:3},velocity:{vx:0,vy:0},
  uwb:{online:true,fresh:true,distanceM:8,azimuthDeg:20,measurementUpdatedAt:now,updatedAt:now}});
for(const kind of ['traditional','neural']) {
  const create=()=>kind==='traditional'?new UwbFollowController(config):new NeuralUwbFollowController(config);
  test(`${kind} selects ACRO with GPS present and caps forward speed at 2 m/s`,()=>{
    const c=create();let result;
    for(let now=1000;now<=6000;now+=100)result=c.update({now,telemetry:telemetry(now),connected:true});
    assert.equal(result.safe,true);assert.equal(result.controlMode,'relative_acro');assert.equal(result.requiredFlightMode,'ACRO');
    assert.ok(result.forwardSpeedMps<=2);assert.ok(result.speedLimitMps<=2);assert.ok(Math.abs(result.yawRateDegS)<=14);
  });
  test(`${kind} requires a valid velocity estimate and calibrated ACRO parameters`,()=>{
    const c=create(),t=telemetry();t.gps.fixType=0;
    assert.equal(c.preflight({now:1000,telemetry:t,connected:true}).reason,'acro_velocity_unavailable');
    t.gps.fixType=3;t.acroControl.ready=false;
    assert.equal(c.preflight({now:1000,telemetry:t,connected:true}).reason,'acro_parameters_not_ready');
    t.acroControl.ready=true;t.flightMode='MANUAL';
    assert.equal(c.update({now:1000,telemetry:t,connected:true}).reason,'acro_required');
  });
}
test('prediction accepts ACRO, retains ACRO during waiting and never increases blind propulsion',()=>{
  const c=new UwbFollowController(config),p=new IntermittentUwbFollow({enabled:true},config);
  for(let now=1000;now<=1500;now+=100){const r=p.update({controller:c,now,telemetry:telemetry(now),connected:true,controlMode:'relative_acro'});assert.equal(r.safe,true);}
  const bad=telemetry(2000);bad.uwb.online=false;bad.uwb.fresh=false;bad.uwb.measurementUpdatedAt=1500;
  const result=p.update({controller:c,now:2000,telemetry:bad,connected:true,controlMode:'relative_acro'});
  assert.notEqual(result.reason,'manual_required');assert.equal(result.controlMode,'relative_acro');
  p.governAcro({safe:true,forwardSpeedMps:.1,yawRateDegS:10},2,14,18,70);
  const command=p.governAcro({safe:true,forwardSpeedMps:2,yawRateDegS:14,intermittentThrottleCap:15,
    intermittentSteeringCap:6,targetEstimate:{source:'predicted'}},2,14,18,70);
  assert.ok(command.forwardSpeedMps<=.1);assert.ok(Math.abs(command.yawRateDegS)<=14/3+1e-6);
  p.neutral('waiting',null,true);
  assert.equal(p.governAcro({safe:true,forwardSpeedMps:1,intermittentThrottleCap:15,targetEstimate:{source:'predicted'}},2,14,18,70).forwardSpeedMps,0);
});
test('both server adapters emit physical-unit ROVER_ACRO rather than the manual propulsion mixer',()=>{
  const source=fs.readFileSync(new URL('../backend/server.js',import.meta.url),'utf8');
  const fn=source.slice(source.indexOf('function sendUwbRelativeDrive('),source.indexOf('function holdOutputsAreNeutral('));
  const commands=[];const sandbox={Date,Math,intermittentUwbFollow:new IntermittentUwbFollow(),uwbFollowController:new UwbFollowController(config),
    sendMavlinkCommand:(name,payload)=>commands.push({name,payload}),lastRoverCommandAt:0};
  const context=vm.createContext(sandbox);vm.runInContext(fn,context);
  for(const trackingMethod of ['traditional','neural']){
    context.input={safe:true,controlMode:'relative_acro',trackingMethod,forwardSpeedMps:5,yawRateDegS:30,speedLimitMps:2};
    vm.runInContext('sendUwbRelativeDrive(input)',context);
  }
  assert.equal(commands.length,2);
  for(const {name,payload} of commands){assert.equal(name,'ROVER_ACRO');assert.equal(payload.forwardSpeedMps,2);assert.equal(payload.yawRateDegS,14);}
  context.input={safe:true,controlMode:'relative_acro',trackingMethod:'traditional',reason:'holding_distance',forwardSpeedMps:1,yawRateDegS:14,speedLimitMps:2};
  vm.runInContext('sendUwbRelativeDrive(input)',context);
  assert.equal(commands[2].payload.forwardSpeedMps,0);assert.equal(commands[2].payload.yawRateDegS,0);
});
test('ACRO close approach uses physical low speed without MANUAL minimum-throttle floor',()=>{
  const c=new UwbFollowController({...config,following_distance_m:.5,safety_distance_m:.3,safety_resume_distance_m:1.2,
    close_approach_enabled:true,close_approach_min_speed_mps:.12,close_approach_max_speed_mps:.3,
    relative_min_effective_throttle_percent:35,close_approach_min_throttle_percent:20,close_approach_max_throttle_percent:30});
  let result;
  for(let now=1000;now<=6000;now+=100){const t=telemetry(now);t.uwb.distanceM=.8;t.uwb.azimuthDeg=0;result=c.update({now,telemetry:t,connected:true});}
  assert.equal(result.closeApproachActive,true);assert.ok(result.forwardSpeedMps>=.12 && result.forwardSpeedMps<=.3);
  assert.equal(result.minimumEffectiveThrottleApplied,false);
});
