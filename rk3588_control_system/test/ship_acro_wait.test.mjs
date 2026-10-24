// Legacy behavioral baseline; replacement is checked by check_native_*.py/mjs.
import {UwbGpsDriftHold} from '../backend/uwb_gps_drift_hold.js';
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import {IntermittentUwbFollow} from '../backend/uwb_target_prediction.js';
import {UwbFollowController} from '../backend/uwb_follow.js';
import {UwbAutoResume} from '../backend/uwb_auto_resume.js';
const source=fs.readFileSync(new URL('./fixtures/legacy-server-control.txt',import.meta.url),'utf8');
const holding=source.slice(source.indexOf('function maintainUwbHoldingMode('),source.indexOf('function maintainUwbResumeTransition('));
const loop=source.slice(source.indexOf('function updateUwbFollow('),source.indexOf('function updateTelemetry('));
const calibration={bearing_sign:1,bearing_scale:1,bearing_offset_deg:0};
function packet(now,valid=true,mode='ACRO') {
  return {armed:true,flightMode:mode,pilotSteering:{ready:true,type:0},acroControl:{ready:true,updatedAt:now},
    attitude:{yaw:0,updatedAt:now},velocity:{vx:0,vy:0},position:{updatedAt:now},gps:{fixType:6},ekf:{healthy:true,updatedAt:now},
    uwb:{online:valid,fresh:valid,distanceM:valid?.9:null,azimuthDeg:valid?0:null,measurementUpdatedAt:valid?now:1200,
      measurementStatus:valid?'trusted':'suspect',activeReceiverIds:valid?[1]:[],
      sensors:[{receiverId:1,sequenceId:valid?now:1200,receivedMeasurementAt:valid?now:1200,
        rawDistanceM:.9,rawAzimuthDeg:0,measurementStatus:valid?'trusted':'suspect',valid,filterUncertaintyM:.2}]}};
}
function setup() {
  const commands=[];
  const controller=new UwbFollowController({...calibration,execution_mode:'acro',prediction:{enabled:true},manual_propulsion:'single_side',
    following_distance_m:.5,safety_distance_m:.3,hard_stop_enabled:false,controller_profile:'traditional_fast'});
  const session=new IntermittentUwbFollow({enabled:true},calibration);
  const sandbox={now:1000,Date:{now:()=>sandbox.now},lastRoverCommandAt:0,uwbGpsDriftHold:new UwbGpsDriftHold(),uwbAutoResume:new UwbAutoResume(),
    systemState:{isConnected:true,telemetry:packet(1000)},uwbFollowState:{active:true,status:'tracking'},
    uwbFollowController:controller,intermittentUwbFollow:session,uwbFollowControlMode:'relative_acro',
    uwbFollowTransitionId:0,uwbFollowTransitionBusy:false,uwbHoldingEnteredAt:0,uwbResumeStartedAt:0,
    sharedUwbControlTelemetry:()=>sandbox.systemState.telemetry,
    requireUwbFollowStartClearance:()=>({safe:false,reason:'idle'}),idleUwbFollowResult:reason=>({safe:false,reason}),
    holdOutputsAreNeutral:()=>false,
    publishUwbFollowState:(r,active)=>sandbox.uwbFollowState={...r,active},
    sendMavlinkCommand:(name,payload)=>commands.push({name,payload}),
    sendUwbFollowStop:reason=>commands.push({name:'STOP',reason}),
    disarmAndReturnManual:reason=>{commands.push({name:'DISARM',reason});sandbox.systemState.telemetry.armed=false;},
    maintainUwbResumeTransition:()=>{throw Error('Must not override an operator mode change');},
    resetUwbHoldTransition:()=>{},sendUwbRelativeDrive:r=>({safe:true,throttle:r.throttlePercent,steering:r.steeringDegrees,sameDirectionSteeringLimit:18}),
    finishUwbTestLog:()=>{},recordUwbTestSample:()=>{},addLog:()=>{}};
  const context=vm.createContext(sandbox);vm.runInContext(holding+'\n'+loop,context);
  const step=(now,valid=true,mode='ACRO')=>{sandbox.now=now;sandbox.systemState.telemetry=packet(now,valid,mode);vm.runInContext('updateUwbFollow()',context);return sandbox.uwbFollowState;};
  for(const t of [1000,1100,1200])assert.equal(step(t).active,true);
  return {commands,sandbox,step};
}
test('production server holds zero ACRO targets during loss and resumes without self-switching to MANUAL',()=>{
  const {commands,sandbox,step}=setup();
  for(let t=1300;t<=2800;t+=100) {
    const r=step(t,false);assert.equal(r.active,true);assert.equal(r.status,'waiting_target');
    assert.equal(r.holdFlightMode,'ACRO');assert.equal(sandbox.systemState.telemetry.flightMode,'ACRO');
    assert.equal(commands.at(-1).name,'ROVER_ACRO');assert.equal(commands.at(-1).payload.forwardSpeedMps,0);
    assert.equal(commands.at(-1).payload.yawRateDegS,0);
  }
  step(2900);step(3000);assert.equal(step(3100).status,'recovering');
  assert.ok(commands.every(c=>c.name!=='SET_MODE' && c.name!=='DISARM'));
});
for(const previous of ['waiting_target','holding'])test(`operator mode change while ${previous} remains terminal`,()=>{
  const {commands,sandbox,step}=setup();step(1300,false);sandbox.uwbFollowState.status=previous;
  assert.equal(step(1400,true,'MANUAL').active,false);
  assert.ok(commands.some(c=>c.name==='DISARM' && c.reason==='acro_required'));
  assert.ok(commands.every(c=>c.name!=='SET_MODE'));
});
