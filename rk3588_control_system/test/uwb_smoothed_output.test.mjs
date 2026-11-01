import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import {SmoothedUwbOutput} from '../backend/uwb_smoothed_output.js';
import {calibrationUwbObservation} from '../backend/beacon_calibration_input.js';
import {computeBeaconControl} from '../backend/gimbal_beacon.js';
import {UwbFollowController} from '../backend/uwb_follow.js';
import {IntermittentUwbFollow} from '../backend/uwb_target_prediction.js';

const now=10000;
const packet=()=>({online:true,fresh:true,measurementStatus:'trusted',source:'USB_SERIAL',
  streamId:'one',measurementUpdatedAt:now-10,updatedAt:now-10,relativeXM:2,relativeYM:0,
  distanceM:2,azimuthDeg:0,elevationDeg:-60,rawDistanceM:99,rawAzimuthDeg:179,rawElevationDeg:-80,
  activeReceiverIds:[1],receiverCount:1,observingReceiverCount:1,
  smoothing:{ready:true,elevationSmoothed:false,rawStepRmsCm:99,outputSpeedGate:{lastDecision:{distanceM:99}}},
  speedGate:{acceptedCount:10,lastDecision:{xM:99,yM:99}},
  sensors:[{receiverId:1,tagId:5565,valid:true,fresh:true,rawDistanceM:99,rawObservation:{at:now-5,distanceM:99,azimuthDeg:179,elevationDeg:-80}}]});

test('public coordinates use only admitted smoothed output and omit all raw observations',()=>{
  const input=packet(),saved=JSON.stringify(input),u=new SmoothedUwbOutput().view(input,now);
  assert.equal(u.distanceM,2);assert.equal(u.azimuthDeg,0);assert.equal(u.relativeXM,2);
  assert.equal(u.coordinateSource,'smoothed');assert.equal(u.controlSource,'smoothed');
  assert.equal(u.elevationDeg,null);assert.equal(u.elevationValid,false);
  assert.equal(u.smoothing.elevationSmoothed,false);
  assert.doesNotMatch(JSON.stringify(u),/rawDistance|rawAzimuth|rawElevation|rawObservation|lastDecision|rawStep/);
  assert.equal(JSON.stringify(input),saved);
});

for(const [name,change] of [
  ['warming',{smoothing:{ready:false}}],['rejected',{measurementStatus:'unavailable'}],
  ['offline',{online:false}],['not fresh',{fresh:false}],['old clock',{measurementUpdatedAt:now-501}],
  ['future clock',{measurementUpdatedAt:now+1}],['missing XY',{relativeXM:null}],['missing bearing',{azimuthDeg:null}]
]) test(`${name} cannot fall back to raw coordinates`,()=>{
  const u=new SmoothedUwbOutput().view({...packet(),...change},now);
  assert.equal(u.fresh,false);
  for(const k of ['distanceM','azimuthDeg','relativeXM','relativeYM','elevationDeg'])assert.equal(u[k],null);
});

test('verified elevation smoothing follows output clocks; duplicate repaints cannot retarget',()=>{
  const filter=new SmoothedUwbOutput({elevationVerified:true,elevationTauMs:100});
  assert.equal(filter.view({...packet(),elevationDeg:0},now).elevationDeg,0);
  assert.equal(filter.view({...packet(),elevationDeg:80},now).elevationDeg,0);
  const u=filter.view({...packet(),measurementUpdatedAt:now+90,elevationDeg:10},now+100);
  assert.ok(u.elevationDeg>6&&u.elevationDeg<7);
  const reset=filter.view({...packet(),streamId:'two',measurementUpdatedAt:now+100,elevationDeg:20},now+100);
  assert.equal(reset.elevationDeg,20);
});

test('3D calibration refuses unverified elevation and uses smooth coordinates when verified',()=>{
  assert.equal(calibrationUwbObservation(new SmoothedUwbOutput().view(packet(),now),now),null);
  const u=new SmoothedUwbOutput({elevationVerified:true}).view({...packet(),elevationDeg:0},now);
  const s=calibrationUwbObservation(u,now);
  assert.equal(s.distanceM,2);assert.equal(s.uwbAzimuthDeg,0);assert.equal(s.uwbElevationDeg,0);
  assert.equal(s.at,now-10);
  assert.equal(calibrationUwbObservation({...u,measurementUpdatedAt:now-800},now),null);
});

const feedback={checksumValid:true,attitudeValid:true,updatedAt:now-10,yawDeg:5,pitchDeg:7};
const config={control_mode:'absolute_angle',extrinsic_3d:{enabled:true,matrix:[1,0,0,0,-1,0,0,0,-1],translation_m:[0,0,.04]}};
test('installed mount tracks horizontal bearing and holds measured pitch for invalid elevation',()=>{
  const u=new SmoothedUwbOutput().view({...packet(),azimuthDeg:30},now);
  const r=computeBeaconControl({now,uwb:u,feedback,config});
  assert.equal(r.valid,true);assert.equal(r.targetYawDeg,-30);assert.equal(r.targetPitchDeg,7);
  assert.equal(r.pitchHeld,true);assert.equal(r.feedbackRequired,true);assert.equal(r.calibrationModel,'planar_yaw');
});
test('pitch hold gates stale feedback and cannot invent elevation for a tilted or translated mount',()=>{
  const u=new SmoothedUwbOutput().view(packet(),now);
  assert.equal(computeBeaconControl({now,uwb:u,feedback:{...feedback,updatedAt:now-500},config}).valid,false);
  for(const ex of [{...config.extrinsic_3d,translation_m:[.01,0,.04]},
    {...config.extrinsic_3d,matrix:[1,0,.1,0,-1,0,0,0,-1]}]) {
    assert.equal(computeBeaconControl({now,uwb:u,feedback,config:{...config,extrinsic_3d:ex}}).valid,false);
  }
});

test('explicit horizontal hold targets zero instead of retaining a tilted pose, regardless of UWB elevation',()=>{
  for(const control_mode of ['absolute_angle','feedback_rate'])for(const elevationDeg of [-80,0,80]){
    const u=new SmoothedUwbOutput().view({...packet(),elevationDeg},now);
    const r=computeBeaconControl({now,uwb:u,feedback,config:{...config,control_mode,use_pitch:false,hold_pitch_deg:0}});
    assert.equal(r.valid,true);assert.equal(r.targetPitchDeg,0);assert.equal(r.pitchHeld,true);
    if(control_mode==='feedback_rate')assert.ok(r.rateY<0);
  }
});

test('server control path shares smooth telemetry without selecting raw candidates',()=>{
  const source=fs.readFileSync(new URL('../backend/server.js',import.meta.url),'utf8');
  const start=source.indexOf('function stabilizeBeaconUwb('),end=source.indexOf('\nfunction setBeaconAbsoluteCommand',start);
  const u=new SmoothedUwbOutput().view(packet(),now);
  const ctx={systemState:{telemetry:{uwb:u}},directUwbGuidance:true,Date,Number};
  vm.createContext(ctx);vm.runInContext(source.slice(start,end),ctx);
  const result=ctx.sharedUwbControlTelemetry(now).uwb;
  assert.equal(result.controlSource,'smoothed');assert.equal(result.distanceM,2);assert.equal(result.directControl,true);
  assert.doesNotMatch(source,/rawUwbControl|uwb_raw_distance_m|uwb_raw_azimuth_deg/);
});

test('status UI shows unavailable elevation explicitly and removes raw overlay selection',()=>{
  const s=fs.readFileSync(new URL('../frontend/mobile-preview-kimi-k26.html',import.meta.url),'utf8');
  assert.match(s,/仰角未验证/);assert.doesNotMatch(s,/elevation=Number\(uwb\.elevationDeg\)/);
  assert.match(s,/type="checkbox" disabled id="relativeShowRaw"/);
  assert.doesNotMatch(s,/var raw=rawUwbDisplay\.view/);
});

test('switching to smoothed coordinates preserves direct guidance without enabling NMPC or prediction',()=>{
  const config={execution_mode:'relative_imu',bearing_sign:1,bearing_scale:1,bearing_offset_deg:0,
    following_distance_m:.5,safety_distance_m:.3,hard_stop_enabled:false,controller_profile:'traditional_fast',
    max_speed_mps:2,commissioning_mode:false,relative_nmpc:{enabled:true},prediction:{enabled:true}};
  const controllers=[new UwbFollowController(config),new UwbFollowController(config)];
  const sessions=[new IntermittentUwbFollow(config.prediction,config),new IntermittentUwbFollow(config.prediction,config)];
  for(let at=1000;at<=3000;at+=100){
    const u={...new SmoothedUwbOutput().view({...packet(),measurementUpdatedAt:at,
      distanceM:3,azimuthDeg:at<2000?20:-20},at),directControl:true};
    const t={armed:true,flightMode:'MANUAL',pilotSteering:{ready:true,type:0},
      attitude:{yaw:0,yawRateDegS:0,updatedAt:at},gps:{fixType:0},velocity:{vx:null,vy:null},uwb:u};
    const a=sessions[0].update({controller:controllers[0],now:at,telemetry:t,connected:true});
    const b=sessions[1].update({controller:controllers[1],now:at,telemetry:{...t,uwb:{...u,controlSource:'raw'}},connected:true});
    assert.equal(a.safe,true);assert.notEqual(a.planner,'nmpc');assert.equal(a.rawControl,false);
    for(const k of ['distanceM','controlBearingDeg','throttlePercent','steeringDegrees'])assert.equal(a[k],b[k]);
  }
});

test('smoothed direct guidance still stops for a close target, lost data, or disarming',()=>{
  const c=new UwbFollowController({execution_mode:'relative_imu',safety_distance_m:.3});
  const s=new IntermittentUwbFollow({enabled:true},{});
  const t={armed:true,flightMode:'MANUAL',pilotSteering:{ready:true,type:0},
    attitude:{yaw:0,yawRateDegS:0,updatedAt:now},gps:{fixType:0},velocity:{vx:null,vy:null},
    uwb:{...new SmoothedUwbOutput().view({...packet(),distanceM:.2},now),directControl:true}};
  assert.equal(s.update({controller:c,now,telemetry:t,connected:true}).throttlePercent,0);
  t.uwb={...t.uwb,fresh:false,online:false,distanceM:null};
  assert.equal(s.update({controller:c,now,telemetry:t,connected:true}).safe,false);
  t.armed=false;
  assert.equal(s.update({controller:c,now,telemetry:t,connected:true}).throttlePercent,0);
});
