import test from 'node:test';
import assert from 'node:assert/strict';
import {LastUwbPointApproach} from '../backend/uwb_loss_policy.js';

const config={following_distance_m:.5,relative_throttle_limit_percent:70,max_yaw_rate_deg_s:14,
 relative_steering_limit_deg:18,steering_slew_deg_s:9,uwb_timeout_ms:450,
 prediction:{horizon_ms:1000,max_speed_mps:.35,max_throttle_percent:15,max_steering_deg:6},
 relative_nmpc:{model_cruise_speed_mps:.8,surge_time_constant_s:1,surge_map_exponent:.5}};
function fixture(){
 let now=1000;
 const model=new LastUwbPointApproach(config);
 const telemetry={armed:true,flightMode:'MANUAL',pilotSteering:{ready:true,type:3},
  attitude:{yaw:0,yawRateDegS:0,updatedAt:now},
  uwb:{online:true,fresh:true,measurementStatus:'trusted',measurementUpdatedAt:now,
   streamId:'one',distanceM:2,azimuthDeg:0,transport:{connected:true},
   sensors:[{receiverId:1,anchorId:5565,tagId:5565}]}};
 const apply=(result={safe:true,reason:'tracking'},changes={})=>model.update({
  enabled:true,result,telemetry,connected:true,now,controlMode:'relative_manual',
  appliedCommand:{throttle:20,steering:0},calibration:{bearingSign:1,bearingScale:1,bearingOffsetDeg:0},...changes});
 const advance=ms=>{now+=ms;telemetry.attitude.updatedAt=now;};
 const lose=()=>{telemetry.uwb={...telemetry.uwb,online:false,fresh:false,rejectionReason:'no_radio_position',distanceM:null,azimuthDeg:null};};
 return {model,telemetry,apply,advance,lose};
}

test('last position is frozen while model displacement advances; estimates do not refresh measurements',()=>{
 const f=fixture();f.apply();const fixed={...f.model.last};f.advance(200);f.apply();
 assert.deepEqual(f.model.last,fixed);
 f.lose();const original=JSON.stringify(f.telemetry.uwb);f.advance(100);
 const first=f.apply({safe:false,reason:'uwb_stale'});
 assert.equal(first.lastPointApproach.active,true);assert.equal(first.targetEstimate.modelBased,true);
 assert.equal(first.targetEstimate.measurementConfirmed,false);assert.equal(first.absoluteSpeedObservable,false);
 assert.ok(first.throttlePercent>0&&first.throttlePercent<=15);assert.ok(Math.abs(first.steeringDegrees)<=6);
 f.advance(100);const second=f.apply({safe:false,reason:'uwb_stale'});
 assert.ok(second.distanceM<first.distanceM);assert.deepEqual(f.model.last,fixed);
 assert.equal(JSON.stringify(f.telemetry.uwb),original);
});

test('heartbeats cannot extend the estimate horizon or restart it after expiry',()=>{
 const f=fixture();f.apply();f.advance(100);f.apply();f.lose();
 for(let i=0;i<20;i++){
  f.advance(100);f.telemetry.uwb.updatedAt=1000+(i+2)*100;
  const r=f.apply({safe:false,reason:'uwb_stale'});
  if(i>=9){assert.equal(r.positionWaiting,true);assert.equal(r.throttlePercent,0);assert.equal(r.pausePropulsion,true);}
  assert.equal(f.model.last.at,1000);
 }
});

test('estimated arrival retains the original following stand-off instead of driving onto the tag',()=>{
 const f=fixture();f.telemetry.uwb.distanceM=.52;f.apply();f.lose();f.advance(100);
 const r=f.apply({safe:false,reason:'uwb_stale'});
 assert.equal(r.lastPointApproach.reason,'estimated_arrival');assert.equal(r.throttlePercent,0);
});

test('a fresh close-range hold replaces the old far point before a later loss',()=>{
 const f=fixture();f.apply();f.advance(100);f.telemetry.uwb.distanceM=.2;f.telemetry.uwb.measurementUpdatedAt=1100;
 f.apply({safe:true,pausePropulsion:true,reason:'safety_distance'});f.lose();f.advance(100);
 assert.equal(f.apply({safe:false,reason:'uwb_stale'}).throttlePercent,0);
});

test('fresh measurements restore ordinary control and may replace the fixed point',()=>{
 const f=fixture();f.apply();f.lose();f.advance(100);f.apply({safe:false,reason:'uwb_stale'});
 f.advance(100);Object.assign(f.telemetry.uwb,{online:true,fresh:true,measurementStatus:'trusted',
  measurementUpdatedAt:1200,distanceM:3,azimuthDeg:20,rejectionReason:null});
 const real={safe:true,reason:'tracking',throttlePercent:5};assert.equal(f.apply(real),real);
 assert.equal(f.model.last.at,1200);assert.equal(f.model.lossThrottle,null);
});

for(const [name,change,reason] of [
 ['disconnected',f=>{},'pixhawk_offline'],
 ['disarmed',f=>f.telemetry.armed=false,'armed_required'],
 ['bad yaw',f=>f.telemetry.attitude.yaw=NaN,'imu_invalid'],
 ['wrong mode',f=>f.telemetry.flightMode='HOLD','manual_required'],
 ['mixer unavailable',f=>f.telemetry.pilotSteering.ready=false,'pilot_steering_not_ready'],
 ['calibrating',f=>f.telemetry.imuCalibration={active:true},'imu_calibration_active'],
])test(`${name} cannot authorize model propulsion`,()=>{
 const f=fixture();f.apply();f.lose();f.advance(100);change(f);
 const r=f.apply({safe:false,reason:'uwb_stale'},name==='disconnected'?{connected:false}:{});
 assert.equal(r.safe,false);assert.equal(r.reason,reason);assert.equal(f.model.last,null);
});

test('stale attitude, identity changes, disconnected UART and invalid range cannot drive the old target',()=>{
 for(const kind of ['imu','identity','usb','range']){
  const f=fixture();f.apply();f.lose();f.advance(100);
  if(kind==='imu')f.telemetry.attitude.updatedAt=0;
  if(kind==='identity')f.telemetry.uwb.sensors[0].tagId=9999;
  if(kind==='usb')f.telemetry.uwb.transport.connected=false;
  if(kind==='range')f.telemetry.uwb.rejectionReason='distance_out_of_range';
  const r=f.apply({safe:false,reason:'uwb_stale'});assert.ok(!r.safe||r.throttlePercent===0,kind);
 }
});

test('IMU rotation changes bearing to the fixed point, never rotates the fixed target with the boat',()=>{
 const f=fixture();f.apply();const point={...f.model.last};f.lose();f.advance(100);f.telemetry.attitude.yaw=90;
 const r=f.apply({safe:false,reason:'uwb_stale'});
 assert.deepEqual(f.model.last,point);assert.equal(r.lastPointApproach.reason,'last_position_outside_forward_sector');
 assert.equal(r.throttlePercent,0);
});

test('no recorded point and a long model scheduling gap both wait neutrally',()=>{
 const f=fixture();f.lose();assert.equal(f.apply({safe:false,reason:'uwb_stale'}).throttlePercent,0);
 const g=fixture();g.apply();g.lose();g.advance(800);
 const r=g.apply({safe:false,reason:'uwb_stale'});assert.equal(r.throttlePercent,0);assert.equal(g.model.last,null);
});
