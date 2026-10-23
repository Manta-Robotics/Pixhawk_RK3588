import test from 'node:test';
import assert from 'node:assert/strict';
import {UwbTargetEstimator,IntermittentUwbFollow} from '../backend/uwb_target_prediction.js';
import {UwbFollowController} from '../backend/uwb_follow.js';
const calibration={bearing_sign:1,bearing_scale:1,bearing_offset_deg:0};
function frame(now,range=3,status='trusted',at=now) {
  return {armed:true,flightMode:'ACRO',pilotSteering:{ready:true,type:0},acroControl:{ready:true,updatedAt:now},
    attitude:{yaw:0,updatedAt:now},velocity:{vx:0,vy:0},position:{updatedAt:now},ekf:{healthy:true,updatedAt:now},gps:{fixType:6},
    uwb:{online:status==='trusted',fresh:status==='trusted',distanceM:range,azimuthDeg:0,measurementStatus:status,
      measurementUpdatedAt:at,activeReceiverIds:status==='trusted'?[1]:[],sensors:[{receiverId:1,tagId:1,sequenceId:at,
        filterAdmissionVersion:1,measurementUpdatedAt:at,receivedMeasurementAt:at,measurementStatus:status,
        valid:true,rawDistanceM:range,rawAzimuthDeg:0,filterUncertaintyM:.2}]}};
}
test('admitted posterior recovers position without re-fitting raw points, velocity remains independent',()=>{
  const e=new UwbTargetEstimator({enabled:true},calibration);
  e.update(1000,frame(1000)); e.update(1100,frame(1100,3,'suspect',1000));
  const p=frame(1200,4);p.uwb.sensors[0].rawDistanceM=4.4;
  const before=structuredClone(p),r=e.update(1200,p);
  assert.equal(r.measurementConfirmed,true); assert.equal(r.positionAdmission,'filter_admitted');
  assert.equal(r.source,'recovering');assert.equal(r.lastMeasuredAt,1200);
  assert.equal(r.velocityEstimate.trusted,false);assert.equal(e.lossAt,null);
  assert.equal(e.rawWindow.reacquisition.length,0);assert.deepEqual(p,before);
});
test('multiple admitted receivers may have distinct fresh clocks, with fused clock equal to latest',()=>{
  const e=new UwbTargetEstimator({enabled:true},calibration),p=frame(1200);
  p.uwb.sensors.push({...p.uwb.sensors[0],receiverId:2,measurementUpdatedAt:1150,receivedMeasurementAt:1150});
  p.uwb.activeReceiverIds=[1,2];assert.equal(e.update(1200,p).measurementConfirmed,true);
});
test('rejected, duplicate and metadata-only updates cannot extend five-second loss deadline',()=>{
  const e=new UwbTargetEstimator({enabled:true},calibration);
  e.update(1000,frame(1000));e.update(1100,frame(1100,12,'suspect',1000));
  for(let t=1200;t<6100;t+=100){
    const p=frame(t,3,'trusted',1000);p.uwb.sensors[0].sequenceId=t;
    const r=e.update(t,p);assert.equal(r.lastMeasuredAt,1000);
  }
  assert.equal(e.update(6100,frame(6100)).reason,'prediction_wait_timeout');
});

test('contract rejects invalid covariance, mismatched clocks, changed target and stale navigation',()=>{
  for(const mutate of [p=>p.uwb.sensors[0].filterUncertaintyM=NaN,
    p=>p.uwb.sensors[0].receivedMeasurementAt=1201,p=>p.uwb.sensors[0].measurementStatus='suspect',
    p=>p.attitude.updatedAt=1]) {
    const e=new UwbTargetEstimator({enabled:true},calibration);e.update(1000,frame(1000));
    const p=frame(1200);mutate(p);assert.notEqual(e.update(1200,p).measurementConfirmed,true);
  }
  const e=new UwbTargetEstimator({enabled:true},calibration);e.update(1000,frame(1000));
  const p=frame(1200);p.uwb.sensors[0].tagId=2;
  assert.equal(e.update(1200,p).reason,'target_identity_changed');
});
test('confirmed recovery is acceleration bounded but not capped at blind prediction speed',()=>{
  const c=new UwbFollowController({...calibration,prediction:{enabled:true},execution_mode:'acro',
    safety_distance_m:.3,hard_stop_enabled:false,controller_profile:'traditional_fast'});
  const s=new IntermittentUwbFollow({enabled:true},calibration);
  const run=(t,status='trusted')=>{const p=frame(t,5,status);p.velocity.vx=.8;
    return s.update({controller:c,now:t,telemetry:p,connected:true,controlMode:'relative_acro'});};
  for(let t=1000;t<=2000;t+=100)run(t);
  const previous=s.liveCommandSpeed;run(2100,'suspect');const r=run(2200);
  assert.equal(r.targetEstimate.measurementConfirmed,true);
  assert.ok(r.liveRecoverySpeedCapMps<=previous+r.liveRecoveryAccelerationMps2*.2+1e-9);
  // Controller braking and slew may reduce speed further; the recovery governor
  // itself must preserve a legitimate >0.35 request.
  const governed=s.governAcro({...r,forwardSpeedMps:.7},2,14,18,70);
  assert.equal(governed.forwardSpeedMps,.7);
  assert.equal(r.intermittentThrottleCap,undefined);
  const close=frame(2300,.4);close.uwb.sensors[0].filterUncertaintyM=.2;
  const stopped=s.update({controller:c,now:2300,telemetry:close,connected:true,controlMode:'relative_acro'});
  assert.equal(stopped.reason,'measurement_clearance');assert.equal(stopped.forwardSpeedMps,0);
  assert.equal(c.config.safetyDistanceM,.3);assert.equal(s.config.wait_timeout_ms,5000);
});

test('rejected gap holds at most issued speed for 450ms, never refreshing five-second loss',()=>{
  const c=new UwbFollowController({...calibration,prediction:{enabled:true},execution_mode:'acro',
    safety_distance_m:.3,hard_stop_enabled:false,controller_profile:'traditional_fast'});
  const s=new IntermittentUwbFollow({enabled:true},calibration);
  const run=(t,status='trusted',range=6)=>{
    const p=frame(t,range,status,status==='trusted'?t:2000);p.velocity.vx=.6;
    const r=s.update({controller:c,now:t,telemetry:p,connected:true,controlMode:'relative_acro'});
    const drive=s.governAcro(r,2,14,18,70);return {...r,issued:drive.forwardSpeedMps};
  };
  for(let t=1000;t<=1900;t+=100)run(t);
  const prior=run(2000),lost=run(2100,'suspect');
  assert.equal(lost.reason,'accepted_position_coast');assert.ok(lost.issued>0);
  assert.ok(lost.issued<=prior.issued);assert.equal(lost.targetEstimate.lastMeasuredAt,2000);
  assert.equal(lost.targetEstimate.measurementConfirmed,false);assert.equal(s.estimator.lossAt,2100);
  assert.ok(run(2400,'suspect').issued<=lost.issued);
  assert.notEqual(run(2500,'suspect').reason,'accepted_position_coast');
  for(let t=2600;t<7100;t+=100)run(t,'suspect');
  assert.equal(run(7100,'suspect').reason,'prediction_wait_timeout');
});

test('short-gap continuation is vetoed by close raw range, FOV, missing drive and insufficient braking clearance',()=>{
  const c=new UwbFollowController({...calibration,prediction:{enabled:true},execution_mode:'acro'});
  const s=new IntermittentUwbFollow({enabled:true},calibration);s.controlMode='relative_acro';
  const e={admissionVersion:1,source:'predicted',lastMeasuredAt:1000,uncertaintyM:.3,relativeBodyM:{x:6,y:0}};
  const p=frame(1100,6,'suspect',1000);p.velocity.vx=.5;
  assert.equal(s.coastAcceptedPosition(1100,e,p,c),null);
  s.admittedDrive={at:1000,speed:.6};
  assert.equal(s.coastAcceptedPosition(1100,{...e,relativeBodyM:{x:1,y:3}},p,c),null);
  assert.equal(s.coastAcceptedPosition(1100,{...e,relativeBodyM:{x:1.9,y:0}},p,c),null);
  p.uwb.sensors[0].rawDistanceM=.25;
  assert.equal(s.coastAcceptedPosition(1100,e,p,c),null);
});

test('fresh distant target permits acceleration through FCU speed lag without changing stopping guards',()=>{
  const c=new UwbFollowController({...calibration,prediction:{enabled:true},execution_mode:'acro',
    max_speed_mps:2,commissioning_speed_mps:2,
    relative_throttle_limit_percent:70,relative_throttle_accel_percent_s:30,
    safety_distance_m:.3,hard_stop_enabled:false,controller_profile:'traditional_fast'});
  const s=new IntermittentUwbFollow({enabled:true},calibration);
  let previous=null,last;
  for(let t=1000;t<=3000;t+=100){
    const p=frame(t,8);p.velocity.vx=.1;
    last=s.update({controller:c,now:t,telemetry:p,connected:true,controlMode:'relative_acro'});
    const issued=s.governAcro(last,2,14,18,70).forwardSpeedMps;
    assert.equal(last.liveRecoveryAccelerationMps2,2*30/70);
    if(previous!==null)assert.ok(issued<=previous+(2*30/70)*.1+1e-9);
    assert.ok(issued<=last.measurementSpeedLimitMps+1e-9);previous=issued;
  }
  assert.ok(last.forwardSpeedMps>.7,'a permanent own-speed +0.2 cap would prevent this');
  const p=frame(3100,8,'suspect',3000);p.velocity.vx=.1;
  const lost=s.update({controller:c,now:3100,telemetry:p,connected:true,controlMode:'relative_acro'});
  assert.ok(lost.forwardSpeedMps<=previous);assert.equal(lost.targetEstimate.lastMeasuredAt,3000);
  assert.equal(s.config.wait_timeout_ms,5000);assert.equal(c.config.safetyDistanceM,.3);
});
