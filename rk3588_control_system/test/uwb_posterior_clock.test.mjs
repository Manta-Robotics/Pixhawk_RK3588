import test from 'node:test';
import assert from 'node:assert/strict';
import {UwbTargetEstimator, IntermittentUwbFollow} from '../backend/uwb_target_prediction.js';
import {UwbFollowController} from '../backend/uwb_follow.js';

const config = {execution_mode:'relative_imu', bearing_sign:1, bearing_scale:1, bearing_offset_deg:0,
  safety_distance_m:.3, hard_stop_enabled:false, prediction:{enabled:true}};
function frame(now, at=now, sequence=42) {
  return {armed:true, flightMode:'MANUAL', pilotSteering:{ready:true,type:0},
    attitude:{yaw:0,yawRateDegS:0,updatedAt:now}, gps:{fixType:1}, ekf:{healthy:false},
    velocity:{vx:null,vy:null}, uwb:{online:true,fresh:true,distanceM:3.45,azimuthDeg:8,
      measurementStatus:'trusted',measurementUpdatedAt:at,activeReceiverIds:[1],sensors:[{
        receiverId:1,tagId:5313,sequenceId:sequence,filterAdmissionVersion:1,valid:true,
        measurementStatus:'trusted',measurementUpdatedAt:at,receivedMeasurementAt:at,
        filterUncertaintyM:.35,rawDistanceM:3.45,rawAzimuthDeg:8}]}};
}

test('fresh admitted posterior advances while diagnostic sequence is unchanged for five seconds',()=>{
  const e=new UwbTargetEstimator(config.prediction,config);
  for(let now=1000;now<=7000;now+=100) {
    const r=e.update(now,frame(now));
    assert.equal(r.lastMeasuredAt,now);
    assert.equal(r.measurementConfirmed,true);
    assert.equal(r.predictionAgeMs,0);
    assert.ok(r.uncertaintyM<.5);
  }
});

test('real no-GPS follow wrapper does not pause on fresh admitted positions with a slow diagnostic sequence',()=>{
  const c=new UwbFollowController(config),p=new IntermittentUwbFollow(config.prediction,config);
  for(let now=1000;now<=7000;now+=100) {
    const r=p.update({controller:c,now,telemetry:frame(now),connected:true});
    assert.equal(r.targetEstimate.lastMeasuredAt,now);
    assert.equal(r.safe,true);
    assert.notEqual(r.pausePropulsion,true);
    assert.notEqual(r.status,'waiting_target');
  }
});

test('metadata sequence changes cannot refresh an unchanged accepted posterior or its loss deadline',()=>{
  const e=new UwbTargetEstimator(config.prediction,config);
  e.update(1000,frame(1000));
  for(let now=1100;now<6500;now+=100) {
    const r=e.update(now,frame(now,1000,now));
    assert.equal(r.lastMeasuredAt,1000);
    if(now>=1500) {
      assert.notEqual(r.measurementConfirmed,true);
      assert.equal(e.lossAt,1500);
    }
  }
  assert.equal(e.update(6500,frame(6500)).reason,'prediction_wait_timeout');
});

test('fresh raw reception without filter admission cannot advance the accepted clock',()=>{
  const e=new UwbTargetEstimator(config.prediction,config);
  e.update(1000,frame(1000));
  const rejected=frame(1100);rejected.uwb.sensors[0].measurementUpdatedAt=1000;
  rejected.uwb.sensors[0].measurementStatus='suspect';
  const r=e.update(1100,rejected);
  assert.equal(r.lastMeasuredAt,1000);
  assert.notEqual(r.measurementConfirmed,true);
});
