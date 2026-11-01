import test from 'node:test';
import assert from 'node:assert/strict';
import { generateBeaconCalibrationPoints, assertBeaconCalibrationGrid, fitBeaconCalibration } from '../backend/gimbal_beacon_calibration.js';
import { summarizeBeaconCalibrationReadings } from '../backend/beacon_calibration_sampling.js';
import { predictGimbalAnglesFromExtrinsic, normalizeGeometryAngleDeg as wrap } from '../backend/gimbal_beacon_geometry.js';
import { calibrationGimbalPose } from '../backend/beacon_calibration_input.js';

const limits={yawMinDeg:-135,yawMaxDeg:135,pitchMinDeg:-90,pitchMaxDeg:25};
function grid(){let x=0;return generateBeaconCalibrationPoints(()=>((x+=.173)%1),{profile:'precision25',limits});}
function sample(point,i) {
  // A rotated, vertically offset sensor; several raw azimuths cross +/-180.
  const theta=120*Math.PI/180, range=1.5+i*.08;
  const y=point.yawDeg*Math.PI/180,p=point.pitchDeg*Math.PI/180;
  const c=[Math.cos(p)*Math.cos(y)*range,Math.cos(p)*Math.sin(y)*range,Math.sin(p)*range-.04];
  const u=[Math.cos(theta)*c[0]+Math.sin(theta)*c[1],-Math.sin(theta)*c[0]+Math.cos(theta)*c[1],c[2]];
  return {...point,distanceM:Math.hypot(...u),uwbAzimuthDeg:Math.atan2(u[1],u[0])*180/Math.PI,
    uwbElevationDeg:Math.atan2(u[2],Math.hypot(u[0],u[1]))*180/Math.PI,gimbalYawDeg:point.yawDeg,gimbalPitchDeg:point.pitchDeg};
}
test('precision grid widens coverage within actual limits and reserves five validation points',()=>{
  const points=grid();assert.equal(points.length,25);assert.equal(points.filter(p=>p.role==='fit').length,20);
  assert.equal(points.filter(p=>p.role==='validation').length,5);assertBeaconCalibrationGrid(points,limits);
  assert.equal(Math.max(...points.map(p=>p.yawDeg))-Math.min(...points.map(p=>p.yawDeg)),220);
  assert.throws(()=>assertBeaconCalibrationGrid([...points.slice(0,24),points[0]],limits),/distinct/);
  assert.throws(()=>generateBeaconCalibrationPoints(Math.random,{profile:'precision25',limits:{...limits,yawMinDeg:-20,yawMaxDeg:20}}),/limits/);
});
test('25-point transform generalizes to independent points and the azimuth wrap seam',()=>{
  const samples=grid().map(sample);const fit=fitBeaconCalibration(samples);
  assert.equal(fit.metrics.fitPointCount,20);assert.equal(fit.metrics.validationPointCount,5);
  assert.ok(fit.metrics.validationAngularRmsErrorDeg<.25);
  for(const [yaw,pitch] of [[-120,-12],[120,8],[-59,-3],[61,14]]) {
    const h=sample({yawDeg:yaw,pitchDeg:pitch},12);
    const predicted=predictGimbalAnglesFromExtrinsic({distanceM:h.distanceM,azimuthDeg:h.uwbAzimuthDeg,elevationDeg:h.uwbElevationDeg},fit.extrinsic_3d);
    assert.ok(Math.abs(wrap(predicted.yawDeg-yaw))<.5);assert.ok(Math.abs(predicted.pitchDeg-pitch)<.5);
  }
});
test('good fitting residuals cannot hide bad independent validation points',()=>{
  const samples=grid().map(sample).map(s=>s.role==='validation'?{...s,uwbAzimuthDeg:wrap(s.uwbAzimuthDeg+15)}:s);
  assert.throws(()=>fitBeaconCalibration(samples),/Independent validation rejected/);
});
function readings(){return Array.from({length:32},(_,i)=>({at:10000+i*45,key:'[1,5565]',distanceM:2+(i%3-1)*.005,
  uwbAzimuthDeg:i%2?179.7:-179.7,uwbElevationDeg:5+(i%3-1)*.4,gimbalYawDeg:0,gimbalPitchDeg:5,feedbackVerified:true}));}
test('capture handles the 360-degree seam and removes a minority of spikes',()=>{
  const data=readings();data[4].uwbAzimuthDeg=25;data[8].uwbElevationDeg=40;
  const r=summarizeBeaconCalibrationReadings(data);assert.equal(r.sampleCount,30);assert.equal(r.quality.rejectedCount,2);
  assert.ok(Math.abs(Math.abs(r.uwbAzimuthDeg)-180)<.1);assert.ok(r.quality.yawRmsDeg<.4);
});
test('capture refuses unstable bursts, duplicated frames, switched tags and insufficient hold time',()=>{
  assert.throws(()=>summarizeBeaconCalibrationReadings(readings().map((s,i)=>({...s,uwbAzimuthDeg:i%2?170:-170}))),/outliers|stable/);
  assert.throws(()=>summarizeBeaconCalibrationReadings(readings().map(s=>({...s,at:10000}))),/distinct/);
  assert.throws(()=>summarizeBeaconCalibrationReadings(readings().map((s,i)=>({...s,key:i===0?'other':s.key}))),/same receiver/);
  assert.throws(()=>summarizeBeaconCalibrationReadings(readings().map((s,i)=>({...s,at:10000+i*5}))),/one second/);
});
test('SIYI calibration requires fresh measured pose and rejects a gimbal still in motion',()=>{
  assert.throws(()=>calibrationGimbalPose(null,{yawDeg:0,pitchDeg:0},1000,true),/feedback/);
  assert.throws(()=>calibrationGimbalPose({checksumValid:true,attitudeValid:false,updatedAt:990,yawDeg:0,pitchDeg:0},{yawDeg:0,pitchDeg:0},1000,true),/feedback/);
  assert.throws(()=>calibrationGimbalPose({checksumValid:true,updatedAt:990,yawDeg:10,pitchDeg:0},{yawDeg:0,pitchDeg:0},1000,true),/reached/);
});
test('static servo offset uses measured pose while drift during the capture is rejected',()=>{
  const pose=calibrationGimbalPose({checksumValid:true,attitudeValid:true,updatedAt:990,yawDeg:33.8,pitchDeg:.1},{yawDeg:32,pitchDeg:0},1000,true);
  assert.equal(pose.gimbalYawDeg,33.8);
  assert.throws(()=>summarizeBeaconCalibrationReadings(readings().map((s,i)=>({...s,gimbalYawDeg:i*.1}))),/still moving/);
});
