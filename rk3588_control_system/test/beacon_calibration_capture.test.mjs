import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import { summarizeBeaconCalibrationReadings } from '../backend/beacon_calibration_sampling.js';
import { calibrationGimbalPose } from '../backend/beacon_calibration_input.js';
const source=fs.readFileSync(new URL('../backend/server.js',import.meta.url),'utf8');
const start=source.indexOf('async function collectBeaconCalibrationReading()');
const end=source.indexOf('\nfunction ',source.indexOf('async function captureBeaconCalibrationPoint()',start));
function harness({unstable=false,cancel=false}={}) {
  let clock=10000,n=0;
  const session={active:true,index:0,total:25,point:{yawDeg:0,pitchDeg:0},points:[{role:'fit'}],samples:[],readyAt:0};
  const gimbalState={feedback:{}};
  const ctx={gimbalBeaconCalibrationState:session,gimbalState,systemState:{telemetry:{uwb:{}}},IS_SIYI:true,
    performance:{now:()=>clock},Date:{now:()=>clock},setTimeout:resolve=>{clock+=40;if(cancel&&clock>10500)session.active=false;resolve();},
    assertBeaconCalibrationSession:s=>{if(!s.active)throw new Error('Calibration cancelled');},
    calibrationUwbObservation:()=>{n++;if(n<9||n%10===0)return null;gimbalState.feedback={checksumValid:true,attitudeValid:true,updatedAt:clock,yawDeg:0,pitchDeg:0};return {at:clock,key:'[1,5565]',distanceM:2,uwbAzimuthDeg:unstable?(n%2?20:-20):.1,uwbElevationDeg:0};},
    calibrationGimbalPose,summarizeBeaconCalibrationReadings,
    publicBeaconCalibrationState:()=>session,moveGimbalToCalibrationPoint:()=>{},
    fitBeaconCalibration:()=>{throw new Error('Should not fit the first point');},persistBeaconCalibration:()=>{throw new Error('Should not save the first point');},
    stopGimbalTracking:()=>{},startGimbalBeaconTracking:()=>{},addLog:()=>{},Promise,Number,Set,Math};
  vm.createContext(ctx);vm.runInContext(source.slice(start,end),ctx);return {ctx,session};
}
test('real capture path waits through transient invalid frames and records only stable unique observations',async()=>{
  const {ctx,session}=harness();await ctx.captureBeaconCalibrationPoint();
  assert.equal(session.index,1);assert.equal(session.samples.length,1);assert.ok(session.samples[0].sampleCount>=24);
  assert.ok(session.samples[0].quality.durationMs>=1000);assert.equal(session.captureInProgress,false);
});
test('unstable input times out without consuming the current point',async()=>{
  const {ctx,session}=harness({unstable:true});await assert.rejects(ctx.captureBeaconCalibrationPoint(),/outliers|stable/);
  assert.equal(session.index,0);assert.equal(session.samples.length,0);assert.equal(session.captureInProgress,false);
});
test('capture cancellation prevents a late observation from advancing the session',async()=>{
  const {ctx,session}=harness({cancel:true});await assert.rejects(ctx.captureBeaconCalibrationPoint(),/cancelled/);
  assert.equal(session.index,0);assert.equal(session.samples.length,0);
});
