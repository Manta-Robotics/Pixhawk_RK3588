import assert from 'node:assert/strict';
import test from 'node:test';
import {UwbStreamMeter} from '../scripts/measure_uwb_stream.mjs';

test('40 Hz source is counted once despite duplicate full telemetry and UI refreshes',()=>{
  const meter=new UwbStreamMeter();
  for(let i=0;i<400;i++){
    const uwb={sensors:[{receiverId:1,tagId:5313,receivedMeasurementAt:1000+i*25,
      measurementUpdatedAt:1000+i*25,goodCount:i,sentCount:i,checksumErrors:0,filterReason:'kalman_updated'}]};
    meter.observe(uwb,i*25);
    for(let j=0;j<3;j++)meter.observe(uwb,i*25+j,'telemetry_update');
  }
  const report=meter.report(10).receivers[0];
  assert.equal(report.receivedHz,40);
  assert.equal(report.acceptedHz,40);
  assert.equal(report.fcValidFrameHz,40);
  assert.equal(report.receiveJitter.meanMs,25);
  assert.equal(report.receiveJitter.over700ms,0);
});

test('40 Hz rejected raw frames are not misreported as accepted measurements',()=>{
  const meter=new UwbStreamMeter();
  for(let i=0;i<400;i++)meter.observe({rejectionReason:'innovation_outlier',sensors:[{
    receiverId:1,receivedMeasurementAt:1000+i*25,measurementUpdatedAt:null,
    filterReason:'innovation_outlier'}]},i*25);
  const report=meter.report(10).receivers[0];
  assert.equal(report.receivedHz,40);
  assert.equal(report.acceptedHz,0);
  assert.equal(report.reasons.innovation_outlier,400);
});

test('clock regressions and long delivery gaps remain visible instead of inventing samples',()=>{
  const meter=new UwbStreamMeter();
  for(const [clock,at] of [[1000,0],[1025,25],[1000,30],[900,900]])meter.observe({sensors:[{
    receiverId:1,receivedMeasurementAt:clock,measurementUpdatedAt:clock}]},at);
  const report=meter.report(1).receivers[0];
  assert.equal(report.receivedHz,3);
  assert.equal(report.sourceClockRegressions,1);
  assert.equal(report.receiveJitter.over700ms,1);
});
