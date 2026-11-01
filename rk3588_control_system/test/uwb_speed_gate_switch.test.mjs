import test from 'node:test';
import assert from 'node:assert/strict';
import {UwbUsbSource} from '../backend/uwb_usb_source.js';
import {SmoothedUwbOutput} from '../backend/uwb_smoothed_output.js';

function fixture(config){
 let time=10000;const s=new UwbUsbSource(config,{}, {monotonic:()=>time,wall:()=>1000000+time});
 s.accept({type:'connection',connected:true});
 const frame=(seq,changes={})=>s.accept({type:'position',version:0x0102,sequenceId:seq,
  anchorId:5565,tagId:5565,distanceCm:100,azimuthDeg:0,elevationDeg:0,...changes});
 for(let i=1;i<=5;i++){frame(i);time+=25;}
 return {s,frame,advance:()=>time+=25};
}
test('disabled speed rejection admits faster raw points but still runs the smoother',()=>{
 const f=fixture({speed_gate_enabled:false,output_speed_gate_enabled:false});f.frame(6,{distanceCm:120});
 assert.equal(f.s.speedGate.acceptedCount,6);assert.equal(f.s.speedGate.rejectedCount,0);
 assert.equal(f.s.snapshot().rawDistanceM,1.2);assert.equal(f.s.snapshot().fresh,true);
 assert.equal(f.s.snapshot().smoothing.parameters.maxSpeedCmPerSecond,100);
 const u=new SmoothedUwbOutput().view(f.s.snapshot(),f.s.snapshot().updatedAt);
 assert.equal(u.speedGate.enabled,false);
});
test('disabled output rejection publishes new smoothing outputs rather than discarding them',()=>{
 const f=fixture({output_speed_gate_enabled:false});const old=f.s.snapshot().measurementUpdatedAt;
 f.s.smoother.update=()=>({ready:true,smoothedDistanceCm:200,smoothedAzimuthDeg:0,smoothedXcm:200,smoothedYcm:0});
 f.frame(6);assert.equal(f.s.outputSpeedGate.rejectedCount,0);assert.equal(f.s.snapshot().distanceM,2);
 assert.ok(f.s.snapshot().measurementUpdatedAt>old);
});
test('turning off speed rejection does not disable duplicate, range, or ordering checks',()=>{
 const f=fixture({speed_gate_enabled:false,output_speed_gate_enabled:false});f.frame(5);
 assert.equal(f.s.snapshot().transport.duplicateFrames,1);
 f.frame(6,{receivedMonoMs:1000});f.advance();f.frame(7,{receivedMonoMs:999});
 assert.equal(f.s.speedGate.lastRejectionReason,'non_positive_time_delta');
 f.frame(8,{distanceCm:2100});assert.equal(f.s.snapshot().fresh,false);
});
