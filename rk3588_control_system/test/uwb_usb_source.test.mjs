import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import vm from 'node:vm';
import { createRequire } from 'node:module';
import { UwbUsbSource } from '../backend/uwb_usb_source.js';
import { computeBeaconControl } from '../backend/gimbal_beacon.js';
const Smoother=createRequire(import.meta.url)('../backend/vendor/responsive_smoother.cjs');
const position=(sequenceId,changes={})=>({type:'position',version:0x0102,sequenceId,
  anchorId:5565,tagId:5565,distanceCm:100,azimuthDeg:0,elevationDeg:0,...changes});

function fixture(config={}) {
  let time=10000;
  const source=new UwbUsbSource(config,{}, {monotonic:()=>time, wall:()=>1000000+time});
  source.accept({type:'connection',connected:true});
  const frame=(sequenceId,changes={})=>source.accept({type:'position',version:0x0102,
    sequenceId,anchorId:5565,tagId:5565,distanceCm:100,azimuthDeg:0,elevationDeg:0,...changes});
  return {source,frame,advance:(ms=25)=>time+=ms,
    warm(changes={}){for(let i=1;i<=5;i++){frame(i,changes);time+=25;}}};
}

test('shared capture clock rejects old pipe data and expires data waiting for sampling',()=>{
  let time=10000;
  const source=new UwbUsbSource({}, {}, {monotonic:()=>time,captureMonotonic:()=>time,wall:()=>1000000+time});
  source.receive({type:'connection',connected:true});
  source.receive(position(1,{receivedMonoMs:9499}));source.sampleLatest();
  assert.equal(source.snapshot().transport.expiredCaptureFrames,1);
  assert.equal(source.snapshot().transport.processedFrames,0);
  source.receive(position(2,{receivedMonoMs:9600}));time+=101;source.sampleLatest();
  assert.equal(source.snapshot().transport.expiredPendingFrames,1);
  assert.equal(source.snapshot().transport.processedFrames,0);
  source.receive(position(3,{receivedMonoMs:10000}));source.sampleLatest();
  assert.equal(source.snapshot().smoothing.inputSampleAgeMs,101);
  assert.equal(source.snapshot().sensors[0].receivedMeasurementAt,1010000);
});

test('requires five real frames; duplicate sequences cannot warm or refresh',()=>{
  const f=fixture();
  for(let i=0;i<10;i++){f.frame(1);f.advance();}
  assert.equal(f.source.snapshot().smoothing.ready,false);
  assert.equal(f.source.snapshot().smoothing.warmupCount,1);
  for(let i=2;i<=5;i++){f.frame(i);f.advance();}
  assert.equal(f.source.snapshot().smoothing.ready,true);
  const clock=f.source.snapshot().measurementUpdatedAt;
  f.advance(501);f.frame(5);f.source.expire();
  assert.equal(f.source.snapshot().fresh,false);
  assert.equal(f.source.snapshot().measurementUpdatedAt,clock);
});

test('0/359 seam produces a forward target, never a 180-degree swing',()=>{
  const f=fixture();
  for(let i=1;i<=30;i++){f.frame(i,{azimuthDeg:i%2?359:0});f.advance();}
  const p=f.source.snapshot();
  assert.ok(Math.abs(p.azimuthDeg)<2);
  assert.ok(p.relativeXM>.99);
  assert.equal(p.rawAzimuthDeg,0);
  assert.equal(p.smoothing.parameters.deadbandCm,10);
});

test('null angle is rejected before vendor Number(null) coercion',()=>{
  const f=fixture();f.warm();f.frame(6,{azimuthDeg:null});
  assert.equal(f.source.snapshot().fresh,false);
  assert.equal(f.source.snapshot().rejectionReason,'invalid_measurement');
});

test('heartbeat and reconnect cannot renew or resurrect an old position',()=>{
  const f=fixture();f.warm();const clock=f.source.snapshot().measurementUpdatedAt;
  f.source.accept({type:'heartbeat'});
  assert.equal(f.source.snapshot().fresh,false);
  assert.equal(f.source.snapshot().measurementUpdatedAt,clock);
  f.source.accept({type:'connection',connected:false});
  f.source.accept({type:'connection',connected:true});
  assert.equal(f.source.snapshot().sensors[0].valid,false);
  f.frame(6);assert.equal(f.source.snapshot().smoothing.warmupCount,1);
});

test('identity switch resets the filter and selected target is enforced',()=>{
  const f=fixture({target_tag_id:5565});f.warm();
  const clock=f.source.snapshot().measurementUpdatedAt;
  f.frame(6,{tagId:7777,distanceCm:1500});
  assert.equal(f.source.snapshot().measurementUpdatedAt,clock);
  f.frame(6,{anchorId:2222});
  assert.equal(f.source.snapshot().smoothing.ready,false);
  assert.equal(f.source.snapshot().smoothing.warmupCount,1);
});

test('16-bit sequence wrap is continuous and a long gap warms up again',()=>{
  const f=fixture();
  for(const seq of [65533,65534,65535,0,1]){f.frame(seq);f.advance();}
  assert.equal(f.source.snapshot().fresh,true);
  f.advance(501);f.frame(2);
  assert.equal(f.source.snapshot().fresh,false);
  assert.equal(f.source.snapshot().smoothing.warmupCount,1);
});

test('range validation and speed-limited output keep a jump from reaching gimbal',()=>{
  const f=fixture();f.warm();const previous=f.source.snapshot();
  f.frame(6,{distanceCm:1500,azimuthDeg:90});
  const next=f.source.snapshot();
  assert.ok(Math.hypot(next.relativeXM-previous.relativeXM,next.relativeYM-previous.relativeYM)<=3.5*.025+.000001);
  const result=computeBeaconControl({now:next.updatedAt,uwb:next,
    config:{control_mode:'absolute_angle',use_pitch:true}});
  assert.equal(result.valid,true);
  assert.ok(Math.abs(result.targetYawDeg)<2);
  f.frame(7,{distanceCm:2100});
  assert.equal(f.source.snapshot().fresh,false);
});

test('browser displays processed samples directly; no extra EMA or duplicate advancement',()=>{
  const context={};vm.createContext(context);
  vm.runInContext(readFileSync(new URL('../frontend/js/uwb-display-smoother.js',import.meta.url),'utf8'),context);
  const view=new context.MantaUwbDisplaySmoother(120);
  const packet={online:true,fresh:true,distanceM:1,azimuthDeg:30,measurementUpdatedAt:100};
  assert.equal(view.view(packet,100).azimuthDeg,30);
  packet.azimuthDeg=45;packet.measurementUpdatedAt=125;
  assert.equal(view.view(packet,125).azimuthDeg,45);
  assert.equal(view.view(packet,200).azimuthDeg,45);
  assert.equal(view.view(packet,826).valid,false);
});

test('out-of-range raw reports remain inspectable without admitting them to tracking',()=>{
  const f=fixture();f.warm();f.frame(6,{distanceCm:1});
  const p=f.source.snapshot();
  assert.equal(p.fresh,false);
  assert.equal(p.rejectionReason,'distance_out_of_range');
  assert.equal(p.sensors[0].rawDistanceM,.01);
  assert.equal(p.sensors[0].rawRejectedAt,p.sensors[0].receivedMeasurementAt);
});

test('speed gate accepts exactly 1.0 m/s and rejects faster coordinates before smoothing',()=>{
  const f=fixture();f.warm();
  f.frame(6,{distanceCm:102.5});
  const accepted=f.source.snapshot();
  assert.equal(accepted.speedGate.acceptedCount,6);
  assert.equal(accepted.speedGate.lastCandidateSpeedMps,1);
  assert.equal(accepted.speedGate.thresholdMps,1);
  const filterState=JSON.stringify(f.source.smoother);
  f.advance();f.frame(7,{distanceCm:105.01});
  const rejected=f.source.snapshot();
  assert.equal(rejected.speedGate.rejectedCount,1);
  assert.equal(rejected.speedGate.lastRejectionReason,'speed_outlier');
  assert.equal(rejected.rawDistanceM,1.025);
  assert.equal(rejected.speedGate.lastDecision.distanceM,1.0501);
  assert.equal(rejected.speedGate.lastDecision.accepted,false);
  assert.equal(rejected.speedGate.lastDecision.referenceSequenceId,6);
  assert.equal(rejected.measurementUpdatedAt,accepted.measurementUpdatedAt);
  assert.equal(rejected.sensors[0].receivedMeasurementAt,accepted.sensors[0].receivedMeasurementAt);
  assert.equal(JSON.stringify(f.source.smoother),filterState);
});

test('a point at 1.2 m/s is rejected by the new 1.0 m/s threshold',()=>{
  const f=fixture();f.warm();f.frame(6,{distanceCm:103});
  const p=f.source.snapshot();
  assert.equal(p.speedGate.lastDecision.speedMps,1.2);
  assert.equal(p.speedGate.lastDecision.accepted,false);
  assert.equal(p.rawDistanceM,1);
  assert.equal(p.smoothing.parameters.maxSpeedCmPerSecond,100);
});

test('same-range angular jumps are rejected by XY displacement',()=>{
  const f=fixture();f.warm();f.frame(6,{azimuthDeg:90});
  const p=f.source.snapshot();
  assert.equal(p.speedGate.rejectedCount,1);
  assert.equal(p.rawAzimuthDeg,0);
  assert.equal(p.sensors[0].sequenceId,5);
  assert.ok(p.speedGate.lastCandidateSpeedMps>50);
});

test('rejected coordinates do not poison the next speed reference',()=>{
  const f=fixture();f.warm();f.frame(6,{distanceCm:180});
  f.advance();f.frame(7,{distanceCm:105});
  const p=f.source.snapshot();
  assert.equal(p.speedGate.rejectedCount,1);
  assert.equal(p.speedGate.acceptedCount,6);
  assert.equal(p.rawDistanceM,1.05);
  assert.equal(p.speedGate.lastCandidateSpeedMps,1);
});

test('uses reader monotonic timing rather than nominal frame rate or Node burst timing',()=>{
  const f=fixture();
  f.frame(1,{receivedMonoMs:1000});
  f.advance(1);f.frame(2,{distanceCm:110,receivedMonoMs:1100});
  assert.equal(f.source.snapshot().speedGate.lastCandidateSpeedMps,1);
  assert.equal(f.source.snapshot().speedGate.acceptedCount,2);
  f.advance(100);f.frame(3,{distanceCm:112,receivedMonoMs:1110});
  assert.equal(f.source.snapshot().speedGate.rejectedCount,1);
  f.frame(4,{distanceCm:110,receivedMonoMs:1100});
  assert.equal(f.source.snapshot().speedGate.lastRejectionReason,'non_positive_time_delta');
});

test('a stream of rejected jumps never refreshes position or silently seeds after timeout',()=>{
  const f=fixture();f.warm();const clock=f.source.snapshot().measurementUpdatedAt;
  for(let seq=6;seq<=30;seq++){f.frame(seq,{distanceCm:1000});f.advance();}
  const p=f.source.snapshot();
  assert.equal(p.fresh,false);
  assert.equal(p.measurementUpdatedAt,clock);
  assert.equal(p.speedGate.acceptedCount,5);
  assert.equal(p.speedGate.rejectedCount,25);
  f.frame(31,{distanceCm:101});
  assert.equal(f.source.snapshot().smoothing.warmupCount,1);
});

test('a new radio session or identity initializes its own speed reference',()=>{
  const f=fixture();f.warm();f.source.accept({type:'heartbeat'});
  f.advance();f.frame(6,{distanceCm:500});
  assert.equal(f.source.snapshot().rawDistanceM,5);
  assert.equal(f.source.snapshot().smoothing.warmupCount,1);
  f.advance();f.frame(7,{anchorId:2222,distanceCm:1000});
  assert.equal(f.source.snapshot().rawDistanceM,10);
  assert.equal(f.source.snapshot().speedGate.rejectedCount,0);
});

test('40 Hz arrivals sampled on 35 Hz deadlines yield 35 genuine samples, not 20 or repeated frames',()=>{
  const f=fixture();let seq=0;
  for(let tick=0;tick<280;tick++){
    if(tick%7===0){seq++;f.source.receive(position(seq));}
    if(tick%8===0)f.source.sampleLatest();
    f.advance(1000/280);
  }
  const p=f.source.snapshot();
  assert.equal(seq,40);
  assert.equal(p.speedGate.acceptedCount,35);
  assert.equal(p.transport.processedFrames,35);
  assert.equal(p.transport.targetSampleRateHz,35);
  assert.ok(Math.abs(p.transport.processedRateHz-35)<1e-8);
  assert.equal(p.transport.rateLimitedFrames,4);
  assert.equal(p.smoothing.parameters.periodMs,1000/35);
  assert.equal(p.smoothing.parameters.maxSpeedCmPerSecond,100);
});

test('sampling retains capture timestamps for the 1.0 m/s gate',()=>{
  const f=fixture();
  f.source.receive(position(1,{receivedMonoMs:1000}));f.source.sampleLatest();
  f.advance(1000/35);
  f.source.receive(position(2,{distanceCm:102.6,receivedMonoMs:1025}));f.source.sampleLatest();
  const p=f.source.snapshot();
  assert.equal(p.speedGate.lastDecision.accepted,false);
  assert.equal(p.speedGate.lastDecision.deltaMs,25);
  assert.ok(Math.abs(p.speedGate.lastDecision.speedMps-1.04)<1e-10);
});

test('sampler does not reuse the last coordinate when no new UART frame exists',()=>{
  const f=fixture();f.warm();
  const at=f.source.snapshot().measurementUpdatedAt;
  for(let i=0;i<20;i++){f.source.sampleLatest();f.advance(1000/35);}
  f.source.expire();
  assert.equal(f.source.snapshot().measurementUpdatedAt,at);
  assert.equal(f.source.snapshot().fresh,false);
  assert.equal(f.source.snapshot().transport.processedFrames,0);
});

test('heartbeat or disconnect discards any pending position before the next sampling tick',()=>{
  for(const loss of [{type:'heartbeat'},{type:'connection',connected:false}]){
    const f=fixture();f.warm();
    f.source.receive(position(6));f.source.receive(loss);f.source.sampleLatest();
    assert.equal(f.source.snapshot().fresh,false);
    assert.equal(f.source.snapshot().transport.processedFrames,0);
  }
});

test('a delayed queued position cannot be promoted into a new fresh observation',()=>{
  const f=fixture();f.warm();
  f.source.receive(position(6));f.advance(501);f.source.sampleLatest();
  const p=f.source.snapshot();
  assert.equal(p.fresh,false);
  assert.equal(p.transport.expiredPendingFrames,1);
  assert.equal(p.transport.processedFrames,0);
});

test('35 Hz smoother timestep changes integration while preserving other defaults and independent instances',()=>{
  const old=new Smoother(),thirtyFive=new Smoother({periodMs:1000/35});
  for(let i=0;i<5;i++){old.update(100,0,i*25);thirtyFive.update(100,0,i*1000/35);}
  const a=old.update(1000,0,125),b=thirtyFive.update(1000,0,5000/35);
  assert.ok(Math.abs(a.smoothedXcm-103.125)<1e-8);
  assert.ok(Math.abs(b.smoothedXcm-(100+5000*(1/35)**2))<1e-8);
  assert.equal(old.parameters.periodMs,25);
  assert.equal(Smoother.parameters.periodMs,25);
  for(const key of Object.keys(old.parameters).filter(key=>key!=='periodMs'))
    assert.equal(thirtyFive.parameters[key],old.parameters[key]);
});

test('smoothed overspeed keeps the previous output coordinate and observation time',()=>{
  const f=fixture();f.warm();const old=f.source.snapshot();
  f.source.smoother.update=()=>({ready:true,smoothedDistanceCm:200,smoothedAzimuthDeg:0,smoothedXcm:200,smoothedYcm:0});
  f.frame(6);
  const p=f.source.snapshot();
  assert.equal(p.distanceM,old.distanceM);
  assert.equal(p.measurementUpdatedAt,old.measurementUpdatedAt);
  assert.equal(p.smoothing.outputMonotonicMs,old.smoothing.outputMonotonicMs);
  assert.equal(p.smoothing.outputSpeedGate.rejectedCount,1);
  assert.equal(p.smoothing.outputSpeedGate.lastDecision.accepted,false);
  assert.ok(p.smoothing.outputSpeedGate.lastDecision.speedMps>1);
  for(let seq=7;seq<=27;seq++){f.advance();f.frame(seq);f.source.expire();}
  assert.equal(f.source.snapshot().fresh,false);
  assert.equal(f.source.snapshot().measurementUpdatedAt,old.measurementUpdatedAt);
});

test('output guard uses actual elapsed time and accepts the 1.0 m/s boundary',()=>{
  const f=fixture();f.warm();let x=102.5;
  f.source.smoother.update=()=>({ready:true,smoothedDistanceCm:x,smoothedAzimuthDeg:0,smoothedXcm:x,smoothedYcm:0});
  f.frame(6);const old=f.source.snapshot();
  assert.equal(old.smoothing.outputSpeedGate.lastDecision.speedMps,1);
  assert.equal(old.distanceM,1.025);
  f.advance();x=105.001;f.frame(7);const p=f.source.snapshot();
  assert.equal(p.smoothing.outputSpeedGate.lastDecision.accepted,false);
  assert.equal(p.distanceM,old.distanceM);
  assert.equal(p.measurementUpdatedAt,old.measurementUpdatedAt);
});

test('continuous 35 Hz movement never advances the published smoothed coordinate faster than 1 m/s',()=>{
  const f=fixture();f.warm();let old=f.source.snapshot();let moved=false;
  for(let i=1;i<=140;i++){
    f.advance(1000/35);f.frame(i+5,{distanceCm:100+i*100/35});
    const p=f.source.snapshot();
    if(p.measurementUpdatedAt!==old.measurementUpdatedAt){
      const dt=(p.smoothing.outputMonotonicMs-old.smoothing.outputMonotonicMs)/1000;
      const step=Math.hypot(p.relativeXM-old.relativeXM,p.relativeYM-old.relativeYM);
      assert.ok(step<=dt+1e-8);moved ||= step>0;
    }
    old=p;
  }
  assert.equal(moved,true);
});

test('same-target radio recovery cannot bypass the output speed guard by warming a distant point',()=>{
  const f=fixture();f.warm();const old=f.source.snapshot();
  f.source.accept({type:'heartbeat'});
  for(let seq=6;seq<=10;seq++){f.advance(1000/35);f.frame(seq,{distanceCm:500});}
  const p=f.source.snapshot();
  assert.equal(p.fresh,false);
  assert.equal(p.measurementUpdatedAt,old.measurementUpdatedAt);
  assert.equal(p.smoothing.outputSpeedGate.lastDecision.accepted,false);
  assert.ok(p.smoothing.outputSpeedGate.lastDecision.speedMps>1);
  assert.equal(f.source.outputReference.x,100);
});

test('USB reconnect to the same identity preserves the previous safe output reference',()=>{
  const f=fixture();f.warm();const reference={...f.source.outputReference};
  f.source.accept({type:'connection',connected:false});
  f.source.accept({type:'connection',connected:true});
  for(let seq=6;seq<=10;seq++){f.advance(1000/35);f.frame(seq,{distanceCm:500});}
  assert.equal(f.source.snapshot().fresh,false);
  assert.deepEqual(f.source.outputReference,reference);
});
