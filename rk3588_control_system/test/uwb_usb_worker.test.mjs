import test from 'node:test';
import assert from 'node:assert/strict';
import { EventEmitter,once } from 'node:events';
import { Worker } from 'node:worker_threads';
import { UwbUsbWorkerSource } from '../backend/uwb_usb_worker_source.js';

function fixture(){
  let time=10000;const workers=[];
  class FakeWorker extends EventEmitter{
    constructor(data){super();this.data=data;this.sent=[];workers.push(this);}
    postMessage(message){this.sent.push(message);}
    terminate(){this.emit('exit',0);}
  }
  const source=new UwbUsbWorkerSource({}, {}, {monotonic:()=>time,
    workerFactory:data=>new FakeWorker(data),restartDelayMs:1});
  source.start();
  const packet=(age=0)=>({type:'packet',generation:source.generation,id:1,emittedMonoMs:time-age,
    threadId:7,coalesced:0,packet:{online:true,fresh:true,distanceM:1,measurementUpdatedAt:12345,
      sensors:[{valid:true,fresh:true}],smoothing:{sampleAgeMs:0,inputSampleAgeMs:0}},
    checkpoint:{identity:'5565:5565',outputReference:{x:100,y:0,at:time},measurementWall:12345}});
  return {source,workers,packet,advance:ms=>time+=ms};
}

test('proxy exposes parallel mode, acknowledges packets and never refreshes held clocks',()=>{
  const f=fixture();f.workers[0].emit('message',f.packet(40));
  let p=f.source.snapshot();assert.equal(p.transport.processingMode,'worker_thread');
  assert.equal(p.transport.workerThreadId,7);assert.equal(p.smoothing.sampleAgeMs,40);
  assert.deepEqual(f.workers[0].sent,[{type:'ack',id:1}]);
  f.advance(461);p=f.source.snapshot();assert.equal(p.fresh,false);
  assert.equal(p.measurementUpdatedAt,12345);assert.equal(p.sensors[0].valid,false);
  f.source.close();f.workers[0].emit('exit',0);
});

test('delayed worker delivery is invalidated before listeners see it',()=>{
  const f=fixture();let seen;f.source.on('packet',p=>seen=p);
  f.workers[0].emit('message',f.packet(501));assert.equal(seen.fresh,false);
  assert.equal(seen.rejectionReason,'stale_worker_delivery');assert.equal(seen.distanceM,null);
  f.source.close();f.workers[0].emit('exit',0);
});

test('worker crash invalidates cached data and restart retains output speed reference',async()=>{
  const f=fixture();const first=f.workers[0];first.emit('message',f.packet());first.emit('exit',1);
  assert.equal(f.source.snapshot().fresh,false);
  await new Promise(resolve=>setTimeout(resolve,10));assert.equal(f.workers.length,2);
  assert.deepEqual(f.workers[1].data.checkpoint.outputReference,{x:100,y:0,at:10000});
  first.emit('message',f.packet());assert.equal(f.source.snapshot().fresh,false);
  f.source.close();f.workers[1].emit('exit',0);
});

test('real worker runs off main thread with bounded latest-only mailbox',async()=>{
  const worker=new Worker(new URL('./fixtures/uwb_worker_fixture.mjs',import.meta.url),{workerData:{generation:1}});
  const messages=[];worker.on('message',m=>messages.push(m));
  try{
    const [first]=await once(worker,'message');assert.ok(first.threadId>0);assert.equal(first.packet.sequence,1);
    await new Promise(resolve=>setTimeout(resolve,30));assert.equal(messages.length,1);
    const next=once(worker,'message');worker.postMessage({type:'ack',id:first.id});
    const [last]=await next;assert.equal(last.packet.sequence,100);assert.equal(last.coalesced,98);
    assert.ok(last.emittedMonoMs>=first.emittedMonoMs);
    worker.postMessage({type:'close'});await once(worker,'exit');
  }finally{await worker.terminate();}
});
