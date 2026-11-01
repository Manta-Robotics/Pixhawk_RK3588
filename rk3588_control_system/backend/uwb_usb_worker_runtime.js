import { threadId } from 'node:worker_threads';
import { UwbUsbSource } from './uwb_usb_source.js';

export const bootMonotonicMs=()=>Number(process.hrtime.bigint())/1e6;

// Stateful filters stay ordered inside one worker. One credit bounds outbound
// messages to one in flight plus the newest pending snapshot.
export function runUwbWorker(port,data,sourceFactory=(config,geometry)=>
  new UwbUsbSource(config,geometry,{monotonic:bootMonotonicMs,captureMonotonic:bootMonotonicMs})) {
  const source=sourceFactory(data.config,data.geometry);
  const checkpoint=data.checkpoint;
  if(checkpoint?.identity&&checkpoint.outputReference
    &&[checkpoint.outputReference.x,checkpoint.outputReference.y,checkpoint.outputReference.at].every(Number.isFinite)){
    source.identity=checkpoint.identity;
    source.outputReference={...checkpoint.outputReference};
    source.measurementWall=checkpoint.measurementWall;
  }
  source.streamId+=`-worker${data.generation}`;
  let closed=false,inFlight=null,pending=null,sequence=0,coalesced=0;
  const send=envelope=>{
    inFlight=++sequence;
    port.postMessage({type:'packet',generation:data.generation,id:inFlight,
      ...envelope,threadId,coalesced});
  };
  source.on('packet',packet=>{
    if(closed)return;
    const envelope={packet,emittedMonoMs:bootMonotonicMs(),serialParserProcessId:source.child?.pid??null,
      checkpoint:{identity:source.identity,outputReference:source.outputReference?{...source.outputReference}:null,
        measurementWall:source.measurementWall}};
    if(inFlight!==null){if(pending)coalesced++;pending=envelope;}else send(envelope);
  });
  source.on('diagnostic',message=>{if(!closed)port.postMessage({type:'diagnostic',message:String(message).slice(0,1000)});});
  port.on('message',message=>{
    if(message.type==='ack'&&message.id===inFlight){
      inFlight=null;
      if(pending){const next=pending;pending=null;send(next);}
    } else if(message.type==='close'){
      closed=true;pending=null;source.close();port.close();
    }
  });
  port.on('close',()=>{closed=true;source.close();});
  source.start();
  return source;
}
