import { EventEmitter } from 'node:events';
import { Worker } from 'node:worker_threads';
import { bootMonotonicMs } from './uwb_usb_worker_runtime.js';

const MAX_AGE_MS=500;
function unavailable(packet,reason){
  return {...packet,online:false,fresh:false,distanceM:null,azimuthDeg:null,elevationDeg:null,
    relativeXM:null,relativeYM:null,receiverCount:0,observingReceiverCount:0,activeReceiverIds:[],
    fusionMode:'no_valid_receiver',measurementStatus:'unavailable',rejectionReason:reason,
    sensors:(packet.sensors||[]).map(sensor=>({...sensor,valid:false,fresh:false,
      distanceM:null,azimuthDeg:null,measurementStatus:'unavailable',invalidReason:reason})),
    smoothing:{...packet.smoothing,ready:false}};
}

export class UwbUsbWorkerSource extends EventEmitter {
  constructor(config={},geometry={},runtime={}){
    super();this.config=config;this.geometry=geometry;
    this.mono=runtime.monotonic||bootMonotonicMs;
    this.factory=runtime.workerFactory||((data)=>new Worker(new URL('./uwb_usb_worker.js',import.meta.url),{workerData:data}));
    this.restartDelayMs=runtime.restartDelayMs??1200;
    this.worker=null;this.stopped=false;this.generation=0;this.restarts=0;this.checkpoint=null;
    this.packet={source:'USB_SERIAL',fresh:false,online:false,sensors:[],transport:{kind:'usb_serial',
      connected:false,targetSampleRateHz:35,processingMode:'worker_thread',workerState:'starting'}};
    this.receivedMono=this.mono();
  }
  start(){
    if(this.stopped||this.worker)return;
    const generation=++this.generation;
    const worker=this.factory({config:this.config,geometry:this.geometry,generation,checkpoint:this.checkpoint});
    this.worker=worker;
    const failed=reason=>{
      if(this.worker!==worker||this.stopped)return;
      this.worker=null;this.restarts++;
      this.packet=unavailable(this.snapshot(),reason);
      this.packet.transport={...this.packet.transport,workerState:'restarting',workerRestarts:this.restarts};
      this.receivedMono=this.mono();this.emit('packet',this.packet);
      this.restartTimer=setTimeout(()=>this.start(),this.restartDelayMs);this.restartTimer.unref();
    };
    worker.on('message',message=>{
      if(this.worker!==worker||this.stopped)return;
      if(message.type==='diagnostic'){this.emit('diagnostic',message.message);return;}
      if(message.type!=='packet'||message.generation!==generation)return;
      try{
        const elapsed=this.mono()-message.emittedMonoMs;
        const age=Math.max(0,elapsed);
        let packet=message.packet;
        const inputAge=packet.smoothing?.inputSampleAgeMs;
        const outputAge=packet.smoothing?.sampleAgeMs;
        packet={...packet,smoothing:{...packet.smoothing,
          inputSampleAgeMs:Number.isFinite(inputAge)?inputAge+age:null,
          sampleAgeMs:Number.isFinite(outputAge)?outputAge+age:null},
          transport:{...packet.transport,processingMode:'worker_thread',workerState:'running',
            workerThreadId:message.threadId,workerGeneration:generation,workerRestarts:this.restarts,
            workerMailboxCoalesced:message.coalesced,deliveryAgeMs:age,
            serialParserProcessId:message.serialParserProcessId}};
        if(!Number.isFinite(elapsed)||elapsed< -1||age>MAX_AGE_MS)
          packet=unavailable(packet,'stale_worker_delivery');
        else if(packet.fresh&&(!Number.isFinite(inputAge)||!Number.isFinite(outputAge)
          ||inputAge<0||outputAge<0))packet=unavailable(packet,'invalid_worker_measurement');
        else if(packet.fresh&&(inputAge+age>MAX_AGE_MS||outputAge+age>MAX_AGE_MS))
          packet=unavailable(packet,'stale_measurement');
        // A promptly delivered heartbeat-only/USB-disconnected packet is a
        // valid diagnostic. Its old position clock must not hide that reason.
        this.packet=packet;this.receivedMono=this.mono();this.checkpoint=message.checkpoint;
        this.emit('packet',packet);
      }finally{worker.postMessage({type:'ack',id:message.id});}
    });
    worker.on('error',error=>{this.emit('diagnostic',error.message);failed('uwb_worker_error');});
    worker.on('exit',()=>failed('uwb_worker_exited'));
    if(!this.watchdog){this.watchdog=setInterval(()=>{
      const packet=this.snapshot();
      if(this.packet.fresh&&!packet.fresh){this.packet=packet;this.receivedMono=this.mono();this.emit('packet',packet);}
    },50);this.watchdog.unref();}
  }
  snapshot(){
    const elapsed=Math.max(0,this.mono()-this.receivedMono);
    const packet={...this.packet,smoothing:{...this.packet.smoothing}};
    for(const key of ['sampleAgeMs','inputSampleAgeMs']){
      if(Number.isFinite(packet.smoothing[key]))packet.smoothing[key]+=elapsed;
    }
    if(packet.fresh&&(packet.smoothing.sampleAgeMs>MAX_AGE_MS||packet.smoothing.inputSampleAgeMs>MAX_AGE_MS))
      return unavailable(packet,'stale_measurement');
    return packet;
  }
  close(){
    this.stopped=true;clearTimeout(this.restartTimer);clearInterval(this.watchdog);
    const worker=this.worker;this.worker=null;
    if(worker){
      worker.postMessage({type:'close'});
      const fallback=setTimeout(()=>worker.terminate(),1000);fallback.unref();
      worker.once('exit',()=>clearTimeout(fallback));
    }
  }
}
