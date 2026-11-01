import { EventEmitter } from 'node:events';
import { parentPort,workerData } from 'node:worker_threads';
import { runUwbWorker } from '../../backend/uwb_usb_worker_runtime.js';

class FakeSource extends EventEmitter{
  constructor(){super();this.streamId='test';this.identity='5565:5565';}
  start(){for(let i=1;i<=100;i++)this.emit('packet',{sequence:i,streamId:this.streamId});}
  close(){}
}
if (parentPort) runUwbWorker(parentPort,workerData,()=>new FakeSource());
