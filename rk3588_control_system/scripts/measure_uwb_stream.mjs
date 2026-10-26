#!/usr/bin/env node
// Passive measurement of real Socket.IO observations, never a motion command.
import { pathToFileURL } from 'node:url';

const finite = value => typeof value === 'number' && Number.isFinite(value);
function statistics(values) {
  if (!values.length) return null;
  const ordered = [...values].sort((a,b)=>a-b);
  return {meanMs:values.reduce((a,b)=>a+b,0)/values.length,
    p95Ms:ordered[Math.ceil(ordered.length*.95)-1],maxMs:ordered.at(-1),
    over75ms:values.filter(v=>v>75).length,over700ms:values.filter(v=>v>700).length};
}

export class UwbStreamMeter {
  constructor() { this.receivers = new Map(); this.events = {}; }
  observe(uwb, at, event = 'uwb_update') {
    this.events[event] = (this.events[event] || 0) + 1;
    for (const sensor of uwb.sensors || []) {
      const key = `${sensor.receiverId}:${sensor.tagId ?? ''}`;
      if (!this.receivers.has(key)) this.receivers.set(key, {receiverId:sensor.receiverId,tagId:sensor.tagId,
        raw:new Set(),accepted:new Set(),rawCount:0,acceptedCount:0,gaps:[],sourceGaps:[],lastAt:null,
        reasons:{},good:null,sent:null,crc:null,crcIncrease:0,clockRegressions:0,maxClock:null});
      const row = this.receivers.get(key);
      for (const [field,target] of [['goodCount','good'],['sentCount','sent']]) {
        const value = sensor[field];
        if (!finite(value)) continue;
        if (!row[target] || value < row[target].lastValue) row[target] = {firstValue:value,firstAt:at,lastValue:value,lastAt:at};
        else if (value !== row[target].lastValue) Object.assign(row[target],{lastValue:value,lastAt:at});
      }
      if (finite(sensor.checksumErrors)) {
        if (row.crc !== null) row.crcIncrease += Math.max(0,sensor.checksumErrors-row.crc);
        row.crc = sensor.checksumErrors;
      }
      const raw = sensor.receivedMeasurementAt, accepted = sensor.measurementUpdatedAt;
      if (finite(raw) && raw>0 && !row.raw.has(raw)) {
        if (row.maxClock !== null && raw < row.maxClock) row.clockRegressions++;
        if (row.maxClock !== null && raw > row.maxClock) row.sourceGaps.push(raw-row.maxClock);
        row.maxClock = Math.max(row.maxClock || 0,raw);
        row.raw.add(raw); if(row.raw.size>4096)row.raw.delete(row.raw.values().next().value);
        row.rawCount++;
        if (row.lastAt !== null) row.gaps.push(at-row.lastAt);
        row.lastAt=at;
        const reason = sensor.filterReason || uwb.rejectionReason || 'none';
        row.reasons[reason]=(row.reasons[reason] || 0)+1;
      }
      if (finite(accepted) && accepted>0 && !row.accepted.has(accepted)) {
        row.accepted.add(accepted); if(row.accepted.size>4096)row.accepted.delete(row.accepted.values().next().value);
        row.acceptedCount++;
      }
    }
  }
  report(seconds) {
    const counterHz = c => c && c.lastAt>c.firstAt ? (c.lastValue-c.firstValue)*1000/(c.lastAt-c.firstAt) : null;
    return {seconds,targetHz:40,nominalPeriodMs:25,events:this.events,
      note:'Counts distinct received/accepted clocks; redraws and duplicate telemetry do not create measurements. FC counters are separate estimates. No hardware commands sent.',
      receivers:[...this.receivers.values()].map(r=>({receiverId:r.receiverId,tagId:r.tagId,
        receivedHz:r.rawCount/seconds,acceptedHz:r.acceptedCount/seconds,
        fcValidFrameHz:counterHz(r.good),fcForwardAttemptHz:counterHz(r.sent),
        receiveJitter:statistics(r.gaps),bridgeMeasurementIntervals:statistics(r.sourceGaps),
        reasons:r.reasons,checksumErrorIncrease:r.crcIncrease,
        sourceClockRegressions:r.clockRegressions}))};
  }
}

async function main() {
  const args=process.argv.slice(2);
  if(args.includes('--help')){console.log('node scripts/measure_uwb_stream.mjs [--url http://127.0.0.1:3000] [--seconds 30]');return;}
  const value=(key,fallback)=>{const i=args.indexOf(key);return i<0?fallback:args[i+1];};
  const url=new URL(value('--url','http://127.0.0.1:3000'));
  const seconds=Number(value('--seconds','30'));
  if(!['http:','https:'].includes(url.protocol)||!finite(seconds)||seconds<5||seconds>3600)throw new Error('Use an HTTP(S) URL and a duration from 5 to 3600 seconds');
  const status=await (await fetch(new URL('/api/status',url),{signal:AbortSignal.timeout(4000)})).json();
  if(status.data?.isConnected!==true)throw new Error('Flight controller is disconnected; no live rate measurement taken');
  const {default:WebSocket}=await import('ws');
  const address=new URL('/socket.io/?EIO=4&transport=websocket',url);
  address.protocol=url.protocol==='https:'?'wss:':'ws:';
  const socket=new WebSocket(address), meter=new UwbStreamMeter();
  let started=null;
  await new Promise((resolve,reject)=>{
    let done=false;
    const finish=error=>{if(done)return;done=true;clearTimeout(connectTimer);clearTimeout(sampleTimer);socket.close();error?reject(error):resolve();};
    let sampleTimer;
    const connectTimer=setTimeout(()=>finish(new Error('Socket.IO connection timed out')),5000);
    socket.on('error',finish);
    socket.on('close',()=>{if(!done)finish(new Error('Web connection closed during measurement'));});
    socket.on('message',buffer=>{
      const raw=buffer.toString();
      if(raw.startsWith('0'))socket.send('40');
      else if(raw==='2')socket.send('3');
      else if(raw.startsWith('40')&&started===null){
        clearTimeout(connectTimer);started=performance.now();sampleTimer=setTimeout(()=>finish(),seconds*1000);
      }else if(raw.startsWith('42')&&started!==null){
        try {
          const [event,payload]=JSON.parse(raw.slice(2));
          const uwb=event==='uwb_update'?payload:event==='telemetry_update'?payload?.uwb:null;
          if(uwb)meter.observe(uwb,performance.now(),event);
        }catch(_){}
      }
    });
  });
  const report=meter.report((performance.now()-started)/1000);
  console.log(JSON.stringify(report,null,2));
  if(!report.receivers.some(r=>r.receivedHz>0))process.exitCode=2;
}

if(process.argv[1]&&import.meta.url===pathToFileURL(process.argv[1]).href)main().catch(error=>{console.error(error.message);process.exitCode=2;});
