import test from 'node:test';
import assert from 'node:assert/strict';
import { UwbStepMetrics } from '../backend/uwb_step_metrics.js';

test('rolling RMS equals full scan across warmup, time/capacity eviction, holds and resets',()=>{
  const metrics=new UwbStepMetrics(5000,80);let samples=[];
  for(let i=0;i<900;i++){
    if(i===400){samples=[];metrics.clear();}
    const s={mono:i*37+(i>600?6000:0),rawX:100*Math.sin(i),rawY:100*Math.cos(i),
      x:i%50<5?undefined:Math.floor(i/4),y:i%50<5?undefined:Math.floor(i/7)};
    metrics.append(s);samples.push(s);samples=samples.filter(p=>s.mono-p.mono<=5000).slice(-80);
    let raw=0,out=0,pairs=0;
    for(let j=1;j<samples.length;j++){
      const a=samples[j-1],b=samples[j];
      if(![a.x,a.y,b.x,b.y].every(Number.isFinite))continue;
      raw+=(a.rawX-b.rawX)**2+(a.rawY-b.rawY)**2;out+=(a.x-b.x)**2+(a.y-b.y)**2;pairs++;
    }
    const actual=metrics.snapshot();assert.equal(actual.pairs,pairs);
    for(const [key,sum] of [['rawStepRmsCm',raw],['smoothedStepRmsCm',out]]){
      if(!pairs)assert.equal(actual[key],null);else assert.ok(Math.abs(actual[key]-Math.sqrt(sum/pairs))<1e-8);
    }
  }
});
