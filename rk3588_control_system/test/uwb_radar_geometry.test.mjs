import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
const context={};vm.createContext(context);
vm.runInContext(fs.readFileSync(new URL('../frontend/js/uwb-radar-geometry.js',import.meta.url),'utf8'),context);
const radar=context.MantaUwbRadar;
test('cardinal directions share equal metre scaling on both axes',()=>{
  for(const [x,y,px,py] of [[1,0,120,68],[0,1,78,110],[-1,0,120,152],[0,-1,162,110]]){
    const p=radar.project(x,y,2);assert.equal(p.x,px);assert.equal(p.y,py);assert.equal(p.distanceM,1);assert.equal(p.outside,false);
  }
});
test('diagonal and seam positions round-trip to the same physical coordinates',()=>{
  for(const az of [-180,-179,-90,-1,0,1,90,179,180]){
    const a=az*Math.PI/180,x=1.2*Math.cos(a),y=1.2*Math.sin(a),p=radar.project(x,y,2);
    assert.ok(Math.abs((110-p.y)/84*2-x)<1e-12);
    assert.ok(Math.abs((120-p.x)/84*2-y)<1e-12);
  }
});
test('off-scale projection is explicitly a bearing marker and retains the true range',()=>{
  const p=radar.project(3,4,2);
  assert.equal(p.outside,true);assert.equal(p.distanceM,5);
  assert.ok(Math.abs(Math.hypot(p.x-120,p.y-110)-84)<1e-12);
  assert.deepEqual(Array.from(radar.ticks(2)),[2/3,4/3,2]);
  assert.equal(radar.meters(50),'50 m');
});
test('invalid coordinates are never drawn',()=>{
  for(const args of [[NaN,0,2],[1,Infinity,2],[1,1,0],[1,1,null],[1.7e308,1.7e308,2]])assert.equal(radar.project(...args),null);
});
