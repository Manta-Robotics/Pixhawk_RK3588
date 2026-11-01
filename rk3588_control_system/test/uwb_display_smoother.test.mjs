import assert from 'node:assert/strict';
import test from 'node:test';
import '../frontend/js/uwb-display-smoother.js';
const Smoother = globalThis.MantaUwbDisplaySmoother;
const packet = (at, azimuthDeg=0) => ({online:true,fresh:true,distanceM:3,azimuthDeg,
  measurementUpdatedAt:at,activeReceiverIds:[1],sensors:[{receiverId:1,tagId:'a'}]});

test('display preserves admitted coordinates and real 40 Hz clocks without a second filter',()=>{
  const f=new Smoother();
  for(let i=0;i<100;i++){
    const p=packet(1000+i*25,i%2?2:-2),before=structuredClone(p);
    assert.deepEqual(f.view(p,p.measurementUpdatedAt),{valid:true,distanceM:3,azimuthDeg:p.azimuthDeg});
    assert.deepEqual(p,before);
  }
});
test('repainting cannot change a measurement coordinate or its source timestamp',()=>{
  const f=new Smoother(),p=packet(1000,20),before=structuredClone(p);
  for(let now=1000;now<=1300;now+=25)assert.deepEqual(f.view(p,now),{valid:true,distanceM:3,azimuthDeg:20});
  assert.deepEqual(p,before);assert.equal(f.view(p,1701).valid,false);
});
test('invalid, suspect, stale and future observations disappear immediately',()=>{
  const f=new Smoother();
  for(const p of [{...packet(1000),fresh:false},{...packet(1000),online:false},
    {...packet(1000),measurementStatus:'suspect'},{...packet(1000),distanceM:null},
    {...packet(1000),azimuthDeg:NaN},packet(100),packet(1100)]){
    const before=structuredClone(p);assert.equal(f.view(p,1050).valid,false);assert.deepEqual(p,before);
  }
});
test('changed sources are displayed directly without previous-source interpolation',()=>{
  const f=new Smoother();f.view(packet(1000,179),1000);
  const p=packet(1025,-179);p.sensors[0].tagId='b';assert.equal(f.view(p,1025).azimuthDeg,-179);
  f.reset();assert.equal(f.view(packet(1050,30),1050).azimuthDeg,30);
});
test('legacy demo packets do not earn fabricated measurement clocks',()=>{
  const f=new Smoother(),p=packet(1000,12);delete p.measurementUpdatedAt;
  const before=structuredClone(p);assert.deepEqual(f.view(p,5000),{valid:true,distanceM:3,azimuthDeg:12});assert.deepEqual(p,before);
});
