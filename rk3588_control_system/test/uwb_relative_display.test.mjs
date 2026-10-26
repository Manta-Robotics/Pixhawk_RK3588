import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import test from 'node:test';

const root=new URL('../',import.meta.url);
const read=file=>fs.readFileSync(new URL(file,root),'utf8');
const html=read('frontend/mobile-preview-kimi-k26.html');
const start=html.indexOf('function renderRelativePosition(){');
const end=html.indexOf('\nfunction currentMapLocation()',start);

function renderer(){
  const nodes=new Map();let now=100000;
  const element=id=>{
    if(!nodes.has(id))nodes.set(id,{attributes:{},style:{},dataset:{},classList:{toggle(){}},setAttribute(key,value){this.attributes[key]=value;}});
    return nodes.get(id);
  };
  const context=vm.createContext({window:{},Math,Number,JSON,Array,NaN,
    Date:{now:()=>now},uwbDisplayNow:()=>now,uwbDisplayIdentity:u=>JSON.stringify((u.sensors||[]).map(s=>[s.receiverId,s.tagId])),$:element,clamp:(n,min,max)=>Math.max(min,Math.min(max,n)),localize:zh=>zh});
  for(const file of ['uwb-display-smoother.js','uwb-raw-display.js'])vm.runInContext(read('frontend/js/'+file),context);
  vm.runInContext(`var uwbDisplaySmoother=new window.MantaUwbDisplaySmoother(120),
    relativeDisplaySmoother=new window.MantaUwbDisplaySmoother(120),
    rawUwbDisplay=new window.MantaUwbRawDisplay(),relativePlotScale=new window.MantaUwbPlotScale(),relativePositionHold=new window.MantaUwbDisplayHold();
    var state={connected:true,uwbFollow:{active:false},uwb:{}};`+html.slice(start,end),context);
  return {context,element,frame({rawM=.8,follow={active:false},status='trusted',extra={}}={}){
    now+=25;
    context.state.uwb={online:status==='trusted',fresh:status==='trusted',measurementStatus:status,
      distanceM:.8,azimuthDeg:0,measurementUpdatedAt:now-10,activeReceiverIds:[1],
      sensors:[{receiverId:1,tagId:5313,valid:true,receivedMeasurementAt:now-10,
        rawDistanceUpdatedAt:now-10,rawDistanceM:rawM,rawAzimuthDeg:0}],...extra};
    context.state.uwbFollow=follow;
    vm.runInContext('renderRelativePosition()',context);
    return {point:{...element('#relativeTargetPoint').attributes},scale:element('#relativeRingLabel').textContent,
      coordinates:element('#relativePositionValue').textContent,label:element('#relativeJudgedLabel').textContent,
      rawOutside:element('#relativeRawPoint').attributes['data-outside-range']};
  }};
}

test('raw jitter and rejected raw spikes cannot zoom or relocate a fixed trusted target',()=>{
  const r=renderer(), first=r.frame({rawM:.89});
  for(const rawM of [.91,.88,.92,20,.9]){
    const next=r.frame({rawM});
    assert.deepEqual(next.point,first.point);
    assert.equal(next.scale,first.scale);
    assert.equal(next.rawOutside,String(rawM>1));
  }
});

test('raw control targets never replace the green trusted measurement',()=>{
  const r=renderer(), first=r.frame();
  for(const x of [.3,.7,10]){
    const next=r.frame({follow:{active:true,relativeBodyM:{x,y:0}}});
    assert.deepEqual(next.point,first.point);
    assert.equal(next.coordinates,'前 0.80 · 左 0.00');
  }
});

test('admitted body coordinates and prediction labels retain their correct frame',()=>{
  const r=renderer();
  const measured=r.frame({extra:{relativeXM:.6,relativeYM:.8}});
  assert.equal(measured.coordinates,'前 0.60 · 左 0.80');
  const predicted=r.frame({follow:{active:true,targetEstimate:{usable:true,source:'predicted',relativeBodyM:{x:.5,y:.2}}}});
  assert.equal(predicted.label,'◌ 预测');
  assert.equal(predicted.coordinates,'前 0.50 · 左 -0.20');
});

test('suspect measurements retain a labelled last position without making it fresh',()=>{
  const r=renderer();r.frame();
  assert.equal(r.frame({status:'suspect'}).point.visibility,'visible');
  assert.equal(r.element('#relativePositionPanel').dataset.stale,'true');
  assert.equal(r.element('#relativeJudgedLabel').textContent,'● 上次位置');
  assert.equal(r.frame().point.visibility,'visible');
  assert.equal(r.frame({extra:{measurementUpdatedAt:1}}).point.visibility,'visible');
  assert.equal(r.element('#relativePositionPanel').dataset.stale,'true');
});

test('chart zoom shrinks only after a sustained smaller range',()=>{
  const {context}=renderer(), scale=new context.window.MantaUwbPlotScale();
  assert.equal(scale.view(4,1000),5);
  assert.equal(scale.view(.4,1100),5);
  assert.equal(scale.view(.4,3099),5);
  assert.equal(scale.view(.4,3100),1);
  assert.equal(scale.view(.91,3125),1);
  assert.equal(scale.view(1.1,3150),2);
  const wide=new context.window.MantaUwbPlotScale();
  assert.equal(wide.view(20,1000),50);
  assert.equal(wide.view(.8,1100),50);
  assert.equal(wide.view(.8,3100),2);
});

test('a prediction is never retained later as a last trusted measurement',()=>{
  const r=renderer();r.frame();
  r.frame({follow:{active:true,targetEstimate:{usable:true,source:'predicted',relativeBodyM:{x:.5,y:.2}}}});
  const held=r.frame({status:'suspect'});
  assert.equal(held.coordinates,'上次：前 0.80 · 左 0.00');
  assert.equal(held.label,'● 上次位置');
});
