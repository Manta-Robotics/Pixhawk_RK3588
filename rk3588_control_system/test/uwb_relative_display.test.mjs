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
    Date:{now:()=>now},uwbDisplayNow:()=>now,uwbDisplayIdentity:u=>JSON.stringify((u.sensors||[]).map(s=>[s.receiverId,s.tagId])),
    uwbDisplayConnected:u=>context.state.connected||Boolean(u?.source==='USB_SERIAL'&&u.transport?.connected),
    $:element,clamp:(n,min,max)=>Math.max(min,Math.min(max,n)),localize:zh=>zh});
  for(const file of ['uwb-display-smoother.js','uwb-raw-display.js','uwb-radar-geometry.js'])vm.runInContext(read('frontend/js/'+file),context);
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
    assert.equal(next.rawOutside,'false');
    assert.equal(r.element('#relativeRawPoint').attributes.visibility,'hidden');
  }
});

test('USB panel shows the active speed threshold and rejected coordinate count',()=>{
  const r=renderer();
  r.frame({extra:{source:'USB_SERIAL',transport:{connected:true,positionRateHz:40},
    speedGate:{thresholdMps:1.0,rejectedCount:7}}});
  assert.match(r.element('#relativeDecision').textContent,/1 m\/s 门限；已拒绝 7/);
});

test('USB panel distinguishes 35 Hz sampling from the original 40 Hz UART stream',()=>{
  const r=renderer();
  r.frame({extra:{source:'USB_SERIAL',transport:{connected:true,positionRateHz:40,targetSampleRateHz:35}}});
  assert.match(r.element('#relativeDecision').textContent,/35 Hz 采样 \/ 串口 40\.0 Hz/);
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

test('radar range stays fixed through movement, elapsed time and clock changes',()=>{
  const {context}=renderer(), scale=new context.window.MantaUwbPlotScale();
  for(const magnitude of [.4,.99,1.01,2.1,20,.1])assert.equal(scale.view(magnitude,1000),2);
  assert.equal(scale.view(.1,100000),2);
  assert.equal(scale.view(.1,1),2);
  assert.equal(scale.setRange(5),5);
  assert.equal(scale.view(100,1000000),5);
  assert.equal(scale.setRange(-1),5);
});

test('crossing one metre moves the point slightly forward without a zoom jump',()=>{
  const r=renderer();
  const a=r.frame({extra:{distanceM:.99,azimuthDeg:0}});
  const b=r.frame({extra:{distanceM:1.01,azimuthDeg:0}});
  assert.equal(a.scale,'2 m');assert.equal(b.scale,'2 m');
  assert.ok(Number(b.point.cy)<Number(a.point.cy));
  assert.ok(Math.abs(Number(b.point.cy)-Number(a.point.cy))<1);
});

test('operator range changes never modify coordinates and persist through disconnect',()=>{
  const r=renderer();const a=r.frame();
  r.context.relativePlotScale.setRange(5);const b=r.frame();
  assert.equal(a.coordinates,b.coordinates);assert.equal(b.scale,'5 m');
  r.context.state.connected=false;r.frame({status:'unavailable'});
  assert.equal(r.context.relativePlotScale.view(),5);
});

test('out-of-range targets are marked instead of silently changing the scale',()=>{
  const r=renderer();const shown=r.frame({extra:{distanceM:3}});
  assert.equal(shown.scale,'2 m');
  assert.equal(shown.point['data-outside-range'],'true');
  assert.equal(shown.point.visibility,'hidden');
  assert.equal(r.element('#relativeTargetOffscale').attributes.visibility,'visible');
  assert.match(r.element('#relativeTargetDistanceLabel').textContent,/超量程 3\.00 m/);
  assert.match(r.element('#relativeDecision').textContent,/目标超出当前量程/);
});

test('distance rings and plotted distance use the selected radius in metres',()=>{
  const r=renderer();r.frame();
  assert.equal(r.element('#relativeRingInnerLabel').textContent,'0.67 m');
  assert.equal(r.element('#relativeRingMiddleLabel').textContent,'1.33 m');
  assert.equal(r.element('#relativeRingLabel').textContent,'2 m');
  assert.equal(r.element('#relativeTargetDistanceLabel').textContent,'平滑 0.80 m');
  const dot=r.element('#relativeTargetPoint').attributes;
  assert.equal(dot.cx,'120.00');assert.equal(dot.cy,'76.40');
});

test('body XY point and its polar annotation cannot use different coordinate origins',()=>{
  const r=renderer();r.frame({extra:{relativeXM:.6,relativeYM:.8}});
  assert.equal(r.element('#relativeTargetDistanceLabel').textContent,'平滑 1.00 m');
  assert.equal(r.element('#relativeRangeValue').textContent,'1.00 m / 53.1°');
});

test('raw overlay remains unavailable even if a client checks the retired toggle',()=>{
  const r=renderer();r.frame({rawM:1.2});
  assert.equal(r.element('#relativeRawPoint').attributes.visibility,'hidden');
  assert.equal(r.element('#relativeTargetDistanceLabel').textContent,'平滑 0.80 m');
  r.element('#relativeShowRaw').checked=true;r.frame({rawM:1.2});
  assert.equal(r.element('#relativeRawPoint').attributes.visibility,'hidden');
  assert.equal(r.element('#relativeTargetDistanceLabel').textContent,'平滑 0.80 m');
});

test('a prediction is never retained later as a last trusted measurement',()=>{
  const r=renderer();r.frame();
  r.frame({follow:{active:true,targetEstimate:{usable:true,source:'predicted',relativeBodyM:{x:.5,y:.2}}}});
  const held=r.frame({status:'suspect'});
  assert.equal(held.coordinates,'上次：前 0.80 · 左 0.00');
  assert.equal(held.label,'● 上次位置');
});

test('USB raw and smoothed points display independently of Pixhawk connection',()=>{
  const r=renderer();r.context.state.connected=false;
  const shown=r.frame({extra:{source:'USB_SERIAL',transport:{connected:true,positionRateHz:40},
    smoothing:{ready:true},relativeXM:.8,relativeYM:0}});
  assert.equal(shown.point.visibility,'visible');
  assert.equal(shown.label,'● 平滑');
  assert.match(r.element('#relativeDecision').textContent,/Jetson 平滑输出/);
  r.frame({status:'unavailable',extra:{source:'USB_SERIAL',transport:{connected:false}}});
  assert.equal(r.element('#relativeTargetPoint').attributes.visibility,'hidden');
});
