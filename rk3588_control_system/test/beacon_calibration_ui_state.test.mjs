import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
const html=fs.readFileSync(new URL('../frontend/mobile-preview-kimi-k26.html',import.meta.url),'utf8');
function ui(initial) {
  const nodes=new Map();const $=selector=>{
    if(!nodes.has(selector))nodes.set(selector,{value:selector==='#beaconCalibrationProfile'?'precision25':'',hidden:false,style:{}});
    return nodes.get(selector);
  };
  const calls=[];const transport={
    getBeaconCalibration:async()=>({calibration:initial}),
    captureBeaconCalibration:async()=>{calls.push('capture');return {calibration:{...initial,index:initial.index+1}};},
    startBeaconCalibration:async options=>{calls.push(options);return {calibration:{active:true,total:25,index:0,status:'awaiting_center',point:{yawDeg:0,pitchDeg:0}}};},
    cancelBeaconCalibration:async()=>{calls.push('cancel');}
  };
  const context={$,beaconCalibration:null,beaconCalibrationBusy:false,beaconCalibrationPollTimer:0,
    state:{gimbalMode:'click',gimbalOutputAvailable:false},localize:a=>a,showToast:(...x)=>calls.push(x),
    pageCanControl:()=>true,ensureTransport:()=>transport,setModal:(id,value)=>{$('#'+id).hidden=!value;},
    startBeaconCalibrationPreview:()=>{},stopBeaconCalibrationPreview:()=>{},setInterval:()=>1,clearInterval:()=>{},
    Promise,Number,Boolean};
  vm.createContext(context);
  const render=html.slice(html.indexOf('function renderBeaconCalibration(){'),html.indexOf('\nvar beaconCalibrationPreviewFrame'));
  const actions=html.slice(html.indexOf('function stopBeaconCalibrationPolling(){'),html.indexOf('\nfunction gimbalControlAvailable()'));
  vm.runInContext(render+'\n'+actions,context);return {context,nodes,$,calls};
}
const flush=()=>new Promise(resolve=>setImmediate(resolve));
test('opening from click mode resumes active backend calibration instead of starting a duplicate',async()=>{
  const app=ui({active:true,status:'awaiting_center',index:3,total:9,profile:'quick9',available:true,point:{yawDeg:32,pitchDeg:10}});
  app.context.openBeaconCalibration();await flush();
  assert.equal(app.context.beaconCalibration.index,3);assert.equal(app.$('#beaconCalibrationCounter').textContent,'3 / 9');
  assert.equal(app.$('#beaconCalibrationProfile').disabled,true);assert.deepEqual(app.calls,[]);
  app.context.advanceBeaconCalibration();await flush();assert.equal(app.calls[0],'capture');
});
test('idle backend state starts selected precision profile rather than capturing a nonexistent session',async()=>{
  const app=ui({active:false,status:'idle',available:true});app.context.openBeaconCalibration();await flush();
  assert.equal(app.$('#beaconCalibrationCounter').textContent,'0 / 25');app.context.advanceBeaconCalibration();await flush();
  assert.equal(app.calls[0].profile,'precision25');
});
test('unavailable inputs explain the reason and disable the start button without hiding the workflow',async()=>{
  const app=ui({active:false,status:'idle',available:false,unavailableReason:'等待实时云台姿态反馈'});
  app.context.openBeaconCalibration();await flush();
  assert.equal(app.$('#beaconCalibrationModal').hidden,false);assert.equal(app.$('#beaconCalibrationNext').disabled,true);
  assert.equal(app.$('#beaconCalibrationInstruction').textContent,'等待实时云台姿态反馈');
});
