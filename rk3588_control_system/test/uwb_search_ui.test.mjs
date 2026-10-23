import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
const html=fs.readFileSync(new URL('../frontend/mobile-preview-kimi-k26.html',import.meta.url),'utf8');
const functions=html.slice(html.indexOf('function uwbFollowIsStarting()'),html.indexOf('function renderDriveMode()'))
 +html.slice(html.indexOf('function renderUwbFollow()'),html.indexOf('function renderBeaconCalibration()'));
function ui(follow={}){
 const elements=new Map();const el=id=>{if(!elements.has(id))elements.set(id,{textContent:'',disabled:false,classList:{toggle(){}},setAttribute(){}});return elements.get(id);};
 const calls=[];let handler;
 const box={state:{driveMode:'auto',connected:true,controlLink:true,uwb:{online:false,fresh:false},uwbFollow:follow},
  $:el,$$:()=>[],renderRelativePosition(){},localize:zh=>zh,showToast:message=>calls.push(['toast',message]),
  ensureTransport:()=>({startUwbFollow:async()=>{calls.push(['start']);return {searching:true};},stopUwbFollow:()=>{calls.push(['stop']);return Promise.resolve();}})};
 const context=vm.createContext(box);vm.runInContext(functions,context);
 box.$=id=>{const node=el(id);if(id==='#driveFollowSwitch')node.addEventListener=(_event,fn)=>handler=fn;return node;};
 const start=html.indexOf('$("#driveFollowSwitch").addEventListener("click",function(){');
 const end=html.indexOf('  $$("[data-tracking-method]")',start);
 vm.runInContext(html.slice(start,end),context);
 return {box,context,calls,el,click:()=>handler()};
}
test('the follow switch submits a search request even without UWB',async()=>{
 const u=ui();u.click();await new Promise(r=>setImmediate(r));
 assert.equal(u.calls.filter(([n])=>n==='start').length,1);assert.ok(u.calls.some(([n,m])=>n==='toast'&&m.includes('搜索 UWB')));
});
test('search state displays a stop action and clicking it cancels instead of restarting',()=>{
 const u=ui({active:false,status:'awaiting_signal',reason:'awaiting_trusted_uwb',followRequested:true,autoResumePending:true});
 vm.runInContext('renderUwbFollow()',u.context);
 assert.equal(u.el('#driveFollowSwitchLabel').textContent,'停止搜索');
 assert.match(u.el('#driveFollowSwitchStatus').textContent,/搜索 UWB/);
 assert.equal(u.el('#driveFollowSwitch').disabled,false);
 u.click();assert.equal(u.calls.filter(([n])=>n==='stop').length,1);assert.equal(u.calls.filter(([n])=>n==='start').length,0);
});

test('switching to manual mode cancels a pending search',async()=>{
 const u=ui({active:false,status:'awaiting_signal',followRequested:true,autoResumePending:true});
 u.box.renderDriveMode=()=>{};u.box.renderArmControl=()=>{};
 vm.runInContext(html.slice(html.indexOf('function setDriveMode('),html.indexOf('function renderUwbFollow(')),u.context);
 vm.runInContext("setDriveMode('manual')",u.context);await new Promise(r=>setImmediate(r));
 assert.equal(u.box.state.driveMode,'manual');assert.equal(u.calls.filter(([n])=>n==='stop').length,1);
});
