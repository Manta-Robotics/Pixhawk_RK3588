import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import {SmoothedUwbPath,usableUwbPath,uwbPathIdentity} from '../backend/uwb_path_integration.js';
import {UwbFollowController} from '../backend/uwb_follow.js';
import {validateSystemConfig} from '../scripts/validate_config.mjs';
const RAD=Math.PI/180,EARTH=6378137;
const rawConfig={execution_mode:'relative_imu',bearing_sign:1,bearing_scale:1,bearing_offset_deg:0,
 following_distance_m:.5,safety_distance_m:.3,hard_stop_enabled:false,controller_profile:'traditional_fast',
 max_speed_mps:2,commissioning_mode:false,path_follow:{enabled:true,gps_acquire_ms:1000},
 relative_nmpc:{enabled:true},gps_speed_closed_loop:{enabled:true,acquire_ms:1000}};
function telemetry(at,{x=0,y=0,targetX=3,targetY=0,yaw=0}={}){
 const dx=targetX-x,dy=targetY-y,d=Math.hypot(dx,dy),az=Math.atan2(dy,dx)/RAD-yaw;
 return {armed:true,flightMode:'MANUAL',pilotSteering:{ready:true,type:0},
 gps:{fixType:3,updatedAt:at},ekf:{healthy:true,updatedAt:at},position:{lat:30+x/(RAD*EARTH),lon:120+y/(RAD*EARTH*Math.cos(30*RAD)),updatedAt:at},
 attitude:{yaw,yawRateDegS:0,updatedAt:at},velocity:{vx:0,vy:0,updatedAt:at},
 uwb:{online:true,fresh:true,coordinateSource:'smoothed',controlSource:'smoothed',directControl:true,smoothing:{ready:true},
 measurementStatus:'trusted',measurementUpdatedAt:at,streamId:'test-one',activeReceiverIds:[1],sensors:[{receiverId:1,tagId:5565,valid:true}],
 relativeXM:d*Math.cos(az*RAD),relativeYM:d*Math.sin(az*RAD),distanceM:d,azimuthDeg:az,
 rawDistanceM:99,rawAzimuthDeg:179,elevationDeg:null}};
}
function planner(){return new SmoothedUwbPath(new UwbFollowController(rawConfig).config);}

test('real quality module and smoothed observations build a bounded route without reading conflicting raw fields',()=>{
 const p=planner();let r;
 for(let i=0;i<70;i++)r=p.update(10000+i*100,telemetry(10000+i*100,{targetX:3+i*.06,targetY:.25*Math.sin(i*.05)}));
 assert.equal(r.active,true);assert.ok(r.pointCount>=4);assert.ok(r.bodyPath.length>4&&r.bodyPath.length<=64);
 assert.ok(r.bodyTarget.x<9);assert.ok(r.bodyLookahead.x<=r.bodyTarget.x-.3);
 assert.equal(r.coordinateSource,'smoothed');assert.equal(usableUwbPath(r,telemetry(r.evaluatedAt,{targetX:7.14}),r.evaluatedAt),true);
 assert.doesNotMatch(JSON.stringify(r),/rawDistance|rawAzimuth|rawObservation/);
});
test('raw-only, invalid, warming, replayed, stale and switched observations cannot continue the old route',()=>{
 for(const patch of [{coordinateSource:'raw'},{fresh:false},{smoothing:{ready:false}},{measurementStatus:'suspect'},
   {measurementUpdatedAt:8000},{measurementUpdatedAt:11000}]){
  const p=planner(),t=telemetry(10000);p.update(10000,t);t.uwb={...t.uwb,...patch};
  const r=p.update(10000,t);assert.equal(r.active,false);assert.deepEqual(r.bodyPath,[]);assert.equal(r.pointCount,0);
 }
 const p=planner();for(let i=0;i<50;i++)p.update(10000+i*100,telemetry(10000+i*100,{targetX:3+i*.06}));
 const t=telemetry(15000);t.uwb.streamId='new';assert.equal(p.update(15000,t).active,false);
});
test('GPS loss clears the world route, while body UWB input remains available for fallback',()=>{
 const p=planner(),t=telemetry(10000);t.gps.fixType=0;
 const r=p.update(10000,t);assert.equal(r.reason,'gps_fix_invalid');assert.equal(r.active,false);assert.deepEqual(r.bodyPath,[]);
 assert.equal(t.uwb.distanceM,3);assert.equal(t.uwb.fresh,true);
});
test('all displayed route points rotate into the current boat frame',()=>{
 const p=planner();const t=telemetry(10000,{targetX:0,targetY:3,yaw:90});
 p.planner.c.fieldOfViewHalfDeg=180;
 p.update(10000,t);
 assert.ok(Math.abs(p.planner.points[0].x)<1e-6);assert.ok(Math.abs(p.planner.points[0].y-3)<1e-6);
 const r=p.update(10100,telemetry(10100,{targetX:0,targetY:3,yaw:90}));
 assert.ok(Math.abs(r.bodyObservations[0].x-3)<1e-6);assert.ok(Math.abs(r.bodyObservations[0].y)<1e-6);
});
function reference(t,extra={}){return {active:true,coordinateSource:'smoothed',evaluatedAt:t.uwb.measurementUpdatedAt,
 inputUpdatedAt:t.uwb.measurementUpdatedAt,targetKey:uwbPathIdentity(t.uwb),
 desiredSpeedMps:.4,headingErrorDeg:15,yawRateDegS:7,reason:'gps_path_follow',pointCount:10,...extra};}
test('only a fresh same-target reference can steer real guidance after GPS speed feedback acquires',()=>{
 const c=new UwbFollowController(rawConfig);let result;
 for(let at=10000;at<=12000;at+=100){const t=telemetry(at);t.uwbPath=reference(t);result=c.update({now:at,telemetry:t,connected:true});}
 assert.equal(result.pathFollowActive,true);assert.ok(result.steeringDegrees>0);assert.notEqual(result.planner,'nmpc');
 const t=telemetry(12100);t.uwbPath=reference(t,{evaluatedAt:11700});
 assert.equal(c.update({now:12100,telemetry:t,connected:true}).pathFollowActive,false);
 t.uwbPath=reference(t,{targetKey:'other'});assert.equal(usableUwbPath(t.uwbPath,t,12100),false);
});
test('route endpoint, near-range safety, loss and disarming cannot retain propulsion',()=>{
 const c=new UwbFollowController(rawConfig);let t;
 for(let at=10000;at<=12000;at+=100){t=telemetry(at);t.uwbPath=reference(t);c.update({now:at,telemetry:t,connected:true});}
 t=telemetry(12100);t.uwbPath=reference(t,{desiredSpeedMps:0,headingErrorDeg:0,yawRateDegS:0});
 const r=c.update({now:12100,telemetry:t,connected:true});assert.equal(r.throttlePercent,0);assert.equal(r.steeringDegrees,0);assert.equal(r.status,'holding');
 t=telemetry(12200,{targetX:.2});t.uwbPath=reference(t);assert.equal(c.update({now:12200,telemetry:t,connected:true}).throttlePercent,0);
 t.armed=false;assert.equal(c.update({now:12200,telemetry:t,connected:true}).safe,false);
 t.armed=true;t.uwb.fresh=false;assert.equal(c.update({now:12200,telemetry:t,connected:true}).safe,false);
});
test('invalid path configuration is rejected before deployment',()=>{
 const cfg=JSON.parse(fs.readFileSync(new URL('../config/system.config.json',import.meta.url)));
 cfg.uwb_follow={...cfg.uwb_follow,...rawConfig,path_follow:{enabled:true,lookahead_min_m:5,lookahead_max_m:1}};
 assert.ok(validateSystemConfig(cfg).some(v=>v.includes('lookahead')));
 cfg.uwb_follow.path_follow={enabled:true};cfg.uwb_follow.gps_speed_closed_loop.enabled=false;
 assert.ok(validateSystemConfig(cfg).some(v=>v.includes('GPS speed feedback')));
});
function panel(){
 const nodes=new Map(),doc={getElementById:id=>{if(!nodes.has(id))nodes.set(id,{dataset:{},setAttribute(k,v){this[k]=v;}});return nodes.get(id);}};
 const ctx={Number,Math,JSON,Array};vm.createContext(ctx);vm.runInContext(fs.readFileSync(new URL('../frontend/js/uwb-path-panel.js',import.meta.url),'utf8'),ctx);
 return {api:ctx.MantaUwbPathPanel,doc,get:id=>doc.getElementById(id)};
}
test('UI distinguishes route readiness from control, and clears overlays when reference ages out',()=>{
 const a=panel(),t=telemetry(10000),p={...reference(t),enabled:true,bodyPath:[{x:0,y:0},{x:1,y:1}],bodyObservations:[],bodyLookahead:{x:1,y:1},lookaheadM:.6};
 a.api.render({uwb:t.uwb,path:p,follow:{active:false},now:10000,range:2,document:a.doc});
 assert.equal(a.get('uwbPathStatus').textContent,'路径已就绪');assert.equal(a.get('uwbOptimizedPath').visibility,'visible');
 a.api.render({uwb:t.uwb,path:p,follow:{active:true,pathFollowActive:true,updatedAt:10000},now:10000,range:2,document:a.doc});
 assert.equal(a.get('uwbPathStatus').textContent,'优化跟随中');
 a.api.render({uwb:t.uwb,path:p,now:11000,range:2,document:a.doc});assert.equal(a.get('uwbOptimizedPath').visibility,'hidden');assert.equal(a.get('uwbPathSpeed').textContent,'--');
});
test('green target and optimized route share body calibration and fixed chart range',()=>{
 const a=panel();const body=a.api.bodyMeasurement({distanceM:2,azimuthDeg:30,coordinateCalibration:{bearingSign:-1,bearingScale:1,bearingOffsetDeg:0}});
 assert.ok(Math.abs(body.x-Math.sqrt(3))<1e-6);assert.ok(Math.abs(body.y+1)<1e-6);
 const t=telemetry(10000),p={...reference(t),enabled:true,bodyPath:[{x:0,y:0},{x:10,y:0}]};
 a.api.render({uwb:t.uwb,path:p,now:10000,range:2,document:a.doc});
 assert.equal(a.get('uwbOptimizedPath').points,'120.00,110.00 120.00,-310.00');
 // The SVG clips at the fixed range instead of bending an offscale path.
 const html=fs.readFileSync(new URL('../frontend/mobile-preview-kimi-k26.html',import.meta.url),'utf8');assert.match(html,/clip-path="url\(#uwbRouteClip\)"/);
});
