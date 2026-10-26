import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import {UwbGpsDriftHold} from '../backend/uwb_gps_drift_hold.js';
import {UwbFollowController} from '../backend/uwb_follow.js';
import {IntermittentUwbFollow} from '../backend/uwb_target_prediction.js';
const config={waiting_drift_hold:true,safety_distance_m:.3,max_speed_mps:2,max_reverse_speed_mps:.2,max_yaw_rate_deg_s:14,
 relative_steering_limit_deg:18,prediction:{max_speed_mps:.35,max_steering_deg:6,wait_timeout_ms:5000}};
const waiting={safe:true,pausePropulsion:true,status:'waiting_target',controlMode:'relative_acro'};
function packet(t=1000){return {armed:true,flightMode:'ACRO',gps:{fixType:3},ekf:{healthy:true,updatedAt:t},
 position:{lat:30,lon:120,updatedAt:t},attitude:{yaw:0,yawRateDegS:0,updatedAt:t},velocity:{vx:0,vy:0},acroControl:{ready:true,updatedAt:t}};}
test('waiting entry activates native speed loop, backward drift is corrected with bounded forward speed',()=>{
 const h=new UwbGpsDriftHold(config);let p=packet();assert.equal(h.update(1000,p,waiting).forwardSpeedMps,-.03);
 p=packet(2000);p.position.lat-=1/111319.49;p.velocity.vx=-.8;
 const r=h.update(2000,p,waiting);assert.equal(r.active,true);assert.equal(r.forwardSpeedMps,.35);assert.ok(r.forwardDisplacementM<-.99);
});
test('forward overshoot retreats, heading correction is limited and uses only own navigation',()=>{
 const h=new UwbGpsDriftHold(config);h.track(900,packet(900),{safe:true,bearingDeg:30,distanceM:2});h.update(1000,packet(),waiting);
 const p=packet(1100);p.position.lat+=1/111319.49;p.velocity.vx=2;
 const r=h.update(1100,p,waiting);assert.equal(r.forwardSpeedMps,-.2);assert.equal(r.yawRateDegS,14*6/18);assert.equal(r.anchorHeadingDeg,30);
});
test('navigation failure, missing fix, hard stop and five-second expiry return zero commands',()=>{
 for(const mutate of [p=>p.armed=false,p=>p.flightMode='MANUAL',p=>delete p.gps,p=>p.gps.fixType=2,
  p=>p.ekf.healthy=false,p=>p.position.updatedAt=200,p=>p.attitude.updatedAt=1200,p=>p.position.lat=NaN,
  p=>p.velocity.vx=NaN,p=>p.acroControl.ready=false,p=>p.uwb={fresh:true,distanceM:.3}]){
  const h=new UwbGpsDriftHold(config),p=packet();mutate(p);const r=h.update(1000,p,waiting);assert.equal(r.active,false);assert.equal(r.forwardSpeedMps,0);assert.equal(r.yawRateDegS,0);
 }
 const h=new UwbGpsDriftHold(config);h.update(1000,packet(),waiting);assert.equal(h.update(6000,packet(6000),waiting).reason,'hold_timeout');
 assert.equal(h.update(6100,packet(6100),{...waiting,status:'holding'}).active,false);
});
test('tracking clears waiting anchor, reset removes old target heading, feature defaults disabled',()=>{
 const h=new UwbGpsDriftHold(config);h.update(1000,packet(),waiting);h.track(1100,packet(1100),{safe:true,bearingDeg:10,distanceM:2});assert.equal(h.anchor,null);
 h.reset();assert.equal(h.lastTracking,null);assert.equal(new UwbGpsDriftHold().update(1000,packet(),waiting).active,false);
});

test('three calibrated live headings steer during position wait without moving anchor or extending timeout',()=>{
 const h=new UwbGpsDriftHold({...config,bearing_sign:-1,bearing_scale:1,bearing_offset_deg:0});let r;
 for(const now of [1000,1100,1200]){
  const p=packet(now);p.uwb={fresh:true,measurementStatus:'trusted',distanceM:4,azimuthDeg:-30,
   measurementUpdatedAt:now,sensors:[{receiverId:1,tagId:2,valid:true,fresh:true,measurementStatus:'trusted',receivedMeasurementAt:now}]};
  r=h.update(now,p,waiting);
  if(now<1200)assert.equal(r.liveBearingConfirmed,false);
 }
 assert.equal(r.liveBearingConfirmed,true);assert.equal(r.yawRateDegS,14);
 assert.equal(r.forwardSpeedMps,-.03);assert.equal(h.anchor.enteredAt,1000);
 const stale=h.update(1800,packet(1800),waiting);assert.equal(stale.liveBearingConfirmed,false);
 assert.ok(Math.abs(stale.yawRateDegS)<=14*6/18);assert.equal(h.update(6000,packet(6000),waiting).reason,'hold_timeout');
});

test('single angular spike, rejected data and duplicate snapshots cannot unlock waiting yaw',()=>{
 const h=new UwbGpsDriftHold({...config,bearing_sign:1,bearing_scale:1,bearing_offset_deg:0});
 for(const [now,at,az,status] of [[1000,1000,0,'trusted'],[1100,1100,40,'trusted'],[1200,1100,40,'trusted'],[1300,1300,0,'suspect']]){
  const p=packet(now);p.uwb={fresh:true,measurementStatus:status,distanceM:4,azimuthDeg:az,measurementUpdatedAt:at,
   sensors:[{receiverId:1,fresh:true,valid:true,measurementStatus:status,receivedMeasurementAt:at}]};
  assert.equal(h.update(now,p,waiting).liveBearingConfirmed,false);
 }
});
test('production waiting branch sends GPS assistance over ROVER_ACRO and records diagnostics',()=>{
 const source=fs.readFileSync(new URL('../backend/server.js',import.meta.url),'utf8');
 const holding=source.slice(source.indexOf('function maintainUwbHoldingMode('),source.indexOf('function maintainUwbResumeTransition('));
 const commands=[],p=packet();p.velocity.vx=-.5;
 const box={Date:{now:()=>1000},systemState:{telemetry:p},uwbGpsDriftHold:new UwbGpsDriftHold(config),
  lastRoverCommandAt:0,holdOutputsAreNeutral:()=>false,sendMavlinkCommand:(name,payload)=>commands.push({name,payload}),r:{...waiting}};
 const context=vm.createContext(box);vm.runInContext(holding+';maintainUwbHoldingMode(r)',context);
 assert.equal(commands[0].name,'ROVER_ACRO');assert.equal(commands[0].payload.forwardSpeedMps,.125);assert.equal(box.r.gpsDriftHold.active,true);
});


const directionalConfig={...config,bearing_sign:1,bearing_scale:1,bearing_offset_deg:0};
function rawPacket(t,az=0,range=12,status='suspect'){
 const p=packet(t);p.uwb={fresh:status==='trusted',online:status==='trusted',measurementStatus:status,
  distanceM:status==='trusted'?range:null,azimuthDeg:status==='trusted'?az:null,measurementUpdatedAt:t,
  sensors:[{receiverId:1,tagId:2,receivedMeasurementAt:t,rawDistanceUpdatedAt:t,rawDistanceM:range,
   rawAzimuthDeg:az,measurementStatus:status,valid:true,filterUncertaintyM:.4}]};return p;
}
function certifiedHold(range=12,uncertainty=.2){
 const h=new UwbGpsDriftHold(directionalConfig),p=rawPacket(900,0,range,'trusted');
 h.track(900,p,{safe:true,bearingDeg:0,distanceM:range,targetEstimate:{measurementConfirmed:true,uncertaintyM:uncertainty,lastMeasuredAt:900}});
 return h;
}
test('stable raw bearing permits bounded progress despite rejected joint position KF and inconsistent ranges',()=>{
 const h=certifiedHold();let r;
 for(let t=1000;t<=1600;t+=100){r=h.update(t,rawPacket(t,0,t%200?8:15),waiting);
  if(t<1200)assert.equal(r.bearingProgress,false);
 }
 assert.equal(r.bearingProgress,true);assert.equal(r.reason,'bearing_progress');
 assert.ok(r.forwardSpeedMps>0&&r.forwardSpeedMps<=.35);assert.ok(r.creepClearanceM>0);
 assert.equal(r.positionHoldIsLongitudinal,false);assert.equal(h.anchor.enteredAt,1000);
 assert.equal(h.rangeCertificate.at,900,'suspect position cannot refresh range certificate');
});
test('no range certificate, exhausted range bound or near raw measurement veto directional progress',()=>{
 for(const h of [new UwbGpsDriftHold(directionalConfig),certifiedHold(1.8),certifiedHold(12,11)]){
  let r;for(let t=1000;t<=1400;t+=100)r=h.update(t,rawPacket(t),waiting);
  assert.equal(r.bearingProgress,false);
 }
 const h=certifiedHold();let r;for(let t=1000;t<=1400;t+=100)r=h.update(t,rawPacket(t),waiting);
 assert.equal(r.bearingProgress,true);r=h.update(1500,rawPacket(1500,0,.8),waiting);
 assert.equal(r.bearingProgress,false);assert.equal(h.creepSpeed,0);
});
test('bearing identity, single angular outlier, radio loss and duplicate clocks cannot authorize progress',()=>{
 const h=certifiedHold();for(let t=1000;t<=1400;t+=100)h.update(t,rawPacket(t),waiting);
 assert.equal(h.update(1500,rawPacket(1500,40),waiting).bearingProgress,false);
 const bad=rawPacket(1600);bad.uwb.sensors[0].valid=false;
 assert.equal(h.update(1600,bad,waiting).bearingProgress,false);
 const changed=rawPacket(1700);changed.uwb.sensors[0].tagId=3;
 assert.equal(h.update(1700,changed,waiting).bearingProgress,false);assert.equal(h.rangeCertificate,null);
 const duplicate=certifiedHold();for(let t=1000;t<=1400;t+=100){const p=rawPacket(t);p.uwb.sensors[0].receivedMeasurementAt=1000;
  assert.equal(duplicate.update(t,p,waiting).bearingProgress,false);
 }
});
test('live bearing rotates with IMU, slows before FOV edge and expires at the same five seconds',()=>{
 const h=certifiedHold();let r;
 for(let t=1000;t<=1500;t+=100){const p=rawPacket(t,-(t-1000)*.01);p.attitude.yaw=(t-1000)*.01;p.attitude.yawRateDegS=10;
  r=h.update(t,p,waiting);}
 assert.equal(r.liveBearingConfirmed,true);assert.ok(Math.abs(r.anchorHeadingDeg)<1e-6);
 for(let t=1600;t<=2200;t+=100)r=h.update(t,rawPacket(t,36),waiting);
 assert.equal(r.liveBearingConfirmed,true);assert.equal(r.bearingProgress,false);
 assert.equal(h.update(6000,rawPacket(6000),waiting).reason,'hold_timeout');
 const missing=h.update(7000,packet(7000),waiting);assert.equal(missing.forwardSpeedMps,0);
});
test('three independent range observations can refresh clearance without accepting the joint position trajectory',()=>{
 const h=new UwbGpsDriftHold(directionalConfig);let r;
 for(let t=1000;t<=1500;t+=100)r=h.update(t,rawPacket(t,0,12,'trusted'),waiting);
 assert.equal(r.bearingProgress,true);assert.equal(h.rangeCertificate.measurementAt,1500);
 const before=h.rangeCertificate.at;
 const p=rawPacket(1550,0,12,'trusted');p.uwb.measurementUpdatedAt=1500;p.uwb.sensors[0].receivedMeasurementAt=1500;
 h.update(1550,p,waiting);assert.equal(h.rangeCertificate.at,before);
 assert.equal(h.anchor.enteredAt,1000);
});
test('entering a new wait after legitimate progress does not undo gained GPS distance',()=>{
 const h=certifiedHold();let p,r;
 for(let t=1000;t<=1600;t+=100){p=rawPacket(t);p.position.lat+=(t-1000)*.00000001;r=h.update(t,p,waiting);}
 assert.equal(r.bearingProgress,true);const heldLat=h.anchor.lat;
 p=packet(1700);p.position.lat=heldLat;r=h.update(1700,p,waiting);
 assert.equal(r.bearingProgress,false);assert.ok(Math.abs(r.forwardDisplacementM)<1e-8);
 assert.equal(h.anchor.enteredAt,1000);
});

test('directional progress cannot refresh the production estimator five-second position-loss deadline',()=>{
 const c=new UwbFollowController({...directionalConfig,execution_mode:'acro',controller_profile:'traditional_fast',hard_stop_enabled:false});
 const session=new IntermittentUwbFollow({enabled:true},{bearing_sign:1,bearing_scale:1,bearing_offset_deg:0});
 const hold=new UwbGpsDriftHold(directionalConfig);let progressed=false,result;
 const run=(t,status)=>{
  const p=rawPacket(t,0,12,status);p.pilotSteering={ready:true,type:0};
  result=session.update({controller:c,now:t,telemetry:p,connected:true,controlMode:'relative_acro'});
  if(result.pausePropulsion){const h=hold.update(t,p,result);progressed||=h.bearingProgress===true;}
  else if(result.safe)hold.track(t,p,result);
 };
 for(let t=1000;t<2000;t+=100)run(t,'trusted');
 for(let t=2000;t<7000;t+=100){run(t,'suspect');assert.equal(session.estimator.lossAt,2000);}
 assert.equal(progressed,true);run(7000,'suspect');
 assert.equal(result.reason,'prediction_wait_timeout');assert.equal(result.safe,false);
 assert.equal(result.forwardSpeedMps,0);assert.equal(c.config.safetyDistanceM,.3);
});

test('one bad angle pauses progress but preserves earned speed for a fresh safe reacquisition',()=>{
 const h=certifiedHold();let r;
 for(let t=1000;t<=1800;t+=100)r=h.update(t,rawPacket(t),waiting);
 const earned=r.forwardSpeedMps;assert.ok(earned>.15);
 r=h.update(1900,rawPacket(1900,40),waiting);assert.equal(r.bearingProgress,false);
 assert.equal(h.creepSpeed,earned);assert.ok(r.forwardSpeedMps<=0);
 r=h.update(2000,rawPacket(2000),waiting);assert.equal(r.bearingProgress,true);
 assert.ok(r.forwardSpeedMps>=earned);assert.ok(r.forwardSpeedMps<=.35);
 const missing=packet(2600);h.update(2600,missing,waiting);assert.equal(h.creepSpeed,0);
 assert.equal(h.anchor.enteredAt,1000);
});

test('transition from normal moving follow retains a bounded measured speed, navigation faults clear it',()=>{
 const h=certifiedHold(),p=rawPacket(1000,0,12,'trusted');p.velocity.vx=.8;
 h.track(1000,p,{safe:true,bearingDeg:0,distanceM:12,targetEstimate:{measurementConfirmed:true,uncertaintyM:.2,lastMeasuredAt:1000}});
 assert.equal(h.creepSpeed,.35);let r;
 for(let t=1100;t<=1300;t+=100)r=h.update(t,rawPacket(t),waiting);
 assert.equal(r.forwardSpeedMps,.35);
 const broken=rawPacket(1400);broken.ekf.healthy=false;
 assert.equal(h.update(1400,broken,waiting).forwardSpeedMps,0);assert.equal(h.creepSpeed,0);
});

function predictedWait(t,{range=4,bearing=0,uncertainty=.4,at=900}={}){
 return {...waiting,targetEstimate:{admissionVersion:1,source:'predicted',evaluatedAt:t,
  lastMeasuredAt:at,uncertaintyM:uncertainty,relativeBodyM:{x:range*Math.cos(bearing*Math.PI/180),y:range*Math.sin(bearing*Math.PI/180)}}};
}
test('far pursuit tolerates rejected XY using admitted range bound and live bearing, never raw jump alone',()=>{
 const h=certifiedHold(12);h.c={...h.c,far_pursuit_enabled:true};let r;
 for(let t=1000;t<=2800;t+=100)r=h.update(t,rawPacket(t,0,t%200?8:18),waiting);
 assert.equal(r.farProgress,true);assert.ok(r.forwardSpeedMps>.35&&r.forwardSpeedMps<=1);
 assert.equal(r.liveBearingConfirmed,true);assert.ok(r.farRangeLowerBoundM>=3);
 assert.ok(r.farClearanceM>0);assert.equal(h.rangeCertificate.measurementAt,900);
 const no=new UwbGpsDriftHold({...directionalConfig,far_pursuit_enabled:true});
 for(let t=1000;t<=1800;t+=100)r=no.update(t,rawPacket(t,0,19),waiting);
 assert.equal(r.farProgress,false);assert.ok(r.forwardSpeedMps<=.35);
 assert.equal(h.update(6000,rawPacket(6000),waiting).reason,'hold_timeout');
});
test('far pursuit closes at low clearance, excessive stopping distance, angular edge and navigation loss',()=>{
 for(const mode of ['near','braking','angle','navigation','radio','identity','duplicate']){
  const h=certifiedHold(6);h.c={...h.c,far_pursuit_enabled:true};let r;
  for(let t=1000;t<=1300;t+=100)r=h.update(t,rawPacket(t),waiting);
  assert.equal(r.farProgress,true);
  const p=rawPacket(1400);
  if(mode==='near')p.uwb.sensors[0].rawDistanceM=.8;
  if(mode==='braking')p.velocity.vx=2;
  if(mode==='angle')p.uwb.sensors[0].rawAzimuthDeg=40;
  if(mode==='navigation')p.ekf.healthy=false;
  if(mode==='radio')p.uwb.sensors[0].valid=false;
  if(mode==='identity')p.uwb.sensors[0].tagId=3;
  if(mode==='duplicate')p.uwb.sensors[0].receivedMeasurementAt=1000;
  r=h.update(1400,p,waiting);assert.notEqual(r.farProgress,true);assert.ok(r.forwardSpeedMps<=.35);
 }
 const h=certifiedHold(6);h.c={...h.c,far_pursuit_enabled:true};let r;
 for(let t=1000;t<=1900;t+=100)r=h.update(t,rawPacket(t),waiting);
 assert.equal(r.farProgress,true);assert.ok(r.farRangeLowerBoundM<3);
 r=h.update(2100,rawPacket(2100),waiting);assert.equal(r.farProgress,false);
 assert.equal(h.rangeCertificate.at,900);assert.equal(h.anchor.enteredAt,1000);
});
test('accepted GPS/IMU prediction permits bounded progress without a new raw range or live bearing',()=>{
 const h=certifiedHold(4),p=packet(1500);p.velocity.vx=.4;
 const r=h.update(1500,p,predictedWait(1500));
 assert.equal(r.predictedProgress,true);assert.equal(r.bearingProgress,false);
 assert.equal(r.liveBearingConfirmed,false);assert.ok(r.forwardSpeedMps>0&&r.forwardSpeedMps<=.35);
 assert.ok(r.acceptedPrediction.clearanceM>0);assert.equal(r.acceptedPrediction.ageMs,600);
 assert.equal(h.rangeCertificate.measurementAt,900);assert.equal(h.rangeCertificate.at,900);
 assert.equal(h.update(1901,packet(1901),predictedWait(1901)).predictedProgress,false);
 assert.equal(h.update(6500,packet(6500),predictedWait(6500)).reason,'hold_timeout');
});
test('prediction requires the exact admitted anchor, fresh evaluation, braking room and no close raw veto',()=>{
 for(const change of [r=>r.targetEstimate.lastMeasuredAt=1000,r=>r.targetEstimate.evaluatedAt=1399,
  r=>r.targetEstimate.evaluatedAt=1501,r=>r.targetEstimate.admissionVersion=0,
  r=>r.targetEstimate.uncertaintyM=NaN,r=>r.targetEstimate.uncertaintyM=3,
  r=>r.targetEstimate.relativeBodyM={x:1,y:3}]){
  const h=certifiedHold(4),r=predictedWait(1500);change(r);
  assert.equal(h.update(1500,packet(1500),r).predictedProgress,false);
 }
 const fast=packet(1500);fast.velocity.vx=1.5;
 assert.equal(certifiedHold(4).update(1500,fast,predictedWait(1500)).predictedProgress,false);
 const near=rawPacket(1500,0,.8);
 assert.equal(certifiedHold(4).update(1500,near,predictedWait(1500)).predictedProgress,false);
 const h=certifiedHold(4),changed=rawPacket(1500);changed.uwb.sensors[0].tagId=3;
 assert.equal(h.update(1500,changed,predictedWait(1500)).predictedProgress,false);
});
test('live heading has priority during admitted prediction, then exhausted margin returns to GPS hold',()=>{
 const h=certifiedHold(4);let r;
 for(const t of [1300,1400,1500])r=h.update(t,rawPacket(t,25),predictedWait(t));
 assert.equal(r.liveBearingConfirmed,true);assert.equal(r.anchorHeadingDeg,25);
 const lat=h.anchor.lat;
 r=h.update(1600,packet(1600),predictedWait(1600,{uncertainty:4}));
 assert.equal(r.predictedProgress,false);assert.equal(r.reason,'gps_drift_hold');
 assert.equal(h.anchor.lat,lat);assert.equal(h.anchor.enteredAt,1300);
 h.track(1700,rawPacket(1700,0,4,'trusted'),{safe:true,bearingDeg:0,distanceM:4,
  targetEstimate:{measurementConfirmed:true,uncertaintyM:.2,lastMeasuredAt:1700}});
 assert.equal(h.anchor,null);assert.equal(h.rangeCertificate.measurementAt,1700);
});
