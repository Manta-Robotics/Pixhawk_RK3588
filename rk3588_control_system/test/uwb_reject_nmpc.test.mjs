import test from 'node:test';
import assert from 'node:assert/strict';
import {UwbRejectNmpc} from '../backend/uwb_reject_nmpc.js';
import {UwbGpsDriftHold} from '../backend/uwb_gps_drift_hold.js';
import {IntermittentUwbFollow} from '../backend/uwb_target_prediction.js';
import {UwbFollowController} from '../backend/uwb_follow.js';
const c={waiting_drift_hold:true,reject_nmpc:{enabled:true,solve_budget_ms:25},safety_distance_m:.3,
 max_speed_mps:2,max_yaw_rate_deg_s:14,bearing_sign:1,bearing_scale:1,bearing_offset_deg:0,
 prediction:{wait_timeout_ms:5000,measurement_gap_ms:450,clearance_m:1.2,braking_deceleration_mps2:.3}};
function packet(now){return {armed:true,flightMode:'ACRO',gps:{fixType:3},ekf:{healthy:true,updatedAt:now},
 position:{lat:30,lon:120,updatedAt:now},attitude:{yaw:0,yawRateDegS:0,updatedAt:now},velocity:{vx:.5,vy:0},
 acroControl:{ready:true,updatedAt:now},uwb:{sensors:[{receiverId:1,tagId:2,valid:true,measurementStatus:'trusted',receivedMeasurementAt:now}]}};}
function accepted(now,range=6){return {safe:true,bearingDeg:0,distanceM:range,forwardSpeedMps:.6,yawRateDegS:0,
 targetEstimate:{admissionVersion:1,measurementConfirmed:true,referenceFrame:'local_ned',evaluatedAt:now,lastMeasuredAt:now,
 relativeBodyM:{x:range,y:0},uncertaintyM:.12,targetVelocityNedMps:{x:.6,y:0},velocityEstimate:{trusted:true}}};}
const waiting={safe:true,status:'waiting_target',pausePropulsion:true};
test('one rejected frame retains admitted posterior and yields bounded nonlinear forward trajectory',()=>{
 const n=new UwbRejectNmpc(c);n.observe(1000,packet(1000),accepted(1000));
 const bad=accepted(1100,99);bad.targetEstimate.measurementConfirmed=false;n.observe(1100,packet(1100),bad);
 const r=n.solve(1200,packet(1200),null);
 assert.equal(n.accepted.measurementAt,1000);assert.equal(r.feasible,true);assert.ok(r.forwardSpeedMps>0);
 assert.ok(r.forwardSpeedMps<=.63+1e-9);assert.equal(r.path.length,12);assert.ok(r.evaluations>10);
 for(const p of r.path){assert.ok(p.clearanceM>0);assert.ok(p.bearingBoundDeg<45);
  assert.ok(Math.abs(p.clearanceM-(p.rangeM-p.uncertaintyM-1.2-p.brakingDistanceM))<1e-9);}
});
test('independently trusted bearing permits an anticipatory turn without overwriting admitted range',()=>{
 const n=new UwbRejectNmpc(c);n.observe(1000,packet(1000),accepted(1000));
 const r=n.solve(1200,packet(1200),{trusted:true,at:1200,headingDeg:20,rateDegS:5});
 assert.equal(r.feasible,true);assert.ok(r.yawRateDegS>0);assert.ok(r.yawRateDegS<=8);
 assert.equal(n.accepted.measurementAt,1000);assert.equal(r.liveBearing,true);
});
test('close clearance, uncertain position, near raw sample, stale state and wrong target never authorize NMPC',()=>{
 for(const variant of ['close','uncertain','raw','stale','identity','no_admission','nan']){
  const n=new UwbRejectNmpc(c),a=accepted(1000,variant==='close'?1:6),p=packet(1200);
  if(variant==='uncertain')a.targetEstimate.uncertaintyM=6;
  if(variant==='no_admission')a.targetEstimate.admissionVersion=0;
  n.observe(1000,packet(1000),a);
  if(variant==='identity')p.uwb.sensors[0].tagId=3;
  if(variant==='nan')p.velocity.vx=NaN;
  assert.equal(n.solve(variant==='stale'?6000:1200,p,null,variant==='raw').feasible,false,variant);
 }
});
test('duplicates cannot replace posterior, prediction and raw bearing cannot refresh five-second clock',()=>{
 const n=new UwbRejectNmpc(c);n.observe(1000,packet(1000),accepted(1000));
 n.observe(1000,packet(1000),accepted(1000,99));assert.equal(n.accepted.x,6);
 for(let t=1100;t<=6000;t+=100){n.solve(t,packet(t),{trusted:true,at:t,headingDeg:0,rateDegS:0});assert.equal(n.accepted.measurementAt,1000);}
 assert.equal(n.solve(6000,packet(6000),null).reason,'accepted_state_expired');
});
test('future FOV violation rejects all trajectories and trusted velocity is not fabricated',()=>{
 const n=new UwbRejectNmpc(c);const a=accepted(1000);a.targetEstimate.velocityEstimate.trusted=false;
 n.observe(1000,packet(1000),a);assert.deepEqual(n.accepted.velocity,{x:0,y:0});
 assert.equal(n.solve(1200,packet(1200),{trusted:true,at:1200,headingDeg:60,rateDegS:0}).feasible,false);
});
test('production hold uses NMPC on rejection, preserves GPS fallback, navigation guards and timeout',()=>{
 const h=new UwbGpsDriftHold(c);h.track(1000,packet(1000),accepted(1000));
 let r=h.update(1200,packet(1200),waiting);assert.equal(r.reason,'reject_nmpc');assert.equal(r.nmpcProgress,true);
 assert.equal(h.update(6200,packet(6200),waiting).reason,'hold_timeout');
 const p=packet(1300);p.ekf.healthy=false;assert.equal(h.update(1300,p,waiting).active,false);
 const close=new UwbGpsDriftHold(c);close.track(1000,packet(1000),accepted(1000,1));
 r=close.update(1200,packet(1200),waiting);assert.equal(r.reason,'gps_drift_hold');assert.equal(r.nmpcProgress,false);
 assert.ok(r.forwardSpeedMps<=.35);
});
test('fresh accepted observation updates target after a rejected interval; reset discards the previous plan',()=>{
 const n=new UwbRejectNmpc(c);n.observe(1000,packet(1000),accepted(1000));n.solve(1200,packet(1200),null);
 n.observe(1300,packet(1300),accepted(1300,5));assert.equal(n.accepted.measurementAt,1300);
 assert.equal(n.accepted.x,5);n.reset();assert.equal(n.solve(1400,packet(1400),null).feasible,false);
});
test('solver deadline discards its old feasible trajectory instead of executing a stale plan',()=>{
 const n=new UwbRejectNmpc(c);n.observe(1000,packet(1000),accepted(1000));
 assert.equal(n.solve(1200,packet(1200),null).feasible,true);
 const original=globalThis.performance;let clock=0;
 try{
  globalThis.performance={now:()=>clock+=30};
  const r=n.solve(1300,packet(1300),null);assert.equal(r.feasible,false);assert.equal(r.reason,'solver_deadline');assert.equal(n.plan,null);
 }finally{globalThis.performance=original;}
});
test('clock rollback cannot reduce uncertainty or advance from a future admitted state',()=>{
 const n=new UwbRejectNmpc(c),r=accepted(1100);r.targetEstimate.lastMeasuredAt=1000;
 const p=packet(1100);p.uwb.sensors[0].receivedMeasurementAt=1000;n.observe(1100,p,r);
 assert.ok(n.accepted);assert.equal(n.solve(1050,packet(1050),null).feasible,false);
});
test('first Reject routes directly to NMPC instead of 450 ms coast; admitted recovery returns to normal',()=>{
 const cfg={...c,execution_mode:'acro',prediction:{...c.prediction,enabled:true},hard_stop_enabled:false};
 const session=new IntermittentUwbFollow(cfg.prediction,cfg),controller=new UwbFollowController(cfg),hold=new UwbGpsDriftHold(cfg);
 function telemetry(t,rejected=false){
  const p=packet(t);p.pilotSteering={ready:true,type:0};
  const at=rejected?t-100:t;
  p.uwb={online:true,fresh:!rejected,distanceM:6,azimuthDeg:0,measurementUpdatedAt:at,
   measurementStatus:rejected?'suspect':'trusted',activeReceiverIds:[1],sensors:[{
    receiverId:1,tagId:2,valid:true,filterAdmissionVersion:1,measurementStatus:rejected?'suspect':'trusted',
    measurementUpdatedAt:at,receivedMeasurementAt:t,rawAzimuthDeg:0,rawDistanceM:rejected?15:6,
    filterUncertaintyM:.12,sequenceId:t}]};return p;
 }
 const run=(now,rejected=false)=>session.update({controller,now,telemetry:telemetry(now,rejected),connected:true,controlMode:'relative_acro'});
 for(let t=1000;t<=2000;t+=100){const r=run(t);assert.equal(r.safe,true);hold.track(t,telemetry(t),r);}
 // Seed an eligible old coast; the enabled path must not invoke it at all.
 session.admittedDrive={at:2000,speed:.5};session.coastAcceptedPosition=()=>{throw new Error('legacy coast called');};
 const r=run(2100,true);assert.equal(r.nmpcRequested,true);assert.equal(r.pausePropulsion,true);
 assert.equal(r.reason,'reject_nmpc_planning');assert.equal(r.targetEstimate.predictionAgeMs,100);
 const command=hold.update(2100,telemetry(2100,true),r);assert.ok(command.nmpc,'NMPC invoked on first rejection');
 const recovered=run(2200);assert.equal(recovered.targetEstimate.measurementConfirmed,true);
 assert.notEqual(recovered.nmpcRequested,true);assert.notEqual(recovered.pausePropulsion,true);
 const lost=telemetry(7200,true);lost.uwb.measurementUpdatedAt=2200;lost.uwb.sensors[0].measurementUpdatedAt=2200;
 // Rejections cannot renew the estimator loss clock.
 run(2300,true);
 const stopped=session.update({controller,now:7300,telemetry:lost,connected:true,controlMode:'relative_acro'});
 assert.equal(stopped.safe,false);
});
const adaptive={...c,reject_nmpc:{...c.reject_nmpc,clearance_policy:'adaptive'}};
test('invalid zero and invalid radio packet are unknown, not close obstacles or admitted measurements',()=>{
 for(const [range,valid,kind] of [[0,false,'invalid_zero_unknown'],[.2,false,'no_near_range']]){
  const n=new UwbRejectNmpc(adaptive);n.observe(1000,packet(1000),accepted(1000));
  const p=packet(1200);Object.assign(p.uwb.sensors[0],{rawDistanceM:range,valid,rawDistanceUpdatedAt:1200,measurementStatus:'suspect'});
  const e=n.rangeEvidence(1200,p);assert.equal(e.kind,kind);assert.equal(e.veto,false);assert.equal(e.lowerBoundM,null);
  const r=n.solve(1200,p,null,e);assert.equal(r.feasible,true);assert.ok(r.forwardSpeedMps>0);
  assert.equal(n.accepted.measurementAt,1000);assert.equal(n.solve(6000,packet(6000),null,e).feasible,false);
 }
});
test('positive plausible near ranges still veto or tighten braking even when their position KF rejected them',()=>{
 const n=new UwbRejectNmpc(adaptive);n.observe(1000,packet(1000),accepted(1000));
 for(const range of [.2,.4,.8]){
  const p=packet(1200);Object.assign(p.uwb.sensors[0],{rawDistanceM:range,valid:true,rawDistanceUpdatedAt:1200,measurementStatus:'suspect'});
  const e=n.rangeEvidence(1200,p);assert.equal(e.kind,'positive_near_range');assert.ok(e.lowerBoundM<range);
  const r=n.solve(1200,p,null,e);assert.equal(r.feasible,false);assert.equal(n.accepted.x,6);
 }
});
test('adaptive reserve admits a braking-safe trajectory below old 1.2 m buffer while retaining safety and uncertainty',()=>{
 const p=packet(1000);p.velocity.vx=.15;const a=accepted(1000,1.1);a.forwardSpeedMps=.2;
 const old=new UwbRejectNmpc(c),n=new UwbRejectNmpc(adaptive);old.observe(1000,p,a);n.observe(1000,p,a);
 const t=packet(1100);t.velocity.vx=.15;
 assert.equal(old.solve(1100,t,null).reason,'no_initial_braking_clearance');
 const r=n.solve(1100,t,null,n.rangeEvidence(1100,t));assert.equal(r.feasible,true);assert.ok(r.forwardSpeedMps>0);
 assert.ok(r.bufferM>=.4&&r.bufferM<1.2);assert.equal(r.safetyDistanceM,.3);
 for(const q of r.path){assert.ok(q.rangeLowerBoundM-q.brakingDistanceM>r.bufferM);assert.ok(q.uncertaintyM>0);}
});
test('near-range evidence ages conservatively and can only reduce the planning envelope',()=>{
 const n=new UwbRejectNmpc(adaptive);n.observe(1000,packet(1000),accepted(1000));
 const p=packet(1200);p.velocity.vx=.1;Object.assign(p.uwb.sensors[0],{rawDistanceM:1.1,rawDistanceUpdatedAt:1200,measurementStatus:'suspect'});
 const e=n.rangeEvidence(1200,p),later=n.rangeEvidence(1300,p);assert.ok(later.lowerBoundM<e.lowerBoundM);
 const r=n.solve(1200,p,null,e);assert.equal(r.feasible,true);
 for(const q of r.path){assert.ok(q.rangeLowerBoundM<=q.rangeM-q.uncertaintyM);assert.ok(q.rangeLowerBoundM<=q.rawRangeLowerBoundM);}
 assert.equal(n.accepted.measurementAt,1000);
});
test('credible near return blocks forward GPS hold, but zero with fresh flag does not assert zero distance',()=>{
 const h=new UwbGpsDriftHold(adaptive);h.track(1000,packet(1000),accepted(1000));h.update(1100,packet(1100),waiting);
 const near=packet(1200);near.position.lat-=1/111319.49;near.velocity.vx=-.5;
 Object.assign(near.uwb.sensors[0],{rawDistanceM:.2,rawDistanceUpdatedAt:1200,measurementStatus:'suspect'});
 const held=h.update(1200,near,waiting);assert.equal(held.nmpcProgress,false);assert.ok(held.forwardSpeedMps<=0);
 const zero=packet(1300);zero.uwb.fresh=true;zero.uwb.distanceM=0;
 Object.assign(zero.uwb.sensors[0],{rawDistanceM:0,rawDistanceUpdatedAt:1300,valid:false});
 const unknown=h.update(1300,zero,waiting);assert.notEqual(unknown.reason,'safety_distance');assert.notEqual(unknown.nmpc?.reason,'near_raw_veto');
});
