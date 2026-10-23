import test from 'node:test';
import assert from 'node:assert/strict';
import {UwbTargetEstimator,IntermittentUwbFollow,usableControlEstimate} from '../backend/uwb_target_prediction.js';
import {UwbFollowController} from '../backend/uwb_follow.js';

const calibration={bearing_sign:1,bearing_scale:1,bearing_offset_deg:0};
function frame(now,range=1.4,status='trusted',at=now) {
  const trusted=status==='trusted';
  return {armed:true,flightMode:'ACRO',pilotSteering:{ready:true,type:0},acroControl:{ready:true,updatedAt:now},
    attitude:{yaw:0,updatedAt:now},velocity:{vx:0,vy:0},position:{updatedAt:now},ekf:{healthy:true,updatedAt:now},gps:{fixType:6},
    uwb:{online:trusted,fresh:trusted,distanceM:trusted?range:null,azimuthDeg:trusted?0:null,
      measurementStatus:status,measurementUpdatedAt:trusted?at:1400,activeReceiverIds:trusted?[1]:[],
      sensors:[{receiverId:1,sequenceId:at,receivedMeasurementAt:at,rawDistanceM:range,rawAzimuthDeg:0,
        tagId:1,valid:status!=='invalid',measurementStatus:status,filterUncertaintyM:.2}]}};
}
function estimator() {
  const e=new UwbTargetEstimator({enabled:true},calibration);
  for(const t of [1000,1100,1200])e.update(t,frame(t,.9));
  e.update(1300,frame(1300,1.15));e.update(1400,frame(1400,1.4));
  for(let t=1500;t<=3000;t+=100)e.update(t,frame(t,1.4,'suspect',1400));
  return e;
}
function update(e,t,packet) {
  const before=structuredClone(packet),result=e.update(t,packet);
  assert.deepEqual(packet,before);return result;
}
test('shared vessel estimator gathers raw candidates despite main KF rejection and a bad intervening frame',()=>{
  const e=estimator();
  update(e,3100,frame(3100,1.42,'suspect'));
  update(e,3200,frame(3200,9,'invalid'));
  update(e,3300,frame(3300,1.46,'suspect'));
  assert.equal(e.rawWindow.reacquisition.filter(p=>p.position).length,2);
  const r=update(e,3400,frame(3400,1.48));
  assert.equal(r.source,'recovering');assert.equal(r.lastMeasuredAt,3400);
  assert.equal(r.reacquisitionFit.samples,3);assert.equal(r.reacquisitionFit.window,4);
  assert.equal(r.velocityEstimate.trusted,false);
  assert.equal(r.targetVelocityNedMps.x,0);
  assert.ok(Math.abs(r.reacquisitionFit.rawFitVelocityNedMps.x-.2)<1e-9);
});
test('shared candidate clock ignores duplicate snapshots and metadata-only sequence changes',()=>{
  const e=estimator();update(e,3100,frame(3100,1.42,'suspect'));
  const duplicate=frame(3150,1.42,'suspect',3100);duplicate.uwb.sensors[0].sequenceId=999;
  update(e,3150,duplicate);assert.equal(e.rawWindow.reacquisition.length,1);
});
test('shared handoff rejects invalid covariance without accepting the candidate',()=>{
  const e=estimator();
  for(let t=3100;t<=3400;t+=100){
    const p=frame(t,1.42+(t-3100)*.0002);p.uwb.sensors[0].filterUncertaintyM=NaN;
    const r=update(e,t,p);assert.notEqual(r.source,'recovering');assert.equal(r.lastMeasuredAt,1400);
  }
  assert.equal(e.rawWindow.reacquisitionDiagnostic.reason,'measurement_handoff_invalid_uncertainty');
});
test('shared window never extends the original loss deadline',()=>{
  const e=estimator();
  for(let t=3100;t<6500;t+=100)update(e,t,frame(t,1.4,'suspect',1400));
  assert.equal(update(e,6500,frame(6500)).reason,'prediction_wait_timeout');
});

test('five fresh consistent measurements at 0.55 m covariance recover independently of the blind 0.5 m bound',()=>{
 const e=new UwbTargetEstimator({enabled:true},calibration);
 for(let t=1000;t<=1400;t+=100)e.update(t,frame(t,8));
 e.update(1500,frame(1500,8,'invalid'));
 let r;
 for(let t=1600;t<=2000;t+=100){const p=frame(t,8);p.uwb.sensors[0].filterUncertaintyM=.55;r=e.update(t,p);}
 assert.equal(r.usable,true);assert.equal(r.measurementConfirmed,true);assert.equal(r.source,'recovering');
 assert.equal(r.measurementUncertaintyM,.55);assert.equal(e.config.max_uncertainty_m,.5);
 assert.equal(usableControlEstimate(r,2000,{enabled:true}),true);
 const lost=e.update(2100,frame(2100,8,'invalid'));assert.equal(lost.source,'predicted');
 assert.equal(lost.usable,false);assert.equal(usableControlEstimate(lost,2100,{enabled:true}),false);
});

test('real ACRO recovery ramp completes on fresh 0.55 m measurements and produces a positive governed request',()=>{
 const c=new UwbFollowController({...calibration,prediction:{enabled:true},execution_mode:'acro',
  safety_distance_m:.3,hard_stop_enabled:false,controller_profile:'traditional_fast'});
 const s=new IntermittentUwbFollow({enabled:true},calibration);
 const run=(t,status='trusted',cov=.55)=>{const p=frame(t,8,status);p.uwb.sensors[0].filterUncertaintyM=cov;
  return s.update({controller:c,now:t,telemetry:p,connected:true,controlMode:'relative_acro'});};
 for(let t=1000;t<=1400;t+=100)run(t,'trusted',.2);
 assert.equal(run(1500,'invalid').status,'waiting_target');
 let r;for(let t=1600;t<=3000;t+=100)r=run(t);
 assert.equal(r.status,undefined);assert.equal(r.reason,'tracking');assert.equal(s.recoveryGoodMs,800);
 assert.ok(r.measurementClearanceM>7);assert.ok(r.forwardSpeedMps>0);
 assert.ok(s.governAcro(r,2,14,18,70).forwardSpeedMps>0);
 assert.equal(run(3100,'invalid').status,'waiting_target');
 for(let t=3200;t<8100;t+=100)run(t,'invalid');
 assert.equal(run(8100,'invalid').reason,'prediction_wait_timeout');
});

test('live covariance still constrains safety clearance and never gets erased on handoff',()=>{
 const c=new UwbFollowController({...calibration,prediction:{enabled:true},execution_mode:'acro',
  safety_distance_m:.3,hard_stop_enabled:false,controller_profile:'traditional_fast'});
 const s=new IntermittentUwbFollow({enabled:true},calibration);
 const p=frame(1000,.8);p.uwb.sensors[0].filterUncertaintyM=.55;
 const r=s.update({controller:c,now:1000,telemetry:p,connected:true});
 assert.equal(r.reason,'measurement_clearance');assert.equal(r.forwardSpeedMps,0);
 assert.equal(r.targetEstimate.measurementUncertaintyM,.55);assert.equal(c.config.safetyDistanceM,.3);
});
test('production ACRO wrapper recovers through the shared estimator and does not replace the vessel mode',()=>{
  const c=new UwbFollowController({...calibration,prediction:{enabled:true},execution_mode:'acro',
    manual_propulsion:'single_side',following_distance_m:.5,safety_distance_m:.3,hard_stop_enabled:false,controller_profile:'traditional_fast'});
  const session=new IntermittentUwbFollow({enabled:true},calibration);
  const run=(t,p)=>session.update({controller:c,now:t,telemetry:p,connected:true,controlMode:'relative_acro'});
  for(const t of [1000,1100,1200])assert.equal(run(t,frame(t,.9)).safe,true);
  const lost=run(1300,frame(1300,.9,'suspect',1200));assert.equal(lost.status,'waiting_target');
  assert.equal(lost.requiredFlightMode,'ACRO');assert.equal(lost.forwardSpeedMps,0);
  run(1400,frame(1400,.91,'suspect'));run(1500,frame(1500,.92,'suspect'));
  const recovered=run(1600,frame(1600,.93));
  assert.equal(recovered.status,'recovering');assert.equal(recovered.requiredFlightMode,'ACRO');
  assert.ok(recovered.targetEstimate.reacquisitionFit.samples>=3);
});

test('longer position history confirms velocity gradually within the existing acceleration bound',()=>{
  const e=new UwbTargetEstimator({enabled:true},calibration);let previous=0,result;
  for(let t=1000;t<=3000;t+=100) {
    result=update(e,t,frame(t,4+(t-1000)*.0002));
    assert.ok(Math.abs(result.targetVelocityNedMps.x-previous)<=.060001);
    previous=result.targetVelocityNedMps.x;
    if(t<1800)assert.equal(result.velocityEstimate.trusted,false);
  }
  assert.equal(result.velocityEstimate.trusted,true);
  assert.ok(Math.abs(result.targetVelocityNedMps.x-.2)<1e-8);
});

test('short position reacquisition cannot inject its two m/s raw slope into prediction',()=>{
  const e=new UwbTargetEstimator({enabled:true},calibration);
  for(let t=1000;t<=3000;t+=100)update(e,t,frame(t,4));
  for(let t=3100;t<=3800;t+=100)update(e,t,frame(t,4,'invalid'));
  for(let t=3900;t<=4300;t+=100)update(e,t,frame(t,6+(t-3900)*.002));
  assert.equal(e.last.at,4300);assert.equal(e.last.velocityTrusted,false);
  assert.equal(e.last.velocity.x,0);
  const predicted=update(e,4500,frame(4500,6.8,'suspect',4300));
  assert.equal(predicted.lastMeasuredAt,4300);
  assert.ok(Math.abs(predicted.relativeBodyM.x-6.8)<1e-8);
});

test('an isolated invalid observation retains distinct prior evidence without refreshing accepted time',()=>{
  const e=new UwbTargetEstimator({enabled:true},calibration);
  for(let t=1000;t<=1200;t+=100)update(e,t,frame(t,4+(t-1000)*.0002));
  const bad=update(e,1300,frame(1300,12,'invalid'));
  assert.equal(bad.lastMeasuredAt,1200);assert.equal(e.lossAt,1300);
  const restored=update(e,1400,frame(1400,4.08));
  assert.equal(restored.source,'recovering');assert.equal(restored.lastMeasuredAt,1400);
  assert.ok(restored.reacquisitionFit.samples>=3);
  assert.ok(restored.reacquisitionFit.window<=5);
});

test('a short interruption preserves earned recovery progress but never stored propulsion',()=>{
  const c=new UwbFollowController({...calibration,prediction:{enabled:true},execution_mode:'acro',
    manual_propulsion:'single_side',following_distance_m:.5,safety_distance_m:.3,hard_stop_enabled:false,controller_profile:'traditional_fast'});
  const session=new IntermittentUwbFollow({enabled:true},calibration);
  const run=(t,status='trusted')=>session.update({controller:c,now:t,telemetry:frame(t,1.4,status),connected:true});
  for(let t=1000;t<=1500;t+=100)run(t);
  assert.equal(run(1600,'invalid').status,'waiting_target');
  run(1700);run(1800);const before=run(1900);
  assert.ok(before.intermittentThrottleCap>0);
  const lost=run(2000,'invalid');assert.equal(lost.status,'waiting_target');assert.equal(lost.forwardSpeedMps,0);
  assert.equal(c.throttleCommandPercent,0);
  const after=run(2100);assert.equal(after.status,'recovering');
  assert.ok(after.intermittentThrottleCap>=before.intermittentThrottleCap);
  assert.ok(after.intermittentThrottleCap<=15);
});

test('completed recovery survives one bad frame without restarting its low-speed cap',()=>{
  const c=new UwbFollowController({...calibration,prediction:{enabled:true},execution_mode:'acro',
    manual_propulsion:'single_side',following_distance_m:.5,safety_distance_m:.3,hard_stop_enabled:false,
    max_speed_mps:2,commissioning_speed_mps:2,relative_throttle_limit_percent:70,
    controller_profile:'traditional_fast'});
  const session=new IntermittentUwbFollow({enabled:true},calibration);
  const run=(t,status='trusted')=>session.update({controller:c,now:t,telemetry:frame(t,1.4,status),connected:true});
  for(let t=1000;t<=1500;t+=100)run(t);
  run(1600,'invalid');for(let t=1700;t<=2800;t+=100)run(t);
  assert.equal(session.recoveryGoodMs,800);
  const missing=run(2900,'invalid');assert.equal(missing.forwardSpeedMps,0);assert.equal(c.throttleCommandPercent,0);
  const recovered=run(3000);
  assert.equal(recovered.safe,true);assert.equal(recovered.reason,'tracking');
  assert.equal(recovered.intermittentThrottleCap,undefined);
  assert.ok(recovered.forwardSpeedMps<.1,'outer propulsion state must still restart from zero');
  for(let t=3100;t<=3700;t+=100)run(t,'invalid');
  assert.equal(run(3800).status,'waiting_target');run(3900);const restored=run(4000);
  assert.equal(restored.status,'recovering');assert.equal(restored.intermittentThrottleCap,0);
});


test('far recovery uses five distinct hits, bounded angular allowance and unchanged radial safety',()=>{
 const e=new UwbTargetEstimator({enabled:true},calibration);
 for(let t=1000;t<=1400;t+=100)e.update(t,frame(t,12));
 e.update(1500,frame(1500,12,'invalid'));
 // Expire old window without extending the five-second loss deadline.
 for(let t=1600;t<=2500;t+=100)e.update(t,frame(t,12,'suspect',1400));
 let r;
 for(let i=0;i<7;i++){
  const t=2600+i*100, p=frame(t,12+.01*i);
  p.uwb.sensors[0].rawAzimuthDeg=i%2?1.4:-1.4;
  r=e.update(t,p);
  if(i<4)assert.notEqual(r.source,'recovering');
 }
 assert.ok(r.measurementConfirmed);assert.ok(e.rawWindow.reacquisition.length<=9);
 const fit=e.rawWindow.reacquisitionFit;
 assert.ok(fit.samples>=5);assert.equal(fit.radialToleranceM,.24);
 assert.ok(fit.transverseToleranceM<=.6);assert.equal(e.config.wait_timeout_ms,5000);
 const at=e.last.at,p=frame(3300,18);p.uwb.sensors[0].rawAzimuthDeg=30;
 const bad=e.update(3300,p);assert.equal(bad.lastMeasuredAt,at);
});

function posteriorOffsetTrial(offset,uncertainty,range=8) {
 const e=new UwbTargetEstimator({enabled:true},calibration);
 for(let t=1000;t<=1400;t+=100)e.update(t,frame(t,range));
 e.update(1500,frame(1500,range,'invalid'));
 for(let t=1600;t<=2500;t+=100)e.update(t,frame(t,range,'suspect',1400));
 const results=[];
 for(let i=0;i<5;i++){
  const t=2600+i*100,p=frame(t,range+.01*i-offset);
  p.uwb.sensors[0].rawDistanceM=range+.01*i;
  p.uwb.sensors[0].filterUncertaintyM=uncertainty;
  results.push(update(e,t,p));
 }
 return {e,results};
}
test('posterior smoothing offset needs five raw inliers and preserves uncertainty at handoff',()=>{
 const {e,results}=posteriorOffsetTrial(.4,.8);
 for(const r of results.slice(0,4))assert.equal(r.measurementConfirmed,false);
 const r=results.at(-1);
 assert.equal(r.measurementConfirmed,true);assert.equal(r.source,'recovering');
 assert.equal(r.lastMeasuredAt,3000);assert.equal(r.reacquisitionFit.handoffMode,'bounded_posterior_overlap');
 assert.equal(r.reacquisitionFit.radialToleranceM,.24);
 assert.ok(r.reacquisitionFit.handoffRadialToleranceM<=.6);
 assert.ok(r.measurementUncertaintyM>=.8);assert.equal(r.targetVelocityNedMps.x,0);
 assert.equal(e.config.wait_timeout_ms,5000);
 const lost=e.update(3100,frame(3100,8,'invalid'));
 assert.equal(lost.usable,false,'accepted posterior uncertainty is not erased to permit blind prediction');
});
test('large covariance cannot admit an arbitrarily displaced posterior or refresh the loss deadline',()=>{
 const {e,results}=posteriorOffsetTrial(.8,10);
 assert.equal(results.at(-1).measurementConfirmed,false);
 assert.equal(e.lossAt,1500);assert.equal(e.last.at,1400);
 for(let t=3100;t<=6500;t+=100)e.update(t,frame(t,8,'suspect',1400));
 assert.equal(e.update(6500,frame(6500,8)).reason,'prediction_wait_timeout');
});
test('low-uncertainty contradictory posterior cannot borrow the maximum handoff tolerance',()=>{
 const {results}=posteriorOffsetTrial(.4,.1);
 assert.equal(results.at(-1).measurementConfirmed,false);
});
test('close posterior overlap retains conservative clearance instead of authorizing unsafe propulsion',()=>{
 const {results}=posteriorOffsetTrial(.4,.5,1.1),r=results.at(-1);
 assert.equal(r.measurementConfirmed,true);
 assert.ok(r.measurementUncertaintyM>.7);
 assert.ok(Math.hypot(r.relativeBodyM.x,r.relativeBodyM.y)-r.measurementUncertaintyM<.3);
});

test('delayed trusted posterior can hand off the same raw observation without counting it twice',()=>{
 const e=estimator();
 for(const t of [3100,3200,3300])e.update(t,frame(t,1.4,'suspect'));
 assert.equal(e.rawWindow.reacquisitionDiagnostic.reason,'awaiting_measurement_handoff');
 const size=e.rawWindow.reacquisition.length,lossAt=e.lossAt;
 for(const t of [3310,3320])e.update(t,frame(t,1.4,'suspect',3300));
 assert.equal(e.lossAt,lossAt);assert.equal(e.rawWindow.reacquisition.length,size);
 const recovered=e.update(3340,frame(3340,1.4,'trusted',3300));
 assert.equal(recovered.measurementConfirmed,true);assert.equal(recovered.lastMeasuredAt,3300);
 assert.equal(recovered.reacquisitionFit.handoffRetrySameObservation,true);
 assert.equal(e.rawWindow.reacquisition.length,size);
 assert.equal(e.last.samples,3);
});
test('late duplicate posterior after freshness expiry cannot revive a candidate or extend timeout',()=>{
 const e=estimator();
 for(const t of [3100,3200,3300])e.update(t,frame(t,1.4,'suspect'));
 const lossAt=e.lossAt;
 const r=e.update(3800,frame(3800,1.4,'trusted',3300));
 assert.equal(r.measurementConfirmed,false);assert.equal(e.lossAt,lossAt);
 assert.equal(e.last.at,1400);
});
