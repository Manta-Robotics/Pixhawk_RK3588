// Shared vessel UWB recovery. Runtime inputs are vessel telemetry only.
// Simulation truth and local-computer state are never estimator inputs.
const finite = value => typeof value === 'number' && Number.isFinite(value);
const fresh = (at, now, timeout) => finite(at) && at > 0 && at <= now && now-at <= timeout;

export class RawWindowRecovery {
  constructor(config) { this.config={recovery_min_samples:3,...config};this.reset(); }
  reset() {
    this.reacquisition=[];this.reacquisitionToken=null;this.reacquisitionAt=null;
    this.reacquisitionFit=null;this.reacquisitionDiagnostic={reason:'idle',hits:0,window:0};
  }
  observe(now, telemetry, estimator, anchor, allowHandoff = true) {
    const c = this.config, uwb = telemetry.uwb || {};
    const diagnostic=(reason,hits=0,extra={})=>{
      this.reacquisitionDiagnostic={reason,hits,window:this.reacquisition.length,...extra};return false;
    };
    // Active receiver IDs describe *accepted* fusion, so they disappear during
    // innovation rejection. Select the one raw observing receiver independently.
    const sensors=(uwb.sensors || []).filter(s=>finite(s.receivedMeasurementAt) || finite(s.rawRejectedAt));
    if (!anchor || !estimator.lastPose || sensors.length!==1) return diagnostic('raw_receiver_or_pose_unavailable');
    const sensor=sensors[0],at=Math.max(sensor.receivedMeasurementAt || 0,sensor.rawRejectedAt || 0);
    if (!fresh(at,now,c.measurement_gap_ms)) return diagnostic('raw_stale');
    // Window age is measured at the newest *observation*, not receipt of this
    // HTTP/control snapshot; transport latency must not delete good old hits.
    const far=finite(sensor.rawDistanceM)&&sensor.rawDistanceM>5;
    this.reacquisition=this.reacquisition.filter(p=>at-p.at<=(far?900:500));
    if (this.reacquisitionAt!==null && at<this.reacquisitionAt) return false;
    const sameObservation=at===this.reacquisitionAt;
    const token=finite(sensor.sequenceId)?`${sensor.receiverId}:${sensor.sequenceId}`:`time:${at}`;
    if (!sameObservation && token===this.reacquisitionToken) return false;
    // A later transport snapshot can contain the accepted KF result for an
    // already collected raw observation. Retry only that handoff; never count
    // it as another sample or replace its original position/time with new data.
    if(sameObservation && (!allowHandoff || uwb.measurementStatus!=='trusted'
      || sensor.measurementStatus!=='trusted' || uwb.measurementUpdatedAt!==at))return false;
    if(!sameObservation){this.reacquisitionAt=at;this.reacquisitionToken=token;}
    const baseValid=at===sensor.receivedMeasurementAt && sensor.valid!==false && finite(sensor.rawDistanceM) && sensor.rawDistanceM>=.15
      && sensor.rawDistanceM<=20 && finite(sensor.rawAzimuthDeg) && Math.abs(sensor.rawAzimuthDeg)<=180;
    const raw={...uwb,online:true,fresh:true,measurementUpdatedAt:at,activeReceiverIds:[sensor.receiverId],
      sensors:[sensor],distanceM:sensor.rawDistanceM,azimuthDeg:sensor.rawAzimuthDeg+(sensor.mountYawDeg || 0)};
    const saved=sameObservation?this.reacquisition.find(p=>p.at===at&&p.position):null;
    const sample=baseValid?(sameObservation?saved:estimator.sample(raw,estimator.lastPose,now)):null;
    // Missing/invalid observations occupy a slot; an isolated miss cannot erase
    // all previous evidence. No invalid value is ever fitted or accepted.
    if(!sameObservation){
      this.reacquisition.push(sample || {at,position:null});
      this.reacquisition=this.reacquisition.slice(far?-9:-5);
    }
    if (!sample) return diagnostic('basic_invalid');
    if (anchor.id!==null && sample.id!==null && anchor.id!==sample.id) {
      this.reacquisition=[];return diagnostic('identity_mismatch');
    }
    const reachable=this.reacquisition.filter(p=>p.position && p.at>=anchor.at-500
      && Math.hypot(p.position.x-anchor.position.x,p.position.y-anchor.position.y)
        <=c.innovation_gate_m+c.max_target_speed_mps*Math.abs(p.at-anchor.at)/1000);
    const minimum=Math.max(c.min_samples,c.recovery_min_samples);
    if (reachable.length<minimum) return diagnostic('insufficient_hits',reachable.length);
    // Angular error creates transverse position noise proportional to range.
    // Keep the radial tolerance unchanged; bound the transverse allowance and
    // require a longer, five-hit window before using it. No uncertainty reset.
    const radial=2*c.measurement_noise_m;
    const transverse=far?Math.max(radial,Math.min(c.innovation_gate_m,sensor.rawDistanceM*Math.sin(2*Math.PI/180))):radial;
    const dx=sample.position.x-estimator.lastPose.position.x,dy=sample.position.y-estimator.lastPose.position.y;
    const length=Math.max(.15,Math.hypot(dx,dy)),ux=dx/length,uy=dy/length;
    function fit(points, allowance=transverse) {
      const ts=points.map(p=>(p.at-at)/1000),meanT=ts.reduce((s,t)=>s+t,0)/points.length;
      const mean={x:0,y:0},velocity={x:0,y:0},position={x:0,y:0};
      const denominator=ts.reduce((s,t)=>s+(t-meanT)**2,0);
      if (denominator<=0) return null;
      for(const axis of ['x','y']) {
        mean[axis]=points.reduce((s,p)=>s+p.position[axis],0)/points.length;
        velocity[axis]=points.reduce((s,p,i)=>s+(ts[i]-meanT)*(p.position[axis]-mean[axis]),0)/denominator;
        position[axis]=mean[axis]-velocity[axis]*meanT;
      }
      const error=p=>{
        const x=p.position.x-position.x-velocity.x*(p.at-at)/1000;
        const y=p.position.y-position.y-velocity.y*(p.at-at)/1000;
        return Math.hypot((x*ux+y*uy)/radial,(-x*uy+y*ux)/allowance);
      };
      return {position,velocity,error,residual:Math.max(...points.map(error))};
    }
    const hypotheses=[];
    for(let i=0;i<reachable.length-2;i++)for(let j=i+1;j<reachable.length-1;j++)for(let k=j+1;k<reachable.length;k++) {
     for(const allowance of far?[radial,transverse]:[radial]){
      const required=allowance>radial?Math.max(5,minimum):minimum;
      const initial=fit([reachable[i],reachable[j],reachable[k]],allowance);
      if(!initial)continue;
      const points=reachable.filter(p=>initial.error(p)<=1);
      if(points.length<required || points.at(-1).at!==at || at-points[0].at<(required-1)*90)continue;
      if(points.some((p,n)=>n && p.at-points[n-1].at>c.measurement_gap_ms))continue;
      const model=fit(points,allowance);
      if(!model || Math.hypot(model.velocity.x,model.velocity.y)>c.max_target_speed_mps
        || model.residual>1)continue;
      hypotheses.push({...model,points,allowance});
     }
    }
    hypotheses.sort((a,b)=>b.points.length-a.points.length || a.residual-b.residual);
    let best=hypotheses[0];
    if(!best)return diagnostic('inconsistent_window');
    let hits=best.points.length;
    if (!allowHandoff) return diagnostic('candidate_ready',hits);
    if (at<=anchor.at) return diagnostic('awaiting_new_measurement',hits);
    // Candidate accumulation is independent; handoff must agree with a fresh,
    // trusted measurement-KF posterior, which has its own raw-window recovery.
    if(uwb.measurementStatus!=='trusted' || sensor.measurementStatus!=='trusted'
      || uwb.measurementUpdatedAt!==at) return diagnostic('awaiting_measurement_handoff',hits);
    // The blind extrapolation uncertainty bound is not a live-measurement
    // admission threshold. Preserve the posterior uncertainty for the control
    // clearance/speed guard instead of clearing or shrinking it on recovery.
    if(!finite(sensor.filterUncertaintyM) || sensor.filterUncertaintyM<0)
      return diagnostic('measurement_handoff_invalid_uncertainty',hits);
    const filtered=estimator.sample(uwb,estimator.lastPose,now);
    // The raw-fit residual bounds raw sensor consistency, not agreement with
    // a smoothed posterior. Check all validated hypotheses before rejecting;
    // a different inlier subset may agree with the current accepted posterior.
    let handoffMode='strict_raw_fit',handoffRadial=radial,handoffTransverse=best.allowance;
    if(filtered){
      const strict=hypotheses.find(h=>h.error({...filtered,at})<=1);
      if(strict)best=strict;
      else {
        const bounded=Math.min(c.innovation_gate_m,sensor.filterUncertaintyM);
        const compatible=hypotheses.find(h=>{
          // Extra tolerance needs five distinct observations spanning >=360 ms.
          if(h.points.length<Math.max(5,minimum)||at-h.points[0].at<360)return false;
          const x=filtered.position.x-h.position.x,y=filtered.position.y-h.position.y;
          return Math.hypot((x*ux+y*uy)/Math.max(radial,bounded),
            (-x*uy+y*ux)/Math.max(h.allowance,bounded))<=1;
        });
        if(compatible){best=compatible;handoffMode='bounded_posterior_overlap';
          handoffRadial=Math.max(radial,bounded);handoffTransverse=Math.max(best.allowance,bounded);}
      }
      hits=best.points.length;
      if(handoffMode==='strict_raw_fit')handoffTransverse=best.allowance;
    }
    const handoffError=filtered?best.error({...filtered,at}):null;
    const difference=filtered?{x:filtered.position.x-best.position.x,y:filtered.position.y-best.position.y}:null;
    const handoffDiagnostic={handoffNormalizedError:handoffError,
      handoffRadialErrorM:difference?difference.x*ux+difference.y*uy:null,
      handoffTransverseErrorM:difference?-difference.x*uy+difference.y*ux:null,
      posteriorUncertaintyM:sensor.filterUncertaintyM,radialToleranceM:radial,transverseToleranceM:best.allowance,
      handoffMode,handoffRetrySameObservation:sameObservation,
      handoffRadialToleranceM:handoffRadial,handoffTransverseToleranceM:handoffTransverse};
    if(!filtered || (handoffMode==='strict_raw_fit'&&handoffError>1))
      return diagnostic('measurement_handoff_disagrees',hits,handoffDiagnostic);
    // Preserve the KF uncertainty and conservatively cover the alternative raw
    // fit. Agreement never erases covariance or manufactures a new timestamp.
    if(handoffMode==='bounded_posterior_overlap')filtered.measurementUncertaintyM=Math.max(
      filtered.measurementUncertaintyM,sensor.filterUncertaintyM,
      Math.hypot(difference.x,difference.y)+Math.hypot(radial,best.allowance));
    const velocity=best.velocity,residual=best.residual;
    // Position and motion confidence are separate: a short raw window confirms
    // position only. Its fitted slope is diagnostic, never a control velocity.
    estimator.accept(filtered,estimator.last,hits);
    // Preserve pose, identity and measurement clocks; replace only the obsolete
    // target trajectory after the independent candidate track is confirmed.
    estimator.lastSeenMeasurementAt = filtered.at;
    estimator.lastSeenToken = filtered.token;
    estimator.recovery = null;
    estimator.lossAt = null;
    estimator.recoveryAnchor = null;
    estimator.recoveredAt = now;
    this.reacquisitionFit = {source:'independent_raw_window',samples:hits,window:this.reacquisition.length,
      normalizedResidual:residual,radialToleranceM:radial,transverseToleranceM:best.allowance,
      finalPosteriorUncertaintyM:sensor.filterUncertaintyM,acceptedUncertaintyM:filtered.measurementUncertaintyM,
      ...handoffDiagnostic,rawFitVelocityNedMps:velocity,
      velocityNedMps:{...estimator.last.velocity},velocityTrusted:estimator.last.velocityTrusted};
    this.reacquisitionDiagnostic={reason:'confirmed',hits,window:this.reacquisition.length};
    // Retain only this timestamp-bounded evidence, not an accepted-time lease.
    return true;
  }

}
