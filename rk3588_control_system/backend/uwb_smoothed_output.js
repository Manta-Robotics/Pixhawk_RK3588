const finite = Number.isFinite;
const cloneKeys = (value, keys) => Object.fromEntries(keys.filter(k => value?.[k] !== undefined).map(k => [k,value[k]]));
const transportKeys=['kind','connected','port','baud','targetSampleRateHz','processedRateHz','processedFrames','rateLimitedFrames','expiredPendingFrames','expiredCaptureFrames','compute','reconnects','duplicateFrames','rxBytes','goodCount','heartbeatCount','checksumErrors','lengthErrors','noiseBytes','unsupportedVersions','coalescedPositions','positionRateHz','processingMode','workerState','workerThreadId','workerGeneration','workerRestarts','workerMailboxCoalesced','deliveryAgeMs','serialParserProcessId'];
const smoothingKeys=['name','parameters','warmupCount','smoothedStepRmsCm','sampleAgeMs','inputSampleAgeMs','outputMonotonicMs','input'];
const gateKeys=['enabled','thresholdMps','comparison','acceptedCount','rejectedCount','lastRejectedAt','lastRejectedSequenceId','lastRejectionReason'];

// Public position contract. Incoming serial observations are only filter inputs;
// no diagnostic candidate can replace the admitted smoothed position here.
export class SmoothedUwbOutput {
  constructor({elevationVerified=false,elevationSign=1,elevationOffsetDeg=0,elevationTauMs=100}={}) {
    this.elevationVerified=elevationVerified;this.elevationSign=elevationSign;
    this.elevationOffsetDeg=elevationOffsetDeg;this.elevationTauMs=elevationTauMs;
    this.pitch=null;this.pitchAt=null;this.stream=null;
  }
  view(packet={},now=Date.now()) {
    const sm=packet.smoothing||{};
    const at=packet.measurementUpdatedAt;
    const ready=packet.online===true&&packet.fresh===true&&packet.measurementStatus==='trusted'
      &&sm.ready===true&&finite(at)&&at<=now&&now-at<=500
      &&finite(packet.relativeXM)&&finite(packet.relativeYM)&&finite(packet.distanceM)&&finite(packet.azimuthDeg);
    const identity=JSON.stringify([packet.streamId,packet.activeReceiverIds,(packet.sensors||[]).map(s=>s.tagId)]);
    if(identity!==this.stream){this.stream=identity;this.pitch=null;this.pitchAt=null;}
    if(ready&&this.elevationVerified&&finite(packet.elevationDeg)&&Math.abs(packet.elevationDeg)<=90
      &&(this.pitchAt===null||at>this.pitchAt)) {
      const value=Math.max(-90,Math.min(90,packet.elevationDeg*this.elevationSign+this.elevationOffsetDeg));
      const dt=this.pitchAt===null?Infinity:at-this.pitchAt;
      this.pitch=this.pitch===null||dt>500?value:this.pitch+(1-Math.exp(-dt/this.elevationTauMs))*(value-this.pitch);
      this.pitchAt=at;
    }
    const elevationReady=ready&&this.elevationVerified&&this.pitchAt===at&&finite(this.pitch);
    const out={...cloneKeys(packet,['source','streamId','protocolVersion','updatedAt','measurementUpdatedAt','goodCount','fusionMode','bearingDisagreementDeg','positionDisagreementM','measurementNis']),
      coordinateSource:'smoothed',controlSource:'smoothed',online:ready,fresh:ready,
      distanceM:ready?packet.distanceM:null,azimuthDeg:ready?packet.azimuthDeg:null,
      relativeXM:ready?packet.relativeXM:null,relativeYM:ready?packet.relativeYM:null,
      elevationDeg:elevationReady?this.pitch:null,elevationValid:elevationReady,
      elevationStatus:!this.elevationVerified?'unverified_protocol_field':elevationReady?'smoothed':'unavailable',
      measurementStatus:ready?'trusted':'unavailable',rejectionReason:ready?null:(packet.rejectionReason||'smoothed_position_unavailable'),
      receiverCount:ready?(packet.receiverCount||0):0,observingReceiverCount:ready?(packet.observingReceiverCount||0):0,
      activeReceiverIds:ready?[...(packet.activeReceiverIds||[])]:[],
      sensors:(packet.sensors||[]).map(s=>({...cloneKeys(s,['receiverId','anchorId','tagId','sequenceId','tagStatus','rxBytes','goodCount','heartbeatCount','checksumErrors','lengthErrors','noiseBytes','unsupportedVersions','positionRateHz']),valid:ready&&s.valid!==false,fresh:ready&&s.fresh!==false,measurementStatus:ready?'trusted':'unavailable'})),
      smoothing:{...cloneKeys(sm,smoothingKeys),ready,elevationSmoothed:elevationReady},
      transport:cloneKeys(packet.transport,transportKeys),speedGate:cloneKeys(packet.speedGate,gateKeys)};
    return out;
  }
}
