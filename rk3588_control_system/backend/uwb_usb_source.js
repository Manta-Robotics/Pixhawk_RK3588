import { EventEmitter } from 'node:events';
import { spawn } from 'node:child_process';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';
import { performance } from 'node:perf_hooks';
import { createInterface } from 'node:readline';
import { UwbStepMetrics } from './uwb_step_metrics.js';

const Smoother = createRequire(import.meta.url)('./vendor/responsive_smoother.cjs');
const finite = Number.isFinite;
const wrap = angle => ((angle + 180) % 360 + 360) % 360 - 180;
const MAX_POSITION_SPEED_MPS = 1.0;
const SAMPLE_RATE_HZ = 35;
const SAMPLE_PERIOD_MS = 1000/SAMPLE_RATE_HZ;
const statsKeys = ['rxBytes','goodCount','heartbeatCount','checksumErrors','lengthErrors',
  'noiseBytes','unsupportedVersions','coalescedPositions','positionRateHz'];

export class UwbUsbSource extends EventEmitter {
  constructor(config = {}, geometry = {}, clocks = {}) {
    super();
    this.config = config;
    this.speedGateEnabled = config.speed_gate_enabled !== false;
    this.outputSpeedGateEnabled = config.output_speed_gate_enabled !== false;
    this.geometry = geometry;
    this.mono = clocks.monotonic || (() => performance.now());
    this.wall = clocks.wall || (() => Date.now());
    this.captureMono = clocks.captureMonotonic || null;
    this.smoother = new Smoother({periodMs:SAMPLE_PERIOD_MS,maxSpeedCmPerSecond:100});
    this.pendingPosition = null;
    this.processedFrames = 0;
    this.rateLimitedFrames = 0;
    this.expiredPendingFrames = 0;
    this.expiredCaptureFrames = 0;
    this.sampleTimes = [];
    this.computeTimes=[];this.computeAt=this.mono();
    this.computeSummary={samples:0,meanMs:null,p95Ms:null,maxMs:null};
    this.stats = Object.fromEntries(statsKeys.map(key => [key, 0]));
    this.connected = false;
    this.identity = null;
    this.lastSequence = null;
    this.measurementMono = null;
    this.measurementWall = null;
    this.receivedWall = null;
    this.updatedAt = this.wall();
    this.lastWall = this.updatedAt;
    this.streamGeneration = 0;
    this.streamId = `usb-${this.updatedAt}-${process.pid}-0`;
    this.raw = null;
    this.output = null;
    this.hasMeasurement = false;
    this.reason = 'usb_disconnected';
    this.warmupCount = 0;
    this.duplicates = 0;
    this.reconnects = 0;
    this.stepMetrics = new UwbStepMetrics();
    this.speedReference = null;
    this.outputReference = null;
    this.outputSpeedGate = {enabled:this.outputSpeedGateEnabled,thresholdMps:1,acceptedCount:0,rejectedCount:0,lastDecision:null};
    this.speedGate = { enabled:this.speedGateEnabled,thresholdMps:MAX_POSITION_SPEED_MPS, comparison:'previous_accepted_xy',
      acceptedCount:0, rejectedCount:0, lastCandidateSpeedMps:null,
      lastRejectedAt:null, lastRejectedSequenceId:null, lastRejectionReason:null,
      lastDecision:null };
    this.child = null;
    this.stopped = false;
  }

  reset(reason) {
    this.smoother.reset();
    this.output = null;
    this.warmupCount = 0;
    this.hasMeasurement = false;
    this.stepMetrics.clear();
    this.reason = reason;
  }

  publish() {
    this.updatedAt = this.wall();
    this.emit('packet', this.snapshot());
  }

  receive(event) {
    if (event?.type === 'position') {
      if (!this.connected) return;
      if (Number.isInteger(this.config.target_tag_id) && event.tagId !== this.config.target_tag_id) return;
      if(this.captureMono&&finite(event.receivedMonoMs)){
        const age=this.captureMono()-event.receivedMonoMs;
        if(age>Smoother.parameters.resetGapMs||age< -1){this.expiredCaptureFrames++;return;}
      }
      if (this.pendingPosition) this.rateLimitedFrames += 1;
      this.pendingPosition = {event,queuedAt:this.mono()};
      return;
    }
    // Loss notifications bypass sampling and invalidate pending measurements.
    if (event?.type === 'connection' || event?.type === 'heartbeat') this.pendingPosition = null;
    this.accept(event);
  }

  sampleLatest() {
    const pending = this.pendingPosition;
    this.pendingPosition = null;
    if (!pending) return; // Never reuse a frame to invent a fresh coordinate.
    const now = this.mono();
    if (now-pending.queuedAt > this.smoother.parameters.resetGapMs
      || this.captureMono&&finite(pending.event.receivedMonoMs)
        &&this.captureMono()-pending.event.receivedMonoMs>Smoother.parameters.resetGapMs) {
      this.expiredPendingFrames += 1;
      this.expire();
      return;
    }
    this.processedFrames += 1;
    this.sampleTimes.push(now);
    this.sampleTimes = this.sampleTimes.filter(at=>now-at<=2000).slice(-120);
    const began=performance.now();
    this.accept(pending.event);
    this.computeTimes.push(performance.now()-began);
    if(this.computeTimes.length>240)this.computeTimes.shift();
    if(now-this.computeAt>=1000||this.computeSummary.samples===0){
      const sorted=[...this.computeTimes].sort((a,b)=>a-b);
      this.computeSummary={samples:sorted.length,meanMs:sorted.reduce((a,b)=>a+b,0)/sorted.length,
        p95Ms:sorted[Math.max(0,Math.ceil(sorted.length*.95)-1)],maxMs:sorted.at(-1)};
      this.computeAt=now;
    }
  }

  scheduleSamples() {
    // Follow absolute deadlines instead of skipping arrivals or accumulating
    // integer-millisecond timer error.
    this.sampleDeadline = this.mono()+SAMPLE_PERIOD_MS;
    const tick = () => {
      if (this.stopped) return;
      const now = this.mono();
      if (now>=this.sampleDeadline) {
        this.sampleLatest();
        this.sampleDeadline += (Math.floor((now-this.sampleDeadline)/SAMPLE_PERIOD_MS)+1)*SAMPLE_PERIOD_MS;
      }
      this.sampleTimer = setTimeout(tick,Math.max(1,Math.ceil(this.sampleDeadline-this.mono())));
      this.sampleTimer.unref();
    };
    this.sampleTimer = setTimeout(tick,Math.ceil(SAMPLE_PERIOD_MS));
    this.sampleTimer.unref();
  }

  accept(event) {
    if (!event || typeof event !== 'object') return;
    if (typeof event.lastUnknownHeaderHex === 'string') this.lastUnknownHeaderHex = event.lastUnknownHeaderHex.slice(0,192);
    if (event.stats) for (const key of statsKeys) {
      if (finite(event.stats[key]) && event.stats[key] >= 0) this.stats[key] = event.stats[key];
    }
    if (event.type === 'connection') {
      const connected = event.connected === true;
      if (connected && !this.connected) this.reconnects += 1;
      this.connected = connected;
      this.lastSequence = null;
      this.speedReference = null;
      this.reset(connected ? 'warmup' : 'usb_disconnected');
      this.publish();
      return;
    }
    if (event.type === 'heartbeat') {
      this.speedReference = null;
      this.reset('no_radio_position');
      this.publish();
      return;
    }
    if (event.type === 'stats') {
      this.expire();
      this.publish();
      return;
    }
    if (event.type !== 'position' || !this.connected) return;
    if (Number.isInteger(this.config.target_tag_id) && event.tagId !== this.config.target_tag_id) return;
    const mono = this.mono(), now = this.wall();
    const measurementMono=this.captureMono&&finite(event.receivedMonoMs)?event.receivedMonoMs:mono;
    const receivedWall=now-Math.max(0,mono-measurementMono);
    if (now < this.lastWall) {
      this.streamId = `usb-${now}-${process.pid}-${++this.streamGeneration}`;
      this.lastSequence = null;
      this.speedReference = null;
      this.reset('clock_reset');
    }
    this.lastWall = now;
    const validVersion = event.version === 0x0100 || event.version === 0x0102;
    const validAngle = finite(event.azimuthDeg) && (event.version === 0x0102
      ? event.azimuthDeg >= 0 && event.azimuthDeg <= 359 : Math.abs(event.azimuthDeg) <= 180);
    const minimum = this.geometry.minimum_distance_m ?? 0.15;
    const maximum = this.geometry.maximum_distance_m ?? 20;
    if (!validVersion || !validAngle || !finite(event.distanceCm)
      || event.distanceCm < 0
      || !finite(event.elevationDeg) || Math.abs(event.elevationDeg) > 90
      || !Number.isInteger(event.sequenceId) || event.sequenceId < 0 || event.sequenceId > 65535
      || !Number.isInteger(event.anchorId) || !Number.isInteger(event.tagId)) {
      this.reset('invalid_measurement');
      this.publish();
      return;
    }
    const identity = `${event.anchorId}:${event.tagId}`;
    if (identity !== this.identity) {
      this.identity = identity;
      this.lastSequence = null;
      this.speedReference = null;
      this.outputReference = null;
      this.reset('warmup');
    }
    if (event.sequenceId === this.lastSequence) {
      this.duplicates += 1;
      return;
    }
    if (this.lastSequence !== null && ((event.sequenceId-this.lastSequence+65536)%65536) > 32768) {
      this.speedReference = null;
      this.reset('sequence_reset');
    }
    this.lastSequence = event.sequenceId;
    if (event.distanceCm < minimum*100 || event.distanceCm > maximum*100) {
      this.reset('distance_out_of_range');
      this.measurementMono = measurementMono;
      this.receivedWall = receivedWall;
      this.raw = { ...event, azimuthDeg:wrap(event.azimuthDeg), protocolAzimuthDeg:event.azimuthDeg };
      this.hasMeasurement = true; // Keep complete raw diagnostics, never admit this target.
      this.rawRejectedAt = now;
      this.publish();
      return;
    }
    const azimuth = wrap(event.azimuthDeg), radians = azimuth*Math.PI/180;
    const candidate = { x:event.distanceCm*Math.cos(radians), y:event.distanceCm*Math.sin(radians),
      at:finite(event.receivedMonoMs)?event.receivedMonoMs:mono, sequenceId:event.sequenceId };
    const previous = this.speedReference;
    const dtMs = previous?candidate.at-previous.at:null;
    const displacementCm = previous?Math.hypot(candidate.x-previous.x,candidate.y-previous.y):null;
    const speedMps = previous&&dtMs>0?displacementCm*10/dtMs:null;
    const rejected = Boolean(previous&&(dtMs<=0 || this.speedGateEnabled&&displacementCm>MAX_POSITION_SPEED_MPS*dtMs/10+1e-9));
    this.speedGate.lastCandidateSpeedMps = speedMps;
    // Bounded diagnostics only: rejected coordinates never replace the public
    // accepted observation, smoothing input, or tracking target.
    this.speedGate.lastDecision = { sequenceId:event.sequenceId, receivedAt:now,
      receivedMonoMs:candidate.at, referenceSequenceId:previous?.sequenceId??null,
      referenceMonoMs:previous?.at??null, distanceM:event.distanceCm/100,
      azimuthDeg:azimuth, elevationDeg:event.elevationDeg, xM:candidate.x/100, yM:candidate.y/100,
      deltaMs:dtMs, displacementM:previous?displacementCm/100:null, speedMps,
      accepted:!rejected, reason:rejected?(dtMs<=0?'non_positive_time_delta':'speed_outlier'):
        previous?'accepted':'initial_reference' };
    if (previous) {
      // Reject before changing the accepted point, timestamps or smoother. A
      // rejected point never becomes the reference for the next observation.
      if (rejected) {
        this.speedGate.rejectedCount += 1;
        this.speedGate.lastRejectedAt = now;
        this.speedGate.lastRejectedSequenceId = event.sequenceId;
        this.speedGate.lastRejectionReason = dtMs<=0?'non_positive_time_delta':'speed_outlier';
        this.expire();
        this.publish();
        return;
      }
    }
    this.speedReference = candidate;
    this.speedGate.acceptedCount += 1;
    if (this.measurementMono !== null && mono-this.measurementMono > Smoother.parameters.resetGapMs
      || finite(event.gapMs) && event.gapMs > Smoother.parameters.resetGapMs) this.reset('warmup');
    this.measurementMono = measurementMono;
    this.receivedWall = receivedWall;
    this.raw = { ...event, azimuthDeg:azimuth, protocolAzimuthDeg:event.azimuthDeg };
    this.hasMeasurement = true;
    this.rawRejectedAt = null;
    this.warmupCount += 1;
    const result = this.smoother.update(event.distanceCm, this.raw.azimuthDeg, mono);
    if (!result.ready) {
      this.output = null;
      this.reason = result.reason;
    } else {
      const previousOutput=this.outputReference;
      const elapsedMs=previousOutput?mono-previousOutput.at:null;
      const stepCm=previousOutput?Math.hypot(result.smoothedXcm-previousOutput.x,
        result.smoothedYcm-previousOutput.y):null;
      const speedMps=previousOutput&&elapsedMs>0?stepCm*10/elapsedMs:null;
      const rejected=Boolean(previousOutput&&(elapsedMs<=0||this.outputSpeedGateEnabled&&stepCm>elapsedMs/10+1e-9));
      this.outputSpeedGate.lastDecision={at:now,monotonicMs:mono,sequenceId:event.sequenceId,
        deltaMs:elapsedMs,displacementM:stepCm===null?null:stepCm/100,speedMps,accepted:!rejected};
      if (rejected) {
        this.outputSpeedGate.rejectedCount += 1;
        this.reason='smoothed_speed_outlier';
        // Keep both the last output coordinate and its original timestamp.
      } else {
        this.outputSpeedGate.acceptedCount += 1;
        this.output=result;
        this.outputReference={x:result.smoothedXcm,y:result.smoothedYcm,at:mono};
        this.measurementWall=now;
        this.reason=null;
      }
    }
    this.stepMetrics.append({ mono, rawX: event.distanceCm*Math.cos(this.raw.azimuthDeg*Math.PI/180),
      rawY: event.distanceCm*Math.sin(this.raw.azimuthDeg*Math.PI/180),
      x: this.output?.smoothedXcm, y: this.output?.smoothedYcm });
    this.publish();
  }

  expire() {
    if (this.output && (this.mono()-this.measurementMono > Smoother.parameters.resetGapMs
      || this.mono()-this.outputReference.at > Smoother.parameters.resetGapMs)) {
      this.reset('stale_measurement');
      this.publish();
    }
  }

  snapshot() {
    const now = this.wall(), raw = this.raw;
    const age = this.measurementMono === null ? null : Math.max(0,this.mono()-this.measurementMono);
    const outputAge = this.outputReference === null ? null : Math.max(0,this.mono()-this.outputReference.at);
    const fresh = this.connected && Boolean(this.output) && age <= Smoother.parameters.resetGapMs
      && outputAge !== null && outputAge <= Smoother.parameters.resetGapMs;
    const rawFresh = this.connected && this.hasMeasurement && raw !== null && age !== null && age <= Smoother.parameters.resetGapMs
      && !['no_radio_position','invalid_measurement','usb_disconnected'].includes(this.reason);
    const distanceM = fresh ? this.output.smoothedDistanceCm/100 : null;
    const azimuthDeg = fresh ? wrap(this.output.smoothedAzimuthDeg) : null;
    const receiverId = this.config.receiver_id ?? 1;
    const mount = this.geometry.mount_yaw_deg?.[receiverId-1] || 0;
    const positionX = this.geometry.position_x_m?.[receiverId-1] || 0;
    const positionY = this.geometry.position_y_m?.[receiverId-1] || 0;
    const angle = fresh ? (azimuthDeg+mount)*Math.PI/180 : null;
    const rawObservation = raw ? { at:this.receivedWall, distanceM:raw.distanceCm/100,
      azimuthDeg:raw.azimuthDeg, elevationDeg:raw.elevationDeg, tagId:raw.tagId } : null;
    const sensor = { receiverId, anchorId:raw?.anchorId ?? null, tagId:raw?.tagId ?? null,
      sequenceId:raw?.sequenceId ?? null, tagStatus:raw?.tagStatus ?? null,
      valid:rawFresh, fresh:rawFresh, rawObservation,
      receivedMeasurementAt:this.receivedWall, measurementUpdatedAt:this.measurementWall,
      rawRejectedAt:this.rawRejectedAt ?? null,
      updatedAt:this.updatedAt, rawDistanceM:raw?.distanceCm/100 || null,
      rawAzimuthDeg:raw?.azimuthDeg ?? null, rawElevationDeg:raw?.elevationDeg ?? null,
      protocolAzimuthDeg:raw?.protocolAzimuthDeg ?? null, mountYawDeg:mount,
      positionXM:positionX, positionYM:positionY, distanceM, azimuthDeg,
      elevationDeg:raw?.elevationDeg ?? null, measurementStatus:fresh?'trusted':'unavailable',
      invalidReason:this.reason, ...this.stats };
    const steps=this.stepMetrics.snapshot();
    return { source:'USB_SERIAL', streamId:this.streamId, online:fresh, fresh,
      distanceM, azimuthDeg, elevationDeg:fresh?raw.elevationDeg:null,
      rawDistanceM:sensor.rawDistanceM, rawAzimuthDeg:sensor.rawAzimuthDeg,
      rawElevationDeg:sensor.rawElevationDeg,
      relativeXM:fresh?positionX+distanceM*Math.cos(angle):null,
      relativeYM:fresh?positionY+distanceM*Math.sin(angle):null,
      updatedAt:this.updatedAt, measurementUpdatedAt:this.measurementWall,
      goodCount:this.stats.goodCount, receiverCount:fresh?1:0,
      observingReceiverCount:rawFresh?1:0, activeReceiverIds:rawFresh?[receiverId]:[],
      fusionMode:fresh?'single_usb_responsive':'no_valid_receiver',
      measurementStatus:fresh?'trusted':'unavailable', rejectionReason:this.reason,
      sensors:raw?[sensor]:[], protocolVersion:raw?`0x${raw.version.toString(16).padStart(4,'0')}`:null,
      filter:'responsive_smoother',
      speedGate:{...this.speedGate},
      smoothing:{ name:'responsive',ready:fresh,parameters:{...this.smoother.parameters},
        warmupCount:this.warmupCount, rawStepRmsCm:steps.rawStepRmsCm,
        smoothedStepRmsCm:steps.smoothedStepRmsCm,
        sampleAgeMs:outputAge,inputSampleAgeMs:age,outputMonotonicMs:this.outputReference?.at??null,
        outputSpeedGate:{...this.outputSpeedGate},input:'one_complete_position_frame', elevationSmoothed:false },
      transport:{kind:'usb_serial',connected:this.connected,port:this.config.port,
        targetSampleRateHz:SAMPLE_RATE_HZ,
        processedRateHz:this.sampleTimes.length>1&&this.mono()-this.sampleTimes.at(-1)<1000
          ?(this.sampleTimes.length-1)*1000/(this.sampleTimes.at(-1)-this.sampleTimes[0]):0,
        processedFrames:this.processedFrames,rateLimitedFrames:this.rateLimitedFrames,
        expiredPendingFrames:this.expiredPendingFrames,
        expiredCaptureFrames:this.expiredCaptureFrames,compute:{...this.computeSummary},
        lastUnknownHeaderHex:this.lastUnknownHeaderHex ?? null,
        baud:this.config.baud_rate ?? 115200,reconnects:this.reconnects,
        duplicateFrames:this.duplicates,...this.stats} };
  }

  start() {
    if (this.stopped || this.child) return;
    const reader = fileURLToPath(new URL('./uwb_usb_reader.py',import.meta.url));
    this.child = spawn(this.config.python || '/usr/bin/python3', ['-u',reader,'--port',this.config.port,
      '--baud',String(this.config.baud_rate ?? 115200)], {stdio:['ignore','pipe','pipe']});
    const lines = createInterface({input:this.child.stdout});
    lines.on('line',line => {
      if (line.length > 16384) return;
      try {this.receive(JSON.parse(line));} catch(error) {this.emit('diagnostic',error.message);}
    });
    this.child.stderr.on('data', data => this.emit('diagnostic',String(data).slice(0,1000)));
    this.child.on('error', error => this.emit('diagnostic',error.message));
    this.child.on('close', () => {
      lines.close(); this.child=null;
      this.receive({type:'connection',connected:false});
      if (!this.stopped) {this.restartTimer=setTimeout(()=>this.start(),1000);this.restartTimer.unref();}
    });
    if (!this.expireTimer) {this.expireTimer=setInterval(()=>this.expire(),50);this.expireTimer.unref();}
    if (!this.sampleTimer) this.scheduleSamples();
  }

  close() {
    this.stopped=true;
    clearInterval(this.expireTimer); clearTimeout(this.restartTimer); clearTimeout(this.sampleTimer);
    this.pendingPosition = null;
    this.child?.kill('SIGTERM');
  }
}
