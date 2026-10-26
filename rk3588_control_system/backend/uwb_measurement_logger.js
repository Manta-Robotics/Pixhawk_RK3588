import fs from 'node:fs';
import path from 'node:path';
import { randomUUID } from 'node:crypto';

export const MEASUREMENT_COLUMNS = [
  'recordedAt', 'sourceInstance', 'sourceSession', 'recordId', 'sequenceGap', 'outOfOrder',
  'receiverId', 'frameKey', 'source', 'protocol', 'sourceTimeKind', 'sourceTimeBootMs',
  'sampleAt', 'receivedAt', 'finalizedAt', 'filterTimeMs', 'transportDelayMs', 'clockUncertaintyMs', 'clockLeadMs', 'clockOffsetCorrectionMs',
  'headingDeg', 'headingMethod', 'headingSpanMs', 'status', 'reason', 'complete', 'field', 'fieldValue',
  'tagId', 'rawDistanceM', 'rawAzimuthDeg', 'rawElevationDeg', 'rawBearingDeg',
  'filteredDistanceM', 'filteredAzimuthDeg', 'filteredBearingDeg', 'unsmoothedAzimuthDeg',
  'distanceCorrectionM', 'azimuthCorrectionDeg', 'filterNis', 'filterInnovationM',
  'filterUncertaintyM', 'angularFilterMode', 'referenceDistanceM', 'referenceBearingDeg',
  'trustChecksEnabled', 'trustDiagnostics', 'withinReceiverFov',
  'rawDistanceErrorM', 'distanceErrorM', 'rawBearingErrorDeg', 'bearingErrorDeg'
];
const finite = value => typeof value === 'number' && Number.isFinite(value);
const wrap = value => ((value + 180) % 360 + 360) % 360 - 180;
const cell = value => {
  if (value == null || (typeof value === 'number' && !Number.isFinite(value))) return '';
  const text = typeof value === 'object' ? JSON.stringify(value) : String(value);
  return /[",\r\n]/.test(text) ? `"${text.replaceAll('"', '""')}"` : text;
};

// The native process emits immutable records before the latest telemetry can
// overwrite them. Recording never drives/arms the vehicle. Disk backpressure
// has a bounded queue and explicit drop counts instead of blocking reception.
export class UwbMeasurementLogger {
  constructor(directory, { maxPendingBytes = 256 * 1024 } = {}) {
    this.directory = path.resolve(directory);
    this.maxPendingBytes = maxPendingBytes;
    this.active = false;
    this.stream = null;
    this.rows = this.droppedRows = this.sequenceGaps = this.duplicates = 0;
    this.error = null;
    this.fileName = null;
    this.reference = {};
    this.seen = new Set();
    this.highWater = new Map();
    this.lifecycle = Promise.resolve();
  }
  serialize(action) {
    const operation = this.lifecycle.then(action);
    this.lifecycle = operation.catch(() => {});
    return operation;
  }
  start(metadata = {}, now = Date.now()) { return this.serialize(() => this.startSession(metadata, now)); }
  stop() { return this.serialize(() => this.stopSession()); }
  async startSession(metadata = {}, now = Date.now()) {
    for (const key of ['referenceDistanceM', 'referenceBearingDeg']) {
      if (metadata[key] != null && !finite(metadata[key])) throw new Error(`invalid_${key}`);
    }
    if (this.active || this.stream) await this.stopSession();
    fs.mkdirSync(this.directory, { recursive: true });
    this.fileName = `uwb_measurements_${new Date(now).toISOString().replace(/[:.]/g, '-')}_${randomUUID().slice(0, 8)}.csv`;
    this.filePath = path.join(this.directory, this.fileName);
    this.reference = { referenceDistanceM: metadata.referenceDistanceM ?? null,
                       referenceBearingDeg: metadata.referenceBearingDeg ?? null };
    this.rows = this.droppedRows = this.sequenceGaps = this.duplicates = 0;
    this.error = null;
    this.seen.clear(); this.highWater.clear();
    this.startedAt = now;
    fs.writeFileSync(this.filePath + '.json', JSON.stringify({ schemaVersion: 1, startedAt: now,
      sourceTimeKind: 'lua_processing_not_radio_sampling', ...metadata }, null, 2));
    const stream = this.stream = fs.createWriteStream(this.filePath, { flags: 'wx', highWaterMark: 64 * 1024 });
    stream.on('error', error => { this.error = error.message; this.active = false; });
    await new Promise((resolve, reject) => { stream.once('open', resolve); stream.once('error', reject); });
    stream.write(MEASUREMENT_COLUMNS.join(',') + '\n');
    this.active = true;
    return this.state();
  }
  record(measurement, now = Date.now()) {
    if (!this.active || !measurement || !Number.isSafeInteger(measurement.recordId)) return false;
    const instance = String(measurement.sourceInstance ?? 'unknown');
    const key = `${instance}:${measurement.recordId}`;
    if (this.seen.has(key)) { this.duplicates++; return false; }
    this.seen.add(key);
    if (this.seen.size > 512) this.seen.delete(this.seen.values().next().value);
    const previous = this.highWater.get(instance);
    const gap = previous == null ? 0 : Math.max(0, measurement.recordId - previous - 1);
    this.sequenceGaps += gap;
    this.highWater.set(instance, Math.max(previous ?? 0, measurement.recordId));
    if (this.highWater.size > 32) this.highWater.delete(this.highWater.keys().next().value);
    const row = { ...measurement, ...this.reference, recordedAt: now, sequenceGap: gap,
                  outOfOrder: previous != null && measurement.recordId < previous };
    for (const [source, reference, dest, angular] of [
      ['rawDistanceM', 'referenceDistanceM', 'rawDistanceErrorM', false],
      ['filteredDistanceM', 'referenceDistanceM', 'distanceErrorM', false],
      ['rawBearingDeg', 'referenceBearingDeg', 'rawBearingErrorDeg', true],
      ['filteredBearingDeg', 'referenceBearingDeg', 'bearingErrorDeg', true]
    ]) {
      if (finite(row[source]) && finite(row[reference])) {
        const delta = row[source] - row[reference];
        row[dest] = angular ? wrap(delta) : delta;
      }
    }
    const line = MEASUREMENT_COLUMNS.map(column => cell(row[column])).join(',') + '\n';
    if (this.stream.writableLength + Buffer.byteLength(line) > this.maxPendingBytes) {
      this.droppedRows++; return false;
    }
    this.stream.write(line);
    this.rows++;
    return true;
  }
  async stopSession() {
    this.active = false;
    const stream = this.stream;
    this.stream = null;
    if (stream && !stream.destroyed) await new Promise(resolve => {
      stream.once('error', resolve);
      stream.end(resolve);
    });
    const result = this.state();
    if (this.filePath) fs.writeFileSync(this.filePath + '.summary.json', JSON.stringify(result, null, 2));
    return result;
  }
  state() {
    return { active: this.active, fileName: this.fileName, rows: this.rows, droppedRows: this.droppedRows,
      observedSequenceGaps: this.sequenceGaps, duplicates: this.duplicates, error: this.error,
      startedAt: this.startedAt ?? null };
  }
}
