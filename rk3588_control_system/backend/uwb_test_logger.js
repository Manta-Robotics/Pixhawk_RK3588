import fs from 'fs';
import path from 'path';

const COLUMNS = [
  'timestamp_iso', 'elapsed_ms', 'event', 'session_id', 'follow_status', 'follow_reason',
  'measurement_status', 'measurement_rejection_reason', 'measurement_nis', 'measurement_filter_uncertainty_m',
  'target_source', 'target_last_measured_at', 'prediction_age_ms', 'prediction_uncertainty_m', 'recovery_samples',
  'control_mode', 'flight_mode', 'armed', 'gps_lat', 'gps_lon', 'gps_alt_m',
  'yaw_deg', 'yaw_rate_deg_s', 'velocity_north_mps', 'velocity_east_mps',
  'estimated_north_m', 'estimated_east_m',
  'uwb_raw_distance_m', 'uwb_distance_m', 'uwb_raw_azimuth_deg', 'uwb_azimuth_deg', 'uwb_elevation_deg',
  'target_relative_x_m', 'target_relative_y_m', 'bearing_deg', 'control_bearing_deg', 'guidance_bearing_deg',
  'cross_track_error_m', 'ilos_integral_ms', 'ilos_correction_deg', 'lookahead_m', 'curvature_m_inv',
  'tracking_phase', 'close_approach_active', 'distance_hold_error_m', 'distance_hold_active',
  'distance_recovery_active', 'desired_forward_mps', 'commanded_forward_mps',
  'throttle_percent', 'steering_deg',
  'desired_yaw_rate_deg_s', 'yaw_rate_error_deg_s', 'requested_throttle_percent', 'requested_steering_deg',
  'turn_headroom_percent', 'throttle_reduced_for_turn', 'left_demand_percent', 'right_demand_percent',
  'trust_checks_enabled', 'trust_diagnostics',
  'command_left_pwm', 'command_right_pwm', 'actual_left_pwm', 'actual_right_pwm', 'output_neutral_verified'
];

function finite(value, fallback = '') {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function csvCell(value) {
  if (value === null || value === undefined) return '';
  const text = String(value);
  return /[",\r\n]/.test(text) ? `"${text.replaceAll('"', '""')}"` : text;
}

function fileTimestamp(now) {
  return new Date(now).toISOString().replace(/[-:]/g, '').replace('T', '_').replace('Z', 'Z');
}

export class UwbTestLogger {
  constructor(directory, { flushRows = 10 } = {}) {
    this.directory = path.resolve(directory);
    this.flushRows = Math.max(1, Math.round(finite(flushRows, 10)));
    this.active = false;
    this.buffer = [];
    this.sequence = 0;
    this.resetEstimate();
  }

  resetEstimate() {
    this.startedAt = 0;
    this.lastAt = 0;
    this.estimatedNorthM = 0;
    this.estimatedEastM = 0;
  }

  start(metadata = {}, now = Date.now()) {
    if (this.active) this.stop({ event: 'replaced', follow_reason: 'new_session_started' }, now);
    fs.mkdirSync(this.directory, { recursive: true });
    this.sequence += 1;
    const suffix = String(this.sequence).padStart(2, '0');
    this.sessionId = `uwb_${fileTimestamp(now)}_${suffix}`;
    this.fileName = `${this.sessionId}.csv`;
    this.filePath = path.join(this.directory, this.fileName);
    this.buffer = [];
    this.resetEstimate();
    this.startedAt = now;
    this.lastAt = now;
    this.active = true;
    fs.writeFileSync(this.filePath, `${COLUMNS.join(',')}\n`, 'utf8');
    this.record({ ...metadata, event: 'start' }, now);
    return { sessionId: this.sessionId, fileName: this.fileName, filePath: this.filePath };
  }

  record(sample = {}, now = Date.now()) {
    if (!this.active) return false;
    const currentAt = finite(now, Date.now());
    const dt = Math.max(0, Math.min(1, (currentAt - this.lastAt) / 1000));
    const yawDeg = finite(sample.yaw_deg, 0);
    const speedMps = finite(sample.commanded_forward_mps, 0);
    const yawRad = yawDeg * Math.PI / 180;
    this.estimatedNorthM += speedMps * Math.cos(yawRad) * dt;
    this.estimatedEastM += speedMps * Math.sin(yawRad) * dt;
    this.lastAt = currentAt;

    const row = {
      ...sample,
      timestamp_iso: new Date(currentAt).toISOString(),
      elapsed_ms: currentAt - this.startedAt,
      event: sample.event || 'sample',
      session_id: this.sessionId,
      estimated_north_m: this.estimatedNorthM,
      estimated_east_m: this.estimatedEastM
    };
    this.buffer.push(`${COLUMNS.map((column) => csvCell(row[column])).join(',')}\n`);
    if (this.buffer.length >= this.flushRows) this.flush();
    return true;
  }

  flush() {
    if (!this.active || this.buffer.length === 0) return;
    const payload = this.buffer.join('');
    this.buffer = [];
    fs.appendFileSync(this.filePath, payload, 'utf8');
  }

  stop(sample = {}, now = Date.now()) {
    if (!this.active) return null;
    this.record({ ...sample, event: sample.event || 'stop' }, now);
    this.flush();
    const result = { sessionId: this.sessionId, fileName: this.fileName, filePath: this.filePath };
    this.active = false;
    this.sessionId = null;
    this.fileName = null;
    this.filePath = null;
    this.buffer = [];
    this.resetEstimate();
    return result;
  }

  state() {
    return {
      active: this.active,
      sessionId: this.sessionId || null,
      fileName: this.fileName || null,
      startedAt: this.startedAt || null,
      estimatedNorthM: this.estimatedNorthM,
      estimatedEastM: this.estimatedEastM
    };
  }
}

export { COLUMNS as UWB_TEST_LOG_COLUMNS };
