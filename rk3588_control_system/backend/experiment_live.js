import fs from 'node:fs';
import { randomUUID } from 'node:crypto';

export const schema = JSON.parse(
  fs.readFileSync(new URL('../config/experiment_schema.json', import.meta.url))
);
const ageMs = (now, then) => Number(now - BigInt(then)) / 1e6;
const wrap = (angle) => ((((angle + 180) % 360) + 360) % 360) - 180;
const validNs = (value) => typeof value === 'string' && /^\d{1,24}$/.test(value);
const moduleSources = {
  fc: ['fc', 'att', 'nav'],
  mode: ['mode'],
  imu: ['imu'],
  uwb: ['uwb', 'uwb_pred'],
  gimbal: ['gimbal', 'gimbal_cmd'],
  vision: ['vision'],
  motors: ['pwm', 'motor_cmd', 'esc'],
  video: ['video'],
  logger: []
};
const moduleFields = {
  fc: 'fc_mode armed heading_deg yaw_rate_deg_s ground_speed_m_s',
  mode: 'control_owner follow_active follow_side follow_method follow_control_mode gimbal_mode',
  imu: 'ax_m_s2 ay_m_s2 az_m_s2 acc_norm_m_s2 gx_rad_s gy_rad_s gz_rad_s',
  uwb: 'uwb_range_m uwb_bearing_deg uwb_elevation_deg uwb_quality uwb_pred_range_m uwb_pred_bearing_deg bearing_error_deg',
  gimbal:
    'gimbal_yaw_deg gimbal_pitch_deg gimbal_cmd_yaw_deg gimbal_cmd_pitch_deg gimbal_cmd_yaw_rate_deg_s gimbal_cmd_pitch_rate_deg_s',
  vision: 'vision_state target_count detected target_id confidence cx_norm cy_norm',
  motors:
    'left_pwm_us right_pwm_us cmd_left_pwm_us cmd_right_pwm_us left_erpm right_erpm left_rpm right_rpm',
  video: 'preview_active record_phase record_elapsed_ms clip_no',
  logger: 'writer_state last_flush_age_ms drop_total'
};
const labels = {
  fc_mode: '飞控模式',
  armed: '解锁',
  heading_deg: '船首航向',
  yaw_rate_deg_s: '右转角速度',
  ground_speed_m_s: '地速',
  control_owner: '船控来源',
  follow_active: '跟随启动',
  follow_side: '跟随位置',
  follow_method: '跟随方法',
  follow_control_mode: '控制模式',
  gimbal_mode: '云台模式',
  ax_m_s2: '前向加速度',
  ay_m_s2: '右向加速度',
  az_m_s2: '下向加速度',
  acc_norm_m_s2: '加速度模',
  gx_rad_s: 'X 角速率',
  gy_rad_s: 'Y 角速率',
  gz_rad_s: 'Z 角速率',
  uwb_range_m: '实测距离',
  uwb_bearing_deg: '实测方位',
  uwb_elevation_deg: '仰角',
  uwb_quality: '定位质量',
  uwb_pred_range_m: '预测距离',
  uwb_pred_bearing_deg: '预测方位',
  bearing_error_deg: '目标相对镜头',
  gimbal_yaw_deg: '云台水平角',
  gimbal_pitch_deg: '云台俯仰角',
  gimbal_cmd_yaw_deg: '指令水平角',
  gimbal_cmd_pitch_deg: '指令俯仰角',
  gimbal_cmd_yaw_rate_deg_s: '指令水平角速率',
  gimbal_cmd_pitch_rate_deg_s: '指令俯仰角速率',
  vision_state: '本帧识别',
  target_count: '目标数',
  detected: '实测检出',
  target_id: '选中目标',
  confidence: '置信度',
  cx_norm: '中心 X',
  cy_norm: '中心 Y',
  left_pwm_us: '飞控输出左',
  right_pwm_us: '飞控输出右',
  cmd_left_pwm_us: '指令左',
  cmd_right_pwm_us: '指令右',
  left_erpm: '左电转速',
  right_erpm: '右电转速',
  left_rpm: '左机械转速',
  right_rpm: '右机械转速',
  preview_active: '预览',
  record_phase: '录像',
  record_elapsed_ms: '录制时长',
  clip_no: '录像段',
  writer_state: '写入状态',
  last_flush_age_ms: '距成功写盘',
  drop_total: '累计丢弃'
};
const unitFor = (k) =>
  Object.entries({
    _deg_s: '°/s',
    _deg: '°',
    _m_s2: 'm/s²',
    _m_s: 'm/s',
    _rad_s: 'rad/s',
    _pwm_us: 'µs',
    _erpm: 'eRPM',
    _rpm: 'RPM',
    _m: 'm',
    _ms: 'ms'
  }).find(([end]) => k.endsWith(end))?.[1] ?? null;
const staleRules = {
  fc: ['fc', 'FC_LINK_DOWN', '飞控心跳'],
  att: ['fc', 'ATTITUDE_STALE', '船体姿态'],
  imu: ['imu', 'IMU_STALE', 'IMU'],
  uwb: ['uwb', 'UWB_STALE', 'UWB 实测'],
  gimbal: ['gimbal', 'GIMBAL_STALE', '云台姿态'],
  pwm: ['motors', 'MOTOR_PWM_STALE', '飞控 PWM'],
  video: ['video', 'VIDEO_STALE', '视频入口']
};
const sourceOf = {
  heading_deg: 'att',
  yaw_rate_deg_s: 'att',
  ground_speed_m_s: 'nav',
  fc_mode: 'fc',
  armed: 'fc'
};
const ns = () => process.hrtime.bigint();
const bootId = () => {
  try {
    return fs.readFileSync('/proc/sys/kernel/random/boot_id', 'utf8').trim();
  } catch {
    return randomUUID();
  }
};

export class ExperimentLive {
  constructor({
    clock = ns,
    boot = bootId(),
    diagnostic = {},
    publish = () => {},
    journal = () => {},
    utc = () => new Date().toISOString()
  } = {}) {
    this.clock = clock;
    this.boot = boot;
    this.epoch = randomUUID();
    this.utc = utc;
    this.publish = publish;
    this.write = journal;
    this.d = {
      ...schema.diagnostic,
      ...Object.fromEntries(Object.entries(diagnostic).filter(([, v]) => Number.isFinite(v) && v > 0))
    };
    this.samples = new Map();
    this.seen = new Map();
    this.alerts = new Map();
    this.enabled = new Map();
    this.streak = new Map();
    this.pending = new Map();
    this.mode = {};
    this.record = { phase: 'idle' };
    this.writer = {};
    this.worker = {};
    this.seq = 0;
    this.row = 0;
    this.session = null;
    this.linkDown = false;
  }
  setExpected(source, enabled) {
    if (enabled && !this.enabled.has(source)) {
      this.enabled.set(source, this.clock());
      this.streak.set(source, 0);
    }
    if (!enabled && this.enabled.delete(source)) {
      for (const a of [...this.alerts.values()])
        if (staleRules[source]?.[1] === a.code || (source === 'vision' && a.module === 'vision'))
          this.closeAlert(a.code, 'disabled');
    }
  }
  enter(id, start, lastRow) {
    if (this.session === id) {
      this.row = Math.max(this.row, (lastRow ?? -1) + 1);
      return;
    }
    this.leave();
    this.session = id;
    this.start = BigInt(start);
    this.row = (lastRow ?? -1) + 1;
    this.lastRowAt = null;
    this.writer = { state: 'starting', at: this.clock(), drop_total: 0 };
    // Capture retained source identities at the boundary so the first row is reproducible offline.
    for (const sample of this.samples.values()) this.write('sample', sample);
    this.check();
  }
  leave() {
    for (const alert of [...this.alerts.values()]) this.closeAlert(alert.code, 'session_end');
    this.session = null;
    this.writer = {};
    this.lastRowAt = null;
  }
  event(module, type, extra = {}, detail = {}) {
    const now = this.clock();
    const payload = {
      session_id: this.session,
      event_id: randomUUID(),
      mono_ns: String(now),
      t_ms: this.session ? ageMs(now, this.start) : null,
      utc: this.utc(),
      module,
      type,
      level: 'info',
      code: null,
      alert_id: null,
      request_id: null,
      sample_id: null,
      recording_id: null,
      clip_id: null,
      frame_id: null,
      message: type,
      detail_json: detail,
      ...extra
    };
    this.write('event', payload);
    return payload;
  }
  alert(
    module,
    code,
    level,
    message,
    { observed = null, threshold = null, unit = null, sample_id = null, request_id = null, extra = {} } = {}
  ) {
    const old = this.alerts.get(code),
      now = String(this.clock());
    const a = {
      alert_id: old?.alert_id ?? randomUUID(),
      module,
      code,
      level,
      first_ns: old?.first_ns ?? now,
      changed_ns: old?.level === level ? old.changed_ns : now,
      message,
      observed,
      threshold,
      unit,
      sample_id
    };
    this.alerts.set(code, a);
    if (old?.level !== level) {
      this.event(
        module,
        old ? 'alert_level' : 'alert_open',
        { level, code, message, alert_id: a.alert_id, sample_id, request_id },
        { first_ns: a.first_ns, observed, threshold, unit, end_reason: null, ...extra }
      );
      this.publish(this.snapshot());
    }
  }
  closeAlert(code, reason = 'recovered') {
    const a = this.alerts.get(code);
    if (!a) return;
    this.alerts.delete(code);
    this.event(
      a.module,
      'alert_close',
      { level: a.level, code, alert_id: a.alert_id, sample_id: a.sample_id, message: a.message },
      {
        first_ns: a.first_ns,
        observed: a.observed,
        threshold: a.threshold,
        unit: a.unit,
        end_reason: reason,
        duration_ms: ageMs(this.clock(), a.first_ns)
      }
    );
    this.publish(this.snapshot());
  }
  ingest(s) {
    if (
      !s ||
      !schema.sources.includes(s.source) ||
      typeof s.sample_id !== 'string' ||
      !s.sample_id ||
      !s.values ||
      typeof s.values !== 'object'
    )
      return false;
    const now = this.clock(),
      avail = String(now),
      foreign = s.boot_id && s.boot_id !== this.boot;
    const sample = {
      ...s,
      boot_id: s.boot_id ?? this.boot,
      avail_ns: avail,
      obs_ns: validNs(s.obs_ns) ? s.obs_ns : null,
      meta: s.meta ?? {}
    };
    const key = sample.source + '/' + sample.sample_id;
    if (this.seen.has(key)) return false;
    this.seen.set(key, true);
    if (this.seen.size > 65536) this.seen.delete(this.seen.keys().next().value);
    this.write('sample', sample);
    if (
      foreign ||
      sample.obs_ns === null ||
      BigInt(sample.obs_ns) > now ||
      sample.quality === 'invalid' ||
      sample.meta.primary === false
    )
      return false;
    if (
      !['measured', 'predicted'].includes(sample.quality) ||
      (sample.quality === 'predicted' &&
        (!sample.meta.based_on_sample_id || !['uwb_pred', 'vision'].includes(sample.source)))
    )
      return false;
    const old = this.samples.get(sample.source);
    if (old && BigInt(old.obs_ns) > BigInt(sample.obs_ns)) return false;
    if (sample.source === 'vision' && sample.values.vision_state === 'error') return false;
    this.samples.set(sample.source, sample);
    if (sample.quality === 'measured' && ageMs(now, sample.obs_ns) <= (this.d[sample.source] ?? Infinity)) {
      const count = (this.streak.get(sample.source) ?? 0) + 1;
      this.streak.set(sample.source, count);
      if (count >= 2 && staleRules[sample.source]) this.closeAlert(staleRules[sample.source][1]);
    }
    return true;
  }
  local(source, values, meta = {}, options = {}) {
    return this.ingest({
      source,
      sample_id: randomUUID(),
      obs_ns: String(this.clock()),
      time_kind: 'host_receive',
      quality: 'measured',
      values,
      meta,
      ...options
    });
  }
  check() {
    const now = this.clock();
    for (const [source, [module, code, label]] of Object.entries(staleRules)) {
      if (!this.enabled.has(source)) continue;
      const s = this.samples.get(source),
        age = ageMs(now, s?.obs_ns ?? this.enabled.get(source));
      const T = this.d[source],
        down = source === 'fc' && this.linkDown;
      if (!down && ((!s && age < this.d.initial_wait) || age <= T)) continue;
      this.streak.set(source, 0);
      const error = down || age > 3 * T || this.alerts.get(code)?.level === 'error';
      this.alert(module, code, error ? 'error' : 'warn', down ? '飞控链路断开' : label + '更新超时', {
        observed: age,
        threshold: error ? 3 * T : T,
        unit: 'ms',
        sample_id: s?.sample_id
      });
    }
    if (this.enabled.has('vision')) {
      const age = ageMs(now, this.worker.lastSuccess ?? this.enabled.get('vision'));
      if (this.worker.failed || age > this.d.vision)
        this.alert('vision', 'VISION_FAILED', 'error', this.worker.failed || '视觉推理超时', {
          observed: age,
          threshold: this.d.vision,
          unit: 'ms'
        });
      if (this.worker.lastLock && ageMs(now, this.worker.lastLock) > 1000)
        this.alert('vision', 'TARGET_LOST', 'warn', '已锁定目标持续一秒未实测', {
          observed: ageMs(now, this.worker.lastLock),
          threshold: 1000,
          unit: 'ms'
        });
    }
    if (
      (this.session || this.writer.state === 'starting') &&
      this.writer.at != null &&
      ageMs(now, this.writer.at) > this.d.logger_heartbeat
    )
      this.alert('logger', 'LOGGER_OFFLINE', 'error', '记录器心跳超时', {
        observed: ageMs(now, this.writer.at),
        threshold: this.d.logger_heartbeat,
        unit: 'ms'
      });
    if (this.writer.lastDrop && ageMs(now, this.writer.lastDrop) >= this.d.drop_quiet)
      this.closeAlert('LOG_DROPPED');
    if (
      this.record.phase === 'starting' &&
      this.recordStart &&
      ageMs(now, this.recordStart) > this.d.record_start
    )
      this.alert('video', 'RECORD_FAILED', 'error', '录像启动后未写入有效画面', {
        observed: ageMs(now, this.recordStart),
        threshold: this.d.record_start,
        unit: 'ms'
      });
    for (const [axis, p] of this.pending)
      if (p.state === 'waiting' && ageMs(now, p.at) > this.d.mode_timeout)
        this.modeOutcome(axis, p.id, 'timeout', '等待实际模式确认超时');
  }
  status(s) {
    const now = this.clock(),
      prev = this.writer;
    this.writer = { ...prev, ...s, at: now };
    this.closeAlert('LOGGER_OFFLINE');
    if (s.state === 'failed' || s.error)
      this.alert('logger', 'LOG_WRITE_FAILED', 'error', s.error?.message || '日志写盘失败');
    else if (
      validNs(s.last_flush_ns) &&
      (!prev.last_flush_ns || BigInt(s.last_flush_ns) > BigInt(prev.last_flush_ns))
    )
      this.closeAlert('LOG_WRITE_FAILED');
    if (s.drop_total > (prev.drop_total ?? 0)) {
      this.writer.lastDrop = now;
      this.alert('logger', 'LOG_DROPPED', 'warn', '实验记录存在缺口', {
        observed: s.drop_total,
        unit: '条',
        extra: { delta: s.drop_total - (prev.drop_total ?? 0), total: s.drop_total }
      });
    }
  }
  values(now = this.clock()) {
    const row = {};
    const expired = new Set();
    const sources = {};
    for (const [key, s] of this.samples) {
      const age = ageMs(now, s.obs_ns),
        ttl = key === 'mode' ? Infinity : (s.meta.valid_for_ms ?? this.d[key] ?? 350);
      row[key + '_sample_id'] = s.sample_id;
      row[key + '_age_ms'] = age;
      const stale = age > ttl || (key === 'fc' && this.linkDown);
      if (stale) expired.add(key);
      for (const [field, value] of Object.entries(s.values)) {
        sources[field] = key;
        row[field] = stale ? null : value;
      }
    }
    const u = this.samples.get('uwb'),
      g = this.samples.get('gimbal');
    row.uwb_quality = Number.isFinite(row.uwb_range_m)
      ? 'measured'
      : Number.isFinite(row.uwb_pred_range_m)
        ? 'predicted'
        : u
          ? 'stale'
          : 'unknown';
    row.bearing_error_deg =
      u?.meta.calibration_id &&
      g?.meta.calibration_id &&
      Number.isFinite(row.uwb_bearing_deg) &&
      Number.isFinite(row.gimbal_yaw_deg)
        ? wrap(row.uwb_bearing_deg - row.gimbal_yaw_deg)
        : null;
    Object.assign(row, {
      record_phase: this.record.phase,
      recording_id: this.record.recording_id ?? null,
      clip_id: this.record.clip_id ?? null,
      active_alert_ids: [...this.alerts.values()].map((a) => a.alert_id)
    });
    return { row, expired, sources };
  }
  overview() {
    if (!this.session) return;
    const now = this.clock();
    if (this.lastRowAt && ageMs(now, this.lastRowAt) >= 100)
      this.event(
        'logger',
        'data_gap',
        { level: 'warn', message: '总表采样间隔超出一个周期' },
        { from_ns: String(this.lastRowAt), to_ns: String(now) }
      );
    this.lastRowAt = now;
    const { row } = this.values(now);
    Object.assign(row, {
      session_id: this.session,
      row_no: this.row++,
      mono_ns: String(now),
      t_ms: ageMs(now, this.start),
      utc: this.utc()
    });
    this.write('overview', Object.fromEntries(schema.overview.map((k) => [k, row[k] ?? null])));
  }
  snapshot() {
    const now = this.clock(),
      { row, expired, sources } = this.values(now);
    const alerts = [...this.alerts.values()];
    Object.assign(row, {
      preview_active: !!this.preview,
      record_elapsed_ms:
        this.record.phase === 'recording' && this.recordAt
          ? Math.max(0, (this.record.elapsed_ms ?? 0) + ageMs(now, this.recordAt))
          : (this.record.elapsed_ms ?? null),
      clip_no: this.record.clip_no ?? null,
      writer_state: this.writer.state ?? 'idle',
      last_flush_age_ms: validNs(this.writer.last_flush_ns) ? ageMs(now, this.writer.last_flush_ns) : null,
      drop_total: this.writer.drop_total ?? 0
    });
    const modules = Object.entries(moduleSources).map(([id, keys]) => {
      const aa = alerts.filter((a) => a.module === id),
        representative = keys.map((k) => this.samples.get(k)).find(Boolean);
      const demanded =
        id === 'logger'
          ? !!this.session || !!this.writer.state
          : id === 'mode' || keys.some((k) => this.enabled.has(k));
      const readings = moduleFields[id].split(' ').map((key) => {
        const source = sources[key] ?? sourceOf[key],
          sample = this.samples.get(source);
        let value = row[key] ?? null,
          quality = 'current';
        if (expired.has(source) && sample?.values[key] != null) {
          value = sample.values[key];
          quality = 'last_valid';
        }
        if (key === 'fc_mode' || key === 'armed') {
          if (this.linkDown) {
            value = null;
            quality = 'unavailable';
          }
        }
        return {
          key,
          label: labels[key] ?? key,
          value,
          unit: unitFor(key),
          quality:
            value === null
              ? 'unavailable'
              : quality === 'current' && sample?.quality === 'predicted'
                ? 'predicted'
                : quality
        };
      });
      const uncalibrated = id === 'imu' && representative?.meta.axes === 'sensor';
      const state = aa.some((a) => a.level === 'error')
        ? 'error'
        : aa.length
          ? 'warn'
          : !demanded
            ? 'idle'
            : id === 'logger' && this.writer.state === 'writing'
              ? 'ok'
              : uncalibrated
                ? 'unknown'
                : representative
                  ? 'ok'
                  : keys.some(
                        (k) => this.enabled.has(k) && ageMs(now, this.enabled.get(k)) < this.d.initial_wait
                      )
                    ? 'waiting'
                    : 'unknown';
      const note = aa.length
        ? aa.map((a) => a.message).join(' · ')
        : uncalibrated
          ? '已收到 IMU 数据，安装轴向待确认'
          : id === 'mode' &&
              this.mode.follow_active === false &&
              ['left', 'right'].includes(this.mode.follow_side)
            ? (this.mode.follow_side === 'left' ? '左侧' : '右侧') + '已选 · 跟随待机'
            : {
                ok: '数据正常',
                waiting: '等待首个有效样本',
                idle: '已停用',
                unknown: '尚无可验证的数据来源'
              }[state];
      return {
        id,
        state,
        readings,
        age_ms: representative ? ageMs(now, representative.obs_ns) : null,
        sample_id: representative?.sample_id ?? null,
        alert_ids: aa.map((a) => a.alert_id),
        note
      };
    });
    return {
      schema_version: 2,
      server_epoch: this.epoch,
      seq: ++this.seq,
      boot_id: this.boot,
      session_id: this.session,
      mono_ns: String(now),
      t_ms: this.session ? ageMs(now, this.start) : null,
      modules,
      active_alerts: alerts
    };
  }
  modeRequest(axis, requested, origin = 'web') {
    const prior = this.pending.get(axis);
    if (prior) this.modeOutcome(axis, prior.id, 'cancelled', '被新请求取代');
    const p = {
      id: randomUUID(),
      axis,
      requested,
      origin,
      from: this.actual(axis),
      at: this.clock(),
      state: 'waiting'
    };
    this.pending.set(axis, p);
    this.modeEvent(p, 'request', null);
    return p.id;
  }
  actual(axis) {
    return axis === 'fc'
      ? this.linkDown
        ? null
        : (this.samples.get('fc')?.values.fc_mode ?? null)
      : (this.mode[axis] ?? null);
  }
  modeEvent(p, stage, reason) {
    this.event(
      'mode',
      'mode_' + stage,
      {
        request_id: p.id,
        level: ['rejected', 'timeout'].includes(stage) ? 'error' : 'info',
        message: reason ?? p.axis
      },
      {
        axis: p.axis,
        origin: p.origin,
        from: p.from,
        requested: p.requested,
        actual: this.actual(p.axis),
        reason
      }
    );
  }
  modeOutcome(axis, id, stage, reason) {
    const p = this.pending.get(axis);
    if (!p || p.id !== id) return;
    if (['rejected', 'timeout'].includes(stage)) {
      p.state = stage;
      this.modeEvent(p, stage, reason);
      this.alert('mode', stage === 'timeout' ? 'MODE_TIMEOUT' : 'MODE_REJECTED', 'error', reason, {
        request_id: id
      });
      return;
    }
    this.modeEvent(p, stage, reason);
    this.pending.delete(axis);
    for (const [code, state] of [
      ['MODE_REJECTED', 'rejected'],
      ['MODE_TIMEOUT', 'timeout']
    ]) {
      if (![...this.pending.values()].some((other) => other.state === state))
        this.closeAlert(code, stage === 'cancelled' ? 'superseded' : 'recovered');
    }
  }
  applied(axis, value, previous) {
    const p = this.pending.get(axis);
    if (p && p.requested === value) {
      this.modeOutcome(axis, p.id, 'applied', null);
      return;
    }
    if (previous !== undefined && previous !== value)
      this.modeEvent(
        { axis, id: null, from: previous, requested: value, origin: 'device' },
        'applied',
        null
      );
  }
  modes(next) {
    const old = this.mode;
    if (Object.keys(next).every((k) => old[k] === next[k])) return;
    this.mode = { ...old, ...next };
    this.local(
      'mode',
      { ...this.mode },
      {
        mode_generation: this.worker.mode_generation ?? null,
        worker_epoch: this.worker.worker_epoch ?? null
      }
    );
    for (const [key, value] of Object.entries(next)) this.applied(key, value, old[key]);
  }
  mediaFrame(meta) {
    if (!meta?.frame_id) return;
    this.local('video', { frame_id: meta.frame_id }, meta, {
      sample_id: meta.frame_id,
      obs_ns: meta.obs_ns ?? null,
      time_kind: meta.time_kind ?? 'unknown'
    });
  }
  videoDemand(value) {
    this.preview = !!value;
    this.updateVideoDemand();
  }
  updateVideoDemand() {
    this.setExpected(
      'video',
      !!this.preview || ['starting', 'recording'].includes(this.record.phase) || this.enabled.has('vision')
    );
  }
  visionWorker(state, meta = {}) {
    if (['start', 'starting', 'running', 'enabled'].includes(state)) {
      this.worker = { ...meta };
      this.enabled.set('vision', this.clock());
      this.samples.delete('vision');
      this.local(
        'mode',
        { ...this.mode },
        { mode_generation: meta.mode_generation, worker_epoch: meta.worker_epoch }
      );
    } else if (['exit', 'failed', 'error', 'unexpected_exit'].includes(state)) {
      if (meta.worker_epoch !== this.worker.worker_epoch) return;
      this.worker.failed = '识别进程异常退出';
      this.worker.deadEpoch = meta.worker_epoch;
      this.check();
    } else if (['stopped', 'stop', 'disabled'].includes(state)) {
      this.worker = {};
      this.samples.delete('vision');
      this.setExpected('vision', false);
    }
    this.updateVideoDemand();
  }
  visionResult(result, frame = {}) {
    if (!result) return;
    const primary =
      this.enabled.has('vision') &&
      result.mode_generation === this.worker.mode_generation &&
      result.worker_epoch === this.worker.worker_epoch;
    const detected = result.state === 'detected',
      empty = result.state === 'no_target',
      pred = result.state === 'predicted';
    const targets = Array.isArray(result.targets) ? result.targets : [],
      target =
        result.selected_id == null
          ? Number.isInteger(result.selected_index)
            ? targets[result.selected_index]
            : null
          : targets.find((t) => t.id === result.selected_id);
    const last = this.samples.get('vision');
    const based = result.last_measured_frame_id ?? null;
    const meta = {
      ...frame,
      ...result,
      primary,
      based_on_sample_id:
        pred && last?.values.vision_last_measured_frame_id === based
          ? (last.meta.based_on_sample_id ?? last.sample_id)
          : null
    };
    const obs = frame?.obs_ns ?? null;
    this.local(
      'vision',
      {
        vision_state: result.state,
        vision_frame_id: result.frame_id ?? null,
        vision_last_measured_frame_id: based,
        detected: detected ? true : empty ? false : null,
        target_count: detected || empty ? targets.length : null,
        target_id: target?.id ?? null,
        confidence: target?.confidence ?? null,
        cx_norm: target ? target.x + target.w / 2 : null,
        cy_norm: target ? target.y + target.h / 2 : null
      },
      meta,
      {
        obs_ns: obs,
        time_kind: frame?.time_kind ?? 'unknown',
        quality: pred ? 'predicted' : ['error', 'not_run'].includes(result.state) ? 'invalid' : 'measured'
      }
    );
    if (!primary) return;
    if (result.state === 'error') {
      this.worker.failed = '视觉推理失败';
      this.check();
    } else if ((detected || empty) && this.worker.deadEpoch !== result.worker_epoch) {
      this.worker.lastSuccess = this.clock();
      this.worker.failed = null;
      this.closeAlert('VISION_FAILED');
      if (detected && target) {
        this.worker.lastLock = this.clock();
        this.closeAlert('TARGET_LOST');
      }
    }
  }
  recordState(state) {
    const old = this.record;
    this.recordAt = this.clock();
    this.record = { ...old, ...state };
    this.updateVideoDemand();
    if (state.phase === 'starting' && old.phase !== 'starting') this.recordStart = this.clock();
    if (old.phase !== state.phase)
      this.event(
        'video',
        'record_phase',
        { recording_id: state.recording_id, clip_id: state.clip_id, message: state.phase },
        { from: old.phase, to: state.phase, reason: state.reason ?? null }
      );
    if (state.phase === 'failed')
      this.alert('video', 'RECORD_FAILED', 'error', state.reason?.message || '录像失败');
    else if (['recording', 'ready', 'idle'].includes(state.phase))
      this.closeAlert('RECORD_FAILED', state.phase === 'idle' ? 'cancelled' : 'recovered');
    if (state.index_state === 'failed')
      this.alert('video', 'VIDEO_INDEX_FAILED', 'error', state.reason?.message || '视频索引失败');
    if (state.index_state === 'ready') this.closeAlert('VIDEO_INDEX_FAILED');
  }
}
