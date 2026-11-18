(function (window) {
    "use strict";

    const prefix = '/api/experiments';
    const mediaUrl = '/assets/manta-app/gimbal-demo.mp4';
    const demoClip = { clip_id: 'demo-clip-01', recording_id: 'demo-video', file_name: 'gimbal-demo.mp4', stream_url: mediaUrl };
    const moduleIds = ['fc', 'mode', 'imu', 'uwb', 'gimbal', 'vision', 'motors', 'video', 'logger'];

    function endpoint(path, query = {}) {
        const parameters = new URLSearchParams();
        for (const [key, value] of Object.entries(query)) {
            if (value != null && value !== '') parameters.set(key, value);
        }
        return prefix + path + (parameters.size ? '?' + parameters.toString() : '');
    }

    function sessionPath(id) {
        return '/sessions/' + encodeURIComponent(id);
    }

    function liveApi(request) {
        async function get(path, query, options = {}) {
            const response = await request(endpoint(path, query), { cache: 'no-store', ...options });
            return decode(response);
        }
        async function decode(response) {
            const payload = await response.json().catch(() => ({}));
            if (response.ok && payload.success === true) return payload.data;
            const error = new Error(payload.message || '请求失败（HTTP ' + response.status + '）');
            error.status = response.status;
            error.code = payload.code;
            throw error;
        }
        async function action(name) {
            return decode(await request(endpoint('/' + name), {
                method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}'
            }));
        }
        return {
            snapshot: signal => get('/live', {}, { signal }),
            sessions: query => get('/sessions', query),
            detail: id => get(sessionPath(id)),
            events: (id, query) => get(sessionPath(id) + '/events', query),
            locate: (id, query) => get(sessionPath(id) + '/locate', query),
            async download(id, kind) {
                const response = await request(endpoint(sessionPath(id) + '/files/' + encodeURIComponent(kind)));
                if (!response.ok) return decode(response);
                return response.blob();
            },
            start: () => action('start'),
            stop: () => action('stop')
        };
    }

    function validSnapshot(value) {
        return value && value.schema_version === 2 && typeof value.server_epoch === 'string'
            && Number.isSafeInteger(value.seq) && value.seq >= 0 && Array.isArray(value.modules) && Array.isArray(value.active_alerts);
    }

    // GET establishes the epoch; packets buffered during that request may only advance that epoch.
    class SnapshotFeed {
        constructor(api, emit, clock = () => performance.now()) {
            this.api = api;
            this.emit = emit;
            this.clock = clock;
            this.generation = 0;
            this.buffer = new Map();
            this.retired = new Set();
            this.snapshot = null;
            this.online = false;
            this.ready = false;
            this.pending = null;
        }
        open() {
            this.close();
            this.online = true;
            this.emit('experimentConnection', { connected: true, awaiting: true });
            this.refresh();
        }
        close() {
            if (this.online) this.emit('experimentConnection', { connected: false });
            this.generation++;
            this.online = false;
            this.ready = false;
            this.buffer.clear();
            this.pending?.controller.abort();
        }
        receive(snapshot) {
            if (!this.online || !validSnapshot(snapshot) || this.retired.has(snapshot.server_epoch)) return;
            if (this.ready && snapshot.server_epoch === this.snapshot?.server_epoch) {
                if (snapshot.seq >= this.snapshot.seq) this.lastPush = this.clock();
                this.apply(snapshot);
                return;
            }
            if (!this.buffer.has(snapshot.server_epoch) || this.buffer.get(snapshot.server_epoch).seq < snapshot.seq) {
                this.buffer.set(snapshot.server_epoch, snapshot);
            }
            this.ready = false;
            this.refresh();
        }
        apply(snapshot) {
            if (snapshot.server_epoch === this.snapshot?.server_epoch && snapshot.seq <= this.snapshot.seq) return;
            if (this.snapshot && snapshot.server_epoch !== this.snapshot.server_epoch) this.retired.add(this.snapshot.server_epoch);
            this.snapshot = snapshot;
            this.emit('experimentSnapshot', snapshot);
        }
        poll() {
            if (this.online && (!this.ready || this.clock() - this.lastPush >= 1500)) this.refresh();
        }
        refresh() {
            if (!this.online || this.pending) return;
            const controller = new AbortController();
            const generation = this.generation;
            const deadline = setTimeout(() => controller.abort(), 3500);
            this.pending = { controller };
            Promise.resolve().then(() => this.api.snapshot(controller.signal)).then(snapshot => {
                if (!this.online || generation !== this.generation || controller.signal.aborted) return;
                if (!validSnapshot(snapshot) || this.retired.has(snapshot.server_epoch)) throw new Error('日志状态版本无效');
                const pushed = this.buffer.get(snapshot.server_epoch);
                this.ready = true;
                this.buffer.clear();
                this.lastPush = this.clock();
                this.apply(pushed?.seq > snapshot.seq ? pushed : snapshot);
                this.emit('experimentConnection', { connected: true, synced: true });
            }).catch(error => {
                if (this.online && generation === this.generation) {
                    this.emit('experimentConnection', { connected: true, error: error.message });
                }
            }).finally(() => {
                clearTimeout(deadline);
                this.pending = null;
                if (generation !== this.generation && this.online) this.refresh();
            });
        }
    }

    const demoState = {
        fc_mode: 'MANUAL', armed: false, control_owner: 'uwb', follow_side: 'left', follow_active: false,
        heading_deg: 82, yaw_rate_deg_s: 4, ground_speed_m_s: 0.6,
        ax_m_s2: 0, ay_m_s2: 0.2, az_m_s2: 9.8, uwb_range_m: 4.2, uwb_bearing_deg: 18,
        gimbal_yaw_deg: 12, gimbal_pitch_deg: 0, gimbal_cmd_yaw_deg: 15, bearing_error_deg: 6,
        vision_state: 'detected', detected: true, target_count: 1, left_pwm_us: 1530, right_pwm_us: 1510, left_rpm: null
    };
    const demoFrames = [0, 1, 2, 3, 4, 5].map(display_no => ({ ...demoClip, display_no,
        frame_id: 'mock/f' + display_no, play_us: display_no * 200000, t_ms: 10000 + display_no * 200,
        time_kind: 'host_decode', identity: 'exact' }));
    const demoHistory = [
        [1, 10000, 'mode', 'info', 'MODE_APPLIED', '已选择左侧伴随 · 跟随待机', 'mode_applied', null],
        [2, 10350, 'uwb', 'warn', 'UWB_STALE', 'UWB 实测更新超时', 'alert_open', 'demo-uwb'],
        [3, 11050, 'uwb', 'error', 'UWB_STALE', 'UWB 实测更新超时，已升为错误', 'alert_level', 'demo-uwb'],
        [4, 11700, 'uwb', 'info', 'UWB_STALE', '已恢复收到有效 UWB 实测', 'alert_close', 'demo-uwb'],
        [5, 12200, 'video', 'info', 'RECORD_STOPPED', '本段录像已封装', 'record_stop', null]
    ].map(([event_no, t_ms, module, level, code, message, type, alert_id]) => ({
        event_no, event_id: 'demo/event-' + event_no, t_ms, module, level, code, message, type, alert_id,
        session_id: 'demo-session', mono_ns: String(t_ms * 1e6), detail_json: type === 'alert_close' ? { reason: 'recovered' } : {}
    }));

    function demoReading(key, label, unit, value = demoState[key], quality = 'current') {
        return { key, label, value: value ?? null, unit: unit ?? null, quality: value == null ? 'unavailable' : quality };
    }

    function mockApi(transport) {
        let seq = 0;
        let started;
        let videoStarted;
        const state = {};
        for (const key of ['uwbFollow', 'gimbalState', 'recordingState', 'armed']) {
            transport.on(key, value => {
                state[key] = value;
                if (key === 'recordingState') videoStarted = value.active ? Date.now() : null;
            });
        }
        const startedUtc = '2026-11-18T03:00:00Z';
        const recent = () => ({ session_id: 'demo-session', started_utc: startedUtc, status: 'closed' });
        const snapshot = () => {
            seq++;
            const elapsed = started == null ? null : Math.max(0, Date.now() - started);
            const readings = {
                fc: [demoReading('heading_deg', '航向', '°'), demoReading('yaw_rate_deg_s', '艏向角速度', '°/s'), demoReading('ground_speed_m_s', '航速', 'm/s')],
                mode: [demoReading('fc_mode', '飞控'), demoReading('armed', '解锁', null, state.armed?.armed ?? false), demoReading('follow_side', '伴随位置', null, state.uwbFollow?.followMode || 'left'), demoReading('follow_active', '跟随', null, state.uwbFollow?.active ?? false)],
                imu: [demoReading('ax_m_s2', '前轴加速度', 'm/s²'), demoReading('ay_m_s2', '右轴加速度', 'm/s²'), demoReading('az_m_s2', '下轴加速度', 'm/s²')],
                uwb: [demoReading('uwb_range_m', '上次实测距离', 'm', 4.2, 'last_valid'), demoReading('uwb_bearing_deg', '上次实测方位', '°', 18, 'last_valid')],
                gimbal: [demoReading('gimbal_yaw_deg', '实测偏航', '°'), demoReading('gimbal_cmd_yaw_deg', '目标偏航', '°'), demoReading('bearing_error_deg', '目标相对镜头方位', '°')],
                vision: [demoReading('vision_state', '本帧', null, state.gimbalState?.trackingActive === false ? 'not_run' : 'detected'), demoReading('target_count', '目标', '人', state.gimbalState?.trackingActive === false ? null : 1)],
                motors: [demoReading('left_pwm_us', '飞控左输出', 'µs'), demoReading('right_pwm_us', '飞控右输出', 'µs'), demoReading('left_rpm', '左转速', 'RPM')],
                video: [demoReading('preview_active', '画面', null, true), demoReading('record_phase', '录像', null, videoStarted == null ? 'idle' : 'recording'), demoReading('record_elapsed_ms', '录制时长', 'ms', videoStarted == null ? null : Date.now() - videoStarted)],
                logger: [demoReading('writer_state', '写入', null, elapsed === null ? 'closed' : 'writing'), demoReading('last_flush_age_ms', '最近写盘', 'ms', elapsed === null ? null : 400), demoReading('drop_total', '丢弃', null, 0)]
            };
            return { schema_version: 2, server_epoch: 'mock-epoch', seq, boot_id: 'mock-boot', session_id: elapsed === null ? null : 'demo-session',
                mono_ns: String(12e9 + seq * 250e6), t_ms: elapsed, simulated: true,
                modules: moduleIds.map(id => ({ id, state: id === 'uwb' ? 'warn' : (id === 'logger' && elapsed === null || id === 'vision' && state.gimbalState?.trackingActive === false) ? 'idle' : 'ok',
                    readings: readings[id], age_ms: id === 'uwb' ? 870 : id === 'logger' && elapsed === null ? null : 0,
                    sample_id: id + '/mock', alert_ids: id === 'uwb' ? ['demo-live-uwb'] : [], note: id === 'uwb' ? '870 ms 未收到有效实测' : '' })),
                active_alerts: [{ alert_id: 'demo-live-uwb', module: 'uwb', code: 'UWB_STALE', level: 'warn', first_ns: '10351000000',
                    changed_ns: '10351000000', message: 'UWB 实测更新超时', observed: 870, threshold: 350, unit: 'ms', sample_id: 'uwb/mock' }] };
        };
        return {
            snapshot: async () => snapshot(),
            sessions: async () => ({ items: [{ ...recent(), status: started == null ? 'closed' : 'writing' }], next_cursor: null }),
            detail: async () => ({ ...recent(), schema_version: 2, simulated: true, clips: [demoClip], files: ['overview', 'events', 'video'].map(kind => ({ kind, available: true })) }),
            events: async (_id, { after = 0, limit = 100, module, level } = {}) => {
                const matches = demoHistory.filter(event => event.event_no > after && (!module || event.module === module) && (!level || event.level === level));
                const items = matches.slice(0, limit);
                return { items, next_after: items.at(-1)?.event_no ?? after, has_more: matches.length > items.length };
            },
            locate: async (_id, query) => {
                const frame = query.display_no != null ? demoFrames.find(item => item.display_no === Number(query.display_no))
                    : query.t_ms != null ? demoFrames.find(item => item.t_ms <= query.t_ms && query.t_ms < item.t_ms + 200)
                    : demoFrames.find(item => item.play_us <= query.play_us && query.play_us < item.play_us + 200000);
                return { match: frame ? 'matched' : 'no_video', frame: frame || null, state: frame ? demoState : null };
            },
            download: async (_id, kind) => {
                const source = kind === 'video' ? demoFrames : kind === 'events' ? demoHistory : [{ ...demoState, t_ms: 10000 }];
                const keys = Object.keys(source[0]);
                const escape = value => value == null ? '' : '"' + String(typeof value === 'object' ? JSON.stringify(value) : value).replaceAll('"', '""') + '"';
                return new Blob([keys.join(',') + '\n' + source.map(row => keys.map(key => escape(row[key])).join(',')).join('\n') + '\n'], { type: 'text/csv;charset=utf-8' });
            },
            start: async () => { started ??= Date.now(); return recent(); },
            stop: async () => { started = null; return recent(); }
        };
    }

    window.MantaExperimentApi = { liveApi, mockApi, SnapshotFeed };
})(window);
