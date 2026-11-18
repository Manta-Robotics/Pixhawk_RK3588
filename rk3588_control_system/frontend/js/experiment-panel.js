const MODULES = {
    fc: '飞控与船体', mode: '控制模式', imu: 'IMU', uwb: 'UWB', gimbal: '云台',
    vision: '视觉', motors: '电机', video: '视频与录像', logger: '日志写入'
};
const STATES = { ok: '正常', warn: '警告', error: '错误', idle: '待机', waiting: '等待数据', unknown: '未知' };
const LEVELS = { info: '信息', warn: '警告', error: '错误' };
const CSV = { overview: 'state_overview.csv', events: 'events.csv', video: 'video_aligned.csv' };
const ENUMS = {
    follow_side: { rear: '后方', left: '左侧', right: '右侧' },
    follow_method: { traditional: '传统', neural: '神经网络' },
    control_owner: { uwb: 'UWB', manual: '手动', joystick: '摇杆' },
    vision_state: { detected: '本帧检测到目标', no_target: '本帧未检测到目标', predicted: '预测中', not_run: '本帧未推理', error: '推理错误' },
    uwb_quality: { measured: '实测', predicted: '预测', stale: '实测已过期', unknown: '未知' },
    record_phase: { idle: '未录像', recording: '录制中', stopping: '正在封装', finalizing: '正在封装', starting: '等待首帧', ready: '已封装', stopped: '已停止', failed: '录像失败', error: '录像错误' },
    writer_state: { idle: '未记录', starting: '正在启动', writing: '写入中', failed: '写入失败', closed: '未记录' }
};
const STATE_FIELDS = {
    fc_mode: ['飞控模式'], armed: ['解锁'], control_owner: ['船体控制'], follow_side: ['跟随位置'],
    follow_active: ['跟随'], follow_method: ['跟随方法'], follow_control_mode: ['跟随控制模式'],
    gimbal_mode: ['云台模式'], heading_deg: ['航向', '°'], yaw_rate_deg_s: ['转向', '°/s'],
    ground_speed_m_s: ['航速', 'm/s'], ax_m_s2: ['前轴加速度', 'm/s²'], ay_m_s2: ['右轴加速度', 'm/s²'],
    az_m_s2: ['下轴加速度', 'm/s²'], uwb_range_m: ['UWB 距离', 'm'], uwb_bearing_deg: ['UWB 方位', '°'],
    uwb_quality: ['UWB 质量'], uwb_pred_range_m: ['UWB 预测距离', 'm'], uwb_pred_bearing_deg: ['UWB 预测方位', '°'],
    gimbal_yaw_deg: ['云台实测偏航', '°'], gimbal_pitch_deg: ['云台实测俯仰', '°'],
    gimbal_cmd_yaw_deg: ['云台目标偏航', '°'], bearing_error_deg: ['目标相对镜头方位', '°'],
    frame_vision_state: ['该帧识别结果'], frame_target_count: ['该帧目标数'], frame_detected: ['该帧检出'],
    vision_state: ['识别状态'], vision_frame_id: ['当时可用的识别帧'], target_count: ['目标数'],
    left_pwm_us: ['飞控左输出', 'µs'], right_pwm_us: ['飞控右输出', 'µs'], left_rpm: ['左机械转速', 'RPM'], right_rpm: ['右机械转速', 'RPM']
};
const SESSION_STATUS = { writing: '记录中', active: '记录中', starting: '正在启动', closed: '已结束', partial: '不完整', failed: '失败' };
const MATCH_LABELS = { matched: '已定位对应录像', no_video: '该时段无录像', time_unknown: '画面时间无法与实验对齐', media_missing: '录像文件已缺失，可查看留存的历史数据', ambiguous: '存在多个可用候选，请选择' };
const $ = selector => document.querySelector(selector);
const decimal = new Intl.NumberFormat('zh-CN', { useGrouping: false, maximumFractionDigits: 3 });

function element(tag, className, text) {
    const node = document.createElement(tag);
    if (className) node.className = className;
    if (text != null) node.textContent = text;
    return node;
}

function button(text, onClick) {
    const node = element('button', 'experiment-secondary', text);
    node.type = 'button';
    node.addEventListener('click', onClick);
    return node;
}

function number(value) {
    return typeof value === 'number' && Number.isFinite(value) ? decimal.format(value) : '—';
}

function hasTime(value) {
    return typeof value === 'number' && Number.isFinite(value) && value >= 0;
}

function measurement(value, unit, key) {
    if (value == null) return '—';
    if (typeof value === 'boolean') {
        if (key === 'armed') return value ? '已解锁' : '已解除武装';
        if (key === 'follow_active') return value ? '跟随中' : '跟随待机';
        if (key === 'preview_active') return value ? '正常' : '未开启';
        if (key === 'frame_detected') return value ? '是' : '否';
        return value ? '是' : '否';
    }
    if (key === 'frame_vision_state') key = 'vision_state';
    const formatted = typeof value === 'number' ? number(value) : ENUMS[key]?.[value] || String(value);
    return formatted + (unit ? (unit === '°' ? '' : ' ') + unit : '');
}

function monotonicLabel(stamp, snapshot) {
    if (!/^[0-9]+$/.test(stamp || '')) return '—';
    if (hasTime(snapshot?.t_ms) && /^[0-9]+$/.test(snapshot.mono_ns)) {
        const elapsed = snapshot.t_ms + Number(BigInt(stamp) - BigInt(snapshot.mono_ns)) / 1e6;
        if (elapsed >= 0) return number(elapsed / 1000) + ' s（实验）';
    }
    return number(Number(BigInt(stamp) / 1000n) / 1e6) + ' s（开机）';
}

function listOptions(select, values, includeAll) {
    if (includeAll) select.add(new Option('全部', ''));
    for (const [value, label] of Object.entries(values)) select.add(new Option(label, value));
}

class LiveView {
    constructor(root, openHistory) {
        this.root = root;
        this.openHistory = openHistory;
        const summary = element('div', 'experiment-summary');
        this.summary = element('strong', '', '等待连接');
        this.connection = element('span', 'experiment-connection', '尚未连接设备');
        summary.append(this.summary, this.connection);
        const wrapper = element('div', 'experiment-table-wrap');
        const table = element('table', 'experiment-table');
        table.setAttribute('aria-label', '船端实时模块状态');
        const head = table.createTHead().insertRow();
        for (const text of ['模块', '状态', '当前读数', '距上次有效更新', '原因']) {
            const th = element('th', '', text); th.scope = 'col'; head.append(th);
        }
        const body = table.createTBody();
        this.rows = new Map(Object.entries(MODULES).map(([id, label]) => {
            const row = body.insertRow();
            row.dataset.module = id;
            const title = element('th'); title.scope = 'row';
            const toggle = button(label, () => {
                detail.hidden = !detail.hidden;
                toggle.setAttribute('aria-expanded', String(!detail.hidden));
                if (!detail.hidden) this.details(id);
            });
            toggle.setAttribute('aria-expanded', 'false');
            title.append(toggle); row.append(title);
            const cells = Array.from({ length: 4 }, () => row.insertCell());
            cells[0].className = 'experiment-state';
            cells[2].className = 'experiment-age';
            const detail = body.insertRow(); detail.className = 'experiment-detail'; detail.hidden = true;
            const detailCell = detail.insertCell(); detailCell.colSpan = 5;
            return [id, { row, cells, detail, detailCell }];
        }));
        wrapper.append(table);
        root.replaceChildren(summary, wrapper);
        this.render(null, { connected: false }, false);
    }
    render(snapshot, link, simulated) {
        this.snapshot = snapshot;
        const alerts = snapshot?.active_alerts || [];
        const counts = { error: new Set(), warn: new Set() };
        for (const alert of alerts) counts[alert.level]?.add(alert.alert_id);
        const total = counts.error.size + counts.warn.size;
        this.summary.textContent = snapshot ? total ? `${counts.error.size} 个错误 · ${counts.warn.size} 个警告` : '当前无活动告警' : '等待首个状态快照';
        this.summary.dataset.state = counts.error.size ? 'error' : counts.warn.size ? 'warn' : 'ok';
        const linkText = link?.connected ? link.error ? '状态读取失败 · ' + link.error
            : link.awaiting ? '正在同步状态' : '实时连接正常' : snapshot ? '实时连接已断开 · 显示最后快照' : '尚未连接设备';
        this.connection.textContent = (simulated ? '演示数据 · ' : '') + linkText;
        this.connection.dataset.offline = String(!link?.connected);
        this.root.dataset.frozen = String(Boolean(snapshot && !link?.connected));
        const byId = new Map((snapshot?.modules || []).map(module => [module.id, module]));
        for (const [id, { row, cells, detail }] of this.rows) {
            const module = byId.get(id);
            const ownAlerts = alerts.filter(alert => alert.module === id);
            const state = ownAlerts.some(alert => alert.level === 'error') ? 'error' : ownAlerts.length ? 'warn' : module?.state || 'unknown';
            row.dataset.state = state; cells[0].textContent = STATES[state] || STATES.unknown;
            const readings = module?.readings || [];
            const signature = JSON.stringify(readings);
            if (cells[1].dataset.signature !== signature) {
                cells[1].dataset.signature = signature;
                const values = readings.map(reading => {
                    const label = reading.label || reading.key;
                    const value = measurement(reading.value, reading.unit, reading.key);
                    const qualifier = reading.quality === 'last_valid' && !label.includes('上次实测') ? ' · 上次实测'
                        : reading.quality === 'predicted' ? ' · 预测' : '';
                    return element('span', 'experiment-reading', `${label} ${value}${qualifier}`);
                });
                cells[1].replaceChildren(...(values.length ? values : [document.createTextNode('—')]));
            }
            cells[2].textContent = hasTime(module?.age_ms) ? number(module.age_ms) + ' ms' : '—';
            cells[3].textContent = module?.note || ownAlerts.map(alert => alert.message).filter(Boolean).join('、') || '—';
            if (!detail.hidden) this.details(id);
        }
    }
    details(id) {
        const { detailCell } = this.rows.get(id);
        const alerts = (this.snapshot?.active_alerts || []).filter(alert => alert.module === id);
        const module = this.snapshot?.modules.find(item => item.id === id);
        const signature = JSON.stringify([alerts, module?.sample_id, module?.readings]);
        if (detailCell.dataset.signature === signature) return;
        detailCell.dataset.signature = signature;
        const content = alerts.map(alert => {
            const item = element('div', 'experiment-alert');
            item.append(element('strong', '', `${LEVELS[alert.level] || alert.level} · ${alert.message}`));
            item.append(element('span', '', `首次 ${monotonicLabel(alert.first_ns, this.snapshot)} · 观测 ${measurement(alert.observed, alert.unit)} · 阈值 ${measurement(alert.threshold, alert.unit)}`));
            item.append(element('small', '', [alert.code, alert.alert_id, alert.sample_id].filter(Boolean).join(' · ')));
            item.append(button('查看相关事件', () => this.openHistory({ module: id, alert: alert.alert_id })));
            return item;
        });
        const last = (module?.readings || []).filter(item => ['current', 'last_valid'].includes(item.quality) && item.value != null)
            .map(item => (item.label || item.key) + ' ' + measurement(item.value, item.unit, item.key));
        const evidence = [module?.sample_id, ...last].filter(Boolean).join(' · ');
        if (evidence) content.unshift(element('p', 'experiment-status', '最近有效读数 · ' + evidence));
        detailCell.replaceChildren(...(content.length ? content : [element('span', '', '暂无关联告警')]));
    }
}

const CONTROL_ENTRIES = {
    mode: [['跟随启停', '#driveFollowSwitch']],
    uwb: [['UWB 跟随启停', '#driveFollowSwitch'], ['信标跟踪', '[data-gimbal-mode="beacon"]']],
    gimbal: [['信标跟踪', '[data-gimbal-mode="beacon"]'], ['停止云台', '[data-gimbal-action="stop"]']],
    vision: [['人脸跟踪', '[data-gimbal-mode="face"]'], ['泳者跟踪', '[data-gimbal-mode="swimmer"]'], ['停止跟踪', '[data-gimbal-action="stop"]']],
    video: [['录像启停', '#recordButton'], ['刷新预览', '#refreshGimbalVideo']]
};
const preview = window.MantaPreview;
const inlineRoot = $('#experimentLive');
const playbackRoot = $('#experimentPlayback');
const dialog = element('dialog', 'experiment-dialog');
dialog.setAttribute('aria-labelledby', 'experimentDialogTitle');
dialog.innerHTML = `
  <header class="experiment-dialog-head">
    <div><h2 id="experimentDialogTitle">完整日志</h2><p id="experimentHistoryLink"></p></div>
    <button type="button" class="experiment-secondary" data-exp-close>关闭</button>
  </header>
  <div class="experiment-dialog-body">
    <details class="experiment-panel-section"><summary>实时模块状态</summary><div id="experimentFullLive"></div></details>
    <section class="experiment-panel-section" aria-label="模块控制入口">
      <div class="experiment-tools">
        <label>模块视图 <select id="experimentControlModule"></select></label>
        <div class="experiment-tools" id="experimentControlLinks"></div>
        <button class="experiment-secondary" type="button" data-cinematography-open aria-haspopup="dialog">自主运镜</button>
      </div>
      <p id="experimentSelectedModule" class="experiment-status"></p>
    </section>
    <section class="experiment-panel-section" aria-label="日志记录">
      <div class="experiment-tools" id="experimentRecordActions">
        <button type="button" class="experiment-secondary" data-record-action="start">开始记录</button>
        <button type="button" class="experiment-secondary" data-record-action="stop">结束记录</button>
        <span id="experimentRecordingNote" role="status"></span>
      </div>
    </section>
    <section class="experiment-panel-section" aria-label="历史会话">
      <div class="experiment-tools">
        <label>实验会话 <select id="experimentSessions"></select></label>
        <button type="button" class="experiment-secondary" id="experimentMoreSessions" hidden>更多会话</button>
        <button type="button" class="experiment-secondary" id="experimentRefresh">刷新历史</button>
      </div>
      <div class="experiment-tools" id="experimentFiles"></div>
      <form class="experiment-tools" id="experimentTimeForm">
        <label>实验时间 t_ms <input id="experimentTime" type="number" min="0" step="any" required placeholder="如 10000"></label>
        <button type="submit" class="experiment-secondary" title="输入 CSV 中的 t_ms">查找对应画面</button>
        <label>录像段 <select id="experimentClips"></select></label>
        <button type="button" class="experiment-secondary" id="experimentPlayClip">查看此录像</button>
      </form>
      <p id="experimentHistoryStatus" class="experiment-status" role="status"></p>
      <ul id="experimentGaps" class="experiment-gap-list" hidden></ul>
      <div id="experimentHistoryResult"></div>
    </section>
    <section class="experiment-panel-section" aria-labelledby="experimentEventsTitle">
      <div class="experiment-events-head"><h3 id="experimentEventsTitle">历史事件</h3>
        <div class="experiment-tools">
          <label>模块 <select id="experimentModule"></select></label>
          <label>级别 <select id="experimentLevel"></select></label>
          <button type="button" class="experiment-secondary" id="experimentClearAlert" hidden>查看全部告警</button>
        </div>
      </div>
      <p id="experimentEventStatus" class="experiment-status" role="status"></p>
      <ol id="experimentEvents" class="experiment-events"></ol>
      <button type="button" class="experiment-secondary" id="experimentOlderEvents" hidden>查看更多旧事件</button>
    </section>
  </div>`;
document.body.append(dialog);
const full = new LiveView($('#experimentFullLive'), openHistory);
const inline = new LiveView(inlineRoot, openHistory);
listOptions($('#experimentModule'), MODULES, true);
listOptions($('#experimentLevel'), LEVELS, true);
listOptions($('#experimentControlModule'), MODULES, false);
const views = [inline, full];

let transport, snapshot, simulated, activeAlert, statusBusy, lastAlertKey, lastSession, activeSession;
let link = { connected: false };
let detach = [];
let transportVersion = 0;
let connectionVersion = 0;
let sessionVersion = 0;
let selectedId = '';
let sessions = new Map();
let sessionCursor = null;
let manifest = null;
let events = new Map();
let after = 0;
let eventTask = null;
let listTask = null;
let visibleEvents = 100;
let locateAbort;
let selectedClipContext;
let presentedFrame;
let openingMedia;
let lastEventPoll = -Infinity;

function renderLive() {
    for (const view of views) view.render(snapshot, link, simulated);
    $('#experimentHistoryLink').textContent = full.connection.textContent;
    const writable = Boolean(transport && (simulated ? !transport.experimentReadOnly : transport.canControl) && link.connected && link.synced);
    const writer = snapshot?.modules.find(module => module.id === 'logger')?.readings.find(reading => reading.key === 'writer_state')?.value;
    const hasTask = Boolean(snapshot?.session_id) || ['starting', 'writing', 'failed'].includes(writer);
    const canRecover = snapshot?.active_alerts.some(alert => alert.code === 'LOGGER_OFFLINE') === true;
    const start = $('#experimentRecordActions').querySelector('[data-record-action="start"]');
    start.textContent = canRecover ? '恢复记录' : '开始记录';
    start.disabled = !writable || hasTask && !canRecover || statusBusy;
    $('#experimentRecordActions').querySelector('[data-record-action="stop"]').disabled = !writable || !hasTask || statusBusy;
    if (!statusBusy) $('#experimentRecordingNote').textContent = snapshot?.session_id ? '当前会话 ' + snapshot.session_id
        : link.connected && !writable ? '当前页面只读' : hasTask ? ENUMS.writer_state[writer] : '无活动记录任务';
    renderControlLinks(writable);
}

function renderControlLinks(writable) {
    const slot = $('#experimentControlLinks');
    const id = $('#experimentControlModule').value;
    const selected = snapshot?.modules.find(module => module.id === id);
    const alert = snapshot?.active_alerts.filter(item => item.module === id) || [];
    const observedState = alert.some(item => item.level === 'error') ? 'error' : alert.length ? 'warn' : selected?.state;
    const reading = (selected?.readings || []).map(item => (item.label || item.key) + ' ' + measurement(item.value, item.unit, item.key));
    $('#experimentSelectedModule').textContent = [STATES[observedState] || '等待数据', ...reading, selected?.note].filter(Boolean).join(' · ');
    const listing = CONTROL_ENTRIES[id] || [];
    const keys = [id, writable, ...listing.map(([, selector]) => Boolean($(selector) && !$(selector).disabled))].join('|');
    if (slot.dataset.keys === keys) return;
    slot.dataset.keys = keys;
    if (!listing.length) {
        const note = id === 'logger' ? '下方可开始或结束记录' : '该模块随设备状态运行';
        slot.replaceChildren(element('span', 'experiment-status', note));
        return;
    }
    const links = listing.map(([label, selector]) => {
        const trigger = $(selector);
        const linkButton = button(label, () => {
            if (!transport?.connected || (simulated ? transport.experimentReadOnly : !transport.canControl) || !trigger || trigger.disabled) return;
            if (trigger.closest('[hidden]') && selector === '#driveFollowSwitch') {
                dialog.close();
                preview.showView('device');
                $('#autoFollowButton')?.focus();
                return;
            }
            trigger.click();
        });
        linkButton.disabled = !writable || !trigger || trigger.disabled;
        return linkButton;
    });
    slot.replaceChildren(...links);
}

function bind(next) {
    if (!next || next === transport) return;
    for (const dispose of detach) dispose();
    transport = next;
    transportVersion++;
    connectionVersion++;
    listTask = null;
    sessions = new Map();
    sessionCursor = null;
    selectedId = '';
    lastSession = null;
    activeSession = null;
    lastAlertKey = null;
    snapshot = null;
    link = { connected: false };
    simulated = transport.mode === 'mock';
    detach = [
        next.on('experimentSnapshot', incoming => {
            snapshot = incoming;
            if (incoming.session_id) lastSession = incoming.session_id;
            renderLive();
            const active = incoming.session_id;
            const alertKey = incoming.active_alerts.map(alert => `${alert.alert_id}/${alert.level}/${alert.changed_ns}`).join('|');
            if (link.synced && dialog.open && performance.now() - lastEventPoll >= 2000) {
                lastEventPoll = performance.now();
                refreshEvents();
            }
            if (activeSession !== active || lastAlertKey !== alertKey) {
                const sessionChanged = activeSession !== active;
                activeSession = active;
                lastAlertKey = alertKey;
                if (link.synced && dialog.open) {
                    if (sessionChanged) loadSessions();
                    refreshEvents();
                }
            }
        }),
        next.on('experimentConnection', state => {
            if (!state.connected || state.awaiting) connectionVersion++;
            link = state;
            renderLive();
            if (state.synced && dialog.open) { loadSessions(); loadManifest(); refreshEvents(); }
        }),
        next.on('controlSession', renderLive)
    ];
    selectSession('');
    renderLive();
}

function openHistory(options = {}) {
    if (!dialog.open) dialog.showModal();
    if (options.module) $('#experimentModule').value = options.module;
    if (options.alert) {
        activeAlert = options.alert;
        $('#experimentLevel').value = '';
        $('#experimentClearAlert').hidden = false;
        if (lastSession && selectedId !== lastSession) selectSession(lastSession);
    }
    renderEvents();
    renderLive();
    if (transport?.experiments && link.connected) loadSessions();
    else $('#experimentHistoryStatus').textContent = '连接并配对后可查看实验会话';
}

function renderSessions() {
    const select = $('#experimentSessions');
    select.replaceChildren(new Option(sessions.size ? '选择实验会话' : '暂无实验会话', ''));
    for (const [id, session] of sessions) {
        const date = session.started_utc ? new Date(session.started_utc).toLocaleString('zh-CN', { hour12: false }) : id;
        select.add(new Option([date, SESSION_STATUS[session.status] || session.status, id].filter(Boolean).join(' · '), id));
    }
    select.value = selectedId;
    $('#experimentMoreSessions').hidden = !sessionCursor;
}

async function loadSessions(more = false) {
    if (!transport?.experiments || !link.connected || listTask) return;
    const task = { transportVersion, connectionVersion };
    listTask = task;
    try {
        const result = await transport.experiments.sessions({ before: more ? sessionCursor : null, limit: 20 });
        if (transportVersion !== task.transportVersion || connectionVersion !== task.connectionVersion) return;
        const first = new Map(result.items.map(item => [item.session_id, item]));
        sessions = more ? new Map([...sessions, ...first]) : new Map([...first, ...[...sessions].filter(([id]) => !first.has(id))]);
        sessionCursor = result.next_cursor;
        renderSessions();
        if (!selectedId) selectSession(lastSession && sessions.has(lastSession) ? lastSession : sessions.keys().next().value || '');
        else if (!manifest) loadManifest();
        if (!sessions.size) $('#experimentHistoryStatus').textContent = '暂无实验会话';
    } catch (error) {
        if (transportVersion === task.transportVersion && connectionVersion === task.connectionVersion) $('#experimentHistoryStatus').textContent = error.message;
    } finally {
        if (listTask === task) {
            listTask = null;
            if (task.connectionVersion !== connectionVersion && link.synced && dialog.open) loadSessions();
        }
    }
}

function sessionClips() {
    const found = new Map();
    for (const clip of manifest?.clips || []) if (clip?.clip_id) found.set(clip.clip_id, clip);
    for (const recording of manifest?.recordings || []) {
        for (const clip of recording.clips || []) if (clip?.clip_id) found.set(clip.clip_id, { recording_id: recording.recording_id, ...clip });
    }
    return [...found.values()];
}

function renderManifest() {
    const files = Object.entries(CSV).map(([kind, label]) => {
        const entry = manifest?.files?.find(file => file.kind === kind);
        const download = button(label, () => downloadFile(kind, label, download));
        download.disabled = entry?.available !== true;
        if (!download.disabled) download.title = '下载 ' + label;
        else download.title = '当前没有可下载的已提交版本';
        return download;
    });
    $('#experimentFiles').replaceChildren(...files);
    const select = $('#experimentClips');
    const before = select.value;
    select.replaceChildren(new Option('选择录像段', ''));
    for (const clip of sessionClips()) select.add(new Option(clip.file_name || clip.filename || clip.clip_id, clip.clip_id));
    select.value = before;
    if (select.options.length === 2 && !select.value) select.selectedIndex = 1;
    const gapList = $('#experimentGaps');
    const gaps = Array.isArray(manifest?.gaps) ? manifest.gaps : [];
    gapList.hidden = gaps.length === 0;
    gapList.replaceChildren(...gaps.map(gap => {
        const span = gap.first_missing != null && gap.last_missing != null
            ? `序号 ${gap.first_missing}–${gap.last_missing}` : '范围未知';
        const labels = { buffer_expired: '缓冲已过期', node_restart: 'Node 重启', writer_error: '写入失败' };
        return element('li', '', `记录缺口 · ${span} · ${labels[gap.reason] || gap.reason || '来源缺失'}`);
    }));
    renderClipAction();
    $('#experimentTimeForm').querySelector('[type="submit"]').disabled = !selectedId;
}

function renderClipAction() {
    const clip = sessionClips().find(item => item.clip_id === $('#experimentClips').value);
    $('#experimentPlayClip').disabled = !clip?.stream_url;
}

function selectSession(id) {
    selectedId = id;
    sessionVersion++;
    locateAbort?.abort();
    manifest = null;
    events = new Map();
    after = 0;
    visibleEvents = 100;
    $('#experimentSessions').value = id;
    $('#experimentHistoryResult').replaceChildren();
    $('#experimentHistoryStatus').textContent = id ? '正在读取会话…' : '';
    if (id) {
        playbackRoot.hidden = false;
        const tools = element('div', 'experiment-tools');
        tools.append(button('按当前画面反查', reverseLocate), button('实验历史', () => openHistory()));
        playbackRoot.replaceChildren(tools, element('div'));
    }
    renderManifest();
    renderEvents();
    if (id) { loadManifest(); refreshEvents(); }
}

async function loadManifest() {
    if (!selectedId || !transport?.experiments) return;
    const version = sessionVersion;
    const connection = connectionVersion;
    try {
        const detail = await transport.experiments.detail(selectedId);
        if (version !== sessionVersion || connection !== connectionVersion) return;
        manifest = detail;
        $('#experimentHistoryStatus').textContent = detail.schema_version === 2 ? '' : '旧会话 · 按可用索引和录像读取';
        renderManifest();
    } catch (error) {
        if (version === sessionVersion && connection === connectionVersion) $('#experimentHistoryStatus').textContent = error.message;
    }
}

async function refreshEvents() {
    if (!selectedId || !transport?.experiments || !link.connected || eventTask) return;
    const task = { session: sessionVersion, connection: connectionVersion };
    eventTask = task;
    try {
        let hasMore;
        do {
            const result = await transport.experiments.events(selectedId, { after, limit: 100 });
            if (task.session !== sessionVersion || task.connection !== connectionVersion) return;
            for (const event of result.items) events.set(event.event_no, event);
            hasMore = result.has_more && result.next_after > after;
            if (result.next_after >= after) after = result.next_after;
            if (result.items.length) renderEvents();
        } while (hasMore);
    } catch (error) {
        if (task.session === sessionVersion && task.connection === connectionVersion) $('#experimentEventStatus').textContent = error.message;
    } finally {
        if (eventTask === task) {
            eventTask = null;
            if ((task.session !== sessionVersion || task.connection !== connectionVersion) && link.connected) refreshEvents();
        }
    }
}

function renderEvents() {
    const module = $('#experimentModule').value;
    const level = $('#experimentLevel').value;
    const matches = [...events.values()].filter(event => (!module || event.module === module)
        && (!level || event.level === level) && (!activeAlert || activeAlert === event.alert_id));
    $('#experimentEventStatus').textContent = matches.length ? `${matches.length} 条事件` : selectedId ? '当前筛选没有事件' : '选择会话后查看事件';
    $('#experimentOlderEvents').hidden = visibleEvents >= matches.length;
    const alertStarts = new Map([...events.values()].filter(event => event.type === 'alert_open').map(event => [event.alert_id, event]));
    const rows = matches.slice(-visibleEvents).reverse().map(event => {
        const row = element('li', 'experiment-event');
        row.dataset.level = event.level || 'info';
        const start = alertStarts.get(event.alert_id);
        const duration = event.type === 'alert_close' && hasTime(start?.t_ms) && hasTime(event.t_ms)
            ? ` · 持续 ${number(Math.max(0, event.t_ms - start.t_ms))} ms` : '';
        row.append(element('time', '', `${hasTime(event.t_ms) ? number(event.t_ms) + ' ms' : event.utc || '—'}`));
        const text = element('div');
        text.append(element('strong', '', `${MODULES[event.module] || event.module} · ${event.message || event.code || event.type}`));
        text.append(element('small', '', `${LEVELS[event.level] || event.level || ''} · ${[event.code, event.event_id, event.alert_id, event.sample_id, event.request_id].filter(Boolean).join(' · ')}${duration}`));
        row.append(text);
        if (hasTime(event.t_ms)) row.append(button('定位录像', () => {
            $('#experimentTime').value = event.t_ms;
            locate(selectedId, { t_ms: event.t_ms }, '实验时刻');
        }));
        return row;
    });
    $('#experimentEvents').replaceChildren(...rows);
}

function renderResult(root, data, source, retry) {
    const message = element('p', 'experiment-status', `${MATCH_LABELS[data.match] || data.match} · ${source}`);
    const contents = [message];
    if (data.frame) {
        const frame = data.frame;
        const clock = { host_receive: '船端接收时间', host_decode: '船端解码时间', device_mapped: '设备已对时', unknown: '时间未知' };
        const resolution = frame.identity === 'exact' ? '帧身份可证' : '帧身份未知';
        contents.push(element('p', 'experiment-frame', [frame.clip_id, frame.display_no != null ? `显示帧 ${frame.display_no}` : null,
            frame.frame_id, hasTime(frame.t_ms) ? `实验 ${number(frame.t_ms)} ms` : null, clock[frame.time_kind] || frame.time_kind, resolution].filter(Boolean).join(' · ')));
    }
    if (data.match === 'ambiguous') {
        const candidates = element('div', 'experiment-tools');
        for (const id of data.candidate_clip_ids || data.clip_ids || []) candidates.append(button('录像段 ' + id, () => retry({ clip_id: id })));
        for (const display of data.candidate_display_nos || data.display_nos || []) candidates.append(button('显示帧 ' + display, () => retry({ display_no: display })));
        contents.push(candidates);
    }
    if (data.state) {
        const fields = element('dl', 'experiment-state-fields');
        for (const [key, [label, unit]] of Object.entries(STATE_FIELDS)) {
            if (!Object.hasOwn(data.state, key)) continue;
            const wrap = element('div');
            wrap.append(element('dt', '', label), element('dd', '', measurement(data.state[key], unit, key)));
            fields.append(wrap);
        }
        const evidence = Object.entries(data.state).filter(([key, value]) => key.endsWith('_sample_id') && value).map(([key, value]) => `${key} ${value}`);
        contents.push(fields);
        if (evidence.length) contents.push(element('p', 'experiment-frame', evidence.join(' · ')));
    }
    root.replaceChildren(...contents);
}

async function playClip(frame, id, source, result) {
    if (!frame?.stream_url) throw new Error('该录像段没有可用媒体');
    const controller = locateAbort;
    openingMedia = controller;
    try {
        const video = await preview.openExperimentRecording(frame, controller.signal);
        if (!video || controller.signal.aborted) return;
    } finally {
        if (openingMedia === controller) openingMedia = null;
    }
    selectedClipContext = { id, frame };
    playbackRoot.hidden = false;
    playbackRoot.replaceChildren();
    const tools = element('div', 'experiment-tools');
    tools.append(button('按当前画面反查', reverseLocate), button('实验历史', () => openHistory()));
    playbackRoot.append(tools, element('div', '', ''));
    if (result) renderResult(playbackRoot.lastElementChild, result, source);
    if (dialog.open) dialog.close();
}

async function locate(id, query, source, reverse = false) {
    if (!id || !transport?.experiments) return;
    locateAbort?.abort();
    const controller = new AbortController();
    locateAbort = controller;
    const version = transportVersion;
    const root = reverse ? playbackRoot.lastElementChild : $('#experimentHistoryResult');
    root.textContent = '正在查找定位记录…';
    try {
        const result = await transport.experiments.locate(id, query);
        if (controller.signal.aborted || version !== transportVersion) return;
        renderResult(root, result, source, patch => {
            const next = patch.display_no != null ? { clip_id: query.clip_id || result.clip_id || result.frame?.clip_id, display_no: patch.display_no } : { ...query, ...patch };
            locate(id, next, source, reverse);
        });
        if (!reverse && result.match === 'matched' && result.frame?.stream_url) await playClip(result.frame, id, source, result);
    } catch (error) {
        if (controller.signal.aborted) return;
        if (!reverse && !dialog.open) dialog.showModal();
        root.textContent = error.code === 'INDEX_BUILDING' ? '录像索引正在构建，请稍后重试' : error.message;
    }
}

function reverseLocate() {
    const media = preview.selectedExperimentMedia();
    const video = media?.video;
    if (!video || video.readyState < 1) return;
    const names = new Set([media.filename, preview.recordingFilename(media.url)].filter(Boolean));
    const isMedia = clip => [clip.file_name, clip.filename, preview.recordingFilename(clip.stream_url)].some(name => names.has(name));
    const match = sessionClips().find(isMedia);
    const current = selectedClipContext?.frame;
    const remembered = current && isMedia(current);
    const clip = match || (remembered ? current : null);
    const id = match ? selectedId : selectedClipContext?.id;
    if (!clip || !id) { playbackRoot.lastElementChild.textContent = '这段录像不在所选会话索引中，请先在实验历史选择对应会话'; return; }
    const actual = presentedFrame?.source === video.currentSrc && !video.seeking;
    const playUs = Math.round((actual ? presentedFrame.time : video.currentTime) * 1e6);
    if (!Number.isSafeInteger(playUs) || playUs < 0) return;
    locate(id, { clip_id: clip.clip_id, play_us: playUs }, actual ? '播放器已呈现媒体时间' : '播放器位置（帧身份待核）', true);
}

function observeVideo() {
    const video = preview.selectedExperimentMedia()?.video;
    if (!video) return;
    for (const type of ['seeking', 'emptied', 'loadedmetadata']) video.addEventListener(type, () => { presentedFrame = null; });
    if (video.requestVideoFrameCallback) {
        const read = (_now, metadata) => {
            presentedFrame = { source: video.currentSrc, time: metadata.mediaTime };
            video.requestVideoFrameCallback(read);
        };
        video.requestVideoFrameCallback(read);
    }
    video.addEventListener('pause', () => {
        if (!openingMedia && !playbackRoot.hidden && preview.state.view === 'media') reverseLocate();
    });
}

async function downloadFile(kind, name, node) {
    if (!selectedId || !transport?.experiments) return;
    node.disabled = true;
    try {
        const blob = await transport.experiments.download(selectedId, kind);
        const href = URL.createObjectURL(blob);
        const anchor = element('a'); anchor.href = href; anchor.download = name; anchor.click();
        setTimeout(() => URL.revokeObjectURL(href), 1000);
    } catch (error) { $('#experimentHistoryStatus').textContent = error.message; }
    finally { node.disabled = false; }
}

for (const trigger of document.querySelectorAll('[data-open-experiments]')) trigger.addEventListener('click', () => openHistory());
$('[data-exp-close]').addEventListener('click', () => dialog.close());
$('#experimentSessions').addEventListener('change', event => { activeAlert = null; $('#experimentClearAlert').hidden = true; selectSession(event.target.value); });
$('#experimentMoreSessions').addEventListener('click', () => loadSessions(true));
$('#experimentRefresh').addEventListener('click', () => { loadSessions(); loadManifest(); refreshEvents(); });
$('#experimentModule').addEventListener('change', renderEvents);
$('#experimentControlModule').addEventListener('change', renderLive);
$('#experimentLevel').addEventListener('change', renderEvents);
$('#experimentClearAlert').addEventListener('click', event => { activeAlert = null; event.currentTarget.hidden = true; renderEvents(); });
$('#experimentOlderEvents').addEventListener('click', () => { visibleEvents += 100; renderEvents(); });
$('#experimentClips').addEventListener('change', renderClipAction);
$('#experimentTimeForm').addEventListener('submit', event => {
    event.preventDefault();
    const time = $('#experimentTime').valueAsNumber;
    if (hasTime(time)) locate(selectedId, { t_ms: time }, '实验时刻');
});
$('#experimentPlayClip').addEventListener('click', async () => {
    const clip = sessionClips().find(item => item.clip_id === $('#experimentClips').value);
    if (!clip) return;
    locateAbort?.abort(); locateAbort = new AbortController();
    try { await playClip(clip, selectedId, '录像段'); }
    catch (error) { $('#experimentHistoryStatus').textContent = error.message; }
});
for (const action of document.querySelectorAll('[data-record-action]')) {
    action.addEventListener('click', async () => {
        if (statusBusy || !transport?.experiments || (simulated ? transport.experimentReadOnly : !transport.canControl)) return;
        statusBusy = true; renderLive();
        const note = $('#experimentRecordingNote');
        note.textContent = '正在处理记录请求…';
        try { await transport.experiments[action.dataset.recordAction](); loadSessions(); }
        catch (error) { $('#experimentHistoryStatus').textContent = error.message; }
        finally { statusBusy = false; renderLive(); }
    });
}

window.MantaExperimentPanel = { bind, open: openHistory };
bind(preview.getTransport());
observeVideo();
renderManifest();
