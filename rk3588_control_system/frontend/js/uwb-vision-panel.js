import { FUSION_DEFAULTS, evaluateUwbVision, validateFusionConfig } from './uwb-vision-core.js';

const host = document.getElementById('uwbVisionModule');
if (host) mountFusionPanel(host);

function mountFusionPanel(host) {
    const statuses = { disabled: '未启用', waiting: '等待有效测距', uwb_only: '使用 UWB 方位', vision_only: '仅视觉方位 · 等待测距', fused: '融合方位有效', conflict: '方位冲突 · 回退 UWB' };
    const reasons = { disabled: '船体使用 UWB 距离与方位。选择融合定位后可用泳者视觉修正方位。', tracker_inactive: 'YOLO 泳者追踪未运行，请先启动 YOLO。',
        vision_unavailable: '未收到可信视觉检测，使用有效 UWB 定位；预测帧不参与融合。', pose_unavailable: '云台角度不可用或已过期，等待有效反馈。',
        zoom_changed: '检测到相机变焦，请恢复 1× 视角并核对标定。', pose_unsynchronized: '相机检测与云台角度不同步，等待下一帧。',
        calibration_required: '请核对相机视场角与方向标定，确认后才能融合。', uwb_unavailable: 'UWB 不可用，仅显示视觉方位；暂时无法确定目标距离。',
        unsynchronized: '两路测量的时间差过大，暂时使用 UWB 定位。', bearing_conflict: '视觉与 UWB 方位差过大，拒绝融合，请核对目标和标定。', tracking: 'UWB 距离 + YOLO 视觉方位，同步追踪同一目标。' };
    host.innerHTML = `<div class="uv-controls">
      <label for="followPositionSource">跟随定位</label>
      <select id="followPositionSource" disabled><option value="uwb">仅 UWB</option><option value="fusion">UWB＋视觉融合</option></select>
      <button class="uv-btn" type="button" data-action="yolo" hidden>启动泳者跟踪</button>
      <span class="uv-pill" data-uv="status" role="status">等待设备</span></div>
    <p class="uv-guidance" data-uv="guidance" hidden></p>
    <p class="uv-message" data-uv="message" role="status"></p>
    <details id="followFusionOptions"><summary>融合详情与标定</summary>
    <div class="uv-content">
      <p class="uv-state-note" data-uv="reason"></p>
      <span class="uv-note" data-uv="mode"></span>
      <div class="uv-metrics"><div><small>目标距离</small><strong data-uv="distance">-- m</strong></div><div><small>融合方位</small><strong data-uv="bearing">--°</strong></div><div><small>前向 X</small><strong data-uv="x">-- m</strong></div><div><small>左向 Y</small><strong data-uv="y">-- m</strong></div></div>
      <div class="uv-sources"><article class="uv-source"><header>UWB <span data-uv="uwb-health">等待</span></header><p>距离 <b data-uv="uwb-distance">--</b> · 方位 <b data-uv="uwb-bearing">--</b><br>测量年龄 <b data-uv="uwb-age">--</b></p></article><article class="uv-source"><header>YOLO <span data-uv="vision-health">等待</span></header><p>置信度 <b data-uv="confidence">--</b> · ID <b data-uv="track-id">--</b><br>测量年龄 <b data-uv="vision-age">--</b></p></article></div>

      <p class="uv-note">视觉丢失时使用仍有效的 UWB；UWB 丢失时仅显示视觉方位。两个传感器需指向同一名佩戴信标的泳者。</p>
      <section class="uv-radar-panel"><div class="uv-section-head"><strong>相对目标位置</strong><span data-uv="radar-range">量程 10 m</span></div><canvas class="uv-radar" data-uv="radar" aria-label="UWB、视觉方位与融合目标雷达图" role="img"></canvas><div class="uv-legend"><span>● UWB</span><span>┄ 视觉方位</span><span>● 融合位置</span></div></section>
      <div class="uv-section-head"><span>测量时间差 <b data-uv="skew">--</b></span><span>视觉贡献 <b data-uv="weight">--</b></span></div>
      <details class="uv-config"><summary>融合配置与相机标定</summary><form class="uv-form">
        <div class="uv-fields">
          <label>水平视场角 / °<input name="horizontalFovDeg" type="number" min="5" max="160" step="0.1" required></label>
          <label>云台零点偏移 / °<input name="yawOffsetDeg" type="number" min="-180" max="180" step="0.1" required></label>
          <label>云台角度正方向<select name="yawSign"><option value="1">左转为正 (+1)</option><option value="-1">右转为正 (−1)</option></select></label>
          <label>原始检测画面方向<select name="imageSign"><option value="-1">画面右侧 = 船体右侧</option><option value="1">画面已水平镜像</option></select></label>
          <label>最低视觉置信度<input name="minConfidence" type="number" min="0.05" max="1" step="0.05" required></label>
          <label>视觉融合权重<input name="visionWeight" type="number" min="0" max="1" step="0.05" required></label>
          <label>最大方位差 / °<input name="maxBearingDifferenceDeg" type="number" min="1" max="90" step="1" required></label>
          <label>最大时间差 / ms<input name="maxSkewMs" type="number" min="20" max="1000" step="10" required></label>
        </div><label class="uv-check"><input name="calibrated" type="checkbox"><span>已用同一目标核对 1× 视场角、云台方向和零点。设备默认值仅为起始参数。</span></label>
        <button class="uv-btn" type="submit">保存融合配置</button>
      </form></details>
      <div data-uv="demo-controls" hidden><label class="uv-check">演示场景<select data-uv="scenario" class="uv-btn"><option value="normal">两路正常</option><option value="vision-lost">视觉丢失</option><option value="uwb-lost">UWB 丢失</option><option value="conflict">目标方位冲突</option><option value="stale">两路过期</option></select></label></div>
      <p class="uv-foot"><span data-uv="model"></span> · <span data-uv="clock"></span></p></div></details>`;
    const details = host.querySelector('#followFusionOptions');
    const source = host.querySelector('#followPositionSource');
    const form = host.querySelector('form');
    const yolo = host.querySelector('[data-action="yolo"]');
    const find = key => host.querySelector(`[data-uv="${key}"]`);
    const text = (key, value) => { find(key).textContent = value; };
    const format = (value, suffix = '', digits = 1) => Number.isFinite(value) ? value.toFixed(digits) + suffix : '--' + suffix;
    let readError = '';
    let payload, busy = false, dirty = false, timer, disposed = false, pollGeneration = 0;
    let mode, hasConfig = false, selected = false, enabling = false, available = false;
    let demoOwnsTracker = false, pendingAction;
    let lastConnection = false, pendingOpen = new URLSearchParams(location.search).get('module') === 'uwb-vision';
    let demoConfig = { ...FUSION_DEFAULTS, calibrated: true }, demoEnabled = false;
    function bridge() { return window.MantaPreview; }
    function transport() { return bridge()?.getTransport?.(); }
    function isDemo() { return bridge()?.state.transportMode === 'mock'; }
    function canControl() {
        const current = transport();
        return current?.connected && (isDemo() ? !current.experimentReadOnly : current.canControl);
    }
    function usesGimbal() { return selected || enabling; }
    function applyCameraMode() {
        const occupied = usesGimbal();
        const screen = document.getElementById('controlScreen');
        const previous = screen.classList.contains('fusion-camera-active');
        screen.classList.toggle('fusion-camera-active', occupied);
        screen.querySelector('.fusion-camera-label').hidden = !occupied;
        if (previous !== occupied) window.dispatchEvent(new Event('manta-follow-fusion-change'));
    }
    function open() {
        if (!bridge()?.state.connected) { pendingOpen = true; return; }
        bridge().showFollowView();
        details.open = true;
        details.querySelector('summary').focus();
    }
    async function setSource(enable) {
        enabling = enable; applyCameraMode();
        try {
            if (isDemo()) {
                const camera = bridge().state;
                if ((enable && camera.trackingActive && camera.runningTrackMode !== 'swimmer')
                    || (!enable && demoOwnsTracker && camera.runningTrackMode === 'swimmer')) {
                    await transport().gimbal('stop');
                }
                if (!enable) demoOwnsTracker = false;
                demoEnabled = enable;
                payload = demo();
            } else {
                payload = await request('/api/uwb-vision/follow-source', { source: enable ? 'uwb+vision' : 'uwb' });
            }
            selected = payload.followSource === 'uwb+vision';
            if (!payload.config.calibrated && selected) details.open = true;
        } finally { enabling = false; applyCameraMode(); }
    }
    async function release() {
        await pendingAction;
        if (!usesGimbal()) return;
        if (!canControl()) throw new Error('切回手动前请先获取控制权');
        clearTimeout(timer); pollGeneration++;
        if (busy) throw new Error('定位源正在切换，请稍后重试');
        busy = true; render();
        try { await setSource(false); }
        finally { busy = false; render(); clearTimeout(timer); poll(); }
    }
    window.MantaFollowFusion = { open, usesGimbal, release };
    function message(value) { text('message', value); }
    async function request(url, body) {
        const fetcher = window.MantaAppTransport?.request || window.fetch.bind(window);
        const response = await fetcher(url, { credentials: 'same-origin', cache: 'no-store', signal: AbortSignal.timeout(4000),
            ...(body === undefined ? {} : { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) }) });
        const result = await response.json();
        if (!response.ok || result.success === false) throw new Error(result.message || `请求失败 (${response.status})`);
        return result;
    }
    function demo() {
        const now = Date.now(), phase = now / 2300, bearing = 12 + Math.sin(phase) * 7;
        const distanceM = 5.8 + Math.sin(phase / 2) * 0.5;
        const scene = find('scenario').value;
        const angle = bearing + (scene === 'conflict' ? 55 : Math.sin(phase * 2) * 2);
        const target = { mode: 'swimmer', id: 7, status: 'track', locked: true, conf: 0.91, coasting: false,
            frame_w: 1920, frame_h: 1080, x: 960, y: 540, w: 250, h: 180, timestamp: now - 50 };
        if (scene === 'vision-lost') target.locked = false;
        if (scene === 'stale') target.timestamp = now - 5000;
        const uwb = { online: scene !== 'uwb-lost', fresh: true, measurementStatus: 'trusted',
            measurementUpdatedAt: scene === 'stale' ? now - 5000 : now - 70, azimuthDeg: bearing, distanceM };
        return { success: true, followSource: demoEnabled ? 'uwb+vision' : 'uwb', model: 'YOLO / 演示模型（无真实推理）', config: demoConfig, state: evaluateUwbVision({ now,
            enabled: demoEnabled, config: demoConfig, uwb, target, trackerActive: bridge()?.state.trackingActive && bridge()?.state.runningTrackMode === 'swimmer',
            pose: { yawDeg: angle, updatedAt: now - 40, checksumValid: true, digitalZoom: 1 } }) };
    }
    function fillConfig(config) {
        for (const node of form.elements) {
            if (!(node.name in config)) continue;
            if (node.type === 'checkbox') node.checked = config[node.name]; else node.value = config[node.name];
        }
    }
    function render() {
        const state = payload?.state, config = payload?.config;
        if (!state) return;
        const demoMode = isDemo();
        text('status', !available ? '状态未知' : !selected ? 'UWB 定位' : statuses[state.status] || '等待数据');
        find('status').dataset.status = available ? state.status : 'unknown';
        const follow = bridge()?.state.uwbFollow;
        const guidance = { 'uwb+vision': '采用融合方位', uwb: '采用 UWB 方位', unavailable: '等待有效测距' };
        const actual = !demoMode && selected && available && bridge().followRequested()
            && follow?.followSource === 'uwb+vision' && guidance[follow.guidanceSource];
        find('guidance').hidden = !actual;
        text('guidance', actual ? '当前船控 · ' + actual : '');
        text('mode', demoMode ? '演示数据 · 不连接硬件' : '设备实时数据');
        text('distance', format(state.result?.distanceM, ' m', 2)); text('bearing', format(state.result?.bearingDeg, '°'));
        text('x', format(state.result?.xM, ' m', 2)); text('y', format(state.result?.yM, ' m', 2));
        text('confidence', state.vision.confidence === null ? '--' : Math.round(state.vision.confidence * 100) + '%');
        text('uwb-distance', format(state.uwb.distanceM, ' m', 2)); text('uwb-bearing', format(state.uwb.bearingDeg, '°'));
        text('uwb-age', format(state.uwb.ageMs, ' ms', 0)); text('vision-age', format(state.vision.ageMs, ' ms', 0));
        text('uwb-health', state.uwb.valid ? '● 有效' : '○ 不可用'); text('vision-health', state.vision.valid ? '● 有效' : '○ 不可用');
        text('track-id', state.vision.trackId ?? '--'); text('skew', format(state.skewMs, ' ms', 0));
        text('weight', state.status === 'fused' ? Math.round(state.visionContribution * 100) + '%' : '--');
        text('reason', reasons[state.reason] || '等待有效数据');
        find('reason').dataset.error = String(state.status === 'conflict');
        text('model', '模型：' + payload.model);
        text('clock', new Date(state.evaluatedAt).toLocaleTimeString('zh-CN') + (demoMode ? ' · 模拟' : ' · 设备更新'));
        source.value = usesGimbal() ? 'fusion' : 'uwb';
        source.disabled = busy || !available || !canControl() || bridge().followRequested();
        source.title = bridge().followRequested() ? '停止船体跟随后可切换定位' : '';
        const swimmerActive = bridge()?.state.trackingActive && bridge()?.state.runningTrackMode === 'swimmer';
        yolo.hidden = !usesGimbal();
        yolo.textContent = swimmerActive ? '泳者跟踪运行中' : '启动泳者跟踪';
        yolo.disabled = busy || !available || !canControl() || swimmerActive || !selected;
        for (const field of form.elements) field.disabled = busy || !available || !canControl();
        find('demo-controls').hidden = !demoMode;
        if (!dirty && !hasConfig && available) { fillConfig(config); hasConfig = true; }
        applyCameraMode();
        if (details.open && !host.hidden && host.offsetWidth) drawRadar(state);
    }
    function canvasContext(canvas) {
        const rect = canvas.getBoundingClientRect(), ratio = Math.min(devicePixelRatio || 1, 2);
        const width = Math.max(1, rect.width), height = Math.max(1, rect.height);
        canvas.width = width * ratio; canvas.height = height * ratio;
        const ctx = canvas.getContext('2d'); ctx.scale(ratio, ratio);
        return { ctx, width, height };
    }
    function drawRadar(state) {
        const { ctx, width, height } = canvasContext(find('radar'));
        const cx = width / 2, cy = height / 2, radius = Math.max(1, Math.min(width / 2 - 38, height / 2 - 22));
        const range = Math.max(10, Math.ceil((state.uwb.distanceM || 0) / 5) * 5);
        text('radar-range', '量程 ' + range + ' m');
        ctx.strokeStyle = '#29434d'; ctx.lineWidth = 1; ctx.fillStyle = '#738e9e'; ctx.font = '9px monospace';
        for (let ring = 1; ring <= 3; ring++) { ctx.beginPath(); ctx.arc(cx, cy, radius * ring / 3, 0, Math.PI * 2); ctx.stroke(); }
        ctx.beginPath(); ctx.moveTo(cx - radius, cy); ctx.lineTo(cx + radius, cy); ctx.moveTo(cx, cy - radius); ctx.lineTo(cx, cy + radius); ctx.stroke();
        ctx.fillText('前 +X', cx - 15, cy - radius - 7); ctx.fillText('左 +Y', cx - radius - 32, cy + 3); ctx.fillText('右 −Y', cx + radius + 4, cy + 3);
        function point(bearing, distance, color) {
            const radians = bearing * Math.PI / 180, r = distance / range * radius;
            const x = cx - Math.sin(radians) * r, y = cy - Math.cos(radians) * r;
            ctx.strokeStyle = color; ctx.fillStyle = color; ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(x, y); ctx.stroke();
            ctx.beginPath(); ctx.arc(x, y, 4, 0, Math.PI * 2); ctx.fill();
        }
        if (state.vision.valid) { ctx.setLineDash([3, 5]); point(state.vision.bearingDeg, range, '#c9b6ff88'); ctx.setLineDash([]); }
        if (state.uwb.valid) point(state.uwb.bearingDeg, state.uwb.distanceM, '#7dbce9');
        if (state.result?.source === 'uwb+vision') point(state.result.bearingDeg, state.result.distanceM, '#77e3ba');
        ctx.fillStyle = '#d5ede5'; ctx.beginPath(); ctx.moveTo(cx, cy - 6); ctx.lineTo(cx + 4, cy + 5); ctx.lineTo(cx - 4, cy + 5); ctx.closePath(); ctx.fill();
    }
    async function poll() {
        if (disposed) return;
        const generation = ++pollGeneration;
        const connected = Boolean(transport()?.connected);
        const nextMode = isDemo() ? 'demo' : 'live';
        if (nextMode !== mode || connected !== lastConnection) {
            mode = nextMode; lastConnection = connected; dirty = false; hasConfig = false;
            if (!connected) { demoOwnsTracker = false; demoEnabled = false; selected = false; details.open = false; }
            message('');
        }
        try {
            const next = !connected ? { model: '等待设备', config: FUSION_DEFAULTS, state: evaluateUwbVision() }
                : isDemo() ? demo() : await request('/api/uwb-vision/state');
            if (generation !== pollGeneration || connected !== Boolean(transport()?.connected)) return;
            if (readError && find('message').textContent === readError) message('');
            readError = '';
            available = connected;
            const entering = !selected && next.followSource === 'uwb+vision';
            selected = connected && next.followSource === 'uwb+vision';
            payload = next;
            if (entering && bridge()?.state.connected && bridge().state.driveMode !== 'auto') bridge().showFollowView();
            if (pendingOpen && connected && bridge()?.state.connected) { pendingOpen = false; open(); }
            render();
        } catch (error) {
            if (generation !== pollGeneration) return;
            available = false;
            payload = { model: '设备暂不可用', config: payload?.config || FUSION_DEFAULTS, state: evaluateUwbVision() };
            readError = '设备数据读取失败：' + error.message;
            render(); message(readError);
        } finally {
            if (!disposed && generation === pollGeneration) { clearTimeout(timer); timer = setTimeout(poll, host.hidden || !details.open ? 900 : 250); }
        }
    }
    async function action(work) {
        if (busy) return;
        clearTimeout(timer); pollGeneration++;
        busy = true; message(''); render();
        try { pendingAction = work(); await pendingAction; } catch (error) { message(error.message); }
        finally { busy = false; pendingAction = null; render(); clearTimeout(timer); poll(); }
    }
    details.addEventListener('toggle', () => { if (!busy) { clearTimeout(timer); poll(); } });
    source.addEventListener('change', () => {
        const enable = source.value === 'fusion';
        if (!canControl() || !available || busy || bridge().followRequested()) { render(); return; }
        action(() => setSource(enable));
    });
    yolo.addEventListener('click', () => action(async () => {
        if (!canControl() || !selected) throw new Error('请先获取控制权并选择融合定位');
        await transport().gimbal('swimmer', { followFusion: true });
        if (isDemo()) demoOwnsTracker = true;
    }));
    form.addEventListener('input', () => { dirty = true; });
    form.addEventListener('submit', event => { event.preventDefault(); action(async () => {
        if (!available || !canControl()) throw new Error('请先在主控制网页配对并获取控制权');
        const next = {};
        for (const field of form.elements) if (field.name) next[field.name] = field.type === 'checkbox' ? field.checked : Number(field.value);
        validateFusionConfig(next);
        if (isDemo()) { demoConfig = { ...demoConfig, ...next }; payload = demo(); }
        else payload = await request('/api/uwb-vision/config', next);
        dirty = false; hasConfig = false; message(isDemo() ? '演示配置已更新，不会写入设备。' : '融合配置已保存到设备。');
    }); });
    find('scenario').addEventListener('change', () => { clearTimeout(timer); poll(); });
    window.addEventListener('resize', () => { if (details.open && !host.hidden && host.offsetWidth && payload) drawRadar(payload.state); });
    window.addEventListener('pagehide', () => { disposed = true; pollGeneration++; clearTimeout(timer); });
    window.addEventListener('pageshow', event => { if (event.persisted) { disposed = false; poll(); } });
    poll();
}
