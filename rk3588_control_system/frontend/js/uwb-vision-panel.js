import { FUSION_DEFAULTS, evaluateUwbVision, validateFusionConfig } from './uwb-vision-core.js';

const host = document.getElementById('uwbVisionModule');
if (host) mountFusionPanel(host);

function mountFusionPanel(host) {
    const statuses = { disabled: '未启用', waiting: '等待数据', uwb_only: 'UWB 定位', vision_only: '仅视觉方位', fused: '融合追踪中', conflict: '目标方位不一致' };
    const reasons = { disabled: '启用后，将同步校验 UWB 与 YOLO 泳者检测结果。', tracker_inactive: 'YOLO 泳者追踪未运行，请先启动 YOLO。',
        vision_unavailable: '未收到可信视觉检测，使用有效 UWB 定位；预测帧不参与融合。', pose_unavailable: '云台角度不可用或已过期，等待有效反馈。',
        zoom_changed: '检测到相机变焦，请恢复 1× 视角并核对标定。', pose_unsynchronized: '相机检测与云台角度不同步，等待下一帧。',
        calibration_required: '请核对相机视场角与方向标定，确认后才能融合。', uwb_unavailable: 'UWB 不可用，仅显示视觉方位；暂时无法确定目标距离。',
        unsynchronized: '两路测量的时间差过大，暂时使用 UWB 定位。', bearing_conflict: '视觉与 UWB 方位差过大，拒绝融合，请核对目标和标定。', tracking: 'UWB 距离 + YOLO 视觉方位，同步追踪同一目标。' };
    host.innerHTML = `<div class="uv-card">
        <div class="uv-identity"><span class="uv-mark" aria-hidden="true">◎</span><div><strong>UWB+视觉追踪</strong><small>YOLO SENSOR FUSION</small></div></div>
        <span class="uv-pill" data-uv="status">未启用</span><span class="uv-mode" data-uv="mode">设备数据</span>
        <div class="uv-mini"><div><small>距离</small><strong data-uv="distance">-- m</strong></div><div><small>方位</small><strong data-uv="bearing">--°</strong></div><div><small>视觉置信度</small><strong data-uv="confidence">--</strong></div></div>
        <button class="uv-btn" type="button" id="openUwbVision">打开融合窗口 ↗</button></div>`;
    const dialog = document.createElement('dialog');
    dialog.id = 'uwbVisionDialog'; dialog.className = 'uv-dialog'; dialog.setAttribute('aria-labelledby', 'uvTitle');
    dialog.innerHTML = `<header class="uv-head"><div><div class="uv-kicker">MANTA / SENSOR FUSION</div><h2 id="uvTitle">UWB+视觉追踪</h2><p>测距与视觉协同定位 · YOLO 泳者追踪</p></div><button class="uv-btn uv-close" type="button" data-action="close" aria-label="关闭融合窗口">×</button></header>
    <div class="uv-content"><div class="uv-sourcebar"><span class="uv-pill" data-uv="status">未启用</span><span class="uv-mode" data-uv="mode">设备数据</span><button class="uv-btn" type="button" data-action="demo">查看演示</button><button class="uv-btn uv-btn-primary" type="button" data-action="toggle">启用融合</button></div>
    <div class="uv-body"><div class="uv-visuals">
      <div class="uv-video"><div class="uv-empty" data-uv="empty"><strong>YOLO × UWB</strong><span>等待相机画面</span></div><video data-uv="demo-video" class="uv-hidden" muted playsinline loop></video><img data-uv="live-video" class="uv-hidden" alt="云台相机实时画面"><canvas class="uv-overlay" data-uv="overlay"></canvas><span class="uv-video-tag" data-uv="video-caption">实时画面 · 检测框</span></div>
      <div class="uv-metrics"><div><small>目标距离</small><strong data-uv="distance">-- m</strong></div><div><small>融合方位</small><strong data-uv="bearing">--°</strong></div><div><small>前向 X</small><strong data-uv="x">-- m</strong></div><div><small>左向 Y</small><strong data-uv="y">-- m</strong></div></div>
      <div class="uv-sources"><article class="uv-source"><header>UWB <span data-uv="uwb-health">等待</span></header><p>距离 <b data-uv="uwb-distance">--</b> · 方位 <b data-uv="uwb-bearing">--</b><br>测量年龄 <b data-uv="uwb-age">--</b></p></article><article class="uv-source"><header>YOLO <span data-uv="vision-health">等待</span></header><p>置信度 <b data-uv="confidence">--</b> · ID <b data-uv="track-id">--</b><br>测量年龄 <b data-uv="vision-age">--</b></p></article></div>
      <div class="uv-state-note" data-uv="reason" role="status"></div>
      <p class="uv-note">视觉丢失时使用仍有效的 UWB；UWB 丢失时仅显示视觉方位。两个传感器需指向同一名佩戴信标的泳者。</p>
    </div><div>
      <section class="uv-radar-panel"><div class="uv-section-head"><strong>相对目标位置</strong><span data-uv="radar-range">量程 10 m</span></div><canvas class="uv-radar" data-uv="radar" aria-label="UWB、视觉方位与融合目标雷达图" role="img"></canvas><div class="uv-legend"><span>● UWB</span><span>┄ 视觉方位</span><span>● 融合位置</span></div></section>
      <div class="uv-section-head"><span>测量时间差 <b data-uv="skew">--</b></span><span>视觉贡献 <b data-uv="weight">--</b></span></div>
      <div class="uv-actions"><button class="uv-btn" type="button" data-action="yolo">启动 YOLO 泳者追踪</button></div>
      <p class="uv-model" data-uv="model">模型：等待设备连接</p>
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
        <button class="uv-btn" type="submit">保存融合配置</button><p class="uv-note">启用融合只计算定位。启动 YOLO 会控制云台追踪，船体推进仍由原有移动模块控制。</p>
      </form></details>
      <div data-uv="demo-controls" class="uv-hidden"><label class="uv-check">演示场景<select data-uv="scenario" class="uv-btn"><option value="normal">两路正常</option><option value="vision-lost">视觉丢失</option><option value="uwb-lost">UWB 丢失</option><option value="conflict">目标方位冲突</option><option value="stale">两路过期</option></select></label></div>
      <p class="uv-message" data-uv="message" role="status" aria-live="polite"></p>
    </div></div></div><footer class="uv-footer"><span>坐标系：X 向前 · Y 向左 · 方位左正右负</span><span data-uv="clock">等待数据</span></footer>`;
    document.body.appendChild(dialog);
    const all = selector => [...host.querySelectorAll(selector), ...dialog.querySelectorAll(selector)];
    const find = key => dialog.querySelector(`[data-uv="${key}"]`);
    const text = (key, value) => all(`[data-uv="${key}"]`).forEach(node => { node.textContent = value; });
    const form = dialog.querySelector('form');
    const format = (value, suffix = '', digits = 1) => Number.isFinite(value) ? value.toFixed(digits) + suffix : '--' + suffix;
    let forcedDemo = false, mode = null, payload = null, busy = false, dirty = false, timer = null, disposed = false;
    let pollGeneration = 0, hasConfig = false, videoKey = '', refreshVideo = false;
    let demoConfig = { ...FUSION_DEFAULTS, calibrated: true }, demoEnabled = false;
    function bridge() { return window.MantaPreview; }
    function transport() { return bridge()?.getTransport?.(); }
    function isDemo() { return forcedDemo || bridge()?.state.transportMode === 'mock'; }
    function canControl() { return isDemo() || Boolean(transport()?.canControl && transport()?.connected); }
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
        return { success: true, model: 'YOLO / 演示模型（无真实推理）', config: demoConfig, state: evaluateUwbVision({ now,
            enabled: demoEnabled, config: demoConfig, uwb, target, trackerActive: true,
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
        text('status', statuses[state.status] || '等待数据');
        all('[data-uv="status"]').forEach(node => { node.dataset.status = state.status; });
        text('mode', demoMode ? '演示数据 · 不连接硬件' : '设备实时数据');
        text('distance', format(state.result?.distanceM, ' m', 2)); text('bearing', format(state.result?.bearingDeg, '°'));
        text('x', format(state.result?.xM, ' m', 2)); text('y', format(state.result?.yM, ' m', 2));
        text('confidence', state.vision.confidence === null ? '--' : Math.round(state.vision.confidence * 100) + '%');
        text('uwb-distance', format(state.uwb.distanceM, ' m', 2)); text('uwb-bearing', format(state.uwb.bearingDeg, '°'));
        text('uwb-age', format(state.uwb.ageMs, ' ms', 0)); text('vision-age', format(state.vision.ageMs, ' ms', 0));
        text('uwb-health', state.uwb.valid ? '● 有效' : '○ 不可用'); text('vision-health', state.vision.valid ? '● 有效' : '○ 不可用');
        text('track-id', state.vision.trackId ?? '--'); text('skew', format(state.skewMs, ' ms', 0));
        text('weight', state.status === 'fused' ? Math.round(state.visionContribution * 100) + '%' : '--');
        text('reason', state.status === 'waiting' && state.reason !== 'calibration_required'
            ? '两路定位数据不可用，等待新鲜测量；当前无目标位置。' : reasons[state.reason] || '等待有效数据');
        find('reason').dataset.error = String(state.status === 'conflict');
        text('model', '模型：' + payload.model);
        text('clock', new Date(state.evaluatedAt).toLocaleTimeString('zh-CN') + (demoMode ? ' · 模拟' : ' · 设备更新'));
        dialog.querySelector('[data-action="toggle"]').textContent = state.enabled ? '暂停融合' : '启用融合';
        dialog.querySelector('[data-action="demo"]').textContent = forcedDemo ? '返回设备数据' : demoMode ? '演示模式' : '查看演示';
        dialog.querySelector('[data-action="demo"]').disabled = busy || (!forcedDemo && bridge()?.state.transportMode === 'mock');
        dialog.querySelector('[data-action="yolo"]').disabled = busy || demoMode || !canControl();
        dialog.querySelector('[data-action="toggle"]').disabled = busy || !canControl();
        form.querySelector('[type="submit"]').disabled = busy || !canControl();
        find('demo-controls').classList.toggle('uv-hidden', !demoMode);
        if (!dirty && (!hasConfig || refreshVideo)) { fillConfig(config); hasConfig = true; }
        if (dialog.open) { updateVideo(); drawRadar(state); drawOverlay(state); }
    }
    function stopVideo() {
        const video = find('demo-video'), image = find('live-video');
        video.pause(); video.removeAttribute('src'); video.load(); image.removeAttribute('src');
        video.classList.add('uv-hidden'); image.classList.add('uv-hidden'); find('empty').hidden = false; videoKey = '';
    }
    function updateVideo() {
        const key = isDemo() ? 'demo' : transport()?.connected ? 'live' : 'offline';
        if (videoKey === key && !refreshVideo) return;
        stopVideo(); videoKey = key; refreshVideo = false;
        if (key === 'demo') {
            const video = find('demo-video'); video.src = 'assets/manta-app/gimbal-demo.mp4'; video.classList.remove('uv-hidden');
            video.onloadeddata = () => { find('empty').hidden = true; }; video.play().catch(() => {});
        } else if (key === 'live') {
            const image = find('live-video'); image.classList.remove('uv-hidden');
            image.onload = () => { find('empty').hidden = true; };
            image.onerror = () => { find('empty').hidden = false; };
            image.src = '/api/gimbal/stream?fusion=' + Date.now();
        }
        text('video-caption', key === 'demo' ? '示例视频 · 检测框为模拟数据' : '原始相机画面 · YOLO 检测框');
    }
    function canvasContext(canvas) {
        const rect = canvas.getBoundingClientRect(), ratio = Math.min(devicePixelRatio || 1, 2);
        const width = Math.max(1, rect.width), height = Math.max(1, rect.height);
        canvas.width = width * ratio; canvas.height = height * ratio;
        const ctx = canvas.getContext('2d'); ctx.scale(ratio, ratio);
        return { ctx, width, height };
    }
    function drawOverlay(state) {
        const { ctx, width, height } = canvasContext(find('overlay')), box = state.vision.box;
        if (!box) return;
        const scale = Math.min(width / box.frame_w, height / box.frame_h);
        const x = (width - box.frame_w * scale) / 2 + (box.x - box.w / 2) * scale;
        const y = (height - box.frame_h * scale) / 2 + (box.y - box.h / 2) * scale;
        ctx.strokeStyle = '#77e3ba'; ctx.lineWidth = 1.5; ctx.strokeRect(x, y, box.w * scale, box.h * scale);
        ctx.font = '11px monospace'; ctx.fillStyle = '#77e3ba'; ctx.fillText('YOLO #' + (state.vision.trackId ?? '--'), Math.max(4, x), Math.max(13, y - 6));
    }
    function drawRadar(state) {
        const { ctx, width, height } = canvasContext(find('radar'));
        const cx = width / 2, cy = height / 2, radius = Math.min(width / 2 - 38, height / 2 - 22);
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
        const nextMode = isDemo() ? 'demo' : 'live';
        if (nextMode !== mode) { mode = nextMode; payload = null; dirty = false; hasConfig = false; refreshVideo = true; message(''); }
        try {
            if (isDemo()) payload = demo();
            else if (transport()?.connected) {
                const next = await request('/api/uwb-vision/state');
                if (generation !== pollGeneration || isDemo()) return;
                payload = next;
            } else payload = { model: '请在主控制网页连接设备', config: FUSION_DEFAULTS, state: evaluateUwbVision() };
            if (generation === pollGeneration) render();
        } catch (error) {
            if (generation !== pollGeneration) return;
            payload = { model: '设备暂不可用', config: payload?.config || FUSION_DEFAULTS, state: evaluateUwbVision() };
            render(); message('设备数据读取失败：' + error.message);
        } finally {
            if (!disposed && generation === pollGeneration) { clearTimeout(timer); timer = setTimeout(poll, dialog.open ? 250 : 1000); }
        }
    }
    async function action(work) {
        if (busy) return;
        clearTimeout(timer); pollGeneration++;
        busy = true; message(''); render();
        try { await work(); } catch (error) { message(error.message); }
        finally { busy = false; render(); clearTimeout(timer); poll(); }
    }
    host.querySelector('button').addEventListener('click', () => { dialog.showModal(); refreshVideo = true; render(); clearTimeout(timer); poll(); });
    dialog.querySelector('[data-action="close"]').addEventListener('click', () => dialog.close());
    dialog.addEventListener('close', stopVideo);
    dialog.addEventListener('click', event => { if (event.target === dialog) { const r = dialog.getBoundingClientRect(); if (event.clientX < r.left || event.clientX > r.right || event.clientY < r.top || event.clientY > r.bottom) dialog.close(); } });
    dialog.querySelector('[data-action="demo"]').addEventListener('click', () => { forcedDemo = !forcedDemo; clearTimeout(timer); poll(); });
    dialog.querySelector('[data-action="toggle"]').addEventListener('click', () => action(async () => {
        if (!canControl()) throw new Error('请先在主控制网页配对并获取控制权');
        if (isDemo()) { demoEnabled = !demoEnabled; payload = demo(); }
        else payload = await request('/api/uwb-vision/enabled', { enabled: !payload.state.enabled });
    }));
    dialog.querySelector('[data-action="yolo"]').addEventListener('click', () => action(async () => {
        if (isDemo() || !canControl()) throw new Error('请先连接设备并获取控制权');
        await transport().gimbal('swimmer'); message('YOLO 泳者追踪已请求启动，等待检测结果。');
    }));
    form.addEventListener('input', () => { dirty = true; });
    form.addEventListener('submit', event => { event.preventDefault(); action(async () => {
        if (!canControl()) throw new Error('请先在主控制网页配对并获取控制权');
        const next = {};
        for (const field of form.elements) if (field.name) next[field.name] = field.type === 'checkbox' ? field.checked : Number(field.value);
        validateFusionConfig(next);
        if (isDemo()) { demoConfig = { ...demoConfig, ...next }; payload = demo(); }
        else payload = await request('/api/uwb-vision/config', next);
        dirty = false; hasConfig = false; message(isDemo() ? '演示配置已更新，不会写入设备。' : '融合配置已保存到设备。');
    }); });
    find('scenario').addEventListener('change', () => { clearTimeout(timer); poll(); });
    window.addEventListener('resize', () => { if (dialog.open && payload) { drawRadar(payload.state); drawOverlay(payload.state); } });
    window.addEventListener('pagehide', () => { disposed = true; pollGeneration++; clearTimeout(timer); stopVideo(); });
    window.addEventListener('pageshow', event => { if (event.persisted) { disposed = false; poll(); } });
    // Pairing stays in the host app. A link from the legacy dashboard highlights the module after connection.
    if (new URLSearchParams(location.search).get('module') === 'uwb-vision') host.querySelector('button').focus();
    poll();
}
