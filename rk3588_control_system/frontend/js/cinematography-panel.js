const endpoint = '/api/cinematography';
const unavailable = Object.freeze({ schema_version: 1, available: false, state: 'unavailable', mode: null,
    reason: '尚未接入自主运镜策略', modes: [], can_start: false, can_stop: false });
const labels = { unavailable: '尚未接入', idle: '待机', starting: '启动中', running: '运行中', stopping: '停止中', stopped: '已停止', failed: '失败', error: '错误' };

export function normalizeCinematography(data) {
    if (data?.schema_version !== 1 || data.available !== true) {
        return { ...unavailable, reason: typeof data?.reason === 'string' && data.reason || unavailable.reason };
    }
    const modes = (Array.isArray(data.modes) ? data.modes : []).filter(mode => typeof mode === 'string' && mode.trim() !== '');
    return { schema_version: 1, available: true, state: typeof data.state === 'string' ? data.state : 'unknown',
        mode: typeof data.mode === 'string' ? data.mode : null, reason: typeof data.reason === 'string' ? data.reason : '',
        modes: [...new Set(modes)], can_start: data.can_start === true, can_stop: data.can_stop === true };
}

export function canRequestCinematography(data, transport, action) {
    return Boolean(['start', 'stop'].includes(action) && data?.available === true && data['can_' + action] === true
        && transport?.mode === 'live' && transport.connected && transport.canControl === true);
}

function mount(preview, request) {
    const dialog = document.createElement('dialog');
    dialog.className = 'experiment-dialog cinematography-dialog';
    dialog.setAttribute('aria-labelledby', 'cinematographyTitle');
    dialog.innerHTML = `
      <header class="experiment-dialog-head">
        <div><h2 id="cinematographyTitle">自主运镜</h2><p id="cinematographyStatus" role="status">正在读取状态…</p></div>
        <button type="button" class="experiment-secondary" data-cinema-close>关闭</button>
      </header>
      <div class="experiment-dialog-body">
        <section class="experiment-panel-section" aria-label="自主运镜策略状态">
          <p id="cinematographyReason" class="experiment-status"></p>
          <div class="experiment-tools">
            <label id="cinematographyModeLabel" hidden>运镜模式 <select id="cinematographyMode"></select></label>
            <button type="button" class="experiment-secondary" data-cinema-action="start" disabled>启动自主运镜</button>
            <button type="button" class="experiment-secondary" data-cinema-action="stop" disabled>停止自主运镜</button>
          </div>
          <p id="cinematographyActionStatus" class="experiment-status" role="status"></p>
        </section>
      </div>`;
    document.body.append(dialog);

    const find = selector => dialog.querySelector(selector);
    const status = find('#cinematographyStatus');
    const reason = find('#cinematographyReason');
    const mode = find('#cinematographyMode');
    const modeLabel = find('#cinematographyModeLabel');
    const actionStatus = find('#cinematographyActionStatus');
    const actions = [...dialog.querySelectorAll('[data-cinema-action]')];
    let authority, data = unavailable, busy = false, pending, timer, requestVersion;
    let subscriptions = [];

    function render() {
        const simulated = preview.state.transportMode === 'mock';
        const caption = labels[data.state] || '状态未知';
        status.textContent = (simulated ? '演示模式 · ' : '') + caption;
        reason.textContent = data.reason || (data.available && !authority?.canControl ? '当前页面只读' : '');
        const signature = JSON.stringify(data.modes);
        if (mode.dataset.signature !== signature) {
            mode.dataset.signature = signature;
            const previous = mode.value;
            mode.replaceChildren(new Option('默认模式', ''), ...data.modes.map(name => new Option(name, name)));
            mode.value = data.modes.includes(previous) ? previous : data.modes.includes(data.mode) ? data.mode : '';
        }
        modeLabel.hidden = !data.available || !data.modes.length;
        mode.disabled = busy || !canRequestCinematography(data, authority, 'start');
        for (const action of actions) action.disabled = busy || !canRequestCinematography(data, authority, action.dataset.cinemaAction);
    }

    async function refresh() {
        if (!dialog.open || pending) return;
        if (preview.state.transportMode === 'mock') {
            data = unavailable;
            render();
            return;
        }
        if (!authority?.connected) {
            data = { ...unavailable, reason: '连接并配对后可查看自主运镜状态' };
            render();
            return;
        }
        const token = new AbortController();
        pending = token;
        const deadline = setTimeout(() => token.abort(), 3500);
        try {
            const response = await request(endpoint + '/state', { cache: 'no-store', signal: token.signal });
            const wire = await response.json().catch(() => ({}));
            if (pending !== token || !dialog.open) return;
            if (!response.ok || wire.success !== true) {
                data = { ...unavailable, reason: response.status === 404 ? unavailable.reason : wire.message || '暂时无法读取自主运镜状态' };
            } else {
                data = normalizeCinematography(wire.data);
            }
        } catch (error) {
            if (pending !== token || !dialog.open) return;
            data = { ...unavailable, reason: error.name === 'AbortError' ? '读取自主运镜状态超时' : '暂时无法读取自主运镜状态' };
        } finally {
            clearTimeout(deadline);
            if (pending === token) { pending = null; render(); }
        }
    }

    function open() {
        if (dialog.open) return;
        data = unavailable;
        busy = false;
        requestVersion = Symbol();
        authority = preview.getTransport();
        subscriptions = authority ? [authority.on('controlSession', render), authority.on('connection', () => { render(); refresh(); })] : [];
        actionStatus.textContent = '';
        render();
        dialog.showModal();
        refresh();
        timer = setInterval(refresh, 2000);
    }

    for (const action of actions) {
        action.addEventListener('click', async () => {
            const name = action.dataset.cinemaAction;
            if (busy || !canRequestCinematography(data, authority, name)) return;
            const version = requestVersion;
            busy = true;
            actionStatus.textContent = '正在提交请求…';
            render();
            try {
                const parameters = name === 'start' && data.modes.includes(mode.value) ? { mode: mode.value } : {};
                const response = await request(endpoint + '/' + name, {
                    method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(parameters)
                });
                const payload = await response.json().catch(() => ({}));
                if (version !== requestVersion || !dialog.open) return;
                if (!response.ok || payload.success !== true) throw new Error(payload.message || '自主运镜请求失败');
                actionStatus.textContent = '请求已提交，等待策略状态确认';
            } catch (error) {
                if (version === requestVersion && dialog.open) actionStatus.textContent = error.message;
            } finally {
                if (version === requestVersion && dialog.open) { busy = false; render(); refresh(); }
            }
        });
    }

    find('[data-cinema-close]').addEventListener('click', () => dialog.close());
    dialog.addEventListener('close', () => {
        for (const unsubscribe of subscriptions) unsubscribe();
        subscriptions = [];
        pending?.abort();
        pending = null;
        clearInterval(timer);
    });
    document.addEventListener('click', event => { if (event.target.closest?.('[data-cinematography-open]')) open(); });
}

if (typeof window !== 'undefined' && window.MantaPreview && window.MantaAppTransport) {
    mount(window.MantaPreview, window.MantaAppTransport.request);
}
