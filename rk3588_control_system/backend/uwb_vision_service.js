import fs from 'node:fs';
import path from 'node:path';
import { FUSION_DEFAULTS, validateFusionConfig, evaluateUwbVision } from '../frontend/js/uwb-vision-core.js';

export function installUwbVisionApi(app, { filename, getInputs, model = '', defaults = {}, followInProgress = () => false, onFollowSourceChange = () => {} }) {
    let config = { ...FUSION_DEFAULTS, ...validateFusionConfig(defaults) }, enabled = false;
    if (fs.existsSync(filename)) {
        try { config = { ...config, ...validateFusionConfig(JSON.parse(fs.readFileSync(filename, 'utf8'))) }; }
        catch (error) { console.error('[UWB Vision] Invalid saved config; using uncalibrated defaults:', error.message); }
    }
    let followSource = 'uwb';
    function selectSource(source) {
        if (source !== followSource) onFollowSourceChange(source);
        followSource = source;
        enabled = source === 'uwb+vision';
    }
    const evaluate = now => evaluateUwbVision({ ...getInputs(now), enabled, config, now });
    const read = () => ({ success: true, model, followSource, config: { ...config }, state: evaluate(Date.now()) });
    app.post('/api/uwb-vision/follow-source', (req, res) => {
        const source = req.body?.source;
        if (source !== 'uwb' && source !== 'uwb+vision')
            return res.status(400).json({ success: false, message: '跟随定位源无效' });
        if (source !== followSource && followInProgress())
            return res.status(409).json({ success: false, code: 'FOLLOW_ACTIVE', message: '请停止船体跟随后再切换定位源' });
        selectSource(source);
        return res.json(read());
    });
    app.get('/api/uwb-vision/state', (_req, res) => res.json(read()));
    // Mounted after the existing device pairing/owner middleware. No new auth bypass.
    app.post('/api/uwb-vision/config', (req, res) => {
        let next;
        try { next = { ...config, ...validateFusionConfig(req.body) }; }
        catch (error) { return res.status(400).json({ success: false, message: error.message }); }
        // Changing camera geometry invalidates the previous calibration unless explicitly reconfirmed.
        const geometryChanged = ['horizontalFovDeg', 'yawSign', 'imageSign', 'yawOffsetDeg'].some(key => next[key] !== config[key]);
        if (geometryChanged && req.body.calibrated !== true) next.calibrated = false;
        try {
            fs.mkdirSync(path.dirname(filename), { recursive: true });
            fs.writeFileSync(filename + '.tmp', JSON.stringify(next, null, 2) + '\n', { mode: 0o600 });
            fs.renameSync(filename + '.tmp', filename);
        } catch (error) {
            console.error('[UWB Vision] Config save failed:', error.message);
            return res.status(500).json({ success: false, message: '融合配置保存失败' });
        }
        config = next;
        return res.json(read());
    });
    app.post('/api/uwb-vision/enabled', (req, res) => {
        if (typeof req.body?.enabled !== 'boolean') return res.status(400).json({ success: false, message: 'enabled 必须为布尔值' });
        if (!req.body.enabled && followSource === 'uwb+vision')
            return res.status(409).json({ success: false, message: '请先将自主跟随切换到纯 UWB' });
        enabled = req.body.enabled;
        return res.json(read());
    });
    return { read, get followSource() { return followSource; }, guidance: now => followSource === 'uwb+vision' ? evaluate(now) : null,
        stop: () => selectSource('uwb') };
}
