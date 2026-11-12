import fs from 'node:fs';
import path from 'node:path';
import { FUSION_DEFAULTS, validateFusionConfig, evaluateUwbVision } from '../frontend/js/uwb-vision-core.js';

export function installUwbVisionApi(app, { filename, getInputs, model = '', defaults = {} }) {
    let config = { ...FUSION_DEFAULTS, ...validateFusionConfig(defaults) }, enabled = false;
    if (fs.existsSync(filename)) {
        try { config = { ...config, ...validateFusionConfig(JSON.parse(fs.readFileSync(filename, 'utf8'))) }; }
        catch (error) { console.error('[UWB Vision] Invalid saved config; using uncalibrated defaults:', error.message); }
    }
    const read = () => ({ success: true, model, config: { ...config }, state: evaluateUwbVision({ ...getInputs(), enabled, config }) });
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
        enabled = req.body.enabled;
        return res.json(read());
    });
    return { read, stop: () => { enabled = false; } };
}
