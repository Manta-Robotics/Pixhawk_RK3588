// A board-specific cinematography provider can be supplied at server assembly.
// Reading capabilities never starts a tracker, claims control or moves hardware.
const unavailable = Object.freeze({
  schema_version: 1,
  available: false,
  state: 'unavailable',
  mode: null,
  reason: '尚未接入自主运镜策略',
  modes: [],
  can_start: false,
  can_stop: false
});
export function installCinematographyApi(app, provider = null) {
  const prefix = '/api/cinematography';
  let changing = false;
  async function state() {
    if (!provider) return unavailable;
    const result = await provider.state();
    return { ...unavailable, ...result, schema_version: 1 };
  }
  function fail(res, status, code, message) {
    return res.status(status).json({ success: false, code, message });
  }
  app.get(prefix + '/state', async (_req, res) => {
    try {
      res.json({ success: true, data: await state() });
    } catch {
      fail(res, 503, 'CINEMATOGRAPHY_OFFLINE', '自主运镜状态暂时不可用');
    }
  });
  for (const action of ['start', 'stop']) {
    app.post(prefix + '/' + action, async (req, res) => {
      if (changing) return fail(res, 409, 'CINEMATOGRAPHY_BUSY', '该模块正在切换状态');
      changing = true;
      try {
        const before = await state();
        if (!before.available) return fail(res, 409, 'CINEMATOGRAPHY_UNAVAILABLE', before.reason);
        if (!before['can_' + action])
          return fail(res, 409, 'CINEMATOGRAPHY_STATE', '当前状态无法执行该操作');
        if (action === 'start') {
          const mode = req.body?.mode;
          if (mode != null && !before.modes.includes(mode))
            return fail(res, 400, 'INVALID_ARGUMENT', '未知的运镜模式');
          await provider.start({ mode: mode ?? null });
        } else await provider.stop();
        res.json({ success: true, data: await state() });
      } catch {
        fail(res, 503, 'CINEMATOGRAPHY_OFFLINE', '自主运镜模块未能完成操作');
      } finally {
        changing = false;
      }
    });
  }
}
