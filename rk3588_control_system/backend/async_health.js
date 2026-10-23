import http from 'node:http';
import https from 'node:https';

// Keep the health request, but never wait for it on the telemetry event loop.
export function createAsyncHealth(url, { intervalMs = 5000, timeoutMs = 600 } = {}) {
  let value = { ok: false, message: 'Waiting for camera health response.' };
  let pending = false;
  let checkedAt = -Infinity;
  return function readHealth() {
    if (!pending && Date.now() - checkedAt >= intervalMs) {
      pending = true;
      checkedAt = Date.now();
      let settled = false;
      let timer;
      const finish = (result) => {
        if (settled) return;
        settled = true;
        clearTimeout(timer);
        value = result;
        pending = false;
      };
      const request = (url.startsWith('https:') ? https : http).get(url, (response) => {
        let body = '';
        response.setEncoding('utf8');
        response.on('data', (chunk) => {
          body += chunk;
          if (body.length > 65536) request.destroy(new Error('Camera health response exceeds 64 KiB'));
        });
        response.on('error', (error) => finish({ ok: false, message: error.message }));
        response.on('end', () => {
          try {
            if (response.statusCode !== 200) throw new Error(`Camera health HTTP ${response.statusCode}`);
            const payload = JSON.parse(body);
            finish({ ok: Boolean(payload.ok), message: String(payload.lastError || payload.message || ''),
              cachedDevice: String(payload.cachedDevice || ''), cachedName: String(payload.cachedName || ''),
              lastFrameAgeSeconds: Number.isFinite(payload.lastFrameAgeSeconds) ? payload.lastFrameAgeSeconds : null });
          } catch (error) { finish({ ok: false, message: error.message }); }
        });
      });
      timer = setTimeout(() => {
        finish({ ok: false, message: 'Camera health request timed out.' });
        request.destroy();
      }, timeoutMs);
      timer.unref();
      request.on('error', (error) => finish({ ok: false, message: error.message }));
    }
    return { ...value };
  };
}
