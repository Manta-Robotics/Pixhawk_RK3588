import express from 'express';
import http from 'node:http';

// Mount after the existing paired-session API middleware. The upstream is a
// fixed loopback endpoint; request headers and URLs cannot select a destination.
export function createGimbalWebrtcProxy({ port = 8889, timeoutMs = 5000 } = {}) {
  const router = express.Router();
  const base = `http://127.0.0.1:${port}`;
  const pathPattern = /^\/whep(?:\/([0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}))?$/i;
  const bodyTypes = new Set(['application/sdp', 'application/trickle-ice-sdpfrag']);
  router.use((req, res, next) => {
    if (!pathPattern.test(req.path)) return res.status(404).json({ success: false, message: 'Unknown video session' });
    if (!['OPTIONS', 'POST', 'PATCH', 'DELETE'].includes(req.method)) {
      return res.set('Allow', 'OPTIONS, POST, PATCH, DELETE').status(405).end();
    }
    if (['POST', 'PATCH'].includes(req.method)
        && !bodyTypes.has(String(req.headers['content-type'] || '').split(';')[0].trim().toLowerCase())) {
      return res.status(415).json({ success: false, message: 'SDP content type required' });
    }
    next();
  });
  router.use(express.raw({ type: () => true, limit: '64kb', inflate: false }));
  router.use((req, res) => {
    const session = pathPattern.exec(req.path)[1];
    const pathname = '/gimbal/whep' + (session ? '/' + session : '');
    const body = Buffer.isBuffer(req.body) ? req.body : Buffer.alloc(0);
    const headers = { Origin: base };
    for (const name of ['content-type', 'if-match']) {
      if (req.headers[name]) headers[name] = req.headers[name];
    }
    if (body.length) headers['content-length'] = body.length;
    const upstream = http.request({ hostname: '127.0.0.1', port, path: pathname,
      method: req.method, headers }, response => {
      const chunks = [];
      let length = 0;
      response.on('data', chunk => {
        length += chunk.length;
        if (length > 256 * 1024) { upstream.destroy(new Error('Video answer too large')); return; }
        chunks.push(chunk);
      });
      response.on('end', () => {
        if (res.destroyed || res.headersSent) return;
        if (response.headers.location) {
          let location;
          try { location = new URL(response.headers.location, base); } catch {}
          const match = location?.pathname.match(/^\/gimbal\/whep\/([0-9a-f-]{36})$/i);
          if (!location || location.origin !== base || !match || location.search || location.hash) {
            res.status(502).json({ success: false, message: 'Invalid video session response' });
            return;
          }
          res.setHeader('Location', '/api/gimbal/webrtc/whep/' + match[1]);
        }
        for (const name of ['content-type', 'etag', 'accept-patch', 'link', 'allow']) {
          if (response.headers[name]) res.setHeader(name, response.headers[name]);
        }
        res.setHeader('Cache-Control', 'no-store');
        res.status(response.statusCode || 502).end(Buffer.concat(chunks));
      });
      response.on('error', error => upstream.destroy(error));
    });
    const fail = () => {
      if (!res.destroyed && !res.headersSent) res.status(502).json({ success: false, message: 'Video relay unavailable' });
    };
    upstream.setTimeout(timeoutMs, () => upstream.destroy(new Error('Video relay timeout')));
    upstream.on('error', fail);
    req.on('aborted', () => upstream.destroy());
    res.on('close', () => { if (!res.writableEnded) upstream.destroy(); });
    upstream.end(body);
  });
  return router;
}
