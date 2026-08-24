export function isSameOriginRequest(request) {
  const origin = String(request?.headers?.origin || '').trim();
  if (!origin) return true;

  const host = String(request?.headers?.host || '').trim().toLowerCase();
  if (!host) return false;

  try {
    const parsed = new URL(origin);
    return (parsed.protocol === 'http:' || parsed.protocol === 'https:') && parsed.host.toLowerCase() === host;
  } catch (_) {
    return false;
  }
}

export function rejectCrossOrigin(request, response, next) {
  if (isSameOriginRequest(request)) {
    next();
    return;
  }
  response.status(403).json({ success: false, message: 'Cross-origin request rejected' });
}
