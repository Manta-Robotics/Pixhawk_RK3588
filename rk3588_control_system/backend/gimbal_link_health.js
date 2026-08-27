export function createGimbalLinkHealth(timeoutMs = 3000) {
  return {
    timeoutMs: Math.max(500, Number(timeoutMs) || 3000),
    status: 'idle',
    command: '',
    sentAt: null,
    deadlineAt: null,
    respondedAt: null
  };
}

export function resetGimbalLinkHealth(health) {
  health.status = 'idle';
  health.command = '';
  health.sentAt = null;
  health.deadlineAt = null;
  health.respondedAt = null;
  return health;
}

export function expectGimbalResponse(health, command, now = Date.now()) {
  health.status = 'waiting';
  health.command = String(command || 'command');
  health.sentAt = Number(now);
  health.deadlineAt = Number(now) + health.timeoutMs;
  health.respondedAt = null;
  return health;
}

export function acceptGimbalFeedback(health, now = Date.now()) {
  if (health.status === 'waiting' || health.status === 'timeout') {
    health.status = 'confirmed';
    health.respondedAt = Number(now);
    health.deadlineAt = null;
  }
  return health;
}

export function evaluateGimbalLink(health, options = {}) {
  const now = Number(options.now ?? Date.now());
  const serialPresent = options.serialPresent !== false;
  const portOpen = Boolean(options.portOpen);
  const feedbackFresh = Boolean(options.feedbackFresh);

  if (health.status === 'waiting' && Number(health.deadlineAt) > 0 && now >= Number(health.deadlineAt)) {
    health.status = 'timeout';
  }
  if (!serialPresent) return { connected: false, linkStatus: 'missing', error: 'Gimbal serial device is missing' };
  if (!portOpen) return { connected: false, linkStatus: 'offline', error: '' };
  if (health.status === 'timeout') {
    return {
      connected: false,
      linkStatus: 'command_timeout',
      error: `Gimbal did not respond to ${health.command} within ${health.timeoutMs} ms`
    };
  }
  if (feedbackFresh) return { connected: true, linkStatus: 'feedback', error: '' };
  if (health.status === 'waiting') return { connected: true, linkStatus: 'checking', error: '' };
  return { connected: true, linkStatus: 'ready', error: '' };
}
