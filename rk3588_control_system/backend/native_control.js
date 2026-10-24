import dgram from 'node:dgram';
import { randomUUID } from 'node:crypto';

// Request/reply IPC carries operator intent only. The C++ process owns every
// actuator tick, sensor-age check and command lease. Requests are never retried:
// a lost reply must not duplicate an arm/start/calibration transaction.
export class NativeControlClient {
  constructor({ host = '127.0.0.1', port = 14551, timeoutMs = 1200 } = {}) {
    this.host = host; this.port = port; this.timeoutMs = timeoutMs;
    this.pending = new Map(); this.socket = dgram.createSocket('udp4'); this.closed = false;
    this.socket.on('message', (bytes, peer) => {
      if (peer.address !== this.host || peer.port !== this.port) return;
      try {
        const packet = JSON.parse(bytes.toString('utf8'));
        if (packet.type !== 'command_result') return;
        const entry = this.pending.get(packet.payload?.requestId);
        if (!entry) return;
        clearTimeout(entry.timer); this.pending.delete(packet.payload.requestId);
        entry.resolve(packet.payload.result);
      } catch { /* Ignore malformed replies without extending their deadline. */ }
    });
    this.socket.on('error', () => this.failPending('native_control_unavailable'));
  }
  failPending(error) {
    for (const entry of this.pending.values()) { clearTimeout(entry.timer); entry.resolve({ ok: false, error }); }
    this.pending.clear();
  }
  command(command, params = {}) {
    if (this.closed) return Promise.resolve({ ok: false, error: 'native_control_unavailable' });
    if (this.pending.size >= 128) return Promise.resolve({ ok: false, error: 'native_control_busy' });
    const requestId = randomUUID();
    return new Promise(resolve => {
      const timer = setTimeout(() => { this.pending.delete(requestId); resolve({ ok: false, error: 'native_control_timeout' }); }, this.timeoutMs);
      this.pending.set(requestId, { resolve, timer });
      this.socket.send(Buffer.from(JSON.stringify({ command, params, requestId, timestamp: Date.now() })), this.port, this.host, error => {
        if (!error) return;
        const entry = this.pending.get(requestId); if (!entry) return;
        clearTimeout(entry.timer); this.pending.delete(requestId); resolve({ ok: false, error: 'native_control_unavailable' });
      });
    });
  }
  async start(params = {}) {
    const result = await this.command('FOLLOW_START', params);
    if (!result.ok || !result.pending) return result;
    const deadline = performance.now() + 20000;
    while (performance.now() < deadline && !this.closed) {
      await new Promise(resolve => setTimeout(resolve, 100));
      const status = await this.command('CONTROL_STATUS');
      if (!status.ok) return status;
      if (status.transitionId !== result.transitionId) return { ok: false, error: status.state?.reason || 'start_cancelled', state: status.state };
      if (!status.busy) return status.state?.active ? { ok: true, state: status.state } : { ok: false, error: status.state?.reason || 'start_failed', state: status.state };
    }
    return { ok: false, error: 'native_control_timeout' };
  }
  close() {
    this.closed = true; this.failPending('native_control_unavailable');
    try { this.socket.close(); } catch { /* Socket may not have bound yet. */ }
  }
}
