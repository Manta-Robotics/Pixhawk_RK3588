import { performance } from 'node:perf_hooks';
import { BlockList, isIP } from 'node:net';

const STATE_EVENTS = ['uwb_path_update', 'uwb_update', 'telemetry_update', 'motor_output_update', 'gimbal_state',
  'uwb_follow_update', 'rover_control_update', 'gimbal_target', 'vision_detections'];
const normalizeAddress = address => String(address || '').replace(/^::ffff:/, '').split('%')[0];
const observationClock = value => Math.max(0, ...[value?.measurementUpdatedAt,
  ...(value?.sensors || []).map(sensor => sensor.receivedMeasurementAt)].filter(Number.isFinite));
function currentUwb(pending, incoming) {
  if (!incoming) return pending;
  const pendingClock = observationClock(pending), incomingClock = observationClock(incoming);
  if (pendingClock !== incomingClock) return pendingClock > incomingClock ? pending : incoming;
  return Number(pending.updatedAt || 0) >= Number(incoming.updatedAt || 0) ? pending : incoming;
}

export function isWirelessPeer(entries, localAddress, remoteAddress) {
  const local = normalizeAddress(localAddress), remote = normalizeAddress(remoteAddress);
  const networks = entries.filter(entry => typeof entry === 'object' && entry?.cidr);
  if (networks.length && isIP(remote)) {
    // A Wi-Fi peer may use a hostname which resolves to the board's Ethernet
    // address. Match the peer's wireless subnet, not only our destination IP.
    return networks.some(entry => {
      const address = normalizeAddress(entry.address);
      if (address.startsWith('fe80:') && address !== local) return false;
      const family = isIP(address), prefix = Number(entry.cidr.split('/')[1]);
      if (!family || family !== isIP(remote) || !Number.isInteger(prefix)
          || prefix < 0 || prefix > (family === 4 ? 32 : 128)) return false;
      const subnet = new BlockList();
      subnet.addSubnet(address, prefix, family === 4 ? 'ipv4' : 'ipv6');
      return subnet.check(remote, family === 4 ? 'ipv4' : 'ipv6');
    });
  }
  return Boolean(local) && entries.some(entry => normalizeAddress(entry.address ?? entry) === local);
}

export const WIFI_COMPRESSION = {
  threshold: 1024, serverNoContextTakeover: true, clientNoContextTakeover: true,
  concurrencyLimit: 2, zlibDeflateOptions: { level: 1 }
};

// WebSocket extensions are negotiated at the HTTP upgrade, before a Socket.IO
// connection exists. Only the actual local wireless address may negotiate
// compression; forwarded headers and addresses supplied by clients are ignored.
export function restrictCompressionToWifi(io, wirelessAddresses) {
  io.engine.use((request, _response, next) => {
    if (!isWirelessPeer(wirelessAddresses(), request.socket?.localAddress, request.socket?.remoteAddress)) {
      delete request.headers['sec-websocket-extensions'];
    }
    next();
  });
}

export function emitWifiAwareState(io, delivery, event, value, { volatile = false } = {}) {
  for (const socket of io.sockets.sockets.values()) {
    if (!delivery.enqueue(socket, event, value)) {
      (volatile ? socket.volatile : socket).emit(event, value);
    }
  }
}

// A successful TCP write only means that the kernel accepted the bytes. On a
// slow radio it can hide seconds of queued state. Native WebSocket pongs
// confirm a bounded transmission window without changing the client.
// Commands, acknowledgements, logs, and wired clients never enter this queue.
export class WifiLatestDelivery {
  constructor({ wirelessAddresses = () => [], retryMs = 5, maxInFlight = 1 } = {}) {
    if (!Number.isInteger(maxInFlight) || maxInFlight < 1 || maxInFlight > 2) {
      throw new RangeError('Wi-Fi state window must contain one or two batches');
    }
    this.wirelessAddresses = wirelessAddresses;
    this.retryMs = retryMs;
    this.maxInFlight = maxInFlight;
    this.bySocket = new WeakMap();
    this.wired = new WeakSet();
    this.clients = new Set();
    this.closed = false;
  }

  enqueue(socket, event, value) {
    if (this.closed || this.wired.has(socket) || !STATE_EVENTS.includes(event)) return false;
    const transport = socket.conn?.transport;
    const ws = transport?.socket;
    if (transport?.name !== 'websocket' || typeof ws?.ping !== 'function') return false;
    let state = this.bySocket.get(socket);
    if (!state) {
      const connection = ws._socket || socket.conn.request?.socket;
      if (!isWirelessPeer(this.wirelessAddresses(), connection?.localAddress, connection?.remoteAddress)) {
        this.wired.add(socket);
        return false;
      }
      state = this.attach(socket, transport, ws);
    }
    if (state.closed) return true;
    if (state.pending.has(event)) state.superseded++;
    if (event === 'telemetry_update' && state.pending.has('uwb_update')) {
      value = { ...value, uwb: currentUwb(state.pending.get('uwb_update'), value.uwb) };
    }
    state.pending.set(event, value);
    if (event === 'uwb_update' && state.pending.has('telemetry_update')) {
      const full = state.pending.get('telemetry_update');
      state.pending.set('telemetry_update', { ...full, uwb: currentUwb(value, full.uwb) });
    }
    this.schedule(state);
    return true;
  }

  attach(socket, transport, ws) {
    const state = { socket, transport, ws, pending: new Map(), phase: 'idle',
      immediate: null, retry: null, closed: false, sequence: 0,
      batches: 0, confirmed: 0, superseded: 0, lastRttMs: null, maxRttMs: 0,
      rttSamples: [], lastUwbNodeWaitMs: null, inFlight: new Map(),
      uwbBatches: 0, uniqueUwbObservations: 0, uwbClocks: new Map() };
    state.onReady = () => {
      if (state.phase === 'sending') this.probe(state);
      else this.schedule(state);
    };
    state.onPong = data => {
      const token = Buffer.from(data).toString();
      const sent = state.inFlight.get(token);
      if (!sent) return;
      state.lastRttMs = performance.now() - sent.at;
      state.maxRttMs = Math.max(state.maxRttMs, state.lastRttMs);
      state.confirmed++;
      state.rttSamples.push({ batch: sent.sequence, rttMs: state.lastRttMs });
      if (state.rttSamples.length > 128) state.rttSamples.shift();
      state.inFlight.delete(token);
      if (state.phase !== 'sending') state.phase = 'idle';
      this.schedule(state);
    };
    state.onClose = () => this.detach(state);
    transport.on('ready', state.onReady);
    ws.on('pong', state.onPong);
    ws.once('close', state.onClose);
    socket.conn.once('close', state.onClose);
    this.bySocket.set(socket, state);
    this.clients.add(state);
    return state;
  }

  writable(state) {
    const { socket, transport, ws } = state;
    return socket.connected && ws.readyState === 1 && transport.writable
      && !socket.conn.writeBuffer?.length && !ws.bufferedAmount;
  }

  schedule(state) {
    if (!this.canFlush(state) || state.immediate) return;
    state.immediate = setImmediate(() => {
      state.immediate = null;
      this.flush(state);
    });
    state.immediate.unref();
  }

  canFlush(state) {
    if (state.closed || state.phase === 'sending' || !state.pending.size
        || state.inFlight.size >= this.maxInFlight) return false;
    // Keep the additional slot available for a new UWB observation. Other
    // dashboard events cannot fill it and force UWB to wait for an entire RTT.
    return state.inFlight.size === 0 || state.pending.has('uwb_update');
  }

  retry(state) {
    if (state.closed || state.retry) return;
    state.retry = setTimeout(() => {
      state.retry = null;
      if (state.phase === 'sending') this.probe(state);
      else this.flush(state);
    }, this.retryMs);
    state.retry.unref();
  }

  flush(state) {
    if (!this.canFlush(state)) return;
    if (!this.writable(state)) { this.retry(state); return; }
    const batch = state.pending;
    state.pending = new Map();
    state.phase = 'sending';
    state.batches++;
    // UWB is first in every batch, including recovery from a radio stall.
    for (const event of STATE_EVENTS) {
      if (!batch.has(event)) continue;
      const value = batch.get(event);
      if (event === 'uwb_update') {
        state.uwbBatches++;
        let changed = false;
        for (const sensor of value.sensors || []) {
          const clock = sensor.receivedMeasurementAt;
          if (Number.isFinite(clock) && state.uwbClocks.get(sensor.receiverId) !== clock) {
            state.uwbClocks.set(sensor.receiverId, clock);
            changed = true;
          }
        }
        if (changed) state.uniqueUwbObservations++;
        if (Number.isFinite(value.backendReceivedAt)) state.lastUwbNodeWaitMs = Date.now() - value.backendReceivedAt;
      }
      state.socket.emit(event, event === 'uwb_update'
        ? { ...value, backendEmittedAt: Date.now() } : value);
    }
    this.probe(state);
  }

  probe(state) {
    if (state.closed || state.phase !== 'sending') return;
    // Engine.IO may still be flushing the rest of this batch. A ping sent
    // directly through ws before 'ready' could overtake those queued packets.
    if (!this.writable(state)) { this.retry(state); return; }
    const sequence = ++state.sequence, token = `manta-state:${sequence}`;
    state.inFlight.set(token, { sequence, at: performance.now() });
    state.phase = state.inFlight.size >= this.maxInFlight ? 'awaiting' : 'idle';
    state.ws.ping(Buffer.from(token), error => {
      if (error) this.detach(state);
    });
    this.schedule(state);
  }

  diagnostics() {
    return [...this.clients].map(state => ({
      socketId: state.socket.id, phase: state.phase,
      pendingEvents: [...state.pending.keys()],
      batches: state.batches, confirmedBatches: state.confirmed,
      inFlightBatches: state.inFlight.size, maxInFlightBatches: this.maxInFlight,
      uwbBatches: state.uwbBatches, uniqueUwbObservations: state.uniqueUwbObservations,
      supersededStates: state.superseded, lastConfirmationRttMs: state.lastRttMs,
      maxConfirmationRttMs: state.maxRttMs,
      recentConfirmations: state.rttSamples,
      lastUwbNodeWaitMs: state.lastUwbNodeWaitMs,
      compression: state.ws.extensions || '',
      confirmationWaitMs: state.inFlight.size ? performance.now() - state.inFlight.values().next().value.at : 0,
      websocketBufferedBytes: state.ws.bufferedAmount
    }));
  }

  detach(state) {
    if (state.closed) return;
    state.closed = true;
    clearImmediate(state.immediate);
    clearTimeout(state.retry);
    state.transport.off('ready', state.onReady);
    state.ws.off('pong', state.onPong);
    state.ws.off('close', state.onClose);
    state.socket.conn.off('close', state.onClose);
    state.pending.clear();
    state.inFlight.clear();
    this.clients.delete(state);
  }

  close() {
    this.closed = true;
    for (const state of this.clients) this.detach(state);
  }
}
