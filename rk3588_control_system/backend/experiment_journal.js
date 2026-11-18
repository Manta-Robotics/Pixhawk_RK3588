import net from 'node:net';
import fs from 'node:fs/promises';
import path from 'node:path';
import { spawn } from 'node:child_process';

const maxLine = 1024 * 1024;
const sane = (value) => Number.isSafeInteger(value) && value >= 0;
const send = (socket, obj) => socket.write(JSON.stringify(obj) + '\n');

// The writer alone commits records. This bounded outbox survives a writer restart.
export class ExperimentJournal {
  constructor(live, config, projectRoot, metadata = {}) {
    this.live = live;
    this.config = config;
    this.root = projectRoot;
    this.metadata = metadata;
    this.queue = new Map();
    this.bytes = 0;
    this.seq = 0;
    this.cursor = 0;
    this.badLines = 0;
  }
  async listen() {
    const name = this.config.journal_socket;
    await fs.mkdir(path.dirname(name), { recursive: true, mode: 0o700 });
    try {
      const info = await fs.lstat(name);
      if (!info.isSocket()) throw new Error('日志通信路径已有其他文件');
      await new Promise((resolve, reject) => {
        const probe = net.createConnection(name);
        probe.once('connect', () => {
          probe.destroy();
          reject(new Error('日志接口已由其他实例使用'));
        });
        probe.once('error', (err) => (err.code === 'ECONNREFUSED' ? resolve() : reject(err)));
      });
      await fs.unlink(name);
    } catch (error) {
      if (error.code !== 'ENOENT') throw error;
    }
    this.server = net.createServer((socket) => this.accept(socket));
    await new Promise((resolve, reject) => {
      this.server.once('error', reject);
      this.server.listen(name, resolve);
    });
    await fs.chmod(name, 0o600);
    this.server.unref();
  }
  accept(socket) {
    let buffer = Buffer.alloc(0);
    socket.on('error', () => {});
    socket.on('data', (data) => {
      buffer = Buffer.concat([buffer, data]);
      let at;
      while ((at = buffer.indexOf(10)) >= 0) {
        if (at > maxLine) {
          this.badLines++;
          socket.destroy();
          return;
        }
        const line = buffer.subarray(0, at);
        buffer = buffer.subarray(at + 1);
        try {
          this.receive(socket, JSON.parse(line.toString('utf8')));
        } catch {
          this.badLines++;
        }
      }
      if (buffer.length > maxLine) {
        this.badLines++;
        socket.destroy();
      }
    });
    socket.on('drain', () => {
      if (this.peer === socket) {
        this.blocked = false;
        this.pump();
      }
    });
    socket.on('close', () => {
      if (this.peer === socket) {
        this.peer = null;
        this.blocked = false;
      }
    });
  }
  receive(socket, m) {
    if (m.v !== 2 || !m.session_id || typeof m.session_id !== 'string') return;
    if (m.kind === 'hello') {
      if (
        m.boot_id !== this.live.boot ||
        !/^\d{1,24}$/.test(m.session_start_mono_ns) ||
        !sane(m.last_ack) ||
        (m.last_row_no != null && !sane(m.last_row_no))
      )
        return socket.destroy();
      if (this.peer && this.peer !== socket && this.live.session !== m.session_id) return socket.destroy();
      if (this.peer !== socket) this.peer?.destroy();
      const changed = this.live.session !== m.session_id;
      if (changed) {
        clearTimeout(this.drainTimer);
        this.live.leave();
        this.queue.clear();
        this.bytes = 0;
        this.end = null;
        this.sentEnd = null;
        this.finishing = false;
        this.sessionFirst = this.seq + 1;
      }
      this.peer = socket;
      this.blocked = false;
      this.cursor =
        m.last_epoch === this.live.epoch && !changed
          ? Math.max((this.sessionFirst ?? 1) - 1, Math.min(this.seq, m.last_ack))
          : (this.sessionFirst ?? 1) - 1;
      const first = this.queue.keys().next().value ?? this.seq + 1;
      send(socket, {
        v: 2,
        kind: 'ready',
        epoch: this.live.epoch,
        first_seq: first,
        latest_seq: this.seq,
        session_first_seq: this.sessionFirst ?? 1,
        diagnostic_config: this.live.d,
        ...this.metadata
      });
      if (m.last_epoch && m.last_epoch !== this.live.epoch)
        send(socket, {
          v: 2,
          kind: 'gap',
          epoch: this.live.epoch,
          first_missing: null,
          last_missing: null,
          reason: 'node_restart'
        });
      this.pump();
      this.live.enter(m.session_id, m.session_start_mono_ns, m.last_row_no);
      this.requested = false;
      if (this.end) this.pump();
      return;
    }
    if (socket !== this.peer || m.session_id !== this.live.session) return;
    if (
      m.kind === 'writer_status' &&
      m.epoch === this.live.epoch &&
      sane(m.ack_seq) &&
      m.ack_seq <= this.seq
    ) {
      this.live.status(m);
      if (m.state === 'closed') {
        this.live.leave();
        this.end = null;
        this.finishing = false;
        this.requested = false;
        this.peer = null;
        socket.end();
      }
    }
    if (m.kind === 'finish' && !this.finishing) {
      this.finishing = true;
      this.cutoff = this.live.clock();
      for (const a of [...this.live.alerts.values()]) this.live.closeAlert(a.code, 'session_end');
      const finish = () => {
        this.end = {
          kind: 'end',
          v: 2,
          session_id: this.live.session,
          epoch: this.live.epoch,
          last_seq: this.seq,
          ended_mono_ns: String(this.cutoff)
        };
        this.pump();
      };
      if (
        this.live.enabled.has('vision') ||
        this.live.record.phase === 'finalizing' ||
        this.live.record.index_state === 'building'
      )
        this.drainTimer = setTimeout(finish, this.live.d.finalize);
      else finish();
    }
  }
  append(kind, payload) {
    if (!this.live.session || this.end) return;
    if (this.finishing) {
      const lateVision =
        kind === 'sample' &&
        payload.source === 'vision' &&
        payload.obs_ns &&
        BigInt(payload.obs_ns) <= this.cutoff;
      if (
        !lateVision &&
        !(kind === 'event' && (payload.module === 'video' || payload.type === 'alert_close'))
      )
        return;
    }
    const seq = ++this.seq,
      now = Date.now();
    const line =
      JSON.stringify({ kind, v: 2, session_id: this.live.session, epoch: this.live.epoch, seq, payload }) +
      '\n';
    const size = Buffer.byteLength(line);
    if (size <= maxLine) {
      this.queue.set(seq, { line, size, now, kind });
      this.bytes += size;
    }
    for (const [id, e] of this.queue) {
      if (now - e.now <= 60000) break;
      this.bytes -= e.size;
      this.queue.delete(id);
    }
    while (this.queue.size > 32768 || this.bytes > 16 * 1024 * 1024) {
      let first = this.queue.keys().next().value;
      // Preserve events under pressure while a later expendable item is available.
      if (this.queue.get(first).kind === 'event')
        for (const [id, e] of this.queue)
          if (e.kind !== 'event') {
            first = id;
            break;
          }
      this.bytes -= this.queue.get(first).size;
      this.queue.delete(first);
    }
    this.pump();
  }
  pump() {
    const s = this.peer;
    if (!s || this.blocked || s.destroyed) return;
    while (this.cursor < this.seq) {
      const id = this.cursor + 1,
        e = this.queue.get(id);
      if (e) {
        this.cursor = id;
        this.blocked = !s.write(e.line);
      } else {
        let next = id + 1;
        while (next <= this.seq && !this.queue.has(next)) next++;
        this.cursor = next - 1;
        this.blocked = !send(s, {
          v: 2,
          kind: 'gap',
          epoch: this.live.epoch,
          first_missing: id,
          last_missing: this.cursor,
          reason: 'buffer_expired'
        });
      }
      if (this.blocked) return;
    }
    if (this.end && this.sentEnd !== s) {
      send(s, this.end);
      this.sentEnd = s;
    }
  }
  start() {
    if ((this.peer && !this.peer.destroyed) || this.child || this.requested)
      return { alreadyRunning: true, session_id: this.live.session };
    if (this.listenerError) throw new Error(this.listenerError);
    this.requested = true;
    this.live.writer = {
      ...this.live.writer,
      state: 'starting',
      at: this.live.clock(),
      drop_total: this.live.writer.drop_total ?? 0
    };
    const child = spawn('bash', [path.join(this.root, 'scripts/experiment_logger/run.sh')], {
      stdio: ['ignore', 'ignore', 'pipe'],
      env: process.env
    });
    this.child = child;
    let error = '';
    child.stderr.on('data', (buf) => {
      error = (error + buf.toString()).slice(-512);
    });
    const ended = (reason) => {
      if (this.child !== child) return;
      this.child = null;
      this.requested = false;
      if (reason) {
        this.live.writer = { ...this.live.writer, state: 'failed' };
        this.live.alert('logger', 'LOG_WRITE_FAILED', 'error', '记录器启动或运行失败：' + reason);
      }
    };
    child.once('error', (e) => ended(e.message));
    child.once('exit', (code, signal) => ended(code || signal ? error || signal || String(code) : null));
    return { session_id: null, state: 'starting' };
  }
  stop() {
    if (this.peer && !this.peer.destroyed)
      send(this.peer, { kind: 'request_finish', v: 2, session_id: this.live.session });
    else if (this.child) this.child.kill('SIGTERM');
    else if (this.live.session) throw new Error('记录器离线，等待重连后可结束记录');
    else if (this.live.writer.state === 'failed') {
      this.live.closeAlert('LOG_WRITE_FAILED', 'cancelled');
      this.live.writer = {};
    }
    return {
      session_id: this.live.session,
      state: this.live.session || this.child ? 'finalizing' : 'idle'
    };
  }
  close() {
    clearTimeout(this.drainTimer);
    this.peer?.destroy();
    this.server?.close();
  }
}
