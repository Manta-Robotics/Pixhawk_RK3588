import fs from 'node:fs';
import path from 'node:path';
import { randomUUID } from 'node:crypto';
import { spawn } from 'node:child_process';

export class GimbalRecording {
  constructor({ alignment, binary, startupTimeoutMs = 5000, stallTimeoutMs = 8000, segmentSeconds = 120,
    onChange, log }) {
    Object.assign(this, { alignment, binary, startupTimeoutMs, stallTimeoutMs, segmentSeconds, onChange, log });
    this.run = null;
    this.phase = 'idle';
    this.clip = null;
    this.current = null;
    this.clips = new Map();
    this.reason = null;
    this.revision = 0;
    this.instanceId = randomUUID();
  }

  state() {
    const active = ['starting', 'recording'].includes(this.phase);
    return { phase: this.phase, active, desired: active, recording_id: this.clip?.recording_id || null,
      clip_id: this.clip?.clip_id || null, clip_no: this.clip?.clip_no || null,
      index_state: this.clip?.index_state || 'none', reason: this.reason || this.clip?.reason || null,
      instanceId: this.instanceId, revision: this.revision,
      current: this.current ? { ...this.current, elapsedMs: Math.max(0, (this.run ? Date.now() : this.endedAt) - this.startedAt) } : null };
  }

  publish() { this.revision++; this.onChange(this.state()); }

  isOpen(name) { return Boolean(this.run && this.run.fileNames.has(name) && this.clips.get(name)?.phase !== 'ready'); }

  start({ outputPath, input, codec, bitrate, transport }) {
    const id = randomUUID(), name = path.basename(outputPath);
    const title = path.basename(name, '.mp4');
    const session = { id, prefix: `${title.slice(0, 55)}_${id.slice(0, 8)}`, alignmentSessionId: this.alignment.getSessionId() };
    const command = this.alignment.command(session, 1, this);
    const args = ['-hide_banner', '-loglevel', 'warning', '-n', '-progress', 'pipe:1', '-nostats'];
    if (input.startsWith('rtsp://')) args.push('-rtsp_transport', transport);
    args.push('-i', input, '-map', '0:v:0', '-an', '-c:v', codec);
    if (codec !== 'copy' && bitrate) args.push('-b:v', bitrate);
    args.push('-movflags', '+faststart', outputPath);
    const child = spawn(command?.binary || this.binary, command?.args || args, { stdio: ['ignore', 'pipe', 'pipe'] });
    const run = { child, session, closing: false, aligned: Boolean(command), fileNames: new Set(),
      lastProgress: Date.now(), frames: 0, pending: '' };
    run.closed = new Promise(resolve => { run.resolve = resolve; });
    const firstFile = command ? `${session.prefix}__000001.mp4` : name;
    run.fileNames.add(firstFile);
    this.run = run;
    this.startedAt = Date.now();
    this.reason = null;
    this.current = { name: firstFile, title, input, startedAt: new Date(this.startedAt).toISOString() };
    this.clip = { recording_id: id, clip_id: `${id}-000001`, clip_no: 1, file_name: firstFile, index_state: 'none' };
    this.phase = 'starting';
    this.publish();
    child.stdout.setEncoding('utf8');
    child.stdout.on('data', chunk => {
      if (this.run !== run) return;
      run.pending += chunk;
      const lines = run.pending.split('\n');
      run.pending = lines.pop();
      for (const line of lines) {
        if (line.startsWith('alignment=')) {
          try {
            const { kind, clip } = JSON.parse(line.slice(10));
            if (clip.recording_id === id) this.updateClip(run, clip, kind);
          } catch (error) { this.log(error.message); }
        } else if (line.startsWith('frame=') && Number(line.slice(6)) > run.frames) {
          run.frames = Number(line.slice(6));
          run.lastProgress = Date.now();
          if (!run.closing && this.phase === 'starting') { this.phase = 'recording'; this.publish(); }
        }
      }
    });
    child.stderr.on('data', chunk => { run.lastError = String(chunk).slice(-2000); });
    child.on('error', error => { run.lastError = error.message; });
    child.once('close', async code => {
      clearInterval(run.monitor); clearTimeout(run.killTimer);
      if (this.run !== run) { run.resolve(this.state()); return; }
      this.phase = 'finalizing';
      if (!run.closing && input.startsWith('rtsp://')) this.reason = { code: 'RECORD_FAILED',
        message: run.lastError?.trim() || `视频录制进程意外退出 (${code})` };
      this.publish();
      if (!run.aligned && run.frames > 0 && fs.existsSync(outputPath)) {
        this.clip.index_state = 'building';
        this.publish();
        const clip = await new Promise(resolve => this.alignment.legacyIndex(session, { name, index: 1 }, resolve));
        this.updateClip(run, clip, 'clip_ready');
      }
      if (this.run !== run) { run.resolve(this.state()); return; }
      this.endedAt = Date.now();
      this.run = null;
      const mediaReady = [...run.fileNames].some(file => this.clips.get(file)?.phase === 'ready');
      this.phase = mediaReady && !this.reason ? 'ready' : 'failed';
      if (this.phase === 'failed' && !this.reason) this.reason = { code: 'RECORD_FAILED',
        message: run.lastError?.trim() || `录制进程退出，尚无可播放录像 (${code})` };
      this.publish();
      run.resolve(this.state());
    });
    run.monitor = setInterval(() => {
      if (this.run !== run || run.closing) return;
      const timeout = run.frames ? this.stallTimeoutMs : this.startupTimeoutMs;
      if (Date.now() - run.lastProgress <= timeout) return;
      this.reason = { code: 'RECORD_FAILED', message: '未收到可写入的视频帧' };
      void this.stop();
    }, 100);
    run.monitor.unref();
    return this.state();
  }

  updateClip(run, clip, kind) {
    if (this.run !== run) return;
    run.fileNames.add(clip.file_name);
    this.clips.set(clip.file_name, clip);
    if (clip.clip_no >= this.clip.clip_no) {
      this.clip = clip;
      this.current.name = clip.file_name;
    }
    this.publish();
    if (kind === 'clip_ready') this.alignment.onClip(clip, kind);
  }

  stop() {
    const run = this.run;
    if (!run) return Promise.resolve(this.state());
    if (!run.closing) {
      run.closing = true;
      this.phase = 'finalizing';
      this.publish();
      run.child.kill('SIGINT');
      run.killTimer = setTimeout(() => run.child.kill('SIGKILL'), Math.max(10000, this.stallTimeoutMs + 1000));
      run.killTimer.unref();
    }
    return run.closed;
  }
}
