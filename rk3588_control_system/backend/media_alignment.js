import fs from 'node:fs';
import path from 'node:path';
import http from 'node:http';
import { spawn } from 'node:child_process';
import { randomUUID } from 'node:crypto';

export function createPreviewDemand(onChange) {
  const browsers = new Set();
  let requests = 0;
  let previous = false;
  function update() {
    const demanded = requests > 0 || browsers.size > 0;
    if (demanded !== previous) { previous = demanded; onChange(demanded); }
  }
  return {
    trackResponse(response) {
      requests++;
      response.once('close', () => { requests--; update(); });
      update();
    },
    trackSocket(socket) {
      socket.on('experiment_preview', payload => {
        const active = payload?.active;
        if (active === true) browsers.add(socket.id);
        else if (active === false) browsers.delete(socket.id);
        update();
      });
      socket.once('disconnect', () => { browsers.delete(socket.id); update(); });
    }
  };
}

export function subscribeMediaFrames(url, onFrame) {
  let stopped = false;
  let request;
  let timer;
  function reconnect() {
    if (stopped || timer) return;
    timer = setTimeout(() => { timer = null; connect(); }, 500);
    timer.unref();
  }
  function connect() {
    request = http.get(url, { timeout: 4000 }, response => {
      if (response.statusCode !== 200) { response.resume(); reconnect(); return; }
      response.setEncoding('utf8');
      let pending = '';
      response.on('data', chunk => {
        pending += chunk;
        if (pending.length > 65536) { request.destroy(); return; }
        const lines = pending.split('\n');
        pending = lines.pop();
        for (const line of lines) {
          if (!line) continue;
          try {
            const sample = JSON.parse(line);
            if (sample.frame_id && sample.stream_epoch && /^\d+$/.test(sample.obs_ns)) onFrame(sample);
          } catch (_) { /* Drop malformed source metadata. */ }
        }
      });
      response.once('close', reconnect);
      response.once('end', reconnect);
      response.once('error', reconnect);
    });
    request.on('timeout', () => request.destroy());
    request.on('error', reconnect);
    request.on('close', reconnect);
  }
  connect();
  return () => { stopped = true; clearTimeout(timer); request?.destroy(); };
}

export class MediaAlignment {
  constructor({ projectRoot, indexDirectory, directory, python, source, codec, fps, ffprobe, bootId,
    getSessionId = () => null, onClip = () => {} }) {
    Object.assign(this, { projectRoot, indexDirectory, directory, python, source, codec, fps, ffprobe,
      bootId, getSessionId, onClip });
    this.script = path.join(projectRoot, 'scripts', 'record_aligned_video.py');
  }

  commonArgs(session, index, provenSession = null) {
    const args = ['-u', this.script, '--recording-id', session.id, '--index-directory', this.indexDirectory,
      '--directory', this.directory, '--prefix', session.prefix, '--start-index', String(index)];
    if (this.bootId) args.push('--boot-id', this.bootId);
    if (provenSession) args.push('--session-id', provenSession);
    if (this.ffprobe) args.push('--ffprobe', this.ffprobe);
    return args;
  }

  command(session, index, recorder) {
    if (!this.source) return null;
    return { binary: this.python, args: [...this.commonArgs(session, index), '--source', this.source,
      '--segment-seconds', String(recorder.segmentSeconds), '--stall-timeout', String(recorder.stallTimeoutMs / 1000),
      '--codec', this.codec, '--fps', String(this.fps)] };
  }

  legacyIndex(session, part, onResult) {
    const provenSession = this.getSessionId() === session.alignmentSessionId ? session.alignmentSessionId : null;
    const args = [...this.commonArgs(session, part.index, provenSession), '--legacy-file', path.join(this.directory, part.name)];
    const child = spawn(this.python, args, { stdio: ['ignore', 'pipe', 'ignore'] });
    let output = '';
    child.stdout.on('data', chunk => { if (output.length < 262144) output += chunk; });
    child.on('error', () => {});
    child.once('close', () => {
      const last = output.trim().split('\n').at(-1);
      let clip;
      try { if (last?.startsWith('alignment=')) clip = JSON.parse(last.slice(10)).clip; } catch (_) {}
      onResult(clip || { recording_id: session.id, clip_id: `${session.id}-${String(part.index).padStart(6, '0')}`, clip_no: part.index,
        file_name: part.name, phase: 'failed', index_state: 'failed',
        reason: { code: 'VIDEO_INDEX_FAILED', message: '录像探测进程未返回结果' } });
    });
  }

  async updateFile(clipId, fileName) {
    if (!/^[a-zA-Z0-9_-]{1,100}$/.test(clipId)) return;
    const target = path.join(this.indexDirectory, `${clipId}.json`);
    let clip;
    try { clip = JSON.parse(await fs.promises.readFile(target, 'utf8')); }
    catch (error) { if (error.code === 'ENOENT') return; throw error; }
    if (fileName) clip.file_name = fileName;
    clip.media_missing = !fileName;
    const temporary = `${target}.${randomUUID()}.tmp`;
    try {
      await fs.promises.writeFile(temporary, JSON.stringify(clip), { flag: 'wx', mode: 0o600 });
      await fs.promises.rename(temporary, target);
    } finally { await fs.promises.rm(temporary, { force: true }); }
  }

  async lookupFile(fileName) {
    let files;
    try { files = await fs.promises.readdir(this.indexDirectory); }
    catch (error) { if (error.code === 'ENOENT') return null; throw error; }
    for (const file of files.filter(name => /^[a-zA-Z0-9_-]{1,100}\.json$/.test(name))) {
      let clip;
      try { clip = JSON.parse(await fs.promises.readFile(path.join(this.indexDirectory, file), 'utf8')); }
      catch (_) { continue; }
      if (clip.file_name === fileName && !clip.media_missing) {
        const { frames: _frames, gaps: _gaps, ...summary } = clip;
        return summary;
      }
    }
    return null;
  }
}
