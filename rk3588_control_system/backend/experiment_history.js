import { spawn } from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';

const prefix = '/api/experiments/sessions';
const maxMetadataBytes = 2 * 1024 * 1024;
const historyUnavailable = { success: false, code: 'HISTORY_UNAVAILABLE', message: '无法读取该实验数据' };

function expandHome(value) {
  return value === '~' || value.startsWith('~/') ? path.join(os.homedir(), value.slice(1)) : value;
}

export function readExperimentLoggerConfig(projectRoot, filename = process.env.MANTA_LOGGER_CONFIG || 'config.json') {
  const loggerDirectory = path.join(projectRoot, 'scripts', 'experiment_logger');
  const config = JSON.parse(fs.readFileSync(path.resolve(loggerDirectory, expandHome(filename)), 'utf8'));
  const directory = path.resolve(loggerDirectory, expandHome(config.directory));
  const socket = process.env.MANTA_JOURNAL_SOCKET || config.journal_socket
    || path.join(path.dirname(directory), 'run', 'experiment.sock');
  return { ...config, directory, journal_socket: path.resolve(loggerDirectory, expandHome(socket)) };
}

export function installExperimentHistory(app, { projectRoot, recordingsDir, loggerConfig = readExperimentLoggerConfig(projectRoot),
  python = process.env.MANTA_LOGGER_PYTHON || 'python3' }) {
  const script = path.join(projectRoot, 'scripts', 'experiment_logger', 'history_query.py');
  let active = 0;
  function handle(action) {
    return (req, res) => {
      if (active >= 4) {
        res.status(503).json({ success: false, code: 'HISTORY_BUSY', message: '历史查询繁忙，请稍后重试' });
        return;
      }
      const command = { action, query: req.query, session: req.params.id, kind: req.params.kind };
      const child = spawn(python, ['-u', script, loggerConfig.directory, recordingsDir || '', JSON.stringify(command)],
        { stdio: ['ignore', 'pipe', 'ignore'] });
      active += 1;
      trackResponse(child, res, action, () => { active -= 1; });
    };
  }
  app.get(prefix, handle('sessions'));
  app.get(`${prefix}/:id`, handle('manifest'));
  app.get(`${prefix}/:id/events`, handle('events'));
  app.get(`${prefix}/:id/files/:kind`, handle('download'));
  app.get(`${prefix}/:id/locate`, handle('locate'));
}

function trackResponse(child, res, action, release) {
  let metadata = Buffer.alloc(0);
  let handled = false;
  const timer = setTimeout(() => {
    child.kill();
    if (res.writableEnded || res.destroyed) return;
    if (res.headersSent) res.destroy();
    else res.status(503).json(historyUnavailable);
  }, 120_000);
  timer.unref();
  res.on('close', () => { if (!child.killed) child.kill(); });
  child.on('error', () => { if (!res.headersSent && !res.destroyed) res.status(500).json(historyUnavailable); });
  child.on('close', () => {
    clearTimeout(timer);
    release();
    if (!handled && !res.headersSent && !res.destroyed) res.status(500).json(historyUnavailable);
  });

  function handleMetadata(chunk) {
    const data = Buffer.concat([metadata, chunk]);
    const end = data.indexOf(10);
    if (end < 0 && data.length <= maxMetadataBytes) {
      metadata = data;
      return;
    }
    child.stdout.off('data', handleMetadata);
    handled = true;
    let header;
    try {
      if (end < 0 || end > maxMetadataBytes) throw new Error('Invalid history metadata');
      header = JSON.parse(data.subarray(0, end).toString('utf8'));
    } catch {
      child.kill();
      if (!res.destroyed) res.status(500).json(historyUnavailable);
      return;
    }
    if (res.destroyed) return;
    if (!header.success) {
      const { status, ...error } = header;
      res.status(status || 500).json(error);
      return;
    }
    if (action !== 'download') {
      res.json(header);
      return;
    }
    if (!['state_overview.csv', 'events.csv', 'video_aligned.csv'].includes(header.data.csv)) {
      res.status(500).json(historyUnavailable);
      return;
    }
    res.set('Content-Type', 'text/csv; charset=utf-8');
    res.attachment(header.data.csv);
    const rest = data.subarray(end + 1);
    if (rest.length && !res.write(rest)) {
      child.stdout.pause();
      res.once('drain', () => child.stdout.resume());
    }
    child.stdout.pipe(res);
  }
  child.stdout.on('data', handleMetadata);
}
