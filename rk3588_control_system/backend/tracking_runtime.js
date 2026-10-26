import fs from 'node:fs';
import path from 'node:path';

export function resolveTrackingPython(projectRoot, environment = process.env) {
  if (environment.PYTHON_EXEC) return environment.PYTHON_EXEC;
  for (const relative of ['.venv/bin/python', '.venv/Scripts/python.exe']) {
    const candidate = path.join(projectRoot, relative);
    try {
      fs.accessSync(candidate, fs.constants.X_OK);
      return candidate;
    } catch (_) {}
  }
  return 'python3';
}

// Spawning a process is not evidence that its detector initialized. Both
// workers report ready only after importing dependencies and loading weights.
export function waitForTrackerReady(child, timeoutMs = 30000) {
  return new Promise((resolve, reject) => {
    let output = '', errors = '', settled = false;
    const finish = (error, event) => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);
      child.stdout.off('data', onOutput);
      child.stderr.off('data', onErrorOutput);
      child.off('error', onError);
      child.off('exit', onExit);
      if (error) reject(error); else resolve(event);
    };
    const onOutput = data => {
      output += String(data);
      let end;
      while ((end = output.indexOf('\n')) >= 0) {
        const line = output.slice(0, end);
        output = output.slice(end + 1);
        try {
          const event = JSON.parse(line);
          if (event.event === 'ready') { finish(null, event); return; }
        } catch (_) {}
      }
      if (output.length > 65536) output = output.slice(-65536);
    };
    const onErrorOutput = data => { errors = (errors + String(data)).slice(-4096); };
    const onError = error => finish(new Error(`Tracker could not start: ${error.message}`));
    const onExit = (code, signal) => {
      const detail = errors.trim().split('\n').filter(Boolean).slice(-2).join(' ');
      finish(new Error(`Tracker initialization failed (${signal || code})${detail ? ': ' + detail : ''}`));
    };
    const timer = setTimeout(() => {
      finish(new Error(`Tracker did not become ready within ${timeoutMs} ms; check Python dependencies and model files`));
      child.kill('SIGTERM');
    }, timeoutMs);
    child.stdout.on('data', onOutput);
    child.stderr.on('data', onErrorOutput);
    child.once('error', onError);
    child.once('exit', onExit);
  });
}
