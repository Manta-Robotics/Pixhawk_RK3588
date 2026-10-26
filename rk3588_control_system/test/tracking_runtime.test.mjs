import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import test from 'node:test';
import { resolveTrackingPython, waitForTrackerReady } from '../backend/tracking_runtime.js';

test('trackers use installed project dependencies and preserve explicit interpreter overrides', t => {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), 'manta-tracking-python-'));
  t.after(() => fs.rmSync(root, { recursive: true, force: true }));
  const interpreter = path.join(root, '.venv/bin/python');
  fs.mkdirSync(path.dirname(interpreter), { recursive: true });
  fs.writeFileSync(interpreter, '', { mode: 0o755 });
  assert.equal(resolveTrackingPython(root, {}), interpreter);
  assert.equal(resolveTrackingPython(root, { PYTHON_EXEC: '/custom/python' }), '/custom/python');
});

test('worker startup waits for a complete ready record rather than a spawn or fallback log', async () => {
  const child = spawn(process.execPath, ['-e', `
    console.log(JSON.stringify({event:'detector_fallback'}));
    process.stdout.write('{"event":"rea');
    setTimeout(()=>console.log('dy","detector":"yolo_face"}'),40);
  `]);
  const event = await waitForTrackerReady(child, 3000);
  assert.equal(event.detector, 'yolo_face');
});

test('missing dependencies reject startup with the worker error', async () => {
  const child = spawn(process.execPath, ['-e', `console.error("ModuleNotFoundError: No module named 'ultralytics'"); process.exit(1)`]);
  await assert.rejects(waitForTrackerReady(child, 3000), /ultralytics/);
});

test('missing Python and stalled initialization reject instead of returning success', async () => {
  const missing = spawn(path.join(os.tmpdir(), 'manta-nonexistent-tracking-python'), []);
  await assert.rejects(waitForTrackerReady(missing, 3000), /could not start/);
  const stalled = spawn(process.execPath, ['-e', 'setInterval(()=>{},1000)']);
  await assert.rejects(waitForTrackerReady(stalled, 100), /did not become ready/);
});
