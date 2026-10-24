// Legacy behavioral baseline; replacement is checked by check_native_*.py/mjs.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import { fileURLToPath } from 'node:url';

const projectRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const server = fs.readFileSync(path.join(projectRoot, 'test', 'fixtures', 'legacy-server-control.txt'), 'utf8');

test('UWB holding leaves ACRO closed-loop control and verifies real neutral outputs', () => {
  assert.match(server, /SET_MODE', \{ mode: 'MANUAL', reason: 'uwb_holding_distance' \}/);
  assert.match(server, /holdOutputsAreNeutral\(\)/);
  assert.match(server, /holding_output_not_neutral/);
  assert.match(server, /UWB_HOLD_PWM_TOLERANCE/);
});

test('UWB following restores the required mode before propulsion resumes', () => {
  assert.match(server, /status: 'resuming'/);
  assert.match(server, /reason: 'uwb_resume_tracking'/);
  assert.match(server, /UWB_RESUME_MODE_TIMEOUT_MS/);
});
