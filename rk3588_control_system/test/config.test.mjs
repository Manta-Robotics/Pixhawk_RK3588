import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import { fileURLToPath } from 'node:url';
import { validateSystemConfig } from '../scripts/validate_config.mjs';

const PROJECT_ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const config = JSON.parse(fs.readFileSync(path.join(PROJECT_ROOT, 'config', 'system.config.json'), 'utf8'));

test('repository system configuration is valid', () => {
  assert.deepEqual(validateSystemConfig(config), []);
});

test('unsafe hotspot credentials are rejected', () => {
  const candidate = structuredClone(config);
  candidate.hotspot.password = 'short';
  assert.ok(validateSystemConfig(candidate).some((error) => error.includes('hotspot.password')));
});

test('invalid face hold hysteresis is rejected', () => {
  const candidate = structuredClone(config);
  candidate.gimbal.face.track_hold_enter_x_px = candidate.gimbal.face.track_hold_exit_x_px;
  assert.ok(validateSystemConfig(candidate).some((error) => error.includes('horizontal hold')));
});
