#!/usr/bin/env node

import fs from 'fs';
import path from 'path';
import { spawnSync } from 'child_process';
import { fileURLToPath } from 'url';
import { loadAndValidateConfig } from './validate_config.mjs';

const PROJECT_ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const SOURCE_DIRS = ['backend', path.join('frontend', 'js'), 'scripts'];
const failures = [];

function walk(directory) {
  if (!fs.existsSync(directory)) return [];
  return fs.readdirSync(directory, { withFileTypes: true }).flatMap((entry) => {
    const fullPath = path.join(directory, entry.name);
    return entry.isDirectory() ? walk(fullPath) : [fullPath];
  });
}

for (const relativeDir of SOURCE_DIRS) {
  for (const filePath of walk(path.join(PROJECT_ROOT, relativeDir))) {
    if (!/\.(?:js|mjs)$/.test(filePath)) continue;
    const result = spawnSync(process.execPath, ['--check', filePath], { stdio: 'inherit' });
    if (result.status !== 0) failures.push(path.relative(PROJECT_ROOT, filePath));
  }
}

for (const relativePath of ['config/system.config.json', 'config/motor_config.json', 'config/bluetooth.config.json']) {
  try {
    JSON.parse(fs.readFileSync(path.join(PROJECT_ROOT, relativePath), 'utf8'));
  } catch (error) {
    failures.push(`${relativePath}: ${error.message}`);
  }
}

try {
  const result = loadAndValidateConfig(path.join(PROJECT_ROOT, 'config', 'system.config.json'));
  failures.push(...result.errors.map((error) => `config/system.config.json: ${error}`));
} catch (error) {
  failures.push(`config/system.config.json: ${error.message}`);
}

if (failures.length) {
  console.error('[check] Failed');
  for (const failure of failures) console.error(`  - ${failure}`);
  process.exit(1);
}

console.log('[check] JavaScript syntax and JSON configuration are valid.');
