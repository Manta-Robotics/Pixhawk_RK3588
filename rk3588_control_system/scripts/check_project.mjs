#!/usr/bin/env node

import fs from 'fs';
import path from 'path';
import { spawnSync } from 'child_process';
import { fileURLToPath } from 'url';
import { loadAndValidateConfig } from './validate_config.mjs';

const PROJECT_ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const SOURCE_DIRS = ['backend', path.join('frontend', 'js'), 'scripts'];
const PYTHON_DIRS = ['backend', 'scripts'];
const ROOT_SHELL_FILES = ['quickstart.sh', 'SETUP_HELP.sh', 'start.sh', 'stop.sh'];
const SYSTEMD_DIR = path.join(PROJECT_ROOT, 'systemd');
const failures = [];

function walk(directory) {
  if (!fs.existsSync(directory)) return [];
  return fs.readdirSync(directory, { withFileTypes: true }).flatMap((entry) => {
    const fullPath = path.join(directory, entry.name);
    return entry.isDirectory() ? walk(fullPath) : [fullPath];
  });
}

function runCheck(command, args, label) {
  const result = spawnSync(command, args, {
    cwd: PROJECT_ROOT,
    encoding: 'utf8',
  });

  if (result.error) {
    failures.push(`${label}: unable to run ${command}: ${result.error.message}`);
    return;
  }

  if (result.status !== 0) {
    const details = [result.stdout, result.stderr].filter(Boolean).join('\n').trim();
    failures.push(details ? `${label}: ${details}` : label);
  }
}

for (const relativeDir of SOURCE_DIRS) {
  for (const filePath of walk(path.join(PROJECT_ROOT, relativeDir))) {
    if (!/\.(?:js|mjs)$/.test(filePath)) continue;
    const result = spawnSync(process.execPath, ['--check', filePath], { stdio: 'inherit' });
    if (result.status !== 0) failures.push(path.relative(PROJECT_ROOT, filePath));
  }
}

const pythonSyntaxCheck = [
  'import ast',
  'import pathlib',
  'import sys',
  'path = pathlib.Path(sys.argv[1])',
  'ast.parse(path.read_text(encoding="utf-8"), filename=str(path))',
].join('; ');

for (const relativeDir of PYTHON_DIRS) {
  for (const filePath of walk(path.join(PROJECT_ROOT, relativeDir))) {
    if (!filePath.endsWith('.py')) continue;
    runCheck('python3', ['-c', pythonSyntaxCheck, filePath], path.relative(PROJECT_ROOT, filePath));
  }
}

const shellFiles = [
  ...ROOT_SHELL_FILES.map((file) => path.join(PROJECT_ROOT, file)),
  ...walk(path.join(PROJECT_ROOT, 'scripts')).filter((file) => file.endsWith('.sh')),
  path.join(PROJECT_ROOT, 'scripts', '99-manta-gimbal-route'),
];

for (const filePath of shellFiles) {
  if (!fs.existsSync(filePath)) continue;
  runCheck('bash', ['-n', filePath], path.relative(PROJECT_ROOT, filePath));
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

for (const filePath of walk(SYSTEMD_DIR).filter((file) => file.endsWith('.service.template'))) {
  const relativePath = path.relative(PROJECT_ROOT, filePath);
  const content = fs.readFileSync(filePath, 'utf8');
  for (const requiredEntry of ['[Unit]', '[Service]', 'ExecStart=', '__PROJECT_DIR__']) {
    if (!content.includes(requiredEntry)) {
      failures.push(`${relativePath}: missing ${requiredEntry}`);
    }
  }
}

if (failures.length) {
  console.error('[check] Failed');
  for (const failure of failures) console.error(`  - ${failure}`);
  process.exit(1);
}

console.log('[check] JavaScript, Python, Shell, JSON, and systemd template checks passed.');
