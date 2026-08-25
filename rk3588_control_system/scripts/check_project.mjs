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
const notices = [];

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

function commandWorks(command, args) {
  const result = spawnSync(command, args, { cwd: PROJECT_ROOT, encoding: 'utf8' });
  return !result.error && result.status === 0;
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

const pythonCommand = process.platform === 'win32'
  ? null
  : ['python3', 'python'].find((command) => commandWorks(command, ['--version']));
if (pythonCommand) {
  for (const relativeDir of PYTHON_DIRS) {
    for (const filePath of walk(path.join(PROJECT_ROOT, relativeDir))) {
      if (!filePath.endsWith('.py')) continue;
      runCheck(pythonCommand, ['-c', pythonSyntaxCheck, filePath], path.relative(PROJECT_ROOT, filePath));
    }
  }
} else {
  notices.push('Python syntax check skipped because no working Python interpreter is available.');
}

const shellFiles = [
  ...ROOT_SHELL_FILES.map((file) => path.join(PROJECT_ROOT, file)),
  ...walk(path.join(PROJECT_ROOT, 'scripts')).filter((file) => file.endsWith('.sh')),
  path.join(PROJECT_ROOT, 'scripts', '99-manta-gimbal-route'),
];

const bashAvailable = process.platform !== 'win32' && commandWorks('bash', ['--version']);
if (bashAvailable) {
  for (const filePath of shellFiles) {
    if (!fs.existsSync(filePath)) continue;
    runCheck('bash', ['-n', filePath], path.relative(PROJECT_ROOT, filePath));
  }
} else {
  notices.push('Shell syntax check skipped because no working Bash interpreter is available.');
}

for (const relativePath of [
  'frontend/assets/manta-app/manta-hero.jpg',
  'frontend/assets/manta-app/gimbal-demo.mp4',
  'frontend/assets/offline-map/manifest.json',
  'frontend/js/gps-map-core.js',
  'frontend/js/offline-satellite-map.js',
  'scripts/generate_device_env.py',
  'scripts/manta_doctor.py',
  'scripts/python_service.sh',
]) {
  const absolutePath = path.join(PROJECT_ROOT, relativePath);
  if (!fs.existsSync(absolutePath) || fs.statSync(absolutePath).size === 0) {
    failures.push(`${relativePath}: required portable-deployment asset is missing or empty`);
  }
}

const mapHtml = fs.readFileSync(path.join(PROJECT_ROOT, 'frontend', 'map.html'), 'utf8');
if (/leaflet|tile\.openstreetmap|unpkg\.com/i.test(mapHtml)) {
  failures.push('frontend/map.html: map runtime must not depend on an external CDN or tile server');
}

const quickstart = fs.readFileSync(path.join(PROJECT_ROOT, 'quickstart.sh'), 'utf8');
for (const requiredEntry of ['install_mediamtx.sh', 'manta_doctor.py --installed', '--offline', '--skip-boot-config']) {
  if (!quickstart.includes(requiredEntry)) failures.push(`quickstart.sh: missing ${requiredEntry}`);
}

const backendTemplate = fs.readFileSync(path.join(SYSTEMD_DIR, 'manta-backend.service.template'), 'utf8');
if (!backendTemplate.includes('EnvironmentFile=-/etc/manta/manta.env')) {
  failures.push('systemd/manta-backend.service.template: missing map credential environment file');
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

for (const template of ['manta-bridge', 'manta-camera', 'manta-captive-portal', 'manta-gimbal-stream', 'manta-bluetooth-pan']) {
  const content = fs.readFileSync(path.join(SYSTEMD_DIR, `${template}.service.template`), 'utf8');
  if (!content.includes('scripts/python_service.sh')) {
    failures.push(`systemd/${template}.service.template: must use scripts/python_service.sh`);
  }
}

for (const template of ['manta-hotspot', 'manta-bluetooth-pan']) {
  const content = fs.readFileSync(path.join(SYSTEMD_DIR, `${template}.service.template`), 'utf8');
  if (!content.includes('EnvironmentFile=-/etc/manta/manta.env')) {
    failures.push(`systemd/${template}.service.template: missing per-device environment file`);
  }
}

for (const notice of notices) console.warn(`[check] ${notice}`);

if (failures.length) {
  console.error('[check] Failed');
  for (const failure of failures) console.error(`  - ${failure}`);
  process.exit(1);
}

const checkedLanguages = ['JavaScript', 'JSON', 'systemd templates'];
if (pythonCommand) checkedLanguages.push('Python');
if (bashAvailable) checkedLanguages.push('Shell');
console.log(`[check] ${checkedLanguages.join(', ')} checks passed.`);
