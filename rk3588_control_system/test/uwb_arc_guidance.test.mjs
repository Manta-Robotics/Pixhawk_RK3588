import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import { fileURLToPath } from 'node:url';

const projectRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const server = fs.readFileSync(path.join(projectRoot, 'backend', 'server.js'), 'utf8');
const controller = fs.readFileSync(path.join(projectRoot, 'backend', 'uwb_follow.js'), 'utf8');

test('UWB guidance uses ILOS lookahead and bounded curvature', () => {
  assert.match(controller, /crossTrackIntegralMs/);
  assert.match(controller, /ilosCorrectionDeg/);
  assert.match(controller, /curvatureMInv/);
  assert.match(controller, /minimumCurveSpeedScale/);
});

test('relative UWB mixer enforces same-direction differential thrust', () => {
  assert.match(server, /sameDirectionSteeringLimit = Math\.abs\(requested\.throttle\) \* steeringSpan \/ throttleSpan/);
  assert.match(server, /clamp\(requested\.steering, -sameDirectionSteeringLimit, sameDirectionSteeringLimit\)/);
});
