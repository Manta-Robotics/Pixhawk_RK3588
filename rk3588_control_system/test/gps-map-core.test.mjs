import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import vm from 'node:vm';
import { fileURLToPath } from 'node:url';

const projectRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const source = fs.readFileSync(path.join(projectRoot, 'frontend', 'js', 'gps-map-core.js'), 'utf8');
const context = vm.createContext({});
vm.runInContext(source, context);
const gps = context.MantaGpsMapCore;

test('GPS fix type, not satellite count, determines validity', () => {
  const location = gps.normalizeTelemetry({
    position: { lat: 22.3, lon: 114.17, alt: 2, updatedAt: 1_000_000 },
    gps: { fixType: 1, satellites: 12, updatedAt: 1_000_000 }
  }, 1_001_000);
  assert.equal(location.fixValid, false);
  assert.equal(location.status, 'unavailable');
});

test('fresh GPS_RAW_INT coordinates are preferred over fused position', () => {
  const location = gps.normalizeTelemetry({
    position: { lat: 1, lon: 2, alt: 3, source: 'GLOBAL_POSITION_INT', updatedAt: 1_000_000 },
    gps: {
      fixType: 3, satellites: 8, latitude: 22.3, longitude: 114.17, altitude: 4,
      horizontalAccuracy: 1.2, groundSpeed: 1, course: 270, updatedAt: 1_000_000
    },
    attitude: { yaw: 90 }
  }, 1_001_000);
  assert.equal(location.status, 'available');
  assert.equal(location.source, 'GPS_RAW_INT');
  assert.equal(location.latitude, 22.3);
  assert.equal(location.heading, 270);
});

test('GPS data older than five seconds is stale', () => {
  const location = gps.normalizeTelemetry({
    gps: { fixType: 3, latitude: 22.3, longitude: 114.17, updatedAt: 1_700_000_000_000 }
  }, 1_700_000_006_001);
  assert.equal(location.fixValid, true);
  assert.equal(location.fresh, false);
  assert.equal(location.status, 'stale');
});

test('coordinates without a source timestamp are not treated as fresh', () => {
  const normalized = gps.normalizeTelemetry({
    gps: { fixType: 3, latitude: 22.3, longitude: 114.17 }
  }, 1_700_000_000_000);

  assert.equal(normalized.fixValid, true);
  assert.equal(normalized.fresh, false);
  assert.equal(normalized.status, 'stale');
});

test('MAVLink unknown-accuracy sentinels are rejected', () => {
  const location = gps.normalizeTelemetry({
    gps: {
      fixType: 3, latitude: 22.3, longitude: 114.17,
      horizontalAccuracy: 4_294_967.295, verticalAccuracy: 4_283_336.448,
      updatedAt: 1_700_000_000_000
    }
  }, 1_700_000_001_000);
  assert.equal(location.horizontalAccuracy, null);
  assert.equal(location.verticalAccuracy, null);
});

test('haversine distance is stable for a short segment', () => {
  const distance = gps.haversineMeters(
    { latitude: 22.3, longitude: 114.17 },
    { latitude: 22.3001, longitude: 114.17 }
  );
  assert.ok(distance > 11 && distance < 11.2);
});
