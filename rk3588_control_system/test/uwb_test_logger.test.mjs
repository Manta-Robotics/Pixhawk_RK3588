import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import test from 'node:test';

import { UwbTestLogger, UWB_TEST_LOG_COLUMNS } from '../backend/uwb_test_logger.js';

test('each UWB follow run creates an independent trajectory CSV and flushes on stop', (t) => {
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'manta-uwb-log-'));
  t.after(() => fs.rmSync(directory, { recursive: true, force: true }));
  const logger = new UwbTestLogger(directory, { flushRows: 20 });

  const first = logger.start({ control_mode: 'relative_acro', follow_status: 'tracking' }, 1000);
  logger.record({
    yaw_deg: 0,
    commanded_forward_mps: 1,
    uwb_distance_m: 3,
    target_relative_x_m: 2.8,
    target_relative_y_m: 1,
    actual_left_pwm: 1675,
    actual_right_pwm: 1600
  }, 2000);
  logger.stop({ follow_reason: 'manual_stop' }, 2100);

  const text = fs.readFileSync(first.filePath, 'utf8');
  assert.equal(text.split('\n')[0], UWB_TEST_LOG_COLUMNS.join(','));
  assert.match(text, /relative_acro/);
  assert.match(text, /manual_stop/);
  assert.match(text, /1675/);
  assert.equal(logger.state().active, false);

  const second = logger.start({ control_mode: 'relative_acro' }, 3000);
  logger.stop({ follow_reason: 'completed' }, 3100);
  assert.notEqual(second.fileName, first.fileName);
  assert.equal(fs.existsSync(second.filePath), true);
});
