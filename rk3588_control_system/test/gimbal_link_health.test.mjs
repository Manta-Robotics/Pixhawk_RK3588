import assert from 'node:assert/strict';
import test from 'node:test';
import {
  acceptGimbalFeedback,
  createGimbalLinkHealth,
  evaluateGimbalLink,
  expectGimbalResponse
} from '../backend/gimbal_link_health.js';

test('an open serial port without passive feedback is normal', () => {
  const health = createGimbalLinkHealth(1000);
  assert.deepEqual(evaluateGimbalLink(health, { portOpen: true, feedbackFresh: false, now: 0 }), {
    connected: true,
    linkStatus: 'ready',
    error: ''
  });
});

test('missing feedback becomes an error only after a sent command times out', () => {
  const health = createGimbalLinkHealth(1000);
  expectGimbalResponse(health, 'home', 100);
  assert.equal(evaluateGimbalLink(health, { portOpen: true, now: 1099 }).linkStatus, 'checking');
  const result = evaluateGimbalLink(health, { portOpen: true, now: 1100 });
  assert.equal(result.connected, false);
  assert.equal(result.linkStatus, 'command_timeout');
  assert.match(result.error, /home/);
});

test('a valid frame after a command confirms and recovers the link', () => {
  const health = createGimbalLinkHealth(1000);
  expectGimbalResponse(health, 'click', 100);
  acceptGimbalFeedback(health, 250);
  assert.deepEqual(evaluateGimbalLink(health, { portOpen: true, feedbackFresh: false, now: 5000 }), {
    connected: true,
    linkStatus: 'ready',
    error: ''
  });
  assert.equal(health.status, 'confirmed');
  assert.equal(health.respondedAt, 250);
});
