import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import {loadTypeScript} from './helpers/load-typescript.mjs';

function fixture(migrated) {
  const requests = [], events = [], timers = new Map(), window = {};
  let id = 0;
  const globals = {
    window, console, Date, Promise,
    setTimeout: callback => { timers.set(++id, callback); return id; },
    clearTimeout: key => timers.delete(key),
    setInterval: callback => { timers.set(++id, callback); return id; },
    clearInterval: key => timers.delete(key),
    fetch: async (url, options = {}) => {
      requests.push({url, method: options.method || 'GET', body: options.body ? JSON.parse(options.body) : null});
      return {ok: true, json: async () => ({success: true, recordings: [{id: 'one'}], state: {active: false}})};
    }
  };
  if (migrated) loadTypeScript('web/src/manta-app-transport.ts', globals);
  else vm.runInNewContext(fs.readFileSync('frontend/js/manta-app-transport.js', 'utf8'), globals);
  const live = window.MantaAppTransport.create('live');
  const socketListeners = new Map();
  live.socket = {connected: true,
    emit: (event, value) => events.push({event, value}),
    once: (event, callback) => socketListeners.set(event, callback),
    off: event => socketListeners.delete(event),
    disconnect: () => events.push({event: 'disconnect'})};
  live.connected = true;
  return {live, requests, events, timers, socketListeners, factory: window.MantaAppTransport};
}

for (const migrated of [false, true]) {
  const label = migrated ? 'React TypeScript' : 'original';
  test(label + ': drive limits, motion lock and cleanup retain the control lease contract', async () => {
    const {live, events, timers} = fixture(migrated);
    live.drive({x: 2, y: -2});
    assert.equal(events[0].event, 'rover_drive');
    assert.deepEqual(JSON.parse(JSON.stringify(events[0].value)), {throttle: 100, steering: 45});
    live.startDriveKeepalive();
    live.drive({x: 0, y: 0}); // The calibration UI resets the joystick before locking.
    live.setMotionLocked(true);
    const count = events.length;
    live.drive({x: 1, y: -1});
    for (const callback of timers.values()) callback();
    assert.equal(events.length, count, 'locked commands and keepalive must not move');
    assert.equal(Math.abs(events.at(-1).value.throttle), 0);
    assert.equal(events.at(-1).value.steering, 0);
    await live.disconnect();
    assert.equal(timers.size, 0);
    assert.equal(events.at(-1).event, 'disconnect');
  });

  test(label + ': arming resolves only after aircraft confirmation', async () => {
    const {live, socketListeners, timers} = fixture(migrated);
    let armed = false;
    const pending = live.arm().then(() => { armed = true; });
    await Promise.resolve();
    assert.equal(armed, false);
    socketListeners.get('aircraft_armed')();
    await pending;
    assert.equal(armed, true);
    assert.equal(timers.size, 0);
    assert.equal(socketListeners.size, 0);
  });
}

test('TypeScript preserves follow, calibration, recording, gimbal and emergency API requests', async () => {
  async function exercise(migrated) {
    const {live, requests} = fixture(migrated);
    await live.startUwbFollow(); await live.stopUwbFollow();
    await live.setUwbFollowMethod('traditional'); await live.setUwbFollowMethod('neural');
    await live.startImuCalibration('ACCEL'); await live.confirmImuCalibration(3);
    await live.startBeaconCalibration(); await live.captureBeaconCalibration();
    await live.cancelBeaconCalibration(); await live.getBeaconCalibration();
    await live.getRecordings(); await live.setGimbalOsd(false); await live.gimbalClick(.2, -.3);
    await live.emergencyStop();
    return requests;
  }
  const original = await exercise(false);
  assert.equal(original.length, 14);
  assert.deepEqual(await exercise(true), original);
});

test('preview nine-point calibration reports unsupported hardware without hanging the UI', async () => {
  const mock = fixture(true).factory.create('mock');
  await assert.rejects(mock.startBeaconCalibration(), /九点校准需要连接实机/);
  await assert.rejects(mock.captureBeaconCalibration(), /九点校准需要连接实机/);
  await mock.cancelBeaconCalibration();
});
