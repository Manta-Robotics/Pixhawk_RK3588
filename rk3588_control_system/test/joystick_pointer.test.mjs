import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import vm from 'node:vm';

function setup(width = 160) {
  const handlers = {}, windowHandlers = {}, documentHandlers = {}, commands = [];
  let captured = null;
  const root = { getBoundingClientRect: () => ({ left: 20, top: 30, width, height: width }),
    setAttribute() {}, addEventListener: (event, fn) => { handlers[event] = fn; },
    setPointerCapture: id => { captured = id; }, hasPointerCapture: id => captured === id,
    releasePointerCapture: () => { captured = null; } };
  const stick = { style: {}, getBoundingClientRect: () => ({ width: width < 100 ? 34 : 48, height: width < 100 ? 34 : 48 }) };
  const output = {};
  const scope = { state: { connected: true, controlLink: true, armed: true, calibrating: false, driveMode: 'manual', vector: { x: 0, y: 0 } },
    $: selector => selector === '#joystick' ? root : selector === '#joyStick' ? stick : output,
    ensureTransport: () => ({ drive: vector => commands.push({ ...vector }) }), localize: text => text,
    setMotorOutput() {}, window: { addEventListener: (name, fn) => { windowHandlers[name] = fn; } },
    document: { hidden: false, addEventListener: (name, fn) => { documentHandlers[name] = fn; } } };
  const html = fs.readFileSync(new URL('../frontend/mobile-preview-kimi-k26.html', import.meta.url), 'utf8');
  vm.runInNewContext(html.slice(html.indexOf('function installJoystick(){'), html.indexOf('\nfunction renderMotorStatus(){')), scope);
  scope.installJoystick();
  const event = (id, x, y, extra = {}) => ({ pointerId: id, pointerType: 'touch', button: 0, clientX: x, clientY: y,
    cancelable: true, defaultPrevented: false, preventDefault() { this.defaultPrevented = true; }, ...extra });
  return { handlers, windowHandlers, documentHandlers, commands, scope, event, root };
}

for (const width of [86, 160]) {
  test(`left/right drag is isolated from scroll and maps correctly at ${width}px`, () => {
    const t = setup(width), cx = 20 + width / 2, cy = 30 + width / 2;
    const down = t.event(1, cx, cy); t.handlers.pointerdown(down);
    assert.equal(down.defaultPrevented, true);
    const right = t.event(1, cx + width, cy); t.handlers.pointermove(right);
    assert.equal(right.defaultPrevented, true);
    assert.equal(t.commands.at(-1).x, 1); assert.equal(t.commands.at(-1).y, 0);
    t.handlers.pointermove(t.event(1, cx - width, cy));
    assert.equal(t.commands.at(-1).x, -1); assert.equal(t.commands.at(-1).y, 0);
    t.handlers.pointerup(t.event(1, cx, cy));
    assert.deepEqual(t.commands.at(-1), { x: 0, y: 0 });
  });
}

test('a second finger cannot hijack or release the active joystick', () => {
  const t = setup(); t.handlers.pointerdown(t.event(1, 100, 110));
  t.handlers.pointermove(t.event(1, 150, 110)); const before = t.commands.length;
  t.handlers.pointerdown(t.event(2, 20, 110)); t.handlers.pointerup(t.event(2, 20, 110));
  assert.equal(t.commands.length, before);
  t.handlers.pointermove(t.event(1, 180, 110)); assert.equal(t.commands.at(-1).x, 1);
});

test('idle blur does not send a stop command; active cancellation stops exactly once', () => {
  const t = setup(); t.windowHandlers.blur(); assert.equal(t.commands.length, 0);
  t.handlers.pointerdown(t.event(1, 150, 110));
  const before = t.commands.length; t.handlers.pointercancel(t.event(1, 150, 110));
  t.handlers.lostpointercapture(t.event(1, 150, 110));
  assert.equal(t.commands.length, before + 1); assert.deepEqual(t.commands.at(-1), { x: 0, y: 0 });
});

test('drag uses a fixed origin and orientation changes stop active input', () => {
  const t = setup(); t.handlers.pointerdown(t.event(1, 100, 110));
  t.root.getBoundingClientRect = () => ({ left: -100, top: 30, width: 160, height: 160 });
  t.handlers.pointermove(t.event(1, 100, 110)); assert.equal(t.commands.at(-1).x, 0);
  t.handlers.pointermove(t.event(1, 180, 110));
  t.windowHandlers.resize(); assert.deepEqual(t.commands.at(-1), { x: 0, y: 0 });
});

test('automatic mode and non-primary mouse button do not start manual control', () => {
  const t = setup(); t.scope.state.driveMode = 'auto';
  t.handlers.pointerdown(t.event(1, 180, 110)); assert.equal(t.commands.length, 0);
  t.scope.state.driveMode = 'manual';
  t.handlers.pointerdown(t.event(1, 180, 110, { pointerType: 'mouse', button: 2 }));
  assert.equal(t.commands.length, 0);
});

test('diagonal input stays inside the unit circle and up/down preserve throttle direction', () => {
  const t = setup(); t.handlers.pointerdown(t.event(1, 100, 110));
  t.handlers.pointermove(t.event(1, 100, 0)); assert.equal(t.commands.at(-1).y, -1);
  t.handlers.pointermove(t.event(1, 100, 220)); assert.equal(t.commands.at(-1).y, 1);
  t.handlers.pointermove(t.event(1, 220, 0));
  const vector = t.commands.at(-1);
  assert.ok(vector.x > 0 && vector.y < 0);
  assert.ok(Math.abs(Math.hypot(vector.x, vector.y) - 1) < 1e-10);
});

test('actual live transport maps right/left/up/down to the intended command signs', () => {
  const sent = [];
  const scope = { window: {}, Date, console, clearTimeout() {}, setTimeout: fn => { fn(); return 1; } };
  vm.runInNewContext(fs.readFileSync(new URL('../frontend/js/manta-app-transport.js', import.meta.url), 'utf8'), scope);
  const transport = scope.window.MantaAppTransport.create('live');
  transport.socket = { connected: true, emit: (event, payload) => sent.push({ event, ...payload }) };
  for (const vector of [{ x: 1, y: 0 }, { x: -1, y: 0 }, { x: 0, y: -1 }, { x: 0, y: 1 }]) {
    transport.lastDriveAt = 0; transport.drive(vector);
  }
  assert.deepEqual(sent.map(p => [p.throttle || 0, p.steering || 0]), [[0, 45], [0, -45], [100, 0], [-100, 0]]);
});
