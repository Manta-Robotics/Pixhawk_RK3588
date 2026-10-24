import test from 'node:test';
import assert from 'node:assert/strict';
import {loadTypeScript} from './helpers/load-typescript.mjs';

test('React cleanup removes local/global listeners, timers and exports before remount', () => {
  let id = 0, calls = 0;
  const pending = new Map();
  const schedule = callback => { pending.set(++id, callback); return id; };
  const cancel = key => pending.delete(key);
  const window = Object.assign(new EventTarget(), {
    setTimeout: schedule, clearTimeout: cancel, setInterval: schedule, clearInterval: cancel,
    requestAnimationFrame: schedule, cancelAnimationFrame: cancel, original: 'preserved'
  });
  const document = new EventTarget(), button = new EventTarget();
  document.createTreeWalker = function () {
    assert.equal(this, document, 'language traversal must use the real Document receiver');
    return 'walker';
  };
  const {createConsoleScope} = loadTypeScript('web/src/lifecycle.ts', {window, document});
  for (let mount = 0; mount < 3; mount++) {
    const scope = createConsoleScope();
    assert.equal(scope.document.createTreeWalker(), 'walker');
    scope.listen(button, 'click', () => calls++);
    scope.window.addEventListener('blur', () => calls++);
    scope.document.addEventListener('visibilitychange', () => calls++);
    scope.window.original = 'temporary'; scope.window.exported = () => {};
    scope.setTimeout(() => calls++); scope.setInterval(() => calls++); scope.requestAnimationFrame(() => calls++);
    button.dispatchEvent(new Event('click'));
    assert.equal(calls, mount + 1, 'one handler per mount');
    const queued = [...pending.values()];
    scope.dispose(); scope.dispose();
    button.dispatchEvent(new Event('click')); window.dispatchEvent(new Event('blur'));
    document.dispatchEvent(new Event('visibilitychange'));
    queued.forEach(callback => callback(123)); // Already queued callbacks must also be inert.
    assert.equal(calls, mount + 1);
    assert.equal(pending.size, 0);
    assert.equal(window.original, 'preserved');
    assert.equal('exported' in window, false);
  }
});
