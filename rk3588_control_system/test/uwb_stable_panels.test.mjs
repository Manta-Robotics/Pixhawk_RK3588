import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import ts from 'typescript';

const read = path => fs.readFileSync(new URL('../' + path, import.meta.url), 'utf8');
const html = read('frontend/mobile-preview-kimi-k26.html');
for (const path of ['frontend/mobile-preview-kimi-k26.html', 'web/src/console-controller.ts']) {
  test(`${path}: signal gaps hold both panels, recovery updates and disconnect clears history`, () => {
    let now = 100000;
    const nodes = new Map(), $ = id => {
      if (!nodes.has(id)) nodes.set(id, { dataset: {}, attributes: {},
        style: { setProperty(key, value) { this[key] = value; } },
        setAttribute(key, value) { this.attributes[key] = value; } });
      return nodes.get(id);
    };
    const state = { connected: true, uwb: {}, uwbFollow: { active: false } };
    const context = vm.createContext({ $, state, Date: { now: () => now },
      localize: zh => zh, clamp: (x, lo, hi) => Math.min(hi, Math.max(lo, x)) });
    for (const name of ['uwb-display-smoother', 'uwb-raw-display']) {
      vm.runInContext('var window=globalThis;\n' + read(`frontend/js/${name}.js`), context);
    }
    let source = read(path);
    source = source.slice(source.indexOf('function renderUwbStatus('), source.indexOf('function currentMapLocation('));
    if (path.endsWith('.ts')) source = ts.transpileModule(source, {
      compilerOptions: { target: ts.ScriptTarget.ES2022 }
    }).outputText;
    vm.runInContext(source, context);
    const render = () => {
      const before = structuredClone(state);
      vm.runInContext('renderUwbStatus()', context);
      assert.deepEqual(state, before, 'display history must never modify input or control state');
    };
    const accept = distance => {
      state.uwb = { online: true, fresh: true, measurementStatus: 'trusted', trustChecksEnabled: false,
        measurementUpdatedAt: now, distanceM: distance, azimuthDeg: 20, elevationDeg: 5,
        sensors: [{ valid: true, receiverId: 1, tagId: 5313, rawDistanceM: distance,
          rawAzimuthDeg: 20, rawDistanceUpdatedAt: now, receivedMeasurementAt: now }] };
    };
    const view = () => Object.fromEntries(['uwbDistance', 'uwbAngles', 'relativeRangeValue',
      'relativeRawValue', 'relativePositionValue', 'relativeRingLabel'].map(id => [id, $('#' + id).textContent]));
    const coordinates = () => ['relativeTargetPoint', 'relativeRawPoint'].map(id => [
      $('#' + id).attributes.cx, $('#' + id).attributes.cy]);
    render();
    assert.equal($('#uwbDistance').textContent, '-- m');
    assert.equal($('#uwbDot').dataset.missing, 'true');
    assert.equal($('#relativeTargetPoint').dataset.missing, 'true');
    assert.equal($('#relativeTargetPoint').attributes.cx, '90.00');
    for (let i = 0; i < 4; i++) {
      accept(3 + i);
      render();
      assert.equal($('#uwbDot').dataset.missing, 'false');
      assert.equal($('#relativeTargetPoint').dataset.missing, 'false');
      assert.equal($('#relativeRawPoint').dataset.missing, 'false');
      const previous = view(), points = coordinates();
      const compactPoint = [$('#uwbDot').style['--uwb-x'], $('#uwbDot').style['--uwb-y']];
      now += 800; // Includes expiry from clocks even if online/fresh flags have not changed.
      for (let gap = 0; gap < 3; gap++) {
        render();
        assert.deepEqual(view(), previous);
        assert.deepEqual(coordinates(), points);
        assert.deepEqual([$('#uwbDot').style['--uwb-x'], $('#uwbDot').style['--uwb-y']], compactPoint);
        for (const id of ['uwbDot', 'relativeTargetPoint', 'relativeRawPoint']) assert.equal($('#' + id).dataset.missing, 'true');
        assert.equal($('#relativeTargetPoint').attributes.visibility, 'visible');
        assert.match($('#uwbDistanceLabel').textContent, /信号丢失.*上次/);
        assert.equal($('#relativeJudgedLabel').textContent, '● 上次位置');
        assert.match($('#relativeDecision').textContent, /信号丢失.*上次/);
        now += 100;
      }
    }
    state.connected = false;
    render();
    assert.equal($('#uwbDistance').textContent, '-- m');
    assert.equal($('#relativeRangeValue').textContent, '-- m / --°');
    assert.equal($('#relativeRawValue').textContent, '-- m / --°');
    assert.equal($('#relativeTargetPoint').attributes.cx, '90.00');
    assert.equal($('#relativeDecision').textContent, '连接已断开');
    state.connected = true;
    state.uwb = {};
    render();
    assert.equal($('#relativeRangeValue').textContent, '-- m / --°');
    accept(1.5);
    render();
    assert.equal($('#uwbDistance').textContent, '1.50 m');
    assert.equal($('#relativeRangeValue').textContent, '1.50 m / 20.0°');
    assert.equal($('#relativeTargetPoint').dataset.missing, 'false');
  });
}

test('only missing target markers animate; panel opacity and borders do not depend on reception', () => {
  const css = html.match(/<style>([\s\S]*?)<\/style>/)[1];
  assert.doesNotMatch(css, /\.(?:uwb-panel|relative-position-panel)\[data-state="[^"]+"\]\s*\{/);
  const rules = [...css.matchAll(/([^{}]+)\{([^{}]*animation:uwb-missing-point[^{}]*)\}/g)];
  assert.equal(rules.length, 1);
  assert.deepEqual(rules[0][1].trim().split(','), [
    '.uwb-dot[data-missing="true"]', '.relative-target[data-missing="true"]', '.relative-raw-target[data-missing="true"]'
  ]);
  assert.match(css, /prefers-reduced-motion:reduce/);
});
