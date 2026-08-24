import assert from 'node:assert/strict';
import test from 'node:test';

import { isSameOriginRequest } from '../backend/origin_policy.js';

function request(origin, host = '10.42.0.1:3000') {
  return { headers: { ...(origin === undefined ? {} : { origin }), host } };
}

test('same-origin browser requests are allowed', () => {
  assert.equal(isSameOriginRequest(request('http://10.42.0.1:3000')), true);
  assert.equal(isSameOriginRequest(request('https://manta.local', 'manta.local')), true);
});

test('requests without Origin remain available to local diagnostics', () => {
  assert.equal(isSameOriginRequest(request(undefined)), true);
});

test('cross-origin and malformed origins are rejected', () => {
  assert.equal(isSameOriginRequest(request('https://attacker.example')), false);
  assert.equal(isSameOriginRequest(request('null')), false);
});
