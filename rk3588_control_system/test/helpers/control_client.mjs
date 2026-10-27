import WebSocket from 'ws';

export const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
export async function until(predicate, message = 'condition timed out', timeout = 1500) {
  const end = Date.now() + timeout;
  while (!predicate()) {
    if (Date.now() >= end) throw new Error(message);
    await delay(5);
  }
}

// Exercise the actual Engine.IO/Socket.IO protocol without a browser or hardware.
export function channel(t, base, cookie = '', clientId = 'test-page-0000000001') {
  const url = new URL('/socket.io/?EIO=4&transport=websocket', base);
  url.protocol = url.protocol === 'https:' ? 'wss:' : 'ws:';
  const socket = new WebSocket(url, { headers: cookie ? { Cookie: cookie } : {} });
  t.after(() => socket.terminate());
  const events = [];
  const ready = new Promise((resolve, reject) => {
    const deadline = setTimeout(() => reject(new Error('Realtime connection timed out')), 2000);
    deadline.unref();
    const finish = error => { clearTimeout(deadline); error ? reject(error) : resolve(); };
    socket.on('error', finish);
    socket.on('message', buffer => {
      const raw = buffer.toString();
      if (raw.startsWith('0')) socket.send(`40${JSON.stringify({ clientId })}`);
      else if (raw === '2') socket.send('3');
      else if (raw.startsWith('40')) finish();
      else if (raw.startsWith('44')) finish(new Error(JSON.parse(raw.slice(2)).message));
      else if (raw.startsWith('42')) events.push(JSON.parse(raw.slice(2)));
    });
  });
  return { socket, ready, events, send: (name, payload) => socket.send(`42${JSON.stringify([name, payload])}`) };
}

export async function pair(base, client = 'test-page-0000000001', cookie = '') {
  const response = await fetch(`${base}/api/auth/pair`, { method: 'POST',
    headers: { 'Content-Type': 'application/json', 'X-Manta-Client': client, Cookie: cookie },
    body: JSON.stringify({ pin: '123456' }) });
  if (!response.ok) throw new Error(`Pairing failed: ${response.status}`);
  return response.headers.get('set-cookie')?.split(';')[0] || cookie;
}

export function api(base, cookie, client) {
  return (route, body) => fetch(base + route, { method: body === undefined ? 'GET' : 'POST',
    headers: { Cookie: cookie, 'X-Manta-Client': client, 'Content-Type': 'application/json' },
    ...(body === undefined ? {} : { body: JSON.stringify(body) }) });
}
