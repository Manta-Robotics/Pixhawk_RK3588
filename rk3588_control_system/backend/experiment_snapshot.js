import dgram from 'node:dgram';

// No API credentials, control messages, serial port access or file I/O here.
export function startExperimentSnapshot(readState) {
  const socket = dgram.createSocket('udp4');
  let sequence = 0, drops = 0, busy = false;
  socket.on('error', () => { drops += 1; });
  socket.unref();
  const timer = setInterval(() => {
    if (busy) { drops += 1; return; }
    try {
      const packet = { kind: 'snapshot', sequence: ++sequence, sender_drops: drops,
        wall_ns: String(BigInt(Date.now()) * 1000000n),
        mono_ns: process.hrtime.bigint().toString(), state: readState() };
      const raw = Buffer.from(JSON.stringify(packet));
      if (raw.length > 60000) { drops += 1; return; }
      busy = true;
      socket.send(raw, 14660, '127.0.0.1', (error) => {
        busy = false;
        if (error) drops += 1;
      });
    } catch { busy = false; drops += 1; }
  }, 100);
  timer.unref();
  return () => { clearInterval(timer); try { socket.close(); } catch {} };
}
