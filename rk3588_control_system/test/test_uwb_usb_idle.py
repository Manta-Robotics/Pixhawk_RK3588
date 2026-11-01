"""Exercise serial recovery with a virtual UART, without touching hardware."""
import collections
import json
import os
from pathlib import Path
import select
import struct
import subprocess
import sys
import time
import unittest


@unittest.skipUnless(sys.platform.startswith('linux'), 'Linux pseudo-terminal integration')
class UsbIdleRecoveryTests(unittest.TestCase):
    def test_silent_uart_reopens_and_radio_heartbeats_keep_connection_alive(self):
        import pty

        master, slave = pty.openpty()
        script = Path(__file__).resolve().parents[1] / 'backend' / 'uwb_usb_reader.py'
        child = subprocess.Popen([sys.executable, '-u', str(script), '--port', os.ttyname(slave)],
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        pending = bytearray()
        events = collections.deque()

        def next_event(timeout):
            deadline = time.monotonic() + timeout
            while not events:
                remaining = deadline-time.monotonic()
                if remaining <= 0 or not select.select([child.stdout], [], [], remaining)[0]:
                    raise AssertionError('Timed out waiting for reader event')
                chunk = os.read(child.stdout.fileno(), 65536)
                if not chunk:
                    raise AssertionError('Reader exited: ' + child.stderr.read().decode())
                pending.extend(chunk)
                while b'\n' in pending:
                    line, _, rest = pending.partition(b'\n')
                    pending[:] = rest
                    events.append(json.loads(line))
            return events.popleft()

        def until(predicate, timeout):
            deadline = time.monotonic() + timeout
            while True:
                event = next_event(max(.01, deadline-time.monotonic()))
                if predicate(event):
                    return event
                self.assertLess(time.monotonic(), deadline)

        try:
            until(lambda e: e['type'] == 'connection' and e['connected'], 3)
            lost = until(lambda e: e['type'] == 'connection' and not e['connected'], 5)
            self.assertIn('serial_idle_timeout', lost['error'])
            until(lambda e: e['type'] == 'connection' and e['connected'], 3)
            frame = bytes.fromhex('FF FF FF FF 00 25 0E 43 20 01 01 02 00 00 15 BD 00 00 15 BD 00 00 00 30 01 52 00 19 12 34 17 01 00 00 00 00 00')
            os.write(master, frame)
            position = until(lambda e: e['type'] == 'position', 2)
            self.assertEqual(position['distanceCm'], 48)
            # A tag can disappear while the receiver still sends heartbeats.
            # That must invalidate position upstream, without reopening USB.
            for seq in range(15):
                heartbeat = struct.pack('>4sHHHHI', b'\xff'*4, 16, seq, 0x2002, 0x0101, 5565)
                os.write(master, heartbeat)
                deadline = time.monotonic()+.25
                while True:
                    event = next_event(1)
                    self.assertNotEqual(event['type'], 'connection')
                    if event['type'] == 'heartbeat':
                        break
                time.sleep(max(0, deadline-time.monotonic()))
            os.write(master, frame)
            while True:
                event = next_event(2)
                self.assertNotEqual(event['type'], 'connection')
                if event['type'] == 'position':
                    break
        finally:
            child.terminate()
            try:
                child.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                child.kill()
                child.communicate()
            os.close(master)
            os.close(slave)


if __name__ == '__main__':
    unittest.main()
