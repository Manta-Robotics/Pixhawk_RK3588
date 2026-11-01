"""Passive ALX USB UART reader. Emits complete observations as JSON lines."""
import argparse
from collections import Counter
import json
import signal
import struct
import sys
import time


SERIAL_IDLE_RECONNECT_SECONDS = 3.0


class AlxParser:
    def __init__(self):
        self.buffer = bytearray()
        self.counts = Counter()
        self.last_unrecognized_header = None

    def feed(self, data):
        self.counts['rxBytes'] += len(data)
        self.buffer.extend(data)
        result = []
        while True:
            start = self.buffer.find(b'\xff\xff\xff\xff')
            if start < 0:
                discarded = max(0, len(self.buffer) - 3)
                self.counts['noiseBytes'] += discarded
                if discarded:
                    del self.buffer[:discarded]
                break
            if start:
                self.counts['noiseBytes'] += start
                del self.buffer[:start]
            if len(self.buffer) < 12:
                break
            length, sequence, command, version = struct.unpack('>HHHH', self.buffer[4:12])
            if (length, command) not in ((37, 0x2001), (16, 0x2002), (18, 0x2002)):
                self.last_unrecognized_header = bytes(self.buffer[:64]).hex(' ').upper()
                self.counts['lengthErrors'] += 1
                del self.buffer[0]
                continue
            versions = (0x0100, 0x0102) if command == 0x2001 else (0x0101, 0x0102)
            if version not in versions:
                self.last_unrecognized_header = bytes(self.buffer[:64]).hex(' ').upper()
                self.counts['unsupportedVersions'] += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < length:
                break
            packet = bytes(self.buffer[:length])
            # The captured 18-byte v1.2 heartbeat includes a reserved byte and
            # XOR checksum. The legacy 16-byte heartbeat has no checksum field.
            checksummed = command == 0x2001 or length == 18
            if checksummed:
                checksum = 0
                for b in packet[:-1]:
                    checksum ^= b
                if checksum != packet[-1]:
                    self.counts['checksumErrors'] += 1
                    del self.buffer[0]
                    continue
            if command == 0x2001:
                anchor, tag, cm, azimuth, elevation, status, batch = struct.unpack('>IIIhhHH', packet[12:32])
                self.counts['goodCount'] += 1
                result.append(dict(type='position', version=version, sequenceId=sequence,
                    anchorId=anchor, tagId=tag, distanceCm=cm,
                    azimuthDeg=azimuth, elevationDeg=elevation,
                    tagStatus=status, batchSn=batch))
            else:
                self.counts['heartbeatCount'] += 1
                result.append(dict(type='heartbeat', version=version, sequenceId=sequence,
                    anchorId=int.from_bytes(packet[12:16], 'big')))
            del self.buffer[:length]
        return result


def emit(event):
    print(json.dumps(event, separators=(',', ':'), allow_nan=False), flush=True)


def main():
    import serial
    args = argparse.ArgumentParser()
    args.add_argument('--port', required=True)
    args.add_argument('--baud', type=int, default=115200)
    options = args.parse_args()
    running = True

    def stop(_signal, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    while running:
        parser = AlxParser()
        port = None
        try:
            port = serial.Serial(port=None, baudrate=options.baud, bytesize=8,
                parity='N', stopbits=1, timeout=0.05, xonxoff=False,
                rtscts=False, dsrdtr=False, exclusive=True)
            port.dtr = False
            port.rts = False
            port.port = options.port
            port.open()
            emit(dict(type='connection', connected=True, port=options.port))
            last_stats = last_data_at = time.monotonic()
            last_good_count = 0
            last_position_at = None
            while running:
                data = port.read(min(8192, port.in_waiting or 1))
                events = parser.feed(data)
                now = time.monotonic()
                if data:
                    last_data_at = now
                elif now-last_data_at >= SERIAL_IDLE_RECONNECT_SECONDS:
                    # An open CDC descriptor can stop delivering bytes without
                    # raising an I/O error. Reopen it instead of reporting a
                    # connected receiver forever. Radio heartbeats count as
                    # serial activity; loss of the tag alone must not reconnect.
                    raise serial.SerialException('serial_idle_timeout: no USB serial bytes for 3 seconds; reopening')
                # A delayed UART burst is drained completely, but only its most
                # recent observation is submitted. Never replay old motion fast.
                positions = [i for i, event in enumerate(events) if event['type'] == 'position']
                newest = positions[-1] if positions else None
                for i, event in enumerate(events):
                    if event['type'] == 'position' and i != newest:
                        parser.counts['coalescedPositions'] += 1
                        continue
                    event['receivedMonoMs'] = now*1000
                    if event['type'] == 'position':
                        event['gapMs'] = None if last_position_at is None else (now-last_position_at)*1000
                        last_position_at = now
                    emit(dict(event, stats=dict(parser.counts)))
                if now-last_stats >= 1:
                    parser.counts['positionRateHz'] = round((parser.counts['goodCount']-last_good_count)/(now-last_stats), 2)
                    emit(dict(type='stats', stats=dict(parser.counts),
                        lastUnknownHeaderHex=parser.last_unrecognized_header))
                    last_good_count = parser.counts['goodCount']
                    last_stats = now
        except (serial.SerialException, OSError) as exc:
            emit(dict(type='connection', connected=False, error=str(exc), stats=dict(parser.counts)))
        finally:
            if port is not None:
                port.close()
        if running:
            time.sleep(1)


if __name__ == '__main__':
    main()
