"""Common frame metadata, multipart transport and completed MP4 frame indexes."""

import json
import math
import os
import subprocess
import threading
import time
import uuid
from collections import deque
from fractions import Fraction
from http.client import HTTPException
from pathlib import Path
from urllib.request import urlopen


FRAME_HEADERS = {
    'frame_id': 'X-Manta-Frame-Id', 'stream_epoch': 'X-Manta-Stream-Epoch',
    'obs_ns': 'X-Manta-Obs-Ns', 'time_kind': 'X-Manta-Time-Kind',
    'source_pts': 'X-Manta-Source-Pts',
    'source_sequence': 'X-Manta-Source-Sequence',
}
MAX_JPEG = 32 * 1024 * 1024


class FrameClock:
    def __init__(self):
        self.reset()

    def reset(self):
        self.epoch = uuid.uuid4().hex
        self.previous = None
        self.occurrence = 0
        self.sequence = 0

    def observe(self, pts, time_base, observed=None):
        if time_base is not None:
            time_base = Fraction(time_base)
        stamp = (pts, time_base) if pts is not None and time_base else None
        if stamp and self.previous and (stamp[1] != self.previous[1] or stamp[0] < self.previous[0]):
            self.reset()
        self.sequence += 1
        if stamp is not None:
            self.occurrence = self.occurrence + 1 if stamp == self.previous else 0
            self.previous = stamp
        key = f'p{pts}/{self.occurrence}' if stamp else f'f{self.sequence}'
        return {'frame_id': f'{self.epoch}/{key}', 'stream_epoch': self.epoch,
                'source_pts': str(pts) if stamp else None,
                'source_sequence': self.sequence,
                'source_tb_num': time_base.numerator if stamp else None,
                'source_tb_den': time_base.denominator if stamp else None,
                'obs_ns': str(time.monotonic_ns() if observed is None else observed), 'time_kind': 'host_decode'}


def frame_headers(metadata):
    if not metadata:
        return b''
    values = [(header, metadata.get(key)) for key, header in FRAME_HEADERS.items()]
    if metadata.get('source_tb_num') and metadata.get('source_tb_den'):
        values.append(('X-Manta-Time-Base', f"{metadata['source_tb_num']}/{metadata['source_tb_den']}"))
    return ''.join(f'{key}: {value}\r\n' for key, value in values if value is not None).encode('ascii')


def _frame_metadata(headers):
    metadata = {key: headers.get(header.lower()) for key, header in FRAME_HEADERS.items()}
    if not metadata['frame_id'] or not metadata['stream_epoch']:
        return None
    if not metadata['frame_id'].startswith(metadata['stream_epoch'] + '/'):
        return None
    if not str(metadata['obs_ns'] or '').isdigit():
        metadata['obs_ns'], metadata['time_kind'] = None, 'unknown'
    try:
        base = Fraction(headers.get('x-manta-time-base', '0'))
        if not base:
            raise ValueError('missing time base')
        metadata['source_tb_num'], metadata['source_tb_den'] = base.numerator, base.denominator
    except (ValueError, ZeroDivisionError):
        metadata['source_tb_num'], metadata['source_tb_den'] = None, None
    sequence = metadata.get('source_sequence')
    metadata['source_sequence'] = int(sequence) if str(sequence or '').isdigit() else None
    return metadata


def multipart_parts(response):
    content_type = response.headers.get('Content-Type', '')
    if 'multipart/' not in content_type.lower():
        raise ValueError('Not an MJPEG multipart response')
    boundary = response.headers.get_param('boundary')
    if not boundary or len(boundary) > 128:
        raise ValueError('Invalid multipart boundary')
    boundary = b'--' + boundary.encode('ascii')
    while True:
        line = response.readline(8193)
        if not line or line.strip() == boundary + b'--':
            return
        if line.strip() != boundary:
            if line.strip():
                raise ValueError('Malformed multipart boundary')
            continue
        headers = {}
        for _ in range(48):
            line = response.readline(8193)
            if line in (b'\r\n', b'\n'):
                break
            if not line or len(line) > 8192:
                raise ValueError('Truncated multipart headers')
            key, separator, value = line.partition(b':')
            if not separator:
                raise ValueError('Malformed multipart header')
            headers[key.decode('ascii').lower()] = value.strip().decode('ascii')
        else:
            raise ValueError('Too many multipart headers')
        length = int(headers.get('content-length', '0'))
        if not 0 < length <= MAX_JPEG or headers.get('content-type') != 'image/jpeg':
            raise ValueError('Invalid JPEG part')
        jpeg = response.read(length)
        if len(jpeg) != length:
            raise ValueError('Truncated JPEG part')
        yield jpeg, _frame_metadata(headers)


class MediaCapture:
    """The production MJPEG path associates metadata with the exact decoded JPEG."""

    def __init__(self, source):
        import cv2
        self.cv2 = cv2
        self.source = source
        self.metadata = None
        self.response = None
        self.parts = None
        self.joined_source = source.startswith(('http://', 'https://')) and source.split('?', 1)[0].endswith('.mjpg')
        self.capture = None if self.joined_source else cv2.VideoCapture(source)

    def get(self, key):
        return self.capture.get(key) if self.capture is not None else 0

    def set(self, key, value):
        return self.capture.set(key, value) if self.capture is not None else False

    def read(self):
        if self.capture is not None:
            return self.capture.read()
        import numpy as np
        self.metadata = None
        try:
            if self.response is None:
                self.response = urlopen(self.source, timeout=4)
                self.parts = multipart_parts(self.response)
            jpeg, metadata = next(self.parts)
            image = self.cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), self.cv2.IMREAD_COLOR)
            if image is not None:
                self.metadata = metadata
            return image is not None, image
        except (ValueError, OSError, HTTPException, StopIteration):
            self.release()
            return False, None

    def release(self):
        if self.capture is not None:
            self.capture.release()
        if self.response is not None:
            self.response.close()
            self.response, self.parts = None, None


class LatestMediaCapture:
    """Bound live delay even when the inference consumer is slower than capture."""

    def __init__(self, source):
        self.capture = MediaCapture(source)
        self.metadata = None
        self.skipped = []
        self._next = None
        self._discarded = deque(maxlen=64)
        self._condition = threading.Condition()
        self._stopped = False
        self._thread = None
        if self.capture.joined_source:
            self._thread = threading.Thread(target=self._receive, daemon=True)
            self._thread.start()

    def get(self, key):
        return self.capture.get(key)

    def _receive(self):
        try:
            while not self._stopped:
                valid, image = self.capture.read()
                if not valid:
                    time.sleep(.05)
                    continue
                with self._condition:
                    if self._next and self._next[1]:
                        self._discarded.append(self._next[1])
                    self._next = (image, self.capture.metadata)
                    self._condition.notify()
        finally:
            self.capture.release()

    def read(self):
        if self._thread is None:
            return self.capture.read()
        with self._condition:
            self._condition.wait_for(lambda: self._next is not None, timeout=4)
            self.skipped = list(self._discarded)
            self._discarded.clear()
            if self._next is None:
                self.metadata = None
                return False, None
            image, self.metadata = self._next
            self._next = None
            return True, image

    def release(self):
        self._stopped = True
        if self._thread is None:
            self.capture.release()


def emit_frame_result(frame, state, detector, targets=(), selected_id=None, last_measured_frame_id=None,
                      mode_generation=None, selected_index=None):
    if frame is None:
        frame = {}
    payload = {'frame_id': None, **frame, 'state': state, 'targets': list(targets), 'selected_id': selected_id,
               'last_measured_frame_id': last_measured_frame_id, 'detector': detector, 'selected_index': selected_index,
               'worker_epoch': os.environ.get('MANTA_WORKER_EPOCH'),
               'mode_generation': int(os.environ.get('MANTA_MODE_GENERATION', '0')) if mode_generation is None else mode_generation}
    print('FRAME_RESULT:' + json.dumps(payload, separators=(',', ':'), allow_nan=False), flush=True)


def normalized_targets(boxes, scores, ids, frame):
    height, width = frame.shape[:2]
    result = []
    for box, score, target_id in zip(boxes, scores, ids):
        x1, y1, x2, y2 = map(float, box)
        confidence = float(score)
        if not all(map(math.isfinite, (x1, y1, x2, y2, confidence))):
            continue
        x1, x2 = (min(1, max(0, x / width)) for x in (x1, x2))
        y1, y2 = (min(1, max(0, y / height)) for y in (y1, y2))
        if x2 > x1 and y2 > y1:
            result.append({'id': str(target_id) if target_id is not None else None,
                           'confidence': min(1, max(0, confidence)), 'x': x1, 'y': y1,
                           'w': x2 - x1, 'h': y2 - y1})
    return result


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f'.{path.name}.{uuid.uuid4().hex}.tmp')
    try:
        with temporary.open('x', encoding='utf-8') as target:
            json.dump(data, target, separators=(',', ':'), allow_nan=False)
            target.flush()
            os.fsync(target.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def unedited_packet_pts(media, ffprobe):
    """Read video packet timestamps before the MP4 demuxer applies edits."""
    try:
        output = subprocess.run([ffprobe, '-v', 'error', '-ignore_editlist', '1', '-select_streams', 'v:0',
                                 '-show_packets', '-show_entries', 'packet=pts,pos', '-of', 'json', str(media)],
                                check=True, capture_output=True, timeout=10)
        packets = json.loads(output.stdout).get('packets', [])
    except (OSError, ValueError, subprocess.SubprocessError):
        return {}
    timestamps = {}
    for packet in packets:
        position = str(packet.get('pos', ''))
        pts = str(packet.get('pts', ''))
        if position.isdigit():
            timestamps[position] = int(pts) if position not in timestamps and pts.lstrip('-').isdigit() else None
    return timestamps


def probe_clip(media, clip, source_by_time=None, ffprobe='ffprobe'):
    """Explicit encoder PTS→common-JPEG map. Ambiguous keys stay unidentified."""
    output = subprocess.run([ffprobe, '-v', 'error', '-select_streams', 'v:0', '-show_streams',
                             '-show_frames', '-show_entries',
                             'stream=time_base,start_pts,duration_ts:frame=pts,pkt_pts,pkt_pos,pkt_duration,duration',
                             '-of', 'json', str(media)], check=True, capture_output=True, timeout=30)
    probe = json.loads(output.stdout)
    streams, shown = probe.get('streams', []), probe.get('frames', [])
    if not streams or not shown:
        raise ValueError('No decodable display frames')
    base = Fraction(streams[0]['time_base'])
    # Older ffprobe versions print the same AVFrame.pts as pkt_pts.
    pts_values = [int(pts) if str(pts).lstrip('-').isdigit() else None
                  for pts in (frame.get('pts', frame.get('pkt_pts')) for frame in shown)]
    # MP4 frame PTS already use the edit-adjusted, zero-based movie timeline.
    # A delayed first picture must keep its leading gap in the player.
    starts_at_zero = str(streams[0].get('start_pts')) == '0'
    source_by_time = (source_by_time or {}) if starts_at_zero else {}
    raw_pts = unedited_packet_pts(media, ffprobe) if source_by_time else {}
    pts_counts = {}
    position_counts = {}
    positions = [str(frame.get('pkt_pos', '')) for frame in shown]
    for pts, position in zip(pts_values, positions):
        pts_counts[pts] = pts_counts.get(pts, 0) + 1
        position_counts[position] = position_counts.get(position, 0) + 1
    frames = []
    for index, (pts, position, data) in enumerate(zip(pts_values, positions, shown)):
        key = pts * base if pts is not None else None
        same_packet_pts = pts is not None and raw_pts.get(position) == pts and position_counts[position] == 1
        meta = source_by_time.get(key) if same_packet_pts and pts_counts[pts] == 1 else None
        play_us = round(key * 1_000_000) if key is not None and key >= 0 else None
        raw_duration = data.get('duration') or data.get('pkt_duration')
        duration_us = round(int(raw_duration) * base * 1_000_000) if str(raw_duration).isdigit() else 0
        current_ns = int(meta['obs_ns']) if meta and meta.get('obs_ns') is not None else None
        frames.append({'display_no': index, 'frame_id': meta['frame_id'] if meta else None,
                       'file_pts': str(pts) if pts is not None else None, 'tb_num': base.numerator,
                       'tb_den': base.denominator, 'play_us': play_us,
                       'play_end_us': play_us + duration_us if play_us is not None and duration_us > 0 else None,
                       'frame_mono_ns': str(current_ns) if current_ns is not None else None,
                       'frame_mono_end_ns': None, 'time_kind': meta['time_kind'] if meta else 'unknown',
                       'time_error_ms': None, 'identity': 'exact' if meta else 'unknown',
                       'source_sequence': meta.get('source_sequence') if meta else None})
    gaps = []
    for current, following in zip(frames, frames[1:]):
        start, end = current['play_us'], following['play_us']
        a, b = current['frame_mono_ns'], following['frame_mono_ns']
        first_seq, next_seq = current['source_sequence'], following['source_sequence']
        continuous = first_seq is not None and next_seq == first_seq + 1
        if continuous and a is not None and b is not None and 0 < int(b) - int(a) <= 500_000_000:
            current['frame_mono_end_ns'] = b
        elif a is not None and b is not None and int(b) > int(a):
            gaps.append({'mono_start_ns': a, 'mono_end_ns': b, 'play_start_us': start,
                         'play_end_us': end, 'reason': 'source_gap'})
            current['play_end_us'] = None
            continue
        if start is not None and end is not None and end > start:
            current['play_end_us'] = end
    last = frames[-1]
    duration_ts = streams[0].get('duration_ts')
    if starts_at_zero and last['play_end_us'] is None and last['play_us'] is not None and str(duration_ts).isdigit():
        stream_end = round(int(duration_ts) * base * 1_000_000)
        if stream_end > last['play_us']:
            last['play_end_us'] = stream_end
    if last['frame_mono_ns'] is not None and last['play_end_us'] is not None and last['play_us'] is not None:
        duration_ns = (last['play_end_us'] - last['play_us']) * 1000
        if 0 < duration_ns <= 500_000_000:
            last['frame_mono_end_ns'] = str(int(last['frame_mono_ns']) + duration_ns)
    return {**clip, 'phase': 'ready', 'index_state': 'ready' if all(f['identity'] == 'exact' for f in frames) else 'partial',
            'play_origin_pts': '0', 'tb_num': base.numerator,
            'tb_den': base.denominator, 'frames': frames, 'gaps': gaps}
