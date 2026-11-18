"""Encode the exact frames received from the shared camera broker."""

import argparse
import json
import signal
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor
from fractions import Fraction
from pathlib import Path
from urllib.request import urlopen

from media_alignment import atomic_json, multipart_parts, probe_clip


ENCODER_BASE = Fraction(1, 90000)


def notify(kind, **data):
    print('alignment=' + json.dumps({'kind': kind, **data}, separators=(',', ':')), flush=True)


def source_time(metadata):
    if metadata.get('source_pts') is None or not metadata.get('source_tb_num') or not metadata.get('source_tb_den'):
        return None
    return int(metadata['source_pts']) * Fraction(metadata['source_tb_num'], metadata['source_tb_den'])


def clip_info(args, index, epoch=None):
    return {'boot_id': args.boot_id or None, 'session_id': args.session_id or None,
            'recording_id': args.recording_id, 'clip_id': f'{args.recording_id}-{index:06d}',
            'clip_no': index, 'stream_epoch': epoch, 'file_name': f'{args.prefix}__{index:06d}.mp4',
            'phase': 'starting', 'index_state': 'none', 'play_origin_pts': None, 'tb_num': None,
            'tb_den': None, 'mono_start_ns': None, 'mono_end_ns': None}


def finish_index(args, path, clip, mapping=None):
    try:
        aligned = probe_clip(path, clip, mapping, ffprobe=args.ffprobe)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        failed = {**clip, 'phase': 'failed', 'index_state': 'failed', 'reason': {'code': 'VIDEO_INDEX_FAILED', 'message': str(error)}}
        notify('clip_ready', clip=failed)
        return
    try:
        atomic_json(Path(args.index_directory) / (clip['clip_id'] + '.json'), aligned)
    except OSError as error:
        failed = {**aligned, 'index_state': 'failed', 'reason': {'code': 'VIDEO_INDEX_FAILED', 'message': str(error)}}
        notify('clip_ready', clip={key: value for key, value in failed.items() if key not in ('frames', 'gaps')})
        return
    notify('clip_ready', clip={key: value for key, value in aligned.items() if key not in ('frames', 'gaps')})


class Clip:
    def __init__(self, args, index, metadata, image):
        import av
        self.av, self.args = av, args
        self.info = clip_info(args, index, metadata['stream_epoch'])
        self.info['mono_start_ns'] = metadata['obs_ns']
        self.media = Path(args.directory) / self.info['file_name']
        if self.media.exists():
            raise FileExistsError(self.media)
        self.output = av.open(str(self.media), 'w', format='mp4', options={
            'movflags': '+frag_keyframe+empty_moov+default_base_moof', 'video_track_timescale': '90000'})
        self.stream = self.output.add_stream(args.codec, rate=args.fps)
        self.stream.width, self.stream.height = image.shape[1], image.shape[0]
        self.stream.pix_fmt = 'yuv420p'
        self.stream.time_base = ENCODER_BASE
        self.stream.codec_context.time_base = ENCODER_BASE
        self.stream.codec_context.max_b_frames = 0
        self.stream.codec_context.gop_size = args.fps
        self.stream.codec_context.thread_count = 2
        if args.codec == 'libx264':
            self.stream.options = {'preset': 'ultrafast', 'tune': 'zerolatency', 'crf': '19'}
        self.started = int(metadata['obs_ns'])
        self.first_source_time = source_time(metadata)
        self.last_pts = -1
        self.mapping = {}
        self.written = set()
        self.last_id = None
        notify('clip_open', clip=self.info)

    def write(self, image, metadata):
        frame_time = source_time(metadata)
        if frame_time is None or self.first_source_time is None:
            elapsed = Fraction(int(metadata['obs_ns']) - self.started, 1_000_000_000)
        else:
            elapsed = frame_time - self.first_source_time
        pts = max(self.last_pts + 1, round(elapsed / ENCODER_BASE))
        frame = self.av.VideoFrame.from_ndarray(image, format='bgr24')
        frame.pts, frame.time_base = pts, ENCODER_BASE
        for packet in self.stream.encode(frame):
            self.mux(packet)
        self.last_pts, self.last_id = pts, metadata['frame_id']
        self.info['mono_end_ns'] = metadata['obs_ns']
        self.mapping[pts * ENCODER_BASE] = metadata

    def mux(self, packet):
        pts = packet.pts * packet.time_base if packet.pts is not None and packet.time_base is not None else None
        self.output.mux(packet)
        if pts is not None:
            self.written.add(pts)

    def close(self, indexer):
        for packet in self.stream.encode(None):
            self.mux(packet)
        self.output.close()
        proven = {key: meta for key, meta in self.mapping.items() if key in self.written}
        notify('clip_indexing', clip={**self.info, 'phase': 'finalizing', 'index_state': 'building'})
        indexer.submit(finish_index, self.args, self.media, self.info, proven)


def capture(args):
    import cv2
    import numpy as np

    class Stopping(Exception):
        pass

    stopping = False

    def stop(_signal, _frame):
        nonlocal stopping
        if not stopping:
            stopping = True
            raise Stopping

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    clip, index, total, last_progress = None, args.start_index, 0, 0.0
    indexer = ThreadPoolExecutor(max_workers=1)
    try:
        with urlopen(args.source, timeout=args.stall_timeout) as stream:
            for jpeg, metadata in multipart_parts(stream):
                if metadata is None or not metadata.get('obs_ns'):
                    continue
                if clip is not None and metadata['frame_id'] == clip.last_id:
                    continue
                image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    continue
                if image.shape[0] % 2 or image.shape[1] % 2:
                    image = cv2.copyMakeBorder(image, 0, image.shape[0] % 2, 0, image.shape[1] % 2, cv2.BORDER_REPLICATE)
                if clip and (metadata['stream_epoch'] != clip.info['stream_epoch']
                             or int(metadata['obs_ns']) - clip.started >= args.segment_seconds * 1e9
                             or image.shape[:2] != (clip.stream.height, clip.stream.width)):
                    finished, clip = clip, None
                    finished.close(indexer)
                    index += 1
                if clip is None:
                    clip = Clip(args, index, metadata, image)
                clip.write(image, metadata)
                total += 1
                progress = time.monotonic()
                if total == 1 or progress - last_progress >= .3:
                    print(f'frame={total}', flush=True)
                    last_progress = progress
    except Stopping:
        pass
    finally:
        try:
            if clip is not None:
                clip.close(indexer)
        finally:
            indexer.shutdown(wait=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--recording-id', required=True)
    parser.add_argument('--index-directory', required=True)
    parser.add_argument('--boot-id')
    parser.add_argument('--session-id')
    parser.add_argument('--ffprobe', default='ffprobe')
    parser.add_argument('--source')
    parser.add_argument('--directory', required=True)
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--start-index', type=int, default=1)
    parser.add_argument('--segment-seconds', type=float, default=120)
    parser.add_argument('--stall-timeout', type=float, default=8)
    parser.add_argument('--codec', default='libx264')
    parser.add_argument('--fps', type=int, default=25)
    parser.add_argument('--legacy-file')
    args = parser.parse_args()
    if args.legacy_file:
        clip = clip_info(args, args.start_index)
        clip['file_name'] = Path(args.legacy_file).name
        finish_index(args, args.legacy_file, clip)
    else:
        capture(args)


if __name__ == '__main__':
    main()
