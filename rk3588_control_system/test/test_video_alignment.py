import contextlib
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from fractions import Fraction
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import io


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / 'scripts'))
from media_alignment import FrameClock, frame_headers, MediaCapture  # noqa: E402


@unittest.skipUnless(all(importlib.util.find_spec(module) for module in ('av', 'cv2', 'numpy')),
                     'PyAV and OpenCV are media integration dependencies')
@unittest.skipUnless(shutil.which('ffprobe'), 'ffprobe is required for actual MP4 display indexing')
class SharedVideoTest(unittest.TestCase):
    def test_real_worker_protocol_reports_detection_empty_skip_failure_and_coast(self):
        import numpy as np
        import face_track

        class Finished(Exception):
            pass

        pixels = np.zeros((96, 160, 3), np.uint8)
        clock = FrameClock()
        metadata = [clock.observe(p, Fraction(1, 25)) for p in range(41, 47)]
        box = [40, 22, 80, 62]

        class FaceGrabber:
            def __init__(self):
                self.frames = iter((1, 3, 4))

            def read_latest(self):
                index = next(self.frames, None)
                if index is None:
                    raise Finished
                return True, pixels, index, time.monotonic(), '', metadata[index]

            def skipped(self, start, end):
                return metadata[start + 1:end]

        class FaceDetector:
            def __init__(self):
                self.index = 0

            def detect(self, frame):
                self.index += 1
                if self.index == 1:
                    return [box], [.9], [None]
                if self.index == 2:
                    return [], [], []
                raise ValueError('model failed')

        source = io.StringIO()
        with contextlib.redirect_stdout(source), self.assertRaises(Finished):
            face_track.DetectionWorker(FaceDetector(), FaceGrabber(), face_track.ActivationController())._loop()

        def results(output):
            return [json.loads(line.removeprefix('FRAME_RESULT:')) for line in output.getvalue().splitlines()
                    if line.startswith('FRAME_RESULT:')]

        face = results(source)
        self.assertEqual([r['state'] for r in face], ['detected', 'not_run', 'no_target', 'error'])
        self.assertEqual([r['frame_id'] for r in face], [frame['frame_id'] for frame in metadata[1:5]])
        self.assertEqual(face[0]['selected_index'], 0)
        self.assertIsNone(face[0]['selected_id'])
        self.assertEqual(face[0]['targets'][0]['x'], .25)

        class FakeCapture:
            def __init__(self, _source):
                self.frames = iter(range(1, 5))

            def get(self, _property):
                return 25

            def read(self):
                index = next(self.frames, None)
                if index is None:
                    raise Finished
                self.metadata = metadata[index]
                self.skipped = metadata[:1] if index == 1 else []
                return True, pixels

        class Model:
            def __init__(self, _source):
                self.index = 0

            def track(self, *_args, **_kwargs):
                self.index += 1
                if self.index == 1:
                    rows = np.array([box], dtype=float)
                    return [SimpleNamespace(boxes=type('Boxes', (), {'__len__': lambda self: 1, 'id': None,
                        'xyxy': SimpleNamespace(cpu=lambda: SimpleNamespace(numpy=lambda: rows)),
                        'conf': SimpleNamespace(cpu=lambda: SimpleNamespace(numpy=lambda: np.array([.95])))})())]
                if self.index < 4:
                    return [SimpleNamespace(boxes=None)]
                raise RuntimeError('model failed')

            def predict(self, *_args, **_kwargs):
                raise RuntimeError('model fallback also failed')

        with patch.dict(sys.modules, {'ultralytics': SimpleNamespace(YOLO=Model)}):
            import infer_video
        with tempfile.TemporaryDirectory() as root:
            weights, tracker = (Path(root) / name for name in ('model.pt', 'tracker.yaml'))
            weights.touch()
            tracker.touch()
            args = ['infer_video.py', '--weights', str(weights), '--tracker', str(tracker), '--max-coast', '1', '--loop-hz', '1000']
            with patch.object(sys, 'argv', args):
                options = infer_video.parse_args()
            source = io.StringIO()
            with patch.object(infer_video, 'LatestMediaCapture', FakeCapture), patch.object(infer_video, 'YOLO', Model), \
                    contextlib.redirect_stdout(source), self.assertRaises(Finished):
                infer_video.run(options)
        swimmer = results(source)
        self.assertEqual([r['state'] for r in swimmer], ['not_run', 'detected', 'predicted', 'no_target', 'error'])
        self.assertEqual([r['frame_id'] for r in swimmer], [frame['frame_id'] for frame in metadata[:5]])
        self.assertEqual(swimmer[2]['last_measured_frame_id'], swimmer[1]['frame_id'])

    def test_numbered_vfr_bframes_use_one_jpeg_identity_per_displayed_frame(self):
        import av
        import cv2
        import numpy as np

        present = os.getenv('MANTA_MEDIA_FIXTURE_DIR')
        with (contextlib.nullcontext(present) if present else tempfile.TemporaryDirectory()) as directory:
            base = Path(directory)
            base.mkdir(parents=True, exist_ok=True)
            for index in range(2):
                with av.open(str(base / f'source-{index}.mp4'), 'w') as media:
                    track = media.add_stream('libx264', rate=25)
                    track.width, track.height, track.pix_fmt = 160, 96, 'yuv420p'
                    track.codec_context.max_b_frames = 2
                    track.codec_context.time_base = Fraction(1, 1000)
                    elapsed = 100 + 80 * index
                    for frame_no in range(41 + 8 * index, 49 + 8 * index):
                        pixels = np.zeros((96, 160, 3), np.uint8)
                        for bit in range(8):
                            if frame_no & (1 << bit):
                                pixels[8:76, bit * 20 + 2:bit * 20 + 18] = 255
                        frame = av.VideoFrame.from_ndarray(pixels, format='bgr24')
                        frame.pts, frame.time_base = elapsed, Fraction(1, 1000)
                        elapsed += (33, 47, 55)[frame_no % 3]
                        media.mux(track.encode(frame))
                    media.mux(track.encode())

            def number(pixels):
                return sum((1 << bit) for bit in range(8) if pixels[38, bit * 20 + 9].mean() > 140)

            frames, clock = [], FrameClock()
            observed = time.monotonic_ns()
            for index in range(2):
                clock.reset()
                with av.open(str(base / f'source-{index}.mp4')) as source:
                    for offset, frame in enumerate(source.decode(video=0)):
                        # Separate decoded epochs emulate a disconnect without
                        # shortening the missing interval in the experiment.
                        metadata = clock.observe(frame.pts, frame.time_base, observed + index * 950_000_000 + offset * 40_000_000)
                        pixels = frame.to_ndarray(format='bgr24')
                        ok, jpeg = cv2.imencode('.jpg', pixels)
                        self.assertTrue(ok)
                        frames.append((jpeg.tobytes(), metadata, number(pixels)))

            class Handler(BaseHTTPRequestHandler):
                def do_GET(self):
                    self.send_response(200)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=fixture')
                    self.end_headers()
                    try:
                        for offset, (jpeg, metadata, _number) in enumerate(frames):
                            if self.path.endswith('/mobile.mjpg') and offset % 2:
                                continue
                            part = (b'--fixture\r\nContent-Type: image/jpeg\r\n'
                                    + f'Content-Length: {len(jpeg)}\r\n'.encode() + frame_headers(metadata)
                                    + b'\r\n' + jpeg + b'\r\n')
                            self.wfile.write(part[:19])
                            self.wfile.write(part[19:])
                    except OSError:
                        pass

                def log_message(self, *_args):
                    pass

            with ThreadingHTTPServer(('127.0.0.1', 0), Handler) as server:
                worker = threading.Thread(target=server.serve_forever, daemon=True)
                worker.start()
                address = f'http://127.0.0.1:{server.server_port}'
                try:
                    for endpoint, expected in [('/mobile.mjpg', frames[::2]), ('/stream.mjpg', frames)]:
                        capture = MediaCapture(address + endpoint)
                        for _, metadata, value in expected:
                            ok, pixels = capture.read()
                            self.assertTrue(ok)
                            self.assertEqual(number(pixels), value)
                            self.assertEqual(capture.metadata, metadata)
                        capture.release()
                    out = subprocess.run([sys.executable, str(PROJECT / 'scripts' / 'record_aligned_video.py'),
                                          '--source', address + '/_record.mjpg', '--directory', str(base),
                                          '--index-directory', str(base / 'clip_indexes'), '--recording-id', 'fixture',
                                          '--prefix', 'numbered', '--boot-id', 'fixture-boot', '--session-id', 'fixture-session'],
                                         timeout=35, capture_output=True, text=True)
                    self.assertEqual(out.returncode, 0, out.stderr)
                finally:
                    server.shutdown()
                    worker.join()

            expected = {meta['frame_id']: (meta['obs_ns'], value) for _, meta, value in frames}
            clips = []
            for clip_no in (1, 2):
                info = json.loads((base / 'clip_indexes' / f'fixture-{clip_no:06d}.json').read_text())
                clips.append(info)
                with av.open(str(base / info['file_name'])) as media:
                    decoded = [number(frame.to_ndarray(format='bgr24')) for frame in media.decode(video=0)]
                self.assertEqual(info['index_state'], 'ready')
                self.assertEqual(len(decoded), len(info['frames']))
                for index, (pixels, frame) in enumerate(zip(decoded, info['frames'])):
                    self.assertEqual(frame['display_no'], index)
                    self.assertEqual(frame['identity'], 'exact')
                    self.assertEqual(expected.pop(frame['frame_id']), (frame['frame_mono_ns'], pixels))
                plays = [frame['play_us'] for frame in info['frames']]
                self.assertGreater(len(set(b - a for a, b in zip(plays, plays[1:]))), 1)
            self.assertFalse(expected)
            self.assertNotEqual(clips[0]['stream_epoch'], clips[1]['stream_epoch'])
            self.assertLess(int(clips[0]['mono_end_ns']), int(clips[1]['mono_start_ns']))


if __name__ == '__main__':
    unittest.main()
