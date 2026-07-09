#!/usr/bin/env python3
"""Face tracker for gimbal target following.

Uses YOLOv8 face weights when configured, otherwise Ultra-Light-Fast-Generic-
Face-Detector ONNX with an OpenCV Haar face cascade fallback. It emits the same
TARGET:/STATUS JSON line protocol as infer_video.py so the Node backend can
reuse its gimbal loop.
"""
from __future__ import annotations

import argparse
from collections import deque
import json
import os
import threading
import time
from pathlib import Path

THREAD_ENV_DEFAULT = os.environ.get("MANTA_TRACK_THREADS") or os.environ.get("TORCH_NUM_THREADS") or os.environ.get("OMP_NUM_THREADS") or "2"
for env_name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS", "TORCH_NUM_THREADS"):
    os.environ.setdefault(env_name, THREAD_ENV_DEFAULT)
os.environ.setdefault("OPENCV_OPENCL_RUNTIME", "disabled")

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MODEL = PROJECT_ROOT / "scripts" / "models" / "version-slim-320_without_postprocessing.onnx"
DEFAULT_HAAR_CASCADE = PROJECT_ROOT / "scripts" / "models" / "haarcascade_frontalface_default.xml"
DEFAULT_SOURCE = "http://127.0.0.1:8091/stream.mjpg"


class AdaptiveTargetFilter:
    def __init__(
        self,
        alpha: float,
        max_center_speed: float,
        static_jitter_px: float = 26.0,
        fast_move_px: float = 120.0,
        prediction_seconds: float = 0.08,
    ) -> None:
        self.alpha = float(np.clip(alpha, 0.01, 1.0))
        self.max_center_speed = float(max(max_center_speed, 1.0))
        self.static_jitter_px = float(max(static_jitter_px, 1.0))
        self.fast_move_px = float(max(fast_move_px, self.static_jitter_px + 1.0))
        self.prediction_seconds = float(np.clip(prediction_seconds, 0.0, 0.25))
        self.prev: np.ndarray | None = None
        self.velocity = np.zeros(2, dtype=np.float32)
        self.updated_at = 0.0

    def reset(self) -> None:
        self.prev = None
        self.velocity[:] = 0
        self.updated_at = 0.0

    def apply(self, box: np.ndarray, timestamp: float) -> np.ndarray:
        box = np.asarray(box, dtype=np.float32)
        if self.prev is None:
            self.prev = box.copy()
            self.updated_at = timestamp
            return self.output(timestamp)

        dt = float(np.clip(timestamp - self.updated_at, 0.015, 0.25))
        predicted = self.prev[:2] + self.velocity * dt
        innovation = box[:2] - predicted
        innovation_dist = float(np.linalg.norm(innovation))
        mix = float(np.clip(
            (innovation_dist - self.static_jitter_px) / (self.fast_move_px - self.static_jitter_px),
            0.0,
            1.0,
        ))
        mix = mix * mix * (3.0 - 2.0 * mix)
        center_alpha = 0.42 + mix * (max(self.alpha, 0.90) - 0.42)
        beta = 0.08 + mix * 0.30
        size_alpha = 0.10 + mix * 0.35

        next_center = predicted + center_alpha * innovation
        next_velocity = self.velocity + (beta / dt) * innovation
        speed = float(np.linalg.norm(next_velocity))
        if speed > self.max_center_speed:
            next_velocity *= self.max_center_speed / speed
        if innovation_dist <= self.static_jitter_px and speed < self.static_jitter_px * 3.0:
            next_velocity *= 0.55

        self.prev[:2] = next_center
        self.prev[2:] += size_alpha * (box[2:] - self.prev[2:])
        self.velocity = next_velocity.astype(np.float32)
        self.updated_at = timestamp
        return self.output(timestamp)

    def output(self, timestamp: float, prediction_seconds: float | None = None) -> np.ndarray:
        if self.prev is None:
            raise RuntimeError("target filter is not initialized")
        horizon = self.prediction_seconds if prediction_seconds is None else prediction_seconds
        age = float(np.clip(timestamp - self.updated_at, 0.0, 0.25))
        output = self.prev.copy()
        output[:2] += self.velocity * float(np.clip(age + horizon, 0.0, 0.25))
        return output


class LatestFrameGrabber:
    def __init__(self, source: str) -> None:
        self.source = str(source)
        self.lock = threading.Lock()
        self.frame: np.ndarray | None = None
        self.frame_id = 0
        self.updated_at = 0.0
        self.error = ""
        self.stopped = False
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def _open(self) -> cv2.VideoCapture:
        cap = cv2.VideoCapture(self.source)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        return cap

    def _loop(self) -> None:
        cap = self._open()
        while not self.stopped:
            ok, frame = cap.read()
            if not ok:
                with self.lock:
                    self.error = "stream_unavailable"
                cap.release()
                time.sleep(0.08)
                cap = self._open()
                continue
            with self.lock:
                self.frame = frame
                self.frame_id += 1
                self.updated_at = time.monotonic()
                self.error = ""
        cap.release()

    def read_latest(self) -> tuple[bool, np.ndarray | None, int, float, str]:
        with self.lock:
            if self.frame is None:
                return False, None, self.frame_id, self.updated_at, self.error
            return True, self.frame.copy(), self.frame_id, self.updated_at, self.error


def resolve_path(value: str | Path) -> Path:
    path = Path(value)
    if path.is_absolute() or path.exists():
        return path
    return PROJECT_ROOT / path


def emit_status(status: str, frame_width: int = 0, frame_height: int = 0, detections: int = 0, **extra: object) -> None:
    payload = {
        "locked": False,
        "status": status,
        "message": "CAN NOT FIND FACE",
        "frame_w": int(frame_width or 0),
        "frame_h": int(frame_height or 0),
        "detections": int(detections or 0),
        **extra,
    }
    print("STATUS:" + json.dumps(payload, separators=(",", ":")), flush=True)


def nms(boxes: list[list[float]], scores: list[float], threshold: float) -> list[int]:
    if not boxes:
        return []
    x1 = np.array([b[0] for b in boxes], dtype=np.float32)
    y1 = np.array([b[1] for b in boxes], dtype=np.float32)
    x2 = np.array([b[2] for b in boxes], dtype=np.float32)
    y2 = np.array([b[3] for b in boxes], dtype=np.float32)
    areas = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
    order = np.argsort(np.asarray(scores, dtype=np.float32))[::-1]
    keep: list[int] = []
    while order.size:
        i = int(order[0])
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        union = areas[i] + areas[order[1:]] - inter
        iou = inter / np.maximum(union, 1e-6)
        order = order[1:][iou <= threshold]
    return keep


class UltraFaceDetector:
    def __init__(self, model_path: Path, input_width: int, input_height: int, conf: float, iou: float) -> None:
        self.input_width = int(input_width)
        self.input_height = int(input_height)
        self.conf = float(conf)
        self.iou = float(iou)
        self.net = cv2.dnn.readNetFromONNX(str(model_path))
        self.output_names = self.net.getUnconnectedOutLayersNames()

    def detect(self, frame: np.ndarray) -> tuple[list[list[float]], list[float], list[int | None]]:
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(
            frame,
            scalefactor=1.0 / 128.0,
            size=(self.input_width, self.input_height),
            mean=(127.0, 127.0, 127.0),
            swapRB=True,
            crop=False,
        )
        self.net.setInput(blob)
        outputs = self.net.forward(self.output_names)
        mapped = {name: out for name, out in zip(self.output_names, outputs)}
        boxes_raw = mapped.get("boxes", outputs[0]).reshape(-1, 4)
        scores_raw = mapped.get("scores", outputs[1]).reshape(-1, 2)
        face_scores = scores_raw[:, 1] if scores_raw.shape[1] > 1 else scores_raw[:, 0]

        boxes: list[list[float]] = []
        scores: list[float] = []
        for box, score in zip(boxes_raw, face_scores):
            score_f = float(score)
            if score_f < self.conf:
                continue
            x1, y1, x2, y2 = [float(v) for v in box]
            x1 = np.clip(x1, 0.0, 1.0) * w
            y1 = np.clip(y1, 0.0, 1.0) * h
            x2 = np.clip(x2, 0.0, 1.0) * w
            y2 = np.clip(y2, 0.0, 1.0) * h
            if x2 - x1 < 10 or y2 - y1 < 10:
                continue
            boxes.append([x1, y1, x2, y2])
            scores.append(score_f)
        keep = nms(boxes, scores, self.iou)
        return [boxes[i] for i in keep], [scores[i] for i in keep], [None] * len(keep)


class YoloFaceDetector:
    def __init__(self, model_path: Path, conf: float, iou: float, tracker: Path | None, imgsz: int, threads: int) -> None:
        import torch
        from ultralytics import YOLO

        torch.set_num_threads(max(1, int(threads)))
        try:
            torch.set_num_interop_threads(1)
        except RuntimeError:
            pass
        self.model = YOLO(str(model_path))
        self.conf = float(conf)
        self.iou = float(iou)
        self.tracker = str(tracker) if tracker else "bytetrack.yaml"
        self.imgsz = int(max(160, imgsz))

    def detect(self, frame: np.ndarray) -> tuple[list[list[float]], list[float], list[int | None]]:
        results = self.model.track(
            frame,
            persist=True,
            tracker=self.tracker,
            conf=self.conf,
            iou=self.iou,
            imgsz=self.imgsz,
            max_det=8,
            verbose=False,
        )
        if not results:
            return [], [], []
        result = results[0]
        if result.boxes is None or len(result.boxes) == 0:
            return [], [], []
        xyxy = result.boxes.xyxy.detach().cpu().numpy()
        confs = result.boxes.conf.detach().cpu().numpy()
        track_ids = result.boxes.id.detach().cpu().numpy().astype(int).tolist() if result.boxes.id is not None else [None] * len(xyxy)
        boxes: list[list[float]] = []
        scores: list[float] = []
        ids: list[int | None] = []
        h, w = frame.shape[:2]
        for box, score, track_id in zip(xyxy, confs, track_ids):
            x1, y1, x2, y2 = [float(v) for v in box]
            x1 = float(np.clip(x1, 0, w - 1))
            y1 = float(np.clip(y1, 0, h - 1))
            x2 = float(np.clip(x2, 0, w - 1))
            y2 = float(np.clip(y2, 0, h - 1))
            if x2 - x1 < 8 or y2 - y1 < 8:
                continue
            boxes.append([x1, y1, x2, y2])
            scores.append(float(score))
            ids.append(None if track_id is None else int(track_id))
        return boxes, scores, ids


class HaarFaceDetector:
    def __init__(self, conf: float) -> None:
        cascade_path = self._resolve_cascade_path()
        self.cascade = cv2.CascadeClassifier(str(cascade_path))
        if self.cascade.empty():
            raise RuntimeError(f"Cannot load Haar cascade: {cascade_path}")
        self.conf = float(conf)

    @staticmethod
    def _resolve_cascade_path() -> Path:
        candidates = [DEFAULT_HAAR_CASCADE]
        cv2_data = getattr(cv2, "data", None)
        cv2_haarcascades = getattr(cv2_data, "haarcascades", "") if cv2_data else ""
        if cv2_haarcascades:
            candidates.append(Path(cv2_haarcascades) / "haarcascade_frontalface_default.xml")
        candidates.extend([
            Path("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"),
            Path("/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml"),
            Path("/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"),
        ])
        for candidate in candidates:
            if candidate.exists():
                return candidate
        searched = ", ".join(str(path) for path in candidates)
        raise RuntimeError(f"Cannot find Haar cascade. Searched: {searched}")

    def detect(self, frame: np.ndarray) -> tuple[list[list[float]], list[float], list[int | None]]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        faces = self.cascade.detectMultiScale(gray, scaleFactor=1.08, minNeighbors=4, minSize=(28, 28))
        boxes = [[float(x), float(y), float(x + w), float(y + h)] for x, y, w, h in faces]
        return boxes, [max(self.conf, 0.7)] * len(boxes), [None] * len(boxes)


def choose_target(
    boxes: list[list[float]],
    scores: list[float],
    track_ids: list[int | None],
    previous: np.ndarray | None,
    locked_id: int | None,
) -> tuple[np.ndarray | None, float, int | None]:
    if not boxes:
        return None, 0.0, locked_id
    arr = np.asarray(boxes, dtype=np.float32)
    widths = arr[:, 2] - arr[:, 0]
    heights = arr[:, 3] - arr[:, 1]
    centers = np.column_stack(((arr[:, 0] + arr[:, 2]) * 0.5, (arr[:, 1] + arr[:, 3]) * 0.5))
    score_arr = np.asarray(scores, dtype=np.float32)
    area_score = np.sqrt(np.maximum(widths * heights, 1.0))
    if locked_id is not None and locked_id in track_ids:
        idx = track_ids.index(locked_id)
    elif previous is not None:
        dist = np.linalg.norm(centers - previous[:2], axis=1)
        gate = max(120.0, 2.5 * float(max(previous[2], previous[3])))
        continuity = np.exp(-0.5 * np.square(dist / gate))
        normalized_area = area_score / max(float(np.max(area_score)), 1.0)
        rank = score_arr * (0.35 + 0.65 * normalized_area) * (0.25 + 3.75 * continuity)
        id_bonus = np.array([1.25 if item is not None else 1.0 for item in track_ids], dtype=np.float32)
        rank *= id_bonus
        idx = int(np.argmax(rank))
    else:
        rank = score_arr * area_score
        idx = int(np.argmax(rank))
    cx, cy = centers[idx]
    return np.array([cx, cy, widths[idx], heights[idx]], dtype=np.float32), float(score_arr[idx]), track_ids[idx]


class DetectionWorker:
    def __init__(self, detector: object, grabber: LatestFrameGrabber) -> None:
        self.detector = detector
        self.grabber = grabber
        self.lock = threading.Lock()
        self.latest: dict[str, object] = {
            "frame_id": -1,
            "timestamp": 0.0,
            "target": None,
            "conf": 0.0,
            "detections": 0,
            "locked_id": None,
            "error": "",
        }
        self.previous: np.ndarray | None = None
        self.predicted: np.ndarray | None = None
        self.locked_id: int | None = None
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def snapshot(self) -> dict[str, object]:
        with self.lock:
            item = dict(self.latest)
            if isinstance(item.get("target"), np.ndarray):
                item["target"] = item["target"].copy()
            return item

    def set_prediction(self, box: np.ndarray | None) -> None:
        with self.lock:
            self.predicted = None if box is None else np.asarray(box, dtype=np.float32).copy()

    def _loop(self) -> None:
        last_frame_id = -1
        while True:
            ok, frame, frame_id, captured_at, stream_error = self.grabber.read_latest()
            if not ok or frame is None or frame_id == last_frame_id:
                time.sleep(0.008)
                continue
            last_frame_id = frame_id
            try:
                boxes, scores, track_ids = self.detector.detect(frame)
                with self.lock:
                    reference = None if self.predicted is None else self.predicted.copy()
                target, conf, self.locked_id = choose_target(
                    boxes,
                    scores,
                    track_ids,
                    reference if reference is not None else self.previous,
                    self.locked_id,
                )
                if target is not None:
                    self.previous = target.copy()
                with self.lock:
                    self.latest = {
                        "frame_id": frame_id,
                        "timestamp": captured_at,
                        "target": target,
                        "conf": conf,
                        "detections": len(boxes),
                        "locked_id": self.locked_id,
                        "error": "",
                    }
            except Exception as exc:
                with self.lock:
                    self.latest = {
                        **self.latest,
                        "frame_id": frame_id,
                        "timestamp": captured_at,
                        "target": None,
                        "error": str(exc),
                    }


class OpticalFlowBridge:
    def __init__(self, scale: float = 0.5, max_coast_seconds: float = 0.8) -> None:
        self.scale = float(np.clip(scale, 0.25, 1.0))
        self.max_coast_seconds = float(max(max_coast_seconds, 0.2))
        self.box: np.ndarray | None = None
        self.points: np.ndarray | None = None
        self.prev_gray: np.ndarray | None = None
        self.prev_frame_id = -1
        self.prev_at = 0.0
        self.last_detector_at = 0.0
        self.detector_missing = False
        self.force_reseed = False
        self.flow_quality = 0.0
        self.velocity = np.zeros(2, dtype=np.float32)
        self.cumulative_shift = np.zeros(2, dtype=np.float32)
        self.history: deque[tuple[int, np.ndarray]] = deque(maxlen=90)

    def reset(self) -> None:
        self.box = None
        self.points = None
        self.last_detector_at = 0.0
        self.detector_missing = False
        self.force_reseed = False
        self.flow_quality = 0.0
        self.velocity[:] = 0
        self.cumulative_shift[:] = 0
        self.history.clear()

    def accept_detection(self, box: np.ndarray, detector_frame_id: int, timestamp: float) -> None:
        measurement = np.asarray(box, dtype=np.float32).copy()
        if self.history:
            nearest = min(self.history, key=lambda item: abs(item[0] - detector_frame_id))
            if abs(nearest[0] - detector_frame_id) <= 3:
                measurement[:2] += self.cumulative_shift - nearest[1]

        if self.box is None:
            self.box = measurement
        else:
            distance = float(np.linalg.norm(measurement[:2] - self.box[:2]))
            correction = 0.38 if distance < max(18.0, 0.35 * max(self.box[2], self.box[3])) else 0.72
            self.box[:2] += correction * (measurement[:2] - self.box[:2])
            self.box[2:] += 0.28 * (measurement[2:] - self.box[2:])
            self.velocity *= 0.7 if correction < 0.5 else 0.35
        self.last_detector_at = timestamp
        self.detector_missing = False
        self.force_reseed = True

    def mark_detector_miss(self) -> None:
        if self.last_detector_at:
            self.detector_missing = True

    def _gray(self, frame: np.ndarray) -> np.ndarray:
        if self.scale != 1.0:
            frame = cv2.resize(frame, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def _seed(self, gray: np.ndarray) -> None:
        if self.box is None:
            self.points = None
            return
        mask = np.zeros_like(gray)
        cx, cy, bw, bh = self.box
        x1 = int(np.clip((cx - bw * 0.45) * self.scale, 0, gray.shape[1] - 1))
        y1 = int(np.clip((cy - bh * 0.45) * self.scale, 0, gray.shape[0] - 1))
        x2 = int(np.clip((cx + bw * 0.45) * self.scale, x1 + 1, gray.shape[1]))
        y2 = int(np.clip((cy + bh * 0.45) * self.scale, y1 + 1, gray.shape[0]))
        mask[y1:y2, x1:x2] = 255
        self.points = cv2.goodFeaturesToTrack(
            gray,
            mask=mask,
            maxCorners=60,
            qualityLevel=0.008,
            minDistance=5,
            blockSize=5,
        )
        self.flow_quality = min(1.0, 0.04 * (0 if self.points is None else len(self.points)))

    def update(self, frame: np.ndarray, frame_id: int, timestamp: float) -> tuple[np.ndarray | None, float, bool, float]:
        if frame_id == self.prev_frame_id:
            age = max(0.0, timestamp - self.last_detector_at) if self.last_detector_at else 0.0
            return None if self.box is None else self.box.copy(), self.flow_quality, self.detector_missing, age
        gray = self._gray(frame)
        shift = np.zeros(2, dtype=np.float32)
        dt = float(np.clip(timestamp - self.prev_at, 0.015, 0.20)) if self.prev_at else 0.04
        flow_succeeded = False
        was_reseed = self.force_reseed

        if self.box is not None and self.prev_gray is not None and self.points is not None and not self.force_reseed:
            next_points, status, _ = cv2.calcOpticalFlowPyrLK(
                self.prev_gray,
                gray,
                self.points,
                None,
                winSize=(21, 21),
                maxLevel=2,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01),
            )
            if next_points is not None and status is not None:
                back_points, back_status, _ = cv2.calcOpticalFlowPyrLK(
                    gray,
                    self.prev_gray,
                    next_points,
                    None,
                    winSize=(21, 21),
                    maxLevel=2,
                    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01),
                )
                if back_points is not None and back_status is not None:
                    old = self.points.reshape(-1, 2)
                    new = next_points.reshape(-1, 2)
                    back = back_points.reshape(-1, 2)
                    valid = (status.reshape(-1) == 1) & (back_status.reshape(-1) == 1)
                    valid &= np.linalg.norm(old - back, axis=1) < 1.8
                    if int(np.count_nonzero(valid)) >= 5:
                        deltas = new[valid] - old[valid]
                        median = np.median(deltas, axis=0)
                        deviation = np.linalg.norm(deltas - median, axis=1)
                        inliers = deviation < max(1.5, float(np.median(deviation)) * 2.5)
                        if int(np.count_nonzero(inliers)) >= 4:
                            shift = np.median(deltas[inliers], axis=0).astype(np.float32) / self.scale
                            max_shift = 1200.0 * max(timestamp - self.prev_at, 0.015)
                            norm = float(np.linalg.norm(shift))
                            if norm > max_shift:
                                shift *= max_shift / norm
                            self.box[:2] += shift
                            instant_velocity = shift / dt
                            self.velocity = (0.68 * self.velocity + 0.32 * instant_velocity).astype(np.float32)
                            self.points = new[valid][inliers].reshape(-1, 1, 2).astype(np.float32)
                            self.flow_quality = min(1.0, len(self.points) / 24.0)
                            flow_succeeded = True
                        else:
                            self.points = None
                    else:
                        self.points = None

        detector_age = max(0.0, timestamp - self.last_detector_at) if self.last_detector_at else 0.0
        predicting = self.detector_missing or (not flow_succeeded and not was_reseed and detector_age > 0.35)
        if self.box is not None and predicting and not flow_succeeded:
            decay = float(np.exp(-dt / 0.85))
            predicted_shift = self.velocity * dt
            max_prediction_step = 500.0 * dt
            prediction_norm = float(np.linalg.norm(predicted_shift))
            if prediction_norm > max_prediction_step:
                predicted_shift *= max_prediction_step / prediction_norm
            self.box[:2] += predicted_shift
            shift += predicted_shift
            self.velocity *= decay
            self.flow_quality *= decay

        if self.box is not None:
            frame_h, frame_w = frame.shape[:2]
            self.box[0] = float(np.clip(self.box[0], 0, frame_w - 1))
            self.box[1] = float(np.clip(self.box[1], 0, frame_h - 1))
            if self.force_reseed or self.points is None or len(self.points) < 10:
                self._seed(gray)
            self.force_reseed = False

        self.cumulative_shift += shift
        self.history.append((frame_id, self.cumulative_shift.copy()))
        self.prev_gray = gray
        self.prev_frame_id = frame_id
        self.prev_at = timestamp

        if self.box is None or not self.last_detector_at or detector_age > self.max_coast_seconds:
            self.box = None
            self.points = None
            self.velocity[:] = 0
            return None, 0.0, False, detector_age
        return self.box.copy(), self.flow_quality, predicting, detector_age


def run(args: argparse.Namespace) -> None:
    cv2.setNumThreads(max(1, int(args.flow_threads)))
    try:
        cv2.ocl.setUseOpenCL(False)
    except Exception:
        pass
    model_path = resolve_path(args.model)
    requested_detector = str(args.detector or "auto").strip().lower()
    detector_name = "ultra_face"
    try:
        if requested_detector in {"haar", "haar_face", "opencv_haar"}:
            detector_name = "haar_face"
            detector = HaarFaceDetector(args.conf)
        elif requested_detector in {"yolo", "yolo_face", "yolov8", "yolov8_face"} or model_path.suffix.lower() == ".pt":
            detector_name = "yolo_face"
            tracker_path = resolve_path(args.tracker) if args.tracker else None
            detector = YoloFaceDetector(model_path, args.conf, args.iou, tracker_path, args.imgsz, args.detector_threads)
        else:
            detector = UltraFaceDetector(model_path, args.input_width, args.input_height, args.conf, args.iou)
    except Exception as exc:
        fallback_error = str(exc)
        if detector_name == "yolo_face":
            try:
                detector_name = "ultra_face"
                detector = UltraFaceDetector(DEFAULT_MODEL, args.input_width, args.input_height, args.conf, args.iou)
            except Exception as ultra_exc:
                detector_name = "haar_face"
                print(json.dumps({"event": "detector_fallback", "error": fallback_error, "ultra_error": str(ultra_exc), "fallback": detector_name}), flush=True)
                detector = HaarFaceDetector(args.conf)
        else:
            detector_name = "haar_face"
            print(json.dumps({"event": "detector_fallback", "error": fallback_error, "fallback": detector_name}), flush=True)
            detector = HaarFaceDetector(args.conf)

    grabber = LatestFrameGrabber(str(args.source))
    grabber.start()
    detector_worker = DetectionWorker(detector, grabber)
    detector_worker.start()
    bridge = OpticalFlowBridge(args.flow_scale, args.max_coast_seconds)
    smoother = AdaptiveTargetFilter(
        args.smooth_alpha,
        args.max_center_speed,
        args.static_jitter_px,
        args.fast_move_px,
        args.prediction_seconds,
    )
    min_interval = 1.0 / max(float(args.loop_hz), 1.0)
    status_interval = max(0.4, min_interval)
    last_emit = 0.0
    last_status = 0.0
    last_frame_id = -1
    last_detection_frame_id = -1
    locked_id: int | None = None
    last_conf = 0.0
    last_detection_count = 0

    print(json.dumps({
        "event": "ready",
        "detector": detector_name,
        "source": str(args.source),
        "model": str(model_path),
        "conf": args.conf,
    }, separators=(",", ":")), flush=True)

    while True:
        started = time.monotonic()
        ok, frame, frame_id, captured_at, stream_error = grabber.read_latest()
        if not ok or frame is None:
            now = time.monotonic()
            if now - last_status >= status_interval:
                emit_status(stream_error or "stream_unavailable", source=str(args.source))
                last_status = now
            smoother.reset()
            time.sleep(0.05)
            continue

        if frame_id == last_frame_id:
            time.sleep(0.01)
            continue
        last_frame_id = frame_id

        frame_h, frame_w = frame.shape[:2]
        now = time.monotonic()
        detection = detector_worker.snapshot()
        detection_frame_id = int(detection.get("frame_id", -1) or -1)
        if detection_frame_id != last_detection_frame_id:
            last_detection_frame_id = detection_frame_id
            last_detection_count = int(detection.get("detections", 0) or 0)
            target = detection.get("target")
            if isinstance(target, np.ndarray):
                bridge.accept_detection(target, detection_frame_id, now)
                last_conf = float(detection.get("conf", 0.0) or 0.0)
                locked_id = detection.get("locked_id") if isinstance(detection.get("locked_id"), int) else None
            else:
                bridge.mark_detector_miss()

        target, flow_quality, predicting, prediction_age = bridge.update(frame, frame_id, captured_at or now)
        detector_worker.set_prediction(target)
        if target is not None and now - last_emit >= min_interval:
            output = smoother.apply(target, now)
            cx, cy, bw, bh = [float(v) for v in output]
            cx = float(np.clip(cx, 0, frame_w - 1))
            cy = float(np.clip(cy, 0, frame_h - 1))
            detector_age = max(prediction_age, now - bridge.last_detector_at)
            status = "face_predict" if predicting else "face_track"
            message = "PREDICTING FACE" if predicting else "FACE LOCKED"
            confidence = last_conf * float(np.exp(-detector_age / max(args.max_coast_seconds, 0.1))) if predicting else last_conf
            payload = {
                "id": locked_id if locked_id is not None else 1,
                "status": status,
                "x": cx,
                "y": cy,
                "w": bw,
                "h": bh,
                "raw_x": float(target[0]),
                "raw_y": float(target[1]),
                "frame_w": frame_w,
                "frame_h": frame_h,
                "dx": cx - frame_w * 0.5,
                "dy": cy - frame_h * 0.5,
                "vx": float(smoother.velocity[0]),
                "vy": float(smoother.velocity[1]),
                "conf": confidence,
                "detections": last_detection_count,
                "locked": True,
                "message": message,
                "detector": detector_name,
                "tracker": "bytetrack+lk" if detector_name == "yolo_face" else f"{detector_name}+lk",
                "flow_quality": round(float(flow_quality), 3),
                "detector_age_ms": round(detector_age * 1000.0),
                "prediction_age_ms": round(detector_age * 1000.0) if predicting else 0,
                "coasting": predicting,
            }
            print("TARGET:" + json.dumps(payload, separators=(",", ":")), flush=True)
            last_emit = now
            last_status = now
        elif now - last_status >= status_interval:
            smoother.reset()
            bridge.reset()
            emit_status(
                "lost",
                frame_width=frame_w,
                frame_height=frame_h,
                detections=last_detection_count,
                detector=detector_name,
                error=str(detection.get("error", "") or ""),
            )
            last_status = now

        elapsed = time.monotonic() - started
        if elapsed < min_interval:
            time.sleep(min_interval - elapsed)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Face target worker for gimbal tracking")
    parser.add_argument("--source", default=DEFAULT_SOURCE)
    parser.add_argument("--model", default=str(DEFAULT_MODEL))
    parser.add_argument("--detector", default="auto")
    parser.add_argument("--tracker", default="")
    parser.add_argument("--conf", type=float, default=0.65)
    parser.add_argument("--iou", type=float, default=0.3)
    parser.add_argument("--imgsz", type=int, default=416)
    parser.add_argument("--loop-hz", type=float, default=12.0)
    parser.add_argument("--input-width", type=int, default=320)
    parser.add_argument("--input-height", type=int, default=240)
    parser.add_argument("--smooth-alpha", type=float, default=0.6)
    parser.add_argument("--max-center-speed", type=float, default=1800.0)
    parser.add_argument("--static-jitter-px", type=float, default=26.0)
    parser.add_argument("--fast-move-px", type=float, default=120.0)
    parser.add_argument("--prediction-seconds", type=float, default=0.08)
    parser.add_argument("--flow-scale", type=float, default=0.5)
    parser.add_argument("--flow-threads", type=int, default=2)
    parser.add_argument("--detector-threads", type=int, default=3)
    parser.add_argument("--max-coast-seconds", type=float, default=2.0)
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
