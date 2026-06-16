#!/usr/bin/env python3
"""Vision-based hand tracking that steers the rover using a YOLO hand model."""
from __future__ import annotations

import json
import signal
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

try:
    import onnxruntime as ort
except Exception:
    ort = None

try:
    from rknnlite.api import RKNNLite
except Exception:
    RKNNLite = None

PROJECT_ROOT = Path(__file__).resolve().parent.parent
CFG = json.loads((PROJECT_ROOT / "config" / "system.config.json").read_text())
VCFG = CFG.get("vision", {}) or {}
CAMERA = CFG.get("camera", {}) or {}

SNAPSHOT_URL = VCFG.get("snapshot_url") or CAMERA.get("local_source_url", "http://127.0.0.1:8090/snapshot.jpg")
MOTOR_API = VCFG.get("motor_api") or f"http://127.0.0.1:{int(CFG.get('web_port', 3000))}/api/control/motor"
LEFT_CH = int(CFG.get("rover_left_channel", 1))
RIGHT_CH = int(CFG.get("rover_right_channel", 3))
PWM_CENTER = int(CFG.get("default_motor_pwm", 1500))
PWM_MAX_FORWARD = int(VCFG.get("max_pwm", 1800))
PWM_DELTA = PWM_MAX_FORWARD - PWM_CENTER
TRACK_MIN_FORWARD_PWM = int(VCFG.get("track_min_forward_pwm", 80))
LOOP_HZ = float(VCFG.get("loop_hz", 10.0))
DETECT_WIDTH = int(VCFG.get("detect_width", 640))
INFER_IMGSZ = int(VCFG.get("infer_imgsz", DETECT_WIDTH))
LOST_SECONDS = float(VCFG.get("lost_seconds", 1.5))
YOLO_CONF = float(VCFG.get("conf_threshold", 0.6))
FAR_CONF = float(VCFG.get("far_conf_threshold", min(0.4, YOLO_CONF)))
INFER_CONF = min(YOLO_CONF, FAR_CONF)
FAR_AREA_RATIO = float(VCFG.get("far_area_ratio", 0.02))
YOLO_IOU = float(VCFG.get("iou_threshold", 0.45))
SMOOTH_ALPHA = float(VCFG.get("smooth_alpha", 0.8))  # weight on the newest measurement
MAX_BOX_AREA_RATIO = float(VCFG.get("max_box_area_ratio", 0.9))
MIN_BOX_AREA_RATIO = float(VCFG.get("min_box_area_ratio", 0.002))
TILE_RETRY_ENABLED = bool(VCFG.get("tile_retry_enabled", True))
TILE_GRID = max(1, int(VCFG.get("tile_grid", 2)))
TILE_OVERLAP = max(0.0, min(0.45, float(VCFG.get("tile_overlap", 0.25))))
TILE_MAX_PASSES = max(1, int(VCFG.get("tile_max_passes", 4)))


def _resolve_model_paths() -> list[str]:
    cfg_path = VCFG.get("model_path")
    candidates = []
    if cfg_path:
        candidates.append(Path(cfg_path))
        candidates.append(PROJECT_ROOT / cfg_path)
    candidates += [
        PROJECT_ROOT / "scripts" / "models" / "best.rknn",
        PROJECT_ROOT / "scripts" / "models" / "best.onnx",
        PROJECT_ROOT / "scripts" / "models" / "best.pt",
    ]
    resolved = []
    for c in candidates:
        if c and c.exists():
            path_str = str(c)
            if path_str not in resolved:
                resolved.append(path_str)
    return resolved


MODEL_PATHS = _resolve_model_paths()

_stop = False


def _on_signal(_signum, _frame):
    global _stop
    _stop = True


signal.signal(signal.SIGTERM, _on_signal)
signal.signal(signal.SIGINT, _on_signal)


def _letterbox(frame, size):
    h, w = frame.shape[:2]
    target = (size, size)
    ratio = min(target[0] / max(1, h), target[1] / max(1, w))
    new_w = max(1, int(round(w * ratio)))
    new_h = max(1, int(round(h * ratio)))
    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    dw = target[1] - new_w
    dh = target[0] - new_h
    left = int(round(dw / 2 - 0.1))
    right = int(round(dw / 2 + 0.1))
    top = int(round(dh / 2 - 0.1))
    bottom = int(round(dh / 2 + 0.1))
    canvas = cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return canvas, ratio, left, top


def _blob_from_frame(frame):
    padded, ratio, pad_x, pad_y = _letterbox(frame, INFER_IMGSZ)
    rgb = padded[:, :, ::-1].astype(np.float32) / 255.0
    blob_nchw = rgb.transpose(2, 0, 1)[None]
    blob_nhwc = rgb[None]
    return blob_nchw, blob_nhwc, ratio, pad_x, pad_y


def _nms_numpy(boxes, scores, iou_thresh):
    if boxes.size == 0:
        return []
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        inter_w = np.maximum(0.0, xx2 - xx1)
        inter_h = np.maximum(0.0, yy2 - yy1)
        inter = inter_w * inter_h
        union = areas[i] + areas[order[1:]] - inter
        iou = np.where(union > 0.0, inter / union, 0.0)
        order = order[np.where(iou <= iou_thresh)[0] + 1]
    return keep


def _decode_raw_output(raw, frame_shape, ratio, pad_x, pad_y):
    h_img, w_img = frame_shape[:2]
    frame_area = float(max(1, w_img * h_img))
    pred = np.array(raw)
    if pred.ndim == 3:
        pred = pred[0]
    if pred.shape[0] < pred.shape[1]:
        pred = pred.transpose(1, 0)
    boxes_xywh = pred[:, :4]
    scores = pred[:, 4]

    x = boxes_xywh[:, 0]
    y = boxes_xywh[:, 1]
    w = boxes_xywh[:, 2]
    h = boxes_xywh[:, 3]
    boxes = np.stack((x - w / 2.0, y - h / 2.0, x + w / 2.0, y + h / 2.0), axis=1)
    boxes[:, [0, 2]] -= pad_x
    boxes[:, [1, 3]] -= pad_y
    boxes /= max(ratio, 1e-6)
    boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, w_img)
    boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, h_img)

    candidate_idx = []
    for i, score in enumerate(scores):
        x1, y1, x2, y2 = boxes[i]
        bw = max(0.0, x2 - x1)
        bh = max(0.0, y2 - y1)
        if bw <= 0.0 or bh <= 0.0:
            continue
        area_ratio = (bw * bh) / frame_area
        if area_ratio > MAX_BOX_AREA_RATIO or area_ratio < MIN_BOX_AREA_RATIO:
            continue
        conf_floor = FAR_CONF if area_ratio < FAR_AREA_RATIO else YOLO_CONF
        if float(score) < conf_floor:
            continue
        if bw >= w_img * 0.95 and bh >= h_img * 0.95:
            continue
        candidate_idx.append(i)

    if not candidate_idx:
        return []

    cand_boxes = boxes[candidate_idx]
    cand_scores = scores[candidate_idx]
    keep_local = _nms_numpy(cand_boxes, cand_scores, YOLO_IOU)

    dets = []
    for local_i in keep_local:
        idx = candidate_idx[local_i]
        x1, y1, x2, y2 = boxes[idx]
        dets.append(([int(x1), int(y1), int(max(0.0, x2 - x1)), int(max(0.0, y2 - y1))], float(scores[idx])))
    return dets


class OnnxDetector:
    name = "onnx-hand"

    def __init__(self, model_path):
        if ort is None:
            raise RuntimeError("onnxruntime is not available")
        self.model_path = model_path
        self.session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        print(f"[vision] model={model_path} backend=onnxruntime imgsz={INFER_IMGSZ}", flush=True)

    def detect(self, frame):
        blob_nchw, _, ratio, pad_x, pad_y = _blob_from_frame(frame)
        raw = self.session.run(None, {self.input_name: blob_nchw})[0]
        return _decode_raw_output(raw, frame.shape, ratio, pad_x, pad_y)


class RKNNDetector:
    name = "rknn-hand"

    def __init__(self, model_path):
        if RKNNLite is None:
            raise RuntimeError("rknn-toolkit-lite2 is not available")
        self.model_path = model_path
        self.runtime = RKNNLite()
        ret = self.runtime.load_rknn(model_path)
        if ret != 0:
            raise RuntimeError(f"load_rknn failed: {ret}")
        ret = self.runtime.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
        if ret != 0:
            raise RuntimeError(f"init_runtime failed: {ret}")
        print(f"[vision] model={model_path} backend=rknnlite imgsz={INFER_IMGSZ}", flush=True)

    def detect(self, frame):
        _, blob_nhwc, ratio, pad_x, pad_y = _blob_from_frame(frame)
        outputs = self.runtime.inference(inputs=[blob_nhwc], data_format=["nhwc"])
        if not outputs:
            return []
        return _decode_raw_output(outputs[0], frame.shape, ratio, pad_x, pad_y)


def _scale_rect(rect, scale_x, scale_y):
    return [
        int(rect[0] * scale_x),
        int(rect[1] * scale_y),
        int(rect[2] * scale_x),
        int(rect[3] * scale_y),
    ]


def _rect_iou(a, b):
    ax1, ay1, aw, ah = a
    bx1, by1, bw, bh = b
    ax2, ay2 = ax1 + aw, ay1 + ah
    bx2, by2 = bx1 + bw, by1 + bh
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    iw = max(0, ix2 - ix1)
    ih = max(0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    union = aw * ah + bw * bh - inter
    return inter / max(1.0, float(union))


def _dedupe_dets(dets, iou_thresh=0.45):
    kept = []
    for rect, conf in sorted(dets, key=lambda item: item[1], reverse=True):
        if any(_rect_iou(rect, existing[0]) >= iou_thresh for existing in kept):
            continue
        kept.append((rect, conf))
    return kept


def detect_with_fallback(detector, frame):
    h, w = frame.shape[:2]
    dets = []
    if w > DETECT_WIDTH:
        scaled_w = DETECT_WIDTH
        scaled_h = max(1, int(h * DETECT_WIDTH / w))
        base = cv2.resize(frame, (scaled_w, scaled_h))
        scale_x = w / float(scaled_w)
        scale_y = h / float(scaled_h)
        for rect, conf in detector.detect(base):
            dets.append((_scale_rect(rect, scale_x, scale_y), conf))
    else:
        dets = detector.detect(frame)

    if dets or not TILE_RETRY_ENABLED:
        return _dedupe_dets(dets, YOLO_IOU), w, h

    step_x = max(1, int(w / TILE_GRID))
    step_y = max(1, int(h / TILE_GRID))
    overlap_x = int(step_x * TILE_OVERLAP)
    overlap_y = int(step_y * TILE_OVERLAP)
    tile_dets = []
    passes = 0
    y = 0
    while y < h and passes < TILE_MAX_PASSES:
        x = 0
        tile_h = min(h, y + step_y + overlap_y) - y
        while x < w and passes < TILE_MAX_PASSES:
            tile_w = min(w, x + step_x + overlap_x) - x
            tile = frame[y:y + tile_h, x:x + tile_w]
            for rect, conf in detector.detect(tile):
                tile_dets.append(([rect[0] + x, rect[1] + y, rect[2], rect[3]], conf))
            passes += 1
            if x + step_x >= w:
                break
            x += step_x - overlap_x
        if y + step_y >= h:
            break
        y += step_y - overlap_y

    return _dedupe_dets(tile_dets, YOLO_IOU), w, h


def build_detector():
    errors = []
    for model_path in MODEL_PATHS:
        suffix = Path(model_path).suffix.lower()
        try:
            if suffix == ".rknn":
                det = RKNNDetector(model_path)
            elif suffix == ".onnx":
                det = OnnxDetector(model_path)
            else:
                errors.append(f"unsupported model format: {model_path}")
                continue
            print(f"[vision] using hand model: {model_path}", flush=True)
            return det
        except Exception as exc:
            errors.append(f"{model_path}: {exc}")
            print(f"[vision] hand model load failed ({model_path}): {exc}", flush=True)
    raise RuntimeError("; ".join(errors) if errors else "no usable hand model found")


def post_motors(left_pwm: int, right_pwm: int) -> None:
    body = json.dumps({
        "motors": [
            {"channel": LEFT_CH, "pwm": int(left_pwm)},
            {"channel": RIGHT_CH, "pwm": int(right_pwm)},
        ]
    }).encode()
    req = urllib.request.Request(
        MOTOR_API, data=body, headers={"Content-Type": "application/json"}, method="POST"
    )
    try:
        urllib.request.urlopen(req, timeout=0.5).read()
    except Exception as exc:
        print(f"[vision] motor POST failed: {exc}", flush=True)


def fetch_frame():
    try:
        with urllib.request.urlopen(SNAPSHOT_URL, timeout=0.6) as resp:
            buf = resp.read()
    except Exception as exc:
        print(f"[vision] snapshot fetch failed: {exc}", flush=True)
        return None
    arr = np.frombuffer(buf, dtype=np.uint8)
    if arr.size == 0:
        return None
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


def main() -> int:
    detector = build_detector()
    period = 1.0 / max(LOOP_HZ, 1.0)
    last_detect_t = 0.0
    last_rect = None
    last_left = PWM_CENTER
    last_right = PWM_CENTER
    print(
        f"[vision] start backend={detector.name} url={SNAPSHOT_URL} motor_api={MOTOR_API} "
        f"channels L={LEFT_CH} R={RIGHT_CH} max_pwm={PWM_MAX_FORWARD} hz={LOOP_HZ}",
        flush=True,
    )
    post_motors(PWM_CENTER, PWM_CENTER)
    while not _stop:
        t0 = time.time()
        frame = fetch_frame()
        left_pwm = PWM_CENTER
        right_pwm = PWM_CENTER
        det_payload = None
        if frame is not None:
            h, w = frame.shape[:2]
            try:
                dets, work_w, work_h = detect_with_fallback(detector, frame)
            except Exception as exc:
                print(f"[vision] detect error: {exc}", flush=True)
                dets = []
                work_w, work_h = w, h

            if dets:
                best = max(dets, key=lambda d: d[1])
                rect, conf = best
                if last_rect is not None:
                    alpha = SMOOTH_ALPHA
                    rect = [int(alpha * n + (1.0 - alpha) * o) for n, o in zip(rect, last_rect)]
                last_rect = rect
                biggest = rect
                cx = biggest[0] + biggest[2] * 0.5
                offset = (cx - work_w * 0.5) / (work_w * 0.5)
                offset = max(-1.0, min(1.0, offset))
                forward = max(TRACK_MIN_FORWARD_PWM, int((1.0 - abs(offset)) * PWM_DELTA))
                turn = int(offset * (PWM_DELTA * 0.35))
                left_pwm = max(PWM_CENTER, min(PWM_MAX_FORWARD, PWM_CENTER + forward - turn))
                right_pwm = max(PWM_CENTER, min(PWM_MAX_FORWARD, PWM_CENTER + forward + turn))
                last_detect_t = time.time()
                det_payload = {
                    "w": work_w, "h": work_h,
                    "rects": [biggest],
                    "offset": round(offset, 3),
                    "conf": round(conf, 3),
                    "left_pwm": left_pwm, "right_pwm": right_pwm,
                    "t": time.time(),
                }
            elif (time.time() - last_detect_t) <= LOST_SECONDS and last_rect is not None:
                left_pwm = last_left
                right_pwm = last_right
                det_payload = {
                    "w": w, "h": h, "rects": [last_rect], "stale": True, "t": time.time(),
                }
            else:
                last_rect = None
                det_payload = {"w": w, "h": h, "rects": [], "t": time.time()}

        if det_payload is not None:
            print("DETECT:" + json.dumps(det_payload), flush=True)

        if left_pwm != last_left or right_pwm != last_right:
            post_motors(left_pwm, right_pwm)
            last_left, last_right = left_pwm, right_pwm

        elapsed = time.time() - t0
        if elapsed < period:
            time.sleep(period - elapsed)
    post_motors(PWM_CENTER, PWM_CENTER)
    print("[vision] stop", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
