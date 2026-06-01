#!/usr/bin/env python3
"""Vision-based person tracking that steers the rover.

Detection backends, in priority order:
  1. Ultralytics YOLOv8 (if ``ultralytics`` is installed and a model file is
     present at ``scripts/models/yolov8n.pt`` / .onnx).
  2. OpenCV HOG pedestrian detector + MOG2 background-subtraction fallback
     (used when no YOLO model is available; works fully offline with stock
     OpenCV 4.5).
"""
from __future__ import annotations

import json
import signal
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

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
LOOP_HZ = float(VCFG.get("loop_hz", 5.0))
DETECT_WIDTH = int(VCFG.get("detect_width", 480))
LOST_SECONDS = float(VCFG.get("lost_seconds", 1.5))
YOLO_CONF = float(VCFG.get("conf_threshold", 0.25))
YOLO_IOU = float(VCFG.get("iou_threshold", 0.45))
MODEL_DIR = PROJECT_ROOT / "scripts" / "models"

_stop = False


def _on_signal(_signum, _frame):
    global _stop
    _stop = True


signal.signal(signal.SIGTERM, _on_signal)
signal.signal(signal.SIGINT, _on_signal)


class YoloDetector:
    name = "yolov8"

    def __init__(self, model_path: Path):
        from ultralytics import YOLO  # noqa: WPS433
        self.model = YOLO(str(model_path))

    def detect(self, frame):
        res = self.model.predict(
            frame,
            classes=[0],
            verbose=False,
            imgsz=DETECT_WIDTH,
            conf=YOLO_CONF,
            iou=YOLO_IOU,
            device='cpu'
        )[0]
        rects = []
        for box in res.boxes.xyxy.cpu().numpy():
            x1, y1, x2, y2 = [int(v) for v in box[:4]]
            rects.append([x1, y1, x2 - x1, y2 - y1])
        return rects


class HogMog2Detector:
    name = "hog+mog2"

    def __init__(self):
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self.mog = cv2.createBackgroundSubtractorMOG2(history=200, varThreshold=25, detectShadows=False)
        self._warm = 0

    def detect(self, frame):
        rects = []
        try:
            hog_rects, weights = self.hog.detectMultiScale(
                frame, winStride=(4, 4), padding=(8, 8), scale=1.05
            )
            for (x, y, w, h), wt in zip(hog_rects, weights):
                if wt >= 0.4:
                    rects.append([int(x), int(y), int(w), int(h)])
        except cv2.error:
            pass

        mask = self.mog.apply(frame)
        self._warm += 1
        if self._warm < 8:
            return rects
        mask = cv2.medianBlur(mask, 5)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.dilate(mask, np.ones((9, 9), np.uint8), iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        h_img, w_img = frame.shape[:2]
        min_area = (w_img * h_img) * 0.01
        best = None
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            area = w * h
            if area < min_area:
                continue
            if h < w * 0.8:
                continue
            if best is None or area > best[2] * best[3]:
                best = [x, y, w, h]
        if best is not None and not rects:
            rects.append(best)
        return rects


def build_detector():
    for ext in ("pt", "onnx"):
        model_path = MODEL_DIR / f"yolov8n.{ext}"
        if model_path.exists():
            try:
                det = YoloDetector(model_path)
                print(f"[vision] using YOLOv8 model {model_path.name}", flush=True)
                return det
            except Exception as exc:
                print(f"[vision] YOLO load failed ({exc}); falling back", flush=True)
                break
    print("[vision] using HOG + MOG2 fallback (no YOLO model)", flush=True)
    return HogMog2Detector()


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
            if w > DETECT_WIDTH:
                new_w = DETECT_WIDTH
                new_h = max(1, int(h * DETECT_WIDTH / w))
                small = cv2.resize(frame, (new_w, new_h))
            else:
                small = frame
                new_w, new_h = w, h
            try:
                rects = detector.detect(small)
            except Exception as exc:
                print(f"[vision] detect error: {exc}", flush=True)
                rects = []

            if rects:
                biggest = max(rects, key=lambda r: r[2] * r[3])
                if last_rect is not None:
                    biggest = [int(0.5 * a + 0.5 * b) for a, b in zip(last_rect, biggest)]
                last_rect = biggest
                cx = biggest[0] + biggest[2] * 0.5
                offset = (cx - new_w * 0.5) / (new_w * 0.5)
                offset = max(-1.0, min(1.0, offset))
                forward = max(TRACK_MIN_FORWARD_PWM, int((1.0 - abs(offset)) * PWM_DELTA))
                turn = int(offset * (PWM_DELTA * 0.35))
                left_pwm = max(PWM_CENTER, min(PWM_MAX_FORWARD, PWM_CENTER + forward - turn))
                right_pwm = max(PWM_CENTER, min(PWM_MAX_FORWARD, PWM_CENTER + forward + turn))
                last_detect_t = time.time()
                det_payload = {
                    "w": new_w, "h": new_h,
                    "rects": [biggest],
                    "offset": round(offset, 3),
                    "left_pwm": left_pwm, "right_pwm": right_pwm,
                    "t": time.time(),
                }
            elif (time.time() - last_detect_t) <= LOST_SECONDS and last_rect is not None:
                left_pwm = last_left
                right_pwm = last_right
                det_payload = {
                    "w": new_w, "h": new_h, "rects": [last_rect], "stale": True, "t": time.time(),
                }
            else:
                last_rect = None
                det_payload = {"w": new_w, "h": new_h, "rects": [], "t": time.time()}

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
