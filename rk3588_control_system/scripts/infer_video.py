"""Swimmer detection worker that emits smoothed target coordinates for gimbal control.

Pipeline: YOLO(best.pt) -> ByteTrack ID association -> target lock/Re-ID ->
Kalman stabilization -> tuned VFT target point -> TARGET JSON lines.
"""
from __future__ import annotations
import argparse
import json
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_WEIGHTS = PROJECT_ROOT / "best.pt"
DEFAULT_SOURCE = "http://127.0.0.1:8091/stream.mjpg"
DEFAULT_TRACKER = PROJECT_ROOT / "bytetrack_swimmer.yaml"


# ----------------------------------------------------------------------------
# 卡尔曼框滤波器 (Constant-Velocity Box Kalman)
#   状态 x = [cx, cy, w, h, vx, vy, vw, vh]^T  (位置+尺寸 及其速度)
#   测量 z = [cx, cy, w, h]^T                  (YOLO 检测框中心+尺寸)
#   恒速模型: 位置每帧 += 速度*dt; 速度随机游走。
#   调参直觉:
#     q (过程噪声) 越大 -> 越信任新测量、跟手但抖;
#     r (测量噪声) 越大 -> 越信任预测、平滑但滞后。
#   稳定优先就把 r 调大 (默认 r=20)。
# ----------------------------------------------------------------------------
class BoxKalman:
    def __init__(self, q: float = 1.0, r: float = 20.0) -> None:
        self.q = float(q)
        self.r = float(r)
        self.kf = cv2.KalmanFilter(8, 4)
        self.kf.measurementMatrix = np.eye(4, 8, dtype=np.float32)
        self._set_dt(1.0 / 30.0)
        self.kf.processNoiseCov = np.eye(8, dtype=np.float32) * self.q
        # 速度分量给更小过程噪声 -> 速度估计平滑, 避免恒速模型外推过冲(抖动主因之一)
        self.kf.processNoiseCov[4:, 4:] *= 0.25
        self.kf.measurementNoiseCov = np.eye(4, dtype=np.float32) * self.r
        self.initialized = False
        self.miss = 0  # 连续未校正(纯预测)帧数
        self._last_pred = np.zeros(4, dtype=np.float32)

    def _set_dt(self, dt: float) -> None:
        F = np.eye(8, dtype=np.float32)
        for i in range(4):
            F[i, i + 4] = dt
        self.kf.transitionMatrix = F

    def reset(self) -> None:
        self.initialized = False
        self.miss = 0

    def _init_state(self, box: np.ndarray) -> None:
        cx, cy, w, h = box
        st = np.array([cx, cy, w, h, 0, 0, 0, 0], dtype=np.float32).reshape(8, 1)
        self.kf.statePost = st.copy()
        self.kf.statePre = st.copy()
        self.kf.errorCovPost = np.eye(8, dtype=np.float32) * 10.0
        self.initialized = True
        self.miss = 0

    def predict(self, dt: float) -> np.ndarray | None:
        """推进一帧 (predict 步)。返回预测 [cx,cy,w,h] 或 None(未初始化)。"""
        if not self.initialized:
            return None
        self._set_dt(dt)
        p = self.kf.predict()
        self._last_pred = p[:4, 0].copy()
        return self._last_pred

    def correct(self, box: np.ndarray) -> np.ndarray:
        """用检测框校正 (update 步)。box=[cx,cy,w,h]。返回校正后状态。"""
        if not self.initialized:
            self._init_state(box)
            return box.astype(np.float32).copy()
        z = np.asarray(box, dtype=np.float32).reshape(4, 1)
        s = self.kf.correct(z)
        self.miss = 0
        return s[:4, 0].copy()

    def coast(self) -> np.ndarray:
        """无检测时: 把预测当成本帧状态写回 statePost, 让下一帧 predict 接着推。"""
        self.kf.statePost = self.kf.statePre.copy()
        self.kf.errorCovPost = self.kf.errorCovPre.copy()
        self.miss += 1
        return self.kf.statePost[:4, 0].copy()

    def is_measurement_valid(self, meas: np.ndarray, gate_dist_px: float,
                             gate_scale: float) -> bool:
        """基于当前预测状态做测量门限检查，拒绝突发大跳框。"""
        if not self.initialized:
            return True
        px, py, pw, ph = self.kf.statePre[:4, 0].astype(np.float32)
        mx, my, mw, mh = np.asarray(meas, dtype=np.float32)
        dist = float(np.hypot(mx - px, my - py))
        if dist > float(gate_dist_px):
            return False
        # 尺寸突变门限，例如 gate_scale=2.2 表示单帧尺寸变化不能超过 2.2x
        if pw > 1.0 and ph > 1.0:
            wr = max(mw, 1.0) / max(pw, 1.0)
            hr = max(mh, 1.0) / max(ph, 1.0)
            if wr > gate_scale or wr < 1.0 / gate_scale:
                return False
            if hr > gate_scale or hr < 1.0 / gate_scale:
                return False
        return True


class OutputLimiter:
    """输出端二级平滑(给云台的目标点)。与卡尔曼互补:
       1) 安全护栏: 限制单帧中心位移/尺寸变化 -> 砍掉漏网的大跳;
       2) 指数低通(EMA): 真正消除高频小抖, 无粘滑阶梯效应。
    ema_alpha 越小越平滑(滞后越大); 尺寸用更狠的平滑(size_alpha)。
    """
    def __init__(self, ema_alpha: float = 0.45,
                 max_center_speed: float = 800.0,
                 max_size_rate: float = 1.25,
                 hold_x_px: float = 100,
                 hold_y_px: float = 100,
                 hold_release_px: float = 150,
                 deadzone_beta: float = 0.15) -> None:
        self.ema_alpha = float(ema_alpha)
        self.size_alpha = min(self.ema_alpha * 0.5, 1.0)  # 框尺寸变化慢, 更狠地平滑
        self.max_center_speed = float(max_center_speed)
        self.max_size_rate = float(max_size_rate)
        self.hold_x_px = float(hold_x_px)
        self.hold_y_px = float(hold_y_px)
        self.hold_release_px = float(max(hold_release_px, hold_x_px, hold_y_px))
        self.deadzone_beta = float(deadzone_beta)  # 死区内误差衰减系数 0~1
        self.prev: np.ndarray | None = None  # [cx,cy,w,h]
        self.holding = False

    def reset(self) -> None:
        self.prev = None
        self.holding = False

    def apply(self, box: np.ndarray, dt: float) -> np.ndarray:
        x = np.asarray(box, dtype=np.float32).copy()
        if self.prev is None:
            self.prev = x.copy()
            return x
        out = self.prev.copy()

        # 1) 安全护栏: 限制单帧中心位移 (防大跳)
        max_step = max(self.max_center_speed * max(dt, 1e-3), 1.0)
        vx, vy = float(x[0] - out[0]), float(x[1] - out[1])
        v = float(np.hypot(vx, vy))
        if v > max_step and v > 1e-6:
            s = max_step / v
            x[0], x[1] = out[0] + vx * s, out[1] + vy * s

        # 2) 安全护栏: 限制单帧尺寸变化
        for i in (2, 3):
            old = max(float(out[i]), 1.0)
            x[i] = float(np.clip(x[i], old / self.max_size_rate, old * self.max_size_rate))

        # 3) 指数低通: 平滑消抖 (无死区, 不产生阶梯跳变)
        res = out.copy()
        res[0] = out[0] + self.ema_alpha * (x[0] - out[0])
        res[1] = out[1] + self.ema_alpha * (x[1] - out[1])
        res[2] = out[2] + self.size_alpha * (x[2] - out[2])
        res[3] = out[3] + self.size_alpha * (x[3] - out[3])

        # 4) 连续推箱死区 (Continuous Deadzone): 
        #    如果误差在阈值内, 按 beta 衰减 (beta=0则绝对冻结, 杜绝微小画圆)。
        #    如果误差超出阈值, 只移动超出死区的那部分, 保证无突跳跳变。
        dx = float(res[0] - out[0])
        dy = float(res[1] - out[1])
        
        if dx > self.hold_x_px:
            res[0] = out[0] + (dx - self.hold_x_px)
        elif dx < -self.hold_x_px:
            res[0] = out[0] + (dx + self.hold_x_px)
        else:
            res[0] = out[0] + dx * self.deadzone_beta
            
        if dy > self.hold_y_px:
            res[1] = out[1] + (dy - self.hold_y_px)
        elif dy < -self.hold_y_px:
            res[1] = out[1] + (dy + self.hold_y_px)
        else:
            res[1] = out[1] + dy * self.deadzone_beta

        self.prev = res.copy()
        return res


# ----------------------------------------------------------------------------
# 目标锁定器 (单目标 + 轻量颜色 Re-ID 保锁)
#   - 首次: 选 conf*sqrt(area) 最大者锁定其 tracker id
#   - 该 id 在 -> 直接用; 该 id 没了 -> 用 HSV 颜色直方图在候选里找回
#   - 找回看 颜色相似度*0.7 + 尺寸合理性*0.3, 超阈值就把锁转到新 id
# ----------------------------------------------------------------------------
class TargetLock:
    def __init__(self, reid_sim: float = 0.5) -> None:
        self.locked_id: int | None = None
        self.reid_sim = float(reid_sim)
        self.feat: np.ndarray | None = None      # 锁定目标的颜色特征 (EMA 更新)
        self.last_h: float = 0.0
        self.status = "init"

    def reset(self) -> None:
        self.locked_id = None
        self.feat = None
        self.last_h = 0.0
        self.status = "init"

    @staticmethod
    def _hsv_feat(frame: np.ndarray, box: np.ndarray) -> np.ndarray | None:
        x1, y1, x2, y2 = [int(v) for v in box]
        H, W = frame.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(W, x2), min(H, y2)
        if x2 - x1 < 8 or y2 - y1 < 8:
            return None
        hsv = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [16, 16], [0, 180, 0, 256])
        cv2.normalize(hist, hist, norm_type=cv2.NORM_L2)
        return hist.flatten().astype(np.float32)

    def _update_feat(self, frame: np.ndarray, box: np.ndarray) -> None:
        f = self._hsv_feat(frame, box)
        if f is None:
            return
        self.feat = f if self.feat is None else 0.7 * self.feat + 0.3 * f
        self.last_h = float(box[3] - box[1])

    def update(self, frame: np.ndarray, ids: np.ndarray, xyxy: np.ndarray,
               confs: np.ndarray) -> tuple[int | None, np.ndarray | None, bool, float]:
        """返回 (locked_id, xyxy or None, just_reacquired, conf)。"""
        if len(ids) == 0:
            self.status = "no_det"
            return self.locked_id, None, False, 0.0

        # 锁定 id 仍在
        if self.locked_id is not None:
            hit = np.where(ids == self.locked_id)[0]
            if len(hit) > 0:
                i = int(hit[0])
                self._update_feat(frame, xyxy[i])
                self.status = "track"
                return self.locked_id, xyxy[i], False, float(confs[i])

        # 锁定 id 丢了, 且有颜色记忆 -> Re-ID 找回
        if self.locked_id is not None and self.feat is not None:
            best_i, best_s = -1, 0.0
            for i in range(len(ids)):
                f = self._hsv_feat(frame, xyxy[i])
                if f is None:
                    continue
                app = float(np.dot(self.feat, f))
                nh = float(xyxy[i][3] - xyxy[i][1])
                size = min(nh, self.last_h) / max(nh, self.last_h, 1.0)
                score = 0.7 * app + 0.3 * size
                if score > best_s:
                    best_s, best_i = score, i
            if best_i >= 0 and best_s >= self.reid_sim:
                self.locked_id = int(ids[best_i])
                self._update_feat(frame, xyxy[best_i])
                self.status = f"reid({best_s:.2f})"
                return self.locked_id, xyxy[best_i], True, float(confs[best_i])
            self.status = "lost"
            return self.locked_id, None, False, 0.0

        # 首次锁定
        w = xyxy[:, 2] - xyxy[:, 0]
        h = xyxy[:, 3] - xyxy[:, 1]
        area = np.clip(w * h, 1.0, None)
        best = int(np.argmax(confs * np.sqrt(area)))
        self.locked_id = int(ids[best])
        self._update_feat(frame, xyxy[best])
        self.status = "init_pick"
        return self.locked_id, xyxy[best], True, float(confs[best])


def tuned_target_point_from_box(box: np.ndarray, vft_alpha: float,
                                center_hist: deque | None = None) -> tuple[float, float, float, float]:
    """Return the calibrated smooth target point basis used by debug and gimbal tracking."""
    x1, y1, x2, y2 = [float(value) for value in box]
    box_width = float(x2 - x1)
    box_height = float(y2 - y1)
    raw_cx = float((x1 + x2) / 2.0)
    raw_cy = float(y1 + float(vft_alpha) * box_height)
    if center_hist is not None:
        center_hist.append((raw_cx, raw_cy))
        centers = np.asarray(center_hist, dtype=np.float32)
        raw_cx = float(np.median(centers[:, 0]))
        raw_cy = float(np.median(centers[:, 1]))
    return raw_cx, raw_cy, box_width, box_height


def size_within_tolerance(width: float, height: float, locked_wh: np.ndarray,
                          size_tol: float) -> bool:
    wr = float(width) / max(float(locked_wh[0]), 1.0)
    hr = float(height) / max(float(locked_wh[1]), 1.0)
    lo, hi = 1.0 / (1.0 + float(size_tol)), 1.0 + float(size_tol)
    return bool(lo <= wr <= hi and lo <= hr <= hi)


def resolve_path(value: str | Path) -> Path:
    path = Path(value)
    if path.is_absolute() or path.exists():
        return path
    return PROJECT_ROOT / path


def emit_status(status: str, message: str = "can not find swimmer", frame_width: int = 0,
                frame_height: int = 0, detections: int = 0, locked_id: int | None = None,
                conf: float = 0.0, **extra: object) -> None:
    payload = {
        "locked": False,
        "status": status,
        "message": message,
        "frame_w": int(frame_width or 0),
        "frame_h": int(frame_height or 0),
        "detections": int(detections or 0),
        "id": int(locked_id) if locked_id is not None else None,
        "conf": float(conf or 0.0),
        **extra,
    }
    print("STATUS:" + json.dumps(payload, separators=(",", ":")), flush=True)


def run(args: argparse.Namespace) -> None:
    weights = resolve_path(args.weights)
    tracker = resolve_path(args.tracker)
    source = str(args.source)
    if not weights.exists():
        raise FileNotFoundError(f"weights not found: {weights}")
    if not tracker.exists():
        raise FileNotFoundError(f"tracker config not found: {tracker}")

    model = YOLO(str(weights))
    cap = cv2.VideoCapture(source)
    fps = cap.get(cv2.CAP_PROP_FPS) or float(args.loop_hz or 10.0)
    dt = 1.0 / max(float(fps), 1.0)
    min_interval = 1.0 / max(float(args.loop_hz), 1.0)
    status_interval = max(0.5, min_interval)

    lock = TargetLock(reid_sim=args.reid_sim)
    kf = BoxKalman(q=args.q, r=args.r)
    limiter = OutputLimiter(
        ema_alpha=args.smooth_alpha,
        max_center_speed=args.max_center_speed,
        max_size_rate=args.max_size_rate,
        hold_x_px=args.hold_x_px,
        hold_y_px=args.hold_y_px,
        hold_release_px=args.hold_release,
        deadzone_beta=args.deadzone_beta,
    )
    center_hist: deque = deque(maxlen=max(int(args.center_median_window), 1))
    locked_wh = None
    last_emit = 0.0
    last_status_emit = 0.0

    print(json.dumps({
        "event": "ready",
        "source": source,
        "weights": str(weights),
        "tracker": str(tracker),
        "vft_alpha": args.vft_alpha,
        "smooth_alpha": args.smooth_alpha,
        "hold_x_px": args.hold_x_px,
        "hold_y_px": args.hold_y_px,
    }, separators=(",", ":")), flush=True)

    while True:
        frame_started = time.monotonic()
        ok, frame = cap.read()
        if not ok:
            now = time.monotonic()
            if now - last_status_emit >= status_interval:
                emit_status("stream_unavailable", source=source)
                last_status_emit = now
            cap.release()
            time.sleep(0.05)
            cap = cv2.VideoCapture(source)
            continue

        frame_height, frame_width = frame.shape[:2]
        frame_center_x = frame_width * 0.5
        frame_center_y = frame_height * 0.5

        tracker_warning = None
        try:
            result = model.track(
                frame,
                persist=True,
                tracker=str(tracker),
                conf=args.conf,
                iou=args.iou,
                imgsz=args.imgsz,
                device=args.device,
                verbose=False,
            )[0]
        except Exception as exc:
            tracker_warning = str(exc)[:160]
            try:
                result = model.predict(
                    frame,
                    conf=args.conf,
                    iou=args.iou,
                    imgsz=args.imgsz,
                    device=args.device,
                    verbose=False,
                )[0]
            except Exception as predict_exc:
                now = time.monotonic()
                if now - last_status_emit >= status_interval:
                    emit_status("model_error", frame_width=frame_width, frame_height=frame_height,
                                error=str(predict_exc)[:160], tracker_error=tracker_warning)
                    last_status_emit = now
                elapsed = time.monotonic() - frame_started
                if elapsed < min_interval:
                    time.sleep(min_interval - elapsed)
                continue
        if result.boxes is not None and len(result.boxes) > 0:
            xyxy = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            if result.boxes.id is not None:
                ids = result.boxes.id.cpu().numpy().astype(int)
            else:
                ids = np.arange(1, len(xyxy) + 1, dtype=int)
        else:
            ids = np.empty((0,), dtype=int)
            xyxy = np.empty((0, 4))
            confs = np.empty((0,))

        locked_id, locked_box, reacquired, locked_conf = lock.update(frame, ids, xyxy, confs)
        if reacquired:
            kf.reset()
            limiter.reset()
            center_hist.clear()
            locked_wh = None

        kf.predict(dt)
        stable_box = None
        status = lock.status
        raw_cx = raw_cy = None
        if locked_box is not None:
            raw_cx, raw_cy, box_width, box_height = tuned_target_point_from_box(locked_box, args.vft_alpha, center_hist)
            if locked_wh is None:
                locked_wh = np.array([box_width, box_height], dtype=np.float32)
            measurement = np.array([raw_cx, raw_cy, locked_wh[0], locked_wh[1]], dtype=np.float32)
            conf_ok = float(locked_conf) >= args.conf_lock
            size_ok = size_within_tolerance(box_width, box_height, locked_wh, args.size_tol)
            if conf_ok and size_ok and kf.is_measurement_valid(measurement, args.gate_dist, args.gate_scale):
                stable_box = kf.correct(measurement)
                locked_wh = 0.95 * locked_wh + 0.05 * np.array([box_width, box_height], dtype=np.float32)
            elif kf.initialized and kf.miss < args.max_coast:
                stable_box = kf.coast()
                status = "coast"
            else:
                stable_box = kf.correct(measurement)
        elif kf.initialized and kf.miss < args.max_coast:
            stable_box = kf.coast()
            status = "coast"
        else:
            lock.reset()
            kf.reset()
            limiter.reset()
            center_hist.clear()
            locked_wh = None
            status = "lost"

        now = time.monotonic()
        if stable_box is not None and now - last_emit >= min_interval:
            output_box = limiter.apply(stable_box, dt)
            sx, sy, sw, sh = [float(value) for value in output_box]
            payload = {
                "id": int(locked_id) if locked_id is not None else None,
                "status": status,
                "x": sx,
                "y": sy,
                "w": sw,
                "h": sh,
                "raw_x": raw_cx,
                "raw_y": raw_cy,
                "frame_w": frame_width,
                "frame_h": frame_height,
                "dx": sx - frame_center_x,
                "dy": sy - frame_center_y,
                "conf": float(locked_conf),
                "detections": int(len(ids)),
                "locked": True,
                "message": "SWIMMER LOCKED",
                "tracker_warning": tracker_warning,
            }
            print("TARGET:" + json.dumps(payload, separators=(",", ":")), flush=True)
            last_emit = now
            last_status_emit = now
        elif now - last_status_emit >= status_interval:
            emit_status(status, frame_width=frame_width, frame_height=frame_height,
                        detections=int(len(ids)), locked_id=locked_id, conf=float(locked_conf),
                        tracker_error=tracker_warning)
            last_status_emit = now

        elapsed = time.monotonic() - frame_started
        if elapsed < min_interval:
            time.sleep(min_interval - elapsed)

    cap.release()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Swimmer coordinate worker for gimbal tracking")
    p.add_argument("--source", default=DEFAULT_SOURCE)
    p.add_argument("--weights", default=str(DEFAULT_WEIGHTS))
    p.add_argument("--tracker", default=str(DEFAULT_TRACKER))
    p.add_argument("--conf", type=float, default=0.1)
    p.add_argument("--iou", type=float, default=0.5)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--device", type=str, default="0")
    p.add_argument("--loop-hz", type=float, default=10.0)
    p.add_argument("--q", type=float, default=1.0)
    p.add_argument("--r", type=float, default=50.0)
    p.add_argument("--max-coast", type=int, default=45)
    p.add_argument("--reid-sim", type=float, default=0.5)
    p.add_argument("--gate-dist", type=float, default=140.0)
    p.add_argument("--gate-scale", type=float, default=2.2)
    p.add_argument("--smooth-alpha", type=float, default=0.3)
    p.add_argument("--max-center-speed", type=float, default=800.0)
    p.add_argument("--max-size-rate", type=float, default=1.0)
    p.add_argument("--hold-x-px", type=float, default=500.0)
    p.add_argument("--hold-y-px", type=float, default=500.0)
    p.add_argument("--hold-release", type=float, default=300.0)
    p.add_argument("--conf-lock", type=float, default=0.35)
    p.add_argument("--size-tol", type=float, default=0.35)
    p.add_argument("--vft-alpha", type=float, default=0.35)
    p.add_argument("--deadzone-beta", type=float, default=0.15)
    p.add_argument("--center-median-window", type=int, default=11)
    return p.parse_args()


if __name__ == "__main__":
    run(parse_args())
