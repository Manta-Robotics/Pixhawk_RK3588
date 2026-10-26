#!/usr/bin/env python3
"""Export the configured YOLOv8 face model and compile it for RK3588 NPU."""
from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert YOLOv8 face weights to an RK3588 FP16 RKNN model")
    parser.add_argument("--weights", default="scripts/models/yolov8n-face-lindevs.pt")
    parser.add_argument("--onnx", default="scripts/models/yolov8n-face-lindevs.onnx")
    parser.add_argument("--output", default="scripts/models/yolov8n-face-lindevs-rk3588-fp16.rknn")
    parser.add_argument("--imgsz", type=int, default=384)
    parser.add_argument("--force-onnx", action="store_true")
    args = parser.parse_args()

    weights = Path(args.weights)
    onnx_path = Path(args.onnx)
    output_path = Path(args.output)
    imgsz = int(max(160, args.imgsz))
    if not weights.exists():
        raise SystemExit(f"Face weights not found: {weights}")

    if args.force_onnx or not onnx_path.exists():
        from ultralytics import YOLO

        exported = Path(YOLO(str(weights)).export(
            format="onnx",
            imgsz=imgsz,
            opset=12,
            simplify=False,
            dynamic=False,
            batch=1,
            device="cpu",
        ))
        if exported.resolve() != onnx_path.resolve():
            onnx_path.parent.mkdir(parents=True, exist_ok=True)
            exported.replace(onnx_path)

    from rknn.api import RKNN

    output_path.parent.mkdir(parents=True, exist_ok=True)
    rknn = RKNN(verbose=False)
    try:
        ret = rknn.config(
            target_platform="rk3588",
            mean_values=[[0, 0, 0]],
            std_values=[[255, 255, 255]],
            optimization_level=3,
        )
        if ret != 0:
            raise SystemExit(f"rknn.config failed: {ret}")
        ret = rknn.load_onnx(model=str(onnx_path))
        if ret != 0:
            raise SystemExit(f"rknn.load_onnx failed: {ret}")
        ret = rknn.build(do_quantization=False)
        if ret != 0:
            raise SystemExit(f"rknn.build failed: {ret}")
        ret = rknn.export_rknn(str(output_path))
        if ret != 0:
            raise SystemExit(f"rknn.export_rknn failed: {ret}")
    finally:
        rknn.release()

    print(f"ONNX: {onnx_path}")
    print(f"RKNN: {output_path}")
    print(f"Input: {imgsz}x{imgsz}, FP16, target rk3588")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
