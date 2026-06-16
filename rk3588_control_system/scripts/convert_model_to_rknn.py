#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from rknn.api import RKNN


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert ONNX hand model to RKNN for RK3588")
    parser.add_argument("--input", default="scripts/models/best.onnx", help="Input ONNX model path")
    parser.add_argument("--output", default="scripts/models/best.rknn", help="Output RKNN model path")
    parser.add_argument("--target", default="rk3588", help="RKNN target platform")
    parser.add_argument("--quantized", action="store_true", help="Enable post-training quantization")
    parser.add_argument("--dataset", default="", help="Calibration dataset txt path for quantization")
    args = parser.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if not input_path.exists():
        raise SystemExit(f"Input model not found: {input_path}")
    if args.quantized and not args.dataset:
        raise SystemExit("--dataset is required when --quantized is used")

    rknn = RKNN(verbose=True)
    ret = rknn.config(target_platform=args.target)
    if ret != 0:
        raise SystemExit(f"rknn.config failed: {ret}")

    ret = rknn.load_onnx(model=str(input_path))
    if ret != 0:
        raise SystemExit(f"rknn.load_onnx failed: {ret}")

    ret = rknn.build(do_quantization=args.quantized, dataset=args.dataset if args.quantized else None)
    if ret != 0:
        raise SystemExit(f"rknn.build failed: {ret}")

    ret = rknn.export_rknn(str(output_path))
    if ret != 0:
        raise SystemExit(f"rknn.export_rknn failed: {ret}")

    rknn.release()
    print(f"RKNN written to: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
