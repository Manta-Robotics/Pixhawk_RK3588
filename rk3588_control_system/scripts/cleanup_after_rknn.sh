#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
export PROJECT_DIR

python3 - <<'PY'
import os
from pathlib import Path
cfg = (Path(os.environ['PROJECT_DIR']) / 'config' / 'system.config.json').read_text()
print('Current config points to best.rknn:', 'scripts/models/best.rknn' in cfg)
PY

echo 'This removes heavy fallback ML packages after RKNN runtime is verified.'
echo 'If vision is not yet running from best.rknn, stop here.'

python3 -m pip uninstall -y ultralytics ultralytics-thop torchvision torch onnxruntime onnx triton \
  nvidia-cublas nvidia-cudnn-cu13 nvidia-cusparselt-cu13 nvidia-nccl-cu13 nvidia-nvshmem-cu13 \
  nvidia-nvtx nvidia-curand nvidia-cufile nvidia-cuda-runtime nvidia-cuda-nvrtc nvidia-cuda-cupti \
  nvidia-cusparse nvidia-cufft nvidia-cusolver cuda-bindings cuda-pathfinder cuda-toolkit || true

python3 -m pip cache purge || true
df -h /
