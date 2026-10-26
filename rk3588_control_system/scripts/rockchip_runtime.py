"""Select Rockchip models and core masks from the running board, not the repo name."""
import re
from pathlib import Path


def detect_soc(device_tree=Path('/proc/device-tree/compatible')):
    try:
        compatible = Path(device_tree).read_bytes().decode('ascii', errors='ignore').lower()
    except OSError:
        return None
    for soc in ('rk3576', 'rk3588', 'rk3568', 'rk3566', 'rk3562'):
        if soc in compatible:
            return soc
    return None


def compatible_rknn_model(model_path, soc=None):
    model_path = Path(model_path)
    soc = soc or detect_soc()
    match = re.search(r'rk\d{4}', model_path.name.lower())
    if soc and match and match.group() != soc:
        candidate = model_path.with_name(re.sub(r'rk\d{4}', soc, model_path.name, flags=re.I))
        if not candidate.is_file():
            raise RuntimeError(f'{soc} requires a matching RKNN model: {candidate}; using the configured CPU fallback')
        return candidate
    return model_path


def npu_core_mask(runtime_class, soc=None):
    # RK3576 and RK3588 have different core counts. New runtimes provide ALL;
    # older runtimes can select the appropriate available pair/triple or AUTO.
    if hasattr(runtime_class, 'NPU_CORE_ALL'):
        return runtime_class.NPU_CORE_ALL
    name = {'rk3576': 'NPU_CORE_0_1', 'rk3588': 'NPU_CORE_0_1_2'}.get(soc or detect_soc())
    return getattr(runtime_class, name, getattr(runtime_class, 'NPU_CORE_AUTO', 0)) if name else getattr(runtime_class, 'NPU_CORE_AUTO', 0)
