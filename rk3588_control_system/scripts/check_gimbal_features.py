#!/usr/bin/env python3
"""Read-only prerequisites for the four gimbal modes; never sends motion/arming commands."""
import argparse
import importlib.util
import json
import os
import sys
import time
import urllib.request
from pathlib import Path
from rockchip_runtime import detect_soc
from validate_board_profile import validate_overlays

ROOT = Path(__file__).resolve().parents[1]


def get_json(url):
    with urllib.request.urlopen(url, timeout=4) as response:
        return json.load(response)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=os.environ.get('MANTA_CONFIG_FILE') or os.environ.get('MANTA_CONFIG_PATH') or str(ROOT / 'config/system.config.json'))
    parser.add_argument('--url', default=None)
    parser.add_argument('--expect-soc', choices=('rk3576', 'rk3588'), default='rk3576')
    parser.add_argument('--load-models', action='store_true', help='also run the CPU YOLO models on a blank frame without controlling hardware')
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text(encoding='utf-8'))
    checks = []

    def check(name, ok, detail):
        checks.append({'name': name, 'ok': bool(ok), 'detail': detail})
        return bool(ok)

    soc = detect_soc()
    check('board', soc == args.expect_soc, f'detected={soc}, expected={args.expect_soc}')
    overlays = validate_overlays(config, soc)
    check('board_profile', not overlays, overlays)
    check('flight_serial', Path(config.get('serial_port', '')).exists(), config.get('serial_port'))
    gimbal = config.get('gimbal', {})
    if gimbal.get('control_transport') == 'udp':
        interface = os.environ.get('MANTA_GIMBAL_INTERFACE') or gimbal.get('network_interface', 'br-manta')
        check('camera_network_interface', Path('/sys/class/net', interface).exists(), interface)
    imports = {module: importlib.util.find_spec(module) is not None for module in ('torch', 'ultralytics', 'cv2', 'numpy')}
    dependencies = check('tracking_dependencies', all(imports.values()), {'python': sys.executable, 'imports': imports})
    face = ROOT / gimbal.get('face', {}).get('fallback_model', 'scripts/models/yolov8n-face-lindevs.pt')
    swimmer = ROOT / gimbal.get('swimmer', {}).get('weights', 'scripts/best.pt')
    face_ok = check('face_weights', face.is_file(), str(face))
    swimmer_ok = check('swimmer_weights', swimmer.is_file(), str(swimmer))
    if args.load_models:
        models_ok = False
        if dependencies and face_ok and swimmer_ok:
            try:
                import numpy as np
                from ultralytics import YOLO
                for weights in (face, swimmer):
                    YOLO(str(weights)).predict(np.zeros((384, 384, 3), dtype=np.uint8), imgsz=384, device='cpu', verbose=False)
                models_ok = True
            except Exception as error:
                check('model_load_error', False, str(error))
        dependencies = check('model_inference', models_ok, 'CPU blank-frame inference; no hardware commands') and dependencies

    url = (args.url or f"http://127.0.0.1:{config.get('web_port', 3000)}").rstrip('/')
    status, control, video_ok = {}, {}, False
    try:
        status = get_json(url + '/api/status').get('data', {})
        control = get_json(url + '/api/gimbal/state').get('state', {})
        check('backend_and_flight_link', status.get('isConnected') is True, 'live flight controller connection')
    except Exception as error:
        check('backend_and_flight_link', False, str(error))
    feedback = control.get('feedback') or {}
    age = time.time() * 1000 - float(feedback.get('updatedAt') or 0)
    link = check('gimbal_control_feedback', control.get('connected') is True and feedback.get('checksumValid') is True and 0 <= age <= 500,
                 {'linkStatus': control.get('linkStatus'), 'error': control.get('lastError'), 'ageMs': age})
    try:
        with urllib.request.urlopen(url + '/api/gimbal/stream', timeout=4) as response:
            content = response.headers.get('Content-Type', '')
            payload = response.read(2048)
            video_ok = response.status == 200 and 'multipart/' in content and b'\xff\xd8' in payload
        check('live_video', video_ok, content)
    except Exception as error:
        check('live_video', False, str(error))
    uwb = status.get('telemetry', {}).get('uwb', {})
    updated = float(uwb.get('measurementUpdatedAt') or 0)
    uwb_age = time.time() * 1000 - updated
    trusted = uwb.get('online') is True and uwb.get('fresh') is True and uwb.get('measurementStatus') == 'trusted' and 0 <= uwb_age <= 700
    check('trusted_uwb', trusted, {'status': uwb.get('measurementStatus'), 'reason': uwb.get('rejectionReason'), 'ageMs': uwb_age})
    features = {'click': link and video_ok, 'face': link and video_ok and dependencies and face_ok,
                'swimmer': link and video_ok and dependencies and swimmer_ok, 'beacon': link and trusted}
    print(json.dumps({'soc': soc, 'checks': checks, 'featuresReady': features,
                      'scope': 'prerequisites only; physical centering and tracking still require live acceptance'}, ensure_ascii=False, indent=2))
    return 0 if all(item['ok'] for item in checks) and all(features.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
