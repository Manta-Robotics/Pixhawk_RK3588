#!/usr/bin/env bash

set -euo pipefail

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
    echo "[install-camera-overlay] Please run with sudo: sudo bash scripts/install_custom_camera_overlay.sh" >&2
    exit 1
fi

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG_FILE="$PROJECT_DIR/config/system.config.json"
OVERLAY_DIR="/boot/firmware/dtbs/rockchip/overlay"

if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "[install-camera-overlay] Config not found: $CONFIG_FILE" >&2
    exit 1
fi

eval "$(python3 - "$CONFIG_FILE" <<'PY'
import json
import shlex
import sys

cfg = json.load(open(sys.argv[1], 'r', encoding='utf-8'))
camera = cfg.get('camera', {})
values = {
    'CAMERA_SENSOR': camera.get('sensor', ''),
    'CAMERA_OVERLAY': camera.get('overlay', ''),
}
for key, value in values.items():
    print(f"{key}={shlex.quote(str(value))}")
PY
)"

if [[ -z "$CAMERA_OVERLAY" ]]; then
    echo "[install-camera-overlay] camera.overlay is empty in $CONFIG_FILE" >&2
    exit 1
fi

SOURCE_FILE="$PROJECT_DIR/overlays/${CAMERA_OVERLAY}.dts"
TARGET_FILE="$OVERLAY_DIR/${CAMERA_OVERLAY}.dtbo"
TMP_FILE="$(mktemp /tmp/${CAMERA_OVERLAY##*/}.XXXXXX.dtbo)"

if [[ ! -f "$SOURCE_FILE" ]]; then
    rm -f "$TMP_FILE"
    if [[ -f "$TARGET_FILE" ]]; then
        echo "[install-camera-overlay] No repo-managed source for ${CAMERA_OVERLAY}."
        echo "[install-camera-overlay] Stock overlay already exists at $TARGET_FILE; no install needed."
        exit 0
    fi
    echo "[install-camera-overlay] Overlay source not found: $SOURCE_FILE" >&2
    exit 1
fi

if [[ ! -d "$OVERLAY_DIR" ]]; then
    echo "[install-camera-overlay] Overlay directory not found: $OVERLAY_DIR" >&2
    exit 1
fi

if ! command -v dtc >/dev/null 2>&1; then
    echo "[install-camera-overlay] dtc is not installed." >&2
    rm -f "$TMP_FILE"
    exit 1
fi

dtc -@ -I dts -O dtb -o "$TMP_FILE" "$SOURCE_FILE"
install -m 0644 "$TMP_FILE" "$TARGET_FILE"
rm -f "$TMP_FILE"

echo "[install-camera-overlay] Installed $TARGET_FILE"
echo "[install-camera-overlay] Next step: bash scripts/enable_camera_overlay.sh"
echo "[install-camera-overlay] Then reboot the LubanCat."
