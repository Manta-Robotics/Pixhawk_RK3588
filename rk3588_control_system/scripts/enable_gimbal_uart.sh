#!/bin/bash

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "[gimbal-uart] Please run with sudo: sudo bash scripts/enable_gimbal_uart.sh" >&2
    exit 1
fi

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG_FILE="$PROJECT_DIR/config/system.config.json"

eval "$(python3 - "$CONFIG_FILE" <<'PY'
import json
import shlex
import sys

cfg = json.load(open(sys.argv[1], 'r', encoding='utf-8'))
gimbal = cfg.get('gimbal', {})
for key, value in {
    'CONTROL_TRANSPORT': gimbal.get('control_transport', 'uart'),
    'BOOT_ENV': gimbal.get('boot_config', '/boot/firmware/ubuntuEnv.txt'),
    'UART_OVERLAY': gimbal.get('uart_overlay', 'rk3588-lubancat-uart3-m0-overlay'),
}.items():
    print(f"{key}={shlex.quote(str(value))}")
PY
)"

if [[ "$CONTROL_TRANSPORT" != "uart" ]]; then
    echo "[gimbal-uart] control_transport=$CONTROL_TRANSPORT; UART overlay is intentionally skipped."
    exit 0
fi

if [[ ! -f "$BOOT_ENV" ]]; then
    echo "[gimbal-uart] Missing $BOOT_ENV" >&2
    exit 1
fi

if [[ -z "$UART_OVERLAY" ]]; then
    echo "[gimbal-uart] gimbal.uart_overlay is empty in $CONFIG_FILE" >&2
    exit 1
fi

overlay_found=0
for candidate in \
    "/boot/firmware/dtbs/rockchip/overlay/${UART_OVERLAY}.dtbo" \
    "/boot/dtbs/rockchip/overlay/${UART_OVERLAY}.dtbo" \
    "/lib/firmware/${UART_OVERLAY}.dtbo"; do
    [[ -f "$candidate" ]] && overlay_found=1 && break
done
if [[ $overlay_found -ne 1 ]]; then
    echo "[gimbal-uart] Overlay file not found: ${UART_OVERLAY}.dtbo" >&2
    exit 1
fi

if grep -E '^overlays=' "$BOOT_ENV" | grep -Eq "(^| )${UART_OVERLAY}( |$)"; then
    echo "[gimbal-uart] UART3 overlay already enabled: $UART_OVERLAY"
    exit 0
fi

cp "$BOOT_ENV" "${BOOT_ENV}.bak.$(date +%Y%m%d-%H%M%S)"
python3 - "$BOOT_ENV" "$UART_OVERLAY" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
overlay = sys.argv[2]
lines = path.read_text(encoding='utf-8').splitlines()
for index, line in enumerate(lines):
    if line.startswith('overlays='):
        current = line[len('overlays='):].strip().split()
        if overlay not in current:
            current.append(overlay)
        lines[index] = 'overlays=' + ' '.join(current) + ' '
        break
else:
    lines.append('overlays=' + overlay + ' ')
path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
PY

echo "[gimbal-uart] Enabled UART3 overlay: $UART_OVERLAY"
echo "[gimbal-uart] Reboot is required before /dev/ttyS3 appears."
