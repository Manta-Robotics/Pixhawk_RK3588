#!/bin/bash

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "[gimbal-uart] Please run with sudo: sudo bash scripts/enable_gimbal_uart.sh" >&2
    exit 1
fi

BOOT_ENV="/boot/firmware/ubuntuEnv.txt"
UART_OVERLAY="rk3588-lubancat-uart3-m0-overlay"

if [[ ! -f "$BOOT_ENV" ]]; then
    echo "[gimbal-uart] Missing $BOOT_ENV" >&2
    exit 1
fi

if grep -Eq "(^| )${UART_OVERLAY}( |$)" "$BOOT_ENV"; then
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
