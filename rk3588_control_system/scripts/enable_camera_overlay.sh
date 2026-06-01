#!/bin/bash

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG_FILE="$PROJECT_DIR/config/system.config.json"

eval "$(python3 - "$CONFIG_FILE" <<'PY'
import json
import shlex
import sys

cfg = json.load(open(sys.argv[1], 'r', encoding='utf-8'))
camera = cfg.get('camera', {})
values = {
	'BOOT_CONFIG': camera.get('boot_config', '/boot/firmware/ubuntuEnv.txt'),
	'CAMERA_OVERLAY': camera.get('overlay', ''),
	'CAMERA_SENSOR': camera.get('sensor', 'camera'),
	'CAMERA_PORT_NAME': camera.get('port', '')
}

for key, value in values.items():
	print(f"{key}={shlex.quote(str(value))}")
PY
)"

if [[ ! -f "$BOOT_CONFIG" ]]; then
	echo "[enable-camera] Boot config not found: $BOOT_CONFIG" >&2
	exit 1
fi

if [[ -z "$CAMERA_OVERLAY" ]]; then
	echo "[enable-camera] camera.overlay is empty in $CONFIG_FILE" >&2
	exit 1
fi

OVERLAY_DTBO=""
for candidate in \
	"/boot/firmware/dtbs/rockchip/overlay/${CAMERA_OVERLAY}.dtbo" \
	"/boot/dtbs/rockchip/overlay/${CAMERA_OVERLAY}.dtbo" \
	"/lib/firmware/${CAMERA_OVERLAY}.dtbo"; do
	if [[ -f "$candidate" ]]; then
		OVERLAY_DTBO="$candidate"
		break
	fi
done

if [[ -z "$OVERLAY_DTBO" ]]; then
	port_hint=""
	if [[ -n "$CAMERA_PORT_NAME" ]]; then
		port_hint=" on ${CAMERA_PORT_NAME}"
	fi
	echo "[enable-camera] Overlay file not found for ${CAMERA_SENSOR}${port_hint}: ${CAMERA_OVERLAY}.dtbo" >&2
	echo "[enable-camera] Expected it under /boot/firmware/dtbs/rockchip/overlay, /boot/dtbs/rockchip/overlay, or /lib/firmware." >&2
	echo "[enable-camera] The current kernel image does not appear to ship this sensor profile yet. Install a matching dtbo first, then rerun this script." >&2
	exit 1
fi

backup_path="${BOOT_CONFIG}.bak_$(date +%Y%m%d_%H%M%S)"
cp "$BOOT_CONFIG" "$backup_path"

python3 - "$BOOT_CONFIG" "$CAMERA_OVERLAY" <<'PY'
import pathlib
import sys

config_path = pathlib.Path(sys.argv[1])
overlay = sys.argv[2].strip()
lines = config_path.read_text(encoding='utf-8').splitlines()

updated = []
overlays_line_found = False

for line in lines:
	stripped = line.strip()
	bare = stripped[1:].strip() if stripped.startswith('#') else stripped

	if stripped.startswith('overlays='):
		overlays_line_found = True
		raw_items = stripped.split('=', 1)[1].split()
		kept_items = []
		for item in raw_items:
			if item.startswith('rk3588-lubancat-5-cam') and item.endswith('-overlay'):
				continue
			kept_items.append(item)

		if overlay not in kept_items:
			kept_items.append(overlay)

		updated.append('overlays=' + ' '.join(kept_items))
		continue

	if bare.startswith('rk3588-lubancat-5-cam') and bare.endswith('-overlay'):
		updated.append(f'# {bare}')
		continue

	updated.append(line)

if not overlays_line_found:
	raise SystemExit('[enable-camera] overlays= line not found in boot config')

config_path.write_text('\n'.join(updated) + '\n', encoding='utf-8')
print(f'[enable-camera] Enabled overlay in overlays=: {overlay}')
PY

echo "[enable-camera] Using overlay file: $OVERLAY_DTBO"
echo "[enable-camera] Updated $BOOT_CONFIG"
echo "[enable-camera] Backup created at $backup_path"
echo "[enable-camera] Reboot is required before ${CAMERA_SENSOR} becomes available."
