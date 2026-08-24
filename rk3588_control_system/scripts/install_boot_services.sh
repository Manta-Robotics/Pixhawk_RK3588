#!/bin/bash

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "[install-boot] Please run with sudo: sudo bash scripts/install_boot_services.sh" >&2
    exit 1
fi

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG_FILE="$PROJECT_DIR/config/system.config.json"
RUN_USER="${MANTA_RUN_USER:-root}"
SKIP_BOOT_CONFIG="${MANTA_SKIP_BOOT_CONFIG:-0}"

if [[ -d /opt/node20/bin ]]; then
    export PATH="/opt/node20/bin:$PATH"
fi

node "$PROJECT_DIR/scripts/validate_config.mjs" "$CONFIG_FILE"

if ! id "$RUN_USER" >/dev/null 2>&1; then
    echo "[install-boot] Unknown run user: $RUN_USER" >&2
    exit 1
fi

if ! command -v mediamtx >/dev/null 2>&1 && [[ ! -x /usr/local/bin/mediamtx ]]; then
    echo "[install-boot] MediaMTX is not installed; run scripts/install_mediamtx.sh first" >&2
    exit 1
fi

eval "$(python3 - "$CONFIG_FILE" <<'PY'
import json
import shlex
import sys

cfg = json.load(open(sys.argv[1], 'r', encoding='utf-8'))
hotspot = cfg.get('hotspot', {})
values = {
    'HOTSPOT_SSID': hotspot.get('ssid', 'Manta-Control'),
    'HOTSPOT_PORTAL_IP': hotspot.get('portal_ip', '10.42.0.1')
}
for key, value in values.items():
    print(f"{key}={shlex.quote(str(value))}")
PY
)"

install -d /etc/systemd/system
install -d -m 0700 /etc/manta
install -d /etc/NetworkManager/dnsmasq-shared.d
install -d "$PROJECT_DIR/logs" "$PROJECT_DIR/data" "$PROJECT_DIR/recordings"
chown -R "$RUN_USER":"$RUN_USER" "$PROJECT_DIR/logs" "$PROJECT_DIR/data" "$PROJECT_DIR/recordings"
python3 "$PROJECT_DIR/scripts/generate_device_env.py" /etc/manta/manta.env

if [[ "$SKIP_BOOT_CONFIG" == "1" ]]; then
    echo "[install-boot] skipping camera and UART boot configuration"
else
    bash "$PROJECT_DIR/scripts/enable_camera_overlay.sh"
    bash "$PROJECT_DIR/scripts/enable_gimbal_uart.sh"
fi

for template in manta-backend manta-bridge manta-camera manta-gimbal-route manta-gimbal-stream manta-mediamtx manta-hotspot manta-captive-portal manta-bluetooth-pan; do
    temporary="/etc/systemd/system/${template}.service.tmp"
    sed \
        -e "s|__PROJECT_DIR__|$PROJECT_DIR|g" \
        -e "s|__RUN_USER__|$RUN_USER|g" \
        "$PROJECT_DIR/systemd/${template}.service.template" > "$temporary"
    install -m 0644 "$temporary" "/etc/systemd/system/${template}.service"
    rm -f "$temporary"
done

cat > /etc/NetworkManager/dnsmasq-shared.d/manta-captive-portal.conf <<EOF
address=/#/$HOTSPOT_PORTAL_IP
EOF

systemctl daemon-reload
systemctl enable manta-backend.service manta-bridge.service manta-camera.service manta-gimbal-route.service manta-gimbal-stream.service manta-mediamtx.service manta-hotspot.service manta-captive-portal.service manta-bluetooth-pan.service

echo "[install-boot] Installed boot services and captive portal settings."
echo "[install-boot] Hotspot SSID    : $HOTSPOT_SSID"
echo "[install-boot] Hotspot security: configured"
echo "[install-boot] Dashboard URL   : http://$HOTSPOT_PORTAL_IP:3000"
echo "[install-boot] Units were enabled only; no MANTA service was started or restarted."
echo "[install-boot] Reboot the LubanCat only after reviewing camera/UART overlay changes."
