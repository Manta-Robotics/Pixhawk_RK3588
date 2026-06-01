#!/bin/bash

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "[install-boot] Please run with sudo: sudo bash scripts/install_boot_services.sh" >&2
    exit 1
fi

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG_FILE="$PROJECT_DIR/config/system.config.json"
RUN_USER="${SUDO_USER:-cat}"

eval "$(python3 - "$CONFIG_FILE" <<'PY'
import json
import shlex
import sys

cfg = json.load(open(sys.argv[1], 'r', encoding='utf-8'))
hotspot = cfg.get('hotspot', {})
values = {
    'HOTSPOT_SSID': hotspot.get('ssid', 'Manta-Control'),
    'HOTSPOT_PASSWORD': hotspot.get('password', 'manta8888'),
    'HOTSPOT_PORTAL_IP': hotspot.get('portal_ip', '10.42.0.1')
}
for key, value in values.items():
    print(f"{key}={shlex.quote(str(value))}")
PY
)"

install -d /etc/systemd/system
install -d /etc/NetworkManager/dnsmasq-shared.d
install -d "$PROJECT_DIR/logs"
chown -R "$RUN_USER":"$RUN_USER" "$PROJECT_DIR/logs"

bash "$PROJECT_DIR/scripts/enable_camera_overlay.sh"

for template in manta-backend manta-bridge manta-camera manta-hotspot manta-captive-portal; do
    sed \
        -e "s|__PROJECT_DIR__|$PROJECT_DIR|g" \
        -e "s|__RUN_USER__|$RUN_USER|g" \
        "$PROJECT_DIR/systemd/${template}.service.template" > "/etc/systemd/system/${template}.service"
done

cat > /etc/NetworkManager/dnsmasq-shared.d/manta-captive-portal.conf <<EOF
address=/#/$HOTSPOT_PORTAL_IP
EOF

systemctl daemon-reload
systemctl enable manta-backend.service manta-bridge.service manta-camera.service manta-hotspot.service manta-captive-portal.service

echo "[install-boot] Installed boot services and captive portal settings."
echo "[install-boot] Hotspot SSID    : $HOTSPOT_SSID"
if [[ ${#HOTSPOT_PASSWORD} -lt 8 ]]; then
    echo "[install-boot] Hotspot security: OPEN (requested password '$HOTSPOT_PASSWORD' is shorter than WPA minimum 8 characters)"
else
    echo "[install-boot] Hotspot password: $HOTSPOT_PASSWORD"
fi
echo "[install-boot] Dashboard URL   : http://$HOTSPOT_PORTAL_IP:3000"
echo "[install-boot] Reboot the LubanCat to apply the configured camera overlay and hotspot changes."