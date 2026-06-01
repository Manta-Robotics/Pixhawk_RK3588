#!/bin/bash

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG_FILE="$PROJECT_DIR/config/system.config.json"
ACTION="${1:-start}"
HOTSPOT_WAIT_SECONDS="${HOTSPOT_WAIT_SECONDS:-20}"

eval "$(python3 - "$CONFIG_FILE" <<'PY'
import json
import shlex
import sys

cfg = json.load(open(sys.argv[1], 'r', encoding='utf-8'))
hotspot = cfg.get('hotspot', {})
values = {
    'HOTSPOT_ENABLED': '1' if hotspot.get('enabled', True) else '0',
    'HOTSPOT_CONNECTION_NAME': hotspot.get('connection_name', 'Manta-Control-Hotspot'),
    'HOTSPOT_SSID': hotspot.get('ssid', 'Manta-Control'),
    'HOTSPOT_PASSWORD': hotspot.get('password', 'manta8888'),
    'HOTSPOT_INTERFACE': hotspot.get('interface', 'p2p0'),
    'HOTSPOT_FALLBACK_INTERFACE': hotspot.get('fallback_interface', 'wlan0'),
    'HOTSPOT_BAND': hotspot.get('band', 'bg'),
    'HOTSPOT_CHANNEL': hotspot.get('channel', 6),
    'PREFERRED_WIRELESS': cfg.get('wireless_interface', 'p2p0')
}

for key, value in values.items():
    print(f"{key}={shlex.quote(str(value))}")
PY
)"

if [[ "$HOTSPOT_ENABLED" != "1" ]]; then
    echo "[hotspot] Disabled in config, nothing to do."
    exit 0
fi

pick_interface() {
    local seen=""
    local candidate

    for candidate in "$HOTSPOT_INTERFACE" "$HOTSPOT_FALLBACK_INTERFACE" "$PREFERRED_WIRELESS" p2p0 wlan0; do
        if [[ -z "$candidate" ]]; then
            continue
        fi
        if [[ " $seen " == *" $candidate "* ]]; then
            continue
        fi
        seen="$seen $candidate"
        if nmcli -t -f DEVICE,TYPE device status | grep -Eq "^${candidate}:wifi(:|$)"; then
            echo "$candidate"
            return 0
        fi
    done

    return 1
}

nmcli radio wifi on >/dev/null 2>&1 || true

if [[ "$ACTION" == "stop" ]]; then
    nmcli connection down "$HOTSPOT_CONNECTION_NAME" >/dev/null 2>&1 || true
    echo "[hotspot] Hotspot stopped: $HOTSPOT_CONNECTION_NAME"
    exit 0
fi

HOTSPOT_IFACE=""
for (( attempt = 1; attempt <= HOTSPOT_WAIT_SECONDS; attempt++ )); do
    HOTSPOT_IFACE="$(pick_interface || true)"
    if [[ -n "$HOTSPOT_IFACE" ]]; then
        break
    fi

    if [[ $attempt -eq 1 ]]; then
        echo "[hotspot] Waiting for a Wi-Fi interface managed by NetworkManager..." >&2
    fi
    sleep 1
done

if [[ -z "$HOTSPOT_IFACE" ]]; then
    echo "[hotspot] No Wi-Fi interface managed by NetworkManager was found after ${HOTSPOT_WAIT_SECONDS}s." >&2
    nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status >&2 || true
    exit 1
fi

if ! nmcli -t -f NAME connection show | grep -Fxq "$HOTSPOT_CONNECTION_NAME"; then
    nmcli connection add type wifi ifname "$HOTSPOT_IFACE" con-name "$HOTSPOT_CONNECTION_NAME" ssid "$HOTSPOT_SSID" autoconnect yes >/dev/null
fi

if [[ ${#HOTSPOT_PASSWORD} -lt 8 ]]; then
    nmcli connection modify "$HOTSPOT_CONNECTION_NAME" \
        connection.interface-name "$HOTSPOT_IFACE" \
        connection.autoconnect yes \
        connection.autoconnect-priority 100 \
        802-11-wireless.mode ap \
        802-11-wireless.ssid "$HOTSPOT_SSID" \
        802-11-wireless.band "$HOTSPOT_BAND" \
        802-11-wireless.channel "$HOTSPOT_CHANNEL" \
        802-11-wireless-security.key-mgmt "" \
        ipv4.method shared \
        ipv6.method disabled >/dev/null

    nmcli device disconnect "$HOTSPOT_IFACE" >/dev/null 2>&1 || true
    nmcli connection up "$HOTSPOT_CONNECTION_NAME" ifname "$HOTSPOT_IFACE" >/dev/null

    echo "[hotspot] Hotspot started on $HOTSPOT_IFACE"
    echo "[hotspot] SSID: $HOTSPOT_SSID"
    echo "[hotspot] Password 'manta' is shorter than WPA allows, so the hotspot was started as OPEN."
    exit 0
fi

nmcli connection modify "$HOTSPOT_CONNECTION_NAME" \
    connection.interface-name "$HOTSPOT_IFACE" \
    connection.autoconnect yes \
    connection.autoconnect-priority 100 \
    802-11-wireless.mode ap \
    802-11-wireless.ssid "$HOTSPOT_SSID" \
    802-11-wireless.band "$HOTSPOT_BAND" \
    802-11-wireless.channel "$HOTSPOT_CHANNEL" \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "$HOTSPOT_PASSWORD" \
    ipv4.method shared \
    ipv6.method disabled >/dev/null

nmcli device disconnect "$HOTSPOT_IFACE" >/dev/null 2>&1 || true
nmcli connection up "$HOTSPOT_CONNECTION_NAME" ifname "$HOTSPOT_IFACE" >/dev/null

echo "[hotspot] Hotspot started on $HOTSPOT_IFACE"
echo "[hotspot] SSID: $HOTSPOT_SSID"
echo "[hotspot] Connection name: $HOTSPOT_CONNECTION_NAME"
