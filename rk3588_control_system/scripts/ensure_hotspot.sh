#!/bin/bash

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG_FILE="${MANTA_CONFIG_FILE:-$PROJECT_DIR/config/system.config.json}"
ACTION="${1:-start}"
HOTSPOT_WAIT_SECONDS="${HOTSPOT_WAIT_SECONDS:-20}"

case "$ACTION" in
    start|stop|ensure) ;;
    watch)
        trap 'exit 0' TERM INT
        while true; do
            if ! bash "$0" ensure; then
                echo "[hotspot] Recovery failed; retrying in ${HOTSPOT_CHECK_INTERVAL_SECONDS:-15}s." >&2
            fi
            sleep "${HOTSPOT_CHECK_INTERVAL_SECONDS:-15}" &
            wait $! || true
        done
        ;;
    *)
        echo "Usage: $0 {start|stop|ensure|watch}" >&2
        exit 2
        ;;
esac

eval "$(python3 - "$CONFIG_FILE" <<'PY'
import ipaddress
import json
import shlex
import sys

cfg = json.load(open(sys.argv[1], 'r', encoding='utf-8'))
hotspot = cfg.get('hotspot', {})
values = {
    'HOTSPOT_ENABLED': '1' if hotspot.get('enabled', True) else '0',
    'HOTSPOT_CONNECTION_NAME': hotspot.get('connection_name', 'Manta-Control-Hotspot'),
    'HOTSPOT_SSID': hotspot.get('ssid', 'Manta-Control'),
    'HOTSPOT_SECURITY': str(hotspot.get('security', 'wpa2')).lower(),
    'HOTSPOT_PASSWORD': hotspot.get('password', '__GENERATED_PER_DEVICE__'),
    'HOTSPOT_INTERFACE': hotspot.get('interface', 'p2p0'),
    'HOTSPOT_FALLBACK_INTERFACE': hotspot.get('fallback_interface', 'wlan0'),
    'HOTSPOT_BAND': hotspot.get('band', 'bg'),
    'HOTSPOT_CHANNEL': hotspot.get('channel', 6),
    'HOTSPOT_PORTAL_IP': str(ipaddress.IPv4Address(hotspot.get('portal_ip', '10.42.0.1'))),
    'PREFERRED_WIRELESS': cfg.get('wireless_interface', 'p2p0')
}

for key, value in values.items():
    print(f"{key}={shlex.quote(str(value))}")
PY
)"

HOTSPOT_PASSWORD="${MANTA_HOTSPOT_PASSWORD:-$HOTSPOT_PASSWORD}"

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

    # Jetson uses predictable names such as wlP1p1s0 instead of wlan0.
    nmcli -t -f DEVICE,TYPE device status | awk -F: '$2 == "wifi" {print $1; exit}'
}

hotspot_is_active() {
    [[ "$(nmcli -g GENERAL.CONNECTION device show "$HOTSPOT_IFACE")" == "$HOTSPOT_CONNECTION_NAME" ]] &&
        nmcli -g IP4.ADDRESS device show "$HOTSPOT_IFACE" | grep -Fxq "$HOTSPOT_PORTAL_IP/24"
}

if ! command -v nmcli >/dev/null 2>&1; then
    echo "[hotspot] NetworkManager (nmcli) is required." >&2
    exit 1
fi

if [[ "$ACTION" == "stop" ]]; then
    nmcli connection down "$HOTSPOT_CONNECTION_NAME" >/dev/null 2>&1 || true
    echo "[hotspot] Hotspot stopped: $HOTSPOT_CONNECTION_NAME"
    exit 0
fi

nmcli radio wifi on >/dev/null 2>&1 || true

# An ensure check must not disconnect clients on a healthy hotspot.
HOTSPOT_IFACE="$(pick_interface || true)"
if [[ "$ACTION" == "ensure" && -n "$HOTSPOT_IFACE" ]] && hotspot_is_active; then
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

if [[ "$HOTSPOT_SECURITY" != "open" && "$HOTSPOT_SECURITY" != "none" ]] && \
   [[ "$HOTSPOT_PASSWORD" == "__GENERATED_PER_DEVICE__" || ${#HOTSPOT_PASSWORD} -lt 8 ]]; then
    echo "[hotspot] A per-device WPA password is required in /etc/manta/manta.env." >&2
    exit 1
fi

if [[ "$HOTSPOT_SECURITY" != "open" && "$HOTSPOT_SECURITY" != "none" && \
      "$HOTSPOT_SECURITY" != "wpa2" && "$HOTSPOT_SECURITY" != "wpa-psk" ]]; then
    echo "[hotspot] Unsupported security mode: $HOTSPOT_SECURITY" >&2
    exit 1
fi

nmcli connection modify "$HOTSPOT_CONNECTION_NAME" \
    connection.interface-name "$HOTSPOT_IFACE" \
    connection.autoconnect yes \
    connection.autoconnect-priority 100 \
    connection.autoconnect-retries 0 \
    connection.permissions "" \
    802-11-wireless.mode ap \
    802-11-wireless.ssid "$HOTSPOT_SSID" \
    802-11-wireless.band "$HOTSPOT_BAND" \
    802-11-wireless.channel "$HOTSPOT_CHANNEL" \
    802-11-wireless.powersave 2 \
    ipv4.method shared \
    ipv4.addresses "$HOTSPOT_PORTAL_IP/24" \
    ipv4.never-default yes \
    ipv6.method disabled >/dev/null

if [[ "$HOTSPOT_SECURITY" == "open" || "$HOTSPOT_SECURITY" == "none" ]]; then
    nmcli connection modify "$HOTSPOT_CONNECTION_NAME" remove 802-11-wireless-security >/dev/null 2>&1 || true
else
    nmcli connection modify "$HOTSPOT_CONNECTION_NAME" \
        wifi-sec.key-mgmt wpa-psk \
        wifi-sec.proto rsn \
        wifi-sec.pairwise ccmp \
        wifi-sec.group ccmp \
        wifi-sec.pmf 1 \
        wifi-sec.psk "$HOTSPOT_PASSWORD" >/dev/null
fi

nmcli connection up "$HOTSPOT_CONNECTION_NAME" ifname "$HOTSPOT_IFACE" >/dev/null

echo "[hotspot] Hotspot started on $HOTSPOT_IFACE"
echo "[hotspot] SSID: $HOTSPOT_SSID"
echo "[hotspot] Security: $HOTSPOT_SECURITY"
echo "[hotspot] Connection name: $HOTSPOT_CONNECTION_NAME"
