#!/bin/bash

set -euo pipefail

IFACE="${GIMBAL_IFACE:-eth0}"
HOST="${1:-192.168.144.108}"
LOCAL_CIDR="${GIMBAL_LOCAL_CIDR:-192.168.144.101/24}"

if [[ ${EUID:-$(id -u)} -eq 0 ]]; then
    ip addr show dev "$IFACE" | grep -q "${LOCAL_CIDR%/*}" || ip addr add "$LOCAL_CIDR" dev "$IFACE" 2>/dev/null || true
    ip link set "$IFACE" up 2>/dev/null || true
fi

URLS=(
    "rtsp://$HOST:554/live"
    "rtsp://$HOST:554/stream1"
    "rtsp://$HOST:554/h264"
    "rtsp://$HOST:554/main"
    "rtsp://$HOST:554/main.264"
    "rtsp://$HOST:554/Streaming/Channels/101"
    "rtsp://$HOST:8554/live"
    "rtsp://$HOST:8554/main.264"
)

echo "[probe-rtsp] iface=$IFACE host=$HOST local=$LOCAL_CIDR"
ip -br addr show dev "$IFACE" || true

for url in "${URLS[@]}"; do
    echo "--- $url"
    if timeout 8 ffprobe -rtsp_transport tcp -v error -select_streams v:0 -show_entries stream=codec_name,width,height -of default=noprint_wrappers=1 "$url"; then
        echo "[probe-rtsp] OK $url"
        exit 0
    fi
done

echo "[probe-rtsp] No RTSP stream found for $HOST" >&2
exit 1
