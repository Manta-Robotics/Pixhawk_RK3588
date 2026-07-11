#!/usr/bin/env bash

set -Eeuo pipefail

IFACE="${GIMBAL_IFACE:-eth0}"
HOST="${1:-192.168.144.108}"
LOCAL_CIDR="${GIMBAL_LOCAL_CIDR:-192.168.144.101/32}"
REQUIRE_4K="${GIMBAL_REQUIRE_4K:-0}"

if [[ ${EUID:-$(id -u)} -eq 0 ]]; then
    ip link set "$IFACE" up 2>/dev/null || true
    ip addr show dev "$IFACE" | grep -q "${LOCAL_CIDR%/*}" || ip addr add "$LOCAL_CIDR" dev "$IFACE"
fi

if [[ -r "/sys/class/net/$IFACE/carrier" ]] && [[ $(cat "/sys/class/net/$IFACE/carrier") != "1" ]]; then
    echo "[probe-rtsp] $IFACE has no physical carrier" >&2
    exit 2
fi

URLS=(
    "rtsp://$HOST/live/0"
    "rtsp://$HOST/live/1"
    "rtsp://$HOST/live/2"
    "rtsp://$HOST:554/live"
    "rtsp://$HOST:554/stream1"
    "rtsp://$HOST:554/main"
    "rtsp://$HOST:554/Streaming/Channels/101"
    "rtsp://$HOST:8554/main.264"
)

BEST_URL=""
BEST_WIDTH=0
BEST_HEIGHT=0

echo "[probe-rtsp] interface=$IFACE host=$HOST"
for url in "${URLS[@]}"; do
    result="$(timeout 8 ffprobe -rtsp_transport tcp -v error -select_streams v:0 \
        -show_entries stream=codec_name,width,height,r_frame_rate \
        -of csv=p=0 "$url" 2>/dev/null || true)"
    if [[ -z "$result" ]]; then
        continue
    fi

    IFS=',' read -r codec width height fps <<< "$result"
    echo "[probe-rtsp] $url -> ${codec} ${width}x${height} ${fps}"
    if (( width * height > BEST_WIDTH * BEST_HEIGHT )); then
        BEST_URL="$url"
        BEST_WIDTH="$width"
        BEST_HEIGHT="$height"
    fi
done

if [[ -z "$BEST_URL" ]]; then
    echo "[probe-rtsp] no RTSP stream found" >&2
    exit 1
fi

echo "[probe-rtsp] best=$BEST_URL ${BEST_WIDTH}x${BEST_HEIGHT}"
if [[ "$REQUIRE_4K" == "1" ]] && (( BEST_WIDTH < 3840 || BEST_HEIGHT < 2160 )); then
    echo "[probe-rtsp] a real 4K source was required but not found" >&2
    exit 3
fi
