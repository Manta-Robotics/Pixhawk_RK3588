#!/usr/bin/env bash

set -Eeuo pipefail

CORE_SERVICES=(
    manta-backend.service
    manta-bridge.service
    manta-camera.service
    manta-gimbal-stream.service
    manta-gimbal-route.service
)
CONNECTIVITY_SERVICES=(
    manta-captive-portal.service
    manta-hotspot.service
    manta-bluetooth-pan.service
)

if [[ ${EUID:-$(id -u)} -eq 0 ]]; then
    SUDO=()
elif command -v sudo >/dev/null 2>&1; then
    SUDO=(sudo)
else
    echo "[stop] root or sudo is required" >&2
    exit 1
fi

"${SUDO[@]}" systemctl stop "${CORE_SERVICES[@]}"

if [[ ${1:-} == "--all" ]]; then
    echo "[stop] stopping Wi-Fi and Bluetooth maintenance links"
    "${SUDO[@]}" systemctl stop "${CONNECTIVITY_SERVICES[@]}"
else
    echo "[stop] control services stopped; wireless maintenance links remain active"
    echo "[stop] use 'bash stop.sh --all' to stop every Manta service"
fi
