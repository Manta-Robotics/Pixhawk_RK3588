#!/usr/bin/env bash

set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJECT_DIR"

SERVICES=(
    manta-gimbal-route.service
    manta-gimbal-stream.service
    manta-camera.service
    manta-bridge.service
    manta-backend.service
    manta-hotspot.service
    manta-captive-portal.service
    manta-bluetooth-pan.service
)

if [[ -d /opt/node20/bin ]]; then
    export PATH="/opt/node20/bin:$PATH"
fi

if [[ ${EUID:-$(id -u)} -eq 0 ]]; then
    SUDO=()
elif command -v sudo >/dev/null 2>&1; then
    SUDO=(sudo)
else
    echo "[start] root or sudo is required" >&2
    exit 1
fi

node scripts/validate_config.mjs
"${SUDO[@]}" systemctl start "${SERVICES[@]}"

echo "[start] Manta services started"
bash scripts/status_report.sh
