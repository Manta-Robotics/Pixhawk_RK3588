#!/usr/bin/env bash

set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJECT_DIR"

OFFLINE=0
CHECK_ONLY=0
SKIP_BOOT_CONFIG=0

usage() {
    echo "Usage: sudo bash quickstart.sh [--offline] [--check-only] [--skip-boot-config]"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --offline) OFFLINE=1 ;;
        --check-only) CHECK_ONLY=1 ;;
        --skip-boot-config) SKIP_BOOT_CONFIG=1 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "[quickstart] unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

if [[ $CHECK_ONLY -eq 1 ]]; then
    echo "[quickstart] validating repository"
    npm run maintenance
    python3 scripts/manta_doctor.py
    exit 0
fi

if [[ ${EUID:-$(id -u)} -eq 0 ]]; then
    SUDO=()
elif command -v sudo >/dev/null 2>&1; then
    SUDO=(sudo)
else
    echo "[quickstart] root or sudo is required" >&2
    exit 1
fi

echo "[quickstart] installing runtime dependencies"
install_args=()
[[ $OFFLINE -eq 1 ]] && install_args+=(--offline)
"${SUDO[@]}" bash scripts/install.sh "${install_args[@]}"

echo "[quickstart] validating installed repository"
npm run maintenance

if ! command -v mediamtx >/dev/null 2>&1; then
    mediamtx_args=()
    if [[ $OFFLINE -eq 1 ]]; then
        archive="${MANTA_MEDIAMTX_ARCHIVE:-$PROJECT_DIR/vendor/mediamtx_v1.19.2_linux_arm64.tar.gz}"
        [[ -f "$archive" ]] || { echo "[quickstart] offline MediaMTX archive missing: $archive" >&2; exit 1; }
        mediamtx_args+=("$archive")
    fi
    "${SUDO[@]}" bash scripts/install_mediamtx.sh "${mediamtx_args[@]}"
fi

echo "[quickstart] installing systemd services"
"${SUDO[@]}" env MANTA_RUN_USER="${MANTA_RUN_USER:-root}" MANTA_SKIP_BOOT_CONFIG="$SKIP_BOOT_CONFIG" bash scripts/install_boot_services.sh

"${SUDO[@]}" bash scripts/python_service.sh scripts/manta_doctor.py --installed

echo "[quickstart] installation complete"
echo "[quickstart] services are enabled but were not started or restarted"
echo "[quickstart] review overlay changes before manually rebooting the board"
echo "[quickstart] after reboot, open http://10.42.0.1:3000"
