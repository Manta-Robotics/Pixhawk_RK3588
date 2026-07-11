#!/usr/bin/env bash

set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJECT_DIR"

if [[ ${EUID:-$(id -u)} -eq 0 ]]; then
    SUDO=()
elif command -v sudo >/dev/null 2>&1; then
    SUDO=(sudo)
else
    echo "[quickstart] root or sudo is required" >&2
    exit 1
fi

echo "[quickstart] validating repository"
npm run check
npm test

echo "[quickstart] installing runtime dependencies"
"${SUDO[@]}" bash scripts/install.sh

echo "[quickstart] installing systemd services"
"${SUDO[@]}" env MANTA_RUN_USER=root bash scripts/install_boot_services.sh

echo "[quickstart] installation complete"
echo "[quickstart] review overlay changes before manually rebooting the board"
echo "[quickstart] after reboot, open http://10.42.0.1:3000"
