#!/usr/bin/env bash

set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
OFFLINE=0

for argument in "$@"; do
    case "$argument" in
        --offline) OFFLINE=1 ;;
        *) echo "[install] Unknown argument: $argument" >&2; exit 2 ;;
    esac
done

if [[ $EUID -ne 0 ]]; then
    echo "[install] Run as root: sudo bash scripts/install.sh [--offline]" >&2
    exit 1
fi

if ! grep -qi "Ubuntu" /etc/os-release; then
    echo "[install] Ubuntu is required." >&2
    exit 1
fi

if [[ "$(uname -m)" != "aarch64" ]]; then
    echo "[install] Warning: target hardware is aarch64; detected $(uname -m)." >&2
fi

cd "$PROJECT_DIR"
RUN_USER="${MANTA_RUN_USER:-${SUDO_USER:-root}}"

SYSTEM_PACKAGES=(
    build-essential ca-certificates curl git openssl
    python3 python3-dev python3-pip python3-setuptools python3-venv python3-numpy python3-opencv python3-dbus python3-gi
    libssl-dev libffi-dev libopenblas-dev libjpeg-dev libopenjp2-7
    bluez bluez-tools rfkill network-manager dnsmasq iptables iw
    minicom picocom screen usbutils lsof v4l-utils ffmpeg
)

if [[ $OFFLINE -eq 0 ]]; then
    echo "[install] Installing Ubuntu packages"
    apt-get update
    DEBIAN_FRONTEND=noninteractive apt-get install -y "${SYSTEM_PACKAGES[@]}"
else
    echo "[install] Offline mode: skipping apt repositories"
fi

node_compatible=0
if command -v node >/dev/null 2>&1; then
    if node -e 'const [major, minor] = process.versions.node.split(".").map(Number); process.exit((major === 20 && minor >= 19) || (major === 22 && minor >= 12) || major > 22 ? 0 : 1)'; then
        node_compatible=1
    fi
fi
if [[ $node_compatible -ne 1 ]]; then
    if [[ $OFFLINE -eq 1 ]]; then
        echo "[install] Node.js 20.19+ or 22.12+ is required in offline mode." >&2
        exit 1
    fi
    echo "[install] Installing Node.js 20 LTS"
    curl -fsSL https://deb.nodesource.com/setup_20.x | bash -
    DEBIAN_FRONTEND=noninteractive apt-get install -y nodejs
fi

echo "[install] Installing deterministic Node dependencies"
npm ci
npm run build

echo "[install] Creating project Python environment"
if [[ ! -x "$PROJECT_DIR/.venv/bin/python" ]]; then
    python3 -m venv --system-site-packages "$PROJECT_DIR/.venv"
fi

PIP_ARGS=(install --upgrade-strategy only-if-needed -r "$PROJECT_DIR/requirements.txt")
if [[ $OFFLINE -eq 1 ]]; then
    WHEELHOUSE="${MANTA_WHEELHOUSE:-$PROJECT_DIR/wheelhouse}"
    if [[ ! -d "$WHEELHOUSE" ]]; then
        echo "[install] Offline wheelhouse not found: $WHEELHOUSE" >&2
        exit 1
    fi
    PIP_ARGS+=(--no-index --find-links "$WHEELHOUSE")
else
    "$PROJECT_DIR/.venv/bin/python" -m pip install --upgrade pip setuptools wheel
fi
"$PROJECT_DIR/.venv/bin/python" -m pip "${PIP_ARGS[@]}"

install -d -o "$RUN_USER" -g "$RUN_USER" \
    "$PROJECT_DIR/logs" \
    "$PROJECT_DIR/data" \
    "$PROJECT_DIR/recordings/gimbal" \
    "$PROJECT_DIR/frontend/assets/map"

if getent group dialout >/dev/null; then
    usermod -a -G dialout "$RUN_USER"
fi
if getent group gpio >/dev/null; then
    usermod -a -G gpio "$RUN_USER"
fi
if getent group video >/dev/null; then
    usermod -a -G video "$RUN_USER"
fi

systemctl enable bluetooth.service NetworkManager.service

echo "[install] Runtime dependencies installed; services were not started."
echo "[install] Pixhawk default: /dev/ttyS1 @ 115200"
echo "[install] Gimbal default : /dev/ttyS3 @ 115200"
