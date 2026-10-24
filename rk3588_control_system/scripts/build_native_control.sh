#!/usr/bin/env bash
set -Eeuo pipefail
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${MANTA_NATIVE_BUILD_DIR:-$PROJECT_DIR/control/build}"
PYTHON_BIN="${MANTA_BUILD_PYTHON:-$PROJECT_DIR/.venv/bin/python}"
if [[ ! -x "$PYTHON_BIN" ]]; then PYTHON_BIN="$(command -v python3)"; fi
"$PYTHON_BIN" -c 'import pymavlink' || { echo 'Install requirements-test.txt or requirements.txt before building.' >&2; exit 1; }
cmake -S "$PROJECT_DIR/control" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="$PYTHON_BIN"
cmake --build "$BUILD_DIR" --parallel "${MANTA_BUILD_JOBS:-2}"
cd "$PROJECT_DIR"
"$BUILD_DIR/manta-control" --config "${MANTA_CONFIG_FILE:-${MANTA_CONFIG_PATH:-$PROJECT_DIR/config/system.config.json}}" --check-config
# Atomic replacement preserves the running executable until a deliberate
# service restart. Building/installing never arms, starts or restarts anything.
install -d "$PROJECT_DIR/control/bin"
install -m 0755 "$BUILD_DIR/manta-control" "$PROJECT_DIR/control/bin/manta-control.new"
mv -f "$PROJECT_DIR/control/bin/manta-control.new" "$PROJECT_DIR/control/bin/manta-control"
echo '[native] Installed control/bin/manta-control; no service was started.'
