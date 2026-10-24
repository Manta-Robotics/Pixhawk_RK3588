#!/usr/bin/env bash
set -Eeuo pipefail
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_DIR"
CONTROL_BIN="$PROJECT_DIR/control/bin/manta-control"
if [[ ! -x "$CONTROL_BIN" ]]; then
    echo '[native] Missing executable; run bash scripts/build_native_control.sh' >&2
    exit 1
fi
exec "$CONTROL_BIN" --config "${MANTA_CONFIG_FILE:-${MANTA_CONFIG_PATH:-$PROJECT_DIR/config/system.config.json}}" "$@"
