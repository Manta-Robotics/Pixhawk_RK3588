#!/usr/bin/env bash

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
RELATIVE_SCRIPT="${1:-}"

if [[ -z "$RELATIVE_SCRIPT" || "$RELATIVE_SCRIPT" == /* || "$RELATIVE_SCRIPT" == *".."* ]]; then
    echo "[python-service] A project-relative Python script is required." >&2
    exit 2
fi

SCRIPT_PATH="$PROJECT_DIR/$RELATIVE_SCRIPT"
if [[ ! -f "$SCRIPT_PATH" ]]; then
    echo "[python-service] Script not found: $SCRIPT_PATH" >&2
    exit 2
fi

shift

PYTHON_BIN="$PROJECT_DIR/.venv/bin/python"
if [[ ! -x "$PYTHON_BIN" ]]; then
    PYTHON_BIN="/usr/bin/python3"
fi

cd "$PROJECT_DIR"
exec "$PYTHON_BIN" "$SCRIPT_PATH" "$@"
