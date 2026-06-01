#!/bin/bash

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_DIR"
mkdir -p logs

if [ -d "$HOME/.local/node20/bin" ]; then
    export PATH="$HOME/.local/node20/bin:$PATH"
fi

exec node backend/server.js
