#!/bin/bash

################################################################################
# Pixhawk Link Quick Check
# Prints serial candidates, configured serial_port, bridge logs and API status.
################################################################################

set -u

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_DIR"

echo "=== Pixhawk Link Quick Check ==="
echo "Project: $PROJECT_DIR"
echo "Time: $(date '+%F %T')"
echo ""

echo "[1] Configured serial_port"
SERIAL_PORT=$(grep -o '"serial_port"[^,]*' config/system.config.json | sed -E 's/.*"([^\"]+)".*/\1/' || true)
echo "serial_port: ${SERIAL_PORT:-<not set>}"
if [ -n "${SERIAL_PORT:-}" ] && [ -e "$SERIAL_PORT" ]; then
  echo "status: OK ($SERIAL_PORT exists)"
else
  echo "status: MISSING device node"
fi
echo ""

echo "[2] Detected serial candidates"
ls /dev/ttyUSB* /dev/ttyACM* /dev/ttyS* /dev/ttyAMA* 2>/dev/null | grep -v ttyFIQ0 || echo "(none)"
echo ""

echo "[3] Recent MAVLink bridge log"
if [ -f logs/mavlink.log ]; then
  tail -n 30 logs/mavlink.log
else
  echo "logs/mavlink.log not found"
fi
echo ""

echo "[4] Backend health and status"
if command -v curl >/dev/null 2>&1; then
  echo "health:" 
  curl -s http://127.0.0.1:3000/health || echo "(health endpoint not reachable)"
  echo ""
  echo "status summary:"
  python3 - <<'PY'
import json, urllib.request
try:
    raw = urllib.request.urlopen('http://127.0.0.1:3000/api/status', timeout=2).read()
    d = json.loads(raw.decode('utf-8'))
    data = d.get('data', {})
    print('pixhawkStatus=', data.get('pixhawkStatus'))
    print('isConnected=', data.get('isConnected'))
    print('flightMode=', data.get('telemetry', {}).get('flightMode'))
    print('armed=', data.get('telemetry', {}).get('armed'))
    print('roverControl=', data.get('roverControl'))
except Exception as e:
    print('status endpoint not reachable:', e)
PY
else
  echo "curl not installed"
fi

echo ""
echo "Done."
