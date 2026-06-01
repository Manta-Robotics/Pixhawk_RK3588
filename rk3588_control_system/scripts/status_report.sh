#!/bin/bash

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG_FILE="$PROJECT_DIR/config/system.config.json"

if [ ! -f "$CONFIG_FILE" ]; then
    echo "[status-report] config/system.config.json not found"
    exit 1
fi

WEB_PORT=$(grep -o '"web_port"[^,]*' "$CONFIG_FILE" | grep -o '[0-9]*' | head -n 1)
WEB_PORT=${WEB_PORT:-3000}
STATUS_URL="http://127.0.0.1:${WEB_PORT}/api/status"

if ! command -v curl >/dev/null 2>&1; then
    echo "[status-report] curl not found"
    exit 1
fi

STATUS_JSON=""
for _attempt in 1 2 3 4 5; do
    STATUS_JSON=$(curl -fsS --max-time 2 "$STATUS_URL" 2>/dev/null)
    if [ -n "$STATUS_JSON" ]; then
        break
    fi
    sleep 1
done

if [ -z "$STATUS_JSON" ]; then
    echo "[status-report] service unavailable at $STATUS_URL"
    exit 1
fi

STATUS_JSON="$STATUS_JSON" python3 - <<'PY'
import json
import math
import os

payload = json.loads(os.environ.get("STATUS_JSON", "{}"))
data = payload.get("data", {})
telemetry = data.get("telemetry", {})
battery = telemetry.get("battery", {})
attitude = telemetry.get("attitude", {})
gps = telemetry.get("gps", {})
position = telemetry.get("position", {})
temperature = telemetry.get("temperature", {})
control = data.get("roverControl", {})

def fmt_num(value, digits=1, suffix=""):
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return "--"
    if math.isnan(numeric) or math.isinf(numeric):
        return "--"
    return f"{numeric:.{digits}f}{suffix}"

def fmt_int(value, suffix=""):
    try:
        numeric = int(round(float(value)))
    except (TypeError, ValueError):
        return "--"
    return f"{numeric}{suffix}"

velocity = telemetry.get("velocity", {})
speed = math.sqrt(float(velocity.get("vx", 0) or 0) ** 2 + float(velocity.get("vy", 0) or 0) ** 2)
armed = "ARMED" if telemetry.get("armed") else "SAFE"
connected = "ONLINE" if data.get("isConnected") else "OFFLINE"

print("==== Ground Station Report ====")
print(f"Link        : {connected}")
print(f"Vehicle     : {data.get('vehicleType', 'rover').upper()} / {armed}")
print(f"Flight mode : {telemetry.get('flightMode', '--')}")
print(f"Battery     : {fmt_num(battery.get('voltage'), 1, 'V')} / {fmt_int(battery.get('percentage'), '%')}")
print(f"GPS         : {fmt_int(gps.get('satellites'), ' sat')} / HDOP {fmt_num(gps.get('hdop'), 1)}")
print(f"Position    : {fmt_num(position.get('lat'), 6)}, {fmt_num(position.get('lon'), 6)}")
print(f"Attitude    : roll {fmt_num(attitude.get('roll'), 1, ' deg')} | pitch {fmt_num(attitude.get('pitch'), 1, ' deg')} | yaw {fmt_num(attitude.get('yaw'), 1, ' deg')}")
print(f"Ground speed: {fmt_num(speed, 2, ' m/s')}")
print(f"Drive cmd   : throttle {fmt_int(control.get('throttle'), '%')} | steering {fmt_int(control.get('steering'), ' deg')}")
print(f"PWM         : left {fmt_int(control.get('leftPwm'))} | right {fmt_int(control.get('rightPwm'))}")
print(f"Temp        : board {fmt_num(temperature.get('hostBoard'), 1, ' C')} | fcu {fmt_num(temperature.get('flightController'), 1, ' C')}")
PY