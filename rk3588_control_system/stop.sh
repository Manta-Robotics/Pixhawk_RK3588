#!/bin/bash

################################################################################
# RK3588 Pixhawk Control System - Stop Script
# Gracefully stops all running services
################################################################################

echo "╔════════════════════════════════════════════════════════════╗"
echo "║  RK3588 Pixhawk Control System - Stopping Services         ║"
echo "╚════════════════════════════════════════════════════════════╝"

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJECT_DIR"

SYSTEMD_SERVICES=(
    manta-backend.service
    manta-bridge.service
    manta-camera.service
    manta-hotspot.service
    manta-captive-portal.service
)

check_systemd_conflict() {
    local active_services=()
    local service=""

    if ! command -v systemctl > /dev/null 2>&1; then
        return
    fi

    for service in "${SYSTEMD_SERVICES[@]}"; do
        if systemctl is-active --quiet "$service"; then
            active_services+=("$service")
        fi
    done

    if [[ ${#active_services[@]} -eq 0 ]]; then
        return
    fi

    echo ""
    echo "❌ Detected active systemd-managed services for this project:"
    printf '  - %s\n' "${active_services[@]}"
    echo ""
    echo "This script only stops manually launched processes."
    echo "Use systemd instead:"
    echo "  sudo systemctl stop manta-backend.service manta-bridge.service manta-camera.service"
    echo "  sudo systemctl status manta-backend.service manta-bridge.service manta-camera.service"
    exit 1
}

WEB_PORT=$(grep -o '"web_port"[^,]*' config/system.config.json | grep -o '[0-9]*' | head -n 1)
WEB_PORT=${WEB_PORT:-3000}
BRIDGE_COMMAND_PORT=$(grep -o '"bridge_command_port"[^,]*' config/system.config.json | grep -o '[0-9]*' | head -n 1)
BRIDGE_COMMAND_PORT=${BRIDGE_COMMAND_PORT:-14551}
BRIDGE_TELEMETRY_PORT=$(grep -o '"bridge_telemetry_port"[^,]*' config/system.config.json | grep -o '[0-9]*' | head -n 1)
BRIDGE_TELEMETRY_PORT=${BRIDGE_TELEMETRY_PORT:-14552}
CAMERA_PORT=$(grep -o '"camera_port"[^,]*' config/system.config.json | grep -o '[0-9]*' | head -n 1)
CAMERA_PORT=${CAMERA_PORT:-8090}

check_systemd_conflict

stop_pid_file() {
    local file_path="$1"
    local label="$2"

    if [[ ! -f "$file_path" ]]; then
        return
    fi

    local pid
    pid="$(cat "$file_path")"
    if [[ -n "$pid" ]] && ps -p "$pid" > /dev/null 2>&1; then
        echo "🛑 Stopping $label (PID: $pid)..."
        kill -SIGTERM "$pid" 2>/dev/null || true
        sleep 1
        if ps -p "$pid" > /dev/null 2>&1; then
            echo "⚠️  Force killing $label..."
            kill -9 "$pid" 2>/dev/null || true
        fi
    fi

    rm -f "$file_path"
}

clear_listeners() {
    local proto="$1"
    local port="$2"
    local pids=""

    if [[ "$proto" == "tcp" ]]; then
        pids=$(lsof -t -iTCP:"$port" -sTCP:LISTEN 2>/dev/null | tr '\n' ' ')
    else
        pids=$(lsof -t -iUDP:"$port" 2>/dev/null | tr '\n' ' ')
    fi

    if [[ -z "$pids" ]]; then
        return
    fi

    echo "🛑 Clearing ${proto^^} listeners on $port: $pids"
    kill $pids 2>/dev/null || true
    sleep 1
    for pid in $pids; do
        if ps -p "$pid" > /dev/null 2>&1; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
}

stop_pid_file .pids_node "Node.js server"
stop_pid_file .pids_python "Python bridge"
stop_pid_file .pids_camera "camera snapshot service"

echo ""
echo "🔍 Checking for lingering processes..."
pkill -f "node backend/server.js" 2>/dev/null || true
pkill -f "python3 backend/mavlink_bridge.py" 2>/dev/null || true
pkill -f "python3 scripts/camera_snapshot_server.py" 2>/dev/null || true

clear_listeners tcp "$WEB_PORT"
clear_listeners udp "$BRIDGE_COMMAND_PORT"
clear_listeners udp "$BRIDGE_TELEMETRY_PORT"
clear_listeners tcp "$CAMERA_PORT"

echo "✅ All services stopped"
