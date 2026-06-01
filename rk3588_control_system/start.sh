#!/bin/bash

################################################################################
# RK3588 Pixhawk Control System - Start Script
# Launches all services (Node.js backend + Python MAVLink bridge)
################################################################################

echo "╔════════════════════════════════════════════════════════════╗"
echo "║  RK3588 Pixhawk Control System - Starting Services         ║"
echo "╚════════════════════════════════════════════════════════════╝"

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJECT_DIR"

read_interface_ipv4() {
    local iface="$1"
    ip -4 -o addr show dev "$iface" 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -n 1
}

NODE_PID=""
PYTHON_PID=""
CAMERA_PID=""
TAIL_PID=""

# Prefer user-local Node 20 if installed
if [ -d "/home/cat/.local/node20/bin" ]; then
    export PATH="/home/cat/.local/node20/bin:$PATH"
fi

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

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
    echo -e "${RED}❌ Detected active systemd-managed services for this project:${NC}"
    printf '  - %s\n' "${active_services[@]}"
    echo ""
    echo -e "${YELLOW}This script would start a second backend/bridge/camera stack and cause port conflicts.${NC}"
    echo -e "${YELLOW}Use systemd instead:${NC}"
    echo "  sudo systemctl restart manta-backend.service manta-bridge.service manta-camera.service"
    echo "  sudo systemctl status manta-backend.service manta-bridge.service manta-camera.service"
    echo ""
    echo -e "${YELLOW}If you really want manual control, stop the systemd services first.${NC}"
    exit 1
}

echo ""
echo -e "${YELLOW}📁 Working directory: $PROJECT_DIR${NC}"

check_systemd_conflict

# Check if Node.js is installed
if ! command -v node &> /dev/null; then
    echo -e "${RED}❌ Node.js not found. Please run: bash scripts/install.sh${NC}"
    exit 1
fi

# Check if Python3 is installed
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}❌ Python3 not found. Please run: bash scripts/install.sh${NC}"
    exit 1
fi

# Create logs directory if it doesn't exist
mkdir -p logs
echo -e "${GREEN}✅ Logs directory ready${NC}"

SERIAL_PORT=$(grep -o '"serial_port"[^,]*' config/system.config.json | sed -E 's/.*"([^"]+)".*/\1/')
if [ -n "$SERIAL_PORT" ] && [ ! -e "$SERIAL_PORT" ]; then
    echo -e "${YELLOW}⚠️  Config serial_port=$SERIAL_PORT does not exist on this system${NC}"
    echo -e "${YELLOW}⚠️  If using 40-pin UART, enable the correct UART overlay in /boot/firmware/ubuntuEnv.txt and reboot${NC}"
fi

# Kill any existing processes on the web port
WEB_PORT=$(grep -o '"web_port"[^,]*' config/system.config.json | grep -o '[0-9]*')
WEB_PORT=${WEB_PORT:-3000}
BRIDGE_COMMAND_PORT=$(grep -o '"bridge_command_port"[^,]*' config/system.config.json | grep -o '[0-9]*' | head -n 1)
BRIDGE_COMMAND_PORT=${BRIDGE_COMMAND_PORT:-14551}
CAMERA_PORT=$(grep -o '"camera_port"[^,]*' config/system.config.json | grep -o '[0-9]*' | head -n 1)
CAMERA_PORT=${CAMERA_PORT:-8090}

echo ""
echo -e "${YELLOW}🔄 Checking for existing processes on port $WEB_PORT...${NC}"
EXISTING_PIDS=$(lsof -t -iTCP:$WEB_PORT -sTCP:LISTEN 2>/dev/null | tr '\n' ' ')
if [ -n "$EXISTING_PIDS" ]; then
    echo -e "${YELLOW}⚠️  Found existing process(es): $EXISTING_PIDS${NC}"
    kill $EXISTING_PIDS 2>/dev/null || true
    sleep 1
    for pid in $EXISTING_PIDS; do
        if ps -p "$pid" > /dev/null 2>&1; then
            echo -e "${YELLOW}⚠️  Force killing PID $pid${NC}"
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
fi

echo -e "${YELLOW}🔄 Checking for stale bridge processes on UDP port $BRIDGE_COMMAND_PORT...${NC}"
BRIDGE_PIDS=$(lsof -t -iUDP:$BRIDGE_COMMAND_PORT 2>/dev/null | tr '\n' ' ')
if [ -n "$BRIDGE_PIDS" ]; then
    echo -e "${YELLOW}⚠️  Found bridge-related process(es): $BRIDGE_PIDS${NC}"
    kill $BRIDGE_PIDS 2>/dev/null || true
    sleep 1
    for pid in $BRIDGE_PIDS; do
        if ps -p "$pid" > /dev/null 2>&1; then
            echo -e "${YELLOW}⚠️  Force killing stale bridge PID $pid${NC}"
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
fi

echo -e "${YELLOW}🔄 Checking for existing camera snapshot service on port $CAMERA_PORT...${NC}"
CAMERA_PIDS=$(lsof -t -iTCP:$CAMERA_PORT -sTCP:LISTEN 2>/dev/null | tr '\n' ' ')
if [ -n "$CAMERA_PIDS" ]; then
    echo -e "${YELLOW}⚠️  Found camera-related process(es): $CAMERA_PIDS${NC}"
    kill $CAMERA_PIDS 2>/dev/null || true
    sleep 1
    for pid in $CAMERA_PIDS; do
        if ps -p "$pid" > /dev/null 2>&1; then
            echo -e "${YELLOW}⚠️  Force killing stale camera PID $pid${NC}"
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
fi

pkill -f "python3 backend/mavlink_bridge.py" 2>/dev/null || true
pkill -f "python3 scripts/camera_snapshot_server.py" 2>/dev/null || true

echo ""
echo -e "${GREEN}🚀 Starting Node.js backend server...${NC}"
node backend/server.js > logs/server.log 2>&1 &
NODE_PID=$!
echo "$NODE_PID" > .pids_node
echo -e "${GREEN}✅ Node.js server started (PID: $NODE_PID)${NC}"
sleep 2

# Check if Node.js started successfully
if ps -p $NODE_PID > /dev/null; then
    echo -e "${GREEN}✅ Node.js is running${NC}"
else
    echo -e "${RED}❌ Node.js failed to start. Check logs/server.log${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}🚀 Starting Python MAVLink bridge...${NC}"
python3 backend/mavlink_bridge.py > logs/mavlink.log 2>&1 &
PYTHON_PID=$!
echo "$PYTHON_PID" > .pids_python
echo -e "${GREEN}✅ Python bridge started (PID: $PYTHON_PID)${NC}"
sleep 2

# Check if Python started successfully
if ps -p $PYTHON_PID > /dev/null; then
    echo -e "${GREEN}✅ Python MAVLink bridge is running${NC}"
else
    echo -e "${YELLOW}⚠️  Python bridge may not have started. Check logs/mavlink.log${NC}"
fi

echo ""
echo -e "${GREEN}🚀 Starting local camera snapshot service...${NC}"
python3 scripts/camera_snapshot_server.py > logs/camera.log 2>&1 &
CAMERA_PID=$!
echo "$CAMERA_PID" > .pids_camera
echo -e "${GREEN}✅ Camera snapshot service started (PID: $CAMERA_PID)${NC}"
sleep 2

if ps -p $CAMERA_PID > /dev/null; then
    echo -e "${GREEN}✅ Camera snapshot service is running${NC}"
else
    echo -e "${YELLOW}⚠️  Camera snapshot service may not have started. Check logs/camera.log${NC}"
fi

echo ""
echo "════════════════════════════════════════════════════════════"
echo -e "${GREEN}✅ All services started successfully!${NC}"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "📊 Service Status:"
echo "  Node.js Server: http://0.0.0.0:$WEB_PORT"
echo "  Dashboard: http://localhost:$WEB_PORT"
HOTSPOT_IP=$(python3 - <<'PY'
import json
cfg = json.load(open('config/system.config.json', 'r', encoding='utf-8'))
print(str(cfg.get('hotspot', {}).get('portal_ip', '10.42.0.1')))
PY
)
ETHERNET_IP=$(read_interface_ipv4 eth0)
if [ -n "$HOTSPOT_IP" ]; then
    echo "  Wireless Dashboard: http://$HOTSPOT_IP:$WEB_PORT"
fi
if [ -n "$ETHERNET_IP" ]; then
    echo "  Ethernet Dashboard: http://$ETHERNET_IP:$WEB_PORT"
fi
echo "  PID: $NODE_PID"
echo ""
echo "  Python MAVLink Bridge: PID $PYTHON_PID"
echo "  Camera Snapshot: PID $CAMERA_PID (http://127.0.0.1:$CAMERA_PORT/snapshot.jpg)"
echo ""
echo "📋 Logs:"
echo "  Server:  tail -f logs/server.log"
echo "  MAVLink: tail -f logs/mavlink.log"
echo "  Camera:  tail -f logs/camera.log"
echo "  System:  tail -f logs/system.log"
echo ""
echo "🛑 To stop services:"
echo "  kill $NODE_PID  # Stop Node.js"
echo "  kill $PYTHON_PID  # Stop Python"
echo "  kill $CAMERA_PID  # Stop camera snapshot service"
echo "  Or run: bash stop.sh"
echo ""
echo "💡 Monitor logs:"
echo "  tail -f logs/server.log"
echo "  tail -f logs/mavlink.log"
echo "  tail -f logs/camera.log"
echo "  bash scripts/status_report.sh"
echo "  watch -n 2 bash scripts/status_report.sh"
echo ""

# Save PIDs to file for later reference
echo "$NODE_PID" > .pids_node
echo "$PYTHON_PID" > .pids_python
echo "$CAMERA_PID" > .pids_camera

echo "📌 Basic status report:"
bash scripts/status_report.sh || true
echo ""

cleanup() {
    echo ""
    echo -e "${YELLOW}🛑 Ctrl+C received, stopping services...${NC}"

    if [ -n "$TAIL_PID" ] && ps -p "$TAIL_PID" > /dev/null 2>&1; then
        kill "$TAIL_PID" 2>/dev/null || true
        wait "$TAIL_PID" 2>/dev/null || true
    fi

    if [ -n "$PYTHON_PID" ] && ps -p "$PYTHON_PID" > /dev/null 2>&1; then
        kill -SIGTERM "$PYTHON_PID" 2>/dev/null || true
        sleep 1
        if ps -p "$PYTHON_PID" > /dev/null 2>&1; then
            kill -9 "$PYTHON_PID" 2>/dev/null || true
        fi
    fi

    if [ -n "$CAMERA_PID" ] && ps -p "$CAMERA_PID" > /dev/null 2>&1; then
        kill -SIGTERM "$CAMERA_PID" 2>/dev/null || true
        sleep 1
        if ps -p "$CAMERA_PID" > /dev/null 2>&1; then
            kill -9 "$CAMERA_PID" 2>/dev/null || true
        fi
    fi

    if [ -n "$NODE_PID" ] && ps -p "$NODE_PID" > /dev/null 2>&1; then
        kill -SIGTERM "$NODE_PID" 2>/dev/null || true
        sleep 1
        if ps -p "$NODE_PID" > /dev/null 2>&1; then
            kill -9 "$NODE_PID" 2>/dev/null || true
        fi
    fi

    rm -f .pids_node .pids_python .pids_camera
    exit 0
}

trap cleanup INT TERM

echo -e "${GREEN}✅ Services are running. Press Ctrl+C to stop.${NC}"
echo ""
echo "📡 Live logs:"
tail -n 20 -F logs/server.log logs/mavlink.log logs/camera.log &
TAIL_PID=$!

while true; do
    if ! ps -p "$NODE_PID" > /dev/null 2>&1; then
        echo ""
        echo -e "${RED}❌ Node.js server exited unexpectedly${NC}"
        cleanup
    fi

    if ! ps -p "$PYTHON_PID" > /dev/null 2>&1; then
        echo ""
        echo -e "${RED}❌ Python MAVLink bridge exited unexpectedly${NC}"
        cleanup
    fi

    if ! ps -p "$CAMERA_PID" > /dev/null 2>&1; then
        echo ""
        echo -e "${RED}❌ Camera snapshot service exited unexpectedly${NC}"
        cleanup
    fi

    sleep 1
done
