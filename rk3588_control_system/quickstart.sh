#!/bin/bash

################################################################################
# RK3588 Pixhawk Control System - Quick Start
# One-command installation and launch
################################################################################

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJECT_DIR"

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     RK3588 Pixhawk Control System - Quick Start            ║"
echo "╚════════════════════════════════════════════════════════════╝"

# Check if already initialized
if [ ! -d "node_modules" ]; then
    echo ""
    echo "🔧 First run detected. Installing dependencies..."
    echo "   This may take several minutes..."
    echo ""
    
    bash scripts/install.sh
    
    if [ $? -ne 0 ]; then
        echo ""
        echo "❌ Installation failed. Please check the errors above."
        exit 1
    fi
fi

echo ""
echo "📝 Checking configuration..."

# Check if serial port is configured
SERIAL_PORT=$(grep -o '"serial_port"[^,]*' config/system.config.json | grep -o '"/dev/[^"]*"' | tr -d '"')

if [ -z "$SERIAL_PORT" ] || [ "$SERIAL_PORT" = "/dev/ttyUSB0" ]; then
    echo ""
    echo "⚠️  Serial port not configured or using default"
    echo ""
    echo "Available serial ports:"
    ls -la /dev/tty* 2>/dev/null | grep -E "(USB|ACM)" || echo "No USB serial devices found"
    echo ""
    echo "📝 Please edit config/system.config.json and set the correct serial_port"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""
echo "🚀 Starting system..."
bash start.sh
