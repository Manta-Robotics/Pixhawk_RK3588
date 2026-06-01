#!/bin/bash

################################################################################
# RK3588 Pixhawk Control System - Ubuntu Installation Script
# Run this script first to set up all dependencies
################################################################################

echo "╔════════════════════════════════════════════════════════════╗"
echo "║  RK3588 Pixhawk Control System - Ubuntu Setup              ║"
echo "╚════════════════════════════════════════════════════════════╝"

set -e  # Exit on error

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_DIR"

# Prefer user-local Node 20 if installed
if [ -d "/home/cat/.local/node20/bin" ]; then
    export PATH="/home/cat/.local/node20/bin:$PATH"
fi

# Check if running on Ubuntu
if ! grep -q "Ubuntu" /etc/os-release; then
    echo "⚠️  This script is designed for Ubuntu. Please run on Ubuntu 22.04 LTS"
    exit 1
fi

echo ""
echo "📦 Step 1: Update system packages"
sudo apt-get update

echo ""
echo "📦 Step 2: Install Node.js and npm (LTS version)"
if ! command -v node &> /dev/null; then
    curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
    sudo apt-get install -y nodejs
else
    echo "✅ Node.js already installed: $(node --version)"
fi

echo ""
echo "📦 Step 3: Install Python3 and pip"
sudo apt-get install -y python3 python3-pip python3-dev
python3 --version

echo ""
echo "📦 Step 4: Install system dependencies for serial communication"
sudo apt-get install -y \
    build-essential \
    libssl-dev \
    libffi-dev \
    python3-setuptools \
    libopenblas-dev \
    libatlas-base-dev \
    libjpeg-dev \
    libtiff5 \
    libopenjp2-7

echo ""
echo "📦 Step 5: Install Bluetooth tools"
sudo apt-get install -y \
    bluez \
    bluez-tools \
    rfkill

echo ""
echo "📦 Step 6: Install serial port tools for debugging"
sudo apt-get install -y \
    minicom \
    screen \
    picocom \
    usbutils \
    lsof

echo ""
echo "📦 Step 7: Set up serial port permissions"
sudo usermod -a -G dialout $USER
if getent group gpio > /dev/null; then
    sudo usermod -a -G gpio $USER
else
    echo "ℹ️  Group 'gpio' not found on this system, skipping"
fi
echo "✅ Serial port groups added (logout and login required for effect)"

echo ""
echo "📦 Step 8: Install Node.js dependencies"
npm install

echo ""
echo "📦 Step 9: Install Python dependencies"
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt

echo ""
echo "📦 Step 10: Create necessary directories"
mkdir -p logs
mkdir -p frontend/assets/map
mkdir -p data

echo ""
echo "📦 Step 11: Set up Bluetooth service"
sudo systemctl enable bluetooth
sudo systemctl restart bluetooth

echo ""
echo "════════════════════════════════════════════════════════════"
echo "✅ Installation Complete!"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "📝 Next steps:"
echo "  1. Edit config/system.config.json - Set correct serial port"
echo "  2. Run: bash start.sh"
echo "  3. Open browser: http://localhost:3000"
echo ""
echo "⚙️ Configuration:"
echo "  Serial Port: /dev/ttyUSB0 (check with: ls /dev/tty*)"
echo "  Baud Rate: 57600"
echo "  Web Port: 3000"
echo ""
echo "🔧 To find your serial port:"
echo "  $ dmesg | grep -i tty"
echo "  $ ls -la /dev/tty* | grep USB"
echo ""
echo "💡 To test serial connection:"
echo "  $ minicom -D /dev/ttyUSB0 -b 57600"
echo ""
echo "📱 Bluetooth pairing:"
echo "  $ bluetoothctl"
echo "  > power on"
echo "  > scan on"
echo "  > pair <MAC_ADDRESS>"
echo "  > connect <MAC_ADDRESS>"
echo ""
echo "🚀 System is ready for launch!"
