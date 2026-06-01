#!/bin/bash

################################################################################
# Setup Bluetooth on RK3588
# Configures bluez and enables Bluetooth services
################################################################################

echo "Setting up Bluetooth..."

# Update system
sudo apt-get update

# Install Bluetooth packages
sudo apt-get install -y bluez bluez-tools rfkill

# Enable Bluetooth service
sudo systemctl enable bluetooth
sudo systemctl restart bluetooth

# Check Bluetooth status
echo ""
echo "Bluetooth Status:"
sudo systemctl status bluetooth --no-pager

# Disable Bluetooth power saving (optional, for better stability)
echo ""
echo "Configuring Bluetooth power settings..."

# Create/edit Bluetooth configuration
sudo tee /etc/bluetooth/main.conf > /dev/null <<'EOF'
[General]
Name = RK3588_Pixhawk
Class = 0x000000
DiscoverableTimeout = 0
PairableTimeout = 0
FastConnectable = false
Discoverable = false
Pairable = true
LinkLossTimeout = 60
JustWorksRepairing = never
MaxControllers = 0
MultiProfile = yes
ControllerMode = bredr
AutoEnable = true
Privacy = device
JustWorksRepairing = never
EOF

# Restart Bluetooth service
sudo systemctl restart bluetooth

# Show how to pair devices
echo ""
echo "✅ Bluetooth configured"
echo ""
echo "To pair a device:"
echo "  $ bluetoothctl"
echo "  > power on"
echo "  > discoverable on"
echo "  > scan on"
echo "  > pair <MAC_ADDRESS>"
echo "  > connect <MAC_ADDRESS>"
echo "  > trust <MAC_ADDRESS>"
echo ""
