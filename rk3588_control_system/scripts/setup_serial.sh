#!/bin/bash

################################################################################
# Setup Serial Port Permissions
# Grants access to serial ports without requiring sudo
################################################################################

echo "Setting up serial port permissions..."

# Add current user to dialout and gpio groups
sudo usermod -a -G dialout $USER
if getent group gpio > /dev/null; then
	sudo usermod -a -G gpio $USER
fi

# Set udev rules for USB serial devices
sudo tee /etc/udev/rules.d/99-usb-serial.rules > /dev/null <<EOF
# USB Serial Device Permissions
SUBSYSTEMS=="usb", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666"
# Generic USB to Serial
SUBSYSTEMS=="usb", SUBSYSTEM=="usb_device", MODE="0666"
KERNEL=="ttyUSB[0-9]*", MODE="0666"
KERNEL=="ttyACM[0-9]*", MODE="0666"
KERNEL=="ttyS[0-9]*", MODE="0666"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

if [ ! -e /dev/ttyS3 ] && [ -f /boot/firmware/ubuntuEnv.txt ]; then
	echo "ℹ️  /dev/ttyS3 is not present. Enabling UART3 overlay for the gimbal port..."
	sudo bash "$(cd "$(dirname "$0")" && pwd)/enable_gimbal_uart.sh"
fi

echo "✅ Serial port permissions configured"
echo "ℹ️  If /dev/ttyS3 was missing, reboot before testing gimbal control"
