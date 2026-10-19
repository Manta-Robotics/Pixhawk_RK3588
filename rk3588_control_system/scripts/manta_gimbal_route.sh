#!/bin/sh

set -eu

interface="${MANTA_GIMBAL_INTERFACE:-br-manta}"

if ip link show "$interface" >/dev/null 2>&1; then
  ip link set "$interface" up || true
  ip addr show dev "$interface" | grep -q '192.168.144.101/32' || ip addr add 192.168.144.101/32 dev "$interface" || true
  ip route replace 192.168.144.108/32 dev "$interface" src 192.168.144.101 || true
fi
