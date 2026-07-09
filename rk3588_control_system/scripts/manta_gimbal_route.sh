#!/bin/sh

set -eu

if ip link show eth0 >/dev/null 2>&1; then
  ip link set eth0 up || true
  ip addr show dev eth0 | grep -q '192.168.144.101/32' || ip addr add 192.168.144.101/32 dev eth0 || true
  ip route replace 192.168.144.108/32 dev eth0 src 192.168.144.101 || true
fi
