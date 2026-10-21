#!/bin/sh
set -eu
project_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
config="$project_root/config/system.config.json"
interface="${MANTA_GIMBAL_INTERFACE:-$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["gimbal"].get("network_interface", "br-manta"))' "$config")}"
local_ip="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["gimbal"].get("network_local_ip", "192.168.144.101"))' "$config")"
host="$(python3 -c 'import json,sys,ipaddress; print(ipaddress.IPv4Address(json.load(open(sys.argv[1]))["gimbal"]["udp_host"]))' "$config")"
if ip link show "$interface" >/dev/null 2>&1; then
  ip link set "$interface" up
  ip -4 addr show dev "$interface" | grep -q " $local_ip/32" || ip addr add "$local_ip/32" dev "$interface"
  ip route replace "$host/32" dev "$interface" src "$local_ip"
  mac="$(python3 -c 'import json,sys,re; m=json.load(open(sys.argv[1]))["gimbal"].get("network_mac", ""); assert not m or re.fullmatch(r"[0-9a-fA-F]{2}(:[0-9a-fA-F]{2}){5}",m); print(m)' "$config")"
  if [ -n "$mac" ]; then
    ip neigh replace "$host" lladdr "$mac" nud permanent dev "$interface"
  fi
fi
