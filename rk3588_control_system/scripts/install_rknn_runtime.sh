#!/usr/bin/env bash
set -euo pipefail

LIB_URL="https://raw.githubusercontent.com/airockchip/rknn-toolkit2/master/rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so"
TMP_LIB="/tmp/librknnrt.so"

curl -L --fail --retry 3 --retry-delay 2 -o "$TMP_LIB" "$LIB_URL"
ls -lh "$TMP_LIB"
sudo install -m 0644 "$TMP_LIB" /usr/lib/librknnrt.so
ls -lh /usr/lib/librknnrt.so
