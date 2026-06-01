#!/bin/bash

set -euo pipefail

export normal_no_read_back=0

wait_for_path() {
    local path="$1"
    local attempts="${2:-30}"
    local delay="${3:-1}"
    local count=0

    while [[ ! -e "$path" ]]; do
        count=$((count + 1))
        if [[ "$count" -ge "$attempts" ]]; then
            echo "[rkaiq] Timeout waiting for $path" >&2
            exit 1
        fi
        sleep "$delay"
    done
}

find_video_node_by_name() {
    local expected="$1"
    local entry

    for entry in /sys/class/video4linux/video*; do
        [[ -e "$entry/name" ]] || continue
        if [[ "$(cat "$entry/name")" == "$expected" ]]; then
            basename "$entry"
            return 0
        fi
    done

    return 1
}

wait_for_path /dev/media1 40 1

for _ in $(seq 1 40); do
    if find_video_node_by_name rkisp_mainpath >/dev/null 2>&1; then
        break
    fi
    sleep 1
done

for _ in $(seq 1 40); do
    if find_video_node_by_name rkisp-input-params >/dev/null 2>&1; then
        break
    fi
    sleep 1
done

exec /usr/bin/rkaiq_3A_server