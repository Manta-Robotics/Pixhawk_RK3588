#!/usr/bin/env bash

set -u

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG_FILE="$PROJECT_DIR/config/system.config.json"
SNAPSHOT_PORT="8090"
CAMERA_SENSOR="camera"
CAMERA_OVERLAY=""
CAMERA_LABEL="Camera"
CAMERA_PORT_NAME=""
RAW_CAPTURE_TIMEOUT="12"
HAS_RKISP_MAINPATH=0
HAS_MEDIA_NODE=0
KERNEL_CAMERA_LOGS=""
HAS_SENSOR_STREAM_ON=0

if [[ -f "$CONFIG_FILE" ]]; then
    eval "$(python3 - "$CONFIG_FILE" <<'PY'
import json
import shlex
import sys

cfg = json.load(open(sys.argv[1], 'r', encoding='utf-8'))
hotspot = cfg.get('hotspot', {})
camera = cfg.get('camera', {})
values = {
    'SNAPSHOT_PORT': hotspot.get('camera_port', 8090),
    'CAMERA_SENSOR': camera.get('sensor', 'camera'),
    'CAMERA_OVERLAY': camera.get('overlay', ''),
    'CAMERA_LABEL': camera.get('label', 'Camera'),
    'CAMERA_PORT_NAME': camera.get('port', '')
}

for key, value in values.items():
    print(f"{key}={shlex.quote(str(value))}")
PY
)"
    SNAPSHOT_PORT="${SNAPSHOT_PORT:-8090}"
fi

OVERLAY_DTBO=""
if [[ -n "$CAMERA_OVERLAY" ]]; then
    for candidate in \
        "/boot/firmware/dtbs/rockchip/overlay/${CAMERA_OVERLAY}.dtbo" \
        "/boot/dtbs/rockchip/overlay/${CAMERA_OVERLAY}.dtbo" \
        "/lib/firmware/${CAMERA_OVERLAY}.dtbo"; do
        if [[ -f "$candidate" ]]; then
            OVERLAY_DTBO="$candidate"
            break
        fi
    done
fi

print_section() {
    printf '\n==== %s ====\n' "$1"
}

print_section "Camera Flow Check"
echo "Project: $PROJECT_DIR"
echo "Camera : ${CAMERA_LABEL} (${CAMERA_SENSOR}) ${CAMERA_PORT_NAME:+on ${CAMERA_PORT_NAME}}"
echo "Overlay: ${CAMERA_OVERLAY:-<unset>}"
echo "Reference: A working MIPI camera should enumerate as a Rockchip ISP camera node such as rkisp_mainpath, not only stream_hdmirx."

print_section "Configured Overlay"
if [[ -r /boot/firmware/ubuntuEnv.txt ]]; then
    grep -nE "${CAMERA_SENSOR}|${CAMERA_PORT_NAME}|^overlays=" /boot/firmware/ubuntuEnv.txt || echo "No camera-related overlay lines found in /boot/firmware/ubuntuEnv.txt"
elif [[ -r /boot/uEnv/uEnv.txt ]]; then
    grep -nE "${CAMERA_SENSOR}|${CAMERA_PORT_NAME}|^overlays=" /boot/uEnv/uEnv.txt || echo "No camera-related overlay lines found in /boot/uEnv/uEnv.txt"
else
    echo "No known boot camera config file found."
fi

if [[ -n "$OVERLAY_DTBO" ]]; then
    echo "Overlay file found: $OVERLAY_DTBO"
else
    echo "Overlay file missing for configured camera: ${CAMERA_OVERLAY:-<unset>}"
fi

if [[ -r /boot/firmware/ubuntuEnv.txt && -n "$CAMERA_OVERLAY" ]]; then
    if grep -Eq "^overlays=.*${CAMERA_OVERLAY}" /boot/firmware/ubuntuEnv.txt; then
        echo "Overlay is correctly listed inside overlays=."
    elif grep -Eq "^[[:space:]]*${CAMERA_OVERLAY}[[:space:]]*$" /boot/firmware/ubuntuEnv.txt; then
        echo "Overlay appears as a standalone line, not inside overlays=. On this boot format that does not enable the camera."
    fi
fi

print_section "Video Nodes"
ls -l /dev/video* /dev/media* 2>/dev/null || echo "No /dev/video* or /dev/media* nodes found."

if compgen -G '/dev/media*' >/dev/null; then
    HAS_MEDIA_NODE=1
fi

print_section "Kernel Video Names"
found_names=0
for name_file in /sys/class/video4linux/*/name; do
    [[ -e "$name_file" ]] || continue
    found_names=1
    printf '%s: ' "$name_file"
    cat "$name_file"
done
if [[ $found_names -eq 0 ]]; then
    echo "No /sys/class/video4linux entries found."
fi

if grep -Rqs '^rkisp_mainpath$' /sys/class/video4linux/*/name 2>/dev/null; then
    HAS_RKISP_MAINPATH=1
fi

print_section "Media Topology"
if command -v media-ctl >/dev/null 2>&1; then
    for media_dev in /dev/media*; do
        [[ -e "$media_dev" ]] || continue
        echo "-- $media_dev --"
        media-ctl -d "$media_dev" -p || true
    done
else
    echo "media-ctl not installed; skipping topology dump."
fi

print_section "v4l2 Device Summary"
if command -v v4l2-ctl >/dev/null 2>&1; then
    v4l2-ctl --list-devices || true
else
    echo "v4l2-ctl not installed; skipping device summary."
fi

print_section "Snapshot Service Health"
if command -v curl >/dev/null 2>&1; then
    curl -sf "http://127.0.0.1:${SNAPSHOT_PORT}/healthz" || echo "Snapshot health endpoint unavailable on :${SNAPSHOT_PORT}"
else
    echo "curl not installed; skipping snapshot health check."
fi

print_section "Raw Capture Probe"
RAW_CAPTURE_STATUS="skipped"
RAW_CAPTURE_DEVICE=""
RAW_CAPTURE_SIZE="0"
RAW_CAPTURE_NOTE=""
RAW_CAPTURE_TMP=""

if command -v v4l2-ctl >/dev/null 2>&1; then
    for candidate in /dev/video0 /dev/video11; do
        [[ -e "$candidate" ]] || continue

        RAW_CAPTURE_DEVICE="$candidate"
        RAW_CAPTURE_TMP="$(mktemp /tmp/camera-probe-XXXXXX.bin)"

        if timeout "${RAW_CAPTURE_TIMEOUT}s" v4l2-ctl -d "$candidate" --stream-mmap=4 --stream-count=1 --stream-to="$RAW_CAPTURE_TMP" >/tmp/camera-probe.log 2>&1; then
            RAW_CAPTURE_STATUS="ok"
        else
            capture_rc=$?
            if [[ $capture_rc -eq 124 ]]; then
                RAW_CAPTURE_STATUS="timeout"
            else
                RAW_CAPTURE_STATUS="failed"
            fi
        fi

        RAW_CAPTURE_SIZE="$(wc -c < "$RAW_CAPTURE_TMP" 2>/dev/null || echo 0)"
        echo "Device: $RAW_CAPTURE_DEVICE"
        echo "Result: $RAW_CAPTURE_STATUS"
        echo "Bytes : $RAW_CAPTURE_SIZE"
        sed -n '1,80p' /tmp/camera-probe.log

        if [[ "$RAW_CAPTURE_SIZE" != "0" ]]; then
            RAW_CAPTURE_NOTE="A non-zero raw frame was captured. The sensor-to-CSI data path is alive."
            rm -f "$RAW_CAPTURE_TMP" /tmp/camera-probe.log
            break
        fi

        RAW_CAPTURE_NOTE="The capture command returned without producing a frame. If media nodes exist, this points below the web stack."
        rm -f "$RAW_CAPTURE_TMP" /tmp/camera-probe.log
    done
else
    echo "v4l2-ctl not installed; skipping raw capture probe."
fi

print_section "Recent Kernel Camera Logs"
KERNEL_CAMERA_LOGS="$(dmesg | grep -Ei "${CAMERA_SENSOR}|rkisp|rkcif|mipi|csi|camera" || true)"
if [[ -n "$KERNEL_CAMERA_LOGS" ]]; then
    printf '%s\n' "$KERNEL_CAMERA_LOGS" | tail -n 80
else
    echo "No recent camera-related kernel logs found."
fi

if [[ "$KERNEL_CAMERA_LOGS" == *"${CAMERA_SENSOR}"*"s_stream: on: 1"* ]]; then
    HAS_SENSOR_STREAM_ON=1
fi

print_section "Conclusion"
echo "If you still only see stream_hdmirx and no rkisp_mainpath/media node, the web stack is not the blocker."
echo "For ${CAMERA_SENSOR} on LubanCat, re-check ribbon direction, ${CAMERA_PORT_NAME:-camera} connection, camera power, and reboot after enabling the overlay."
if [[ -z "$OVERLAY_DTBO" ]]; then
    echo "The configured overlay ${CAMERA_OVERLAY:-<unset>} is not installed on this system. This blocks camera bring-up below the web stack."
fi
if [[ "$RAW_CAPTURE_STATUS" == "timeout" && "$RAW_CAPTURE_SIZE" == "0" ]]; then
    echo "Raw capture on ${RAW_CAPTURE_DEVICE:-the tested device} timed out with 0 bytes. If ${CAMERA_SENSOR}, CSI, and rkisp nodes are present, this points to a sensor/CSI/FPC data-path fault rather than the frontend or snapshot service."
elif [[ "$RAW_CAPTURE_STATUS" == "ok" && "$RAW_CAPTURE_SIZE" != "0" ]]; then
    echo "Raw capture on ${RAW_CAPTURE_DEVICE:-the tested device} returned ${RAW_CAPTURE_SIZE} bytes, so the sensor and CSI data path are alive. Any remaining issue is above raw capture, such as ISP, proxying, or frontend rendering."
elif [[ -n "$RAW_CAPTURE_NOTE" ]]; then
    echo "$RAW_CAPTURE_NOTE"
fi

if [[ $HAS_RKISP_MAINPATH -eq 1 && $HAS_MEDIA_NODE -eq 1 ]]; then
    echo "rkisp_mainpath and media nodes are present, so the Rockchip camera pipeline is up. The remaining blocker is now below the web stack."
fi

if [[ $HAS_RKISP_MAINPATH -eq 1 && "$RAW_CAPTURE_SIZE" == "0" && "$KERNEL_CAMERA_LOGS" == *"get remote terminal sensor failed"* ]]; then
    echo "The kernel cannot find a remote sensor on the CSI link. If the camera is not plugged in yet, this output is expected."
    echo "Software-side preparation is far enough for the next real test: power off, insert the camera into ${CAMERA_PORT_NAME:-the configured camera port}, boot again, and rerun this script."
fi

if [[ $HAS_SENSOR_STREAM_ON -eq 1 ]]; then
    echo "Kernel logs show ${CAMERA_SENSOR} entered streaming (s_stream on:1). Sensor detection and CSI link are working."
    echo "If the web page still has no image, focus next on selecting the right capture node/format in snapshot service rather than overlay or wiring."
fi

if [[ "$KERNEL_CAMERA_LOGS" == *"write reg array error"* ]]; then
    echo "Kernel logs still show 'write reg array error', which usually indicates an unstable sensor mode, incompatible overlay, or hardware-side link problem."
fi

if [[ "$KERNEL_CAMERA_LOGS" == *"rkisp_stream_stop id:0 timeout"* ]]; then
    echo "Kernel logs show rkisp stream-stop timeout, which means the ISP was asked to stop a stream that never produced a clean frame sequence."
fi