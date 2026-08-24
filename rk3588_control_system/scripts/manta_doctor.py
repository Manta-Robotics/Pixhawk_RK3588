#!/usr/bin/env python3

import argparse
import importlib.util
import json
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
SYSTEM_CONFIG = PROJECT_DIR / "config" / "system.config.json"
DEVICE_ENV = Path("/etc/manta/manta.env")
SERVICES = (
    "manta-backend.service",
    "manta-bridge.service",
    "manta-camera.service",
    "manta-gimbal-route.service",
    "manta-gimbal-stream.service",
    "manta-mediamtx.service",
    "manta-hotspot.service",
    "manta-captive-portal.service",
    "manta-bluetooth-pan.service",
)


class Report:
    def __init__(self):
        self.failures = 0
        self.warnings = 0

    def emit(self, level, label, detail):
        if level == "FAIL":
            self.failures += 1
        elif level == "WARN":
            self.warnings += 1
        print(f"[{level:<4}] {label:<24} {detail}")

    def ok(self, label, detail):
        self.emit("PASS", label, detail)

    def warn(self, label, detail):
        self.emit("WARN", label, detail)

    def fail(self, label, detail):
        self.emit("FAIL", label, detail)


def run(command):
    return subprocess.run(command, text=True, capture_output=True, check=False)


def parse_env_file(path):
    values = {}
    for raw_line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip().strip("'\"")
    return values


def command_version(command, args=("--version",)):
    path = shutil.which(command)
    if not path:
        return None
    result = run([path, *args])
    if result.returncode != 0 and args == ("--version",):
        result = run([path, "-version"])
    if result.returncode != 0:
        return None
    output = (result.stdout or result.stderr).strip().splitlines()
    return output[0] if output else path


def load_config(report):
    try:
        config = json.loads(SYSTEM_CONFIG.read_text(encoding="utf-8"))
        report.ok("system config", "valid JSON")
        return config
    except Exception as error:
        report.fail("system config", str(error))
        return {}


def check_source(report, config):
    required = (
        "backend/server.js",
        "backend/mavlink_bridge.py",
        "frontend/mobile-preview-kimi-k26.html",
        "frontend/assets/manta-app/manta-hero.jpg",
        "frontend/js/gps-map-core.js",
        "scripts/install.sh",
        "scripts/install_boot_services.sh",
        "systemd/manta-backend.service.template",
    )
    missing = [entry for entry in required if not (PROJECT_DIR / entry).is_file()]
    if missing:
        report.fail("repository assets", "missing: " + ", ".join(missing))
    else:
        report.ok("repository assets", "required source and UI assets present")

    serial_port = str(config.get("serial_port", ""))
    gimbal_port = str(config.get("gimbal", {}).get("serial_port", ""))
    report.ok("protocol config", f"Pixhawk {serial_port or '--'}; gimbal {gimbal_port or '--'}")

    transport = str(config.get("gimbal", {}).get("control_transport", "")).lower()
    if transport not in {"uart", "udp"}:
        report.fail("gimbal control", "control_transport must be uart or udp")
    else:
        report.ok("gimbal control", f"single configured transport: {transport}")

    password = str(config.get("hotspot", {}).get("password", ""))
    if password != "CHANGE_ME_AT_INSTALL":
        report.warn("hotspot defaults", "tracked config should use the install-time placeholder")
    else:
        report.ok("hotspot defaults", "per-device credential placeholder configured")

    try:
        bluetooth = json.loads((PROJECT_DIR / "config" / "bluetooth.config.json").read_text(encoding="utf-8"))
        pin = str(bluetooth.get("security", {}).get("pin", ""))
        if pin == "CHANGE_ME_AT_INSTALL":
            report.ok("Bluetooth defaults", "per-device PIN placeholder configured")
        else:
            report.warn("Bluetooth defaults", "tracked config should use the install-time placeholder")
    except Exception as error:
        report.fail("Bluetooth config", str(error))


def check_runtime(report, installed):
    required_commands = ("node", "npm", "python3", "curl", "ffmpeg", "ffprobe", "nmcli", "dnsmasq", "bluetoothctl")
    for command in required_commands:
        version = command_version(command)
        if version:
            report.ok("command " + command, version[:90])
        elif installed:
            report.fail("command " + command, "not installed")
        else:
            report.warn("command " + command, "not installed")

    mediamtx = command_version("mediamtx")
    if mediamtx:
        report.ok("MediaMTX", mediamtx[:90])
    elif installed:
        report.fail("MediaMTX", "not installed")
    else:
        report.warn("MediaMTX", "not installed yet")

    python_modules = ("pymavlink", "serial", "cv2", "numpy", "ultralytics", "psutil", "dbus", "gi")
    missing_modules = [name for name in python_modules if importlib.util.find_spec(name) is None]
    if missing_modules and installed:
        report.fail("Python modules", "missing: " + ", ".join(missing_modules))
    elif missing_modules:
        report.warn("Python modules", "not installed yet: " + ", ".join(missing_modules))
    else:
        report.ok("Python modules", "runtime imports available")


def check_board(report, config, installed):
    if platform.machine() not in {"aarch64", "arm64"}:
        report.warn("architecture", platform.machine() + " (deployment target is aarch64)")
    else:
        report.ok("architecture", platform.machine())

    for label, device in (
        ("Pixhawk serial", config.get("serial_port", "/dev/ttyS1")),
        ("gimbal serial", config.get("gimbal", {}).get("serial_port", "/dev/ttyS3")),
    ):
        path = Path(str(device))
        if path.exists():
            report.ok(label, f"{path} present")
        else:
            report.warn(label, f"{path} missing; hardware link not validated")

    camera = config.get("camera", {})
    overlay = str(camera.get("overlay", ""))
    pixhawk = config.get("pixhawk", {})
    pixhawk_uart_overlay = str(pixhawk.get("uart_overlay", ""))
    gimbal = config.get("gimbal", {})
    uart_overlay = str(gimbal.get("uart_overlay", ""))
    boot_config = Path(str(pixhawk.get("boot_config") or camera.get("boot_config") or gimbal.get("boot_config") or "/boot/firmware/ubuntuEnv.txt"))
    boot_text = boot_config.read_text(encoding="utf-8", errors="ignore") if boot_config.exists() else ""
    overlay_checks = []
    overlay_checks.append(("Pixhawk UART overlay", pixhawk_uart_overlay))
    if camera.get("enabled", True):
        overlay_checks.append(("camera overlay", overlay))
    if str(gimbal.get("control_transport", "uart")).lower() == "uart":
        overlay_checks.append(("gimbal UART overlay", uart_overlay))
    for label, name in overlay_checks:
        if name and name in boot_text:
            report.ok(label, "enabled in boot config")
        elif installed:
            report.fail(label, "not enabled in boot config")
        else:
            report.warn(label, "not enabled yet")

    if Path("/sys/class/net/wlan0").exists():
        report.ok("Wi-Fi device", "wlan0 present")
    else:
        report.warn("Wi-Fi device", "wlan0 missing")

    bluetooth_adapters = sorted(Path("/sys/class/bluetooth").glob("hci*"))
    if bluetooth_adapters:
        report.ok("Bluetooth adapter", ", ".join(path.name for path in bluetooth_adapters))
    else:
        report.warn("Bluetooth adapter", "no hci device detected; PAN service will wait")

    if DEVICE_ENV.exists():
        mode = DEVICE_ENV.stat().st_mode & 0o777
        if mode == 0o600:
            report.ok("device credentials", "/etc/manta/manta.env mode 0600")
        else:
            report.fail("device credentials", f"unexpected mode {mode:o}")
        device_values = parse_env_file(DEVICE_ENV)
        amap_key = device_values.get("MANTA_AMAP_JS_KEY", "")
        amap_secret = device_values.get("MANTA_AMAP_SECURITY_CODE", "")
        if amap_key and amap_secret:
            report.ok("Amap credentials", "JS key and server-side security code configured")
        elif amap_key or amap_secret:
            report.fail("Amap credentials", "both MANTA_AMAP_JS_KEY and MANTA_AMAP_SECURITY_CODE are required")
        else:
            report.warn("Amap credentials", "not configured; local track fallback will be used")
    elif installed:
        report.fail("device credentials", "missing /etc/manta/manta.env")
    else:
        report.warn("device credentials", "will be generated during installation")


def check_services(report, installed):
    if not shutil.which("systemctl"):
        report.warn("systemd", "systemctl unavailable")
        return
    for service in SERVICES:
        installed_result = run(["systemctl", "cat", service])
        if installed_result.returncode != 0:
            if installed:
                report.fail(service, "unit not installed")
            else:
                report.warn(service, "unit not installed yet")
            continue
        enabled = run(["systemctl", "is-enabled", service]).stdout.strip() or "unknown"
        active = run(["systemctl", "is-active", service]).stdout.strip() or "unknown"
        level = "PASS" if enabled == "enabled" and active == "active" else "WARN"
        report.emit(level, service, f"enabled={enabled}, active={active}")


def main():
    parser = argparse.ArgumentParser(description="Read-only MANTA source and board readiness check")
    parser.add_argument("--installed", action="store_true", help="treat missing runtime components as failures")
    args = parser.parse_args()
    report = Report()
    print("MANTA doctor (read-only)\n")
    config = load_config(report)
    check_source(report, config)
    check_runtime(report, args.installed)
    check_board(report, config, args.installed)
    check_services(report, args.installed)
    print(f"\nSummary: failures={report.failures}, warnings={report.warnings}")
    return 1 if report.failures else 0


if __name__ == "__main__":
    sys.exit(main())
