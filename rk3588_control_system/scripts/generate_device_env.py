#!/usr/bin/env python3

import argparse
import os
import secrets
import shlex
from pathlib import Path


def parse_env(text):
    values = {}
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip().strip("'\"")
    return values


def generated_values(existing):
    values = dict(existing)
    values.setdefault(
        "MANTA_HOTSPOT_PASSWORD",
        os.environ.get("MANTA_HOTSPOT_PASSWORD") or secrets.token_urlsafe(15),
    )
    values.setdefault(
        "MANTA_BLUETOOTH_PIN",
        os.environ.get("MANTA_BLUETOOTH_PIN") or f"{secrets.randbelow(1_000_000):06d}",
    )
    for key in ("MANTA_AMAP_JS_KEY", "MANTA_AMAP_SECURITY_CODE"):
        if key not in values and os.environ.get(key):
            values[key] = os.environ[key]
    return values


def write_env(path, values):
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = ["# Generated once by MANTA installation. Keep this file private."]
    for key in sorted(values):
        lines.append(f"{key}={shlex.quote(str(values[key]))}")
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text("\n".join(lines) + "\n", encoding="utf-8")
    os.chmod(temporary, 0o600)
    os.replace(temporary, path)


def main():
    parser = argparse.ArgumentParser(description="Create persistent per-device MANTA secrets")
    parser.add_argument("path", nargs="?", default="/etc/manta/manta.env")
    args = parser.parse_args()
    path = Path(args.path)
    existing = parse_env(path.read_text(encoding="utf-8")) if path.exists() else {}
    write_env(path, generated_values(existing))
    print(f"[device-env] Ready: {path} (credentials not displayed)")


if __name__ == "__main__":
    main()
