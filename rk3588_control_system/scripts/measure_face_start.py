#!/usr/bin/env python3
"""Measure warm face-tracker activation to first locked target."""

from __future__ import annotations

import argparse
import json
import time
import urllib.request


def request_json(base_url: str, path: str, body: dict[str, object] | None = None) -> dict[str, object]:
    data = None if body is None else json.dumps(body).encode("utf-8")
    request = urllib.request.Request(
        base_url.rstrip("/") + path,
        data=data,
        headers={"Content-Type": "application/json"} if data is not None else {},
        method="POST" if data is not None else "GET",
    )
    with urllib.request.urlopen(request, timeout=4) as response:
        return json.load(response)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-url", default="http://127.0.0.1:3000")
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--poll-ms", type=float, default=50.0)
    args = parser.parse_args()

    started_at = time.monotonic()
    start_result = request_json(args.base_url, "/api/gimbal/track/start", {"mode": "face"})
    print("START " + json.dumps(start_result, separators=(",", ":")))
    last_marker = None
    first_lock_ms = None
    deadline = started_at + max(0.5, args.timeout)
    try:
        while time.monotonic() < deadline:
            state = request_json(args.base_url, "/api/gimbal/state").get("state", {})
            status = state.get("trackStatus", {}) if isinstance(state, dict) else {}
            marker = (
                status.get("status"),
                bool(status.get("locked")),
                status.get("detector_age_ms"),
                status.get("detections"),
            )
            if marker != last_marker:
                elapsed_ms = round((time.monotonic() - started_at) * 1000)
                print(f"SAMPLE_MS {elapsed_ms} {marker}")
                last_marker = marker
            if status.get("locked"):
                first_lock_ms = round((time.monotonic() - started_at) * 1000)
                break
            time.sleep(max(0.01, args.poll_ms / 1000.0))
    finally:
        stop_result = request_json(args.base_url, "/api/gimbal/track/stop", {})
        print("STOP " + json.dumps({"success": stop_result.get("success")}, separators=(",", ":")))

    print(f"FIRST_LOCK_MS {first_lock_ms if first_lock_ms is not None else 'timeout'}")
    return 0 if first_lock_ms is not None else 2


if __name__ == "__main__":
    raise SystemExit(main())
