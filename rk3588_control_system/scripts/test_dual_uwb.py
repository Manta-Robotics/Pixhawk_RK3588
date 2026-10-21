#!/usr/bin/env python3
"""Live acceptance test for the Manta dual-UWB wide-field configuration."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.error import URLError
from urllib.request import urlopen


DEFAULT_URL = "http://127.0.0.1:3000/api/telemetry"


def number(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def fmt(value: Any, width: int = 7, precision: int = 2) -> str:
    result = number(value)
    return f"{result:{width}.{precision}f}" if result is not None else "-".rjust(width)


def angle_error_deg(first: float, second: float) -> float:
    return abs((float(first) - float(second) + 180.0) % 360.0 - 180.0)


def fetch_uwb(url: str, timeout: float) -> Dict[str, Any]:
    with urlopen(url, timeout=timeout) as response:
        payload = json.load(response)
    telemetry = payload.get("telemetry", payload)
    uwb = telemetry.get("uwb")
    if not isinstance(uwb, dict):
        raise ValueError("response does not contain telemetry.uwb")
    return uwb


def receiver_map(uwb: Dict[str, Any]) -> Dict[int, Dict[str, Any]]:
    result: Dict[int, Dict[str, Any]] = {}
    for sensor in uwb.get("sensors", []):
        if not isinstance(sensor, dict):
            continue
        receiver_id = int(number(sensor.get("receiverId")) or 0)
        if receiver_id in (1, 2):
            result[receiver_id] = sensor
    return result


@dataclass
class ReceiverStats:
    samples: int = 0
    valid_samples: int = 0
    fresh_samples: int = 0
    distances: list[float] = field(default_factory=list)
    body_angles: list[float] = field(default_factory=list)
    anchor_ids: set[int] = field(default_factory=set)
    tag_ids: set[int] = field(default_factory=set)
    tag_statuses: set[int] = field(default_factory=set)
    rx_bytes: int = 0
    heartbeat_count: int = 0
    checksum_errors: int = 0
    length_errors: int = 0

    def update(self, sensor: Dict[str, Any]) -> None:
        self.samples += 1
        if sensor.get("valid") is True:
            self.valid_samples += 1
        if sensor.get("fresh") is True:
            self.fresh_samples += 1
        distance = number(sensor.get("distanceM"))
        angle = number(sensor.get("bodyAzimuthDeg"))
        if distance is not None:
            self.distances.append(distance)
        if angle is not None:
            self.body_angles.append(angle)
        for key, target in (
            ("anchorId", self.anchor_ids),
            ("tagId", self.tag_ids),
            ("tagStatus", self.tag_statuses),
        ):
            value = number(sensor.get(key))
            if value is not None:
                target.add(int(value))
        for key, attribute in (
            ("rxBytes", "rx_bytes"),
            ("heartbeatCount", "heartbeat_count"),
            ("checksumErrors", "checksum_errors"),
            ("lengthErrors", "length_errors"),
        ):
            value = number(sensor.get(key))
            if value is not None:
                setattr(self, attribute, int(value))

    @property
    def ever_valid(self) -> bool:
        return self.valid_samples > 0


CSV_COLUMNS = [
    "time_iso",
    "uwb1_valid", "uwb1_fresh", "uwb1_sequence_id", "uwb1_anchor_id", "uwb1_tag_id", "uwb1_tag_status",
    "uwb1_rx_bytes", "uwb1_heartbeat_count", "uwb1_checksum_errors", "uwb1_length_errors",
    "uwb1_distance_m", "uwb1_sensor_azimuth_deg", "uwb1_body_azimuth_deg",
    "uwb2_valid", "uwb2_fresh", "uwb2_sequence_id", "uwb2_anchor_id", "uwb2_tag_id", "uwb2_tag_status",
    "uwb2_rx_bytes", "uwb2_heartbeat_count", "uwb2_checksum_errors", "uwb2_length_errors",
    "uwb2_distance_m", "uwb2_sensor_azimuth_deg", "uwb2_body_azimuth_deg",
    "fused_online", "fused_distance_m", "fused_azimuth_deg", "fusion_mode",
    "observing_receiver_count", "active_receiver_ids", "bearing_disagreement_deg",
    "receiver_distance_error_m", "receiver_body_angle_error_deg",
]


def csv_row(uwb: Dict[str, Any], sensors: Dict[int, Dict[str, Any]]) -> Dict[str, Any]:
    first = sensors.get(1, {})
    second = sensors.get(2, {})
    distance_1 = number(first.get("distanceM"))
    distance_2 = number(second.get("distanceM"))
    angle_1 = number(first.get("bodyAzimuthDeg"))
    angle_2 = number(second.get("bodyAzimuthDeg"))
    return {
        "time_iso": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "uwb1_valid": first.get("valid"),
        "uwb1_fresh": first.get("fresh"),
        "uwb1_sequence_id": first.get("sequenceId"),
        "uwb1_anchor_id": first.get("anchorId"),
        "uwb1_tag_id": first.get("tagId"),
        "uwb1_tag_status": first.get("tagStatus"),
        "uwb1_rx_bytes": first.get("rxBytes"),
        "uwb1_heartbeat_count": first.get("heartbeatCount"),
        "uwb1_checksum_errors": first.get("checksumErrors"),
        "uwb1_length_errors": first.get("lengthErrors"),
        "uwb1_distance_m": first.get("distanceM"),
        "uwb1_sensor_azimuth_deg": first.get("azimuthDeg"),
        "uwb1_body_azimuth_deg": first.get("bodyAzimuthDeg"),
        "uwb2_valid": second.get("valid"),
        "uwb2_fresh": second.get("fresh"),
        "uwb2_sequence_id": second.get("sequenceId"),
        "uwb2_anchor_id": second.get("anchorId"),
        "uwb2_tag_id": second.get("tagId"),
        "uwb2_tag_status": second.get("tagStatus"),
        "uwb2_rx_bytes": second.get("rxBytes"),
        "uwb2_heartbeat_count": second.get("heartbeatCount"),
        "uwb2_checksum_errors": second.get("checksumErrors"),
        "uwb2_length_errors": second.get("lengthErrors"),
        "uwb2_distance_m": second.get("distanceM"),
        "uwb2_sensor_azimuth_deg": second.get("azimuthDeg"),
        "uwb2_body_azimuth_deg": second.get("bodyAzimuthDeg"),
        "fused_online": uwb.get("online"),
        "fused_distance_m": uwb.get("distanceM"),
        "fused_azimuth_deg": uwb.get("azimuthDeg"),
        "fusion_mode": uwb.get("fusionMode"),
        "observing_receiver_count": uwb.get("observingReceiverCount"),
        "active_receiver_ids": ";".join(str(value) for value in uwb.get("activeReceiverIds", [])),
        "bearing_disagreement_deg": uwb.get("bearingDisagreementDeg"),
        "receiver_distance_error_m": abs(distance_1 - distance_2) if distance_1 is not None and distance_2 is not None else None,
        "receiver_body_angle_error_deg": angle_error_deg(angle_1, angle_2) if angle_1 is not None and angle_2 is not None else None,
    }


def print_header() -> None:
    print(
        "TIME       | UWB1 V/F  DIST  SENSOR  BODY | UWB2 V/F  DIST  SENSOR  BODY "
        "| FUSED DIST  BODY  MODE"
    )


def print_sample(uwb: Dict[str, Any], sensors: Dict[int, Dict[str, Any]]) -> None:
    fields = []
    for receiver_id in (1, 2):
        sensor = sensors.get(receiver_id, {})
        valid = "1" if sensor.get("valid") is True else "0"
        fresh = "1" if sensor.get("fresh") is True else "0"
        fields.append(
            f"UWB{receiver_id} {valid}/{fresh} {fmt(sensor.get('distanceM'), 5)} "
            f"{fmt(sensor.get('azimuthDeg'), 7, 1)} {fmt(sensor.get('bodyAzimuthDeg'), 7, 1)}"
        )
    online = "ON " if uwb.get("online") is True else "OFF"
    mode = str(uwb.get("fusionMode") or "unknown")
    print(
        f"{time.strftime('%H:%M:%S')} | {fields[0]} | {fields[1]} | "
        f"{online} {fmt(uwb.get('distanceM'), 5)} {fmt(uwb.get('azimuthDeg'), 7, 1)} {mode}"
    )


def print_receiver_summary(receiver_id: int, stats: ReceiverStats) -> None:
    valid_rate = 100.0 * stats.valid_samples / stats.samples if stats.samples else 0.0
    fresh_rate = 100.0 * stats.fresh_samples / stats.samples if stats.samples else 0.0
    distance_span = "-"
    angle_span = "-"
    if stats.distances:
        distance_span = f"{min(stats.distances):.2f}..{max(stats.distances):.2f} m"
    if stats.body_angles:
        angle_span = f"{min(stats.body_angles):.1f}..{max(stats.body_angles):.1f} deg"
    print(
        f"UWB{receiver_id}: valid={valid_rate:.1f}% fresh={fresh_rate:.1f}% "
        f"distance={distance_span} body-angle={angle_span} "
        f"anchor={sorted(stats.anchor_ids) or '-'} tag={sorted(stats.tag_ids) or '-'} "
        f"status={sorted(stats.tag_statuses) or '-'} "
        f"rx={stats.rx_bytes} hb={stats.heartbeat_count} "
        f"crc={stats.checksum_errors} len={stats.length_errors}"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Test TELEM1/TELEM2 UWB receivers and the RK3588 wide-field fusion output."
    )
    parser.add_argument("--url", default=DEFAULT_URL, help="Manta telemetry HTTP endpoint")
    parser.add_argument("--duration", type=float, default=30.0, help="test duration in seconds; 0 runs until Ctrl+C")
    parser.add_argument("--rate", type=float, default=5.0, help="sampling rate in Hz")
    parser.add_argument("--print-rate", type=float, default=2.0, help="console refresh rate in Hz; 0 disables samples")
    parser.add_argument("--timeout", type=float, default=1.5, help="HTTP timeout in seconds")
    parser.add_argument("--csv", type=Path, help="optional output CSV path")
    parser.add_argument(
        "--allow-partial",
        action="store_true",
        help="return success when at least one receiver becomes valid",
    )
    parser.add_argument("--max-overlap-angle-error", type=float, default=15.0,
                        help="maximum mean body-frame bearing mismatch in degrees")
    parser.add_argument("--max-overlap-distance-error", type=float, default=0.50,
                        help="maximum mean distance mismatch in metres")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.rate <= 0 or args.print_rate < 0 or args.duration < 0:
        print("rate must be > 0; print-rate and duration must be >= 0", file=sys.stderr)
        return 2

    csv_file = None
    writer = None
    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        csv_file = args.csv.open("w", newline="", encoding="utf-8-sig")
        writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNS)
        writer.writeheader()

    stats = {1: ReceiverStats(), 2: ReceiverStats()}
    sample_count = 0
    overlap_samples = 0
    overlap_angle_errors: list[float] = []
    overlap_distance_errors: list[float] = []
    request_errors = 0
    started = time.monotonic()
    next_sample = started
    next_print = started
    print_header()

    try:
        while args.duration == 0 or time.monotonic() - started < args.duration:
            now = time.monotonic()
            if now < next_sample:
                time.sleep(min(0.05, next_sample - now))
                continue
            next_sample += 1.0 / args.rate
            try:
                uwb = fetch_uwb(args.url, args.timeout)
            except (OSError, URLError, ValueError, json.JSONDecodeError) as exc:
                request_errors += 1
                if request_errors <= 3 or request_errors % 10 == 0:
                    print(f"telemetry error: {exc}", file=sys.stderr)
                continue

            sensors = receiver_map(uwb)
            sample_count += 1
            for receiver_id in (1, 2):
                stats[receiver_id].update(sensors.get(receiver_id, {}))
            if int(number(uwb.get("observingReceiverCount")) or 0) >= 2:
                overlap_samples += 1
                first, second = sensors.get(1, {}), sensors.get(2, {})
                angle_1, angle_2 = number(first.get("bodyAzimuthDeg")), number(second.get("bodyAzimuthDeg"))
                distance_1, distance_2 = number(first.get("distanceM")), number(second.get("distanceM"))
                if angle_1 is not None and angle_2 is not None:
                    overlap_angle_errors.append(angle_error_deg(angle_1, angle_2))
                if distance_1 is not None and distance_2 is not None:
                    overlap_distance_errors.append(abs(distance_1 - distance_2))
            if writer:
                writer.writerow(csv_row(uwb, sensors))
            if args.print_rate > 0 and now >= next_print:
                print_sample(uwb, sensors)
                next_print = now + 1.0 / args.print_rate
    except KeyboardInterrupt:
        print("\nStopped by operator.")
    finally:
        if csv_file:
            csv_file.close()

    print("\n--- Dual UWB test summary ---")
    print(f"samples={sample_count} telemetry_errors={request_errors} overlap_samples={overlap_samples}")
    print_receiver_summary(1, stats[1])
    print_receiver_summary(2, stats[2])
    if overlap_angle_errors or overlap_distance_errors:
        mean_angle_error = sum(overlap_angle_errors) / len(overlap_angle_errors) if overlap_angle_errors else math.inf
        mean_distance_error = sum(overlap_distance_errors) / len(overlap_distance_errors) if overlap_distance_errors else math.inf
        print(f"overlap consistency: mean body-angle error={mean_angle_error:.2f} deg, "
              f"mean distance error={mean_distance_error:.3f} m")
        if mean_angle_error > args.max_overlap_angle_error or mean_distance_error > args.max_overlap_distance_error:
            print("RESULT: FAIL - the two receivers disagree after body-frame conversion")
            return 1
    if args.csv:
        print(f"CSV: {args.csv.resolve()}")

    valid_receivers = sum(1 for receiver_stats in stats.values() if receiver_stats.ever_valid)
    if valid_receivers == 2:
        print("RESULT: PASS - both receivers produced valid measurements")
        return 0
    if valid_receivers == 1:
        print("RESULT: PARTIAL - only one receiver produced valid measurements")
        return 0 if args.allow_partial else 1
    print("RESULT: FAIL - neither receiver produced a valid measurement")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
