"""Fuse one or two directional UWB receivers into the vessel body frame."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import re
import time
from typing import Deque, Dict, Iterable, Optional, Tuple


UWB_NAME_PATTERNS = (
    re.compile(r"^UWB([12])_(DIST|AZ|EL|GOOD|VALID|SEQ|ANCH|TAG|STAT|RX|HB|CRC|LEN)$"),
    re.compile(r"^UWB_(DIST|AZ|EL|GOOD|VALID|SEQ|ANCH|TAG|STAT|RX|HB|CRC|LEN)([12])$"),
)
LEGACY_UWB_NAMES = {
    "UWB_DIST", "UWB_AZ", "UWB_EL", "UWB_GOOD", "UWB_VALID",
    "UWB_SEQ", "UWB_ANCH", "UWB_TAG", "UWB_STAT",
    "UWB_RX", "UWB_HB", "UWB_CRC", "UWB_LEN",
}


def wrap_angle_deg(value: float) -> float:
    return (float(value) + 180.0) % 360.0 - 180.0


@dataclass
class UwbReceiver:
    receiver_id: int
    mount_yaw_deg: float
    position_x_m: float = 0.0
    position_y_m: float = 0.0
    source_system: Optional[int] = None
    source_component: Optional[int] = None
    raw_distance_m: Optional[float] = None
    raw_azimuth_deg: Optional[float] = None
    raw_elevation_deg: Optional[float] = None
    distance_m: Optional[float] = None
    azimuth_deg: Optional[float] = None
    elevation_deg: Optional[float] = None
    good_count: int = 0
    sequence_id: Optional[int] = None
    anchor_id: Optional[int] = None
    tag_id: Optional[int] = None
    tag_status: Optional[int] = None
    rx_bytes: int = 0
    heartbeat_count: int = 0
    checksum_errors: int = 0
    length_errors: int = 0
    valid: Optional[bool] = None
    updated_at: Optional[int] = None
    measurement_updated_at: Optional[int] = None
    received_measurement_at: Optional[int] = None
    raw_rejected_at: Optional[int] = None
    require_gate: bool = False
    measurement_status: str = 'legacy'
    filter_reason: Optional[str] = None
    filter_nis: Optional[float] = None
    filter_innovation_m: Optional[float] = None
    filter_uncertainty_m: Optional[float] = None
    filter_recovery_count: int = 0
    filter_admission_version: int = 0
    filter_recovery_diagnostics: Optional[dict] = None
    pending_distance: bool = False
    pending_azimuth: bool = False
    distance_sample_at: Optional[int] = None
    azimuth_sample_at: Optional[int] = None
    rejected_count: int = 0
    invalid_reason: Optional[str] = None

    def complete(self) -> bool:
        return (self.valid is not False and self.distance_m is not None and self.azimuth_deg is not None
                and (not self.require_gate or self.measurement_status == 'trusted'))

    def fresh(self, now_ms: int, timeout_ms: int) -> bool:
        at = self.measurement_updated_at if self.require_gate else self.updated_at
        return bool(self.complete() and at is not None and 0 <= now_ms - at <= timeout_ms)

    def body_azimuth_deg(self) -> Optional[float]:
        if self.azimuth_deg is None:
            return None
        return wrap_angle_deg(self.azimuth_deg + self.mount_yaw_deg)

    def body_position_m(self) -> Optional[Tuple[float, float]]:
        bearing = self.body_azimuth_deg()
        if bearing is None or self.distance_m is None:
            return None
        bearing_rad = math.radians(bearing)
        return (
            self.position_x_m + self.distance_m * math.cos(bearing_rad),
            self.position_y_m + self.distance_m * math.sin(bearing_rad),
        )

    def telemetry(self, now_ms: int, timeout_ms: int) -> dict:
        return {
            "receiverId": self.receiver_id,
            "mountYawDeg": self.mount_yaw_deg,
            "positionXM": self.position_x_m,
            "positionYM": self.position_y_m,
            "sourceSystem": self.source_system,
            "sourceComponent": self.source_component,
            "rawDistanceM": self.raw_distance_m,
            "rawDistanceUpdatedAt": self.distance_sample_at,
            "rawAzimuthDeg": self.raw_azimuth_deg,
            "rawElevationDeg": self.raw_elevation_deg,
            "distanceM": self.distance_m,
            "azimuthDeg": self.azimuth_deg,
            "bodyAzimuthDeg": self.body_azimuth_deg(),
            "elevationDeg": self.elevation_deg,
            "goodCount": self.good_count,
            "sequenceId": self.sequence_id,
            "anchorId": self.anchor_id,
            "tagId": self.tag_id,
            "tagStatus": self.tag_status,
            "rxBytes": self.rx_bytes,
            "heartbeatCount": self.heartbeat_count,
            "checksumErrors": self.checksum_errors,
            "lengthErrors": self.length_errors,
            "valid": self.valid,
            "updatedAt": self.updated_at,
            "measurementUpdatedAt": self.measurement_updated_at,
            "receivedMeasurementAt": self.received_measurement_at,
            "rawRejectedAt": self.raw_rejected_at,
            "measurementStatus": self.measurement_status,
            "filterReason": self.filter_reason,
            "filterNis": self.filter_nis,
            "filterInnovationM": self.filter_innovation_m,
            "filterUncertaintyM": self.filter_uncertainty_m,
            "filterRecoveryCount": self.filter_recovery_count,
            "filterAdmissionVersion": self.filter_admission_version,
            "filterRecoveryDiagnostics": self.filter_recovery_diagnostics,
            "fresh": self.fresh(now_ms, timeout_ms),
            "rejectedCount": self.rejected_count,
            "invalidReason": self.invalid_reason,
        }


class DualUwbFusion:
    """Track two UWB receivers and fuse their bearings using circular averaging.

    Explicit ``UWB1_*``/``UWB2_*`` names are preferred. For backwards
    compatibility, legacy ``UWB_*`` messages are assigned by MAVLink source ID.
    If source IDs are unavailable or identical, they remain receiver 1; the
    flight-controller sender must use suffixed names to distinguish both ports.
    """

    def __init__(
        self,
        mount_yaw_deg: Iterable[float] = (0.0, 90.0),
        freshness_ms: int = 1500,
        receiver_fov_half_deg: float = 70.0,
        overlap_disagreement_deg: float = 30.0,
        overlap_position_disagreement_m: float = 0.45,
        receiver_position_x_m: Iterable[float] = (0.0, 0.0),
        receiver_position_y_m: Iterable[float] = (0.0, 0.0),
        minimum_distance_m: float = 0.15,
        maximum_distance_m: float = 20.0,
        switch_confidence_ratio: float = 1.35,
        switch_confirm_ms: int = 180,
        switch_blend_ms: int = 180,
        enabled_receiver_ids: Iterable[int] = (1, 2),
        require_gated_measurements: bool = False,
    ) -> None:
        mounts = [float(value) for value in mount_yaw_deg]
        positions_x = [float(value) for value in receiver_position_x_m]
        positions_y = [float(value) for value in receiver_position_y_m]
        if len(mounts) != 2:
            raise ValueError("dual UWB needs exactly two mount yaw angles")
        if len(positions_x) != 2 or len(positions_y) != 2:
            raise ValueError("dual UWB needs two receiver position offsets")
        self.receivers = {
            1: UwbReceiver(1, wrap_angle_deg(mounts[0]), positions_x[0], positions_y[0]),
            2: UwbReceiver(2, wrap_angle_deg(mounts[1]), positions_x[1], positions_y[1]),
        }
        for receiver in self.receivers.values():
            receiver.require_gate = require_gated_measurements
            if require_gated_measurements: receiver.measurement_status = "unavailable"
        enabled = {int(value) for value in enabled_receiver_ids}
        if not enabled or not enabled.issubset({1, 2}):
            raise ValueError("enabled_receiver_ids must contain receiver 1 and/or 2")
        self.enabled_receiver_ids = enabled
        self.freshness_ms = max(100, int(freshness_ms))
        self.receiver_fov_half_deg = max(10.0, min(89.0, float(receiver_fov_half_deg)))
        self.overlap_disagreement_deg = max(5.0, min(90.0, float(overlap_disagreement_deg)))
        self.overlap_position_disagreement_m = max(0.1, float(overlap_position_disagreement_m))
        self.minimum_distance_m = max(0.01, float(minimum_distance_m))
        self.maximum_distance_m = max(self.minimum_distance_m + 0.1, float(maximum_distance_m))
        self.switch_confidence_ratio = max(1.0, float(switch_confidence_ratio))
        self.switch_confirm_ms = max(0, int(switch_confirm_ms))
        self.switch_blend_ms = max(0, int(switch_blend_ms))
        self._selected_receiver_id: Optional[int] = None
        self._challenger_receiver_id: Optional[int] = None
        self._challenger_since_ms: Optional[int] = None
        self._blend_from_xy: Optional[Tuple[float, float]] = None
        self._blend_started_ms: Optional[int] = None
        self._last_output_xy: Optional[Tuple[float, float]] = None
        self._legacy_sources: Dict[Tuple[int, int], int] = {}
        # Short raw-sample windows let the outlier selector distinguish a
        # receiver near the centre of its field of view from one producing
        # physically unstable range/bearing estimates.  Twenty samples are
        # about half a second at the vendor receiver's 40 Hz output rate.
        self._distance_history: Dict[int, Deque[float]] = {
            1: deque(maxlen=20),
            2: deque(maxlen=20),
        }
        self._azimuth_history: Dict[int, Deque[float]] = {
            1: deque(maxlen=20),
            2: deque(maxlen=20),
        }

    @staticmethod
    def _median(values: Iterable[float]) -> float:
        ordered = sorted(float(value) for value in values)
        middle = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[middle]
        return (ordered[middle - 1] + ordered[middle]) / 2.0

    def _stability_weight(self, receiver: UwbReceiver) -> float:
        """Return a robust recent-sample quality score in the range [0.01, 1]."""
        distances = self._distance_history[receiver.receiver_id]
        azimuths = self._azimuth_history[receiver.receiver_id]
        if len(distances) < 6 or len(azimuths) < 6:
            return 1.0

        median_distance = self._median(distances)
        distance_mad = self._median(abs(value - median_distance) for value in distances)
        relative_distance_mad = distance_mad / max(0.5, median_distance)
        distance_score = 1.0 / (1.0 + (8.0 * relative_distance_mad) ** 2)

        x = sum(math.cos(math.radians(value)) for value in azimuths)
        y = sum(math.sin(math.radians(value)) for value in azimuths)
        mean_azimuth = math.degrees(math.atan2(y, x))
        azimuth_mad = self._median(
            abs(wrap_angle_deg(value - mean_azimuth)) for value in azimuths
        )
        azimuth_score = 1.0 / (1.0 + (azimuth_mad / 12.0) ** 2)
        return max(0.01, min(1.0, distance_score * azimuth_score))

    def _coverage_weight(self, receiver: UwbReceiver) -> float:
        if receiver.azimuth_deg is None:
            return 0.0
        off_axis_deg = abs(wrap_angle_deg(receiver.azimuth_deg))
        if off_axis_deg > self.receiver_fov_half_deg:
            return 0.0
        normalized = off_axis_deg / self.receiver_fov_half_deg
        return max(0.05, math.cos(normalized * math.pi / 2.0) ** 2)

    def _select_outlier_receiver(
        self,
        candidates: list[Tuple[UwbReceiver, float]],
        now_ms: int,
    ) -> Tuple[UwbReceiver, float]:
        by_id = {receiver.receiver_id: (receiver, weight) for receiver, weight in candidates}
        best = max(candidates, key=lambda item: (item[1], item[0].updated_at or 0))
        selected = by_id.get(self._selected_receiver_id)
        if selected is None:
            self._selected_receiver_id = best[0].receiver_id
            self._challenger_receiver_id = None
            self._challenger_since_ms = None
            return best
        if best[0].receiver_id == selected[0].receiver_id:
            self._challenger_receiver_id = None
            self._challenger_since_ms = None
            return selected
        if best[1] < selected[1] * self.switch_confidence_ratio:
            self._challenger_receiver_id = None
            self._challenger_since_ms = None
            return selected
        if self._challenger_receiver_id != best[0].receiver_id:
            self._challenger_receiver_id = best[0].receiver_id
            self._challenger_since_ms = now_ms
            return selected
        if self._challenger_since_ms is None or now_ms - self._challenger_since_ms < self.switch_confirm_ms:
            return selected
        self._blend_from_xy = self._last_output_xy
        self._blend_started_ms = now_ms
        self._selected_receiver_id = best[0].receiver_id
        self._challenger_receiver_id = None
        self._challenger_since_ms = None
        return best

    @staticmethod
    def parse_name(name: str) -> Optional[Tuple[Optional[int], str]]:
        normalized = str(name).strip().upper()
        if normalized in LEGACY_UWB_NAMES:
            return None, normalized.removeprefix("UWB_")
        for pattern in UWB_NAME_PATTERNS:
            match = pattern.match(normalized)
            if not match:
                continue
            first, second = match.groups()
            if first in {"1", "2"}:
                return int(first), second
            return int(second), first
        return None

    def _legacy_receiver(self, source: Optional[Tuple[int, int]]) -> int:
        if source is None:
            return 1
        source = (int(source[0]), int(source[1]))
        if source in self._legacy_sources:
            return self._legacy_sources[source]
        if len(self._legacy_sources) < 2:
            receiver_id = len(self._legacy_sources) + 1
            self._legacy_sources[source] = receiver_id
            receiver = self.receivers[receiver_id]
            receiver.source_system, receiver.source_component = source
            return receiver_id
        return 1

    def update(
        self,
        name: str,
        value: float,
        *,
        source: Optional[Tuple[int, int]] = None,
        now_ms: Optional[int] = None,
    ) -> Optional[Tuple[int, str]]:
        parsed = self.parse_name(name)
        if parsed is None or not math.isfinite(value):
            return None
        explicit_receiver, field_name = parsed
        receiver_id = explicit_receiver or self._legacy_receiver(source)
        receiver = self.receivers[receiver_id]
        if source is not None:
            receiver.source_system, receiver.source_component = int(source[0]), int(source[1])
        if field_name == 'DIST':
            receiver.distance_sample_at = int(time.time() * 1000) if now_ms is None else int(now_ms)

        if field_name == "GOOD":
            receiver.good_count = max(0, int(value))
            return receiver_id, field_name
        if field_name == "VALID":
            receiver.valid = value > 0
            if not receiver.valid:
                receiver.pending_distance = receiver.pending_azimuth = False
            receiver.updated_at = int(time.time() * 1000) if now_ms is None else int(now_ms)
            return receiver_id, field_name
        if field_name == "SEQ":
            receiver.sequence_id = max(0, int(value))
            return receiver_id, field_name
        if field_name == "ANCH":
            receiver.anchor_id = max(0, int(value))
            return receiver_id, field_name
        if field_name == "TAG":
            receiver.tag_id = max(0, int(value))
            return receiver_id, field_name
        if field_name == "STAT":
            receiver.tag_status = max(0, int(value))
            return receiver_id, field_name
        if field_name == "RX":
            receiver.rx_bytes = max(0, int(value))
            return receiver_id, field_name
        if field_name == "HB":
            receiver.heartbeat_count = max(0, int(value))
            return receiver_id, field_name
        if field_name == "CRC":
            receiver.checksum_errors = max(0, int(value))
            return receiver_id, field_name
        if field_name == "LEN":
            receiver.length_errors = max(0, int(value))
            return receiver_id, field_name
        if field_name == "DIST" and not self.minimum_distance_m <= value <= self.maximum_distance_m:
            receiver.raw_distance_m = float(value)
            receiver.valid = False
            receiver.invalid_reason = "distance_out_of_range"
            receiver.pending_distance = receiver.pending_azimuth = False
            receiver.rejected_count += 1
            receiver.updated_at = int(time.time() * 1000) if now_ms is None else int(now_ms)
            receiver.raw_rejected_at = receiver.updated_at
            return receiver_id, "REJECTED"

        if receiver.require_gate and field_name in ('AZ', 'EL') and abs(value) > (180 if field_name == 'AZ' else 90):
            if field_name == 'AZ': receiver.raw_azimuth_deg = float(value)
            else: receiver.raw_elevation_deg = float(value)
            receiver.invalid_reason = 'azimuth_out_of_range' if field_name == 'AZ' else 'elevation_out_of_range'
            receiver.pending_distance = receiver.pending_azimuth = False
            receiver.rejected_count += 1
            receiver.updated_at = int(time.time() * 1000) if now_ms is None else int(now_ms)
            if field_name == 'AZ': receiver.raw_rejected_at = receiver.updated_at
            return receiver_id, 'REJECTED'

        if field_name == "DIST":
            receiver.invalid_reason = None
            receiver.raw_distance_m = float(value)
            receiver.pending_distance = True
            if not receiver.require_gate: self._distance_history[receiver_id].append(float(value))
        elif field_name == "AZ":
            receiver.raw_azimuth_deg = wrap_angle_deg(value)
            receiver.pending_azimuth = True
            if not receiver.require_gate: self._azimuth_history[receiver_id].append(wrap_angle_deg(value))
        elif field_name == "EL":
            receiver.raw_elevation_deg = float(value)
        receiver.updated_at = int(time.time() * 1000) if now_ms is None else int(now_ms)
        if field_name == 'DIST':
            receiver.distance_sample_at = receiver.updated_at
        elif field_name == 'AZ':
            receiver.azimuth_sample_at = receiver.updated_at
        if receiver.pending_distance and receiver.pending_azimuth:
            if abs(receiver.distance_sample_at - receiver.azimuth_sample_at) <= 100:
                receiver.received_measurement_at = max(receiver.distance_sample_at, receiver.azimuth_sample_at)
                if not receiver.require_gate: receiver.measurement_updated_at = receiver.received_measurement_at
                receiver.pending_distance = receiver.pending_azimuth = False
            elif receiver.distance_sample_at < receiver.azimuth_sample_at:
                receiver.pending_distance = False
            else:
                receiver.pending_azimuth = False
        return receiver_id, field_name

    def apply_measurement_result(self, receiver_id: int, result: dict) -> None:
        receiver = self.receivers[int(receiver_id)]
        if result.get('duplicate'): return
        receiver.measurement_status = result['status']
        receiver.filter_reason = result['reason']
        receiver.filter_nis = result.get('nis')
        receiver.filter_innovation_m = result.get('innovation_m')
        receiver.filter_uncertainty_m = result.get('uncertainty_m')
        receiver.filter_recovery_count = result.get('recovery_count', 0)
        receiver.filter_admission_version = result.get('admission_version', 0)
        receiver.filter_recovery_diagnostics = result.get('recovery_diagnostics')
        receiver.rejected_count = max(receiver.rejected_count, result.get('rejected_count', 0))
        if result.get('accepted'):
            self.set_filtered(receiver_id, 'DIST', result['distance_m'])
            self.set_filtered(receiver_id, 'AZ', result['azimuth_deg'])
            receiver.measurement_updated_at = result['accepted_at']
            receiver.invalid_reason = None
            self._distance_history[receiver_id].append(receiver.raw_distance_m)
            self._azimuth_history[receiver_id].append(receiver.raw_azimuth_deg)
        else:
            receiver.invalid_reason = result['reason']

    def set_filtered(self, receiver_id: int, field_name: str, value: float) -> None:
        receiver = self.receivers[int(receiver_id)]
        if field_name == "DIST":
            receiver.distance_m = min(self.maximum_distance_m, max(self.minimum_distance_m, float(value)))
        elif field_name == "AZ":
            receiver.azimuth_deg = wrap_angle_deg(value)
        elif field_name == "EL":
            receiver.elevation_deg = float(value)

    def fused(self, now_ms: Optional[int] = None) -> dict:
        now_ms = int(time.time() * 1000) if now_ms is None else int(now_ms)
        candidates = [
            (
                receiver,
                self._coverage_weight(receiver) * self._stability_weight(receiver),
            )
            for receiver in self.receivers.values()
            if receiver.receiver_id in self.enabled_receiver_ids
            if receiver.fresh(now_ms, self.freshness_ms)
        ]
        candidates = [(receiver, weight) for receiver, weight in candidates if weight > 0]
        sensors = [
            self.receivers[index].telemetry(now_ms, self.freshness_ms)
            for index in sorted(self.enabled_receiver_ids)
        ]
        for sensor in sensors:
            receiver = self.receivers[sensor["receiverId"]]
            sensor["stabilityWeight"] = self._stability_weight(receiver)
            sensor["fusionWeight"] = self._coverage_weight(receiver) * sensor["stabilityWeight"]
        if not candidates:
            return {
                "online": False,
                "fresh": False,
                "distance_m": None,
                "azimuth_deg": None,
                "elevation_deg": None,
                "good_count": 0,
                "updated_at": None,
                "receiver_count": 0,
                "observing_receiver_count": 0,
                "active_receiver_ids": [],
                "fusion_mode": "no_valid_receiver",
                "bearing_disagreement_deg": None,
                "position_disagreement_m": None,
                "relative_x_m": None,
                "relative_y_m": None,
                "sensors": sensors,
            }

        observing_count = len(candidates)
        bearing_disagreement_deg = None
        position_disagreement_m = None
        fusion_mode = "single_receiver"
        if len(candidates) == 1:
            receiver_id = candidates[0][0].receiver_id
            if self._selected_receiver_id is not None and self._selected_receiver_id != receiver_id:
                self._blend_from_xy = self._last_output_xy
                self._blend_started_ms = now_ms
            self._selected_receiver_id = receiver_id
            self._challenger_receiver_id = None
            self._challenger_since_ms = None
        if len(candidates) == 2:
            first_bearing = candidates[0][0].body_azimuth_deg()
            second_bearing = candidates[1][0].body_azimuth_deg()
            bearing_disagreement_deg = abs(wrap_angle_deg(first_bearing - second_bearing))
            first_point = candidates[0][0].body_position_m()
            second_point = candidates[1][0].body_position_m()
            position_disagreement_m = math.hypot(
                first_point[0] - second_point[0],
                first_point[1] - second_point[1],
            )
            near_range = min(candidates[0][0].distance_m, candidates[1][0].distance_m)
            allowed_position_disagreement = max(
                self.overlap_position_disagreement_m,
                0.3 * near_range,
            )
            if (
                bearing_disagreement_deg > self.overlap_disagreement_deg
                or position_disagreement_m > allowed_position_disagreement
            ):
                candidates = [self._select_outlier_receiver(candidates, now_ms)]
                fusion_mode = "hysteretic_receiver_select"
            else:
                self._selected_receiver_id = None
                self._challenger_receiver_id = None
                self._challenger_since_ms = None
                fusion_mode = "overlap_weighted_fusion"

        active = [receiver for receiver, _ in candidates]
        weights = [weight for _, weight in candidates]
        weight_sum = sum(weights)
        points = [receiver.body_position_m() for receiver in active]
        x = sum(weight * point[0] for weight, point in zip(weights, points)) / weight_sum
        y = sum(weight * point[1] for weight, point in zip(weights, points)) / weight_sum
        if self._blend_from_xy is not None and self._blend_started_ms is not None and self.switch_blend_ms > 0:
            blend = min(1.0, max(0.0, (now_ms - self._blend_started_ms) / self.switch_blend_ms))
            x = self._blend_from_xy[0] + blend * (x - self._blend_from_xy[0])
            y = self._blend_from_xy[1] + blend * (y - self._blend_from_xy[1])
            if blend >= 1.0:
                self._blend_from_xy = None
                self._blend_started_ms = None
        self._last_output_xy = (x, y)
        distance = math.hypot(x, y)
        azimuth = math.degrees(math.atan2(y, x))
        elevations = [
            (weight, receiver.elevation_deg)
            for weight, receiver in zip(weights, active)
            if receiver.elevation_deg is not None
        ]
        elevation = None
        if elevations:
            elevation = sum(weight * value for weight, value in elevations) / sum(weight for weight, _ in elevations)
        return {
            "online": True,
            "fresh": True,
            "distance_m": distance,
            "azimuth_deg": wrap_angle_deg(azimuth),
            "elevation_deg": elevation,
            "good_count": sum(receiver.good_count for receiver in active),
            "updated_at": max(receiver.updated_at for receiver in active),
            "receiver_count": len(active),
            "observing_receiver_count": observing_count,
            "active_receiver_ids": [receiver.receiver_id for receiver in active],
            "fusion_mode": fusion_mode,
            "bearing_disagreement_deg": bearing_disagreement_deg,
            "position_disagreement_m": position_disagreement_m,
            "relative_x_m": x,
            "relative_y_m": y,
            "sensors": sensors,
        }
