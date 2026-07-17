"""Localization-only readiness checks for patrol replay evidence."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from itertools import pairwise
from typing import Any

PASS = "PASS"
FAIL = "FAIL"
INCOMPLETE = "INCOMPLETE"
LOCALIZATION_PASS = "LOCALIZATION_PASS"
LOCALIZATION_FAIL = "LOCALIZATION_FAIL"
LOCALIZATION_INCOMPLETE = "LOCALIZATION_INCOMPLETE"

_SCOPED_STATUS = {
    PASS: LOCALIZATION_PASS,
    FAIL: LOCALIZATION_FAIL,
    INCOMPLETE: LOCALIZATION_INCOMPLETE,
}

UNPROVEN_MOTION_SURFACES = (
    "global_planner",
    "local_planner",
    "path_follower",
    "driver",
    "motor",
    "field_thermal",
    "camera_action",
)


@dataclass(frozen=True)
class EvidenceWindow:
    start_s: float
    end_s: float

    def validate(self, label: str) -> None:
        if not math.isfinite(self.start_s) or not math.isfinite(self.end_s):
            raise ValueError(f"{label} bounds must be finite")
        if self.end_s <= self.start_s:
            raise ValueError(f"{label}.end_s must be greater than start_s")

    def contains(self, stamp_s: float) -> bool:
        return self.start_s <= stamp_s <= self.end_s


@dataclass(frozen=True)
class RelocalizationRecoveryWindow:
    start_s: float
    end_s: float
    min_stable_s: float
    initial_offset_m: float = 0.0
    initial_yaw_offset_rad: float = 0.0
    expected_map_sha256: str = ""
    observed_map_sha256: str = ""

    def validate(self, label: str) -> None:
        EvidenceWindow(self.start_s, self.end_s).validate(label)
        if not math.isfinite(self.min_stable_s) or self.min_stable_s <= 0.0:
            raise ValueError(f"{label}.min_stable_s must be positive")
        for field, value in (
            ("initial_offset_m", self.initial_offset_m),
            ("initial_yaw_offset_rad", self.initial_yaw_offset_rad),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{label}.{field} must be finite and non-negative")

    def contains(self, stamp_s: float) -> bool:
        return self.start_s <= stamp_s <= self.end_s


@dataclass(frozen=True)
class InspectionLocalizationReadinessConfig:
    min_status_samples: int = 20
    min_stationary_duration_s: float = 60.0
    max_sequence_gap: int = 1
    min_degeneracy_coverage: float = 0.95
    min_annotated_degeneracy_detection_rate: float = 0.80
    max_stationary_yaw_drift_deg_per_min: float = 0.50

    def validate(self) -> None:
        if self.min_status_samples < 2:
            raise ValueError("min_status_samples must be at least 2")
        if not math.isfinite(self.min_stationary_duration_s) or self.min_stationary_duration_s <= 0.0:
            raise ValueError("min_stationary_duration_s must be positive")
        if self.max_sequence_gap < 1:
            raise ValueError("max_sequence_gap must be at least 1")
        for label, value in (
            ("min_degeneracy_coverage", self.min_degeneracy_coverage),
            (
                "min_annotated_degeneracy_detection_rate",
                self.min_annotated_degeneracy_detection_rate,
            ),
        ):
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{label} must be in [0, 1]")
        drift = self.max_stationary_yaw_drift_deg_per_min
        if not math.isfinite(drift) or drift < 0.0:
            raise ValueError("max_stationary_yaw_drift_deg_per_min must be non-negative")


def evaluate_inspection_localization_readiness(
    samples: Iterable[Mapping[str, Any]],
    *,
    evidence_window: EvidenceWindow,
    stationary_window: EvidenceWindow | None = None,
    annotated_degeneracy_windows: Iterable[EvidenceWindow] = (),
    relocalization_windows: Iterable[RelocalizationRecoveryWindow] = (),
    config: InspectionLocalizationReadinessConfig | None = None,
) -> dict[str, Any]:
    """Return PASS/FAIL/INCOMPLETE for localization evidence only."""

    cfg = config or InspectionLocalizationReadinessConfig()
    cfg.validate()
    evidence_window.validate("evidence_window")
    if stationary_window is not None:
        stationary_window.validate("stationary_window")
    deg_windows = tuple(annotated_degeneracy_windows)
    for index, window in enumerate(deg_windows):
        window.validate(f"annotated_degeneracy_windows[{index}]")
    reloc_windows = tuple(relocalization_windows)
    for index, window in enumerate(reloc_windows):
        window.validate(f"relocalization_windows[{index}]")

    normalized = [_normalize_sample(sample) for sample in samples if evidence_window.contains(_stamp(sample))]
    checks = {
        "continuity": _continuity(normalized, cfg, reloc_windows),
        "degeneracy_diagnostics": _degeneracy_check(
            normalized,
            cfg,
            deg_windows,
        ),
        "stationary_yaw_drift": _stationary_yaw(
            normalized,
            cfg,
            stationary_window,
        ),
        "relocalization_recovery": _relocalization(normalized, reloc_windows),
    }

    blockers: list[str] = []
    incomplete: list[str] = []
    for name, check in checks.items():
        if check["status"] == FAIL:
            blockers.extend(f"{name}: {reason}" for reason in check["reasons"])
        elif check["status"] == INCOMPLETE:
            incomplete.extend(f"{name}: {reason}" for reason in check["reasons"])

    localization_status = FAIL if blockers else INCOMPLETE if incomplete else PASS
    return {
        "schema_version": "lingtu.inspection.localization_readiness.v2",
        "scope": "localization_only",
        "status": _SCOPED_STATUS[localization_status],
        "localization_status": localization_status,
        "motion_authorization": False,
        "evidence_window": {
            "start_s": evidence_window.start_s,
            "end_s": evidence_window.end_s,
            "samples": len(normalized),
        },
        "checks": checks,
        "blockers": blockers,
        "incomplete": incomplete,
        "patrol_readiness": {
            "status": INCOMPLETE,
            "reason": "motion_surfaces_not_evaluated",
            "unproven_surfaces": list(UNPROVEN_MOTION_SURFACES),
        },
        "does_not_prove": list(UNPROVEN_MOTION_SURFACES),
    }


def _stamp(sample: Mapping[str, Any]) -> float:
    return _finite(sample.get("stamp_s", sample.get("timestamp_s")), "stamp_s")


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be numeric")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be numeric") from exc
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def _int_or_none(value: Any) -> int | None:
    if isinstance(value, bool) or value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _pose_from_sample(sample: Mapping[str, Any]) -> Mapping[str, Any] | None:
    pose = sample.get("pose")
    if isinstance(pose, Mapping):
        return pose
    odometry = sample.get("odometry")
    if isinstance(odometry, Mapping):
        nested_pose = odometry.get("pose")
        if isinstance(nested_pose, Mapping):
            return nested_pose
    return None


def _normalize_sample(sample: Mapping[str, Any]) -> dict[str, Any]:
    pose = _pose_from_sample(sample)
    return {
        "stamp_s": _stamp(sample),
        "runtime_instance_id": str(sample.get("runtime_instance_id") or ""),
        "state": str(sample.get("state") or "").upper(),
        "reported_state": str(sample.get("reported_state") or "").upper(),
        "observation_sequence": _int_or_none(sample.get("observation_sequence")),
        "map_frame_jump": bool(sample.get("map_frame_jump", False)),
        "dropped_lidar_frames": _int_or_none(sample.get("dropped_lidar_frames")),
        "dropped_imu_frames": _int_or_none(sample.get("dropped_imu_frames")),
        "imu_rollback_count": _int_or_none(sample.get("imu_rollback_count")),
        "lidar_rollback_count": _int_or_none(sample.get("lidar_rollback_count")),
        "pose": pose,
        "fastlio_degeneracy": _degeneracy(sample),
        "relocalization_state": str(sample.get("relocalization_state") or "").upper(),
    }


def _degeneracy(sample: Mapping[str, Any]) -> dict[str, bool] | None:
    raw = sample.get("fastlio_degeneracy")
    if isinstance(raw, Mapping):
        return {"detected": bool(raw.get("detected"))}
    if "degeneracy_detected" in sample:
        return {"detected": bool(sample.get("degeneracy_detected"))}
    return None


def _pose_finite(pose: Mapping[str, Any] | None) -> bool:
    if not isinstance(pose, Mapping):
        return False
    for field in ("x", "y", "z", "qx", "qy", "qz", "qw"):
        try:
            _finite(pose.get(field), f"pose.{field}")
        except ValueError:
            return False
    return True


def _result(status: str, reasons: list[str], **metrics: Any) -> dict[str, Any]:
    return {"status": status, "reasons": reasons, "metrics": metrics}


def _continuity(
    samples: list[dict[str, Any]],
    cfg: InspectionLocalizationReadinessConfig,
    relocalization_windows: tuple[RelocalizationRecoveryWindow, ...],
) -> dict[str, Any]:
    if not samples:
        return _result(INCOMPLETE, ["no status samples in evidence window"])

    failures: list[str] = []
    incomplete: list[str] = []
    if len(samples) < cfg.min_status_samples:
        incomplete.append("status sample count below minimum")

    runtime_ids = {item["runtime_instance_id"] for item in samples if item["runtime_instance_id"]}
    if not runtime_ids:
        incomplete.append("runtime_instance_id missing")
    elif len(runtime_ids) > 1:
        failures.append("multiple runtime_instance_id values observed")

    sequence_drops = 0
    sequence_rollbacks = 0
    status_timestamp_rollbacks = 0
    previous_sequence: int | None = None
    previous_stamp_s: float | None = None
    for item in samples:
        in_expected_relocalization = any(window.contains(item["stamp_s"]) for window in relocalization_windows)
        states = {item["state"], item["reported_state"]} - {""}
        if not states:
            incomplete.append("localization state missing")
        if states & {"DIVERGED", "FAILED"}:
            failures.append("failed localization states observed")
        elif any(state != "TRACKING" for state in states):
            allowed_recovery_states = {"LOST", "RELOCALIZING", "SEARCHING"}
            if not (in_expected_relocalization and states <= allowed_recovery_states | {"TRACKING"}):
                failures.append("non-tracking localization states observed")

        if item["map_frame_jump"]:
            failures.append("map_frame_jump observed")
        if not _pose_finite(item["pose"]):
            failures.append("non-finite or missing pose observed")

        sequence = item["observation_sequence"]
        if previous_stamp_s is not None and item["stamp_s"] < previous_stamp_s:
            status_timestamp_rollbacks += 1
        previous_stamp_s = item["stamp_s"]

        if sequence is None:
            incomplete.append("observation_sequence missing")
        elif previous_sequence is not None:
            if sequence < previous_sequence:
                sequence_rollbacks += 1
            elif sequence - previous_sequence > cfg.max_sequence_gap:
                sequence_drops += sequence - previous_sequence - 1
        if sequence is not None:
            previous_sequence = sequence

    if sequence_rollbacks:
        failures.append("observation_sequence rollback observed")
    if sequence_drops:
        failures.append("observation_sequence gaps exceed threshold")
    if status_timestamp_rollbacks:
        failures.append("status timestamp rollback observed")

    counter_names = (
        "dropped_lidar_frames",
        "dropped_imu_frames",
        "imu_rollback_count",
        "lidar_rollback_count",
    )
    counters = {name: [item[name] for item in samples if item[name] is not None] for name in counter_names}
    if any(len(values) != len(samples) for values in counters.values()):
        incomplete.append("sensor transport counters missing")
    if any(any(value < 0 for value in values) for values in counters.values()):
        failures.append("sensor transport counters invalid")

    counter_increases = {
        name: sum(max(0, current - previous) for previous, current in pairwise(values))
        for name, values in counters.items()
    }
    counter_rollbacks = sum(
        current < previous for values in counters.values() for previous, current in pairwise(values)
    )
    sensor_drop_delta = counter_increases["dropped_lidar_frames"] + counter_increases["dropped_imu_frames"]
    sensor_rollback_delta = counter_increases["imu_rollback_count"] + counter_increases["lidar_rollback_count"]
    if sensor_drop_delta > 0:
        failures.append("sensor frame drops observed")
    if sensor_rollback_delta > 0:
        failures.append("sensor timestamp rollback observed")
    if counter_rollbacks:
        failures.append("sensor transport counter rollback observed")

    status = FAIL if failures else INCOMPLETE if incomplete else PASS
    reasons = failures if failures else incomplete
    return _result(
        status,
        sorted(set(reasons)),
        samples=len(samples),
        runtime_ids=sorted(runtime_ids),
        sequence_drop_count=sequence_drops,
        sequence_rollback_count=sequence_rollbacks,
        status_timestamp_rollback_count=status_timestamp_rollbacks,
        sensor_drop_delta=sensor_drop_delta,
        sensor_rollback_delta=sensor_rollback_delta,
        sensor_counter_rollback_count=counter_rollbacks,
    )


def _degeneracy_check(
    samples: list[dict[str, Any]],
    cfg: InspectionLocalizationReadinessConfig,
    annotated_windows: tuple[EvidenceWindow, ...],
) -> dict[str, Any]:
    if not samples:
        return _result(INCOMPLETE, ["no status samples in evidence window"])
    with_diag = [item for item in samples if item["fastlio_degeneracy"] is not None]
    coverage = len(with_diag) / len(samples)
    reasons: list[str] = []
    if coverage < cfg.min_degeneracy_coverage:
        reasons.append("fastlio degeneracy diagnostics coverage too low")
    annotated_rate: float | None = None
    if annotated_windows:
        expected = [item for item in samples if any(window.contains(item["stamp_s"]) for window in annotated_windows)]
        if not expected:
            return _result(
                INCOMPLETE,
                ["annotated degeneracy windows contain no samples"],
                coverage=coverage,
            )
        actual = [item for item in expected if item["fastlio_degeneracy"] is not None]
        if not actual:
            return _result(
                INCOMPLETE,
                ["annotated degeneracy windows have no diagnostic samples"],
                coverage=coverage,
            )
        if len(actual) != len(expected):
            reasons.append("annotated degeneracy window diagnostics missing")
        hits = sum(1 for item in actual if item["fastlio_degeneracy"]["detected"])
        annotated_rate = hits / len(actual)
        if annotated_rate < cfg.min_annotated_degeneracy_detection_rate:
            reasons.append("annotated degeneracy detection rate too low")
    return _result(
        FAIL if reasons else PASS,
        reasons,
        coverage=coverage,
        annotated_detection_rate=annotated_rate,
    )


def _stationary_yaw(
    samples: list[dict[str, Any]],
    cfg: InspectionLocalizationReadinessConfig,
    window: EvidenceWindow | None,
) -> dict[str, Any]:
    if window is None:
        return _result(INCOMPLETE, ["stationary_window is required"])
    selected = [item for item in samples if window.contains(item["stamp_s"])]
    if len(selected) < 2:
        return _result(INCOMPLETE, ["stationary_window needs at least two samples"])
    duration_s = selected[-1]["stamp_s"] - selected[0]["stamp_s"]
    if duration_s < cfg.min_stationary_duration_s:
        return _result(
            INCOMPLETE,
            ["stationary_window duration below minimum"],
            duration_s=duration_s,
            samples=len(selected),
        )
    points: list[tuple[float, float]] = []
    for item in selected:
        pose = item["pose"]
        if not isinstance(pose, Mapping) or not _pose_finite(pose):
            return _result(INCOMPLETE, ["stationary_window pose evidence missing"])
        points.append(
            (
                item["stamp_s"],
                _yaw(
                    _finite(pose.get("qx"), "pose.qx"),
                    _finite(pose.get("qy"), "pose.qy"),
                    _finite(pose.get("qz"), "pose.qz"),
                    _finite(pose.get("qw"), "pose.qw"),
                ),
            )
        )
    slope = _linear_slope(_unwrap(points))
    drift = abs(math.degrees(slope) * 60.0)
    reasons = ["stationary yaw drift exceeds threshold"] if drift > cfg.max_stationary_yaw_drift_deg_per_min else []
    return _result(
        FAIL if reasons else PASS,
        reasons,
        drift_deg_per_min=drift,
        samples=len(points),
        duration_s=duration_s,
    )


def _yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        raise ValueError("quaternion norm must be positive")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _unwrap(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    output: list[tuple[float, float]] = []
    offset = 0.0
    previous: float | None = None
    for stamp_s, yaw in points:
        adjusted = yaw + offset
        if previous is not None:
            while adjusted - previous > math.pi:
                offset -= 2.0 * math.pi
                adjusted = yaw + offset
            while adjusted - previous < -math.pi:
                offset += 2.0 * math.pi
                adjusted = yaw + offset
        output.append((stamp_s, adjusted))
        previous = adjusted
    return output


def _linear_slope(points: list[tuple[float, float]]) -> float:
    mean_t = sum(stamp_s for stamp_s, _ in points) / len(points)
    mean_yaw = sum(yaw for _, yaw in points) / len(points)
    denom = sum((stamp_s - mean_t) ** 2 for stamp_s, _ in points)
    if denom <= 0.0:
        return 0.0
    return sum((stamp_s - mean_t) * (yaw - mean_yaw) for stamp_s, yaw in points) / denom


def _relocalization(
    samples: list[dict[str, Any]],
    windows: tuple[RelocalizationRecoveryWindow, ...],
) -> dict[str, Any]:
    if not windows:
        return _result(PASS, [], windows=0)
    if not samples:
        return _result(INCOMPLETE, ["no status samples in evidence window"])

    failures: list[str] = []
    incomplete: list[str] = []
    recoveries: list[dict[str, Any]] = []
    stable_states = {"", "TRACKING", "LOCKED", "IDLE", "OK"}
    for index, window in enumerate(windows):
        selected = [item for item in samples if window.contains(item["stamp_s"])]
        if not selected:
            incomplete.append(f"relocalization window {index} has no samples")
            recoveries.append({"index": index, "stable_s": 0.0})
            continue

        stable_s = _stable_tracking_tail_s(selected)
        recoveries.append({"index": index, "stable_s": stable_s})
        if not window.expected_map_sha256 or not window.observed_map_sha256:
            incomplete.append(f"relocalization window {index} map identity evidence is required")
        elif window.expected_map_sha256 != window.observed_map_sha256:
            failures.append(f"relocalization window {index} map identity mismatch")

        if window.initial_offset_m <= 1e-9 and window.initial_yaw_offset_rad <= 1e-9:
            incomplete.append(f"relocalization window {index} non-zero initial perturbation is required")

        saw_recovery = any(
            item["state"] not in {"", "TRACKING"}
            or item["reported_state"] not in {"", "TRACKING"}
            or (item["relocalization_state"] and item["relocalization_state"] not in stable_states)
            for item in selected
        )
        if not saw_recovery:
            incomplete.append(f"relocalization window {index} has no observed recovery transition")
        if stable_s < window.min_stable_s:
            failures.append(f"relocalization window {index} lacks stable tracking tail")

    status = FAIL if failures else INCOMPLETE if incomplete else PASS
    reasons = failures if failures else incomplete
    return _result(status, reasons, recoveries=recoveries)


def _stable_tracking_tail_s(samples: list[dict[str, Any]]) -> float:
    end_s = samples[-1]["stamp_s"]
    start_s = end_s
    for item in reversed(samples):
        if item["state"] != "TRACKING" or (item["reported_state"] and item["reported_state"] != "TRACKING"):
            break
        relocalization_state = item["relocalization_state"]
        if relocalization_state and relocalization_state not in {
            "TRACKING",
            "LOCKED",
            "IDLE",
            "OK",
        }:
            break
        start_s = item["stamp_s"]
    return max(0.0, end_s - start_s)
