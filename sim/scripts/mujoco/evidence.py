"""MuJoCo motion acceptance evidence and feeder rate statistics."""

from __future__ import annotations

import json
import math
import os
import secrets
from collections.abc import Mapping
from pathlib import Path
from typing import Any, Callable

from lingtu.sim.stop import process_launch_id
from lingtu.switch_contracts import is_product_session_id

MOTION_EVIDENCE_SCHEMA = "lingtu.sim.motion_evidence.v2"
MOTION_EVIDENCE_FILENAME = "mujoco_feeder.motion.json"
MIN_BASE_HEIGHT_M = 0.20
MAX_BASE_HEIGHT_M = 1.00
MAX_BASE_HEIGHT_SPAN_M = 0.35
MAX_ABS_TILT_RAD = math.radians(30.0)
_MOTION_FIELDS = frozenset(
    {
        "schema",
        "product_session_id",
        "product",
        "process",
        "launch_id",
        "bridge_boot_id",
        "controller_boot_id",
        "terminal_bridge_command_seq",
        "terminal_applied_step_seq",
        "commanded_motion_observed",
        "nonzero_command_count",
        "nonzero_physics_steps",
        "first_bridge_command_seq",
        "last_bridge_command_seq",
        "first_motion_step_seq",
        "last_motion_step_seq",
        "first_producer_boot_id",
        "last_producer_boot_id",
        "first_output_sequence",
        "last_output_sequence",
        "start_position_m",
        "end_position_m",
        "net_displacement_xy_m",
        "path_length_xy_m",
        "pose_sample_count",
        "min_base_height_m",
        "max_base_height_m",
        "max_abs_roll_rad",
        "max_abs_pitch_rad",
        "start_yaw_rad",
        "end_yaw_rad",
        "trajectory",
    }
)

FEEDER_STATUS_FILENAME = "mujoco_feeder.status.json"
FEEDER_STATUS_SCHEMA = "lingtu.sim.feeder_status.v1"
_FEEDER_FIELDS = frozenset(
    {
        "schema",
        "product",
        "product_session_id",
        "process",
        "launch_id",
        "state",
        "sequence",
        "updated_wall_ns",
        "window_s",
        "streams",
    }
)
_STREAM_FIELDS = frozenset(
    {
        "expected_hz",
        "scheduled_count",
        "published_count",
        "dropped_count",
        "actual_hz",
        "max_schedule_lateness_ms",
    }
)
_STREAM_NAMES = frozenset({"imu", "lidar", "camera_rgbd"})
_FEEDER_STATES = frozenset({"running", "stopped", "failed"})


class SimMotionEvidenceError(RuntimeError):
    """Physical motion evidence failed its acceptance contract."""


class SimFeederStatusError(RuntimeError):
    """Feeder rate statistics failed their acceptance contract."""


def publish_motion_evidence(
    *,
    session_root: Path,
    payload: Mapping[str, Any],
    environment: Mapping[str, str] | None = None,
    bound_publisher: Callable[[str, bytes], Path] | None = None,
) -> Mapping[str, Any]:
    complete = _with_launch_id(payload, _MOTION_FIELDS, environment, SimMotionEvidenceError)
    _validate_motion(
        complete,
        product_session_id=complete.get("product_session_id"),
        product=complete.get("product"),
        process=complete.get("process"),
        launch_id=complete["launch_id"],
    )
    _publish(
        session_root / MOTION_EVIDENCE_FILENAME,
        MOTION_EVIDENCE_FILENAME,
        complete,
        bound_publisher,
        SimMotionEvidenceError,
        "motion evidence",
    )
    return complete


def load_motion_evidence(
    *,
    session_root: Path,
    product_session_id: str,
    product: str,
    process: str,
    launch_id: str,
) -> Mapping[str, Any]:
    payload = _load(
        session_root / MOTION_EVIDENCE_FILENAME,
        65536,
        SimMotionEvidenceError,
        "motion evidence",
    )
    _validate_motion(
        payload,
        product_session_id=product_session_id,
        product=product,
        process=process,
        launch_id=launch_id,
    )
    return payload


def publish_feeder_status(
    *,
    session_root: Path,
    payload: Mapping[str, Any],
    environment: Mapping[str, str] | None = None,
    bound_publisher: Callable[[str, bytes], object] | None = None,
) -> Mapping[str, Any]:
    complete = _with_launch_id(payload, _FEEDER_FIELDS, environment, SimFeederStatusError)
    _validate_feeder(
        complete,
        product=complete.get("product"),
        product_session_id=complete.get("product_session_id"),
        process=complete.get("process"),
        launch_id=complete["launch_id"],
    )
    _publish(
        session_root / FEEDER_STATUS_FILENAME,
        FEEDER_STATUS_FILENAME,
        complete,
        bound_publisher,
        SimFeederStatusError,
        "feeder status",
    )
    return complete


def load_feeder_status(
    *,
    session_root: Path,
    product: str,
    product_session_id: str,
    process: str,
    launch_id: str,
) -> Mapping[str, Any]:
    payload = _load(
        session_root / FEEDER_STATUS_FILENAME,
        8192,
        SimFeederStatusError,
        "feeder status",
    )
    _validate_feeder(
        payload,
        product=product,
        product_session_id=product_session_id,
        process=process,
        launch_id=launch_id,
    )
    return payload


def _with_launch_id(
    payload: Mapping[str, Any],
    fields: frozenset[str],
    environment: Mapping[str, str] | None,
    error_type: type[RuntimeError],
) -> dict[str, Any]:
    if type(payload) is not dict or frozenset(payload) != fields - {"launch_id"}:
        raise error_type("evidence fields are invalid")
    return {**payload, "launch_id": process_launch_id(environment)}


def _publish(
    path: Path,
    filename: str,
    payload: Mapping[str, Any],
    bound_publisher: Callable[[str, bytes], object] | None,
    error_type: type[RuntimeError],
    label: str,
) -> None:
    try:
        raw = json.dumps(payload, allow_nan=False, separators=(",", ":")).encode("utf-8")
        if bound_publisher is not None:
            bound_publisher(filename, raw)
        else:
            _atomic_write(path, raw)
    except (OSError, TypeError, ValueError, RecursionError) as exc:
        raise error_type(f"{label} cannot be published") from exc


def _load(
    path: Path,
    limit: int,
    error_type: type[RuntimeError],
    label: str,
) -> dict[str, Any]:
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise error_type(f"{label} file is unavailable") from exc
    if len(raw) > limit:
        raise error_type(f"{label} exceeds {limit} bytes")
    try:
        payload = json.loads(raw)
    except (UnicodeError, ValueError, TypeError) as exc:
        raise error_type(f"{label} is invalid JSON") from exc
    if type(payload) is not dict:
        raise error_type(f"{label} must be a JSON object")
    return payload


def _atomic_write(path: Path, raw: bytes) -> None:
    temporary = path.with_name(f".{path.name}.{secrets.token_hex(8)}.tmp")
    try:
        temporary.write_bytes(raw)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _validate_motion(
    payload: Mapping[str, Any],
    *,
    product_session_id: object,
    product: object,
    process: object,
    launch_id: object,
) -> None:
    if frozenset(payload) != _MOTION_FIELDS:
        raise SimMotionEvidenceError("motion evidence fields are invalid")
    if (
        payload["schema"] != MOTION_EVIDENCE_SCHEMA
        or not isinstance(product_session_id, str)
        or not is_product_session_id(product_session_id)
        or payload["product_session_id"] != product_session_id
        or payload["product"] != product
        or not _plain_text(product)
        or payload["process"] != process
        or not _plain_text(process)
        or payload["launch_id"] != launch_id
        or not _plain_text(launch_id)
    ):
        raise SimMotionEvidenceError("motion evidence identity is invalid")
    if not _plain_text(payload["bridge_boot_id"]) or not _plain_text(
        payload["controller_boot_id"]
    ):
        raise SimMotionEvidenceError("motion evidence boot identity is invalid")
    for field in ("terminal_bridge_command_seq", "terminal_applied_step_seq", "pose_sample_count"):
        _positive_int(payload[field], field, SimMotionEvidenceError)
    for field in ("min_base_height_m", "max_base_height_m", "max_abs_roll_rad", "max_abs_pitch_rad"):
        _finite_number(payload[field], field, SimMotionEvidenceError)
    minimum_height = float(payload["min_base_height_m"])
    maximum_height = float(payload["max_base_height_m"])
    if (
        minimum_height > maximum_height
        or minimum_height < MIN_BASE_HEIGHT_M
        or maximum_height > MAX_BASE_HEIGHT_M
        or maximum_height - minimum_height > MAX_BASE_HEIGHT_SPAN_M
        or not 0.0 <= payload["max_abs_roll_rad"] <= MAX_ABS_TILT_RAD
        or not 0.0 <= payload["max_abs_pitch_rad"] <= MAX_ABS_TILT_RAD
    ):
        raise SimMotionEvidenceError("motion pose stability gate failed")

    observed = payload["commanded_motion_observed"]
    if not isinstance(observed, bool):
        raise SimMotionEvidenceError("motion evidence observed flag is invalid")
    count_fields = ("nonzero_command_count", "nonzero_physics_steps")
    sequence_fields = (
        "first_bridge_command_seq",
        "last_bridge_command_seq",
        "first_motion_step_seq",
        "last_motion_step_seq",
        "first_output_sequence",
        "last_output_sequence",
    )
    for field in (*count_fields, *sequence_fields):
        _nonnegative_int(payload[field], field, SimMotionEvidenceError)
    for field in ("net_displacement_xy_m", "path_length_xy_m"):
        value = _finite_number(payload[field], field, SimMotionEvidenceError)
        if value < 0.0:
            raise SimMotionEvidenceError(f"{field} is invalid")
    if payload["path_length_xy_m"] + 1e-9 < payload["net_displacement_xy_m"]:
        raise SimMotionEvidenceError("motion path is shorter than net displacement")

    if observed:
        if payload["path_length_xy_m"] <= 0.0 or any(
            payload[field] <= 0 for field in (*count_fields, *sequence_fields)
        ):
            raise SimMotionEvidenceError("observed motion evidence is incomplete")
        if (
            payload["first_bridge_command_seq"] > payload["last_bridge_command_seq"]
            or payload["last_bridge_command_seq"] >= payload["terminal_bridge_command_seq"]
            or payload["first_motion_step_seq"] > payload["last_motion_step_seq"]
            or payload["last_motion_step_seq"] >= payload["terminal_applied_step_seq"]
            or payload["first_output_sequence"] > payload["last_output_sequence"]
        ):
            raise SimMotionEvidenceError("motion evidence sequence order is invalid")
        if not _plain_text(payload["first_producer_boot_id"]) or not _plain_text(
            payload["last_producer_boot_id"]
        ):
            raise SimMotionEvidenceError("motion producer identity is invalid")
        _position(payload["start_position_m"], "start_position_m")
        _position(payload["end_position_m"], "end_position_m")
        start_yaw = _finite_number(payload["start_yaw_rad"], "start_yaw_rad", SimMotionEvidenceError)
        end_yaw = _finite_number(payload["end_yaw_rad"], "end_yaw_rad", SimMotionEvidenceError)
        trajectory = payload["trajectory"]
        if type(trajectory) is not list or len(trajectory) < 2 or len(trajectory) > 4096:
            raise SimMotionEvidenceError("motion trajectory is invalid")
        previous_step = -1.0
        for sample in trajectory:
            if type(sample) is not list or len(sample) != 5:
                raise SimMotionEvidenceError("motion trajectory sample is invalid")
            step = _finite_number(sample[0], "trajectory step", SimMotionEvidenceError)
            if step < previous_step:
                raise SimMotionEvidenceError("motion trajectory steps are not monotonic")
            previous_step = step
            for coordinate in sample[1:]:
                _finite_number(coordinate, "motion trajectory sample", SimMotionEvidenceError)
        if not math.isclose(start_yaw, float(trajectory[0][4]), abs_tol=1e-6) or not math.isclose(
            end_yaw, float(trajectory[-1][4]), abs_tol=1e-6
        ):
            raise SimMotionEvidenceError("motion trajectory yaw endpoints are inconsistent")
        for index, expected in enumerate(payload["start_position_m"], start=1):
            if not math.isclose(float(expected), float(trajectory[0][index]), abs_tol=1e-6):
                raise SimMotionEvidenceError("motion trajectory start is inconsistent")
        for index, expected in enumerate(payload["end_position_m"], start=1):
            if not math.isclose(float(expected), float(trajectory[-1][index]), abs_tol=1e-6):
                raise SimMotionEvidenceError("motion trajectory end is inconsistent")
    elif (
        any(payload[field] != 0 for field in (*count_fields, *sequence_fields))
        or payload["first_producer_boot_id"] != ""
        or payload["last_producer_boot_id"] != ""
        or payload["start_position_m"] is not None
        or payload["end_position_m"] is not None
        or payload["net_displacement_xy_m"] != 0.0
        or payload["path_length_xy_m"] != 0.0
        or payload["start_yaw_rad"] is not None
        or payload["end_yaw_rad"] is not None
        or payload["trajectory"] != []
    ):
        raise SimMotionEvidenceError("empty motion evidence contains motion")


def _validate_feeder(
    payload: Mapping[str, Any],
    *,
    product: object,
    product_session_id: object,
    process: object,
    launch_id: object,
) -> None:
    if frozenset(payload) != _FEEDER_FIELDS:
        raise SimFeederStatusError("feeder status fields are invalid")
    if (
        payload["schema"] != FEEDER_STATUS_SCHEMA
        or payload["product"] != product
        or not _plain_text(product)
        or not isinstance(product_session_id, str)
        or not is_product_session_id(product_session_id)
        or payload["product_session_id"] != product_session_id
        or payload["process"] != process
        or not _plain_text(process)
        or payload["launch_id"] != launch_id
        or not _plain_text(launch_id)
    ):
        raise SimFeederStatusError("feeder status identity is invalid")
    if payload["state"] not in _FEEDER_STATES:
        raise SimFeederStatusError("feeder status state is invalid")
    _nonnegative_int(payload["sequence"], "sequence", SimFeederStatusError)
    _positive_int(payload["updated_wall_ns"], "updated_wall_ns", SimFeederStatusError)
    window_s = _nonnegative_number(payload["window_s"], "window_s", SimFeederStatusError)
    streams = payload["streams"]
    if type(streams) is not dict or not set(streams) <= _STREAM_NAMES:
        raise SimFeederStatusError("feeder status streams are invalid")
    for stream in streams.values():
        _validate_stream(stream, window_s, state=str(payload["state"]))


def _validate_stream(stream: object, window_s: float, *, state: str) -> None:
    if type(stream) is not dict or frozenset(stream) != _STREAM_FIELDS:
        raise SimFeederStatusError("feeder stream fields are invalid")
    expected_hz = _nonnegative_number(stream["expected_hz"], "expected_hz", SimFeederStatusError)
    if expected_hz <= 0.0:
        raise SimFeederStatusError("expected_hz must be positive")
    scheduled = _nonnegative_int(stream["scheduled_count"], "scheduled_count", SimFeederStatusError)
    published = _nonnegative_int(stream["published_count"], "published_count", SimFeederStatusError)
    dropped = _nonnegative_int(stream["dropped_count"], "dropped_count", SimFeederStatusError)
    settled = published + dropped
    if scheduled < settled or (state != "running" and scheduled != settled):
        raise SimFeederStatusError("feeder stream counts are inconsistent")
    actual_hz = _nonnegative_number(stream["actual_hz"], "actual_hz", SimFeederStatusError)
    _nonnegative_number(
        stream["max_schedule_lateness_ms"],
        "max_schedule_lateness_ms",
        SimFeederStatusError,
    )
    calculated_hz = published / window_s if window_s > 0.0 else 0.0
    if (window_s == 0.0 and published != 0) or not math.isclose(
        actual_hz, calculated_hz, rel_tol=1e-6, abs_tol=1e-3
    ):
        raise SimFeederStatusError("feeder stream actual_hz is inconsistent")


def _position(value: object, field: str) -> None:
    if type(value) is not list or len(value) != 3:
        raise SimMotionEvidenceError(f"{field} is invalid")
    for coordinate in value:
        _finite_number(coordinate, field, SimMotionEvidenceError)


def _plain_text(value: object) -> bool:
    return isinstance(value, str) and bool(value) and value == value.strip()


def _nonnegative_int(value: object, field: str, error_type: type[RuntimeError]) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise error_type(f"{field} is invalid")
    return value


def _positive_int(value: object, field: str, error_type: type[RuntimeError]) -> int:
    parsed = _nonnegative_int(value, field, error_type)
    if parsed == 0:
        raise error_type(f"{field} must be positive")
    return parsed


def _finite_number(value: object, field: str, error_type: type[RuntimeError]) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise error_type(f"{field} is invalid")
    return float(value)


def _nonnegative_number(value: object, field: str, error_type: type[RuntimeError]) -> float:
    parsed = _finite_number(value, field, error_type)
    if parsed < 0.0:
        raise error_type(f"{field} is invalid")
    return parsed


__all__ = [
    "FEEDER_STATUS_FILENAME",
    "FEEDER_STATUS_SCHEMA",
    "MAX_ABS_TILT_RAD",
    "MAX_BASE_HEIGHT_M",
    "MAX_BASE_HEIGHT_SPAN_M",
    "MIN_BASE_HEIGHT_M",
    "MOTION_EVIDENCE_FILENAME",
    "MOTION_EVIDENCE_SCHEMA",
    "SimFeederStatusError",
    "SimMotionEvidenceError",
    "load_feeder_status",
    "load_motion_evidence",
    "publish_feeder_status",
    "publish_motion_evidence",
]
