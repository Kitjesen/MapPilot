"""Strict same-session evidence summary for compiled sensor streams.

The ordinary :mod:`readiness` model answers whether endpoints completed their
lifecycle transitions.  This module is deliberately stricter: qualification
uses it to prove that every required stream also produced data through the
exact route compiled into the SensorPlan.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from typing import Any

from .contracts import SensorStreamPlan
from .runtime import SensorRuntime

_SCHEMA = "lingtu.sim.sensor-stream-summary.v1"
_STATES = frozenset({"UNBOUND", "PREPARED", "ACTIVE", "FAILED"})
_OBSERVATION_FIELDS = frozenset(
    {
        "sensor_id",
        "state",
        "session_id",
        "model_generation",
        "reset_generation",
        "owner",
        "source",
        "transport",
        "message_type",
        "runtime_source_id",
        "binding_identity",
        "sample_count",
        "last_sample_truth_sequence",
        "last_sample_sim_time_ns",
        "shm_name",
        "failure_reason",
        "fidelity",
    }
)
_MID360_FIDELITY = {
    "qualification": "LIMITED",
    "reflectivity_semantics": "explicit_conservative_proxy",
    "scan_time_profile": "instantaneous_geometry/scheduled_offsets",
    "unknown_line_representation": "line_0_unknown_physical_channel",
    "limitations": [
        "reflectivity_is_a_conservative_proxy",
        "line_0_means_unknown_physical_channel",
        "ray_geometry_is_instantaneous_and_offsets_do_not_model_motion_distortion",
    ],
}
_MID360_EVIDENCE_FIELDS = frozenset(
    {
        "reflectivity_semantics",
        "scan_time_profile",
        "unknown_line_representation",
    }
)
THUNDERV4_NAVIGATION_STREAM_IDS = (
    "thunder_01.front_depth",
    "thunder_01.front_rgb",
    "thunder_01.imu",
    "thunder_01.mid360",
    "thunder_01.truth_odom",
)
_THUNDERV4_NAVIGATION_CONTRACT = {
    "thunder_01.front_depth": (
        "depth",
        "visual",
        "unreal_camera",
        "camera_shm",
        "lingtu.dds.Image",
        None,
    ),
    "thunder_01.front_rgb": (
        "rgb",
        "visual",
        "unreal_camera",
        "camera_shm",
        "lingtu.dds.Image",
        None,
    ),
    "thunder_01.imu": (
        "imu",
        "physics",
        "mujoco_sensor",
        "typed_dds",
        "lingtu.dds.Imu",
        None,
    ),
    "thunder_01.mid360": (
        "mid360",
        "physics",
        "mujoco_livox_model",
        "typed_dds",
        "lingtu.dds.LivoxFrame",
        "thunder_01/lidar1_link_site",
    ),
    "thunder_01.truth_odom": (
        "truth_odom",
        "physics",
        "mujoco_truth",
        "typed_dds",
        "lingtu.dds.Odometry",
        None,
    ),
}


class SensorEvidenceError(ValueError):
    """Raised when stream evidence cannot be joined to one trusted session."""


def sensor_stream_binding_identity(
    stream: SensorStreamPlan,
    *,
    session_id: str,
    model_generation: int,
    reset_generation: int,
    runtime_source_id: str,
) -> str:
    """Return the canonical identity of one generation-bound stream route."""

    if not isinstance(stream, SensorStreamPlan):
        raise SensorEvidenceError("stream must be a SensorStreamPlan")
    parts = (
        _text(session_id, "session_id"),
        str(_generation(model_generation, "model_generation")),
        str(_generation(reset_generation, "reset_generation")),
        stream.sensor_id,
        stream.route.owner,
        stream.route.source,
        stream.route.transport,
        stream.message_type,
        _text(runtime_source_id, "runtime_source_id"),
    )
    return "sensor-binding:" + ":".join(parts)


def build_sensor_stream_summary(
    plan: SensorRuntime,
    observations: Iterable[Mapping[str, Any]],
    *,
    model_generation: int,
    reset_generation: int,
    required_stream_ids: Iterable[str] | None = None,
    shm_allocations: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    """Build one strict, JSON-ready stream summary for qualification.

    Missing or zero-output required streams remain explicit blockers.  Stale,
    duplicate, route-mismatched, or allocation-conflicting evidence is rejected
    because it cannot safely contribute to readiness.
    """

    if not isinstance(plan, SensorRuntime):
        raise SensorEvidenceError("plan must be a SensorRuntime")
    current_model = _generation(model_generation, "model_generation")
    current_reset = _generation(reset_generation, "reset_generation")
    streams = {stream.sensor_id: stream for stream in plan.streams}
    if not streams:
        raise SensorEvidenceError("sensor plan must contain at least one stream")
    required = _required_stream_ids(required_stream_ids, streams)
    allocations = _validate_shm_allocations(shm_allocations or {}, streams)

    accepted: dict[str, dict[str, Any]] = {}
    if isinstance(observations, (str, bytes, Mapping)):
        raise SensorEvidenceError("observations must be an iterable of stream objects")
    try:
        iterator = iter(observations)
    except TypeError as exc:
        raise SensorEvidenceError("observations must be an iterable of stream objects") from exc
    for index, raw in enumerate(iterator):
        accepted_observation = _validate_observation(
            raw,
            index=index,
            plan=plan,
            streams=streams,
            model_generation=current_model,
            reset_generation=current_reset,
            shm_allocations=allocations,
        )
        sensor_id = accepted_observation["stream_id"]
        if sensor_id in accepted:
            raise SensorEvidenceError(
                f"duplicate evidence for sensor stream {sensor_id!r}"
            )
        accepted[sensor_id] = accepted_observation

    stream_documents: dict[str, dict[str, Any]] = {}
    blocking: dict[str, str] = {}
    for sensor_id, stream in sorted(streams.items()):
        required_stream = sensor_id in required
        stream_observation = accepted.get(sensor_id)
        if stream_observation is None:
            document = _missing_stream_document(stream, required=required_stream)
        else:
            document = dict(stream_observation)
            document["required"] = required_stream
        stream_documents[sensor_id] = document
        if not required_stream:
            continue
        if stream_observation is None:
            blocking[sensor_id] = "stream evidence is MISSING"
        elif stream_observation["state"] != "ACTIVE":
            blocking[sensor_id] = f"stream is {stream_observation['state']}"
        elif stream_observation["sample_count"] <= 0:
            blocking[sensor_id] = "ACTIVE stream has no published sample or frame"

    return {
        "schema": _SCHEMA,
        "session_id": plan.session_id,
        "model_generation": current_model,
        "reset_generation": current_reset,
        "is_ready": not blocking,
        "required_stream_ids": sorted(required),
        "blocking_reasons": blocking,
        "streams": stream_documents,
    }


def build_thunderv4_navigation_stream_summary(
    plan: SensorRuntime,
    observations: Iterable[Mapping[str, Any]],
    *,
    model_generation: int,
    reset_generation: int,
    shm_allocations: Mapping[str, str],
) -> dict[str, Any]:
    """Build the exact five-stream ThunderV4 navigation qualification view."""

    if not isinstance(plan, SensorRuntime):
        raise SensorEvidenceError("plan must be a SensorRuntime")
    actual = frozenset(stream.sensor_id for stream in plan.streams)
    expected = frozenset(THUNDERV4_NAVIGATION_STREAM_IDS)
    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        raise SensorEvidenceError(
            "ThunderV4 navigation requires the exact five-stream set; "
            f"missing={missing}, extra={extra}"
        )
    for stream in plan.streams:
        actual_contract = (
            stream.stream_kind,
            stream.route.owner,
            stream.route.source,
            stream.route.transport,
            stream.message_type,
            stream.raycast_frame_stable_id,
        )
        expected_contract = _THUNDERV4_NAVIGATION_CONTRACT[stream.sensor_id]
        if actual_contract != expected_contract:
            raise SensorEvidenceError(
                f"ThunderV4 navigation stream {stream.sensor_id!r} route contract mismatch"
            )
    return build_sensor_stream_summary(
        plan,
        observations,
        model_generation=model_generation,
        reset_generation=reset_generation,
        required_stream_ids=THUNDERV4_NAVIGATION_STREAM_IDS,
        shm_allocations=shm_allocations,
    )


def _validate_observation(
    raw: Mapping[str, Any],
    *,
    index: int,
    plan: SensorRuntime,
    streams: Mapping[str, SensorStreamPlan],
    model_generation: int,
    reset_generation: int,
    shm_allocations: Mapping[str, str],
) -> dict[str, Any]:
    label = f"observations[{index}]"
    if type(raw) is not dict:
        raise SensorEvidenceError(f"{label} must be an object")
    unknown = sorted(set(raw) - _OBSERVATION_FIELDS)
    if unknown:
        raise SensorEvidenceError(
            f"{label} has unknown field(s): {', '.join(unknown)}"
        )
    sensor_id = _text(raw.get("sensor_id"), f"{label}.sensor_id")
    try:
        stream = streams[sensor_id]
    except KeyError as exc:
        raise SensorEvidenceError(f"{label} names unknown stream {sensor_id!r}") from exc
    if raw.get("session_id") != plan.session_id:
        raise SensorEvidenceError(f"{sensor_id} session_id mismatch")
    if _generation(raw.get("model_generation"), f"{sensor_id}.model_generation") != model_generation:
        raise SensorEvidenceError(f"{sensor_id} model_generation is stale or from the future")
    if _generation(raw.get("reset_generation"), f"{sensor_id}.reset_generation") != reset_generation:
        raise SensorEvidenceError(f"{sensor_id} reset_generation is stale or from the future")
    for field, expected in (
        ("owner", stream.route.owner),
        ("source", stream.route.source),
        ("transport", stream.route.transport),
        ("message_type", stream.message_type),
    ):
        actual = _text(raw.get(field), f"{sensor_id}.{field}")
        if actual != expected:
            raise SensorEvidenceError(
                f"{sensor_id} {field} mismatch: expected {expected!r}"
            )
    runtime_source_id = _text(
        raw.get("runtime_source_id"), f"{sensor_id}.runtime_source_id"
    )
    expected_binding = sensor_stream_binding_identity(
        stream,
        session_id=plan.session_id,
        model_generation=model_generation,
        reset_generation=reset_generation,
        runtime_source_id=runtime_source_id,
    )
    binding_identity = _text(
        raw.get("binding_identity"), f"{sensor_id}.binding_identity"
    )
    if binding_identity != expected_binding:
        raise SensorEvidenceError(f"{sensor_id} binding_identity mismatch")
    state = _text(raw.get("state"), f"{sensor_id}.state")
    if state not in _STATES:
        raise SensorEvidenceError(f"{sensor_id} state is unsupported: {state!r}")
    sample_count = _generation(
        raw.get("sample_count"), f"{sensor_id}.sample_count"
    )
    raw_last_truth_sequence = raw.get("last_sample_truth_sequence")
    raw_last_sample_time = raw.get("last_sample_sim_time_ns")
    if raw_last_truth_sequence is None:
        if sample_count > 0:
            raise SensorEvidenceError(
                f"{sensor_id} has published samples but no actual last truth sequence"
            )
        last_sample_truth_sequence = None
    else:
        last_sample_truth_sequence = _generation(
            raw_last_truth_sequence,
            f"{sensor_id}.last_sample_truth_sequence",
        )
        if sample_count == 0:
            raise SensorEvidenceError(
                f"{sensor_id} cannot claim a last truth sequence with zero samples"
            )
    if raw_last_sample_time is None:
        if sample_count > 0:
            raise SensorEvidenceError(
                f"{sensor_id} has published samples but no actual last sample timestamp"
            )
        last_sample_sim_time_ns = None
    else:
        last_sample_sim_time_ns = _generation(
            raw_last_sample_time,
            f"{sensor_id}.last_sample_sim_time_ns",
        )
        if sample_count == 0:
            raise SensorEvidenceError(
                f"{sensor_id} cannot claim a last sample timestamp with zero samples"
            )
    failure_reason = raw.get("failure_reason")
    if state == "FAILED":
        failure_reason = _text(failure_reason, f"{sensor_id}.failure_reason")
    elif failure_reason is not None:
        raise SensorEvidenceError(
            f"{sensor_id} failure_reason is valid only for FAILED"
        )

    shm_name = raw.get("shm_name")
    if stream.route.transport == "camera_shm":
        expected_shm = shm_allocations.get(sensor_id)
        if expected_shm is None:
            raise SensorEvidenceError(f"{sensor_id} has no run SHM allocation")
        shm_name = _text(shm_name, f"{sensor_id}.shm_name")
        if shm_name != expected_shm:
            raise SensorEvidenceError(f"{sensor_id} shm_name does not match run allocation")
    elif shm_name is not None:
        raise SensorEvidenceError(
            f"{sensor_id} must not declare shm_name for {stream.route.transport}"
        )

    fidelity = _validate_fidelity(
        raw.get("fidelity"),
        stream,
        sensor_id,
        state=state,
    )
    document: dict[str, Any] = {
        "stream_id": sensor_id,
        "stream_kind": stream.stream_kind,
        "state": state,
        "session_id": plan.session_id,
        "model_generation": model_generation,
        "reset_generation": reset_generation,
        "owner": stream.route.owner,
        "source": stream.route.source,
        "transport": stream.route.transport,
        "message_type": stream.message_type,
        "runtime_source_id": runtime_source_id,
        "binding_identity": binding_identity,
        "sample_count": sample_count,
        "last_sample_truth_sequence": last_sample_truth_sequence,
        "last_sample_sim_time_ns": last_sample_sim_time_ns,
        "shm_name": shm_name,
        "failure_reason": failure_reason,
    }
    if fidelity is not None:
        document["fidelity"] = fidelity
    return document


def _validate_fidelity(
    raw: Any,
    stream: SensorStreamPlan,
    sensor_id: str,
    *,
    state: str,
) -> dict[str, Any] | None:
    if stream.stream_kind != "mid360":
        if raw is not None:
            raise SensorEvidenceError(
                f"{sensor_id} fidelity is only valid for a mid360 stream"
            )
        return None
    if raw is None and state != "ACTIVE":
        return {
            key: list(value) if isinstance(value, list) else value
            for key, value in _MID360_FIDELITY.items()
        }
    if type(raw) is not dict:
        raise SensorEvidenceError(f"{sensor_id} requires explicit Mid360 fidelity evidence")
    unknown = sorted(set(raw) - _MID360_EVIDENCE_FIELDS)
    missing = sorted(_MID360_EVIDENCE_FIELDS - set(raw))
    if unknown or missing:
        detail = []
        if missing:
            detail.append("missing " + ", ".join(missing))
        if unknown:
            detail.append("unknown " + ", ".join(unknown))
        raise SensorEvidenceError(f"{sensor_id} Mid360 fidelity is invalid: {'; '.join(detail)}")
    for field in sorted(_MID360_EVIDENCE_FIELDS):
        expected = _MID360_FIDELITY[field]
        if raw.get(field) != expected:
            raise SensorEvidenceError(
                f"{sensor_id} Mid360 {field} must disclose {expected!r}"
            )
    return {
        "qualification": _MID360_FIDELITY["qualification"],
        "reflectivity_semantics": _MID360_FIDELITY["reflectivity_semantics"],
        "scan_time_profile": _MID360_FIDELITY["scan_time_profile"],
        "unknown_line_representation": _MID360_FIDELITY[
            "unknown_line_representation"
        ],
        "limitations": list(_MID360_FIDELITY["limitations"]),
    }


def _missing_stream_document(
    stream: SensorStreamPlan,
    *,
    required: bool,
) -> dict[str, Any]:
    document: dict[str, Any] = {
        "stream_id": stream.sensor_id,
        "stream_kind": stream.stream_kind,
        "required": required,
        "state": "MISSING",
        "owner": stream.route.owner,
        "source": stream.route.source,
        "transport": stream.route.transport,
        "message_type": stream.message_type,
        "runtime_source_id": None,
        "binding_identity": None,
        "sample_count": 0,
        "last_sample_truth_sequence": None,
        "last_sample_sim_time_ns": None,
        "shm_name": None,
        "failure_reason": None,
    }
    if stream.stream_kind == "mid360":
        document["fidelity"] = {
            key: list(value) if isinstance(value, list) else value
            for key, value in _MID360_FIDELITY.items()
        }
    return document


def _required_stream_ids(
    values: Iterable[str] | None,
    streams: Mapping[str, SensorStreamPlan],
) -> frozenset[str]:
    if values is None:
        return frozenset(streams)
    if isinstance(values, (str, bytes)):
        raise SensorEvidenceError("required_stream_ids must be an iterable of IDs")
    try:
        normalized = tuple(
            _text(value, "required_stream_ids item") for value in values
        )
    except TypeError as exc:
        raise SensorEvidenceError(
            "required_stream_ids must be an iterable of IDs"
        ) from exc
    if len(set(normalized)) != len(normalized):
        raise SensorEvidenceError("required_stream_ids contains duplicate ID")
    unknown = sorted(set(normalized) - set(streams))
    if unknown:
        raise SensorEvidenceError(
            f"required_stream_ids contains unknown stream(s): {unknown}"
        )
    return frozenset(normalized)


def _validate_shm_allocations(
    allocations: Mapping[str, str],
    streams: Mapping[str, SensorStreamPlan],
) -> dict[str, str]:
    if not isinstance(allocations, Mapping):
        raise SensorEvidenceError("shm_allocations must be an object")
    result: dict[str, str] = {}
    seen_names: dict[str, str] = {}
    for raw_sensor_id, raw_name in allocations.items():
        sensor_id = _text(raw_sensor_id, "shm_allocations key")
        try:
            stream = streams[sensor_id]
        except KeyError as exc:
            raise SensorEvidenceError(
                f"SHM allocation names unknown stream {sensor_id!r}"
            ) from exc
        if stream.route.transport != "camera_shm":
            raise SensorEvidenceError(
                f"SHM allocation for {sensor_id!r} conflicts with transport {stream.route.transport!r}"
            )
        name = _text(raw_name, f"shm_allocations.{sensor_id}")
        key = name.casefold()
        previous = seen_names.get(key)
        if previous is not None:
            raise SensorEvidenceError(
                f"SHM allocation collision between {previous!r} and {sensor_id!r}"
            )
        seen_names[key] = sensor_id
        result[sensor_id] = name
    return result


def _text(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise SensorEvidenceError(f"{field} must be non-empty trimmed text")
    return value


def _generation(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise SensorEvidenceError(f"{field} must be a non-negative integer")
    return value


__all__ = [
    "THUNDERV4_NAVIGATION_STREAM_IDS",
    "SensorEvidenceError",
    "build_sensor_stream_summary",
    "build_thunderv4_navigation_stream_summary",
    "sensor_stream_binding_identity",
]
