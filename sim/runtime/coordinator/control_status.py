"""Truthful owner-thread status for the RobotSimUE playable control surface."""

from __future__ import annotations

import json
import math
import re
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Protocol

from .control_intent_udp import (
    MAX_CONTROL_DATAGRAM_BYTES,
    ControlIntentValidationError,
    OperatorIntentIdentity,
)

CONTROL_STATUS_SCHEMA = "lingtu.sim.ue-control-status.v1"
CONTROL_STATUS_PERIOD_NS = 100_000_000

_MAX_EXACT_JSON_INT = (1 << 53) - 1
_SAFE_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z")
_TOP_LEVEL_FIELDS = frozenset(
    {
        "schema",
        "run_id",
        "session_id",
        "boot_id",
        "model_generation",
        "reset_generation",
        "server_status_sequence",
        "server_monotonic_ns",
        "sim_time_ns",
        "truth_sequence",
        "source_id",
        "source_epoch",
        "source_sequence",
        "event_id",
        "intent_datagram_sha256",
        "status",
        "reason",
        "runtime",
        "motion",
        "readiness",
        "sensors",
        "recording",
        "ui",
    }
)
_RUNTIME_FIELDS = frozenset(
    {
        "runtime_state",
        "control_owner",
        "deadman",
        "sample_age_ns",
        "safe_stop_state",
    }
)
_MOTION_FIELDS = frozenset(
    {
        "requested_axes",
        "admitted_twist_mps_radps",
        "observed_base_velocity_mps_radps",
    }
)
_REQUESTED_FIELDS = frozenset(
    {"available", "forward", "left", "yaw_left"}
)
_VELOCITY_FIELDS = frozenset(
    {"available", "linear_x", "linear_y", "angular_z"}
)
_READINESS_FIELDS = frozenset({"physics", "control", "visual", "sensors"})
_FACET_FIELDS = frozenset({"state", "required", "source_id", "blocker"})
_SENSOR_FIELDS = frozenset({"stream_id", "state", "sample_count", "blocker"})
_RECORDING_FIELDS = frozenset(
    {"state", "elapsed_sim_time_ns", "artifact_id", "blocker"}
)
_UI_FIELDS = frozenset({"ui_mode", "camera_mode"})
_RECORDING_SOURCE_FIELDS = frozenset(
    {
        "schema",
        "run_id",
        "session_id",
        "model_generation",
        "reset_generation",
        "state",
        "elapsed_sim_time_ns",
        "artifact_id",
        "blocker",
    }
)
_ACK_STATUSES = frozenset(
    {"pending", "accepted", "rejected", "released", "timeout_zero", "confirmed"}
)
_RUNTIME_STATES = frozenset(
    {"NEW", "PREPARING", "READY", "RUNNING", "PAUSED", "STOPPED", "FAILED"}
)
_SAFE_STOP_STATES = frozenset(
    {"unavailable", "clear", "pending", "zeroed", "blocked"}
)
_BINDING_STATES = frozenset(
    {"UNAVAILABLE", "UNBOUND", "PREPARED", "ACTIVE", "FAILED"}
)
_SENSOR_STATES = frozenset(
    {"UNAVAILABLE", "MISSING", "UNBOUND", "PREPARED", "ACTIVE", "FAILED"}
)
_RECORDING_STATES = frozenset(
    {
        "unavailable",
        "idle",
        "requested",
        "recording",
        "committed",
        "rejected",
        "failed",
    }
)
_DURABLE_RECORDING_STATES = frozenset(
    {"unavailable", "idle", "recording", "committed", "failed"}
)
_UI_MODES = frozenset({"unavailable", "drive", "build", "tactical", "menu"})
_CAMERA_MODES = frozenset(
    {"unavailable", "follow", "inspection", "free"}
)


class ControlStatusPublisher(Protocol):
    """Publisher accepted by :class:`ControlStatusReporter`."""

    def publish(self, document: Mapping[str, Any]) -> int: ...


@dataclass(frozen=True, slots=True)
class ControlStatusControlSnapshot:
    """Latest control admission owned by the interactive session thread."""

    identity: OperatorIntentIdentity
    source_epoch: int
    source_sequence: int
    event_id: str
    intent_datagram_sha256: str
    status: str
    reason: str
    control_owner: str
    deadman: bool
    sample_age_ns: int
    safe_stop_state: str
    requested_axes: tuple[float, float, float]
    requested_available: bool
    admitted_twist: tuple[float, float, float]
    admitted_available: bool
    ui_mode: str
    camera_mode: str

    def __post_init__(self) -> None:
        _positive_int(self.source_epoch, "source_epoch")
        _positive_int(self.source_sequence, "source_sequence")
        _event_id(
            self.event_id,
            self.identity.boot_id,
            self.source_epoch,
            self.source_sequence,
        )
        _sha256(self.intent_datagram_sha256, "intent_datagram_sha256")
        _enum(self.status, _ACK_STATUSES, "status")
        _reason(self.reason)
        if self.status not in {"accepted", "confirmed"} and not self.reason:
            raise ControlIntentValidationError(
                f"reason must be non-empty when status is {self.status}"
            )
        if self.control_owner != "unavailable":
            _safe_id(self.control_owner, "control_owner")
        _boolean(self.deadman, "deadman")
        _nonnegative_int(self.sample_age_ns, "sample_age_ns")
        _enum(self.safe_stop_state, _SAFE_STOP_STATES, "safe_stop_state")
        _vector(self.requested_axes, "requested_axes")
        _boolean(self.requested_available, "requested_available")
        _vector(self.admitted_twist, "admitted_twist")
        _boolean(self.admitted_available, "admitted_available")
        _enum(self.ui_mode, _UI_MODES, "ui_mode")
        _enum(self.camera_mode, _CAMERA_MODES, "camera_mode")


class ControlStatusReporter:
    """Build and publish full status at 10 Hz and on discrete state changes."""

    def __init__(
        self,
        *,
        publisher: ControlStatusPublisher | Callable[[Mapping[str, Any]], int],
        published_sink: Callable[[Mapping[str, Any]], object] | None = None,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        period_ns: int = CONTROL_STATUS_PERIOD_NS,
    ) -> None:
        if not callable(publisher) and not callable(getattr(publisher, "publish", None)):
            raise TypeError("publisher must be callable or expose publish(document)")
        if published_sink is not None and not callable(published_sink):
            raise TypeError("published_sink must be callable when provided")
        self._publisher = publisher
        self._published_sink = published_sink
        self._monotonic_ns = monotonic_ns
        self._period_ns = _positive_int(period_ns, "period_ns")
        self._owner_thread_id: int | None = None
        self._server_status_sequence = 0
        self._next_periodic_ns: int | None = None
        self._last_state_key: str | None = None

    def build_document(
        self,
        control: ControlStatusControlSnapshot,
        authority: Mapping[str, Any],
        *,
        server_monotonic_ns: int | None = None,
    ) -> dict[str, Any]:
        """Build one exact document without publishing it."""

        self._require_owner_thread()
        now_ns = self._clock() if server_monotonic_ns is None else _nonnegative_int(
            server_monotonic_ns,
            "server_monotonic_ns",
        )
        self._server_status_sequence += 1
        document = _build_document(
            control,
            authority,
            server_status_sequence=self._server_status_sequence,
            server_monotonic_ns=now_ns,
        )
        encode_control_status(document)
        return document

    def publish_after_advance(
        self,
        control: ControlStatusControlSnapshot,
        authority: Mapping[str, Any],
        *,
        force: bool = False,
    ) -> int:
        """Publish after authoritative truth/sensors advance on the owner thread."""

        self._require_owner_thread()
        _boolean(force, "force")
        now_ns = self._clock()
        candidate = _build_document(
            control,
            authority,
            server_status_sequence=self._server_status_sequence + 1,
            server_monotonic_ns=now_ns,
        )
        state_key = _state_key(candidate)
        periodic_due = (
            self._next_periodic_ns is None or now_ns >= self._next_periodic_ns
        )
        changed = state_key != self._last_state_key
        if not force and not periodic_due and not changed:
            return 0
        if periodic_due:
            self._next_periodic_ns = now_ns + self._period_ns
        self._server_status_sequence += 1
        candidate["server_status_sequence"] = self._server_status_sequence
        encode_control_status(candidate)
        sent = self._publish(candidate)
        if sent > 0:
            if self._published_sink is not None:
                self._published_sink(candidate)
            self._last_state_key = state_key
        return sent

    def periodic_due(self) -> bool:
        """Return whether the next 10 Hz status heartbeat needs authority."""

        self._require_owner_thread()
        now_ns = self._clock()
        return self._next_periodic_ns is None or now_ns >= self._next_periodic_ns

    def _require_owner_thread(self) -> None:
        current = threading.get_ident()
        if self._owner_thread_id is None:
            self._owner_thread_id = current
        elif self._owner_thread_id != current:
            raise ControlIntentValidationError(
                "control status may publish only on the interactive owner thread"
            )

    def _clock(self) -> int:
        return _nonnegative_int(self._monotonic_ns(), "server_monotonic_ns")

    def _publish(self, document: Mapping[str, Any]) -> int:
        expected_bytes = len(encode_control_status(document))
        publish = getattr(self._publisher, "publish", None)
        result = publish(document) if callable(publish) else self._publisher(document)  # type: ignore[operator]
        if (
            isinstance(result, bool)
            or not isinstance(result, int)
            or result != expected_bytes
        ):
            raise ControlIntentValidationError(
                "control status publisher must report the complete encoded datagram"
            )
        return result


def build_control_status_authority(
    *,
    identity: Mapping[str, Any],
    runtime_state: str,
    snapshot: Mapping[str, Any],
    runtime_manifest: Mapping[str, Any],
    recording_snapshot: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Project coordinator-owned manifest and MuJoCo truth into wire authority.

    This function never infers endpoint state from UE.  The runtime manifest is
    written by ``RuntimeCoordinator`` after each advance/evidence update, while
    ``snapshot`` is the exact post-advance physics event on the same owner
    thread.  Any identity or generation disagreement fails closed.
    """

    identity_value = _mapping(identity, "identity")
    run_id = _safe_id(identity_value.get("run_id"), "identity.run_id")
    session_id = _safe_id(
        identity_value.get("session_id"),
        "identity.session_id",
    )
    boot_id = _safe_id(identity_value.get("boot_id"), "identity.boot_id")
    model_generation = _nonnegative_int(
        identity_value.get("model_generation"),
        "identity.model_generation",
    )
    reset_generation = _nonnegative_int(
        identity_value.get("reset_generation"),
        "identity.reset_generation",
    )
    state = _enum(runtime_state, _RUNTIME_STATES, "runtime_state")
    manifest = _mapping(runtime_manifest, "runtime_manifest")
    truth = _mapping(snapshot, "snapshot")

    expected_values = {
        "run_id": run_id,
        "session_id": session_id,
        "model_generation": model_generation,
        "reset_generation": reset_generation,
    }
    if manifest.get("schema") != "lingtu.sim.session-runtime.v1":
        raise ControlIntentValidationError(
            "runtime manifest schema is unsupported"
        )
    for field, expected in expected_values.items():
        if manifest.get(field) != expected:
            raise ControlIntentValidationError(
                f"runtime manifest {field} does not match status identity"
            )
    if manifest.get("state") != state:
        raise ControlIntentValidationError(
            "runtime manifest state does not match the interactive owner"
        )
    for field, expected in (
        ("session_id", session_id),
        ("model_generation", model_generation),
        ("reset_generation", reset_generation),
    ):
        if truth.get(field) != expected:
            raise ControlIntentValidationError(
                f"truth snapshot {field} does not match status identity"
            )
    sim_time_ns = _nonnegative_int(truth.get("sim_time_ns"), "snapshot.sim_time_ns")
    truth_sequence = _nonnegative_int(truth.get("sequence"), "snapshot.sequence")

    bindings = _mapping(manifest.get("bindings"), "runtime_manifest.bindings")
    _exact_fields(bindings, _READINESS_FIELDS, "runtime_manifest.bindings")
    readiness: dict[str, dict[str, Any]] = {}
    for facet_name in sorted(_READINESS_FIELDS):
        raw = _mapping(bindings[facet_name], f"runtime_manifest.bindings.{facet_name}")
        facet_state = _enum(
            raw.get("state"),
            _BINDING_STATES - {"UNAVAILABLE"},
            f"runtime_manifest.bindings.{facet_name}.state",
        )
        if raw.get("model_generation") != model_generation:
            raise ControlIntentValidationError(
                f"readiness.{facet_name} model_generation is stale"
            )
        if raw.get("reset_generation") != reset_generation:
            raise ControlIntentValidationError(
                f"readiness.{facet_name} reset_generation is stale"
            )
        source = raw.get("source_id")
        source_id = "unavailable" if source is None else _safe_id(
            source,
            f"readiness.{facet_name}.source_id",
        )
        failure = raw.get("failure_reason")
        if failure is not None:
            failure = _reason(failure)
        blocker = "" if facet_state == "ACTIVE" else (
            failure or f"binding is {facet_state}"
        )
        readiness[facet_name] = {
            "state": facet_state,
            "required": _boolean(
                raw.get("required"),
                f"readiness.{facet_name}.required",
            ),
            "source_id": source_id,
            "blocker": blocker,
        }

    sensor_streams_value = manifest.get("sensor_streams")
    if sensor_streams_value is None:
        sensors: list[dict[str, Any]] = []
    else:
        sensor_streams = _mapping(
            sensor_streams_value,
            "runtime_manifest.sensor_streams",
        )
        summary = _mapping(
            sensor_streams.get("summary"),
            "runtime_manifest.sensor_streams.summary",
        )
        sensors = _sensor_authority(
            summary,
            session_id=session_id,
            model_generation=model_generation,
            reset_generation=reset_generation,
        )

    recording = _recording_authority(
        recording_snapshot,
        run_id=run_id,
        session_id=session_id,
        model_generation=model_generation,
        reset_generation=reset_generation,
        sim_time_ns=sim_time_ns,
    )
    return {
        "run_id": run_id,
        "session_id": session_id,
        "boot_id": boot_id,
        "model_generation": model_generation,
        "reset_generation": reset_generation,
        "runtime_state": state,
        "sim_time_ns": sim_time_ns,
        "truth_sequence": truth_sequence,
        "observed_base_velocity_mps_radps": _observed_base_velocity(truth),
        "readiness": readiness,
        "sensors": sensors,
        "recording": recording,
    }


def _sensor_authority(
    summary: Mapping[str, Any],
    *,
    session_id: str,
    model_generation: int,
    reset_generation: int,
) -> list[dict[str, Any]]:
    """Validate one current-generation summary into canonical wire order."""

    if summary.get("schema") != "lingtu.sim.sensor-stream-summary.v1":
        raise ControlIntentValidationError("sensor summary schema is unsupported")
    for field, expected in (
        ("session_id", session_id),
        ("model_generation", model_generation),
        ("reset_generation", reset_generation),
    ):
        if summary.get(field) != expected:
            raise ControlIntentValidationError(
                f"sensor summary {field} does not match status identity"
            )
    required_stream_ids = _sequence(
        summary.get("required_stream_ids"),
        "sensor summary required_stream_ids",
    )
    expected_stream_ids = tuple(
        _safe_id(stream_id, f"sensor summary required_stream_ids[{index}]")
        for index, stream_id in enumerate(required_stream_ids)
    )
    if expected_stream_ids != tuple(sorted(set(expected_stream_ids))):
        raise ControlIntentValidationError(
            "sensor summary required_stream_ids must be unique and in canonical order"
        )
    summary_streams = _mapping(summary.get("streams"), "sensor summary streams")
    missing_streams = [stream_id for stream_id in expected_stream_ids if stream_id not in summary_streams]
    if missing_streams:
        raise ControlIntentValidationError(
            "sensor summary is missing required streams: " + ", ".join(missing_streams)
        )
    blocking = _mapping(
        summary.get("blocking_reasons"),
        "sensor summary blocking_reasons",
    )
    sensors: list[dict[str, Any]] = []
    for stream_id in expected_stream_ids:
        raw = _mapping(summary_streams[stream_id], f"sensor {stream_id}")
        stream_state = _enum(
            raw.get("state"),
            _SENSOR_STATES - {"UNAVAILABLE"},
            f"sensor {stream_id}.state",
        )
        sample_count = _nonnegative_int(
            raw.get("sample_count"),
            f"sensor {stream_id}.sample_count",
        )
        raw_blocker = blocking.get(stream_id)
        blocker = _reason(raw_blocker) if raw_blocker is not None else ""
        if not blocker and (stream_state != "ACTIVE" or sample_count == 0):
            blocker = (
                f"stream is {stream_state}"
                if stream_state != "ACTIVE"
                else "ACTIVE stream has no published sample or frame"
            )
        sensors.append(
            {
                "stream_id": stream_id,
                "state": stream_state,
                "sample_count": sample_count,
                "blocker": blocker,
            }
        )

    return sensors


def encode_control_status(document: Mapping[str, Any]) -> bytes:
    """Validate and encode one exact full-status document."""

    if not isinstance(document, Mapping):
        raise ControlIntentValidationError("control status must be an object")
    _exact_fields(document, _TOP_LEVEL_FIELDS, "control status")
    if document["schema"] != CONTROL_STATUS_SCHEMA:
        raise ControlIntentValidationError(
            f"control status schema must be {CONTROL_STATUS_SCHEMA}"
        )
    identity = OperatorIntentIdentity(
        run_id=document["run_id"],
        session_id=document["session_id"],
        boot_id=document["boot_id"],
        model_generation=document["model_generation"],
        reset_generation=document["reset_generation"],
        source_id=document["source_id"],
    )
    _positive_int(document["server_status_sequence"], "server_status_sequence")
    _nonnegative_int(document["server_monotonic_ns"], "server_monotonic_ns")
    _nonnegative_int(document["sim_time_ns"], "sim_time_ns")
    _nonnegative_int(document["truth_sequence"], "truth_sequence")
    epoch = _positive_int(document["source_epoch"], "source_epoch")
    sequence = _positive_int(document["source_sequence"], "source_sequence")
    _event_id(document["event_id"], identity.boot_id, epoch, sequence)
    _sha256(document["intent_datagram_sha256"], "intent_datagram_sha256")
    status = _enum(document["status"], _ACK_STATUSES, "status")
    reason = _reason(document["reason"])
    if status not in {"accepted", "confirmed"} and not reason:
        raise ControlIntentValidationError(
            f"reason must be non-empty when status is {status}"
        )
    _validate_runtime(document["runtime"])
    _validate_motion(document["motion"])
    _validate_readiness(document["readiness"])
    _validate_sensors(document["sensors"])
    _validate_recording(document["recording"])
    _validate_ui(document["ui"])
    try:
        payload = json.dumps(
            dict(document),
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise ControlIntentValidationError(
            f"control status is not JSON-serializable: {exc}"
        ) from exc
    if len(payload) > MAX_CONTROL_DATAGRAM_BYTES:
        raise ControlIntentValidationError("control status exceeds 4096 bytes")
    return payload


def _build_document(
    control: ControlStatusControlSnapshot,
    authority: Mapping[str, Any],
    *,
    server_status_sequence: int,
    server_monotonic_ns: int,
) -> dict[str, Any]:
    if not isinstance(control, ControlStatusControlSnapshot):
        raise TypeError("control must be ControlStatusControlSnapshot")
    if not isinstance(authority, Mapping):
        raise ControlIntentValidationError("status authority must be an object")
    identity = control.identity
    for field, expected in (
        ("run_id", identity.run_id),
        ("session_id", identity.session_id),
        ("boot_id", identity.boot_id),
        ("model_generation", identity.model_generation),
        ("reset_generation", identity.reset_generation),
    ):
        if authority.get(field) != expected:
            raise ControlIntentValidationError(
                f"status authority {field} does not match control correlation"
            )
    observed = _mapping(
        authority.get("observed_base_velocity_mps_radps"),
        "observed_base_velocity_mps_radps",
    )
    requested = control.requested_axes
    admitted = control.admitted_twist
    return {
        "schema": CONTROL_STATUS_SCHEMA,
        "run_id": identity.run_id,
        "session_id": identity.session_id,
        "boot_id": identity.boot_id,
        "model_generation": identity.model_generation,
        "reset_generation": identity.reset_generation,
        "server_status_sequence": server_status_sequence,
        "server_monotonic_ns": server_monotonic_ns,
        "sim_time_ns": authority.get("sim_time_ns"),
        "truth_sequence": authority.get("truth_sequence"),
        "source_id": identity.source_id,
        "source_epoch": control.source_epoch,
        "source_sequence": control.source_sequence,
        "event_id": control.event_id,
        "intent_datagram_sha256": control.intent_datagram_sha256,
        "status": control.status,
        "reason": control.reason,
        "runtime": {
            "runtime_state": authority.get("runtime_state"),
            "control_owner": control.control_owner,
            "deadman": control.deadman,
            "sample_age_ns": control.sample_age_ns,
            "safe_stop_state": control.safe_stop_state,
        },
        "motion": {
            "requested_axes": {
                "available": control.requested_available,
                "forward": requested[0],
                "left": requested[1],
                "yaw_left": requested[2],
            },
            "admitted_twist_mps_radps": {
                "available": control.admitted_available,
                "linear_x": admitted[0],
                "linear_y": admitted[1],
                "angular_z": admitted[2],
            },
            "observed_base_velocity_mps_radps": dict(observed),
        },
        "readiness": dict(_mapping(authority.get("readiness"), "readiness")),
        "sensors": list(_sequence(authority.get("sensors"), "sensors")),
        "recording": dict(_mapping(authority.get("recording"), "recording")),
        "ui": {
            "ui_mode": control.ui_mode,
            "camera_mode": control.camera_mode,
        },
    }


def _state_key(document: Mapping[str, Any]) -> str:
    stable = {
        "status": document["status"],
        "reason": document["reason"],
        "runtime": {
            "runtime_state": document["runtime"]["runtime_state"],
            "control_owner": document["runtime"]["control_owner"],
            "deadman": document["runtime"]["deadman"],
            "safe_stop_state": document["runtime"]["safe_stop_state"],
        },
        "requested": document["motion"]["requested_axes"],
        "admitted": document["motion"]["admitted_twist_mps_radps"],
        "readiness": document["readiness"],
        "sensor_states": [
            {
                "stream_id": item["stream_id"],
                "state": item["state"],
                "blocker": item["blocker"],
            }
            for item in document["sensors"]
        ],
        "recording": {
            "state": document["recording"]["state"],
            "artifact_id": document["recording"]["artifact_id"],
            "blocker": document["recording"]["blocker"],
        },
        "ui": document["ui"],
    }
    return json.dumps(stable, sort_keys=True, separators=(",", ":"))


def _observed_base_velocity(snapshot: Mapping[str, Any]) -> dict[str, Any]:
    bodies = snapshot.get("bodies")
    if not isinstance(bodies, list):
        return {
            "available": False,
            "linear_x": 0.0,
            "linear_y": 0.0,
            "angular_z": 0.0,
        }
    matches = [
        body
        for body in bodies
        if isinstance(body, Mapping)
        and body.get("stable_id") == "thunder_01/base_link"
    ]
    if len(matches) != 1:
        return {
            "available": False,
            "linear_x": 0.0,
            "linear_y": 0.0,
            "angular_z": 0.0,
        }
    body = matches[0]
    linear = body.get("linear_velocity_mps")
    angular = body.get("angular_velocity_rps")
    if (
        not isinstance(linear, (list, tuple))
        or len(linear) != 3
        or not isinstance(angular, (list, tuple))
        or len(angular) != 3
    ):
        return {
            "available": False,
            "linear_x": 0.0,
            "linear_y": 0.0,
            "angular_z": 0.0,
        }
    values = (
        _finite_number(linear[0], "base.linear_velocity_mps[0]"),
        _finite_number(linear[1], "base.linear_velocity_mps[1]"),
        _finite_number(angular[2], "base.angular_velocity_rps[2]"),
    )
    return {
        "available": True,
        "linear_x": values[0],
        "linear_y": values[1],
        "angular_z": values[2],
    }


def _recording_authority(
    snapshot: Mapping[str, Any] | None,
    *,
    run_id: str,
    session_id: str,
    model_generation: int,
    reset_generation: int,
    sim_time_ns: int,
) -> dict[str, Any]:
    if snapshot is None:
        return {
            "state": "unavailable",
            "elapsed_sim_time_ns": 0,
            "artifact_id": "",
            "blocker": "recording status source unavailable",
        }
    recording = _mapping(snapshot, "recording_snapshot")
    _exact_fields(
        recording,
        _RECORDING_SOURCE_FIELDS,
        "recording_snapshot",
    )
    if recording.get("schema") != "lingtu.sim.recording-status.v1":
        raise ControlIntentValidationError(
            "recording snapshot schema is unsupported"
        )
    for field, expected in (
        ("run_id", run_id),
        ("session_id", session_id),
        ("model_generation", model_generation),
        ("reset_generation", reset_generation),
    ):
        if recording.get(field) != expected:
            raise ControlIntentValidationError(
                f"recording snapshot {field} does not match status identity"
            )
    state = _enum(
        recording.get("state"),
        _DURABLE_RECORDING_STATES,
        "recording_snapshot.state",
    )
    elapsed = _nonnegative_int(
        recording.get("elapsed_sim_time_ns"),
        "recording_snapshot.elapsed_sim_time_ns",
    )
    if elapsed > sim_time_ns:
        raise ControlIntentValidationError(
            "recording elapsed_sim_time_ns exceeds current simulation time"
        )
    artifact = recording.get("artifact_id")
    if not isinstance(artifact, str):
        raise ControlIntentValidationError(
            "recording_snapshot.artifact_id must be a string"
        )
    if artifact:
        _safe_id(artifact, "recording_snapshot.artifact_id")
    if state == "committed" and not artifact:
        raise ControlIntentValidationError(
            "committed recording snapshot requires artifact_id"
        )
    if state == "unavailable" and (elapsed != 0 or artifact):
        raise ControlIntentValidationError(
            "unavailable recording snapshot must have zero elapsed time and "
            "empty artifact_id"
        )
    blocker = _reason(recording.get("blocker"))
    if state in {"unavailable", "rejected", "failed"} and not blocker:
        raise ControlIntentValidationError(
            f"recording snapshot blocker is required when state is {state}"
        )
    if state not in {"unavailable", "rejected", "failed"} and blocker:
        raise ControlIntentValidationError(
            f"recording snapshot blocker must be empty when state is {state}"
        )
    return {
        "state": state,
        "elapsed_sim_time_ns": elapsed,
        "artifact_id": artifact,
        "blocker": blocker,
    }


def _validate_runtime(value: Any) -> None:
    runtime = _mapping(value, "runtime")
    _exact_fields(runtime, _RUNTIME_FIELDS, "runtime")
    _enum(runtime["runtime_state"], _RUNTIME_STATES, "runtime.runtime_state")
    owner = runtime["control_owner"]
    if owner != "unavailable":
        _safe_id(owner, "runtime.control_owner")
    _boolean(runtime["deadman"], "runtime.deadman")
    _nonnegative_int(runtime["sample_age_ns"], "runtime.sample_age_ns")
    _enum(
        runtime["safe_stop_state"],
        _SAFE_STOP_STATES,
        "runtime.safe_stop_state",
    )


def _validate_motion(value: Any) -> None:
    motion = _mapping(value, "motion")
    _exact_fields(motion, _MOTION_FIELDS, "motion")
    requested = _mapping(motion["requested_axes"], "motion.requested_axes")
    _exact_fields(requested, _REQUESTED_FIELDS, "motion.requested_axes")
    _boolean(requested["available"], "motion.requested_axes.available")
    for field in ("forward", "left", "yaw_left"):
        _finite_number(requested[field], f"motion.requested_axes.{field}")
    if not requested["available"] and any(
        requested[field] != 0 for field in ("forward", "left", "yaw_left")
    ):
        raise ControlIntentValidationError(
            "unavailable requested axes must contain exact zeros"
        )
    for name in (
        "admitted_twist_mps_radps",
        "observed_base_velocity_mps_radps",
    ):
        velocity = _mapping(motion[name], f"motion.{name}")
        _exact_fields(velocity, _VELOCITY_FIELDS, f"motion.{name}")
        _boolean(velocity["available"], f"motion.{name}.available")
        for field in ("linear_x", "linear_y", "angular_z"):
            _finite_number(velocity[field], f"motion.{name}.{field}")
        if not velocity["available"] and any(
            velocity[field] != 0
            for field in ("linear_x", "linear_y", "angular_z")
        ):
            raise ControlIntentValidationError(
                f"unavailable {name} must contain exact zeros"
            )


def _validate_readiness(value: Any) -> None:
    readiness = _mapping(value, "readiness")
    _exact_fields(readiness, _READINESS_FIELDS, "readiness")
    for name in sorted(_READINESS_FIELDS):
        facet = _mapping(readiness[name], f"readiness.{name}")
        _exact_fields(facet, _FACET_FIELDS, f"readiness.{name}")
        state = _enum(facet["state"], _BINDING_STATES, f"readiness.{name}.state")
        _boolean(facet["required"], f"readiness.{name}.required")
        source = facet["source_id"]
        if source != "unavailable":
            _safe_id(source, f"readiness.{name}.source_id")
        blocker = _reason(facet["blocker"])
        if state != "ACTIVE" and not blocker:
            raise ControlIntentValidationError(
                f"readiness.{name}.blocker is required unless ACTIVE"
            )


def _validate_sensors(value: Any) -> None:
    sensors = _sequence(value, "sensors")
    actual: list[str] = []
    for index, raw in enumerate(sensors):
        sensor = _mapping(raw, f"sensors[{index}]")
        _exact_fields(sensor, _SENSOR_FIELDS, f"sensors[{index}]")
        stream_id = _safe_id(sensor["stream_id"], f"sensors[{index}].stream_id")
        actual.append(stream_id)
        state = _enum(sensor["state"], _SENSOR_STATES, f"sensors[{index}].state")
        _nonnegative_int(sensor["sample_count"], f"sensors[{index}].sample_count")
        blocker = _reason(sensor["blocker"])
        if (state != "ACTIVE" or sensor["sample_count"] == 0) and not blocker:
            raise ControlIntentValidationError(
                f"sensors[{index}].blocker is required until ACTIVE with samples"
            )
    if tuple(actual) != tuple(sorted(set(actual))):
        raise ControlIntentValidationError(
            "sensors must contain unique stream ids in canonical order"
        )


def _validate_recording(value: Any) -> None:
    recording = _mapping(value, "recording")
    _exact_fields(recording, _RECORDING_FIELDS, "recording")
    state = _enum(recording["state"], _RECORDING_STATES, "recording.state")
    elapsed = _nonnegative_int(
        recording["elapsed_sim_time_ns"],
        "recording.elapsed_sim_time_ns",
    )
    artifact_id = recording["artifact_id"]
    if artifact_id:
        _safe_id(artifact_id, "recording.artifact_id")
    elif not isinstance(artifact_id, str):
        raise ControlIntentValidationError("recording.artifact_id must be a string")
    if state == "unavailable" and (elapsed != 0 or artifact_id):
        raise ControlIntentValidationError(
            "unavailable recording must have zero elapsed time and empty artifact_id"
        )
    if state == "committed" and not artifact_id:
        raise ControlIntentValidationError(
            "committed recording requires artifact_id"
        )
    blocker = _reason(recording["blocker"])
    if state in {"unavailable", "rejected", "failed"} and not blocker:
        raise ControlIntentValidationError(
            f"recording.blocker is required when state is {state}"
        )
    if state not in {"unavailable", "rejected", "failed"} and blocker:
        raise ControlIntentValidationError(
            f"recording.blocker must be empty when state is {state}"
        )


def _validate_ui(value: Any) -> None:
    ui = _mapping(value, "ui")
    _exact_fields(ui, _UI_FIELDS, "ui")
    _enum(ui["ui_mode"], _UI_MODES, "ui.ui_mode")
    _enum(ui["camera_mode"], _CAMERA_MODES, "ui.camera_mode")


def _exact_fields(value: Mapping[str, Any], expected: frozenset[str], context: str) -> None:
    unknown = sorted(set(value) - expected)
    if unknown:
        raise ControlIntentValidationError(
            f"{context} has unknown field(s): {', '.join(unknown)}"
        )
    missing = sorted(expected - set(value))
    if missing:
        raise ControlIntentValidationError(
            f"{context} is missing required field(s): {', '.join(missing)}"
        )


def _mapping(value: Any, field: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ControlIntentValidationError(f"{field} must be an object")
    return value


def _sequence(value: Any, field: str) -> Sequence[Any]:
    if not isinstance(value, (list, tuple)):
        raise ControlIntentValidationError(f"{field} must be an array")
    return value


def _enum(value: Any, allowed: frozenset[str], field: str) -> str:
    if not isinstance(value, str) or value not in allowed:
        raise ControlIntentValidationError(
            f"{field} must be one of: {', '.join(sorted(allowed))}"
        )
    return value


def _boolean(value: Any, field: str) -> bool:
    if not isinstance(value, bool):
        raise ControlIntentValidationError(f"{field} must be a boolean")
    return value


def _nonnegative_int(value: Any, field: str) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 0 <= value <= _MAX_EXACT_JSON_INT
    ):
        raise ControlIntentValidationError(
            f"{field} must be a non-negative exact JSON integer"
        )
    return value


def _positive_int(value: Any, field: str) -> int:
    result = _nonnegative_int(value, field)
    if result == 0:
        raise ControlIntentValidationError(f"{field} must be positive")
    return result


def _finite_number(value: Any, field: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ControlIntentValidationError(f"{field} must be a finite number")
    return float(value)


def _vector(value: object, field: str) -> tuple[float, float, float]:
    if not isinstance(value, tuple) or len(value) != 3:
        raise ControlIntentValidationError(f"{field} must be a three-value tuple")
    return tuple(_finite_number(item, f"{field}[{index}]") for index, item in enumerate(value))  # type: ignore[return-value]


def _safe_id(value: Any, field: str) -> str:
    if not isinstance(value, str) or _SAFE_ID_RE.fullmatch(value) is None:
        raise ControlIntentValidationError(f"{field} is not a safe identifier")
    return value


def _sha256(value: Any, field: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise ControlIntentValidationError(
            f"{field} must be a lowercase SHA-256 digest"
        )
    return value


def _event_id(value: Any, boot_id: str, epoch: int, sequence: int) -> str:
    expected = f"{boot_id}:{epoch}:{sequence}"
    if value != expected:
        raise ControlIntentValidationError(
            "event_id must equal <boot_id>:<source_epoch>:<source_sequence>"
        )
    return expected


def _reason(value: Any) -> str:
    if not isinstance(value, str):
        raise ControlIntentValidationError("reason/blocker must be a string")
    if len(value.encode("utf-8")) > 512:
        raise ControlIntentValidationError("reason/blocker exceeds 512 UTF-8 bytes")
    if value and value != value.strip():
        raise ControlIntentValidationError("reason/blocker must be trimmed")
    if any(ord(character) < 32 or ord(character) == 127 for character in value):
        raise ControlIntentValidationError("reason/blocker contains a control character")
    return value


__all__ = [
    "CONTROL_STATUS_PERIOD_NS",
    "CONTROL_STATUS_SCHEMA",
    "ControlStatusControlSnapshot",
    "ControlStatusReporter",
    "build_control_status_authority",
    "encode_control_status",
]
