"""Atomic full-state simulation recording for deterministic replay."""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import shutil
import tempfile
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .typed_dds_payload import TypedDdsPayloadSample

RECORDING_SCHEMA = "lingtu.sim.recording.v1"
RECORDING_FRAME_SCHEMA = "lingtu.sim.recording-frame.v1"
RECORDING_FILENAME = "simulation-recording.json"
TIMELINE_FILENAME = "simulation-timeline.jsonl"
SENSOR_PAYLOAD_REFERENCE_SCHEMA = "lingtu.sim.sensor-payload-ref.v1"
SENSOR_PAYLOAD_STORE_SCHEMA = "lingtu.sim.sensor-payload-store.v1"
SENSOR_PAYLOAD_ROOT = "sensor-payloads/sha256"
TYPED_DDS_PAYLOAD_REFERENCE_SCHEMA = "lingtu.sim.typed-dds-payload-ref.v1"
TYPED_DDS_PAYLOAD_STORE_SCHEMA = "lingtu.sim.typed-dds-payload-store.v1"
TYPED_DDS_PAYLOAD_ROOT = "typed-dds-payloads/sha256"
MAX_SENSOR_PAYLOAD_BYTES = 64 * 1024 * 1024

_RUN_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z")
_CONTENT_STREAMS = (
    "command",
    "truth_snapshot",
    "scenario_event",
    "sensor_metadata",
    "sensor_payload",
    "typed_dds_payload",
    "lifecycle_evidence",
)
_TERMINAL_STATES = frozenset({"STOPPED", "FAILED"})


class SimulationRecordingError(RuntimeError):
    """Fail-closed error at the simulation recording seam."""


@dataclass(frozen=True)
class SensorPayloadSample:
    """One immutable sensor payload supplied to the recording boundary."""

    sensor_id: str
    stream_kind: str
    encoding: str
    media_type: str
    sample_sequence: int
    sample_time_ns: int
    payload: bytes
    metadata: Mapping[str, Any] | None = None


def _non_negative_integer(value: object, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise SimulationRecordingError(f"snapshot {field} must be a non-negative integer")
    return value


def _json_value(value: object, field: str) -> Any:
    try:
        encoded = json.dumps(
            value,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
        return json.loads(encoded)
    except (TypeError, ValueError) as exc:
        raise SimulationRecordingError(f"{field} must be finite JSON data") from exc


def _truth_snapshot(event: Mapping[str, Any]) -> dict[str, Any]:
    if event.get("event") != "snapshot":
        raise SimulationRecordingError("recording accepts snapshot events only")
    required = (
        "session_id",
        "model_generation",
        "reset_generation",
        "sequence",
        "physics_step",
        "sim_time_ns",
        "bodies",
    )
    missing = [field for field in required if field not in event]
    if missing:
        raise SimulationRecordingError(
            "snapshot is missing required field(s): " + ", ".join(missing)
        )
    snapshot = {"schema": "lingtu.sim.truth-snapshot.v1"}
    snapshot.update({key: value for key, value in event.items() if key != "event"})
    value = _json_value(snapshot, "snapshot")
    if not isinstance(value, dict):  # pragma: no cover - guaranteed by construction
        raise SimulationRecordingError("snapshot must be an object")
    return value


def _canonical_json(value: object) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def _document_digest(value: object) -> str:
    return hashlib.sha256(_canonical_json(value)).hexdigest()


def _native_filesystem_path(path: Path) -> Path:
    """Use the Win32 extended path form for deep run-owned artifacts."""

    absolute = Path(path).absolute()
    if os.name != "nt":
        return absolute
    value = str(absolute)
    if value.startswith("\\\\?\\"):
        return absolute
    if value.startswith("\\\\"):
        return Path("\\\\?\\UNC\\" + value[2:])
    return Path("\\\\?\\" + value)


def _context_document(value: object, field: str) -> dict[str, Any]:
    candidate = value
    to_dict = getattr(value, "to_dict", None)
    if callable(to_dict):
        candidate = to_dict()
    normalized = _json_value(candidate, field)
    if not isinstance(normalized, dict):
        raise SimulationRecordingError(f"{field} must be an object")
    return normalized


def _run_allocation_document(
    value: object | None,
    *,
    run_id: str,
    session_id: str,
) -> dict[str, Any] | None:
    if value is None:
        return None
    document = _context_document(value, "run_allocation")
    if document.get("schema") != "lingtu.sim.run-allocation.v1":
        raise SimulationRecordingError("run_allocation schema is invalid")
    if document.get("run_id") != run_id:
        raise SimulationRecordingError("run_allocation run_id does not match the recording")
    if document.get("session_id") != session_id:
        raise SimulationRecordingError(
            "run_allocation session_id does not match the recording"
        )
    return document


def _descriptor_documents(
    value: Mapping[str, object] | None,
    *,
    session_id: str,
) -> dict[str, dict[str, Any]] | None:
    if value is None:
        return None
    if not isinstance(value, Mapping) or not value:
        raise SimulationRecordingError("descriptors must be a non-empty object")
    documents: dict[str, dict[str, Any]] = {}
    for name, descriptor in sorted(value.items()):
        if not isinstance(name, str) or not name or name != name.strip():
            raise SimulationRecordingError("descriptor name must be non-empty trimmed text")
        document = _context_document(descriptor, f"descriptor {name}")
        descriptor_session_id = document.get("session_id")
        if descriptor_session_id is not None and descriptor_session_id != session_id:
            raise SimulationRecordingError(
                f"descriptor {name} session_id does not match the recording"
            )
        documents[name] = document
    return documents


def _required_content(value: Iterable[str]) -> frozenset[str]:
    try:
        result = frozenset(value)
    except TypeError as exc:
        raise SimulationRecordingError("required_content must contain stream names") from exc
    invalid = sorted(
        item for item in result if not isinstance(item, str) or item not in _CONTENT_STREAMS
    )
    if invalid:
        raise SimulationRecordingError(
            "required_content contains unsupported stream(s): " + ", ".join(map(str, invalid))
        )
    return result


def _continuous_tolerances(value: Mapping[str, object] | None) -> dict[str, float]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise SimulationRecordingError("continuous_tolerances must be an object")
    result: dict[str, float] = {}
    for path, tolerance in sorted(value.items()):
        if (
            not isinstance(path, str)
            or not path.startswith("/")
            or path != path.strip()
        ):
            raise SimulationRecordingError(
                "continuous tolerance paths must be absolute JSON pointer patterns"
            )
        if (
            isinstance(tolerance, bool)
            or not isinstance(tolerance, (int, float))
            or not math.isfinite(float(tolerance))
            or float(tolerance) < 0.0
        ):
            raise SimulationRecordingError(
                f"continuous tolerance for {path} must be non-negative finite numeric data"
            )
        result[path] = float(tolerance)
    return result


def _evidence_records(
    value: Sequence[Mapping[str, Any]] | None,
    *,
    field: str,
    snapshot: Mapping[str, Any],
) -> list[dict[str, Any]]:
    if value is None:
        return []
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(value, Sequence):
        raise SimulationRecordingError(f"{field} must be an ordered array")
    records: list[dict[str, Any]] = []
    for index, item in enumerate(value):
        document = _context_document(item, f"{field}[{index}]")
        for identity_field in (
            "session_id",
            "model_generation",
            "reset_generation",
        ):
            if identity_field not in document:
                raise SimulationRecordingError(
                    f"{field}[{index}] is missing {identity_field}"
                )
            if document[identity_field] != snapshot[identity_field]:
                raise SimulationRecordingError(
                    f"{field}[{index}] {identity_field} does not match its truth snapshot"
                )
        records.append(document)
    return records


class SimulationRecordingWriter:
    """Write one replayable timeline and publish its manifest as the commit marker."""

    def __init__(
        self,
        run_dir: Path,
        *,
        run_id: str,
        session_id: str,
        run_allocation: object | None = None,
        descriptors: Mapping[str, object] | None = None,
        required_content: Iterable[str] = (),
        continuous_tolerances: Mapping[str, object] | None = None,
    ) -> None:
        if not isinstance(run_id, str) or _RUN_ID_RE.fullmatch(run_id) is None:
            raise ValueError("run_id has an invalid value")
        if not isinstance(session_id, str) or not session_id.strip():
            raise ValueError("session_id must be non-empty")
        self.run_dir = Path(run_dir).resolve()
        self.run_id = run_id
        self.session_id = session_id
        self._run_allocation = _run_allocation_document(
            run_allocation,
            run_id=run_id,
            session_id=session_id,
        )
        self._descriptors = _descriptor_documents(
            descriptors,
            session_id=session_id,
        )
        self._required_content = _required_content(required_content)
        self._continuous_tolerances = _continuous_tolerances(continuous_tolerances)
        self.manifest_path = self.run_dir / RECORDING_FILENAME
        self.timeline_path = self.run_dir / TIMELINE_FILENAME
        self.payload_root = self.run_dir / Path(SENSOR_PAYLOAD_ROOT)
        self.typed_dds_payload_root = self.run_dir / Path(TYPED_DDS_PAYLOAD_ROOT)
        self._temporary_path: Path | None = None
        self._stream: Any = None
        self._hasher = hashlib.sha256()
        self._event_order_hasher = hashlib.sha256()
        self._stream_hashers = {
            name: hashlib.sha256() for name in _CONTENT_STREAMS
        }
        self._stream_counts = dict.fromkeys(_CONTENT_STREAMS, 0)
        self._event_count = 0
        self._bytes_written = 0
        self._frame_count = 0
        self._model_generation: int | None = None
        self._start_reset_generation: int | None = None
        self._end_reset_generation: int | None = None
        self._start_sim_time_ns: int | None = None
        self._end_sim_time_ns: int | None = None
        self._relative_time_ns = 0
        self._previous_sequence: int | None = None
        self._previous_physics_step: int | None = None
        self._previous_sim_time_ns: int | None = None
        self._terminal_state: str | None = None
        self._payload_reference_count = 0
        self._payload_referenced_bytes = 0
        self._payload_unique_bytes = 0
        self._payload_digests: set[str] = set()
        self._typed_dds_reference_count = 0
        self._typed_dds_referenced_bytes = 0
        self._typed_dds_unique_bytes = 0
        self._typed_dds_digests: set[str] = set()
        self._typed_dds_last_sample_by_route: dict[
            tuple[int, str], tuple[str, str, int, int]
        ] = {}
        self._closed = False

    def __enter__(self) -> SimulationRecordingWriter:
        self._open()
        return self

    def __exit__(self, error_type: object, _error: object, _traceback: object) -> None:
        if error_type is None:
            self.close()
        else:
            self.abort()

    def _open(self) -> None:
        if self._stream is not None:
            return
        if self._closed:
            raise SimulationRecordingError("recording writer is already closed")
        self.run_dir.mkdir(parents=True, exist_ok=True)
        if (
            self.manifest_path.exists()
            or self.timeline_path.exists()
            or self.payload_root.exists()
            or self.typed_dds_payload_root.exists()
        ):
            raise SimulationRecordingError("recording artifacts already exist")
        descriptor, name = tempfile.mkstemp(
            dir=self.run_dir,
            prefix=f".{TIMELINE_FILENAME}.",
            suffix=".tmp",
        )
        self._temporary_path = Path(name)
        self._stream = os.fdopen(descriptor, "wb")

    def append(
        self,
        snapshot_event: Mapping[str, Any],
        *,
        command: Mapping[str, Any] | None = None,
        metadata: Mapping[str, Any] | None = None,
        scenario_events: Sequence[Mapping[str, Any]] | None = None,
        sensor_metadata: Sequence[Mapping[str, Any]] | None = None,
        sensor_payloads: Sequence[SensorPayloadSample] | None = None,
        typed_dds_payloads: Sequence[TypedDdsPayloadSample] | None = None,
        lifecycle_evidence: Sequence[Mapping[str, Any]] | None = None,
    ) -> None:
        """Append one complete immutable truth snapshot and its applied command."""

        self._open()
        snapshot = _truth_snapshot(snapshot_event)
        if snapshot["session_id"] != self.session_id:
            raise SimulationRecordingError("snapshot session_id does not match the recording")
        model_generation = _non_negative_integer(snapshot["model_generation"], "model_generation")
        reset_generation = _non_negative_integer(snapshot["reset_generation"], "reset_generation")
        sequence = _non_negative_integer(snapshot["sequence"], "sequence")
        physics_step = _non_negative_integer(snapshot["physics_step"], "physics_step")
        sim_time_ns = _non_negative_integer(snapshot["sim_time_ns"], "sim_time_ns")
        command_value = _json_value(command, "command") if command is not None else None
        if command_value is not None and not isinstance(command_value, dict):
            raise SimulationRecordingError("command must be an object")
        metadata_value = _json_value(metadata, "metadata") if metadata is not None else {}
        if not isinstance(metadata_value, dict):
            raise SimulationRecordingError("metadata must be an object")
        scenario_values = _evidence_records(
            scenario_events,
            field="scenario_events",
            snapshot=snapshot,
        )
        sensor_values = _evidence_records(
            sensor_metadata,
            field="sensor_metadata",
            snapshot=snapshot,
        )
        lifecycle_values = _evidence_records(
            lifecycle_evidence,
            field="lifecycle_evidence",
            snapshot=snapshot,
        )

        if self._model_generation is None:
            self._model_generation = model_generation
            self._start_reset_generation = reset_generation
            self._start_sim_time_ns = sim_time_ns
            self._validate_descriptor_generations(model_generation, reset_generation)
        elif model_generation != self._model_generation:
            raise SimulationRecordingError("model_generation changed inside one recording")
        elif self._end_reset_generation is not None:
            if reset_generation == self._end_reset_generation:
                if self._previous_sequence is not None and sequence <= self._previous_sequence:
                    raise SimulationRecordingError("snapshot sequence must increase within a reset generation")
                if self._previous_physics_step is not None and physics_step <= self._previous_physics_step:
                    raise SimulationRecordingError("physics_step must increase within a reset generation")
                if self._previous_sim_time_ns is not None:
                    if sim_time_ns < self._previous_sim_time_ns:
                        raise SimulationRecordingError("sim_time_ns moved backwards within a reset generation")
                    self._relative_time_ns += sim_time_ns - self._previous_sim_time_ns
            elif reset_generation != self._end_reset_generation + 1:
                raise SimulationRecordingError("reset_generation must remain stable or increase by one")

        self._end_reset_generation = reset_generation
        self._end_sim_time_ns = sim_time_ns
        self._previous_sequence = sequence
        self._previous_physics_step = physics_step
        self._previous_sim_time_ns = sim_time_ns
        payload_references = self._store_sensor_payloads(
            sensor_payloads,
            snapshot=snapshot,
        )
        typed_dds_references = self._store_typed_dds_payloads(
            typed_dds_payloads,
            snapshot=snapshot,
        )
        evidence = {
            "scenario_events": scenario_values,
            "sensor_metadata": sensor_values,
            "lifecycle_evidence": lifecycle_values,
        }
        if payload_references:
            evidence["sensor_payloads"] = payload_references
        if typed_dds_references:
            evidence["typed_dds_payloads"] = typed_dds_references
        frame = {
            "schema": RECORDING_FRAME_SCHEMA,
            "frame_index": self._frame_count,
            "relative_time_ns": self._relative_time_ns,
            "snapshot": snapshot,
            "command": command_value,
            "metadata": metadata_value,
            "evidence": evidence,
        }
        if command_value is not None:
            self._track_content("command", command_value)
        for event in scenario_values:
            self._track_content("scenario_event", event)
        self._track_content("truth_snapshot", snapshot)
        for sample in sensor_values:
            self._track_content("sensor_metadata", sample)
        for reference in payload_references:
            self._track_content("sensor_payload", reference)
        for reference in typed_dds_references:
            self._track_content("typed_dds_payload", reference)
        for evidence in lifecycle_values:
            state = evidence.get("state")
            if not isinstance(state, str) or not state or state != state.strip():
                raise SimulationRecordingError(
                    "lifecycle evidence state must be non-empty trimmed text"
                )
            self._terminal_state = state
            self._track_content("lifecycle_evidence", evidence)
        payload = (
            json.dumps(
                frame,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8")
        self._stream.write(payload)
        self._stream.flush()
        self._hasher.update(payload)
        self._bytes_written += len(payload)
        self._frame_count += 1

    def _store_sensor_payloads(
        self,
        samples: Sequence[SensorPayloadSample] | None,
        *,
        snapshot: Mapping[str, Any],
    ) -> list[dict[str, Any]]:
        if samples is None:
            return []
        if isinstance(samples, (str, bytes, bytearray)) or not isinstance(
            samples, Sequence
        ):
            raise SimulationRecordingError("sensor_payloads must be an ordered array")
        references: list[dict[str, Any]] = []
        for index, sample in enumerate(samples):
            if not isinstance(sample, SensorPayloadSample):
                raise SimulationRecordingError(
                    f"sensor_payloads[{index}] must be a SensorPayloadSample"
                )
            sensor_id = self._payload_text(sample.sensor_id, f"sensor_payloads[{index}].sensor_id")
            stream_kind = self._payload_text(
                sample.stream_kind,
                f"sensor_payloads[{index}].stream_kind",
            )
            encoding = self._payload_text(sample.encoding, f"sensor_payloads[{index}].encoding")
            media_type = self._payload_text(
                sample.media_type,
                f"sensor_payloads[{index}].media_type",
            )
            sample_sequence = _non_negative_integer(
                sample.sample_sequence,
                f"sensor_payloads[{index}].sample_sequence",
            )
            sample_time_ns = _non_negative_integer(
                sample.sample_time_ns,
                f"sensor_payloads[{index}].sample_time_ns",
            )
            if not isinstance(sample.payload, (bytes, bytearray, memoryview)):
                raise SimulationRecordingError(
                    f"sensor_payloads[{index}].payload must be bytes"
                )
            payload = bytes(sample.payload)
            if not payload or len(payload) > MAX_SENSOR_PAYLOAD_BYTES:
                raise SimulationRecordingError(
                    f"sensor_payloads[{index}].payload must contain 1..{MAX_SENSOR_PAYLOAD_BYTES} bytes"
                )
            metadata = _json_value(
                sample.metadata if sample.metadata is not None else {},
                f"sensor_payloads[{index}].metadata",
            )
            if not isinstance(metadata, dict):
                raise SimulationRecordingError(
                    f"sensor_payloads[{index}].metadata must be an object"
                )
            digest = hashlib.sha256(payload).hexdigest()
            relative_path = (
                Path(SENSOR_PAYLOAD_ROOT) / digest[:2] / f"{digest}.bin"
            )
            self._write_sensor_payload_blob(relative_path, payload, digest=digest)
            if digest not in self._payload_digests:
                self._payload_digests.add(digest)
                self._payload_unique_bytes += len(payload)
            self._payload_reference_count += 1
            self._payload_referenced_bytes += len(payload)
            references.append(
                {
                    "schema": SENSOR_PAYLOAD_REFERENCE_SCHEMA,
                    "session_id": snapshot["session_id"],
                    "model_generation": snapshot["model_generation"],
                    "reset_generation": snapshot["reset_generation"],
                    "sensor_id": sensor_id,
                    "stream_kind": stream_kind,
                    "encoding": encoding,
                    "media_type": media_type,
                    "sample_sequence": sample_sequence,
                    "sample_time_ns": sample_time_ns,
                    "path": relative_path.as_posix(),
                    "sha256": digest,
                    "bytes": len(payload),
                    "metadata": metadata,
                }
            )
        return references

    def _store_typed_dds_payloads(
        self,
        samples: Sequence[TypedDdsPayloadSample] | None,
        *,
        snapshot: Mapping[str, Any],
    ) -> list[dict[str, Any]]:
        if samples is None:
            return []
        if isinstance(samples, (str, bytes, bytearray)) or not isinstance(samples, Sequence):
            raise SimulationRecordingError("typed_dds_payloads must be an ordered array")
        references: list[dict[str, Any]] = []
        for index, sample in enumerate(samples):
            if not isinstance(sample, TypedDdsPayloadSample):
                raise SimulationRecordingError(
                    f"typed_dds_payloads[{index}] must be a TypedDdsPayloadSample"
                )
            topic = self._payload_text(sample.topic, f"typed_dds_payloads[{index}].topic")
            type_name = self._payload_text(
                sample.type_name, f"typed_dds_payloads[{index}].type_name"
            )
            encoding = self._payload_text(
                sample.encoding, f"typed_dds_payloads[{index}].encoding"
            )
            model_generation = _non_negative_integer(
                sample.model_generation,
                f"typed_dds_payloads[{index}].model_generation",
            )
            reset_generation = _non_negative_integer(
                sample.reset_generation,
                f"typed_dds_payloads[{index}].reset_generation",
            )
            sequence = _non_negative_integer(
                sample.sequence, f"typed_dds_payloads[{index}].sequence"
            )
            sim_time_ns = _non_negative_integer(
                sample.sim_time_ns, f"typed_dds_payloads[{index}].sim_time_ns"
            )
            if model_generation != snapshot["model_generation"]:
                raise SimulationRecordingError(
                    f"typed_dds_payloads[{index}] model_generation does not match its truth snapshot"
                )
            if reset_generation != snapshot["reset_generation"]:
                raise SimulationRecordingError(
                    f"typed_dds_payloads[{index}] reset_generation does not match its truth snapshot"
                )
            if sim_time_ns > snapshot["sim_time_ns"]:
                raise SimulationRecordingError(
                    f"typed_dds_payloads[{index}] sim_time_ns is after its truth snapshot"
                )
            route = (reset_generation, topic)
            previous = self._typed_dds_last_sample_by_route.get(route)
            if previous is not None:
                previous_type, previous_encoding, previous_sequence, previous_time_ns = previous
                if (type_name, encoding) != (previous_type, previous_encoding):
                    raise SimulationRecordingError(
                        f"typed_dds_payloads[{index}] type or encoding changed within one topic generation"
                    )
                if sequence <= previous_sequence:
                    raise SimulationRecordingError(
                        f"typed_dds_payloads[{index}] sequence must increase within one topic generation"
                    )
                if sim_time_ns < previous_time_ns:
                    raise SimulationRecordingError(
                        f"typed_dds_payloads[{index}] sim_time_ns moved backwards within one topic generation"
                    )
            if not isinstance(sample.payload, (bytes, bytearray, memoryview)):
                raise SimulationRecordingError(
                    f"typed_dds_payloads[{index}].payload must be bytes"
                )
            payload = bytes(sample.payload)
            if not payload or len(payload) > MAX_SENSOR_PAYLOAD_BYTES:
                raise SimulationRecordingError(
                    f"typed_dds_payloads[{index}].payload must contain 1..{MAX_SENSOR_PAYLOAD_BYTES} bytes"
                )
            metadata = _json_value(
                sample.metadata if sample.metadata is not None else {},
                f"typed_dds_payloads[{index}].metadata",
            )
            if not isinstance(metadata, dict):
                raise SimulationRecordingError(
                    f"typed_dds_payloads[{index}].metadata must be an object"
                )
            digest = hashlib.sha256(payload).hexdigest()
            relative_path = Path(TYPED_DDS_PAYLOAD_ROOT) / digest[:2] / f"{digest}.bin"
            self._write_content_addressed_blob(
                relative_path,
                payload,
                digest=digest,
                label="typed DDS payload",
            )
            if digest not in self._typed_dds_digests:
                self._typed_dds_digests.add(digest)
                self._typed_dds_unique_bytes += len(payload)
            self._typed_dds_reference_count += 1
            self._typed_dds_referenced_bytes += len(payload)
            self._typed_dds_last_sample_by_route[route] = (
                type_name,
                encoding,
                sequence,
                sim_time_ns,
            )
            references.append(
                {
                    "schema": TYPED_DDS_PAYLOAD_REFERENCE_SCHEMA,
                    "session_id": snapshot["session_id"],
                    "model_generation": model_generation,
                    "reset_generation": reset_generation,
                    "topic": topic,
                    "type_name": type_name,
                    "encoding": encoding,
                    "sequence": sequence,
                    "sim_time_ns": sim_time_ns,
                    "path": relative_path.as_posix(),
                    "sha256": digest,
                    "bytes": len(payload),
                    "metadata": metadata,
                }
            )
        return references

    def _write_sensor_payload_blob(
        self,
        relative_path: Path,
        payload: bytes,
        *,
        digest: str,
    ) -> None:
        self._write_content_addressed_blob(
            relative_path, payload, digest=digest, label="sensor payload"
        )

    def _write_content_addressed_blob(
        self,
        relative_path: Path,
        payload: bytes,
        *,
        digest: str,
        label: str,
    ) -> None:
        target = self.run_dir / relative_path
        native_target = _native_filesystem_path(target)
        native_target.parent.mkdir(parents=True, exist_ok=True)
        if native_target.exists():
            try:
                existing = native_target.read_bytes()
            except OSError as exc:
                raise SimulationRecordingError(
                    f"cannot validate an existing {label} blob"
                ) from exc
            if len(existing) != len(payload) or hashlib.sha256(existing).hexdigest() != digest:
                raise SimulationRecordingError(
                    f"content-addressed {label} blob does not match its digest"
                )
            return
        descriptor, name = tempfile.mkstemp(
            dir=native_target.parent,
            prefix=".payload-",
            suffix=".tmp",
        )
        temporary = Path(name)
        try:
            with os.fdopen(descriptor, "wb") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            from sim.runtime.coordinator.atomic_file import replace_file_with_retry

            replace_file_with_retry(
                _native_filesystem_path(temporary),
                native_target,
            )
        finally:
            temporary.unlink(missing_ok=True)

    @staticmethod
    def _payload_text(value: object, field: str) -> str:
        if (
            not isinstance(value, str)
            or not value
            or value != value.strip()
            or len(value) > 255
            or any(ord(character) < 32 for character in value)
        ):
            raise SimulationRecordingError(f"{field} must be bounded non-empty text")
        return value

    def _validate_descriptor_generations(
        self,
        model_generation: int,
        reset_generation: int,
    ) -> None:
        if self._descriptors is None:
            return
        for name, descriptor in self._descriptors.items():
            for field, expected in (
                ("model_generation", model_generation),
                ("reset_generation", reset_generation),
            ):
                declared = descriptor.get(field)
                if declared is not None and declared != expected:
                    raise SimulationRecordingError(
                        f"descriptor {name} {field} does not match the recording"
                    )

    def _track_content(self, kind: str, value: Mapping[str, Any]) -> None:
        order_record = {
            "event_index": self._event_count,
            "frame_index": self._frame_count,
            "kind": kind,
        }
        content_record = {
            **order_record,
            "value": value,
        }
        self._event_order_hasher.update(_canonical_json(order_record) + b"\n")
        self._stream_hashers[kind].update(_canonical_json(content_record) + b"\n")
        self._stream_counts[kind] += 1
        self._event_count += 1

    def close(self) -> Path:
        """Commit a non-empty timeline and atomically publish its final manifest."""

        if self._closed:
            return self.manifest_path
        if self._stream is None or self._temporary_path is None or self._frame_count == 0:
            self.abort()
            raise SimulationRecordingError("recording must contain at least one frame")
        missing_content = sorted(
            name for name in self._required_content if self._stream_counts[name] == 0
        )
        if missing_content:
            self.abort()
            raise SimulationRecordingError(
                "recording required content is empty: " + ", ".join(missing_content)
            )
        if (
            "lifecycle_evidence" in self._required_content
            and self._terminal_state not in _TERMINAL_STATES
        ):
            self.abort()
            raise SimulationRecordingError(
                "recording lifecycle evidence has no terminal STOPPED or FAILED state"
            )
        temporary = self._temporary_path
        stream = self._stream
        self._stream = None
        stream.flush()
        os.fsync(stream.fileno())
        stream.close()
        committed = False
        try:
            # Deferred import avoids a recording/coordinator package import cycle.
            from sim.runtime.coordinator.atomic_file import replace_file_with_retry

            replace_file_with_retry(temporary, self.timeline_path)
            committed = True
            manifest = {
                "schema": RECORDING_SCHEMA,
                "run_id": self.run_id,
                "session_id": self.session_id,
                "model_generation": self._model_generation,
                "reset_generation": {
                    "start": self._start_reset_generation,
                    "end": self._end_reset_generation,
                },
                "clock": {
                    "authority": "mujoco",
                    "start_sim_time_ns": self._start_sim_time_ns,
                    "end_sim_time_ns": self._end_sim_time_ns,
                    "duration_ns": self._relative_time_ns,
                },
                "timeline": {
                    "path": TIMELINE_FILENAME,
                    "sha256": self._hasher.hexdigest(),
                    "bytes": self._bytes_written,
                    "frame_count": self._frame_count,
                },
                "content": {
                    "required_streams": sorted(self._required_content),
                    "event_order": {
                        "count": self._event_count,
                        "sha256": self._event_order_hasher.hexdigest(),
                    },
                    "streams": {
                        name: {
                            "count": self._stream_counts[name],
                            "sha256": self._stream_hashers[name].hexdigest(),
                        }
                        for name in _CONTENT_STREAMS
                    },
                },
                "continuous_tolerances": self._continuous_tolerances,
            }
            if self._run_allocation is not None:
                manifest["run_allocation"] = {
                    "sha256": _document_digest(self._run_allocation),
                    "document": self._run_allocation,
                }
            if self._descriptors is not None:
                manifest["descriptors"] = {
                    "sha256": _document_digest(self._descriptors),
                    "documents": self._descriptors,
                }
            if self._payload_reference_count:
                manifest["sensor_payloads"] = {
                    "schema": SENSOR_PAYLOAD_STORE_SCHEMA,
                    "root": SENSOR_PAYLOAD_ROOT,
                    "reference_count": self._payload_reference_count,
                    "unique_blob_count": len(self._payload_digests),
                    "referenced_bytes": self._payload_referenced_bytes,
                    "unique_bytes": self._payload_unique_bytes,
                }
            if self._typed_dds_reference_count:
                manifest["typed_dds_payloads"] = {
                    "schema": TYPED_DDS_PAYLOAD_STORE_SCHEMA,
                    "root": TYPED_DDS_PAYLOAD_ROOT,
                    "reference_count": self._typed_dds_reference_count,
                    "unique_blob_count": len(self._typed_dds_digests),
                    "referenced_bytes": self._typed_dds_referenced_bytes,
                    "unique_bytes": self._typed_dds_unique_bytes,
                }
            _atomic_write_json(self.manifest_path, manifest)
        except BaseException:
            if committed:
                self.timeline_path.unlink(missing_ok=True)
            shutil.rmtree(self.payload_root, ignore_errors=True)
            shutil.rmtree(self.typed_dds_payload_root, ignore_errors=True)
            raise
        finally:
            temporary.unlink(missing_ok=True)
        self._closed = True
        return self.manifest_path

    def abort(self) -> None:
        """Discard an uncommitted partial timeline."""

        if self._stream is not None:
            self._stream.close()
            self._stream = None
        if self._temporary_path is not None:
            self._temporary_path.unlink(missing_ok=True)
        shutil.rmtree(self.payload_root, ignore_errors=True)
        shutil.rmtree(self.typed_dds_payload_root, ignore_errors=True)
        self._closed = True


def _atomic_write_json(path: Path, document: Mapping[str, Any]) -> None:
    # Deferred import avoids a recording/coordinator package import cycle.
    from sim.runtime.coordinator.atomic_file import replace_file_with_retry

    payload = (
        json.dumps(
            document,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    descriptor, name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
    )
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        replace_file_with_retry(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


__all__ = [
    "MAX_SENSOR_PAYLOAD_BYTES",
    "RECORDING_FILENAME",
    "RECORDING_FRAME_SCHEMA",
    "RECORDING_SCHEMA",
    "SENSOR_PAYLOAD_REFERENCE_SCHEMA",
    "SENSOR_PAYLOAD_ROOT",
    "SENSOR_PAYLOAD_STORE_SCHEMA",
    "TIMELINE_FILENAME",
    "SensorPayloadSample",
    "SimulationRecordingError",
    "SimulationRecordingWriter",
]
