"""Durable, auditable storage contracts for inspection evidence captures.

This runtime-owned contract is shared by the perception producer and gateway
reader. The store deliberately separates persistence success from analysis
support and verdicts. Writing an image successfully never implies that an
inspection model ran or that it reached a conclusion.
"""

from __future__ import annotations

import errno
import json
import math
import os
import re
import shutil
import stat
import uuid
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from typing import Any

SCHEMA_VERSION = "lingtu.inspection.evidence.v1"
SUPPORTED_ACTIONS = frozenset(
    {
        "capture:overview",
        "capture:parking",
        "capture:bin_full",
        "capture:plate_ocr",
        "capture:indicator_on",
        "capture:indicator_off",
        "capture:indicator_green",
        "capture:indicator_red",
        "capture:indicator_yellow",
        "capture:door_open",
        "capture:door_closed",
        "capture:thermal",
        "capture:acoustic",
    }
)

_REQUEST_FIELDS = frozenset(
    {
        "task_id",
        "route_id",
        "route_revision",
        "map_id",
        "map_content_epoch",
        "point_index",
        "request_id",
        "action",
        "requested_at_s",
        "deadline_s",
    }
)
_SAFE_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
_UINT32_MAX = 2**32 - 1
_UINT64_MAX = 2**64 - 1
_MEDIA_EXTENSIONS = {
    "image/jpeg": ".jpg",
    "image/png": ".png",
    "image/webp": ".webp",
    "application/x-raw-rgb": ".rgb",
}
_PARKING_VERDICTS = frozenset({"violation", "clear", "inconclusive"})
_VISUAL_EXPECTATIONS = {
    "capture:indicator_on": "on", "capture:indicator_off": "off",
    "capture:indicator_green": "green", "capture:indicator_red": "red",
    "capture:indicator_yellow": "yellow", "capture:door_open": "open",
    "capture:door_closed": "closed",
}
_MEASUREMENT_ACTIONS = {"capture:thermal": "thermal", "capture:acoustic": "acoustic"}


class InspectionEvidenceError(RuntimeError):
    """Base error for inspection evidence storage."""


class EvidenceValidationError(InspectionEvidenceError, ValueError):
    """Raised when a request cannot safely enter the evidence store."""


class EvidenceConflictError(InspectionEvidenceError):
    """Raised when an idempotency key was already used for another request."""


class EvidenceIntegrityError(InspectionEvidenceError):
    """Raised when committed evidence fails its integrity checks."""


def _validate_identifier(field_name: str, value: Any) -> str:
    if not isinstance(value, str) or not _SAFE_IDENTIFIER.fullmatch(value):
        raise EvidenceValidationError(
            f"{field_name} must match {_SAFE_IDENTIFIER.pattern!r} and contain no path separators"
        )
    if value in {".", ".."}:
        raise EvidenceValidationError(f"{field_name} must not be a relative path token")
    return value


def _validate_integer(
    field_name: str,
    value: Any,
    *,
    minimum: int,
    maximum: int | None = None,
) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise EvidenceValidationError(f"{field_name} must be an integer")
    if value < minimum or (maximum is not None and value > maximum):
        if maximum is None:
            expected = f">= {minimum}"
        else:
            expected = f"in [{minimum}, {maximum}]"
        raise EvidenceValidationError(f"{field_name} must be {expected}")
    return value


def _validate_positive_finite_seconds(field_name: str, value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise EvidenceValidationError(f"{field_name} must be a finite number > 0")
    normalized = float(value)
    if not math.isfinite(normalized) or normalized <= 0.0:
        raise EvidenceValidationError(f"{field_name} must be a finite number > 0")
    return normalized


def _canonical_json_bytes(value: Any, *, field_name: str) -> bytes:
    try:
        text = json.dumps(
            value,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
    except (TypeError, ValueError) as exc:
        raise EvidenceValidationError(f"{field_name} must be finite JSON data: {exc}") from exc
    return (text + "\n").encode("utf-8")


def _validate_measurement_metadata(metadata: Any, sensor_id: str, modality: str) -> bytes:
    """Validate the declared envelope, never decode or interpret sensor arrays.

    The source stamp retains the complete generation/clock identity supplied
    by current measurement producers. Calibration contents and units are
    producer-owned; this boundary requires explicit declarations, not estimates.
    """
    if not isinstance(metadata, Mapping):
        raise EvidenceValidationError("measurement metadata must be a JSON object")
    encoded = _canonical_json_bytes(dict(metadata), field_name="measurement.metadata")
    units = metadata.get("units")
    if not isinstance(units, Mapping) or not units or any(
        not isinstance(key, str) or not key.strip()
        or not isinstance(value, str) or not value.strip()
        for key, value in units.items()
    ):
        raise EvidenceValidationError("measurement metadata requires explicit units")
    if "modality" in metadata and metadata["modality"] != modality:
        raise EvidenceValidationError("measurement modality does not match metadata")
    stamp = metadata.get("stamp", metadata.get("source_stamp"))
    if not isinstance(stamp, Mapping):
        raise EvidenceValidationError("measurement metadata requires a full source_stamp")
    for key in ("session_id", "instance_id", "sensor_id", "frame_id"):
        if not isinstance(stamp.get(key), str) or not stamp[key].strip():
            raise EvidenceValidationError(f"measurement source_stamp requires {key}")
    for key in ("model_generation", "reset_generation", "sequence", "sim_time_ns"):
        _validate_integer(f"measurement.source_stamp.{key}", stamp.get(key), minimum=0)
    if stamp["sensor_id"] != sensor_id:
        raise EvidenceValidationError("measurement sensor_id does not match source_stamp")
    # Producers retain their own metadata layout, including the raw NPZ codec's
    # per-modality calibration fields. An explicit null records uncalibrated
    # acoustic data; it is not silently promoted to calibrated measurement.
    calibration_container = metadata
    calibration_key = "calibration"
    if calibration_key not in calibration_container:
        if modality == "semantic":
            calibration_container = metadata.get("configuration", {})
        else:
            calibration_container = metadata.get("frame_metadata", {})
            calibration_key = "camera_calibration" if modality == "thermal" else "calibration"
    if (not isinstance(calibration_container, Mapping)
            or calibration_key not in calibration_container
            or (calibration_container[calibration_key] is not None
                and (not isinstance(calibration_container[calibration_key], Mapping)
                     or not calibration_container[calibration_key]))):
        raise EvidenceValidationError("measurement metadata requires explicit calibration")
    return encoded


def _write_bytes_fsync(path: Path, payload: bytes) -> None:
    """Create one file, flush Python buffers, then flush the file descriptor."""

    with path.open("xb") as handle:
        handle.write(payload)
        handle.flush()
        os.fsync(handle.fileno())


def _fsync_directory(path: Path) -> None:
    """Flush directory metadata where the host filesystem exposes that API.

    Linux, including the S100P deployment target, supports directory fsync.
    Windows does not expose a portable directory descriptor through ``os.open``;
    the unsupported Windows error is ignored while file fsync and atomic rename
    still apply.
    """

    flags = os.O_RDONLY
    if hasattr(os, "O_DIRECTORY"):
        flags |= os.O_DIRECTORY
    try:
        descriptor = os.open(str(path), flags)
    except OSError as exc:
        if os.name == "nt" and exc.errno in {errno.EACCES, errno.EINVAL, errno.EPERM}:
            return
        raise
    try:
        try:
            os.fsync(descriptor)
        except OSError as exc:
            if not (os.name == "nt" and exc.errno in {errno.EACCES, errno.EINVAL, errno.EPERM}):
                raise
    finally:
        os.close(descriptor)


@dataclass(frozen=True, init=False)
class MeasurementArtifact:
    """Immutable raw sensor bytes and an isolated, finite JSON metadata snapshot.

    Payload encoding (including NPZ arrays) belongs to the sensor producer.
    Reading ``metadata`` returns a copy so nested caller mutations cannot alter
    the artifact that will be committed.
    """

    modality: str
    sensor_id: str
    payload: bytes
    media_type: str
    _metadata_json: bytes = field(repr=False)

    def __init__(
        self, *, modality: str, sensor_id: str, payload: bytes,
        media_type: str, metadata: Mapping[str, Any],
    ) -> None:
        if modality not in {"thermal", "acoustic", "semantic"}:
            raise EvidenceValidationError("measurement modality must be thermal, acoustic or semantic")
        _validate_identifier("measurement.sensor_id", sensor_id)
        if not isinstance(payload, bytes) or not payload:
            raise EvidenceValidationError("measurement payload must be non-empty bytes")
        if not isinstance(media_type, str) or not media_type.strip():
            raise EvidenceValidationError("measurement media_type is required")
        object.__setattr__(self, "modality", modality)
        object.__setattr__(self, "sensor_id", sensor_id)
        object.__setattr__(self, "payload", payload)
        object.__setattr__(self, "media_type", media_type)
        object.__setattr__(self, "_metadata_json", _validate_measurement_metadata(metadata, sensor_id, modality))

    @property
    def metadata(self) -> dict[str, Any]:
        return dict(json.loads(self._metadata_json))


@dataclass(frozen=True)
class InspectionEvidenceRequest:
    """Complete audit identity and time window for one inspection capture."""

    task_id: str
    route_id: str
    route_revision: int
    map_id: str
    map_content_epoch: int
    point_index: int
    request_id: str
    action: str
    requested_at_s: float
    deadline_s: float

    def __post_init__(self) -> None:
        _validate_identifier("task_id", self.task_id)
        _validate_identifier("route_id", self.route_id)
        _validate_integer(
            "route_revision",
            self.route_revision,
            minimum=1,
            maximum=_UINT64_MAX,
        )
        _validate_identifier("map_id", self.map_id)
        _validate_integer("map_content_epoch", self.map_content_epoch, minimum=0)
        _validate_integer("point_index", self.point_index, minimum=0, maximum=_UINT32_MAX)
        _validate_identifier("request_id", self.request_id)
        if self.action not in SUPPORTED_ACTIONS:
            raise EvidenceValidationError(
                f"action must be one of {sorted(SUPPORTED_ACTIONS)}, got {self.action!r}"
            )
        requested_at_s = _validate_positive_finite_seconds(
            "requested_at_s", self.requested_at_s
        )
        deadline_s = _validate_positive_finite_seconds("deadline_s", self.deadline_s)
        if deadline_s <= requested_at_s:
            raise EvidenceValidationError("deadline_s must be greater than requested_at_s")
        object.__setattr__(self, "requested_at_s", requested_at_s)
        object.__setattr__(self, "deadline_s", deadline_s)

    @classmethod
    def from_value(
        cls,
        value: InspectionEvidenceRequest | Mapping[str, Any],
    ) -> InspectionEvidenceRequest:
        """Normalize a typed request or a strict complete-audit mapping."""

        if isinstance(value, cls):
            return value
        if not isinstance(value, Mapping):
            raise EvidenceValidationError("request must be InspectionEvidenceRequest or a mapping")
        # Read existing manifests using the current task identity. Older
        # point_id is redundant with the pinned route and point_index.
        normalized = dict(value)
        normalized.pop("point_id", None)
        if "run_id" in normalized:
            normalized.setdefault("task_id", normalized.pop("run_id"))
        unknown = set(normalized) - _REQUEST_FIELDS
        missing = _REQUEST_FIELDS - set(normalized)
        if unknown:
            raise EvidenceValidationError(f"unknown request fields: {sorted(unknown)}")
        if missing:
            raise EvidenceValidationError(f"missing request fields: {sorted(missing)}")
        return cls(**{name: normalized[name] for name in _REQUEST_FIELDS})

    def to_dict(self) -> dict[str, Any]:
        """Return the stable request representation written to the manifest."""

        return {
            "task_id": self.task_id,
            "route_id": self.route_id,
            "route_revision": self.route_revision,
            "map_id": self.map_id,
            "map_content_epoch": self.map_content_epoch,
            "point_index": self.point_index,
            "request_id": self.request_id,
            "action": self.action,
            "requested_at_s": self.requested_at_s,
            "deadline_s": self.deadline_s,
        }


@dataclass(frozen=True)
class TrustedParkingObservation:
    """An observation explicitly admitted through the parking trust boundary.

    Raw dictionaries are intentionally not accepted as trusted observations.
    The caller must construct this type after validating the upstream detector.
    """

    source: str
    verdict: str
    confidence: float | None = None
    details: Mapping[str, Any] = field(default_factory=dict)
    observed_at: str | None = None

    def __post_init__(self) -> None:
        _validate_identifier("trusted_observation.source", self.source)
        if self.verdict not in _PARKING_VERDICTS:
            raise EvidenceValidationError(
                f"parking verdict must be one of {sorted(_PARKING_VERDICTS)}"
            )
        if self.confidence is not None:
            if isinstance(self.confidence, bool) or not isinstance(self.confidence, (int, float)):
                raise EvidenceValidationError("parking confidence must be a finite number in [0, 1]")
            confidence = float(self.confidence)
            if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
                raise EvidenceValidationError("parking confidence must be a finite number in [0, 1]")
        if not isinstance(self.details, Mapping):
            raise EvidenceValidationError("parking observation details must be a JSON object")
        _canonical_json_bytes(dict(self.details), field_name="trusted_observation.details")
        if self.observed_at is not None:
            if not isinstance(self.observed_at, str) or not 1 <= len(self.observed_at) <= 128:
                raise EvidenceValidationError("observed_at must be a non-empty string up to 128 chars")

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-safe snapshot for the immutable manifest."""

        result: dict[str, Any] = {
            "source": self.source,
            "verdict": self.verdict,
            "details": dict(self.details),
        }
        if self.confidence is not None:
            result["confidence"] = float(self.confidence)
        if self.observed_at is not None:
            result["observed_at"] = self.observed_at
        return result


@dataclass(frozen=True)
class InspectionFinding:
    """Caller observation, not an inference performed by the evidence store.

    Visual actions compare states. Thermal/acoustic actions accept an explicit
    verdict only with the matching modality and identified model provenance.
    """

    observed_state: str
    source: str
    reason: str = ""
    details: Mapping[str, Any] = field(default_factory=dict)
    modality: str | None = None
    model_provenance: Mapping[str, Any] | None = None

    def __post_init__(self) -> None:
        if self.modality is not None:
            if self.modality not in {"thermal", "acoustic"} or self.observed_state not in {
                "normal", "anomaly", "inconclusive",
            }:
                raise EvidenceValidationError("measurement findings require thermal/acoustic and an explicit verdict")
            if not isinstance(self.model_provenance, Mapping) or any(
                not isinstance(self.model_provenance.get(key), str) or not self.model_provenance[key].strip()
                for key in ("model_id", "model_version")
            ):
                raise EvidenceValidationError("measurement findings require model_id and model_version provenance")
            _canonical_json_bytes(dict(self.model_provenance), field_name="finding.model_provenance")
        elif self.observed_state not in {"on", "off", "red", "green", "yellow", "open", "closed", "unknown"}:
            raise EvidenceValidationError("unknown visual inspection state")
        if not isinstance(self.source, str) or not self.source.strip():
            raise EvidenceValidationError("visual observation source is required")
        if not isinstance(self.reason, str) or not isinstance(self.details, Mapping):
            raise EvidenceValidationError("invalid visual observation details")
        _canonical_json_bytes(dict(self.details), field_name="finding.details")

    def analysis(self, action: str) -> dict[str, Any]:
        if action in _MEASUREMENT_ACTIONS:
            if self.modality != _MEASUREMENT_ACTIONS[action]:
                raise EvidenceValidationError("finding modality does not match inspection action")
            self.__post_init__()
            return {
                "support": "measurement_observation", "verdict": self.observed_state,
                "reason": self.reason or "caller_provided_finding",
                "observation": {
                    "modality": self.modality, "observed_state": self.observed_state,
                    "source": self.source, "details": dict(self.details),
                    "model_provenance": dict(self.model_provenance or {}),
                },
            }
        if action not in _VISUAL_EXPECTATIONS or self.modality is not None:
            raise EvidenceValidationError("visual findings require a visual inspection action")
        expected = _VISUAL_EXPECTATIONS[action]
        observed = self.observed_state
        if expected in {"open", "closed"}:
            comparable = observed in {"open", "closed"}
            matched = observed == expected
        else:
            comparable = observed in {"on", "off", "red", "green", "yellow"}
            matched = observed == expected or (expected == "on" and observed in {"red", "green", "yellow"})
            if expected in {"red", "green", "yellow"} and observed == "on":
                comparable = False
        verdict = ("normal" if matched else "anomaly") if comparable else "inconclusive"
        return {
            "support": "visual_observation", "verdict": verdict,
            "reason": self.reason or ("matched" if verdict == "normal" else "state_mismatch" if comparable else "not_observed"),
            "observation": {"expected_state": expected, "observed_state": observed,
                            "source": self.source, "details": dict(self.details)},
        }


@dataclass(frozen=True)
class InspectionEvidenceResult:
    """Verified view of one committed evidence directory."""

    request: InspectionEvidenceRequest
    evidence_dir: Path
    manifest_path: Path
    manifest: dict[str, Any]
    persistence_status: str
    analysis_support: str
    analysis_verdict: str


class InspectionEvidenceStore:
    """Commit inspection evidence with atomic directory publication."""

    def __init__(self, root: str | os.PathLike[str], *, staging_ttl_seconds: float = 3600.0):
        if isinstance(staging_ttl_seconds, bool) or not isinstance(
            staging_ttl_seconds, (int, float)
        ):
            raise EvidenceValidationError("staging_ttl_seconds must be a non-negative number")
        if not math.isfinite(float(staging_ttl_seconds)) or staging_ttl_seconds < 0:
            raise EvidenceValidationError("staging_ttl_seconds must be a non-negative number")
        self.root = Path(root).expanduser().resolve()
        self.requests_dir = self.root / "requests"
        self.staging_ttl_seconds = float(staging_ttl_seconds)
        self.requests_dir.mkdir(parents=True, exist_ok=True)
        _fsync_directory(self.requests_dir)
        self.cleanup_staging()

    def persist(
        self,
        request: InspectionEvidenceRequest | Mapping[str, Any],
        *,
        rgb_bytes: bytes | bytearray | memoryview | None = None,
        media_type: str | None = None,
        pose: Mapping[str, Any] | None = None,
        detections: Mapping[str, Any] | Sequence[Any] | None = None,
        trusted_observation: TrustedParkingObservation | None = None,
        finding: InspectionFinding | None = None,
        measurements: tuple[MeasurementArtifact, ...] = (),
    ) -> InspectionEvidenceResult:
        """Persist one request, or return the previously committed result.

        The idempotency identity is the complete audit request contract.
        Payloads supplied by a retry never replace already committed evidence.
        """

        normalized_request = InspectionEvidenceRequest.from_value(request)
        final_dir = self._final_directory(normalized_request.request_id)
        if final_dir.exists():
            return self._load_matching(final_dir, normalized_request)
        self._validate_payload(
            normalized_request,
            rgb_bytes=rgb_bytes,
            media_type=media_type,
            pose=pose,
            detections=detections,
            trusted_observation=trusted_observation,
            finding=finding,
            measurements=measurements,
        )
        self.cleanup_staging()
        staging_dir = self.requests_dir / (
            f".staging-{normalized_request.request_id}-{uuid.uuid4().hex}"
        )
        staging_dir.mkdir(mode=0o750)
        try:
            artifacts: list[dict[str, Any]] = []
            if rgb_bytes is not None:
                payload = bytes(rgb_bytes)
                extension = _MEDIA_EXTENSIONS[media_type or ""]
                artifacts.append(
                    self._write_artifact(
                        staging_dir,
                        name=f"rgb{extension}",
                        kind="rgb",
                        media_type=media_type or "",
                        payload=payload,
                    )
                )
            if pose is not None:
                artifacts.append(
                    self._write_artifact(
                        staging_dir,
                        name="pose.json",
                        kind="pose",
                        media_type="application/json",
                        payload=_canonical_json_bytes(dict(pose), field_name="pose"),
                    )
                )
            if detections is not None:
                detections_value: Any
                if isinstance(detections, Mapping):
                    detections_value = dict(detections)
                else:
                    detections_value = list(detections)
                artifacts.append(
                    self._write_artifact(
                        staging_dir,
                        name="detections.json",
                        kind="detections",
                        media_type="application/json",
                        payload=_canonical_json_bytes(detections_value, field_name="detections"),
                    )
                )

            for measurement in measurements:
                extension = ".npz" if measurement.media_type in {
                    "application/x-npz", "application/vnd.lingtu.measurement+npz",
                } else ".bin"
                record = self._write_artifact(
                    staging_dir,
                    name=f"measurement-{measurement.sensor_id}{extension}",
                    kind="measurement",
                    media_type=measurement.media_type,
                    payload=measurement.payload,
                )
                record.update(modality=measurement.modality, sensor_id=measurement.sensor_id,
                              metadata=measurement.metadata)
                artifacts.append(record)

            manifest = {
                "schema_version": SCHEMA_VERSION,
                "evidence_id": normalized_request.request_id,
                "request": normalized_request.to_dict(),
                "persistence": {
                    "persisted": True,
                    "committed_at": _utc_now(),
                    "artifacts": artifacts,
                },
                "analysis": self._analysis_for(normalized_request.action, trusted_observation, finding),
            }
            manifest_bytes = _canonical_json_bytes(manifest, field_name="manifest")
            _write_bytes_fsync(staging_dir / "manifest.json", manifest_bytes)
            _fsync_directory(staging_dir)

            try:
                self._atomic_rename(staging_dir, final_dir)
            except OSError:
                if final_dir.exists():
                    result = self._load_matching(final_dir, normalized_request)
                    self._remove_staging(staging_dir)
                    return result
                raise
            _fsync_directory(final_dir)
            _fsync_directory(self.requests_dir)
            return self._load_matching(final_dir, normalized_request)
        except BaseException:
            self._remove_staging(staging_dir)
            raise

    def get(self, request_id: str) -> InspectionEvidenceResult:
        """Load and verify committed evidence by request id."""

        safe_request_id = _validate_identifier("request_id", request_id)
        return self._load_committed(self._final_directory(safe_request_id))

    def review(self, request_id: str, *, verdict: str, note: str = "") -> InspectionEvidenceResult:
        """Record a human finding without rewriting the captured observation."""

        if verdict not in {"normal", "anomaly", "inconclusive"}:
            raise EvidenceValidationError("review verdict must be normal, anomaly or inconclusive")
        if not isinstance(note, str) or len(note) > 2000:
            raise EvidenceValidationError("review note must be a string of at most 2000 characters")
        result = self.get(request_id)
        history = self._read_reviews(result.evidence_dir)
        if history and history[-1]["verdict"] == verdict and history[-1]["note"] == note:
            return result
        history.append({"verdict": verdict, "note": note, "reviewed_at": _utc_now()})
        target = result.evidence_dir / "reviews.json"
        temporary = result.evidence_dir / f".reviews-{uuid.uuid4().hex}"
        try:
            _write_bytes_fsync(temporary, _canonical_json_bytes(history, field_name="reviews"))
            os.replace(temporary, target)
            _fsync_directory(result.evidence_dir)
        finally:
            temporary.unlink(missing_ok=True)
        return self.get(request_id)

    @staticmethod
    def _read_reviews(directory: Path) -> list[dict[str, str]]:
        path = directory / "reviews.json"
        if not path.exists():
            return []
        try:
            history = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(history, list):
                raise ValueError("review history must be an array")
            for item in history:
                if (not isinstance(item, dict)
                        or item.get("verdict") not in {"normal", "anomaly", "inconclusive"}
                        or not isinstance(item.get("note"), str)
                        or not isinstance(item.get("reviewed_at"), str)):
                    raise ValueError("invalid review record")
        except (ValueError, UnicodeDecodeError) as exc:
            raise EvidenceIntegrityError(f"invalid review history: {exc}") from exc
        return history

    def cleanup_staging(self, *, now: float | None = None) -> list[Path]:
        """Remove staging directories older than the configured TTL."""

        current_time = time_now() if now is None else float(now)
        removed: list[Path] = []
        for candidate in self.requests_dir.glob(".staging-*"):
            try:
                modified_at = os.lstat(candidate).st_mtime
            except FileNotFoundError:
                continue
            if current_time - modified_at < self.staging_ttl_seconds:
                continue
            self._remove_staging(candidate)
            if not candidate.exists():
                removed.append(candidate)
        if removed:
            _fsync_directory(self.requests_dir)
        return removed

    def _final_directory(self, request_id: str) -> Path:
        return self.requests_dir / request_id

    def _atomic_rename(self, source: Path, destination: Path) -> None:
        os.rename(source, destination)

    def _remove_staging(self, path: Path) -> None:
        try:
            if path.is_symlink():
                path.unlink()
            elif path.exists():
                shutil.rmtree(path)
        except FileNotFoundError:
            pass

    def _write_artifact(
        self,
        directory: Path,
        *,
        name: str,
        kind: str,
        media_type: str,
        payload: bytes,
    ) -> dict[str, Any]:
        _write_bytes_fsync(directory / name, payload)
        return {
            "kind": kind,
            "path": name,
            "media_type": media_type,
            "bytes": len(payload),
        }

    def _validate_payload(
        self,
        request: InspectionEvidenceRequest,
        *,
        rgb_bytes: bytes | bytearray | memoryview | None,
        media_type: str | None,
        pose: Mapping[str, Any] | None,
        detections: Mapping[str, Any] | Sequence[Any] | None,
        trusted_observation: TrustedParkingObservation | None,
        finding: InspectionFinding | None,
        measurements: tuple[MeasurementArtifact, ...],
    ) -> None:
        if not isinstance(measurements, tuple) or any(
            not isinstance(item, MeasurementArtifact) for item in measurements
        ):
            raise EvidenceValidationError("measurements must be a tuple of MeasurementArtifact")
        sensor_ids = [item.sensor_id for item in measurements]
        if len(set(sensor_ids)) != len(sensor_ids):
            raise EvidenceValidationError("duplicate measurement sensor_id")
        required_modality = _MEASUREMENT_ACTIONS.get(request.action)
        if required_modality is not None:
            if not any(item.modality == required_modality for item in measurements):
                raise EvidenceValidationError(f"{request.action} requires a matching measurement")
            if not rgb_bytes or not pose:
                raise EvidenceValidationError(f"{request.action} requires non-empty RGB and pose evidence")
        if rgb_bytes is None and media_type is not None:
            raise EvidenceValidationError("media_type requires rgb_bytes")
        if rgb_bytes is not None:
            if not isinstance(rgb_bytes, (bytes, bytearray, memoryview)):
                raise EvidenceValidationError("rgb_bytes must be bytes-like")
            if media_type is None:
                raise EvidenceValidationError("media_type is required when rgb_bytes are present")
            if media_type not in _MEDIA_EXTENSIONS:
                raise EvidenceValidationError(f"unsupported media_type: {media_type!r}")
        if pose is not None:
            if not isinstance(pose, Mapping):
                raise EvidenceValidationError("pose must be a JSON object")
            _canonical_json_bytes(dict(pose), field_name="pose")
        if detections is not None:
            if isinstance(detections, Mapping):
                detections_value: Any = dict(detections)
            elif isinstance(detections, Sequence) and not isinstance(
                detections, (str, bytes, bytearray, memoryview)
            ):
                detections_value = list(detections)
            else:
                raise EvidenceValidationError("detections must be a JSON object or array")
            _canonical_json_bytes(detections_value, field_name="detections")
        if trusted_observation is not None:
            if not isinstance(trusted_observation, TrustedParkingObservation):
                raise EvidenceValidationError(
                    "trusted_observation must be TrustedParkingObservation"
                )
            if request.action != "capture:parking":
                raise EvidenceValidationError(
                    "TrustedParkingObservation is valid only for capture:parking"
                )
        if finding is not None:
            if not isinstance(finding, InspectionFinding):
                raise EvidenceValidationError("finding must be InspectionFinding")
            finding.analysis(request.action)

    def _analysis_for(
        self,
        action: str,
        trusted_observation: TrustedParkingObservation | None,
        finding: InspectionFinding | None = None,
    ) -> dict[str, Any]:
        if action in _MEASUREMENT_ACTIONS:
            if finding is not None:
                return finding.analysis(action)
            return {"support": "unavailable", "verdict": "inconclusive", "reason": "analyzer_not_integrated"}
        if action in _VISUAL_EXPECTATIONS:
            if finding is not None:
                return finding.analysis(action)
            return {"support": "unavailable", "verdict": "inconclusive", "reason": "visual_reader_unavailable"}
        if action == "capture:overview":
            return {
                "support": "not_applicable",
                "verdict": "not_evaluated",
                "reason": "capture_only",
            }
        if action == "capture:parking":
            if trusted_observation is None:
                return {
                    "support": "unavailable",
                    "verdict": "inconclusive",
                    "reason": "trusted_observation_missing",
                }
            return {
                "support": "trusted_observation",
                "verdict": trusted_observation.verdict,
                "reason": "trusted_observation",
                "observation": trusted_observation.to_dict(),
            }
        return {
            "support": "unavailable",
            "verdict": "inconclusive",
            "reason": "analyzer_not_integrated",
        }

    def _load_matching(
        self,
        directory: Path,
        expected_request: InspectionEvidenceRequest,
    ) -> InspectionEvidenceResult:
        result = self._load_committed(directory)
        if result.request != expected_request:
            raise EvidenceConflictError(
                "request_id already committed for a different audit identity"
            )
        return result

    def _load_committed(self, directory: Path) -> InspectionEvidenceResult:
        if not directory.exists():
            raise FileNotFoundError(directory)
        if directory.is_symlink() or not directory.is_dir():
            raise EvidenceIntegrityError(f"evidence path is not a committed directory: {directory}")
        if directory.resolve().parent != self.requests_dir.resolve():
            raise EvidenceIntegrityError("evidence directory escaped the configured root")

        manifest_path = directory / "manifest.json"
        try:
            file_stat = manifest_path.lstat()
        except OSError as exc:
            raise EvidenceIntegrityError(f"cannot inspect committed manifest: {exc}") from exc
        if manifest_path.is_symlink() or not stat.S_ISREG(file_stat.st_mode):
            raise EvidenceIntegrityError(f"manifest is not a regular file: {manifest_path.name}")
        if manifest_path.resolve().parent != directory.resolve():
            raise EvidenceIntegrityError("manifest escaped the evidence directory")
        try:
            manifest_bytes = manifest_path.read_bytes()
        except OSError as exc:
            raise EvidenceIntegrityError(f"cannot read committed manifest: {exc}") from exc
        try:
            manifest = json.loads(manifest_bytes)
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            raise EvidenceIntegrityError(f"manifest.json is invalid JSON: {exc}") from exc
        if not isinstance(manifest, dict) or manifest.get("schema_version") != SCHEMA_VERSION:
            raise EvidenceIntegrityError("manifest schema_version is missing or unsupported")

        try:
            request = InspectionEvidenceRequest.from_value(manifest["request"])
            persistence = manifest["persistence"]
            analysis = manifest["analysis"]
            if persistence.get("persisted") is not True:
                raise EvidenceIntegrityError("manifest does not describe persisted evidence")
            if manifest.get("evidence_id") != request.request_id:
                raise EvidenceIntegrityError("evidence_id does not match request_id")
            if directory.name != request.request_id:
                raise EvidenceIntegrityError("evidence directory does not match request_id")
            artifacts = persistence["artifacts"]
            if not isinstance(artifacts, list):
                raise EvidenceIntegrityError("manifest artifacts must be an array")
            self._verify_artifacts(directory, artifacts, action=request.action)
            support = analysis["support"]
            verdict = analysis["verdict"]
            if not isinstance(support, str) or not isinstance(verdict, str):
                raise EvidenceIntegrityError("analysis support and verdict must be strings")
        except EvidenceIntegrityError:
            raise
        except (EvidenceValidationError, KeyError, TypeError, AttributeError) as exc:
            raise EvidenceIntegrityError(f"manifest contract is invalid: {exc}") from exc

        reviews = self._read_reviews(directory)
        if reviews:
            manifest["review"] = reviews[-1]
        return InspectionEvidenceResult(
            request=request,
            evidence_dir=directory,
            manifest_path=manifest_path,
            manifest=manifest,
            persistence_status="persisted",
            analysis_support=support,
            analysis_verdict=verdict,
        )

    def _verify_artifacts(self, directory: Path, artifacts: list[Any], *, action: str) -> None:
        seen: set[str] = set()
        measurement_sensors: set[str] = set()
        modalities: set[str] = set()
        nonempty_kinds: set[str] = set()
        for record in artifacts:
            if not isinstance(record, dict):
                raise EvidenceIntegrityError("artifact record must be an object")
            name = record.get("path")
            if not isinstance(name, str) or not _is_safe_artifact_name(name):
                raise EvidenceIntegrityError(f"unsafe artifact path: {name!r}")
            if name in seen:
                raise EvidenceIntegrityError(f"duplicate artifact path: {name}")
            seen.add(name)
            artifact_path = directory / name
            try:
                file_stat = artifact_path.lstat()
            except OSError as exc:
                raise EvidenceIntegrityError(f"cannot inspect artifact {name}: {exc}") from exc
            if artifact_path.is_symlink() or not stat.S_ISREG(file_stat.st_mode):
                raise EvidenceIntegrityError(f"artifact is not a regular file: {name}")
            if artifact_path.resolve().parent != directory.resolve():
                raise EvidenceIntegrityError(f"artifact escaped the evidence directory: {name}")
            expected_bytes = record.get("bytes")
            if (
                isinstance(expected_bytes, bool)
                or not isinstance(expected_bytes, int)
                or expected_bytes < 0
            ):
                raise EvidenceIntegrityError(f"artifact byte count is invalid: {name}")
            if expected_bytes != file_stat.st_size:
                raise EvidenceIntegrityError(f"artifact byte count mismatch: {name}")
            media_type = record.get("media_type")
            if not isinstance(media_type, str) or not media_type:
                raise EvidenceIntegrityError(f"artifact media type is invalid: {name}")
            if expected_bytes > 0 and isinstance(record.get("kind"), str):
                nonempty_kinds.add(record["kind"])
            if record.get("kind") == "measurement":
                sensor_id = _validate_identifier("measurement.sensor_id", record.get("sensor_id"))
                if sensor_id in measurement_sensors:
                    raise EvidenceIntegrityError("duplicate measurement sensor_id")
                measurement_sensors.add(sensor_id)
                modality = record.get("modality")
                if modality not in {"thermal", "acoustic", "semantic"}:
                    raise EvidenceIntegrityError("invalid measurement modality")
                if expected_bytes == 0:
                    raise EvidenceIntegrityError("measurement payload must be non-empty")
                _validate_measurement_metadata(record.get("metadata"), sensor_id, modality)
                modalities.add(modality)
        required_modality = _MEASUREMENT_ACTIONS.get(action)
        if required_modality is not None:
            if required_modality not in modalities:
                raise EvidenceIntegrityError(f"{action} requires a matching measurement")
            if not {"rgb", "pose"} <= nonempty_kinds:
                raise EvidenceIntegrityError(f"{action} requires non-empty RGB and pose evidence")


def _is_safe_artifact_name(value: str) -> bool:
    path = PurePosixPath(value)
    return (
        len(path.parts) == 1
        and value not in {"", ".", ".."}
        and "/" not in value
        and "\\" not in value
    )


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def time_now() -> float:
    """Small seam for deterministic staging cleanup tests."""

    return datetime.now(timezone.utc).timestamp()
