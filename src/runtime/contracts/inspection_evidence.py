"""Durable, auditable storage contracts for inspection evidence captures.

This runtime-owned contract is shared by the perception producer and gateway
reader. The store deliberately separates persistence success from analysis
support and verdicts. Writing an image successfully never implies that an
inspection model ran or that it reached a conclusion.
"""

from __future__ import annotations

import errno
import hashlib
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
    }
)

_REQUEST_FIELDS = frozenset(
    {
        "run_id",
        "route_id",
        "route_revision",
        "map_id",
        "map_version",
        "point_id",
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


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


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


@dataclass(frozen=True)
class InspectionEvidenceRequest:
    """Complete audit identity and time window for one inspection capture."""

    run_id: str
    route_id: str
    route_revision: int
    map_id: str
    map_version: int
    point_id: str
    point_index: int
    request_id: str
    action: str
    requested_at_s: float
    deadline_s: float

    def __post_init__(self) -> None:
        _validate_identifier("run_id", self.run_id)
        _validate_identifier("route_id", self.route_id)
        _validate_integer(
            "route_revision",
            self.route_revision,
            minimum=1,
            maximum=_UINT64_MAX,
        )
        _validate_identifier("map_id", self.map_id)
        _validate_integer("map_version", self.map_version, minimum=0)
        _validate_identifier("point_id", self.point_id)
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
        unknown = set(value) - _REQUEST_FIELDS
        missing = _REQUEST_FIELDS - set(value)
        if unknown:
            raise EvidenceValidationError(f"unknown request fields: {sorted(unknown)}")
        if missing:
            raise EvidenceValidationError(f"missing request fields: {sorted(missing)}")
        return cls(**{name: value[name] for name in _REQUEST_FIELDS})

    def to_dict(self) -> dict[str, Any]:
        """Return the stable request representation written to the manifest."""

        return {
            "run_id": self.run_id,
            "route_id": self.route_id,
            "route_revision": self.route_revision,
            "map_id": self.map_id,
            "map_version": self.map_version,
            "point_id": self.point_id,
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
class InspectionEvidenceResult:
    """Verified view of one committed evidence directory."""

    request: InspectionEvidenceRequest
    evidence_dir: Path
    manifest_path: Path
    manifest_sha256: str
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
    ) -> InspectionEvidenceResult:
        """Persist one request, or return the previously committed result.

        The idempotency identity is the complete audit request contract.
        Payloads supplied by a retry never replace already committed evidence.
        """

        normalized_request = InspectionEvidenceRequest.from_value(request)
        self._validate_payload(
            normalized_request,
            rgb_bytes=rgb_bytes,
            media_type=media_type,
            pose=pose,
            detections=detections,
            trusted_observation=trusted_observation,
        )
        final_dir = self._final_directory(normalized_request.request_id)
        if final_dir.exists():
            return self._load_matching(final_dir, normalized_request)

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

            manifest = {
                "schema_version": SCHEMA_VERSION,
                "evidence_id": normalized_request.request_id,
                "request": normalized_request.to_dict(),
                "persistence": {
                    "persisted": True,
                    "committed_at": _utc_now(),
                    "artifacts": artifacts,
                },
                "analysis": self._analysis_for(normalized_request.action, trusted_observation),
            }
            manifest_bytes = _canonical_json_bytes(manifest, field_name="manifest")
            manifest_hash = _sha256(manifest_bytes)
            _write_bytes_fsync(staging_dir / "manifest.json", manifest_bytes)
            _write_bytes_fsync(
                staging_dir / "manifest.sha256",
                (manifest_hash + "\n").encode("ascii"),
            )
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
            "sha256": _sha256(payload),
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
    ) -> None:
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

    def _analysis_for(
        self,
        action: str,
        trusted_observation: TrustedParkingObservation | None,
    ) -> dict[str, Any]:
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
        checksum_path = directory / "manifest.sha256"
        for label, path in (("manifest", manifest_path), ("manifest checksum", checksum_path)):
            try:
                file_stat = path.lstat()
            except OSError as exc:
                raise EvidenceIntegrityError(f"cannot inspect committed {label}: {exc}") from exc
            if path.is_symlink() or not stat.S_ISREG(file_stat.st_mode):
                raise EvidenceIntegrityError(f"{label} is not a regular file: {path.name}")
            if path.resolve().parent != directory.resolve():
                raise EvidenceIntegrityError(f"{label} escaped the evidence directory")
        try:
            manifest_bytes = manifest_path.read_bytes()
            checksum = checksum_path.read_text(encoding="ascii").strip()
        except (OSError, UnicodeError) as exc:
            raise EvidenceIntegrityError(f"cannot read committed manifest: {exc}") from exc
        actual_checksum = _sha256(manifest_bytes)
        if checksum != actual_checksum:
            raise EvidenceIntegrityError("manifest.sha256 does not match manifest.json")
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
            self._verify_artifacts(directory, artifacts)
            support = analysis["support"]
            verdict = analysis["verdict"]
            if not isinstance(support, str) or not isinstance(verdict, str):
                raise EvidenceIntegrityError("analysis support and verdict must be strings")
        except EvidenceIntegrityError:
            raise
        except (EvidenceValidationError, KeyError, TypeError, AttributeError) as exc:
            raise EvidenceIntegrityError(f"manifest contract is invalid: {exc}") from exc

        return InspectionEvidenceResult(
            request=request,
            evidence_dir=directory,
            manifest_path=manifest_path,
            manifest_sha256=actual_checksum,
            manifest=manifest,
            persistence_status="persisted",
            analysis_support=support,
            analysis_verdict=verdict,
        )

    def _verify_artifacts(self, directory: Path, artifacts: list[Any]) -> None:
        seen: set[str] = set()
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
            try:
                payload = artifact_path.read_bytes()
            except OSError as exc:
                raise EvidenceIntegrityError(f"cannot read artifact {name}: {exc}") from exc
            if record.get("bytes") != len(payload):
                raise EvidenceIntegrityError(f"artifact byte count mismatch: {name}")
            if record.get("sha256") != _sha256(payload):
                raise EvidenceIntegrityError(f"artifact sha256 mismatch: {name}")


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
