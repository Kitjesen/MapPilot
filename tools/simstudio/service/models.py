"""Framework-neutral durable contracts for the local SimStudio service.

The models in this module deliberately contain no FastAPI, process, or field
runtime concerns.  They are small JSON records which can be passed between
the Studio application services and adapters without leaking filesystem
handles or transport objects into the domain layer.
"""

from __future__ import annotations

import copy
import hashlib
import json
from dataclasses import dataclass
from typing import Any, ClassVar, Mapping


class StoreError(RuntimeError):
    """Base error for local Studio persistence failures."""


class StoreValidationError(StoreError, ValueError):
    """Raised when a record or path violates the Studio contract."""


class RecordNotFound(StoreError, KeyError):
    """Raised when an opaque record identifier does not exist."""


class RevisionConflict(StoreError):
    """Raised when a compare-and-swap update uses a stale revision."""

    def __init__(self, message: str, *, expected: int, actual: int) -> None:
        super().__init__(message)
        self.expected = expected
        self.actual = actual


class IdempotencyConflict(StoreError):
    """Raised when one idempotency key is reused for a different request."""


def canonical_json(value: Any) -> str:
    """Serialize JSON-compatible data deterministically.

    ``allow_nan=False`` is intentional: NaN and infinity are not valid JSON
    and would make a persisted record non-portable between Python and UE.
    """

    try:
        return json.dumps(
            value,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
    except (TypeError, ValueError) as exc:
        raise StoreValidationError(f"value is not canonical JSON: {exc}") from exc


def canonical_bytes(value: Any) -> bytes:
    """Return UTF-8 bytes for deterministic persistence and hashing."""

    return canonical_json(value).encode("utf-8")


def canonical_digest(value: Any) -> str:
    """Return the SHA-256 digest of canonical JSON bytes."""

    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def _mapping_copy(value: Mapping[str, Any], context: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise StoreValidationError(f"{context} must be an object")
    copied = copy.deepcopy(dict(value))
    # Validate eagerly so an invalid payload can never enter the store.
    canonical_json(copied)
    return copied


@dataclass(frozen=True)
class StudioRecord:
    """Common immutable view of one persisted Studio resource."""

    kind: ClassVar[str] = "record"
    schema: ClassVar[str] = "lingtu.sim.studio.record.v1"

    id: str
    revision: int
    created_at: str
    updated_at: str
    status: str
    payload: Mapping[str, Any]

    def __post_init__(self) -> None:
        if not isinstance(self.id, str) or not self.id:
            raise StoreValidationError("record id must be a non-empty opaque string")
        if isinstance(self.revision, bool) or not isinstance(self.revision, int) or self.revision < 1:
            raise StoreValidationError("record revision must be a positive integer")
        if not isinstance(self.status, str) or not self.status:
            raise StoreValidationError("record status must be a non-empty string")
        object.__setattr__(self, "payload", _mapping_copy(self.payload, "record.payload"))

    def to_dict(self) -> dict[str, Any]:
        """Return a detached JSON-compatible representation."""

        return {
            "schema": self.schema,
            "kind": self.kind,
            "id": self.id,
            "revision": self.revision,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "status": self.status,
            "payload": copy.deepcopy(dict(self.payload)),
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> StudioRecord:
        """Build a record from a strict persisted envelope."""

        required = {"schema", "kind", "id", "revision", "created_at", "updated_at", "status", "payload"}
        if not isinstance(value, Mapping) or set(value) != required:
            raise StoreValidationError(f"{cls.kind} record has an invalid envelope")
        if value["schema"] != cls.schema or value["kind"] != cls.kind:
            raise StoreValidationError(f"record is not a {cls.kind} v1 envelope")
        if not all(isinstance(value[field], str) and value[field] for field in ("id", "created_at", "updated_at", "status")):
            raise StoreValidationError(f"{cls.kind} record has invalid string fields")
        return cls(
            id=value["id"],
            revision=value["revision"],
            created_at=value["created_at"],
            updated_at=value["updated_at"],
            status=value["status"],
            payload=value["payload"],
        )


@dataclass(frozen=True)
class ImportJobRecord(StudioRecord):
    """Durable state of one robot/world import request."""

    kind: ClassVar[str] = "import_job"
    schema: ClassVar[str] = "lingtu.sim.studio.import-job.v1"


@dataclass(frozen=True)
class SessionDraftRecord(StudioRecord):
    """Versioned authoring document for one simulation session."""

    kind: ClassVar[str] = "session_draft"
    schema: ClassVar[str] = "lingtu.sim.studio.session-draft.v1"


@dataclass(frozen=True)
class SceneDraftRecord(StudioRecord):
    """Versioned, compiler-validated scene authoring document."""

    kind: ClassVar[str] = "scene_draft"
    schema: ClassVar[str] = "lingtu.sim.studio.scene-draft.v1"


@dataclass(frozen=True)
class BundleRecord(StudioRecord):
    """Immutable or staged resolved-session bundle metadata."""

    kind: ClassVar[str] = "bundle"
    schema: ClassVar[str] = "lingtu.sim.studio.bundle.v1"


@dataclass(frozen=True)
class RunRecord(StudioRecord):
    """Durable lifecycle and artifact metadata for one Studio run."""

    kind: ClassVar[str] = "run"
    schema: ClassVar[str] = "lingtu.sim.studio.run.v1"


@dataclass(frozen=True)
class IdempotencyRecord:
    """Durable response cache for one operation key and request digest."""

    schema: ClassVar[str] = "lingtu.sim.studio.idempotency.v1"

    id: str
    operation: str
    key: str
    request_digest: str
    resource_kind: str
    resource_id: str
    response: Mapping[str, Any]
    created_at: str

    def __post_init__(self) -> None:
        for name in ("id", "operation", "key", "request_digest", "resource_kind", "resource_id", "created_at"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value:
                raise StoreValidationError(f"idempotency.{name} must be a non-empty string")
        object.__setattr__(self, "response", _mapping_copy(self.response, "idempotency.response"))

    def to_dict(self) -> dict[str, Any]:
        """Return a detached persisted representation."""

        return {
            "schema": self.schema,
            "id": self.id,
            "operation": self.operation,
            "key": self.key,
            "request_digest": self.request_digest,
            "resource_kind": self.resource_kind,
            "resource_id": self.resource_id,
            "response": copy.deepcopy(dict(self.response)),
            "created_at": self.created_at,
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> IdempotencyRecord:
        """Load and validate a strict idempotency envelope."""

        required = {
            "schema",
            "id",
            "operation",
            "key",
            "request_digest",
            "resource_kind",
            "resource_id",
            "response",
            "created_at",
        }
        if not isinstance(value, Mapping) or set(value) != required or value["schema"] != cls.schema:
            raise StoreValidationError("idempotency record has an invalid envelope")
        return cls(
            id=value["id"],
            operation=value["operation"],
            key=value["key"],
            request_digest=value["request_digest"],
            resource_kind=value["resource_kind"],
            resource_id=value["resource_id"],
            response=value["response"],
            created_at=value["created_at"],
        )


RECORD_TYPES: dict[str, type[StudioRecord]] = {
    ImportJobRecord.kind: ImportJobRecord,
    SessionDraftRecord.kind: SessionDraftRecord,
    SceneDraftRecord.kind: SceneDraftRecord,
    BundleRecord.kind: BundleRecord,
    RunRecord.kind: RunRecord,
}

__all__ = [
    "RECORD_TYPES",
    "BundleRecord",
    "IdempotencyConflict",
    "IdempotencyRecord",
    "ImportJobRecord",
    "RecordNotFound",
    "RevisionConflict",
    "RunRecord",
    "SceneDraftRecord",
    "SessionDraftRecord",
    "StoreError",
    "StoreValidationError",
    "StudioRecord",
    "canonical_bytes",
    "canonical_digest",
    "canonical_json",
]
