"""Atomic, local-only JSON persistence for SimStudio resources."""

from __future__ import annotations

import contextlib
import copy
import os
import re
import secrets
import stat
import threading
import time
import uuid
from collections.abc import Callable, Mapping
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from typing import Any, Iterator

from sim.runtime.coordinator.atomic_file import replace_file_with_retry

from .models import (
    RECORD_TYPES,
    BundleRecord,
    IdempotencyConflict,
    IdempotencyRecord,
    ImportJobRecord,
    RecordNotFound,
    RevisionConflict,
    RunRecord,
    SceneDraftRecord,
    SessionDraftRecord,
    StoreError,
    StoreValidationError,
    StudioRecord,
    canonical_bytes,
    canonical_digest,
)

_ID_PATTERN = re.compile(r"^[a-f0-9]{32}$")
_IDEMPOTENCY_MAX = 256
_REPARSE_POINT = 0x0400
_LOCK_TIMEOUT_S = 30.0
_LOCK_POLL_S = 0.01
_TRANSACTION_SCHEMA = "lingtu.sim.studio.store-transaction.v1"
_TRANSACTION_ID_PATTERN = _ID_PATTERN
_UE_IDENTIFIER_FIELDS = {
    "asset_id",
    "asset_path",
    "level",
    "material_id",
    "mesh_id",
    "primary_asset_id",
    "primary_asset_path",
    "ue_asset",
    "ue_asset_id",
    "ue_path",
    "unreal_asset",
    "unreal_asset_id",
    "unreal_path",
}


class _CrossProcessLock:
    """A one-byte advisory lock that works for Windows and POSIX workers."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self._handle: Any | None = None

    def __enter__(self) -> _CrossProcessLock:
        handle = self.path.open("a+b")
        self._handle = handle
        handle.seek(0, os.SEEK_END)
        if handle.tell() == 0:
            handle.write(b"0")
            handle.flush()
            os.fsync(handle.fileno())
        deadline = time.monotonic() + _LOCK_TIMEOUT_S
        while True:
            handle.seek(0)
            try:
                if os.name == "nt":
                    import msvcrt

                    msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
                else:
                    import fcntl

                    fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                return self
            except (BlockingIOError, OSError) as exc:
                if time.monotonic() >= deadline:
                    handle.close()
                    self._handle = None
                    raise StoreError(f"timed out acquiring Studio store lock: {self.path}") from exc
                time.sleep(_LOCK_POLL_S)

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        handle = self._handle
        self._handle = None
        if handle is None:
            return
        try:
            handle.seek(0)
            if os.name == "nt":
                import msvcrt

                msvcrt.locking(handle.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl

                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        finally:
            handle.close()


def _now() -> str:
    """Return a stable UTC timestamp suitable for a JSON record."""

    return datetime.now(timezone.utc).isoformat(timespec="microseconds").replace("+00:00", "Z")


def _opaque_id() -> str:
    return uuid.uuid4().hex


class StudioStore:
    """Persist Studio state beneath one private, validated root.

    The store accepts only service-owned relative paths.  It never resolves a
    caller-provided absolute path, executable, port, or network endpoint.
    Records use atomic replacement, a cross-process lock, and a replayable
    journal for multi-file idempotent commits; callers use the revision in
    each record for compare-and-swap updates.
    """

    _DIRECTORIES = {
        "inbox": "inbox",
        "import_job": "records/import_jobs",
        "session_draft": "records/session_drafts",
        "scene_draft": "records/scene_drafts",
        "bundle": "records/bundles",
        "run": "records/runs",
        "idempotency": "records/idempotency",
    }

    def __init__(
        self,
        root: Path,
        *,
        clock: Callable[[], str] = _now,
        id_factory: Callable[[], str] = _opaque_id,
    ) -> None:
        self.root = self._prepare_root(root)
        self._clock = clock
        self._id_factory = id_factory
        self._lock = threading.RLock()
        self._lock_path = self.root / ".studio.lock"
        self._transaction_dir = self.root / ".transactions"
        self._ensure_directory(self._transaction_dir)
        self._ensure_lock_file(self._lock_path)
        self._roots: dict[str, Path] = {}
        for kind, relative in self._DIRECTORIES.items():
            directory = self.root / Path(*relative.split("/"))
            self._ensure_directory(directory)
            self._roots[kind] = directory
        self._assert_no_reparse_components(self._lock_path, below=self.root)
        with _CrossProcessLock(self._lock_path):
            self._recover_transactions_locked()

    @classmethod
    def _prepare_root(cls, root: Path) -> Path:
        candidate = Path(os.path.abspath(os.fspath(root)))
        cls._ensure_directory(candidate)
        return candidate

    @classmethod
    def _ensure_directory(cls, directory: Path) -> None:
        candidate = Path(os.path.abspath(os.fspath(directory)))
        current = Path(candidate.anchor)
        for part in candidate.parts:
            if part == candidate.anchor:
                continue
            current = current / part
            if cls._is_reparse_point(current):
                raise StoreValidationError(f"Studio store directory must not be a reparse point: {current}")
            if current.exists():
                if not current.is_dir():
                    raise StoreValidationError(f"Studio store path is not a directory: {current}")
                continue
            try:
                current.mkdir()
            except FileExistsError:
                pass
            if cls._is_reparse_point(current) or not current.is_dir():
                raise StoreValidationError(f"Studio store directory is not a plain directory: {current}")

    @staticmethod
    def _is_reparse_point(path: Path) -> bool:
        try:
            metadata = os.lstat(path)
        except FileNotFoundError:
            return False
        if stat.S_ISLNK(metadata.st_mode):
            return True
        return bool(getattr(metadata, "st_file_attributes", 0) & _REPARSE_POINT)

    @classmethod
    def _assert_no_reparse_components(cls, path: Path, *, below: Path | None = None) -> None:
        candidate = Path(os.path.abspath(os.fspath(path)))
        if below is not None:
            base = Path(os.path.abspath(os.fspath(below)))
            try:
                candidate.relative_to(base)
            except ValueError as exc:
                raise StoreValidationError("candidate path is outside its owned root") from exc
            current = base
            if cls._is_reparse_point(current):
                raise StoreValidationError(f"owned path contains a reparse point: {current}")
            relative_parts = candidate.relative_to(base).parts
        else:
            current = Path(candidate.anchor)
            relative_parts = tuple(part for part in candidate.parts if part != candidate.anchor)
        for part in relative_parts:
            current = current / part
            if cls._is_reparse_point(current):
                raise StoreValidationError(f"owned path contains a reparse point: {current}")

    @classmethod
    def _ensure_lock_file(cls, path: Path) -> None:
        cls._assert_no_reparse_components(path)
        if path.exists() and not path.is_file():
            raise StoreValidationError(f"Studio store lock is not a regular file: {path}")
        with path.open("ab") as handle:
            if handle.tell() == 0:
                handle.write(b"0")
                handle.flush()
                os.fsync(handle.fileno())

    @staticmethod
    def _validate_opaque_id(value: str) -> str:
        if not isinstance(value, str) or _ID_PATTERN.fullmatch(value) is None:
            raise StoreValidationError("record id must be a 32-character opaque hexadecimal id")
        return value

    @staticmethod
    def validate_relative_path(value: str, *, context: str = "path") -> str:
        """Validate a service-owned POSIX relative path.

        A colon is rejected anywhere, including ``file.txt:stream``, because
        NTFS alternate data streams must not be reachable through Studio.
        """

        if not isinstance(value, str) or not value or "\x00" in value:
            raise StoreValidationError(f"{context} must be a non-empty relative path")
        if value.startswith(("/", "\\")) or "\\" in value or ":" in value:
            raise StoreValidationError(f"{context} must be a safe relative POSIX path")
        raw_parts = value.split("/")
        path = PurePosixPath(value)
        if path.is_absolute() or any(part in {"", ".", ".."} for part in raw_parts):
            raise StoreValidationError(f"{context} must not contain traversal components")
        if any(character.isspace() for character in value):
            raise StoreValidationError(f"{context} must not contain whitespace")
        return path.as_posix()

    def owned_path(self, area: str, relative: str, *, must_exist: bool = False) -> Path:
        """Resolve one path below a named store area, rejecting links/escape."""

        if area not in self._roots:
            raise StoreValidationError(f"unknown Studio store area: {area}")
        safe = self.validate_relative_path(relative, context=f"{area} path")
        base = self._roots[area]
        candidate = base.joinpath(*safe.split("/"))
        self._assert_no_reparse_components(candidate, below=self.root)
        try:
            candidate.relative_to(base)
        except ValueError as exc:
            raise StoreValidationError(f"{area} path escapes the owned root") from exc
        if must_exist and not candidate.exists():
            raise RecordNotFound(f"owned {area} path does not exist: {safe}")
        return candidate

    def inbox_path(self, relative: str, *, must_exist: bool = False) -> Path:
        """Resolve an import source relative to the Studio inbox."""

        return self.owned_path("inbox", relative, must_exist=must_exist)

    def record_path(self, kind: str, record_id: str, *, must_exist: bool = False) -> Path:
        """Resolve a persisted record by opaque id."""

        self._validate_opaque_id(record_id)
        return self.owned_path(kind, f"{record_id}.json", must_exist=must_exist)

    def _new_id(self) -> str:
        value = self._id_factory()
        return self._validate_opaque_id(value)

    def _read_json(self, path: Path) -> Any:
        self._assert_no_reparse_components(path, below=self.root)
        try:
            import json

            return json.loads(path.read_text(encoding="utf-8"))
        except FileNotFoundError as exc:
            raise RecordNotFound(str(path)) from exc
        except (OSError, ValueError) as exc:
            raise StoreValidationError(f"cannot read Studio JSON record: {path}") from exc

    def _atomic_write(self, path: Path, value: Any) -> None:
        self._assert_no_reparse_components(path, below=self.root)
        self._ensure_directory(path.parent)
        self._assert_no_reparse_components(path, below=self.root)
        temp = path.with_name(f".{path.name}.{secrets.token_hex(8)}.tmp")
        try:
            with temp.open("xb") as handle:
                handle.write(canonical_bytes(value))
                handle.flush()
                os.fsync(handle.fileno())
            replace_file_with_retry(temp, path)
            self._assert_no_reparse_components(path, below=self.root)
            try:
                directory_handle = os.open(path.parent, os.O_RDONLY)
            except OSError:
                directory_handle = None
            if directory_handle is not None:
                try:
                    os.fsync(directory_handle)
                finally:
                    os.close(directory_handle)
        finally:
            temp.unlink(missing_ok=True)

    @contextlib.contextmanager
    def _operation_lock(self) -> Iterator[None]:
        """Serialize each public operation across Store instances/processes."""

        with self._lock:
            self._assert_no_reparse_components(self._lock_path, below=self.root)
            with _CrossProcessLock(self._lock_path):
                self._recover_transactions_locked()
                yield

    def _transaction_path(self, transaction_id: str) -> Path:
        if _TRANSACTION_ID_PATTERN.fullmatch(transaction_id) is None:
            raise StoreValidationError("invalid Studio store transaction id")
        path = self._transaction_dir / f"{transaction_id}.json"
        self._assert_no_reparse_components(path, below=self.root)
        return path

    def _validated_transaction_writes(self, value: Mapping[str, Any]) -> list[tuple[str, str, Any]]:
        required = {"schema", "id", "writes"}
        if not isinstance(value, Mapping) or set(value) != required or value.get("schema") != _TRANSACTION_SCHEMA:
            raise StoreValidationError("Studio store transaction journal has an invalid envelope")
        transaction_id = value.get("id")
        if not isinstance(transaction_id, str) or _TRANSACTION_ID_PATTERN.fullmatch(transaction_id) is None:
            raise StoreValidationError("Studio store transaction journal has an invalid id")
        raw_writes = value.get("writes")
        if not isinstance(raw_writes, list) or not raw_writes:
            raise StoreValidationError("Studio store transaction journal has no writes")
        writes: list[tuple[str, str, Any]] = []
        seen: set[tuple[str, str]] = set()
        for item in raw_writes:
            if not isinstance(item, Mapping) or set(item) != {"area", "relative", "value"}:
                raise StoreValidationError("Studio store transaction journal has an invalid write")
            area = item["area"]
            relative = item["relative"]
            if not isinstance(area, str) or area not in {
                "import_job",
                "session_draft",
                "scene_draft",
                "bundle",
                "run",
                "idempotency",
            }:
                raise StoreValidationError("Studio store transaction journal targets an invalid area")
            if not isinstance(relative, str):
                raise StoreValidationError("Studio store transaction journal has an invalid relative path")
            relative = self.validate_relative_path(relative, context="transaction.relative")
            if (area, relative) in seen:
                raise StoreValidationError("Studio store transaction journal contains duplicate writes")
            seen.add((area, relative))
            if not isinstance(item["value"], Mapping):
                raise StoreValidationError("Studio store transaction values must be JSON objects")
            self._validate_persisted_value(area, relative, item["value"])
            self.owned_path(area, relative)
            writes.append((area, relative, copy.deepcopy(item["value"])))
        return writes

    def _commit_writes(self, writes: list[tuple[str, str, Any]]) -> None:
        """Commit one or more JSON files with replayable crash recovery.

        The journal makes the operation recoverable after any process crash
        between the resource and idempotency file replacements.  Windows can
        guarantee atomic replacement of each file, but not a cross-file rename;
        the remaining durability boundary is directory-entry persistence when
        Windows does not expose directory fsync.
        """

        if not writes:
            raise StoreValidationError("Studio store transaction must contain writes")
        transaction_id = uuid.uuid4().hex
        journal = {
            "schema": _TRANSACTION_SCHEMA,
            "id": transaction_id,
            "writes": [
                {"area": area, "relative": relative, "value": copy.deepcopy(value)}
                for area, relative, value in writes
            ],
        }
        self._validated_transaction_writes(journal)
        journal_path = self._transaction_path(transaction_id)
        self._atomic_write(journal_path, journal)
        for area, relative, value in writes:
            self._atomic_write(self.owned_path(area, relative), value)
        self._assert_no_reparse_components(journal_path, below=self.root)
        try:
            journal_path.unlink()
        except FileNotFoundError:
            pass
        self._flush_directory_best_effort(self._transaction_dir)

    def _recover_transactions_locked(self) -> None:
        self._assert_no_reparse_components(self._transaction_dir, below=self.root)
        for journal_path in sorted(self._transaction_dir.glob("*.json"), key=lambda item: item.name):
            if self._is_reparse_point(journal_path) or not journal_path.is_file():
                raise StoreValidationError(f"Studio store transaction journal is not a regular file: {journal_path}")
            journal = self._read_json(journal_path)
            writes = self._validated_transaction_writes(journal)
            for area, relative, value in writes:
                self._atomic_write(self.owned_path(area, relative), value)
            self._assert_no_reparse_components(journal_path, below=self.root)
            journal_path.unlink()
        self._flush_directory_best_effort(self._transaction_dir)

    @staticmethod
    def _flush_directory_best_effort(directory: Path) -> None:
        try:
            directory_handle = os.open(directory, os.O_RDONLY)
        except OSError:
            return
        try:
            os.fsync(directory_handle)
        except OSError:
            pass
        finally:
            os.close(directory_handle)

    @staticmethod
    def _validate_payload_path(payload: Mapping[str, Any], key: str, *, context: str) -> None:
        value = payload.get(key)
        if value is not None:
            StudioStore.validate_relative_path(value, context=f"{context}.{key}")

    @staticmethod
    def _is_ue_identifier_field(key: str) -> bool:
        # Keep the exception deliberately closed.  A suffix-based rule would
        # let arbitrary user fields (for example ``metadata_asset_path``)
        # smuggle an Unreal namespace through the filesystem-path guard.
        return key in _UE_IDENTIFIER_FIELDS

    @staticmethod
    def _is_absolute_filesystem_value(value: str) -> bool:
        return bool(re.match(r"^(?:[A-Za-z]:[\\/]|\\\\|//|[\\/])", value))

    def _validate_nested_payload_paths(self, value: Any, *, context: str) -> None:
        """Reject filesystem paths recursively while allowing explicit UE IDs."""

        dangerous = {
            "absolute_path",
            "filesystem_path",
            "output",
            "output_dir",
            "output_path",
            "working_dir",
        }
        managed = {"source_path", "result_path", "bundle_path", "artifact_path"}
        if isinstance(value, Mapping):
            for key, child in value.items():
                if not isinstance(key, str):
                    raise StoreValidationError(f"{context} contains a non-string field name")
                lowered = key.lower()
                child_context = f"{context}.{key}"
                if lowered in dangerous:
                    raise StoreValidationError(f"{child_context} is not a managed Store field")
                if isinstance(child, str):
                    is_ue_id = (
                        self._is_ue_identifier_field(lowered)
                        and child.startswith(("/Game/", "/Engine/", "/Script/"))
                    )
                    if self._is_absolute_filesystem_value(child) and not is_ue_id:
                        raise StoreValidationError(f"{child_context} must not be an absolute filesystem path")
                    if lowered in managed or lowered.endswith("_path") or lowered in {"path", "root"}:
                        if not is_ue_id:
                            self.validate_relative_path(child, context=child_context)
                elif lowered in managed or lowered.endswith("_path") or lowered in {"path", "root"}:
                    if child is not None:
                        raise StoreValidationError(f"{child_context} must be a string path")
                self._validate_nested_payload_paths(child, context=child_context)
        elif isinstance(value, list):
            for index, child in enumerate(value):
                self._validate_nested_payload_paths(child, context=f"{context}[{index}]")

    def _validate_record_payload(self, kind: str, payload: Mapping[str, Any]) -> dict[str, Any]:
        copied = copy.deepcopy(dict(payload))
        if not isinstance(copied, Mapping):  # pragma: no cover - guarded by type contract
            raise StoreValidationError("record payload must be an object")
        self._validate_nested_payload_paths(copied, context=kind)
        # canonical_bytes also rejects non-JSON values and NaN/Infinity.
        canonical_bytes(copied)
        return copied

    def _record_class(self, kind: str) -> type[StudioRecord]:
        try:
            return RECORD_TYPES[kind]
        except KeyError as exc:
            raise StoreValidationError(f"unknown Studio record kind: {kind}") from exc

    def _parse_record(self, kind: str, value: Mapping[str, Any], *, expected_id: str | None = None) -> StudioRecord:
        cls = self._record_class(kind)
        record = cls.from_dict(value)
        self._validate_opaque_id(record.id)
        if expected_id is not None and record.id != expected_id:
            raise StoreValidationError(f"{kind} record id does not match its filename")
        self._validate_record_payload(kind, record.payload)
        return record

    def _parse_idempotency(self, value: Mapping[str, Any]) -> IdempotencyRecord:
        record = IdempotencyRecord.from_dict(value)
        self._validate_opaque_id(record.id)
        self._validate_idempotency_key(record.key)
        self._record_class(record.resource_kind)
        self._validate_opaque_id(record.resource_id)
        response = self._parse_record(record.resource_kind, record.response, expected_id=record.resource_id)
        if record.operation not in {f"create:{record.resource_kind}", f"update:{record.resource_kind}"}:
            raise StoreValidationError("idempotency operation does not match its resource kind")
        if response.to_dict() != dict(record.response):
            raise StoreValidationError("idempotency response is not a canonical resource record")
        return record

    def _validate_persisted_value(self, area: str, relative: str, value: Mapping[str, Any]) -> None:
        if area in RECORD_TYPES:
            record = self._parse_record(area, value)
            if relative != f"{record.id}.json":
                raise StoreValidationError("record path does not match its opaque id")
            return
        if area == "idempotency":
            record = self._parse_idempotency(value)
            if relative != self._idempotency_relative(record.operation, record.key):
                raise StoreValidationError("idempotency path does not match its operation and key")
            return
        raise StoreValidationError(f"unsupported persisted Store area: {area}")

    def _load_record(self, kind: str, record_id: str) -> StudioRecord:
        path = self.record_path(kind, record_id, must_exist=True)
        return self._parse_record(kind, self._read_json(path), expected_id=record_id)

    def _write_record(self, record: StudioRecord) -> StudioRecord:
        path = self.record_path(record.kind, record.id)
        self._atomic_write(path, record.to_dict())
        return record

    def _create(
        self,
        kind: str,
        payload: Mapping[str, Any],
        *,
        status: str,
        idempotency_key: str | None,
    ) -> StudioRecord:
        request = {"kind": kind, "payload": payload, "status": status}
        if idempotency_key is not None:
            existing = self._lookup_idempotency("create:" + kind, idempotency_key, request)
            if existing is not None:
                return self._load_record(existing.resource_kind, existing.resource_id)
        record_id = self._new_id()
        now = self._clock()
        record = self._record_class(kind)(
            id=record_id,
            revision=1,
            created_at=now,
            updated_at=now,
            status=status,
            payload=self._validate_record_payload(kind, payload),
        )
        if idempotency_key is not None:
            idempotency = self._make_idempotency(
                "create:" + kind,
                idempotency_key,
                request,
                resource_kind=kind,
                resource_id=record.id,
                response=record.to_dict(),
            )
            self._commit_writes(
                [
                    (kind, f"{record.id}.json", record.to_dict()),
                    (
                        "idempotency",
                        self._idempotency_relative(idempotency.operation, idempotency.key),
                        idempotency.to_dict(),
                    ),
                ]
            )
        else:
            self._write_record(record)
        return record

    def _update(
        self,
        kind: str,
        record_id: str,
        *,
        expected_revision: int,
        payload: Mapping[str, Any],
        status: str | None,
        idempotency_key: str | None,
    ) -> StudioRecord:
        if isinstance(expected_revision, bool) or not isinstance(expected_revision, int) or expected_revision < 1:
            raise StoreValidationError("expected_revision must be a positive integer")
        request = {
            "kind": kind,
            "record_id": record_id,
            "expected_revision": expected_revision,
            "payload": payload,
            "status": status,
        }
        if idempotency_key is not None:
            existing = self._lookup_idempotency("update:" + kind, idempotency_key, request)
            if existing is not None:
                return self._load_record(existing.resource_kind, existing.resource_id)
        current = self._load_record(kind, record_id)
        if current.revision != expected_revision:
            raise RevisionConflict(
                f"stale {kind} revision for {record_id}: expected {expected_revision}, current {current.revision}",
                expected=expected_revision,
                actual=current.revision,
            )
        updated = self._record_class(kind)(
            id=current.id,
            revision=current.revision + 1,
            created_at=current.created_at,
            updated_at=self._clock(),
            status=current.status if status is None else status,
            payload=self._validate_record_payload(kind, payload),
        )
        if idempotency_key is not None:
            idempotency = self._make_idempotency(
                "update:" + kind,
                idempotency_key,
                request,
                resource_kind=kind,
                resource_id=updated.id,
                response=updated.to_dict(),
            )
            self._commit_writes(
                [
                    (kind, f"{updated.id}.json", updated.to_dict()),
                    (
                        "idempotency",
                        self._idempotency_relative(idempotency.operation, idempotency.key),
                        idempotency.to_dict(),
                    ),
                ]
            )
        else:
            self._write_record(updated)
        return updated

    def _list(self, kind: str) -> tuple[StudioRecord, ...]:
        directory = self._roots[kind]
        self._assert_no_reparse_components(directory, below=self.root)
        records: list[StudioRecord] = []
        for path in sorted(directory.glob("*.json"), key=lambda item: item.name):
            self._assert_no_reparse_components(path, below=self.root)
            records.append(self._load_record(kind, path.stem))
        return tuple(sorted(records, key=lambda record: (record.created_at, record.id)))

    def _idempotency_path(self, operation: str, key: str) -> Path:
        return self.owned_path("idempotency", self._idempotency_relative(operation, key))

    @staticmethod
    def _idempotency_relative(operation: str, key: str) -> str:
        return canonical_digest({"operation": operation, "key": key}) + ".json"

    def _make_idempotency(
        self,
        operation: str,
        key: str,
        request: Mapping[str, Any],
        *,
        resource_kind: str,
        resource_id: str,
        response: Mapping[str, Any],
    ) -> IdempotencyRecord:
        return IdempotencyRecord(
            id=self._new_id(),
            operation=operation,
            key=key,
            request_digest=canonical_digest(request),
            resource_kind=resource_kind,
            resource_id=resource_id,
            response=response,
            created_at=self._clock(),
        )

    @staticmethod
    def _validate_idempotency_key(key: str) -> str:
        if not isinstance(key, str) or not key or len(key) > _IDEMPOTENCY_MAX or "\x00" in key:
            raise StoreValidationError("idempotency_key must be 1-256 characters")
        return key

    def _lookup_idempotency(
        self,
        operation: str,
        key: str,
        request: Mapping[str, Any],
    ) -> IdempotencyRecord | None:
        key = self._validate_idempotency_key(key)
        path = self._idempotency_path(operation, key)
        if not path.exists():
            return None
        record = self._parse_idempotency(self._read_json(path))
        if record.operation != operation or record.key != key:
            raise StoreValidationError("idempotency index does not match its key")
        request_digest = canonical_digest(request)
        if record.request_digest != request_digest:
            raise IdempotencyConflict(f"idempotency key already belongs to a different {operation} request")
        return record

    def _save_idempotency(self, record: IdempotencyRecord) -> None:
        self._atomic_write(self._idempotency_path(record.operation, record.key), record.to_dict())

    def get_idempotency(self, operation: str, key: str) -> IdempotencyRecord | None:
        """Return one idempotency record without mutating the store."""

        with self._operation_lock():
            key = self._validate_idempotency_key(key)
            path = self._idempotency_path(operation, key)
            if not path.exists():
                return None
            record = self._parse_idempotency(self._read_json(path))
            if record.operation != operation or record.key != key:
                raise StoreValidationError("idempotency index does not match its key")
            if path.name != self._idempotency_relative(record.operation, record.key):
                raise StoreValidationError("idempotency path does not match its operation and key")
            return record

    def create_import_job(self, payload: Mapping[str, Any], *, status: str = "queued", idempotency_key: str | None = None) -> ImportJobRecord:
        """Create or replay one import job."""

        with self._operation_lock():
            return self._create("import_job", payload, status=status, idempotency_key=idempotency_key)  # type: ignore[return-value]

    def get_import_job(self, record_id: str) -> ImportJobRecord:
        """Load one import job."""

        with self._operation_lock():
            return self._load_record("import_job", record_id)  # type: ignore[return-value]

    def update_import_job(self, record_id: str, *, expected_revision: int, payload: Mapping[str, Any], status: str | None = None, idempotency_key: str | None = None) -> ImportJobRecord:
        """CAS-update one import job."""

        with self._operation_lock():
            return self._update("import_job", record_id, expected_revision=expected_revision, payload=payload, status=status, idempotency_key=idempotency_key)  # type: ignore[return-value]

    def list_import_jobs(self) -> tuple[ImportJobRecord, ...]:
        """List import jobs in deterministic order."""

        with self._operation_lock():
            return self._list("import_job")  # type: ignore[return-value]

    def create_session_draft(self, payload: Mapping[str, Any], *, status: str = "draft", idempotency_key: str | None = None) -> SessionDraftRecord:
        """Create or replay one session draft."""

        with self._operation_lock():
            return self._create("session_draft", payload, status=status, idempotency_key=idempotency_key)  # type: ignore[return-value]

    def get_session_draft(self, record_id: str) -> SessionDraftRecord:
        """Load one session draft."""

        with self._operation_lock():
            return self._load_record("session_draft", record_id)  # type: ignore[return-value]

    def update_session_draft(self, record_id: str, *, expected_revision: int, payload: Mapping[str, Any], status: str | None = None, idempotency_key: str | None = None) -> SessionDraftRecord:
        """CAS-update one session draft."""

        with self._operation_lock():
            return self._update("session_draft", record_id, expected_revision=expected_revision, payload=payload, status=status, idempotency_key=idempotency_key)  # type: ignore[return-value]

    def list_session_drafts(self) -> tuple[SessionDraftRecord, ...]:
        """List session drafts in deterministic order."""

        with self._operation_lock():
            return self._list("session_draft")  # type: ignore[return-value]

    def create_scene_draft(
        self,
        payload: Mapping[str, Any],
        *,
        status: str = "draft",
        idempotency_key: str | None = None,
    ) -> SceneDraftRecord:
        """Create or replay one compiler-validated scene draft."""

        with self._operation_lock():
            return self._create(
                "scene_draft",
                payload,
                status=status,
                idempotency_key=idempotency_key,
            )  # type: ignore[return-value]

    def get_scene_draft(self, record_id: str) -> SceneDraftRecord:
        """Load one scene draft."""

        with self._operation_lock():
            return self._load_record("scene_draft", record_id)  # type: ignore[return-value]

    def update_scene_draft(
        self,
        record_id: str,
        *,
        expected_revision: int,
        payload: Mapping[str, Any],
        status: str | None = None,
        idempotency_key: str | None = None,
    ) -> SceneDraftRecord:
        """CAS-update one compiler-validated scene draft."""

        with self._operation_lock():
            return self._update(
                "scene_draft",
                record_id,
                expected_revision=expected_revision,
                payload=payload,
                status=status,
                idempotency_key=idempotency_key,
            )  # type: ignore[return-value]

    def list_scene_drafts(self) -> tuple[SceneDraftRecord, ...]:
        """List scene drafts in deterministic order."""

        with self._operation_lock():
            return self._list("scene_draft")  # type: ignore[return-value]

    def create_bundle(self, payload: Mapping[str, Any], *, status: str = "ready", idempotency_key: str | None = None) -> BundleRecord:
        """Create or replay one resolved bundle record."""

        with self._operation_lock():
            return self._create("bundle", payload, status=status, idempotency_key=idempotency_key)  # type: ignore[return-value]

    def get_bundle(self, record_id: str) -> BundleRecord:
        """Load one bundle record."""

        with self._operation_lock():
            return self._load_record("bundle", record_id)  # type: ignore[return-value]

    def list_bundles(self) -> tuple[BundleRecord, ...]:
        """List bundle records in deterministic order."""

        with self._operation_lock():
            return self._list("bundle")  # type: ignore[return-value]

    def create_run(self, payload: Mapping[str, Any], *, status: str = "created", idempotency_key: str | None = None) -> RunRecord:
        """Create or replay one run record."""

        with self._operation_lock():
            return self._create("run", payload, status=status, idempotency_key=idempotency_key)  # type: ignore[return-value]

    def get_run(self, record_id: str) -> RunRecord:
        """Load one run record."""

        with self._operation_lock():
            return self._load_record("run", record_id)  # type: ignore[return-value]

    def update_run(self, record_id: str, *, expected_revision: int, payload: Mapping[str, Any], status: str | None = None, idempotency_key: str | None = None) -> RunRecord:
        """CAS-update one run record."""

        with self._operation_lock():
            return self._update("run", record_id, expected_revision=expected_revision, payload=payload, status=status, idempotency_key=idempotency_key)  # type: ignore[return-value]

    def list_runs(self) -> tuple[RunRecord, ...]:
        """List run records in deterministic order."""

        with self._operation_lock():
            return self._list("run")  # type: ignore[return-value]


__all__ = ["StudioStore"]
