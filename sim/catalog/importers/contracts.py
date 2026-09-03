"""Shared contracts for simulation package intake, quarantine, and promotion."""

from __future__ import annotations

import hashlib
import json
import os
import re
import stat
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path, PurePosixPath
from typing import Any


class ImportCode(str, Enum):
    """Stable diagnostics emitted by every simulation importer."""

    INVALID_REQUEST = "SIMIMPORT_INVALID_REQUEST"
    SOURCE_MISSING = "SIMIMPORT_SOURCE_MISSING"
    UNSAFE_SOURCE = "SIMIMPORT_UNSAFE_SOURCE"
    UNSAFE_ARCHIVE = "SIMIMPORT_UNSAFE_ARCHIVE"
    ARCHIVE_LIMIT = "SIMIMPORT_ARCHIVE_LIMIT"
    LICENSE_REQUIRED = "SIMIMPORT_LICENSE_REQUIRED"
    PROVENANCE_REQUIRED = "SIMIMPORT_PROVENANCE_REQUIRED"
    UNIT_AMBIGUOUS = "SIMIMPORT_UNIT_AMBIGUOUS"
    SOURCE_FORMAT_UNSUPPORTED = "SIMIMPORT_SOURCE_FORMAT_UNSUPPORTED"
    CONVERTER_UNAVAILABLE = "SIMIMPORT_CONVERTER_UNAVAILABLE"
    MODEL_INVALID = "SIMIMPORT_MODEL_INVALID"
    GLOBAL_PHYSICS_OWNERSHIP = "SIMIMPORT_GLOBAL_PHYSICS_OWNERSHIP"
    ASSET_MISSING = "SIMIMPORT_ASSET_MISSING"
    PROJECTION_INVALID = "SIMIMPORT_PROJECTION_INVALID"
    WORLD_ALIGNMENT_INVALID = "SIMIMPORT_WORLD_ALIGNMENT_INVALID"
    QUALIFICATION_FAILED = "SIMIMPORT_QUALIFICATION_FAILED"
    DRAFT_NOT_FOUND = "SIMIMPORT_DRAFT_NOT_FOUND"
    PROMOTION_CONFLICT = "SIMIMPORT_PROMOTION_CONFLICT"
    PROMOTION_INVALID = "SIMIMPORT_PROMOTION_INVALID"


@dataclass(frozen=True)
class ImportDiagnostic:
    """One machine-facing import diagnostic."""

    code: ImportCode | str
    message: str
    context: str | None = None
    details: Mapping[str, Any] = field(default_factory=dict)
    severity: str = "error"

    def to_dict(self) -> dict[str, Any]:
        """Return a stable JSON-compatible diagnostic."""

        result: dict[str, Any] = {
            "code": self.code.value if isinstance(self.code, ImportCode) else str(self.code),
            "severity": self.severity,
            "message": self.message,
        }
        if self.context is not None:
            result["context"] = self.context
        if self.details:
            result["details"] = dict(self.details)
        return result


class ImportFailure(ValueError):
    """Fail-closed importer error carrying one stable diagnostic."""

    def __init__(
        self,
        message: str,
        *,
        code: ImportCode = ImportCode.INVALID_REQUEST,
        context: str | None = None,
        details: Mapping[str, Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.context = context
        self.details = dict(details or {})

    def to_diagnostic(self) -> ImportDiagnostic:
        """Project the exception into its API diagnostic."""

        return ImportDiagnostic(
            code=self.code,
            message=str(self),
            context=self.context,
            details=self.details,
        )


@dataclass(frozen=True)
class SourceFile:
    """One content-identified regular source file."""

    path: str
    size: int
    sha256: str

    def to_dict(self) -> dict[str, Any]:
        """Return the content identity of this file."""

        return {"path": self.path, "size": self.size, "sha256": self.sha256}


@dataclass(frozen=True)
class SourceIntakeResult:
    """Materialized source tree and its deterministic identity."""

    root: Path
    source_kind: str
    source_sha256: str
    files: tuple[SourceFile, ...]

    @property
    def content_digest(self) -> str:
        """Digest the complete ordered file set."""

        return digest_document([item.to_dict() for item in self.files])

    def to_dict(self) -> dict[str, Any]:
        """Return the source intake artifact without host-specific source paths."""

        return {
            "schema": "lingtu.sim.source-intake.v1",
            "source_kind": self.source_kind,
            "source_sha256": self.source_sha256,
            "content_digest": self.content_digest,
            "files": [item.to_dict() for item in self.files],
        }


@dataclass(frozen=True)
class ImportDraft:
    """A validated draft that is still outside the active package catalog."""

    import_id: str
    kind: str
    package_id: str
    version: str
    state: str
    root: Path
    package_root: Path | None = None
    manifest_path: Path | None = None
    provenance_path: Path | None = None
    qualification_path: Path | None = None
    diagnostics: tuple[ImportDiagnostic, ...] = ()

    @property
    def reference(self) -> str:
        """Return the exact package reference."""

        return f"{self.package_id}@{self.version}"

    def to_dict(self) -> dict[str, Any]:
        """Return the draft state for service and UI consumers."""

        result: dict[str, Any] = {
            "schema": "lingtu.sim.import-draft.v1",
            "import_id": self.import_id,
            "package": {
                "kind": self.kind,
                "id": self.package_id,
                "version": self.version,
                "ref": self.reference,
            },
            "state": self.state,
            "root": str(self.root),
            "diagnostics": [item.to_dict() for item in self.diagnostics],
        }
        for key, value in (
            ("package_root", self.package_root),
            ("manifest_path", self.manifest_path),
            ("provenance_path", self.provenance_path),
            ("qualification_path", self.qualification_path),
        ):
            result[key] = str(value) if value is not None else None
        return result


_IDENTITY = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_VERSION = re.compile(r"^[A-Za-z0-9][A-Za-z0-9+_.-]*$")
_REPARSE_POINT = 0x0400


def canonical_json_bytes(value: Any) -> bytes:
    """Return deterministic pretty JSON bytes used for authored artifacts."""

    return (
        json.dumps(
            value,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            separators=(",", ": "),
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")


def canonical_json_compact(value: Any) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def digest_document(value: Any) -> str:
    """Hash canonical compact JSON content."""

    return hashlib.sha256(canonical_json_compact(value)).hexdigest()


def sha256_file(path: Path) -> str:
    """Hash a regular file or raise a structured source diagnostic."""

    digest = hashlib.sha256()
    try:
        with Path(path).open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise ImportFailure(
            f"cannot read source file {path}: {exc}",
            code=ImportCode.SOURCE_MISSING,
            context=str(path),
        ) from exc
    return digest.hexdigest()


def require_mapping(value: Any, context: str) -> dict[str, Any]:
    """Require an object-shaped contract field."""

    if not isinstance(value, Mapping):
        raise ImportFailure(f"{context} must be an object", context=context)
    return dict(value)


def strict_keys(
    value: Mapping[str, Any],
    *,
    required: set[str],
    optional: set[str],
    context: str,
) -> None:
    """Reject missing and unknown contract fields."""

    missing = sorted(required - set(value))
    unknown = sorted(set(value) - required - optional)
    if missing or unknown:
        raise ImportFailure(
            f"{context} has invalid fields",
            context=context,
            details={"missing": missing, "unknown": unknown},
        )


def require_string(value: Any, context: str) -> str:
    """Require a non-empty, already-trimmed string."""

    if not isinstance(value, str) or not value or value != value.strip():
        raise ImportFailure(f"{context} must be a non-empty trimmed string", context=context)
    return value


def require_identity(value: Any, context: str, *, version: bool = False) -> str:
    """Require a path-safe package identity or exact version token."""

    result = require_string(value, context)
    pattern = _VERSION if version else _IDENTITY
    if pattern.fullmatch(result) is None:
        raise ImportFailure(f"{context} is not a safe package identity", context=context)
    return result


def safe_relative_path(value: Any, context: str) -> str:
    """Require a relative POSIX path without traversal components."""

    result = require_string(value, context)
    path = PurePosixPath(result)
    if (
        "\\" in result
        or path.is_absolute()
        or ":" in result
        or any(part in {"", ".", ".."} for part in path.parts)
    ):
        raise ImportFailure(
            f"{context} must be a safe relative POSIX path",
            code=ImportCode.UNSAFE_SOURCE,
            context=context,
        )
    return result


def assert_no_reparse_components(
    path: Path,
    *,
    below: Path | None = None,
    context: str | None = None,
) -> Path:
    """Reject symlinks and Windows reparse points in one lexical source path."""

    candidate = Path(os.path.abspath(os.fspath(path)))
    if below is not None:
        base = Path(os.path.abspath(os.fspath(below)))
        try:
            relative_parts = candidate.relative_to(base).parts
        except ValueError as exc:
            raise ImportFailure(
                "source path is outside its owned root",
                code=ImportCode.UNSAFE_SOURCE,
                context=context or str(candidate),
            ) from exc
        current = base
        paths = (current, *(current / Path(*relative_parts[:index]) for index in range(1, len(relative_parts) + 1)))
    else:
        current = Path(candidate.anchor)
        relative_parts = tuple(part for part in candidate.parts if part != candidate.anchor)
        paths = tuple(current / Path(*relative_parts[:index]) for index in range(1, len(relative_parts) + 1))

    for current in paths:
        try:
            metadata = os.lstat(current)
        except FileNotFoundError:
            continue
        except OSError as exc:
            raise ImportFailure(
                f"cannot inspect source path component {current}: {exc}",
                code=ImportCode.UNSAFE_SOURCE,
                context=context or str(current),
            ) from exc
        if stat.S_ISLNK(metadata.st_mode) or bool(
            getattr(metadata, "st_file_attributes", 0) & _REPARSE_POINT
        ):
            raise ImportFailure(
                f"source path contains a symbolic link or Windows reparse point: {current}",
                code=ImportCode.UNSAFE_SOURCE,
                context=context or str(current),
            )
    return candidate


def resolve_beneath(root: Path, relative: str, context: str) -> Path:
    """Resolve a relative path while enforcing one owned root."""

    root = Path(root).resolve()
    target = (root / safe_relative_path(relative, context)).resolve()
    try:
        target.relative_to(root)
    except ValueError as exc:
        raise ImportFailure(
            f"{context} escapes its owned root",
            code=ImportCode.UNSAFE_SOURCE,
            context=context,
        ) from exc
    return target


def file_records(root: Path) -> tuple[SourceFile, ...]:
    """Hash a regular, non-symlink tree in deterministic path order."""

    source_root = assert_no_reparse_components(Path(root), context=str(root))
    resolved_root = source_root.resolve()
    assert_no_reparse_components(resolved_root, context=str(resolved_root))
    if not resolved_root.is_dir():
        raise ImportFailure(
            f"source directory does not exist: {resolved_root}",
            code=ImportCode.SOURCE_MISSING,
            context=str(resolved_root),
        )
    records: list[SourceFile] = []
    for path in sorted(resolved_root.rglob("*"), key=lambda item: item.relative_to(resolved_root).as_posix()):
        assert_no_reparse_components(path, below=resolved_root, context=str(path))
        if path.is_dir():
            continue
        if not path.is_file():
            raise ImportFailure(
                f"source tree contains a non-regular file: {path}",
                code=ImportCode.UNSAFE_SOURCE,
                context=str(path),
            )
        relative = path.relative_to(resolved_root).as_posix()
        # Reject Windows drive/alternate-data-stream syntax even when the
        # source is being walked on a POSIX host.  The package contract is
        # portable and must describe only ordinary relative files.
        safe_relative_path(relative, str(path))
        records.append(SourceFile(relative, path.stat().st_size, sha256_file(path)))
    if not records:
        raise ImportFailure(
            "source tree contains no regular files",
            code=ImportCode.SOURCE_MISSING,
            context=str(resolved_root),
        )
    return tuple(records)


def validate_provenance(
    value: Any,
    *,
    source_root: Path,
    context: str = "provenance",
) -> dict[str, Any]:
    """Require explicit source ownership and a non-empty license file."""

    provenance = require_mapping(value, context)
    strict_keys(
        provenance,
        required={"owner", "license", "license_file", "source_uri"},
        optional={"third_party_assets"},
        context=context,
    )
    license_path_text = safe_relative_path(provenance["license_file"], f"{context}.license_file")
    license_path = resolve_beneath(source_root, license_path_text, f"{context}.license_file")
    if not license_path.is_file() or license_path.stat().st_size == 0:
        raise ImportFailure(
            f"{context}.license_file must identify a non-empty source file",
            code=ImportCode.LICENSE_REQUIRED,
            context=str(license_path),
        )
    third_party = provenance.get("third_party_assets", [])
    if not isinstance(third_party, Sequence) or isinstance(third_party, (str, bytes)):
        raise ImportFailure(
            f"{context}.third_party_assets must be an array",
            code=ImportCode.PROVENANCE_REQUIRED,
            context=context,
        )
    normalized_third_party: list[dict[str, str]] = []
    for index, item in enumerate(third_party):
        item_context = f"{context}.third_party_assets[{index}]"
        asset = require_mapping(item, item_context)
        strict_keys(
            asset,
            required={"name", "license", "source_uri"},
            optional=set(),
            context=item_context,
        )
        normalized_third_party.append(
            {
                "name": require_string(asset["name"], f"{item_context}.name"),
                "license": require_string(asset["license"], f"{item_context}.license"),
                "source_uri": require_string(asset["source_uri"], f"{item_context}.source_uri"),
            }
        )
    return {
        "owner": require_string(provenance["owner"], f"{context}.owner"),
        "license": require_string(provenance["license"], f"{context}.license"),
        "license_file": license_path_text,
        "source_uri": require_string(provenance["source_uri"], f"{context}.source_uri"),
        "third_party_assets": normalized_third_party,
    }


def write_json(path: Path, value: Any) -> Path:
    """Write a canonical deterministic JSON artifact."""

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(canonical_json_bytes(value))
    return path


__all__ = [
    "ImportCode",
    "ImportDiagnostic",
    "ImportDraft",
    "ImportFailure",
    "SourceFile",
    "SourceIntakeResult",
    "assert_no_reparse_components",
    "canonical_json_bytes",
    "digest_document",
    "file_records",
    "require_identity",
    "require_mapping",
    "require_string",
    "resolve_beneath",
    "safe_relative_path",
    "sha256_file",
    "strict_keys",
    "validate_provenance",
    "write_json",
]
