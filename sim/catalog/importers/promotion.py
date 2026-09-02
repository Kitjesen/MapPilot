"""Explicit, transactional promotion of qualified drafts into SimCatalog."""

from __future__ import annotations

import ctypes
import errno
import filecmp
import json
import os
import re
import shutil
import threading
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, cast

from sim.catalog.resolver import CatalogError, CatalogResolver

from .contracts import (
    ImportCode,
    ImportDraft,
    ImportFailure,
    canonical_json_bytes,
    require_string,
    resolve_beneath,
    safe_relative_path,
)

_KIND_DIRECTORIES = {"robot": "robots", "world": "worlds"}
_IDENTITY = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_VERSION = re.compile(r"^[A-Za-z0-9][A-Za-z0-9+_.-]*$")
_QUALIFICATION_SCHEMA = "lingtu.sim.qualification-record.v1"
_CHECK_STATUSES = {"passed", "failed", "skipped"}

_PROCESS_LOCKS: dict[Path, threading.Lock] = {}
_PROCESS_LOCKS_GUARD = threading.Lock()


def _lexists(path: Path) -> bool:
    """Return whether a path exists, including a dangling symbolic link."""

    return os.path.lexists(str(path))


def _capability_is_subset(qualified: Any, declared: Any) -> bool:
    """Check that qualified capabilities do not exceed the package declaration."""

    if isinstance(qualified, Mapping):
        if not isinstance(declared, Mapping):
            return False
        return all(key in declared and _capability_is_subset(value, declared[key]) for key, value in qualified.items())
    if isinstance(qualified, list):
        if not isinstance(declared, list):
            return False
        declared_values = {
            json.dumps(item, ensure_ascii=False, sort_keys=True, separators=(",", ":")) for item in declared
        }
        return all(
            json.dumps(item, ensure_ascii=False, sort_keys=True, separators=(",", ":")) in declared_values
            for item in qualified
        )
    return cast(bool, qualified == declared)


def _file_paths(root: Path) -> tuple[str, ...]:
    return tuple(sorted(path.relative_to(root).as_posix() for path in root.rglob("*") if path.is_file()))


def _same_tree(left: Path, right: Path) -> bool:
    paths = _file_paths(left)
    return paths == _file_paths(right) and all(
        filecmp.cmp(left / relative, right / relative, shallow=False) for relative in paths
    )


def _process_lock_for(path: Path) -> threading.Lock:
    """Return the process-local lock paired with one cross-process lock file."""

    key = path.resolve()
    with _PROCESS_LOCKS_GUARD:
        lock = _PROCESS_LOCKS.get(key)
        if lock is None:
            lock = threading.Lock()
            _PROCESS_LOCKS[key] = lock
        return lock


class _PackageLock:
    """Small cross-platform advisory lock for one package identity."""

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self._handle: Any = None
        self._thread_lock: threading.Lock | None = None

    def __enter__(self) -> _PackageLock:
        if self.path.is_symlink():
            raise ImportFailure(
                "promotion lock path must not be a symbolic link",
                code=ImportCode.PROMOTION_INVALID,
                context=str(self.path),
            )
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._thread_lock = _process_lock_for(self.path)
        self._thread_lock.acquire()
        try:
            self._handle = self.path.open("a+b")
            self._handle.seek(0, os.SEEK_END)
            if self._handle.tell() == 0:
                self._handle.write(b"\0")
                self._handle.flush()
            self._handle.seek(0)
            if os.name == "nt":
                import msvcrt

                msvcrt.locking(self._handle.fileno(), msvcrt.LK_LOCK, 1)
            else:
                import fcntl

                fcntl.flock(self._handle.fileno(), fcntl.LOCK_EX)  # type: ignore[attr-defined]
        except Exception:
            if self._handle is not None:
                self._handle.close()
                self._handle = None
            self._thread_lock.release()
            self._thread_lock = None
            raise
        return self

    def __exit__(self, exc_type: Any, exc_value: Any, traceback: Any) -> None:
        if self._handle is not None:
            try:
                if os.name == "nt":
                    import msvcrt

                    self._handle.seek(0)
                    msvcrt.locking(self._handle.fileno(), msvcrt.LK_UNLCK, 1)
                else:
                    import fcntl

                    fcntl.flock(self._handle.fileno(), fcntl.LOCK_UN)  # type: ignore[attr-defined]
            finally:
                self._handle.close()
                self._handle = None
        if self._thread_lock is not None:
            self._thread_lock.release()
            self._thread_lock = None


@dataclass(frozen=True)
class PromotionResult:
    """One new immutable Catalog package and its qualification record."""

    kind: str
    package_id: str
    version: str
    package_root: Path
    qualification_path: Path

    def to_dict(self) -> dict[str, Any]:
        """Return the promoted package identity and immutable locations."""

        return {
            "schema": "lingtu.sim.promotion-result.v1",
            "package": {
                "kind": self.kind,
                "id": self.package_id,
                "version": self.version,
                "ref": f"{self.package_id}@{self.version}",
            },
            "package_root": str(self.package_root),
            "qualification_path": str(self.qualification_path),
        }


@dataclass(frozen=True)
class _PromotionLayout:
    """Canonical destinations for one package identity."""

    package_target: Path
    qualification_target: Path
    evidence_target: Path


@dataclass(frozen=True)
class _ValidatedArtifacts:
    """Content identity calculated before any catalog path is published."""

    package_root: Path
    qualification_bytes: bytes
    evidence_root: Path
    provenance_path: str


class CatalogPromoter:
    """Promote a qualified draft without mutating an active version in place."""

    def __init__(self, repo_root: Path) -> None:
        self.repo_root = Path(repo_root).resolve()
        self.qualification_root = self.repo_root / "sim" / "evaluation" / "package_qualifications"
        self._staging_root = self.repo_root / "sim" / ".promotion-staging"
        self._lock_root = self.repo_root / "sim" / ".promotion-locks"

    def promote(self, draft: ImportDraft) -> PromotionResult:
        """Publish one fully validated draft under a per-package lock."""

        kind, package_id, version = self._identity(draft)
        lock_path = self._owned_path(
            self.repo_root,
            "sim",
            ".promotion-locks",
            kind,
            package_id,
            f"{version}.lock",
            context="promotion lock",
        )
        try:
            with _PackageLock(lock_path):
                return self._promote_locked(draft, kind, package_id, version)
        except ImportFailure:
            raise
        except Exception as exc:
            raise ImportFailure(
                f"cannot promote package {package_id}@{version}: {exc}",
                code=ImportCode.PROMOTION_INVALID,
                context=f"{package_id}@{version}",
            ) from exc

    def _promote_locked(self, draft: ImportDraft, kind: str, package_id: str, version: str) -> PromotionResult:
        if draft.state != "qualified":
            raise ImportFailure(
                f"draft {draft.import_id} is not qualified",
                code=ImportCode.PROMOTION_INVALID,
                context=draft.import_id,
                details={"state": draft.state},
            )
        package_root, manifest_path, provenance_path, qualification_path, evidence_source = self._draft_paths(draft)
        layout = self._layout(kind, package_id, version)
        token = uuid.uuid4().hex
        stage_root = self._owned_path(
            self.repo_root,
            "sim",
            ".promotion-staging",
            token,
            context="promotion staging",
        )
        stage_root.mkdir(parents=True, exist_ok=False)
        result_path = draft.root / "promotion-result.json"
        package_committed = False
        committed_details = {
            "committed": True,
            "package": {
                "kind": kind,
                "id": package_id,
                "version": version,
                "ref": f"{package_id}@{version}",
            },
        }
        try:
            package_stage = stage_root / "p"
            qualification_stage = stage_root / "q"
            evidence_stage = stage_root / "evidence" / version
            self._copy_tree(package_root, package_stage)
            self._copy_tree(evidence_source, evidence_stage)
            self._copy_file(qualification_path, qualification_stage)

            manifest_relative = manifest_path.relative_to(package_root).as_posix()
            manifest_stage = package_stage / manifest_relative
            provenance_relative = provenance_path.relative_to(package_root).as_posix()
            validation = self._validate_materialized(
                kind=kind,
                package_id=package_id,
                version=version,
                package_root=package_stage,
                manifest_path=manifest_stage,
                provenance_path=package_stage / provenance_relative,
                qualification_path=qualification_stage,
                evidence_root=evidence_stage,
            )

            if self._existing_is_identical(layout, validation, kind, package_id, version):
                result = self._result(kind, package_id, version, layout)
                package_committed = True
                self._write_promotion_result(result_path, result)
                return result

            self._ensure_target_parents(layout)
            self._publish_or_reuse_prerequisite(
                "evidence",
                evidence_stage,
                layout.evidence_target,
            )
            result = self._result(kind, package_id, version, layout)
            self._publish_or_reuse_prerequisite(
                "qualification",
                qualification_stage,
                layout.qualification_target,
            )
            self._before_package_commit(layout, validation)
            self._publish_step("package", package_stage, layout.package_target)
            package_committed = True
            self._write_promotion_result(result_path, result)
            return result
        except ImportFailure as exc:
            if not package_committed:
                raise
            raise ImportFailure(
                str(exc),
                code=exc.code,
                context=exc.context or draft.reference,
                details={**exc.details, **committed_details},
            ) from exc
        except Exception as exc:
            raise ImportFailure(
                f"cannot promote package {draft.reference}: {exc}",
                code=ImportCode.PROMOTION_INVALID,
                context=draft.reference,
                details=committed_details if package_committed else None,
            ) from exc
        finally:
            self._cleanup_paths([stage_root])

    def _draft_paths(self, draft: ImportDraft) -> tuple[Path, Path, Path, Path, Path]:
        """Resolve draft artifacts and enforce importer-owned containment."""

        try:
            root = Path(draft.root).resolve()
        except (TypeError, ValueError) as exc:
            raise ImportFailure(
                "draft root is invalid",
                code=ImportCode.DRAFT_NOT_FOUND,
                context=str(getattr(draft, "import_id", "unknown")),
            ) from exc
        if Path(draft.root).is_symlink() or not root.is_dir():
            raise ImportFailure(
                "draft root is missing or is a symbolic link",
                code=ImportCode.DRAFT_NOT_FOUND,
                context=str(root),
            )
        values = {
            "package_root": draft.package_root,
            "manifest_path": draft.manifest_path,
            "provenance_path": draft.provenance_path,
            "qualification_path": draft.qualification_path,
        }
        resolved: dict[str, Path] = {}
        for label, value in values.items():
            if value is None:
                raise ImportFailure(
                    f"qualified draft is missing {label}",
                    code=ImportCode.DRAFT_NOT_FOUND,
                    context=draft.import_id,
                )
            path = self._input_path(root, Path(value), label)
            if path.is_symlink():
                raise ImportFailure(
                    f"draft {label} must not be a symbolic link",
                    code=ImportCode.PROMOTION_INVALID,
                    context=str(path),
                )
            resolved[label] = path

        package_root = resolved["package_root"]
        manifest_path = resolved["manifest_path"]
        provenance_path = resolved["provenance_path"]
        qualification_path = resolved["qualification_path"]
        if (
            not package_root.is_dir()
            or not manifest_path.is_file()
            or not provenance_path.is_file()
            or not qualification_path.is_file()
        ):
            raise ImportFailure(
                "qualified draft is missing a required artifact",
                code=ImportCode.DRAFT_NOT_FOUND,
                context=draft.import_id,
            )
        try:
            provenance_path.relative_to(package_root)
        except ValueError as exc:
            raise ImportFailure(
                "package provenance must be inside the staged package",
                code=ImportCode.PROMOTION_INVALID,
                context=str(provenance_path),
            ) from exc
        try:
            manifest_path.relative_to(package_root)
        except ValueError as exc:
            raise ImportFailure(
                "package manifest must be inside the staged package",
                code=ImportCode.PROMOTION_INVALID,
                context=str(manifest_path),
            ) from exc

        expected_qualification_parent = qualification_path.parent
        expected_evidence = expected_qualification_parent / "evidence" / self._version_from_path(qualification_path)
        if not expected_evidence.is_dir():
            raise ImportFailure(
                "draft qualification evidence directory is missing",
                code=ImportCode.QUALIFICATION_FAILED,
                context=str(expected_evidence),
            )
        return package_root, manifest_path, provenance_path, qualification_path, expected_evidence

    def _validate_materialized(
        self,
        *,
        kind: str,
        package_id: str,
        version: str,
        package_root: Path,
        manifest_path: Path,
        provenance_path: Path,
        qualification_path: Path,
        evidence_root: Path,
    ) -> _ValidatedArtifacts:
        manifest_candidate = self._single_manifest(package_root)
        if manifest_candidate != manifest_path.resolve():
            raise ImportFailure(
                "staged package manifest identity is ambiguous or stale",
                code=ImportCode.PROMOTION_INVALID,
                context=f"{package_id}@{version}",
                details={"expected": str(manifest_path), "actual": str(manifest_candidate)},
        )
        try:
            # Imported package artifacts bind paths relative to the package root.
            # Using that same root also confines resolver reads to the staged tree.
            resolver = CatalogResolver(package_root, (package_root,))
            record = resolver.find_package(f"{package_id}@{version}", kind=kind)
        except CatalogError as exc:
            raise ImportFailure(
                f"staged package does not validate: {exc}",
                code=ImportCode.PROMOTION_INVALID,
                context=f"{package_id}@{version}",
            ) from exc
        if record.manifest_path != manifest_path.resolve():
            raise ImportFailure(
                "staged manifest is not the resolved package manifest",
                code=ImportCode.PROMOTION_INVALID,
                context=f"{package_id}@{version}",
            )
        self._validate_provenance_file(package_root, provenance_path)
        report_provenance_path = self._validate_qualification(
            package_root=package_root,
            provenance_path=provenance_path,
            qualification_path=qualification_path,
            evidence_root=evidence_root,
            record=record.data,
            kind=kind,
            package_id=package_id,
            version=version,
        )
        if report_provenance_path != provenance_path.relative_to(package_root).as_posix():
            raise ImportFailure(
                "qualification provenance path does not match the package provenance artifact",
                code=ImportCode.QUALIFICATION_FAILED,
                context=f"{package_id}@{version}",
            )
        return _ValidatedArtifacts(
            package_root=package_root,
            qualification_bytes=qualification_path.read_bytes(),
            evidence_root=evidence_root,
            provenance_path=report_provenance_path,
        )

    def _validate_provenance_file(self, package_root: Path, provenance_path: Path) -> None:
        """Validate one package-owned provenance JSON document."""

        try:
            provenance_path.resolve().relative_to(package_root.resolve())
        except ValueError as exc:
            raise ImportFailure(
                "package provenance escapes the package root",
                code=ImportCode.PROMOTION_INVALID,
                context=str(provenance_path),
            ) from exc
        if not provenance_path.is_file() or provenance_path.is_symlink():
            raise ImportFailure(
                "package provenance is missing or is not a regular file",
                code=ImportCode.PROMOTION_INVALID,
                context=str(provenance_path),
            )
        try:
            value = json.loads(provenance_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise ImportFailure(
                f"cannot read package provenance: {exc}",
                code=ImportCode.PROMOTION_INVALID,
                context=str(provenance_path),
            ) from exc
        if not isinstance(value, Mapping) or not value:
            raise ImportFailure(
                "package provenance must be a non-empty JSON object",
                code=ImportCode.PROMOTION_INVALID,
                context=str(provenance_path),
            )
        nested = value.get("provenance")
        if nested is not None and not isinstance(nested, Mapping):
            raise ImportFailure(
                "package provenance.provenance must be an object",
                code=ImportCode.PROMOTION_INVALID,
                context=str(provenance_path),
            )
        if nested is None and not {"owner", "license", "source_uri"}.issubset(value):
            raise ImportFailure(
                "package provenance must identify owner, license, and source_uri",
                code=ImportCode.PROMOTION_INVALID,
                context=str(provenance_path),
            )
    def _validate_qualification(
        self,
        *,
        package_root: Path,
        provenance_path: Path,
        qualification_path: Path,
        evidence_root: Path,
        record: Mapping[str, Any],
        kind: str,
        package_id: str,
        version: str,
    ) -> str:
        """Validate the qualification identity, provenance path, and evidence paths."""

        context = str(qualification_path)
        try:
            report = json.loads(qualification_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise ImportFailure(
                f"cannot read qualification record: {exc}",
                code=ImportCode.QUALIFICATION_FAILED,
                context=context,
            ) from exc
        if not isinstance(report, Mapping):
            raise ImportFailure(
                "qualification record must be a JSON object",
                code=ImportCode.QUALIFICATION_FAILED,
                context=context,
            )
        self._strict_keys(report, {"schema", "package", "qualified_capabilities", "provenance", "checks"}, context)
        if report.get("schema") != _QUALIFICATION_SCHEMA:
            raise ImportFailure(
                f"qualification schema must be {_QUALIFICATION_SCHEMA}",
                code=ImportCode.QUALIFICATION_FAILED,
                context=context,
            )
        package = report.get("package")
        if not isinstance(package, Mapping):
            raise ImportFailure(
                "qualification package must be an object",
                code=ImportCode.QUALIFICATION_FAILED,
                context=context,
            )
        self._strict_keys(
            package,
            {"kind", "id", "version"},
            context=f"{context}.package",
        )
        expected_identity = {"kind": kind, "id": package_id, "version": version}
        if {key: package.get(key) for key in expected_identity} != expected_identity:
            raise ImportFailure(
                "qualification package identity does not match the draft",
                code=ImportCode.QUALIFICATION_FAILED,
                context=context,
            )
        raw_provenance = report.get("provenance")
        if not isinstance(raw_provenance, Mapping):
            raise ImportFailure(
                "qualification provenance must be an object",
                code=ImportCode.QUALIFICATION_FAILED,
                context=context,
            )
        self._strict_keys(raw_provenance, {"path"}, context=f"{context}.provenance")
        report_provenance_path = safe_relative_path(raw_provenance.get("path"), f"{context}.provenance.path")
        try:
            resolved_provenance = resolve_beneath(package_root, report_provenance_path, f"{context}.provenance.path")
            resolved_provenance.relative_to(provenance_path.resolve().parent)
        except (ImportFailure, ValueError) as exc:
            raise ImportFailure(
                "qualification provenance path is not package-internal",
                code=ImportCode.QUALIFICATION_FAILED,
                context=context,
            ) from exc
        if resolved_provenance != provenance_path.resolve():
            raise ImportFailure(
                "qualification provenance does not bind the staged provenance file",
                code=ImportCode.QUALIFICATION_FAILED,
                context=context,
            )

        qualified_capabilities = report.get("qualified_capabilities")
        if not isinstance(qualified_capabilities, Mapping):
            raise ImportFailure(
                "qualification qualified_capabilities must be an object",
                code=ImportCode.QUALIFICATION_FAILED,
                context=context,
            )
        for capability, values in qualified_capabilities.items():
            if not isinstance(capability, str) or not capability:
                raise ImportFailure(
                    "qualification capability names must be non-empty strings",
                    code=ImportCode.QUALIFICATION_FAILED,
                    context=context,
                )
            if not isinstance(values, list) or any(not isinstance(item, str) or not item for item in values):
                raise ImportFailure(
                    f"qualification capability {capability!r} must be an array of strings",
                    code=ImportCode.QUALIFICATION_FAILED,
                    context=context,
                )
            if len(values) != len(set(values)):
                raise ImportFailure(
                    f"qualification capability {capability!r} contains duplicates",
                    code=ImportCode.QUALIFICATION_FAILED,
                    context=context,
                )
        if not _capability_is_subset(qualified_capabilities, record.get("declared_capabilities", {})):
            raise ImportFailure(
                "qualification capabilities exceed the package declaration",
                code=ImportCode.QUALIFICATION_FAILED,
                context=context,
            )

        raw_checks = report.get("checks")
        if not isinstance(raw_checks, list) or not raw_checks:
            raise ImportFailure(
                "qualification checks must be a non-empty array",
                code=ImportCode.QUALIFICATION_FAILED,
                context=context,
            )
        evidence_paths = set(_file_paths(evidence_root))
        referenced: set[str] = set()
        check_ids: set[str] = set()
        for index, raw_check in enumerate(raw_checks):
            check_context = f"{context}.checks[{index}]"
            if not isinstance(raw_check, Mapping):
                raise ImportFailure(
                    "qualification check must be an object",
                    code=ImportCode.QUALIFICATION_FAILED,
                    context=check_context,
                )
            self._strict_keys(raw_check, {"id", "status"}, check_context, optional={"evidence"})
            check_id = require_string(raw_check.get("id"), f"{check_context}.id")
            status = require_string(raw_check.get("status"), f"{check_context}.status")
            if check_id in check_ids or status not in _CHECK_STATUSES:
                raise ImportFailure(
                    "qualification checks contain a duplicate id or unsupported status",
                    code=ImportCode.QUALIFICATION_FAILED,
                    context=check_context,
                )
            check_ids.add(check_id)
            if status != "passed":
                raise ImportFailure(
                    "a promoted qualification must contain only passed checks",
                    code=ImportCode.QUALIFICATION_FAILED,
                    context=check_context,
                )
            raw_evidence = raw_check.get("evidence")
            if not isinstance(raw_evidence, list) or not raw_evidence:
                raise ImportFailure(
                    "passed qualification checks require evidence",
                    code=ImportCode.QUALIFICATION_FAILED,
                    context=check_context,
                )
            for evidence_index, raw_item in enumerate(raw_evidence):
                evidence_context = f"{check_context}.evidence[{evidence_index}]"
                if not isinstance(raw_item, Mapping):
                    raise ImportFailure(
                        "qualification evidence must be an object",
                        code=ImportCode.QUALIFICATION_FAILED,
                        context=evidence_context,
                    )
                self._strict_keys(raw_item, {"path"}, evidence_context)
                relative = safe_relative_path(raw_item.get("path"), f"{evidence_context}.path")
                try:
                    evidence_path = resolve_beneath(qualification_path.parent, relative, f"{evidence_context}.path")
                    evidence_relative = evidence_path.relative_to(evidence_root.resolve()).as_posix()
                except (ImportFailure, ValueError) as exc:
                    raise ImportFailure(
                        "qualification evidence path escapes the version-owned evidence directory",
                        code=ImportCode.QUALIFICATION_FAILED,
                        context=evidence_context,
                    ) from exc
                if evidence_relative in referenced or evidence_relative not in evidence_paths:
                    raise ImportFailure(
                        "qualification evidence is duplicated or missing",
                        code=ImportCode.QUALIFICATION_FAILED,
                        context=evidence_context,
                    )
                referenced.add(evidence_relative)
        if referenced != evidence_paths:
            raise ImportFailure(
                "qualification evidence contains unbound files",
                code=ImportCode.QUALIFICATION_FAILED,
                context=context,
            )
        return report_provenance_path

    def _existing_is_identical(
        self,
        layout: _PromotionLayout,
        incoming: _ValidatedArtifacts,
        kind: str,
        package_id: str,
        version: str,
    ) -> bool:
        """Return true only for a complete, byte-identical prior promotion."""

        paths = (layout.package_target, layout.qualification_target, layout.evidence_target)
        present = [_lexists(path) for path in paths]
        if not present[0]:
            return False
        if not all(present):
            raise ImportFailure(
                "package identity has an incomplete existing Catalog publication",
                code=ImportCode.PROMOTION_CONFLICT,
                context=f"{package_id}@{version}",
                details={"paths": [str(path) for path, exists in zip(paths, present) if exists]},
            )
        if (
            layout.package_target.is_symlink()
            or layout.qualification_target.is_symlink()
            or layout.evidence_target.is_symlink()
        ):
            raise ImportFailure(
                "existing Catalog publication contains a symbolic link",
                code=ImportCode.PROMOTION_CONFLICT,
                context=f"{package_id}@{version}",
            )
        try:
            existing_manifest = self._single_manifest(layout.package_target)
            existing = self._validate_materialized(
                kind=kind,
                package_id=package_id,
                version=version,
                package_root=layout.package_target,
                manifest_path=existing_manifest,
                provenance_path=layout.package_target / incoming.provenance_path,
                qualification_path=layout.qualification_target,
                evidence_root=layout.evidence_target,
            )
        except (ImportFailure, OSError, CatalogError, ValueError) as exc:
            raise ImportFailure(
                "existing Catalog publication is invalid and cannot be overwritten",
                code=ImportCode.PROMOTION_CONFLICT,
                context=f"{package_id}@{version}",
                details={"reason": str(exc)},
            ) from exc
        if (
            _same_tree(existing.package_root, incoming.package_root)
            and existing.qualification_bytes == incoming.qualification_bytes
            and _same_tree(existing.evidence_root, incoming.evidence_root)
        ):
            return True
        raise ImportFailure(
            "a different package already owns this Catalog identity",
            code=ImportCode.PROMOTION_CONFLICT,
            context=f"{package_id}@{version}",
            details={
                "package_path": str(layout.package_target),
                "qualification_path": str(layout.qualification_target),
            },
        )

    def _layout(self, kind: str, package_id: str, version: str) -> _PromotionLayout:
        if kind == "robot":
            package_target = self._owned_path(
                self.repo_root,
                "sim",
                "packages",
                "robots",
                package_id,
                context="package target",
            )
        else:
            package_parent = self._owned_path(
                self.repo_root,
                "sim",
                "packages",
                _KIND_DIRECTORIES[kind],
                package_id,
                context="package target",
            )
            package_target = self._owned_path(
                package_parent,
                version,
                context="package target version",
            )
        qualification_parent = self._owned_path(
            self.qualification_root,
            kind,
            package_id,
            context="qualification target",
        )
        return _PromotionLayout(
            package_target=package_target,
            qualification_target=self._owned_path(
                qualification_parent,
                f"{version}.qualification.json",
                context="qualification target version",
            ),
            evidence_target=self._owned_path(
                qualification_parent,
                "evidence",
                version,
                context="qualification evidence target",
            ),
        )

    def _ensure_target_parents(self, layout: _PromotionLayout) -> None:
        """Create only the exact parent directories needed by the publication."""

        for path in (layout.package_target.parent, layout.qualification_target.parent, layout.evidence_target.parent):
            path.mkdir(parents=True, exist_ok=True)
            if path.is_symlink():
                raise ImportFailure(
                    "Catalog target parent must not be a symbolic link",
                    code=ImportCode.PROMOTION_INVALID,
                    context=str(path),
                )

    def _publish_step(self, label: str, source: Path, target: Path) -> None:
        """Publish one prepared artifact with an atomic no-replace operation.

        The destination check must be part of the filesystem operation itself.
        A separate ``exists`` check followed by ``Path.replace`` is unsafe:
        another process can create the destination in between and the replace
        would silently clobber it.  Windows ``MoveFileExW`` without
        ``MOVEFILE_REPLACE_EXISTING`` and Linux ``renameat2`` with
        ``RENAME_NOREPLACE`` provide the native atomic operation.  The
        fallback reserves a directory with an exclusive ``mkdir`` and moves
        regular files with exclusive hard links, which keeps the same
        no-clobber guarantee on POSIX systems without ``renameat2``.
        """

        try:
            self._rename_noreplace(source, target)
        except FileExistsError as exc:
            raise ImportFailure(
                f"cannot publish {label}: target appeared during promotion",
                code=ImportCode.PROMOTION_CONFLICT,
                context=str(target),
            ) from exc
        except OSError as exc:
            raise ImportFailure(
                f"cannot publish {label}: {exc}",
                code=ImportCode.PROMOTION_INVALID,
                context=str(target),
            ) from exc

    def _publish_or_reuse_prerequisite(
        self,
        label: str,
        source: Path,
        target: Path,
    ) -> None:
        """Publish an immutable prerequisite or reuse an identical orphan.

        Evidence and qualification are intentionally never rolled back.  They
        are invisible to Catalog discovery until the package manifest commits,
        and retaining them avoids any unlink race with concurrent writers.
        """

        if not _lexists(target):
            try:
                self._publish_step(label, source, target)
                return
            except ImportFailure as exc:
                if exc.code != ImportCode.PROMOTION_CONFLICT or not _lexists(target):
                    raise
        if self._prerequisite_is_identical(source, target):
            return
        raise ImportFailure(
            f"cannot reuse {label}: existing prerequisite has different content",
            code=ImportCode.PROMOTION_CONFLICT,
            context=str(target),
        )

    def _prerequisite_is_identical(
        self,
        source: Path,
        target: Path,
    ) -> bool:
        """Return whether one published prerequisite exactly matches input."""

        try:
            if target.is_symlink():
                return False
            if source.is_dir():
                return target.is_dir() and _same_tree(source, target)
            return source.is_file() and target.is_file() and filecmp.cmp(source, target, shallow=False)
        except (ImportFailure, OSError):
            return False

    def _rename_noreplace(self, source: Path, target: Path) -> None:
        """Atomically move ``source`` to an absent ``target`` without replace."""

        if os.name == "nt":
            self._windows_move_noreplace(source, target)
            return
        if self._posix_rename_noreplace(source, target):
            return
        self._fallback_rename_noreplace(source, target)

    def _windows_move_noreplace(self, source: Path, target: Path) -> None:
        """Use MoveFileExW without the replace-existing flag on Windows."""

        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        move_file_ex = kernel32.MoveFileExW
        move_file_ex.argtypes = [ctypes.c_wchar_p, ctypes.c_wchar_p, ctypes.c_uint32]
        move_file_ex.restype = ctypes.c_bool
        # MOVEFILE_WRITE_THROUGH is safe for the publication marker and does
        # not opt into MOVEFILE_REPLACE_EXISTING (0x1).
        if move_file_ex(str(source), str(target), 0x00000008):
            return
        winerror = ctypes.get_last_error()
        if winerror in {80, 183}:  # ERROR_FILE_EXISTS / ERROR_ALREADY_EXISTS
            raise FileExistsError(errno.EEXIST, "target already exists", str(target))
        raise OSError(winerror, ctypes.FormatError(winerror), str(target))

    def _posix_rename_noreplace(self, source: Path, target: Path) -> bool:
        """Use Linux renameat2 when the host exposes RENAME_NOREPLACE."""

        try:
            libc = ctypes.CDLL(None, use_errno=True)
            renameat2 = libc.renameat2
        except (AttributeError, OSError):
            return False

        renameat2.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
        renameat2.restype = ctypes.c_int
        result = renameat2(
            getattr(os, "AT_FDCWD", -100),
            os.fsencode(source),
            getattr(os, "AT_FDCWD", -100),
            os.fsencode(target),
            0x1,  # RENAME_NOREPLACE
        )
        if result == 0:
            return True
        error_number = ctypes.get_errno()
        if error_number == errno.EEXIST:
            raise FileExistsError(errno.EEXIST, "target already exists", str(target))
        unsupported = {
            getattr(errno, "ENOSYS", 38),
            getattr(errno, "EINVAL", 22),
            getattr(errno, "ENOTSUP", 95),
            getattr(errno, "EOPNOTSUPP", 95),
        }
        if error_number in unsupported:
            return False
        raise OSError(error_number, os.strerror(error_number), str(target))

    def _fallback_rename_noreplace(self, source: Path, target: Path) -> None:
        """Reserve a directory or hard-link a file when no native primitive exists."""

        if source.is_symlink():
            raise OSError(errno.ELOOP, "promotion source must not be a symbolic link", str(source))
        if source.is_dir():
            owned_paths: list[Path] = []
            try:
                # mkdir(..., exist_ok=False) is the reservation.  It is one
                # filesystem operation, so an external creator either wins
                # and receives EEXIST or we own this directory exclusively.
                target.mkdir()
                owned_paths.append(target)
                self._move_directory_noreplace(source, target, owned_paths)
            except Exception:
                self._cleanup_owned_paths(owned_paths)
                raise
            return
        if not source.is_file():
            raise OSError(errno.EINVAL, "promotion source must be a regular file", str(source))
        self._hard_link_noreplace(source, target)

    def _move_directory_noreplace(self, source: Path, target: Path, owned_paths: list[Path]) -> None:
        """Materialize a reserved directory without replacing any child."""

        for child in sorted(source.iterdir(), key=lambda path: path.name):
            target_child = target / child.name
            if child.is_symlink():
                raise OSError(errno.ELOOP, "promotion source must not contain symbolic links", str(child))
            if child.is_dir():
                target_child.mkdir()
                owned_paths.append(target_child)
                self._move_directory_noreplace(child, target_child, owned_paths)
                continue
            if not child.is_file():
                raise OSError(errno.EINVAL, "promotion source must contain regular files only", str(child))
            self._hard_link_noreplace(child, target_child)
            owned_paths.append(target_child)
        source.rmdir()

    def _hard_link_noreplace(self, source: Path, target: Path) -> None:
        """Create a complete file at an absent target and remove the stage copy."""

        try:
            os.link(source, target, follow_symlinks=False)
        except FileExistsError:
            raise
        except OSError as exc:
            raise OSError(
                exc.errno,
                f"cannot create no-replace publication link: {exc}",
                str(target),
            ) from exc
        try:
            source.unlink()
        except OSError:
            # Only remove the target if it is still the hard link we created;
            # do not turn cleanup into another foreign-target clobber.
            try:
                if _lexists(target) and os.path.samefile(source, target):
                    target.unlink()
            except (FileNotFoundError, OSError):
                pass
            raise

    def _cleanup_owned_paths(self, paths: list[Path]) -> None:
        """Roll back only paths created by the fallback publisher."""

        for path in reversed(paths):
            try:
                if path.is_dir() and not path.is_symlink():
                    path.rmdir()
                elif _lexists(path):
                    path.unlink()
            except OSError:
                # A foreign child may have appeared in an owned directory;
                # leaving that directory intact is safer than recursive delete.
                continue

    def _before_package_commit(self, layout: _PromotionLayout, validation: _ValidatedArtifacts) -> None:
        """Test hook immediately before the package becomes Catalog-visible."""

    def _write_promotion_result(self, path: Path, result: PromotionResult) -> None:
        """Atomically publish the post-commit draft-local promotion receipt."""

        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
        try:
            with temporary.open("xb") as handle:
                handle.write(canonical_json_bytes(result.to_dict()))
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)

    def _copy_tree(self, source: Path, target: Path) -> None:
        """Copy a regular tree while detecting source mutation and links."""

        before = _file_paths(source)
        shutil.copytree(source, target, symlinks=False)
        if before != _file_paths(source) or not _same_tree(source, target):
            raise ImportFailure(
                "source tree changed during promotion staging",
                code=ImportCode.PROMOTION_INVALID,
                context=str(source),
            )

    def _copy_file(self, source: Path, target: Path) -> None:
        """Copy one regular file and verify its content identity."""

        source_bytes = source.read_bytes()
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        if source.read_bytes() != source_bytes or target.read_bytes() != source_bytes:
            raise ImportFailure(
                "source file changed during promotion staging",
                code=ImportCode.PROMOTION_INVALID,
                context=str(source),
            )

    def _result(
        self,
        kind: str,
        package_id: str,
        version: str,
        layout: _PromotionLayout,
    ) -> PromotionResult:
        return PromotionResult(
            kind=kind,
            package_id=package_id,
            version=version,
            package_root=layout.package_target,
            qualification_path=layout.qualification_target,
        )

    def _identity(self, draft: ImportDraft) -> tuple[str, str, str]:
        """Validate the identity before deriving any lock or target path."""

        kind = draft.kind
        if kind not in _KIND_DIRECTORIES:
            raise ImportFailure(
                f"promotion does not support package kind {kind!r}",
                code=ImportCode.PROMOTION_INVALID,
                context=str(kind),
            )
        package_id = self._component(draft.package_id, "package id", _IDENTITY)
        version = self._component(draft.version, "package version", _VERSION)
        return kind, package_id, version

    def _component(self, value: Any, label: str, pattern: re.Pattern[str]) -> str:
        if not isinstance(value, str) or pattern.fullmatch(value) is None:
            raise ImportFailure(
                f"{label} is not path-safe",
                code=ImportCode.PROMOTION_INVALID,
                context=str(value),
            )
        return value

    def _input_path(self, root: Path, path: Path, label: str) -> Path:
        resolved = path.resolve()
        try:
            resolved.relative_to(root)
        except ValueError as exc:
            raise ImportFailure(
                f"draft {label} escapes the importer-owned root",
                code=ImportCode.PROMOTION_INVALID,
                context=str(path),
            ) from exc
        return resolved

    def _owned_path(self, root: Path, *parts: str, context: str) -> Path:
        root = Path(root).resolve()
        candidate = root.joinpath(*parts).resolve()
        try:
            candidate.relative_to(root)
        except ValueError as exc:
            raise ImportFailure(
                f"{context} escapes its owned root",
                code=ImportCode.PROMOTION_INVALID,
                context=str(candidate),
            ) from exc
        return candidate

    def _single_manifest(self, package_root: Path) -> Path:
        candidates = sorted(
            path.resolve()
            for suffix in (".package.yaml", ".package.yml")
            for path in package_root.glob(f"*{suffix}")
            if path.is_file() and not path.is_symlink()
        )
        if len(candidates) != 1:
            raise ImportFailure(
                "package root must contain exactly one package manifest",
                code=ImportCode.PROMOTION_INVALID,
                context=str(package_root),
                details={"count": len(candidates)},
            )
        return candidates[0]

    def _version_from_path(self, qualification_path: Path) -> str:
        name = qualification_path.name
        suffix = ".qualification.json"
        if not name.endswith(suffix):
            raise ImportFailure(
                "qualification path has an invalid final marker name",
                code=ImportCode.QUALIFICATION_FAILED,
                context=str(qualification_path),
            )
        return name[: -len(suffix)]

    def _strict_keys(
        self,
        value: Mapping[str, Any],
        required: set[str],
        context: str,
        optional: set[str] | None = None,
    ) -> None:
        optional = optional or set()
        missing = sorted(required - set(value))
        unknown = sorted(set(value) - required - optional)
        if missing or unknown:
            raise ImportFailure(
                f"{context} has invalid fields",
                code=ImportCode.QUALIFICATION_FAILED,
                context=context,
                details={"missing": missing, "unknown": unknown},
            )

    def _cleanup_paths(self, paths: list[Path]) -> None:
        """Remove only exact staging/publication paths owned by this promotion."""

        for path in reversed(paths):
            path = Path(path)
            if not _lexists(path):
                continue
            if path.is_symlink():
                path.unlink(missing_ok=True)
            elif path.is_dir():
                shutil.rmtree(path, ignore_errors=False)
            else:
                path.unlink(missing_ok=True)

__all__ = ["CatalogPromoter", "PromotionResult"]
