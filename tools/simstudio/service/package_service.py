"""SimStudio package catalog/import service."""

from __future__ import annotations

import copy
import hashlib
import os
import tempfile
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from sim.catalog.importers.contracts import ImportDraft, ImportFailure, file_records, sha256_file
from sim.catalog.importers.intake import SourceIntake
from sim.catalog.importers.promotion import CatalogPromoter
from sim.catalog.importers.robot import RobotImporter
from sim.catalog.importers.world import WorldImporter
from sim.catalog.management import SimCatalog
from sim.catalog.resolver import CatalogError

from .models import ImportJobRecord, StoreError, StoreValidationError
from .store import StudioStore

_CATALOG_KINDS = {"controller", "robot", "scenario", "sensor", "sensor_rig", "world"}
_IMPORT_KINDS = {"robot", "world"}
_IMPORT_TEMPLATES: dict[str, dict[str, Any]] = {
    "robot": {
        "schema": "lingtu.sim.robot-import-request.v1",
        "id": "new_robot",
        "version": "1.0.0",
        "description": "Imported robot package",
        "source_format": "mjcf",
        "source_model": "robot.xml",
        "units": {"length": "m", "angle": "radian"},
        "provenance": {
            "owner": "Replace with asset owner",
            "license": "LicenseRef-ReplaceMe",
            "license_file": "LICENSE.txt",
            "source_uri": "file://simstudio-upload",
        },
        "physics": {
            "attach_root": "base_link",
            "root_joint": "floating_base_joint",
        },
        "visual": {"binding": "RobotVisual:NewRobot"},
        "semantic": {"class": "mobile_robot"},
        "frames": [{"name": "base_link", "role": "body"}],
        "interfaces": {
            "state": ["lingtu.sim.base-state.v1"],
            "command": ["lingtu.sim.base-velocity.v1"],
        },
        "defaults": {"controller": None, "sensor_rig": None},
        "declared_capabilities": {},
    },
    "world": {
        "schema": "lingtu.sim.world-import-request.v1",
        "package": {
            "id": "new_world",
            "version": "1.0.0",
            "description": "Imported heightmap world",
        },
        "source": {
            "provenance": {
                "owner": "Replace with asset owner",
                "license": "LicenseRef-ReplaceMe",
                "license_file": "LICENSE.txt",
                "source_uri": "file://simstudio-upload",
                "third_party_assets": [],
            }
        },
        "units": {"length": "m", "up_axis": "Z", "handedness": "RH"},
        "heightmap": {
            "path": "height.r16",
            "width": 1024,
            "height": 1024,
            "extent_m": [100.0, 100.0],
            "elevation_min_m": 0.0,
            "elevation_max_m": 10.0,
            "endian": "little",
        },
        "visual": {
            "binding": "WorldVisual:NewWorld",
            "level": "/Game/RobotSim/Maps/NewWorld",
        },
        "spawn": {
            "position_m": [0.0, 0.0, 0.0],
            "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            "height_tolerance_m": 0.25,
        },
        "entities": [],
    },
}


class PackageServiceError(ValueError):
    """Stable package service error that adapters can render as JSON."""

    def __init__(
        self,
        code: str,
        message: str,
        *,
        details: Mapping[str, Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.message = message
        self.details = dict(details or {})

    def to_dict(self) -> dict[str, Any]:
        """Return the stable error envelope exposed to service adapters."""
        result: dict[str, Any] = {"code": self.code, "message": self.message}
        if self.details:
            result["details"] = copy.deepcopy(self.details)
        return result


class PackageImportService:
    """Catalog query, managed import, and promotion boundary for SimStudio."""

    def __init__(
        self,
        *,
        catalog: SimCatalog,
        store: StudioStore,
        repo_root: Path,
        inbox_root: Path | None = None,
        intake: SourceIntake | None = None,
    ) -> None:
        self.catalog = catalog
        self.store = store
        self.repo_root = Path(repo_root).resolve()
        inbox = Path(inbox_root) if inbox_root is not None else store.root / "inbox"
        # Keep the lexical path until the intake boundary has inspected every
        # component.  resolve() here would erase a Windows junction/symlink's
        # identity before _inbox_source can reject it.
        self.inbox_root = Path(os.path.abspath(os.fspath(inbox)))
        # Importers create several nested evidence paths.  Keeping their
        # staging root short is important on Windows, where a managed Studio
        # store may itself live below a long pytest/workspace path.
        store_name = store.root.name or "store"
        store_scope = store.root.parent.name or "root"
        store_key = hashlib.sha256(str(store.root.resolve()).encode("utf-8")).hexdigest()[:12]
        self.import_root = (
            Path(tempfile.gettempdir())
            / "lingtu-simstudio"
            / f"{store_scope}-{store_key}"
            / store_name
        ).resolve()
        self.import_root.mkdir(parents=True, exist_ok=True)
        self.intake = intake or SourceIntake()

    @classmethod
    def from_repository(
        cls,
        *,
        repo_root: Path,
        store: StudioStore,
        inbox_root: Path | None = None,
    ) -> PackageImportService:
        """Construct a service backed by the repository's current catalog."""
        root = Path(repo_root).resolve()
        return cls(catalog=SimCatalog.from_repository(root), store=store, repo_root=root, inbox_root=inbox_root)

    def list_packages(self, *, kind: str | None = None) -> dict[str, Any]:
        """List catalog packages, optionally restricted to one supported kind."""
        return self.catalog.list_packages(kind=self._catalog_kind(kind) if kind is not None else None)

    def detail_package(self, reference: str, *, kind: str) -> dict[str, Any]:
        """Return the catalog detail view for one package reference."""
        return self.catalog.inspect_package(reference, kind=self._catalog_kind(kind))

    def inspect_package(self, reference: str, *, kind: str) -> dict[str, Any]:
        """Alias matching the existing Catalog/application vocabulary."""
        return self.detail_package(reference, kind=kind)

    def validate_package(self, reference: str, *, kind: str) -> dict[str, Any]:
        """Validate one catalog package and its qualification state."""
        return self.catalog.validate_package(reference, kind=self._catalog_kind(kind))

    def qualification(self, reference: str, *, kind: str) -> dict[str, Any]:
        """Return the qualification view for one catalog package."""
        return self.catalog.qualification(reference, kind=self._catalog_kind(kind))

    def import_contract(self, kind: str) -> dict[str, Any]:
        """Return the canonical Studio request template for one importer."""

        package_kind = self._import_kind(kind)
        schema_path = (
            self.repo_root
            / "sim" / "contracts" / "schemas"
            / f"{package_kind}-import.v1.json"
        )
        if not schema_path.is_file():
            raise PackageServiceError(
                "PACKAGE_IMPORT_CONTRACT_MISSING",
                "package import schema is missing",
                details={"kind": package_kind},
            )
        return {
            "schema": "lingtu.sim.studio.import-contract.v1",
            "kind": package_kind,
            "source_entry_owned_by": "simstudio",
            "request_schema": {
                "id": f"lingtu.sim.{package_kind}-import-request.v1",
                "path": schema_path.relative_to(self.repo_root).as_posix(),
                "sha256": sha256_file(schema_path),
            },
            "request_template": copy.deepcopy(_IMPORT_TEMPLATES[package_kind]),
        }

    def create_import_job(
        self,
        *,
        kind: str,
        request: Mapping[str, Any],
        source_entry: str,
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """Import one managed inbox source and persist its terminal job state."""
        package_kind = self._import_kind(kind)
        source_path = self._inbox_source(source_entry)
        payload: dict[str, Any] = {
            "schema": "lingtu.sim.studio.package-import-request.v1",
            "kind": package_kind,
            "source_path": StudioStore.validate_relative_path(source_entry, context="source_entry"),
            "request": copy.deepcopy(dict(request)),
            "diagnostics": [],
            "source_identity": self._source_identity(source_path),
        }
        if idempotency_key is not None:
            existing = self.store.get_idempotency("create:import_job", idempotency_key)
            if existing is not None:
                current = self.store.get_import_job(existing.resource_id)
                return current.to_dict()
        record = self.store.create_import_job(payload, status="RUNNING", idempotency_key=idempotency_key)
        if record.status != "RUNNING":
            return record.to_dict()
        try:
            draft = self._run_import(record.id, package_kind, dict(request), source_path)
            diagnostics = [item.to_dict() for item in draft.diagnostics]
            status = "READY" if draft.state == "qualified" else "FAILED"
            payload.update(
                {
                    "draft": self._draft_record(draft),
                    "diagnostics": diagnostics,
                    "result": self._draft_result(draft),
                }
            )
            record = self.store.update_import_job(
                record.id,
                expected_revision=record.revision,
                payload=payload,
                status=status,
            )
            return record.to_dict()
        except Exception as exc:
            diagnostic = self._diagnostic(exc)
            payload.update({"diagnostics": [diagnostic], "error": diagnostic})
            record = self.store.update_import_job(
                record.id,
                expected_revision=record.revision,
                payload=payload,
                status="FAILED",
            )
            return record.to_dict()

    def get_import_job(self, job_id: str) -> dict[str, Any]:
        """Return one persisted import job."""
        return self.store.get_import_job(job_id).to_dict()

    def list_import_jobs(self) -> list[dict[str, Any]]:
        """Return all persisted import jobs in store order."""
        return [record.to_dict() for record in self.store.list_import_jobs()]

    def promote_import_job(self, job_id: str, *, idempotency_key: str | None = None) -> dict[str, Any]:
        """Promote a READY draft once and refresh the in-memory catalog."""
        if idempotency_key is not None:
            existing = self.store.get_idempotency("update:import_job", idempotency_key)
            if existing is not None:
                if existing.resource_kind != "import_job" or existing.resource_id != job_id:
                    raise PackageServiceError(
                        "PACKAGE_IDEMPOTENCY_CONFLICT",
                        "idempotency key already belongs to a different import job",
                        details={"resource_kind": existing.resource_kind, "resource_id": existing.resource_id},
                    )
                return self.store.get_import_job(job_id).to_dict()

        record = self.store.get_import_job(job_id)
        if record.status != "READY":
            raise PackageServiceError(
                "PACKAGE_IMPORT_NOT_READY",
                "only READY import jobs can be promoted",
                details={"job_id": job_id, "status": record.status},
            )
        draft = self._draft_from_record(record)
        try:
            promotion_result = CatalogPromoter(self.repo_root).promote(draft)
            promotion = self._promotion_record(promotion_result)
        except ImportFailure as exc:
            raise self._service_error(exc, "PACKAGE_PROMOTION_FAILED") from exc
        payload = copy.deepcopy(dict(record.payload))
        payload["promotion"] = promotion
        payload.setdefault("diagnostics", [])
        updated = self.store.update_import_job(
            record.id,
            expected_revision=record.revision,
            payload=payload,
            status="PROMOTED",
            idempotency_key=idempotency_key,
        )
        self.catalog = SimCatalog.from_repository(self.repo_root)
        return updated.to_dict()

    def _promotion_record(self, promotion: Any) -> dict[str, Any]:
        """Convert promoter host paths to repository-relative references."""
        value = promotion.to_dict()
        result = copy.deepcopy(value)
        for field in ("package_root", "qualification_path"):
            raw = result.get(field)
            if not isinstance(raw, str):
                raise PackageServiceError("PACKAGE_PROMOTION_INVALID", f"promotion.{field} is invalid")
            path = Path(raw).resolve()
            try:
                result[field] = path.relative_to(self.repo_root).as_posix()
            except ValueError as exc:
                raise PackageServiceError(
                    "PACKAGE_PROMOTION_INVALID",
                    f"promotion.{field} escapes the repository root",
                ) from exc
        return result

    def _run_import(self, job_id: str, kind: str, request: dict[str, Any], source_path: Path) -> ImportDraft:
        if kind == "robot":
            import_request = {**request, "source": str(source_path)}
            return RobotImporter(
                self.repo_root,
                work_root=self.import_root,
                intake=self.intake,
            ).import_robot(import_request)
        import_request = copy.deepcopy(request)
        source = import_request.get("source")
        if not isinstance(source, Mapping):
            raise PackageServiceError("PACKAGE_INVALID_REQUEST", "world import request.source must be an object")
        import_request["source"] = {**dict(source), "path": str(source_path)}
        return WorldImporter(self.repo_root, intake=self.intake).import_world(
            import_request,
            draft_root=self.import_root / "world" / job_id,
        )

    def _inbox_source(self, source_entry: str) -> Path:
        try:
            safe = StudioStore.validate_relative_path(source_entry, context="source_entry")
        except StoreValidationError as exc:
            raise PackageServiceError(
                "PACKAGE_UNSAFE_SOURCE",
                str(exc),
                details={"source_entry": source_entry},
            ) from exc
        base = self.inbox_root
        candidate = base.joinpath(*safe.split("/"))
        try:
            StudioStore._assert_no_reparse_components(base)
            StudioStore._assert_no_reparse_components(candidate, below=base)
        except StoreValidationError as exc:
            raise PackageServiceError(
                "PACKAGE_UNSAFE_SOURCE",
                "source entry must not contain symbolic links or Windows reparse points",
                details={"source_entry": source_entry},
            ) from exc
        resolved_base = base.resolve()
        resolved = candidate.resolve()
        try:
            resolved.relative_to(resolved_base)
        except ValueError as exc:
            raise PackageServiceError(
                "PACKAGE_UNSAFE_SOURCE",
                "source entry escapes the inbox root",
                details={"source_entry": source_entry},
            ) from exc
        try:
            StudioStore._assert_no_reparse_components(resolved_base)
            StudioStore._assert_no_reparse_components(resolved, below=resolved_base)
        except StoreValidationError as exc:
            raise PackageServiceError(
                "PACKAGE_UNSAFE_SOURCE",
                "resolved source entry contains a symbolic link or Windows reparse point",
                details={"source_entry": source_entry},
            ) from exc
        if not resolved.exists():
            raise PackageServiceError(
                "PACKAGE_SOURCE_NOT_FOUND",
                "source entry does not exist",
                details={"source_entry": source_entry},
            )
        return resolved

    def _source_identity(self, source_path: Path) -> dict[str, Any]:
        try:
            if source_path.is_dir():
                return {
                    "kind": "directory",
                    "files": [item.to_dict() for item in file_records(source_path)],
                }
            if source_path.is_file():
                return {"kind": "file", "bytes": source_path.stat().st_size, "sha256": sha256_file(source_path)}
        except ImportFailure as exc:
            raise self._service_error(exc, "PACKAGE_UNSAFE_SOURCE") from exc
        raise PackageServiceError("PACKAGE_SOURCE_NOT_FOUND", "source entry is not a regular file or directory")

    @staticmethod
    def _draft_result(draft: ImportDraft) -> dict[str, Any]:
        return {
            "state": draft.state,
            "package": {
                "kind": draft.kind,
                "id": draft.package_id,
                "version": draft.version,
                "ref": draft.reference,
            },
        }

    def _draft_record(self, draft: ImportDraft) -> dict[str, Any]:
        """Persist only opaque identity and relative service-owned locations."""
        root = self._owned_import_path(draft.root, field="draft.root")

        def relative(value: Path | None, *, field: str) -> str | None:
            if value is None:
                return None
            resolved = self._owned_import_path(value, field=field)
            try:
                return resolved.relative_to(root).as_posix()
            except ValueError as exc:
                raise PackageServiceError(
                    "PACKAGE_IMPORT_DRAFT_INVALID",
                    f"{field} escapes the draft staging root",
                ) from exc

        return {
            "schema": "lingtu.sim.import-draft-reference.v1",
            "import_id": draft.import_id,
            "package": {
                "kind": draft.kind,
                "id": draft.package_id,
                "version": draft.version,
                "ref": draft.reference,
            },
            "state": draft.state,
            "staging_ref": root.relative_to(self.import_root).as_posix(),
            "locations": {
                "package": relative(draft.package_root, field="draft.package_root"),
                "manifest": relative(draft.manifest_path, field="draft.manifest_path"),
                "provenance": relative(draft.provenance_path, field="draft.provenance_path"),
                "qualification": relative(draft.qualification_path, field="draft.qualification_path"),
            },
            "diagnostics": [item.to_dict() for item in draft.diagnostics],
        }

    def _draft_from_record(self, record: ImportJobRecord) -> ImportDraft:
        draft = record.payload.get("draft")
        if not isinstance(draft, Mapping):
            raise PackageServiceError(
                "PACKAGE_IMPORT_DRAFT_MISSING",
                "import job does not contain a persisted draft",
                details={"job_id": record.id},
            )
        package = draft.get("package")
        if not isinstance(package, Mapping):
            raise PackageServiceError("PACKAGE_IMPORT_DRAFT_INVALID", "persisted import draft package is invalid")
        if draft.get("schema") != "lingtu.sim.import-draft-reference.v1":
            raise PackageServiceError("PACKAGE_IMPORT_DRAFT_INVALID", "persisted import draft schema is invalid")
        staging_ref = draft.get("staging_ref")
        try:
            safe_staging_ref = StudioStore.validate_relative_path(staging_ref, context="draft.staging_ref")
        except StoreValidationError as exc:
            raise PackageServiceError("PACKAGE_IMPORT_DRAFT_INVALID", str(exc)) from exc
        root = self._owned_import_path(
            self.import_root / Path(*safe_staging_ref.split("/")),
            field="draft.staging_ref",
        )
        locations = draft.get("locations")
        if not isinstance(locations, Mapping):
            raise PackageServiceError("PACKAGE_IMPORT_DRAFT_INVALID", "persisted import draft locations are invalid")

        def location(name: str, *, directory: bool = False) -> Path | None:
            value = locations.get(name)
            if value is None:
                return None
            try:
                safe = StudioStore.validate_relative_path(value, context=f"draft.locations.{name}")
            except StoreValidationError as exc:
                raise PackageServiceError("PACKAGE_IMPORT_DRAFT_INVALID", str(exc)) from exc
            candidate = self._owned_import_path(root / Path(*safe.split("/")), field=f"draft.locations.{name}")
            if directory and not candidate.is_dir():
                raise PackageServiceError("PACKAGE_IMPORT_DRAFT_INVALID", f"draft location is not a directory: {name}")
            if not directory and not candidate.is_file():
                raise PackageServiceError("PACKAGE_IMPORT_DRAFT_INVALID", f"draft location is not a file: {name}")
            return candidate

        package_root = location("package", directory=True)
        manifest_path = location("manifest")
        provenance_path = location("provenance")
        qualification_path = location("qualification")
        return ImportDraft(
            import_id=str(draft.get("import_id", "")),
            kind=str(package.get("kind", "")),
            package_id=str(package.get("id", "")),
            version=str(package.get("version", "")),
            state=str(draft.get("state", "")),
            root=root,
            package_root=package_root,
            manifest_path=manifest_path,
            provenance_path=provenance_path,
            qualification_path=qualification_path,
            diagnostics=tuple(),
        )

    def _owned_import_path(self, value: Any, *, field: str) -> Path:
        if not isinstance(value, (str, Path)) or not value:
            raise PackageServiceError("PACKAGE_IMPORT_DRAFT_INVALID", f"{field} must be a path")
        candidate = Path(value)
        if candidate.is_symlink():
            raise PackageServiceError("PACKAGE_IMPORT_DRAFT_INVALID", f"{field} must not be a symbolic link")
        resolved = candidate.resolve()
        try:
            resolved.relative_to(self.import_root.resolve())
        except ValueError as exc:
            raise PackageServiceError(
                "PACKAGE_IMPORT_DRAFT_INVALID",
                f"{field} escapes the service-owned import root",
                details={"field": field},
            ) from exc
        return resolved

    def _owned_optional_import_path(self, value: Any, *, field: str) -> Path | None:
        if value is None or value == "":
            return None
        return self._owned_import_path(value, field=field)

    @staticmethod
    def _catalog_kind(kind: str | None) -> str:
        if kind not in _CATALOG_KINDS:
            raise PackageServiceError(
                "PACKAGE_UNSUPPORTED_KIND",
                "unsupported package kind",
                details={"kind": kind, "supported": sorted(_CATALOG_KINDS)},
            )
        return kind

    @staticmethod
    def _import_kind(kind: str | None) -> str:
        if kind not in _IMPORT_KINDS:
            raise PackageServiceError(
                "PACKAGE_UNSUPPORTED_IMPORT_KIND",
                "unsupported package import kind",
                details={"kind": kind, "supported": sorted(_IMPORT_KINDS)},
            )
        return kind

    @staticmethod
    def _diagnostic(exc: Exception) -> dict[str, Any]:
        if isinstance(exc, ImportFailure):
            return exc.to_diagnostic().to_dict()
        if isinstance(exc, PackageServiceError):
            return exc.to_dict()
        if isinstance(exc, CatalogError):
            return exc.to_diagnostic().to_dict()
        if isinstance(exc, StoreError):
            return {"code": "PACKAGE_STORE_ERROR", "message": str(exc)}
        return {"code": "PACKAGE_IMPORT_FAILED", "message": str(exc)}

    @staticmethod
    def _service_error(exc: ImportFailure, fallback_code: str) -> PackageServiceError:
        diagnostic = exc.to_diagnostic().to_dict()
        return PackageServiceError(
            str(diagnostic.get("code") or fallback_code),
            str(diagnostic.get("message") or exc),
            details={key: value for key, value in diagnostic.items() if key not in {"code", "message"}},
        )


__all__ = ["PackageImportService", "PackageServiceError"]
