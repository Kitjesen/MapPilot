"""Read-only package management queries over the canonical simulation catalog."""

from __future__ import annotations

import json
import re
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from .diagnostics import CatalogDiagnostic, DiagnosticCode
from .resolver import CatalogError, CatalogResolver, PackageRecord

_QUALIFICATION_SCHEMA = "lingtu.sim.qualification-record.v1"
_SAFE_IDENTITY_COMPONENT = re.compile(r"^[A-Za-z0-9][A-Za-z0-9+_.-]*$")
_CHECK_STATUSES = {"passed", "failed", "skipped"}


def _canonical_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def _identity(record: PackageRecord) -> dict[str, str]:
    return {
        "kind": record.kind,
        "id": record.id,
        "version": record.version,
        "ref": record.ref,
    }


def _relative_or_absolute(path: Path, root: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return str(path.resolve())


def _require_mapping(value: Any, context: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise CatalogError(
            f"{context} must be an object",
            code=DiagnosticCode.QUALIFICATION_INVALID,
            context=context,
        )
    return value


def _require_string(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value:
        raise CatalogError(
            f"{context} must be a non-empty string",
            code=DiagnosticCode.QUALIFICATION_INVALID,
            context=context,
        )
    return value


def _strict_keys(
    value: Mapping[str, Any],
    *,
    required: set[str],
    optional: set[str],
    context: str,
) -> None:
    missing = sorted(required - set(value))
    unknown = sorted(set(value) - required - optional)
    if missing or unknown:
        details: list[str] = []
        if missing:
            details.append(f"missing: {', '.join(missing)}")
        if unknown:
            details.append(f"unknown: {', '.join(unknown)}")
        raise CatalogError(
            f"{context} has invalid fields ({'; '.join(details)})",
            code=DiagnosticCode.QUALIFICATION_INVALID,
            context=context,
            details={"missing": missing, "unknown": unknown},
        )


def _safe_relative_path(value: Any, context: str) -> str:
    path = _require_string(value, context)
    parts = path.split("/")
    if (
        path.startswith("/")
        or "\\" in path
        or ":" in path
        or re.match(r"^[A-Za-z]:", path) is not None
        or any(not part or part in {".", ".."} for part in parts)
        or any(character.isspace() for character in path)
    ):
        raise CatalogError(
            f"{context} must be a safe relative POSIX path",
            code=DiagnosticCode.PATH_TRAVERSAL,
            context=context,
        )
    return path


def _package_file(record: PackageRecord, value: Any, context: str) -> tuple[str, Path]:
    """Resolve one package-internal file without following an unsafe link."""

    relative = _safe_relative_path(value, context)
    package_root = record.manifest_path.parent.resolve()
    raw_candidate = package_root / Path(*relative.split("/"))
    if raw_candidate.is_symlink():
        raise CatalogError(
            f"{context} does not identify a package-internal file: {relative}",
            code=DiagnosticCode.QUALIFICATION_INVALID,
            context=context,
        )
    candidate = raw_candidate.resolve()
    try:
        candidate.relative_to(package_root)
    except ValueError as exc:
        raise CatalogError(
            f"{context} escapes the package root",
            code=DiagnosticCode.QUALIFICATION_INVALID,
            context=context,
        ) from exc
    if not candidate.is_file():
        raise CatalogError(
            f"{context} does not identify a package-internal file: {relative}",
            code=DiagnosticCode.QUALIFICATION_INVALID,
            context=context,
        )
    return relative, candidate


def _capability_is_subset(qualified: Any, declared: Any) -> bool:
    if isinstance(qualified, Mapping):
        if not isinstance(declared, Mapping):
            return False
        return all(key in declared and _capability_is_subset(value, declared[key]) for key, value in qualified.items())
    if isinstance(qualified, list):
        if not isinstance(declared, list):
            return False
        declared_values = {_canonical_json(item) for item in declared}
        return all(_canonical_json(item) in declared_values for item in qualified)
    return bool(qualified == declared)


class SimCatalog:
    """A deterministic, read-only query facade over one ``CatalogResolver``."""

    def __init__(
        self,
        resolver: CatalogResolver,
        *,
        qualification_roots: Sequence[Path] = (),
    ) -> None:
        self.resolver = resolver
        self.repo_root = resolver.repo_root
        roots = qualification_roots or (self.repo_root / "sim" / "qualifications",)
        self.qualification_roots = tuple(Path(root).resolve() for root in roots)

    @classmethod
    def from_repository(cls, repo_root: Path) -> SimCatalog:
        """Open the repository's canonical package and qualification roots."""

        resolved_root = Path(repo_root).resolve()
        return cls(
            CatalogResolver.from_repository(resolved_root),
            qualification_roots=(resolved_root / "sim" / "qualifications",),
        )

    def list_packages(self, *, kind: str | None = None) -> dict[str, Any]:
        """List validated packages in deterministic identity order."""

        records = [record for record in self.resolver.records if kind is None or record.kind == kind]
        return {
            "schema": "lingtu.sim.catalog-list.v1",
            "packages": [self._summary(record) for record in records],
        }

    def inspect_package(self, reference: str, *, kind: str | None = None) -> dict[str, Any]:
        """Return one package with content, dependency, and qualification views."""

        record = self.resolver.find_package(reference, kind=kind)
        return {
            "schema": "lingtu.sim.catalog-package.v1",
            **self._summary(record),
            "description": record.data.get("description"),
            "manifest_spec": record.data,
            "dependencies": self.dependencies(reference, kind=record.kind),
            "qualification": self.qualification(reference, kind=record.kind),
        }

    def validate_package(self, reference: str, *, kind: str | None = None) -> dict[str, Any]:
        """Validate package content, dependency closure, and qualification currency."""

        record = self.resolver.find_package(reference, kind=kind)
        dependency_graph = self.dependencies(reference, kind=record.kind)
        qualification = self.qualification(reference, kind=record.kind)
        diagnostics = list(qualification["diagnostics"])
        return {
            "schema": "lingtu.sim.catalog-validation.v1",
            "package": _identity(record),
            "catalog_valid": True,
            "dependencies_valid": True,
            "qualification_current": qualification["state"] != "invalid",
            "qualified": qualification["state"] == "qualified",
            "dependency_count": len(dependency_graph["packages"]),
            "diagnostics": diagnostics,
        }

    def dependencies(self, reference: str, *, kind: str | None = None) -> dict[str, Any]:
        """Resolve and return the package's transitive dependency graph."""

        root = self.resolver.find_package(reference, kind=kind)
        packages: dict[tuple[str, str, str], PackageRecord] = {}
        edges: list[dict[str, Any]] = []
        visited: set[tuple[str, str, str]] = set()

        def visit(record: PackageRecord) -> None:
            source_key = (record.kind, record.id, record.version)
            if source_key in visited:
                return
            visited.add(source_key)
            for dependency_kind, dependency_ref, field in self._dependency_requirements(record):
                try:
                    dependency = self.resolver.find_package(dependency_ref, kind=dependency_kind)
                except CatalogError as exc:
                    raise CatalogError(
                        f"package {record.ref} has missing {dependency_kind} dependency {dependency_ref!r} at {field}",
                        code=DiagnosticCode.DEPENDENCY_MISSING,
                        context=record.ref,
                        details={
                            "source_kind": record.kind,
                            "source_ref": record.ref,
                            "field": field,
                            "dependency_kind": dependency_kind,
                            "dependency_ref": dependency_ref,
                        },
                    ) from exc
                dependency_key = (dependency.kind, dependency.id, dependency.version)
                packages[dependency_key] = dependency
                edges.append(
                    {
                        "source": _identity(record),
                        "field": field,
                        "target": _identity(dependency),
                    }
                )
                visit(dependency)

        visit(root)
        ordered_records = [packages[key] for key in sorted(packages)]
        return {
            "schema": "lingtu.sim.package-dependencies.v1",
            "root": _identity(root),
            "packages": [self._summary(record) for record in ordered_records],
            "edges": sorted(
                edges,
                key=lambda edge: (
                    edge["source"]["kind"],
                    edge["source"]["ref"],
                    edge["field"],
                    edge["target"]["kind"],
                    edge["target"]["ref"],
                ),
            ),
        }

    def qualification(self, reference: str, *, kind: str | None = None) -> dict[str, Any]:
        """Return declared and externally qualified capabilities, failing closed."""

        record = self.resolver.find_package(reference, kind=kind)
        candidates = self._qualification_candidates(record)
        if not candidates:
            return self._qualification_view(record, state="unverified")
        if len(candidates) > 1:
            diagnostic = CatalogDiagnostic(
                code=DiagnosticCode.QUALIFICATION_INVALID,
                message=f"multiple qualification records exist for {record.ref}",
                context=record.ref,
                details={"paths": [_relative_or_absolute(path, self.repo_root) for path in candidates]},
            )
            return self._qualification_view(record, state="invalid", diagnostics=(diagnostic,))

        report_path = candidates[0]
        try:
            report = json.loads(report_path.read_text(encoding="utf-8"))
            normalized = self._validate_qualification_record(record, report, report_path)
        except (OSError, json.JSONDecodeError) as exc:
            diagnostic = CatalogDiagnostic(
                code=DiagnosticCode.QUALIFICATION_INVALID,
                message=f"cannot read qualification record for {record.ref}: {exc}",
                context=record.ref,
                details={"path": _relative_or_absolute(report_path, self.repo_root)},
            )
            return self._qualification_view(record, state="invalid", diagnostics=(diagnostic,))
        except CatalogError as exc:
            diagnostic = exc.to_diagnostic()
            return self._qualification_view(record, state="invalid", diagnostics=(diagnostic,))

        checks = normalized["checks"]
        state = "qualified" if all(check["status"] == "passed" for check in checks) else "failed"
        qualified_capabilities = normalized["qualified_capabilities"] if state == "qualified" else {}
        return self._qualification_view(
            record,
            state=state,
            report_path=report_path,
            checks=checks,
            qualified_capabilities=qualified_capabilities,
        )

    def _summary(self, record: PackageRecord) -> dict[str, Any]:
        declared = record.data.get("declared_capabilities", {})
        return {
            "package": _identity(record),
            "manifest": {
                "path": _relative_or_absolute(record.manifest_path, self.repo_root),
            },
            "description": record.data.get("description"),
            "declared_capabilities": declared,
        }

    def _dependency_requirements(self, record: PackageRecord) -> list[tuple[str, str, str]]:
        requirements: list[tuple[str, str, str]] = []
        if record.kind == "robot":
            defaults = record.data["defaults"]
            if defaults["controller"] is not None:
                requirements.append(("controller", defaults["controller"], "defaults.controller"))
            if defaults["sensor_rig"] is not None:
                requirements.append(("sensor_rig", defaults["sensor_rig"], "defaults.sensor_rig"))
        elif record.kind == "sensor_rig":
            for index, sensor in enumerate(record.data["sensors"]):
                requirements.append(("sensor", sensor["package"], f"sensors[{index}].package"))
        elif record.kind == "scenario":
            requirements.append(("world", record.data["world"], "world"))
        return requirements

    def _qualification_candidates(self, record: PackageRecord) -> list[Path]:
        for component, label in (
            (record.kind, "kind"),
            (record.id, "id"),
            (record.version, "version"),
        ):
            if _SAFE_IDENTITY_COMPONENT.fullmatch(component) is None:
                raise CatalogError(
                    f"package qualification {label} is not path-safe: {component!r}",
                    code=DiagnosticCode.PATH_TRAVERSAL,
                    context=record.ref,
                )
        candidates: list[Path] = []
        for root in self.qualification_roots:
            path = (root / record.kind / record.id / f"{record.version}.qualification.json").resolve()
            try:
                path.relative_to(root)
            except ValueError as exc:
                raise CatalogError(
                    f"qualification path escapes root for {record.ref}",
                    code=DiagnosticCode.PATH_TRAVERSAL,
                    context=record.ref,
                ) from exc
            if path.is_file():
                candidates.append(path)
        return sorted(candidates)

    def _validate_qualification_record(
        self,
        record: PackageRecord,
        value: Any,
        report_path: Path,
    ) -> dict[str, Any]:
        context = _relative_or_absolute(report_path, self.repo_root)
        report = _require_mapping(value, context)
        _strict_keys(
            report,
            required={"schema", "package", "qualified_capabilities", "provenance", "checks"},
            optional=set(),
            context=context,
        )
        if _require_string(report["schema"], f"{context}.schema") != _QUALIFICATION_SCHEMA:
            raise CatalogError(
                f"{context}.schema must be {_QUALIFICATION_SCHEMA}",
                code=DiagnosticCode.QUALIFICATION_INVALID,
                context=context,
            )
        package = _require_mapping(report["package"], f"{context}.package")
        _strict_keys(
            package,
            required={"kind", "id", "version"},
            optional=set(),
            context=f"{context}.package",
        )
        expected_identity = {
            "kind": record.kind,
            "id": record.id,
            "version": record.version,
        }
        actual_identity = {key: package.get(key) for key in expected_identity}
        if actual_identity != expected_identity:
            raise CatalogError(
                f"{context}.package does not match {record.ref}",
                code=DiagnosticCode.QUALIFICATION_INVALID,
                context=record.ref,
                details={"expected": expected_identity, "actual": actual_identity},
            )
        provenance = _require_mapping(report["provenance"], f"{context}.provenance")
        _strict_keys(
            provenance,
            required={"path"},
            optional=set(),
            context=f"{context}.provenance",
        )
        provenance_path, _ = _package_file(
            record,
            provenance["path"],
            f"{context}.provenance.path",
        )
        qualified_capabilities = _require_mapping(
            report["qualified_capabilities"],
            f"{context}.qualified_capabilities",
        )
        normalized_capabilities: dict[str, list[str]] = {}
        for capability, raw_values in sorted(qualified_capabilities.items()):
            capability_name = _require_string(
                capability,
                f"{context}.qualified_capabilities key",
            )
            if not isinstance(raw_values, list) or any(not isinstance(item, str) or not item for item in raw_values):
                raise CatalogError(
                    f"{context}.qualified_capabilities.{capability_name} must be an array of strings",
                    code=DiagnosticCode.QUALIFICATION_INVALID,
                    context=record.ref,
                )
            if len(raw_values) != len(set(raw_values)):
                raise CatalogError(
                    f"{context}.qualified_capabilities.{capability_name} contains duplicates",
                    code=DiagnosticCode.QUALIFICATION_INVALID,
                    context=record.ref,
                )
            normalized_capabilities[capability_name] = list(raw_values)
        declared_capabilities = record.data.get("declared_capabilities", {})
        if not _capability_is_subset(normalized_capabilities, declared_capabilities):
            raise CatalogError(
                f"{context}.qualified_capabilities is not a subset of declared capabilities",
                code=DiagnosticCode.QUALIFICATION_INVALID,
                context=record.ref,
            )
        raw_checks = report["checks"]
        if not isinstance(raw_checks, list) or not raw_checks:
            raise CatalogError(
                f"{context}.checks must be a non-empty array",
                code=DiagnosticCode.QUALIFICATION_INVALID,
                context=record.ref,
            )
        checks: list[dict[str, Any]] = []
        seen_ids: set[str] = set()
        for index, raw_check in enumerate(raw_checks):
            check_context = f"{context}.checks[{index}]"
            check = _require_mapping(raw_check, check_context)
            _strict_keys(
                check,
                required={"id", "status"},
                optional={"evidence"},
                context=check_context,
            )
            check_id = _require_string(check["id"], f"{check_context}.id")
            status = _require_string(check["status"], f"{check_context}.status")
            if check_id in seen_ids:
                raise CatalogError(
                    f"{context}.checks contains duplicate id {check_id!r}",
                    code=DiagnosticCode.QUALIFICATION_INVALID,
                    context=record.ref,
                )
            if status not in _CHECK_STATUSES:
                raise CatalogError(
                    f"{check_context}.status is unsupported: {status!r}",
                    code=DiagnosticCode.QUALIFICATION_INVALID,
                    context=record.ref,
                )
            if status == "passed" and "evidence" not in check:
                raise CatalogError(
                    f"{check_context}.evidence is required for a passed check",
                    code=DiagnosticCode.QUALIFICATION_INVALID,
                    context=record.ref,
                )
            seen_ids.add(check_id)
            normalized_check: dict[str, Any] = {"id": check_id, "status": status}
            if "evidence" in check:
                raw_evidence = check["evidence"]
                if not isinstance(raw_evidence, list) or not raw_evidence:
                    raise CatalogError(
                        f"{check_context}.evidence must be a non-empty array",
                        code=DiagnosticCode.QUALIFICATION_INVALID,
                        context=record.ref,
                    )
                evidence: list[dict[str, str]] = []
                for evidence_index, raw_item in enumerate(raw_evidence):
                    evidence_context = f"{check_context}.evidence[{evidence_index}]"
                    item = _require_mapping(raw_item, evidence_context)
                    _strict_keys(
                        item,
                        required={"path"},
                        optional=set(),
                        context=evidence_context,
                    )
                    relative_path = _safe_relative_path(
                        item["path"],
                        f"{evidence_context}.path",
                    )
                    evidence_path = (report_path.parent / relative_path).resolve()
                    try:
                        evidence_path.relative_to(report_path.parent.resolve())
                    except ValueError as exc:
                        raise CatalogError(
                            f"{evidence_context}.path escapes the qualification directory",
                            code=DiagnosticCode.QUALIFICATION_INVALID,
                            context=record.ref,
                        ) from exc
                    if not evidence_path.is_file():
                        raise CatalogError(
                            f"{evidence_context}.path does not exist: {relative_path}",
                            code=DiagnosticCode.QUALIFICATION_INVALID,
                            context=record.ref,
                        )
                    evidence.append({"path": relative_path})
                normalized_check["evidence"] = evidence
            checks.append(normalized_check)
        return {
            "qualified_capabilities": normalized_capabilities,
            "provenance": {"path": provenance_path},
            "checks": checks,
        }

    def _qualification_view(
        self,
        record: PackageRecord,
        *,
        state: str,
        report_path: Path | None = None,
        checks: Sequence[Mapping[str, Any]] = (),
        qualified_capabilities: Mapping[str, Any] | None = None,
        diagnostics: Sequence[CatalogDiagnostic] = (),
    ) -> dict[str, Any]:
        report = None
        if report_path is not None:
            report = {
                "path": _relative_or_absolute(report_path, self.repo_root),
            }
        return {
            "schema": "lingtu.sim.qualification-view.v1",
            "package": _identity(record),
            "state": state,
            "declared_capabilities": record.data.get("declared_capabilities", {}),
            "qualified_capabilities": dict(qualified_capabilities or {}),
            "checks": [dict(check) for check in checks],
            "report": report,
            "diagnostics": [diagnostic.to_dict() for diagnostic in diagnostics],
        }


__all__ = ["SimCatalog"]
