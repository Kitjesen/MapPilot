"""Structured diagnostics shared by simulation catalog application surfaces."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping


class DiagnosticCode(str, Enum):
    """Stable machine-facing diagnostic codes for catalog and composition failures."""

    CATALOG_INVALID = "SIMCATALOG_INVALID"
    DUPLICATE_IDENTITY = "SIMCATALOG_DUPLICATE_IDENTITY"
    PATH_TRAVERSAL = "SIMCATALOG_PATH_TRAVERSAL"
    PACKAGE_NOT_FOUND = "SIMCATALOG_PACKAGE_NOT_FOUND"
    DEPENDENCY_MISSING = "SIMCATALOG_DEPENDENCY_MISSING"
    ARTIFACT_CONFLICT = "SIMCATALOG_ARTIFACT_CONFLICT"
    QUALIFICATION_INVALID = "SIMCATALOG_QUALIFICATION_INVALID"
    QUALIFICATION_STALE = "SIMCATALOG_QUALIFICATION_STALE"
    GLOBAL_PHYSICS_OWNERSHIP = "SIMCATALOG_GLOBAL_PHYSICS_OWNERSHIP"
    SENSOR_TIMEBASE_INCOMPATIBLE = "SIMCATALOG_SENSOR_TIMEBASE_INCOMPATIBLE"
    OVERRIDE_INVALID = "SESSION_INTENT_OVERRIDE_INVALID"
    INTENT_INVALID = "SESSION_INTENT_INVALID"


@dataclass(frozen=True)
class CatalogDiagnostic:
    """One deterministic diagnostic suitable for CLI, HTTP, and UI clients."""

    code: DiagnosticCode | str
    message: str
    severity: str = "error"
    context: str | None = None
    details: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable diagnostic view."""

        result: dict[str, Any] = {
            "code": self.code.value if isinstance(self.code, DiagnosticCode) else str(self.code),
            "severity": self.severity,
            "message": self.message,
        }
        if self.context is not None:
            result["context"] = self.context
        if self.details:
            result["details"] = dict(self.details)
        return result
