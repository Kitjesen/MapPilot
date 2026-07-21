"""Gateway access to the assembled inspection domain service."""

from __future__ import annotations

from typing import Any


class InspectionBoundaryError(RuntimeError):
    """Raised when the inspection service is absent or rejects an operation."""


def inspection_service(gateway: Any) -> Any | None:
    service = getattr(gateway, "_inspection", None)
    if service is not None:
        return service
    modules = getattr(gateway, "_all_modules", None)
    if isinstance(modules, dict):
        return modules.get("nav.inspection")
    return None


def invoke_inspection(gateway: Any, method: str, **kwargs: Any) -> Any:
    service = inspection_service(gateway)
    if service is None:
        raise InspectionBoundaryError("inspection service is unavailable")
    operation = getattr(service, method, None)
    if not callable(operation):
        raise InspectionBoundaryError(f"inspection service does not implement {method}")
    try:
        return operation(**kwargs)
    except Exception as exc:
        raise InspectionBoundaryError(str(exc)) from exc


__all__ = ["InspectionBoundaryError", "inspection_service", "invoke_inspection"]
