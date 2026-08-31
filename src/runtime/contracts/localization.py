"""Canonical localization-status values shared by runtime adapters."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

DEGENERACY_LEVELS = frozenset({"NONE", "MILD", "SEVERE", "CRITICAL", "UNKNOWN"})


def degeneracy_level(status: Mapping[str, Any]) -> str:
    """Translate native diagnostics into the public degeneracy enum."""

    existing = str(status.get("degeneracy") or "").strip().upper()
    if existing in DEGENERACY_LEVELS:
        return existing

    raw = status.get("fastlio_degeneracy")
    diagnostics = raw if isinstance(raw, Mapping) else status
    detected_value = diagnostics.get("detected", diagnostics.get("degeneracy_detected"))
    count_value = diagnostics.get("degenerate_dof_count")
    try:
        dof_count = None if count_value is None else max(0, int(count_value))
    except (TypeError, ValueError):
        dof_count = None

    if detected_value is None and dof_count is None:
        return "UNKNOWN"
    detected = bool(detected_value) if detected_value is not None else bool(dof_count)
    if not detected:
        return "NONE"
    if dof_count == 1:
        return "MILD"
    if dof_count is not None and dof_count >= 6:
        return "CRITICAL"
    return "SEVERE"
