"""Read-only runtime diagnostics contract shared by Host adapters."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Protocol


class RuntimeStatusProvider(Protocol):
    """Project lifecycle and component status without owning orchestration."""

    @property
    def startup_state(self) -> str:
        """Return the current application lifecycle phase."""

    @property
    def critical_modules(self) -> tuple[str, ...]:
        """Return component aliases required for application readiness."""

    @property
    def failed_modules(self) -> Mapping[str, str]:
        """Return all currently known component failures by alias."""

    @property
    def critical_failures(self) -> Mapping[str, str]:
        """Return failures that invalidate application readiness."""

    @property
    def modules(self) -> Mapping[str, Any]:
        """Return the read-only active component inventory."""

    @property
    def connections(self) -> tuple[tuple[str, str, str, str], ...]:
        """Return the application's internal diagnostic route inventory."""

    def health(self) -> Mapping[str, Any]:
        """Return a JSON-compatible runtime health snapshot."""


__all__ = ["RuntimeStatusProvider"]
