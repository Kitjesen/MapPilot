"""Shared backend status for pluggable algorithm providers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable


@dataclass
class BackendStatus:
    """Track requested vs. active backend and any degraded runtime state."""

    configured: str
    effective: str
    degraded_reason: str = ""
    _degraded: bool | None = None

    @classmethod
    def configured_as(cls, backend: str) -> "BackendStatus":
        return cls(configured=backend, effective=backend)

    @property
    def degraded(self) -> bool:
        if self._degraded is not None:
            return self._degraded
        return bool(self.degraded_reason) or self.effective != self.configured

    def use(
        self,
        backend: str,
        *,
        reason: str = "",
        degraded: bool | None = None,
    ) -> None:
        self.effective = backend
        self._degraded = degraded
        self.degraded_reason = reason if self.effective != self.configured or reason else ""

    def mark_degraded(self, reason: str) -> None:
        self._degraded = True
        self.degraded_reason = reason

    def as_health_fields(self) -> dict[str, Any]:
        return {
            "configured_backend": self.configured,
            "backend": self.effective,
            "degraded": self.degraded,
            "degraded_reason": self.degraded_reason,
        }


def require_backend(category: str, backend: str, available: Iterable[str]) -> None:
    """Fail fast when a configured backend name is not a known plugin alias."""
    names = sorted(set(available))
    if backend not in names:
        choices = ", ".join(names) if names else "<none>"
        raise ValueError(
            f"Unknown {category} backend '{backend}'. Available: {choices}"
        )
