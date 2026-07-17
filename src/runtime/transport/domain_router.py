"""DDS domain router for multi-domain and cross-host deployments.

Routes topics to different DDS domain participants based on configurable
pattern rules.  When no rules match, a default domain is used.

Example::

    router = DomainRouter({
        "/slam/*":        {"domain_id": 42, "host": "192.168.66.190"},
        "/perception/*":  {"domain_id": 0},
        "*":              {"domain_id": 0},
    })
    info = router.resolve("/slam/map_cloud")
    # info == DomainRoute(domain_id=42, host="192.168.66.190")
"""

from __future__ import annotations

import fnmatch
import logging
from dataclasses import dataclass, field
from typing import Any

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# DomainRoute — a resolved routing decision
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class DomainRoute:
    """Resolved domain routing information for a single topic."""

    domain_id: int = 0
    host: str | None = None
    qos_profile: str | None = None

    @property
    def is_remote(self) -> bool:
        return self.host is not None and self.host not in ("localhost", "127.0.0.1", "::1", "")


# ---------------------------------------------------------------------------
# DomainRouter
# ---------------------------------------------------------------------------


class DomainRouter:
    """Route topics to DDS domains based on fnmatch patterns.

    Parameters:
        routes: Mapping of *topic pattern* → route specification dict.
            Each spec may contain ``domain_id`` (int), ``host`` (str), and
            ``qos_profile`` (str).  Patterns are matched in insertion order;
            the first match wins.  The catch-all ``"*"`` pattern is
            conventionally placed last as a default.

    When *routes* is empty or ``None`` the router is a no-op and always
    returns the *default_domain*.
    """

    def __init__(
        self,
        routes: dict[str, dict[str, Any]] | None = None,
        *,
        default_domain: int = 0,
    ) -> None:
        self._patterns: list[tuple[str, DomainRoute]] = []
        self._default = DomainRoute(domain_id=default_domain)

        if routes:
            for pattern, spec in routes.items():
                route = DomainRoute(
                    domain_id=spec.get("domain_id", default_domain),
                    host=spec.get("host"),
                    qos_profile=spec.get("qos_profile"),
                )
                self._patterns.append((pattern, route))
                logger.debug(
                    "[DomainRouter] pattern=%r → domain=%s host=%s",
                    pattern,
                    route.domain_id,
                    route.host,
                )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def resolve(self, topic: str) -> DomainRoute:
        """Return the :class:`DomainRoute` for *topic*.

        Patterns are checked in order using :func:`fnmatch.fnmatch`.  The
        first matching pattern wins.  If no pattern matches, the default
        domain route is returned.
        """
        for pattern, route in self._patterns:
            if fnmatch.fnmatch(topic, pattern):
                return route
        return self._default

    def domain_id_for(self, topic: str) -> int:
        """Shortcut — return only the domain ID for *topic*."""
        return self.resolve(topic).domain_id

    @property
    def has_routes(self) -> bool:
        """``True`` when at least one routing pattern is configured."""
        return len(self._patterns) > 0

    def patterns(self) -> list[tuple[str, DomainRoute]]:
        """Return a copy of the (pattern, route) pairs."""
        return list(self._patterns)

    def __repr__(self) -> str:
        return f"DomainRouter(patterns={len(self._patterns)}, default={self._default})"


# ---------------------------------------------------------------------------
# Module-level singleton (opt-in)
# ---------------------------------------------------------------------------

_global_router: DomainRouter | None = None


def set_global_router(router: DomainRouter | None) -> None:
    """Install a process-wide :class:`DomainRouter` (or clear it)."""
    global _global_router
    _global_router = router


def get_global_router() -> DomainRouter | None:
    """Return the current global router, or ``None``."""
    return _global_router


def resolve_domain_for_topic(topic: str, *, fallback: int = 0) -> int:
    """Resolve the DDS domain ID for *topic* using the global router.

    Falls back to *fallback* when no global router is installed or no
    pattern matches.
    """
    router = _global_router
    if router is None or not router.has_routes:
        return fallback
    return router.domain_id_for(topic)
