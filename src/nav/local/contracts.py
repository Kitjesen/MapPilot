"""Backend selection contracts for nav.local modules."""

from __future__ import annotations

from runtime.backend_status import require_backend

TERRAIN_BACKENDS = ("nanobind", "simple")
PATH_FOLLOWER_BACKENDS = ("nav_kernel", "pid")


def require_terrain_backend(backend: str) -> None:
    """Validate a terrain backend name."""

    require_backend("terrain", backend, TERRAIN_BACKENDS)


def require_path_follower_backend(backend: str) -> None:
    """Validate a path follower backend name."""

    require_backend("path_follower", backend, PATH_FOLLOWER_BACKENDS)
