"""Backend registry — pluggable reconstruction backends by name.

Usage:
    from .registry import register_backend, get_backend, list_backends

    # Register a new backend
    @register_backend("mybackend")
    class MyBackend(ReconBackendBase):
        ...

    # Retrieve by name
    cls = get_backend("tsdf")
    backend = cls()
    result  = backend.reconstruct(keyframes, output_dir)
"""

from __future__ import annotations

from typing import Type

from .base import ReconBackendBase

_registry: dict[str, Type[ReconBackendBase]] = {}


def register_backend(name: str):
    """Class decorator: register a backend under ``name``."""
    def decorator(cls: Type[ReconBackendBase]) -> Type[ReconBackendBase]:
        cls.name = name
        _registry[name] = cls
        return cls
    return decorator


def get_backend(name: str) -> Type[ReconBackendBase]:
    """Return the backend class for ``name``.

    Tries lazy-importing the built-in backends on first call.

    Raises
    ------
    KeyError if the backend is not registered.
    """
    if not _registry:
        _load_builtin_backends()

    if name not in _registry:
        _load_builtin_backends()

    if name not in _registry:
        raise KeyError(
            f"Unknown reconstruction backend '{name}'. "
            f"Available: {list(_registry.keys())}"
        )
    return _registry[name]


def list_backends() -> list[str]:
    """Return names of all registered backends."""
    if not _registry:
        _load_builtin_backends()
    return sorted(_registry.keys())


def _load_builtin_backends() -> None:
    """Lazy-import built-in backends to populate the registry."""
    import importlib
    for module_name in (
        "perception.reconstruction.server.backends.tsdf_backend",
        "perception.reconstruction.server.backends.open3d_backend",
        "perception.reconstruction.server.backends.nerfstudio_backend",
        "perception.reconstruction.server.backends.gsfusion_backend",
    ):
        try:
            importlib.import_module(module_name)
        except ImportError:
            pass


class BackendRegistry:
    """Namespace wrapper for registry functions (convenience)."""

    @staticmethod
    def register(name: str):
        return register_backend(name)

    @staticmethod
    def get(name: str) -> Type[ReconBackendBase]:
        return get_backend(name)

    @staticmethod
    def list() -> list[str]:
        return list_backends()
