"""File-backed driver catalog helpers."""

from __future__ import annotations

from collections.abc import Mapping
from functools import lru_cache
from pathlib import Path
from typing import Any

from runtime.yaml_helpers import load_yaml

DRIVER_CATALOG_SCHEMA_VERSION = "lingtu.driver_catalog.v1"
_REPO_ROOT = Path(__file__).resolve().parents[2]
DRIVER_CATALOG_DIR = _REPO_ROOT / "config" / "driver_backends"


def driver_catalog_path(name: str) -> Path:
    """Return the expected path for a named driver catalog."""

    return DRIVER_CATALOG_DIR / f"{name}.yaml"


@lru_cache(maxsize=None)
def driver_catalog(name: str) -> dict[str, Any]:
    """Load a driver catalog and validate its schema marker."""

    path = driver_catalog_path(name)
    data = load_yaml(path, default={})
    if not isinstance(data, Mapping):
        raise ValueError(f"{path} must contain a YAML mapping")
    catalog = dict(data)
    schema_version = catalog.get("schema_version")
    if schema_version != DRIVER_CATALOG_SCHEMA_VERSION:
        raise ValueError(
            f"{path} schema_version must be {DRIVER_CATALOG_SCHEMA_VERSION!r}, "
            f"got {schema_version!r}"
        )
    return catalog


def driver_catalog_backends(name: str) -> dict[str, dict[str, Any]]:
    return _copy_mapping_of_mappings(driver_catalog(name), "backends")


def driver_catalog_protocols(
    name: str,
) -> dict[str, tuple[str, dict[str, Any]]]:
    catalog = driver_catalog(name)
    raw_protocols = _mapping(catalog, "protocols")
    protocols: dict[str, tuple[str, dict[str, Any]]] = {}
    for backend, raw_protocol in raw_protocols.items():
        if not isinstance(raw_protocol, Mapping):
            raise ValueError(f"protocols.{backend} must be a mapping")
        protocol = str(raw_protocol.get("protocol", "")).strip()
        if not protocol:
            raise ValueError(f"protocols.{backend}.protocol must not be empty")
        params = raw_protocol.get("params") or {}
        if not isinstance(params, Mapping):
            raise ValueError(f"protocols.{backend}.params must be a mapping")
        protocols[str(backend)] = (protocol, dict(params))
    return protocols


def driver_catalog_modules(name: str) -> dict[str, str]:
    modules = _mapping(driver_catalog(name), "modules")
    return {str(backend): str(module) for backend, module in modules.items()}


def _mapping(data: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = data.get(key) or {}
    if not isinstance(value, Mapping):
        raise ValueError(f"{key} must be a mapping")
    return value


def _copy_mapping_of_mappings(
    data: Mapping[str, Any],
    key: str,
) -> dict[str, dict[str, Any]]:
    raw = _mapping(data, key)
    copied: dict[str, dict[str, Any]] = {}
    for name, value in raw.items():
        if not isinstance(value, Mapping):
            raise ValueError(f"{key}.{name} must be a mapping")
        copied[str(name)] = dict(value)
    return copied
