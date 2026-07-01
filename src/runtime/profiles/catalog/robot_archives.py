"""File-backed robot archive helpers for runtime catalog entries."""

from __future__ import annotations

from collections.abc import Mapping
from functools import lru_cache
from pathlib import Path
from typing import Any

from runtime.yaml_helpers import load_yaml


ROBOT_ARCHIVE_SCHEMA_VERSION = "lingtu.robot_archive.v1"
_REPO_ROOT = Path(__file__).resolve().parents[4]
ROBOT_ARCHIVE_DIR = _REPO_ROOT / "config" / "robots"


def robot_archive_path(robot: str) -> Path:
    """Return the expected archive path for a canonical robot family."""

    return ROBOT_ARCHIVE_DIR / f"{robot}.yaml"


@lru_cache(maxsize=None)
def robot_archive(robot: str) -> dict[str, Any]:
    """Load a robot archive as a plain mapping and validate the schema marker."""

    path = robot_archive_path(robot)
    data = load_yaml(path, default={})
    if not isinstance(data, Mapping):
        raise ValueError(f"{path} must contain a YAML mapping")
    archive = dict(data)
    schema_version = archive.get("schema_version")
    if schema_version != ROBOT_ARCHIVE_SCHEMA_VERSION:
        raise ValueError(
            f"{path} schema_version must be {ROBOT_ARCHIVE_SCHEMA_VERSION!r}, "
            f"got {schema_version!r}"
        )
    return archive


def robot_archive_canonical_presets(robot: str) -> dict[str, dict[str, Any]]:
    archive = robot_archive(robot)
    return _copy_mapping_of_mappings(archive, "canonical_presets")


def robot_archive_compat_presets(robot: str) -> dict[str, dict[str, Any]]:
    canonical = robot_archive_canonical_presets(robot)
    return _alias_mapping(robot_archive(robot), canonical)


def robot_archive_canonical_driver_profiles(
    robot: str,
) -> dict[str, tuple[str, dict[str, Any]]]:
    archive = robot_archive(robot)
    raw_profiles = _mapping(archive, "driver_profiles")
    profiles: dict[str, tuple[str, dict[str, Any]]] = {}
    for name, raw_profile in raw_profiles.items():
        if not isinstance(raw_profile, Mapping):
            raise ValueError(f"driver_profiles.{name} must be a mapping")
        protocol = str(raw_profile.get("protocol", "")).strip()
        if not protocol:
            raise ValueError(f"driver_profiles.{name}.protocol must not be empty")
        params = raw_profile.get("params") or {}
        if not isinstance(params, Mapping):
            raise ValueError(f"driver_profiles.{name}.params must be a mapping")
        profiles[str(name)] = (protocol, dict(params))
    return profiles


def robot_archive_compat_driver_profiles(
    robot: str,
) -> dict[str, tuple[str, dict[str, Any]]]:
    canonical = robot_archive_canonical_driver_profiles(robot)
    compat: dict[str, tuple[str, dict[str, Any]]] = {}
    for alias, target in _compat_aliases(robot_archive(robot)).items():
        protocol, params = canonical[target]
        compat[alias] = (protocol, dict(params))
    return compat


def robot_archive_canonical_driver_modules(robot: str) -> dict[str, str]:
    archive = robot_archive(robot)
    modules = _mapping(archive, "driver_modules")
    return {str(name): str(module) for name, module in modules.items()}


def robot_archive_compat_driver_modules(robot: str) -> dict[str, str]:
    canonical = robot_archive_canonical_driver_modules(robot)
    return {
        alias: canonical[target]
        for alias, target in _compat_aliases(robot_archive(robot)).items()
    }


def robot_archive_canonical_runtime_defaults(
    robot: str,
) -> dict[str, dict[str, Any]]:
    archive = robot_archive(robot)
    return _copy_mapping_of_mappings(archive, "runtime_defaults")


def robot_archive_compat_runtime_defaults(robot: str) -> dict[str, dict[str, Any]]:
    canonical = robot_archive_canonical_runtime_defaults(robot)
    return _alias_mapping(robot_archive(robot), canonical)


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


def _compat_aliases(data: Mapping[str, Any]) -> dict[str, str]:
    aliases = _mapping(data, "compat_aliases")
    return {str(alias): str(target) for alias, target in aliases.items()}


def _alias_mapping(
    archive: Mapping[str, Any],
    canonical: Mapping[str, Mapping[str, Any]],
) -> dict[str, dict[str, Any]]:
    compat: dict[str, dict[str, Any]] = {}
    for alias, target in _compat_aliases(archive).items():
        if target not in canonical:
            raise ValueError(f"compat_aliases.{alias} references unknown preset {target!r}")
        compat[alias] = dict(canonical[target])
    return compat
