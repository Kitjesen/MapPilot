"""Canonical filesystem paths for persistent map packages."""

from __future__ import annotations

import os
from pathlib import Path

DEFAULT_NAV_MAP_DIR = "~/data/nova/maps"
DEFAULT_MAP_ROOTS = (
    DEFAULT_NAV_MAP_DIR,
    "~/data/inovxio/data/maps",
)


def _strip_windows_extended_prefix(path: str) -> str:
    if path.startswith("\\\\?\\UNC\\"):
        return "\\\\" + path[8:]
    if path.startswith("\\\\?\\"):
        return path[4:]
    return path


def _normal_path(path: Path) -> Path:
    return Path(_strip_windows_extended_prefix(str(path))).resolve()


def map_root_candidates() -> tuple[Path, ...]:
    """Return persistent map roots in runtime lookup order."""
    values = (os.environ.get("NAV_MAP_DIR", ""), *DEFAULT_MAP_ROOTS)
    candidates: list[Path] = []
    for value in values:
        if not value:
            continue
        path = _normal_path(Path(value).expanduser())
        if path not in candidates:
            candidates.append(path)
    return tuple(candidates)


def nav_map_root() -> Path:
    """Return the primary persistent map root."""
    return map_root_candidates()[0]


def nav_map_root_str() -> str:
    return str(nav_map_root())


def map_import_root(root: Path | None = None) -> Path:
    """Return the only server-local root accepted for map imports."""

    configured = os.environ.get("LINGTU_MAP_IMPORT_DIR", "").strip()
    if configured:
        return _normal_path(Path(configured).expanduser())
    return _normal_path((root or nav_map_root()) / ".exchange" / "import")


def map_export_root(root: Path | None = None) -> Path:
    """Return the only server-local root accepted for map exports."""

    configured = os.environ.get("LINGTU_MAP_EXPORT_DIR", "").strip()
    if configured:
        return _normal_path(Path(configured).expanduser())
    return _normal_path((root or nav_map_root()) / ".exchange" / "export")


def resolve_exchange_path(
    value: str | Path,
    *,
    root: Path,
    must_exist: bool,
    require_file: bool = False,
    require_dir: bool = False,
    suffixes: tuple[str, ...] = (),
) -> Path:
    """Resolve a map exchange path and reject traversal or host-path escape."""

    raw = str(value or "").strip()
    if not raw:
        raise ValueError("map exchange path is required")
    base = _normal_path(root)
    candidate = Path(raw).expanduser()
    candidate = candidate if candidate.is_absolute() else base / candidate
    try:
        resolved = candidate.resolve(strict=must_exist)
    except OSError as exc:
        raise ValueError(f"map exchange path does not exist: {candidate}") from exc
    resolved = _normal_path(resolved)
    try:
        resolved.relative_to(base)
    except ValueError as exc:
        raise ValueError(f"map exchange path escapes configured root: {base}") from exc
    if require_file and not resolved.is_file():
        raise ValueError(f"map exchange file does not exist: {resolved}")
    if require_dir and not resolved.is_dir():
        raise ValueError(f"map exchange directory does not exist: {resolved}")
    allowed = tuple(item.lower() for item in suffixes)
    if allowed and resolved.suffix.lower() not in allowed:
        raise ValueError(f"map exchange file must use one of: {', '.join(allowed)}")
    return resolved


def active_map_state_file(root: Path | None = None) -> Path:
    return (root or nav_map_root()) / "active_map.txt"


def _direct_child_name(value: str, map_root: Path) -> str | None:
    name = str(value or "").strip()
    if not name:
        return None
    candidate = _normal_path(map_root / name)
    try:
        rel = candidate.relative_to(map_root)
    except ValueError:
        return None
    if len(rel.parts) != 1:
        return None
    return rel.parts[0]


def native_active_map_name(root: Path | None = None) -> str | None:
    """Return the active map from the native maps store state file."""
    map_root = _normal_path(root) if root is not None else nav_map_root()
    state_file = active_map_state_file(map_root)
    try:
        return _direct_child_name(state_file.read_text(encoding="utf-8"), map_root)
    except OSError:
        return None


def active_map_name(root: Path | None = None) -> str | None:
    """Return the active map name from the canonical state file."""

    return native_active_map_name(root)


def active_map_dir(root: Path | None = None) -> Path | None:
    """Resolve the active map package selected by canonical state."""
    map_root = _normal_path(root) if root is not None else nav_map_root()
    name = active_map_name(map_root)
    if name:
        selected = map_root / name
        if selected.is_dir():
            return _normal_path(selected)

    return None


def map_dir_for(name: str, root: Path | None = None) -> Path:
    return (root or nav_map_root()) / name
