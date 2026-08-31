"""Configured local paths used at the mapd exchange boundary."""

from __future__ import annotations

import os
from pathlib import Path

DEFAULT_NAV_MAP_DIR = "~/data/lingtu/maps"


def _strip_windows_extended_prefix(path: str) -> str:
    if path.startswith("\\\\?\\UNC\\"):
        return "\\\\" + path[8:]
    if path.startswith("\\\\?\\"):
        return path[4:]
    return path


def _normal_path(path: Path) -> Path:
    return Path(_strip_windows_extended_prefix(str(path))).resolve()


def nav_map_root() -> Path:
    """Return the single persistent map root."""
    configured = os.environ.get("NAV_MAP_DIR", "").strip() or DEFAULT_NAV_MAP_DIR
    return _normal_path(Path(configured).expanduser())


def map_import_root(root: Path | None = None) -> Path:
    """Return the only server-local root accepted for map imports."""

    configured = os.environ.get("LINGTU_MAP_IMPORT_DIR", "").strip()
    if configured:
        return _normal_path(Path(configured).expanduser())
    return _normal_path((root or nav_map_root()) / ".exchange" / "import")


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
