"""Shared helper to locate and import _nav_core nanobind extension.

_nav_core is built under src/nav/core/build_nb/ on Linux and
src/nav/core/build_nb_win/ on Windows.  Build scripts may also place a symlink
or copy under src/.  This helper prefers fresh build directories before stale
src/ artifacts so local rebuilds take effect without manual PYTHONPATH edits.
"""

from __future__ import annotations

import importlib.util
import logging
import os
import sys
from pathlib import Path

logger = logging.getLogger(__name__)

# Candidate directories where _nav_core native modules might live, in priority order.
_NATIVE_SUFFIXES = (".so", ".pyd", ".dll", ".dylib")
_DLL_DIRECTORY_HANDLES: list[object] = []
_DLL_DIRECTORY_PATHS: set[str] = set()


def _has_nav_core_binary(directory: str) -> bool:
    return any(
        f.startswith("_nav_core") and f.endswith(_NATIVE_SUFFIXES)
        for f in os.listdir(directory)
    )


def _candidate_dirs() -> list[str]:
    # Walk up from this file to find the repo root (contains lingtu.py or src/)
    here = Path(__file__).resolve()
    for parent in [here.parent, here.parent.parent, here.parent.parent.parent,
                   here.parent.parent.parent.parent]:
        if (parent / "lingtu.py").exists() or (parent / "src").is_dir():
            repo = parent
            break
    else:
        repo = here.parent.parent.parent  # best guess

    return [
        str(repo / "src" / "nav" / "core" / "build_nb_win"),  # Windows build output dir
        str(repo / "src" / "nav" / "core" / "build_nb"),      # Linux build output dir
        str(repo / "src"),                              # symlink/copy installed by build script
        str(repo / "install" / "nav_core" / "lib"),     # colcon install (future)
    ]


def _add_windows_dll_search_dirs(candidate_dirs: list[str]) -> None:
    if os.name != "nt" or not hasattr(os, "add_dll_directory"):
        return

    dirs: list[str] = []
    dirs.extend(candidate_dirs)
    for entry in os.environ.get("PATH", "").split(os.pathsep):
        lower = entry.lower()
        if any(token in lower for token in ("msys", "mingw", "clang", "conda")):
            dirs.append(entry)

    for directory in dirs:
        if not directory or directory in _DLL_DIRECTORY_PATHS or not os.path.isdir(directory):
            continue
        try:
            _DLL_DIRECTORY_HANDLES.append(os.add_dll_directory(directory))
            _DLL_DIRECTORY_PATHS.add(directory)
        except OSError:
            logger.debug("_nav_core_loader: failed to add DLL directory %s", directory)


def ensure_nav_core_on_path() -> None:
    """Add _nav_core build dir to sys.path if not already importable."""
    candidate_dirs = _candidate_dirs()
    _add_windows_dll_search_dirs(candidate_dirs)

    for d in reversed(candidate_dirs):
        if not os.path.isdir(d):
            continue
        if _has_nav_core_binary(d) and d not in sys.path:
            sys.path.insert(0, d)
            logger.debug("_nav_core_loader: added %s to sys.path", d)


def try_import_nav_core(required_symbols: tuple[str, ...] = ()):
    """Import and return _nav_core, or None if unavailable/incompatible.

    Native _nav_core artifacts are frequently rebuilt while developing the
    C++ navigation stack. A stale extension can still import successfully but
    lack newer symbols such as LocalPlannerCore; treating that as available
    makes production-backend checks fail later with misleading AttributeError
    messages. Callers can require the symbols they need so stale artifacts are
    rejected at the boundary.
    """
    def import_candidate():
        import _nav_core
        return _nav_core

    def missing_symbols(module) -> list[str]:
        return [name for name in required_symbols if not hasattr(module, name)]

    ensure_nav_core_on_path()
    for retry in range(2):
        try:
            module = import_candidate()
            missing = missing_symbols(module)
            if not missing:
                return module
            origin = getattr(module, "__file__", "<unknown>")
            logger.warning(
                "_nav_core at %s is missing required symbols: %s",
                origin,
                ", ".join(missing),
            )
            if retry == 0:
                sys.modules.pop("_nav_core", None)
                continue
            return None
        except ImportError:
            if retry == 0:
                sys.modules.pop("_nav_core", None)
                continue
            return None
    return None


def nav_core_build_hint() -> str:
    """Return a human-readable hint for building _nav_core."""
    return (
        "Run:  bash scripts/build/build_nav_core.sh\n"
        "      (needs cmake, python3-dev, pip install nanobind)\n"
        "Windows local builds may use src/nav/core/build_nb_win."
    )
