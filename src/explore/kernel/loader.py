"""Native loader for the LingTu exploration algorithm kernel.

The C++ source lives under ``src/explore/cpp`` and builds the Python extension
``lingtu_explore_kernel``. Keep extension discovery and symbol checks here so
Modules and policy adapters do not encode build-layout details.
"""

from __future__ import annotations

import importlib
import logging
import os
import sys
from types import ModuleType

from explore.kernel.paths import explore_kernel_candidate_dirs

logger = logging.getLogger(__name__)

EXPLORE_KERNEL_EXTENSION_MODULE = "lingtu_explore_kernel"
_NATIVE_SUFFIXES = (".so", ".pyd", ".dll", ".dylib")
_DLL_DIRECTORY_HANDLES: list[object] = []
_DLL_DIRECTORY_PATHS: set[str] = set()
PRODUCTION_EXPLORE_KERNEL_SYMBOLS = (
    "Grid2D",
    "Pose2D",
    "ExploreInput",
    "ExploreCandidate",
    "ExploreDecision",
    "TarePolicyConfig",
    "TarePolicy",
)


def _has_explore_kernel_binary(directory: str) -> bool:
    return any(
        name.startswith(EXPLORE_KERNEL_EXTENSION_MODULE) and name.endswith(_NATIVE_SUFFIXES)
        for name in os.listdir(directory)
    )


def _add_windows_dll_search_dirs(candidate_dirs: list[str]) -> None:
    if os.name != "nt" or not hasattr(os, "add_dll_directory"):
        return

    dirs: list[str] = list(candidate_dirs)
    for entry in os.environ.get("PATH", "").split(os.pathsep):
        lower = entry.lower()
        if any(token in lower for token in ("msys", "mingw", "clang", "conda")):
            dirs.append(entry)

    for directory in dirs:
        if not directory or directory in _DLL_DIRECTORY_PATHS:
            continue
        if not os.path.isdir(directory):
            continue
        try:
            _DLL_DIRECTORY_HANDLES.append(os.add_dll_directory(directory))
            _DLL_DIRECTORY_PATHS.add(directory)
        except OSError:
            logger.debug("explore kernel loader: failed to add DLL directory %s", directory)


def ensure_explore_kernel_on_path(anchor: str | os.PathLike[str] | None = None) -> None:
    """Add a local native-kernel build directory to ``sys.path`` when present."""

    candidate_dirs = explore_kernel_candidate_dirs(anchor)
    _add_windows_dll_search_dirs(candidate_dirs)

    for directory in reversed(candidate_dirs):
        if not os.path.isdir(directory):
            continue
        if _has_explore_kernel_binary(directory) and directory not in sys.path:
            sys.path.insert(0, directory)
            logger.debug("explore kernel loader: added %s to sys.path", directory)


def try_import_explore_kernel(
    required_symbols: tuple[str, ...] = (),
    *,
    anchor: str | os.PathLike[str] | None = None,
) -> ModuleType | None:
    """Import ``lingtu_explore_kernel`` only when it exposes required symbols."""

    def missing_symbols(module: ModuleType) -> list[str]:
        return [name for name in required_symbols if not hasattr(module, name)]

    ensure_explore_kernel_on_path(anchor)
    for retry in range(2):
        try:
            module = importlib.import_module(EXPLORE_KERNEL_EXTENSION_MODULE)
            missing = missing_symbols(module)
            if not missing:
                return module

            origin = getattr(module, "__file__", "<unknown>")
            logger.warning(
                "%s at %s is missing required symbols: %s",
                EXPLORE_KERNEL_EXTENSION_MODULE,
                origin,
                ", ".join(missing),
            )
            return None
        except ImportError:
            if retry == 0:
                sys.modules.pop(EXPLORE_KERNEL_EXTENSION_MODULE, None)
                continue
            return None
    return None


def explore_kernel_available(
    required_symbols: tuple[str, ...] = PRODUCTION_EXPLORE_KERNEL_SYMBOLS,
    *,
    anchor: str | os.PathLike[str] | None = None,
) -> bool:
    """Return true only when a compatible native exploration kernel imports."""

    return try_import_explore_kernel(required_symbols, anchor=anchor) is not None


def require_explore_kernel(
    required_symbols: tuple[str, ...] = PRODUCTION_EXPLORE_KERNEL_SYMBOLS,
    *,
    context: str = "production exploration",
    anchor: str | os.PathLike[str] | None = None,
) -> ModuleType:
    """Load ``lingtu_explore_kernel`` or raise a startup-blocking error."""

    module = try_import_explore_kernel(required_symbols, anchor=anchor)
    if module is not None:
        return module

    from explore.kernel.paths import explore_kernel_build_hint

    candidates = "\n".join(f"  - {path}" for path in explore_kernel_candidate_dirs(anchor))
    raise RuntimeError(
        f"{EXPLORE_KERNEL_EXTENSION_MODULE} is required for {context}, "
        "but no compatible native kernel was found.\n"
        f"Checked:\n{candidates}\n"
        f"{explore_kernel_build_hint()}"
    )
