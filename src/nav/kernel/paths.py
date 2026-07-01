"""Path helpers for the navigation L5 algorithm kernel.

The C++ source lives under ``src/nav/kernel``. This module owns the Python
side build/install path calculation so runtime loaders do not encode build
layout details directly.
"""

from __future__ import annotations

from pathlib import Path


def repo_root_for_nav_kernel(anchor: str | Path | None = None) -> Path:
    """Return the repository root used to resolve nav kernel build artifacts."""
    here = Path(anchor if anchor is not None else __file__).resolve()
    start = here.parent if here.is_file() else here

    for parent in (start, *start.parents):
        if (parent / "lingtu.py").exists() or (parent / "src").is_dir():
            return parent

    if here.is_file():
        fallback_chain = (here.parent, *here.parent.parents)
        if len(fallback_chain) > 2:
            return fallback_chain[2]

    return start


def nav_kernel_candidate_dirs(anchor: str | Path | None = None) -> list[str]:
    """Return candidate directories for the ``lingtu_nav_kernel`` native module.

    Priority order is preserved from the legacy loader:
    Windows local build, Linux local build, build-script copy under ``src/``,
    then future colcon install output.
    """
    repo = repo_root_for_nav_kernel(anchor)
    return [
        str(repo / "src" / "nav" / "kernel" / "build_nb_win"),
        str(repo / "src" / "nav" / "kernel" / "build_nb"),
        str(repo / "src"),
        str(repo / "install" / "nav_kernel" / "lib"),
    ]


def nav_kernel_build_hint() -> str:
    """Return a human-readable hint for building ``lingtu_nav_kernel``."""
    return (
        "Run:  bash scripts/build/build_nav_kernel.sh\n"
        "      (needs cmake, python3-dev, pip install nanobind)\n"
        "Windows local builds may use src/nav/kernel/build_nb_win."
    )
