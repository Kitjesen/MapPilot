"""Path helpers for the exploration L5 algorithm kernel.

The C++ source lives under ``src/explore/cpp``. This module owns the Python
side build/install path calculation so runtime loaders do not encode build
layout details directly.
"""

from __future__ import annotations

from pathlib import Path


def repo_root_for_explore_kernel(anchor: str | Path | None = None) -> Path:
    """Return the repository root used to resolve explore kernel build artifacts."""
    here = Path(anchor if anchor is not None else __file__).resolve()
    start = here.parent if here.is_file() else here

    for parent in (start, *start.parents):
        if (parent / "pyproject.toml").exists() and (parent / "AGENTS.md").exists():
            return parent

    if here.is_file():
        fallback_chain = (here.parent, *here.parent.parents)
        if len(fallback_chain) > 2:
            return fallback_chain[2]

    return start


def explore_kernel_candidate_dirs(anchor: str | Path | None = None) -> list[str]:
    """Return candidate directories for the ``lingtu_explore_kernel`` module.

    Priority order is Windows local build, Linux local build, build-script copy
    under ``src/``, then colcon install output.
    """
    repo = repo_root_for_explore_kernel(anchor)
    return [
        str(repo / "src" / "explore" / "cpp" / "build_nb_win"),
        str(repo / "src" / "explore" / "cpp" / "build_nb"),
        str(repo / "src"),
        str(repo / "install" / "explore_kernel" / "lib"),
    ]


def explore_kernel_build_hint() -> str:
    """Return a human-readable hint for building ``lingtu_explore_kernel``."""
    return (
        "Run:  bash scripts/build/build_explore_kernel.sh\n"
        "      (needs cmake, python3-dev, pip install nanobind)\n"
        "Windows local builds may use src/explore/cpp/build_nb_win."
    )
