"""Path helpers for the navigation L5 native extension.

The public Python loader remains under ``src/nav/kernel``. Native C++ source
and CMake ownership live under ``src/nav/cpp``; this module keeps build-layout
details out of runtime callers.
"""

from __future__ import annotations

from pathlib import Path


def repo_root_for_nav_kernel(anchor: str | Path | None = None) -> Path:
    """Return the repository root used to resolve nav kernel build artifacts."""
    here = Path(anchor if anchor is not None else __file__).resolve()
    start = here.parent if here.is_file() else here

    for parent in (start, *start.parents):
        if (parent / "lingtu.py").exists() or (
            (parent / "pyproject.toml").exists()
            and (parent / "AGENTS.md").exists()
        ):
            return parent

    if here.is_file():
        fallback_chain = (here.parent, *here.parent.parents)
        if len(fallback_chain) > 2:
            return fallback_chain[2]

    return start


def nav_kernel_candidate_dirs(anchor: str | Path | None = None) -> list[str]:
    """Return candidate directories for the ``lingtu_nav_kernel`` native module.

    Canonical source-tree builds come first, followed by the build-script
    output, the release artifact copied under ``src/``, and install output.
    """
    repo = repo_root_for_nav_kernel(anchor)
    return [
        str(repo / "src" / "nav" / "cpp" / "build_nb_win"),
        str(repo / "src" / "nav" / "cpp" / "build_nb"),
        str(repo / "build" / "nav_kernel"),
        str(repo / "src"),
        str(repo / "install" / "nav_kernel" / "lib"),
    ]


def nav_kernel_build_hint() -> str:
    """Return a human-readable hint for building ``lingtu_nav_kernel``."""
    return (
        "Run:  bash scripts/build/build_nav_kernel.sh\n"
        "      (needs cmake, python3-dev, pip install nanobind)\n"
        "Windows local builds may use src/nav/cpp/build_nb_win."
    )
