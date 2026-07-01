#!/usr/bin/env python3
"""LingTu single entry point for CLI profiles and the interactive REPL.

Run::

    python lingtu.py              # interactive profile picker
    python lingtu.py nav          # navigation stack
    python lingtu.py --list       # list profiles

When installed via pip, the ``lingtu`` console script calls ``lingtu_cli``.
"""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
_SRC_ROOT = _ROOT / "src"
_PACKAGE_ROOT = _ROOT / "src" / "lingtu"

if _SRC_ROOT.is_dir() and str(_SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(_SRC_ROOT))

if _PACKAGE_ROOT.is_dir():
    __path__ = [str(_PACKAGE_ROOT)]

__all__ = ["Robot"]


def __getattr__(name: str):
    """Expose package-facing symbols when this script shadows ``src/lingtu``."""

    if name == "Robot":
        from lingtu.robot import Robot

        return Robot
    raise AttributeError(f"module 'lingtu' has no attribute {name!r}")


def main() -> None:
    """Run the LingTu CLI entry point."""

    from cli.bootstrap import init
    from cli.paths import set_project_root

    set_project_root(_ROOT)
    init(_ROOT)

    from cli.main import main as cli_main

    cli_main()


if __name__ == "__main__":
    main()
