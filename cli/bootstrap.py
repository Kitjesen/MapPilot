"""sys.path setup for LingTu when running from repo root (not via pip install)."""

from __future__ import annotations

import sys
from pathlib import Path


def init(project_root: Path) -> None:
    root = str(project_root.resolve())
    src = str(project_root / "src")
    if src not in sys.path:
        sys.path.insert(0, src)
    if root not in sys.path:
        sys.path.insert(0, root)
