"""Test path setup for the package-facing LingTu API tests."""

from __future__ import annotations

import sys
from pathlib import Path

_repo = Path(__file__).resolve().parents[3]
_src = _repo / "src"

for _path in (str(_repo), str(_src)):
    if _path not in sys.path:
        sys.path.insert(0, _path)
