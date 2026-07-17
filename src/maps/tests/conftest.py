"""Test path setup for the maps domain."""

from __future__ import annotations

import os
import sys

_repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_src = os.path.join(_repo, "src")

for _path in (_repo, _src):
    if _path not in sys.path:
        sys.path.insert(0, _path)
