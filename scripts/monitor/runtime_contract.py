"""Runtime contract values used by monitor scripts."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
for path in (ROOT, SRC):
    text = str(path)
    if text not in sys.path:
        sys.path.insert(0, text)

from runtime.runtime_interface import TOPICS


STATUS_TOPIC = TOPICS.semantic_status
