"""Run the Thunder typed DDS endpoint service from the deployment tree."""
# ruff: noqa: E402,I001

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from runtime.adapters.dds.endpoint_runner import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
