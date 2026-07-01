"""conftest.py for src/gateway/tests/ — add src/ to sys.path for runtime.* imports."""

import os
import sys

_repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_src = os.path.join(_repo, "src")

for _p in [_repo, _src]:
    if _p not in sys.path:
        sys.path.insert(0, _p)
