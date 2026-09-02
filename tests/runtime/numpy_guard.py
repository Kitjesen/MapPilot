from __future__ import annotations

import importlib
import subprocess
import sys
from functools import lru_cache
from types import ModuleType

import pytest

NUMPY_UNSAFE_REASON = "NumPy import is unsafe in this host Python"


@lru_cache(maxsize=1)
def numpy_import_is_safe() -> bool:
    try:
        completed = subprocess.run(
            [sys.executable, "-c", "import numpy"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=10,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return completed.returncode == 0


def import_numpy_or_skip() -> ModuleType:
    if not numpy_import_is_safe():
        pytest.skip(NUMPY_UNSAFE_REASON, allow_module_level=True)
    return importlib.import_module("numpy")


def numpy_safe_skip_mark() -> pytest.MarkDecorator:
    return pytest.mark.skipif(
        not numpy_import_is_safe(),
        reason=NUMPY_UNSAFE_REASON,
    )
