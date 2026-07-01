"""Lazy NumPy access for message modules.

Control-plane code often needs message classes for type annotations and simple
scalar payloads. Importing NumPy at module load time makes those paths depend
on the local numerical runtime even when no arrays are used.
"""

from __future__ import annotations

import importlib
import subprocess
import sys
from functools import lru_cache
from typing import Any


class LazyNumpy:
    """Import NumPy only when an array operation is actually used."""

    def __getattr__(self, name: str) -> Any:
        if not numpy_import_is_safe():
            raise ImportError("NumPy import is unsafe in this host interpreter")
        module = importlib.import_module("numpy")
        return getattr(module, name)


np = LazyNumpy()


def is_numpy_array(value: Any) -> bool:
    """Best-effort ndarray check that does not import NumPy."""

    module = sys.modules.get("numpy")
    ndarray = getattr(module, "ndarray", None) if module is not None else None
    if ndarray is not None:
        return isinstance(value, ndarray)
    cls = value.__class__
    return (
        cls.__module__.startswith("numpy")
        and hasattr(value, "shape")
        and hasattr(value, "dtype")
    )


@lru_cache(maxsize=1)
def numpy_import_is_safe() -> bool:
    """Return whether importing NumPy is safe in this host interpreter."""

    if "numpy" in sys.modules:
        return True

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
