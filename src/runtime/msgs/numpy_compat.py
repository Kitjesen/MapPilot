"""Lazy NumPy access for message modules.

Control-plane code often needs message classes for type annotations and simple
scalar payloads. Importing NumPy at module load time makes those paths depend
on the local numerical runtime even when no arrays are used.
"""

from __future__ import annotations

import importlib
import sys
from functools import lru_cache
from typing import Any


class LazyNumpy:
    """Import NumPy only when an array operation is actually used."""

    def __getattr__(self, name: str) -> Any:
        if not numpy_import_is_safe():
            raise ImportError("NumPy import is unsafe in this host interpreter")
        module = importlib.import_module("numpy")
        value = getattr(module, name)
        setattr(self, name, value)
        return value


np = LazyNumpy()


def is_numpy_array(value: Any) -> bool:
    """Best-effort ndarray check that does not import NumPy."""

    module = sys.modules.get("numpy")
    ndarray = getattr(module, "ndarray", None) if module is not None else None
    if ndarray is not None:
        return isinstance(value, ndarray)
    cls = value.__class__
    return cls.__module__.startswith("numpy") and hasattr(value, "shape") and hasattr(value, "dtype")


@lru_cache(maxsize=1)
def numpy_import_is_safe() -> bool:
    """Return whether importing NumPy is safe in this host interpreter.

    The probe runs in-process: a subprocess probe is unreliable on Windows
    cold starts, where interpreter startup plus NumPy init can exceed any
    reasonable timeout and wrongly report NumPy as unavailable.
    """

    if "numpy" in sys.modules:
        return True

    try:
        importlib.import_module("numpy")
    except Exception:
        # Drop any partially initialised module left behind by a failed import.
        sys.modules.pop("numpy", None)
        return False
    return True
