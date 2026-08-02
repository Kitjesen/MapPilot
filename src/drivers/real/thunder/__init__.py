"""Thunder quadruped driver package."""

from __future__ import annotations

import importlib
from typing import Any

__all__ = ["ThunderDriver"]


def __getattr__(name: str) -> Any:
    if name == "ThunderDriver":
        return importlib.import_module(f"{__name__}.han_dog_module").ThunderDriver
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
