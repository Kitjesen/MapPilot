"""LingTu exploration package.

This package owns TARE-style exploration goal generation and adapters for
endpoint-owned external TARE runtimes.

Exploration produces goals and candidate paths. Navigation still owns global
planning, local planning, path following, and command output.
"""

from .base import ExploreModule
from .kernel import (
    explore_kernel_available,
    explore_kernel_build_hint,
    require_explore_kernel,
    try_import_explore_kernel,
)

__all__ = [
    "ExploreModule",
    "explore_kernel_available",
    "explore_kernel_build_hint",
    "require_explore_kernel",
    "try_import_explore_kernel",
]
