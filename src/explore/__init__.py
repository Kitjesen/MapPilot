"""LingTu exploration package.

This package owns exploration goal generation:

- wavefront frontier exploration;
- traversability-aware frontier previews;
- LingTu TARE-style viewpoint selection;
- adapters for endpoint-owned external TARE runtimes.

Exploration produces goals and candidate paths. Navigation still owns global
planning, local planning, path following, and command output.
"""

from .base import ExploreModule
from .frontier import WavefrontFrontierExplorer
from .kernel import (
    explore_kernel_available,
    explore_kernel_build_hint,
    require_explore_kernel,
    try_import_explore_kernel,
)
from .traversable_frontier import TraversableFrontierModule

__all__ = [
    "ExploreModule",
    "TraversableFrontierModule",
    "WavefrontFrontierExplorer",
    "explore_kernel_available",
    "explore_kernel_build_hint",
    "require_explore_kernel",
    "try_import_explore_kernel",
]
