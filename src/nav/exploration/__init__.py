"""Compatibility imports for the relocated exploration package.

New code should import from ``explore``. This package remains so older imports
do not break while navigation/exploration ownership is being cleaned up.
"""

from explore import TraversableFrontierModule, WavefrontFrontierExplorer

__all__ = [
    "TraversableFrontierModule",
    "WavefrontFrontierExplorer",
]
