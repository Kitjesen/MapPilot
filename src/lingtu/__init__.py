"""LingTu local Python facade.

Use ``Robot`` for high-level local control, or ``lingtu.runtime`` to resolve
and build Module-First systems directly.
"""

from .robot import Robot

__all__ = ["Robot"]
