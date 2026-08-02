"""LingTu local Python facade.

Use ``Robot`` for high-level local control. Product assembly owns field
orchestration; Blueprint remains scoped to local and managed Host graphs.
"""

from .robot import Robot

__all__ = ["Robot"]
