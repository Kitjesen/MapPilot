"""Re-export shim — hybrid_planner moved to research/perception_experimental/.

This module is a thin re-export for backward compatibility. The actual
implementation lives at ``research/perception_experimental/hybrid_planner.py``.
Planning code does not belong in the perception package; see Phase 1 cleanup.
"""

import importlib
import sys
from pathlib import Path

# Add project root to sys.path so research/ is importable
_project_root = Path(__file__).resolve().parents[3]
_research_dir = str(_project_root / "research")
if _research_dir not in sys.path:
    sys.path.insert(0, _research_dir)

from perception_experimental.hybrid_planner import *
from perception_experimental.hybrid_planner import (
    HybridPath,
    HybridPlanner,
    PathSegment,
    PlannerComparison,
    _astar_on_grid,
    _downsample_cells,
    _find_nearest_free,
    _g2w,
    _w2g,
    compare_planners,
)
