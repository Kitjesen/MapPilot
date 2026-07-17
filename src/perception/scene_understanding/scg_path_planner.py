"""Re-export shim — scg_path_planner moved to research/perception_experimental/.

This module is a thin re-export for backward compatibility. The actual
implementation lives at ``research/perception_experimental/scg_path_planner.py``.
Planning code does not belong in the perception package; see Phase 1 cleanup.
"""

import sys
from pathlib import Path

# Add project root to sys.path so research/ is importable
_project_root = Path(__file__).resolve().parents[3]
_research_dir = str(_project_root / "research")
if _research_dir not in sys.path:
    sys.path.insert(0, _research_dir)

from perception_experimental.scg_path_planner import *
from perception_experimental.scg_path_planner import (
    PathSegment,
    PathSmoother,
    SCGPath,
    SCGPathPlanner,
)
