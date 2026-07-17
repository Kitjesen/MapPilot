#!/usr/bin/env python3
"""MuJoCo navigation gate: map, plan, track, and move the ThunderV4 policy dog.

This is the product-facing alias for the saved-map tracking gate. It keeps the
clear navigation name while reusing the maintained implementation.
"""

import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parents[1]

sys.path = [p for p in sys.path if Path(p or ".").resolve() != SCRIPT_DIR]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.scripts.mujoco import saved_map_tracking_gate as _impl

if __name__ == "__main__":
    raise SystemExit(_impl.main())
sys.modules[__name__] = _impl
