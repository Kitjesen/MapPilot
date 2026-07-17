#!/usr/bin/env python3
"""Compatibility wrapper for sim/scripts/mujoco/saved_map_tracking_gate.py."""

import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parents[1]

# Running this wrapper directly puts sim/scripts at sys.path[0]. That directory
# contains LingTu's sim.scripts.mujoco package, which shadows DeepMind's
# top-level mujoco package used by MuJoCoEngine.
sys.path = [p for p in sys.path if Path(p or ".").resolve() != SCRIPT_DIR]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.scripts.mujoco import saved_map_tracking_gate as _impl

if __name__ == "__main__":
    raise SystemExit(_impl.main())
sys.modules[__name__] = _impl
