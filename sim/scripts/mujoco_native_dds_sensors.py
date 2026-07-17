#!/usr/bin/env python3
"""Compatibility wrapper for sim/scripts/mujoco/native_dds_sensors.py."""

import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parents[1]

# Running this wrapper directly puts sim/scripts at sys.path[0]. Keep the repo
# root importable while avoiding package shadowing under sim/scripts.
sys.path = [p for p in sys.path if Path(p or ".").resolve() != SCRIPT_DIR]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.scripts.mujoco import native_dds_sensors as _impl

if __name__ == "__main__":
    raise SystemExit(_impl.main())
sys.modules[__name__] = _impl
