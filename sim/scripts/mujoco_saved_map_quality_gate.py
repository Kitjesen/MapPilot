#!/usr/bin/env python3
"""Compatibility wrapper for sim/scripts/mujoco/saved_map_quality_gate.py."""

import sys

from sim.scripts.mujoco import saved_map_quality_gate as _impl

if __name__ == "__main__":
    raise SystemExit(_impl.main())
sys.modules[__name__] = _impl
