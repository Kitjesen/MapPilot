#!/usr/bin/env python3
"""Compatibility wrapper for sim/scripts/mujoco/record_thunderv4_stair_showcase.py."""

import sys

from sim.scripts.mujoco import record_thunderv4_stair_showcase as _impl

if __name__ == "__main__":
    raise SystemExit(_impl.main())
sys.modules[__name__] = _impl
