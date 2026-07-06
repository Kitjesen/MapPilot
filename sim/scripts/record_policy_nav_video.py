#!/usr/bin/env python3
"""Compatibility wrapper for sim/scripts/mujoco/record_policy_nav_video.py."""

import sys

from sim.scripts.mujoco import record_policy_nav_video as _impl

if __name__ == "__main__":
    raise SystemExit(_impl.main())
sys.modules[__name__] = _impl
