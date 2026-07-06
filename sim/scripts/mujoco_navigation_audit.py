#!/usr/bin/env python3
"""Compatibility wrapper for sim/scripts/mujoco/navigation_audit.py."""

import sys

from sim.scripts.mujoco import navigation_audit as _impl

if __name__ == "__main__":
    raise SystemExit(_impl.main())
sys.modules[__name__] = _impl
