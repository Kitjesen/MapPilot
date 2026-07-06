#!/usr/bin/env python3
"""Compatibility wrapper for sim/scripts/mujoco/native_pct_gate.py."""

import sys

from sim.scripts.mujoco import native_pct_gate as _impl

if __name__ == "__main__":
    raise SystemExit(_impl.main())
sys.modules[__name__] = _impl
