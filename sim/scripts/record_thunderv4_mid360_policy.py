#!/usr/bin/env python3
"""Compatibility wrapper for sim/scripts/mujoco/record_thunderv4_mid360_policy.py."""

import sys

from sim.scripts.mujoco import record_thunderv4_mid360_policy as _impl

if __name__ == "__main__":
    raise SystemExit(_impl.main())
sys.modules[__name__] = _impl
