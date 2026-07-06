#!/usr/bin/env python3
"""Compatibility wrapper for sim/scripts/mujoco/native_dds_sensors.py."""

import sys

from sim.scripts.mujoco import native_dds_sensors as _impl

if __name__ == "__main__":
    raise SystemExit(_impl.main())
sys.modules[__name__] = _impl
