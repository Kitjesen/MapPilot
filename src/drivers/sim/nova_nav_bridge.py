#!/usr/bin/env python3
"""Compatibility entrypoint for the NOVA ROS simulation bridge.

The implementation lives in :mod:`compat.ros2.nova_nav_bridge` so the
simulation driver package does not own ROS runtime code.
"""

from __future__ import annotations

import sys
from pathlib import Path

SRC_DIR = Path(__file__).resolve().parents[2]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from compat.ros2.nova_nav_bridge import *  # noqa: F401,F403,E402
from compat.ros2.nova_nav_bridge import main  # noqa: E402


if __name__ == "__main__":
    main()
