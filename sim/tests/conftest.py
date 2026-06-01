"""conftest.py for sim/tests/ -- add repo and src paths to sys.path."""

import asyncio
import os
import sys
import warnings

try:
    import rclpy  # noqa: F401

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

_repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_src = os.path.join(_repo, "src")

_paths = [
    _repo,
    _src,
    os.path.join(_src, "semantic", "planner"),
    os.path.join(_src, "semantic", "perception"),
]

for _p in _paths:
    if _p not in sys.path:
        sys.path.insert(0, _p)


class _CompatEventLoopPolicy(asyncio.DefaultEventLoopPolicy):
    """Keep legacy get_event_loop() tests working on newer Python versions."""

    def get_event_loop(self):
        try:
            with warnings.catch_warnings():
                warnings.filterwarnings(
                    "ignore",
                    message="There is no current event loop",
                    category=DeprecationWarning,
                )
                return super().get_event_loop()
        except RuntimeError:
            loop = self.new_event_loop()
            self.set_event_loop(loop)
            return loop


asyncio.set_event_loop_policy(_CompatEventLoopPolicy())
