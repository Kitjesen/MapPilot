"""conftest.py for src/runtime/tests/ -- add semantic package paths to sys.path."""

import asyncio
import os
import sys
import warnings
from inspect import signature
from pathlib import Path

_repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
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

# Integration harnesses run module-level setup at import time.
# Both files now expose proper def test_*() functions and guard sys.exit()
# in `if __name__ == "__main__":`, so pytest can collect them safely.


def _pytest_ignore_collect_impl(candidate):
    try:
        from runtime.tests.numpy_guard import numpy_import_is_safe
    except Exception:
        return False

    candidate = Path(str(candidate))
    if candidate.suffix != ".py" or candidate.name == "conftest.py":
        return False
    if not str(candidate).startswith(os.path.dirname(__file__)):
        return False
    if numpy_import_is_safe():
        return False
    try:
        source = candidate.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return False
    return "import numpy as np" in source or "from numpy" in source


try:
    from _pytest import hookspec as _pytest_hookspec

    _ignore_collect_params = signature(_pytest_hookspec.pytest_ignore_collect).parameters
except Exception:
    _ignore_collect_params = {}

if "collection_path" in _ignore_collect_params:

    def pytest_ignore_collect(collection_path, config):
        return _pytest_ignore_collect_impl(collection_path)

else:

    def pytest_ignore_collect(path, config):
        return _pytest_ignore_collect_impl(path)


def pytest_sessionfinish(session, exitstatus):
    """Release shared ROS2 resources created by mixed module-level smoke tests."""
    try:
        from runtime.adapters.ros2.context import shutdown_shared_executor

        shutdown_shared_executor()
    except Exception:
        pass

    try:
        import rclpy

        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
