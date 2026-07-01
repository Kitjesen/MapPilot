"""Optional ROS2 runtime shutdown hook."""

from __future__ import annotations

import sys

_COMPAT_MODULE = "runtime.adapters.ros2.context"


def shutdown_ros2_runtime() -> None:
    """Shutdown the shared ROS2 executor only when the compat layer was used."""

    compat_context = sys.modules.get(_COMPAT_MODULE)
    if compat_context is None:
        return

    shutdown_shared_executor = getattr(
        compat_context,
        "shutdown_shared_executor",
        None,
    )
    if shutdown_shared_executor is None:
        return
    shutdown_shared_executor()
