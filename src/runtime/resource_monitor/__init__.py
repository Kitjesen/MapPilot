"""runtime.resource_monitor — Per-process CPU and memory tracking.

Usage::

    from runtime.resource_monitor import ResourceMonitor

    monitor = ResourceMonitor(poll_interval=5.0)
    monitor.register("worker-0", pid=12345)
    monitor.start()
    print(monitor.summary())
    monitor.stop()
"""

from .monitor import ResourceMonitor

__all__ = ["ResourceMonitor"]
