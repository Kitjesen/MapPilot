"""core.resource_monitor.monitor — Per-process CPU and memory tracking.

Uses ``psutil`` for polling. Silently disabled if psutil is not installed.

Usage::

    from core.resource_monitor import ResourceMonitor

    monitor = ResourceMonitor(poll_interval=5.0)
    monitor.register("worker-0", pid=12345)
    monitor.register("worker-1", pid=12346)
    monitor.start()

    # ... later ...
    print(monitor.summary())
    # worker-0(pid=12345): cpu=3.2%  rss=128.4MB
    # worker-1(pid=12346): cpu=1.0%  rss=96.2MB

    stats = monitor.stats()
    # {'worker-0': {'pid': 12345, 'cpu_pct': 3.2, 'rss_mb': 128.4, ...}, ...}

    monitor.stop()
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

logger = logging.getLogger(__name__)


class ResourceMonitor:
    """Tracks CPU and memory for a set of registered processes.

    Thread-safe background polling. Requires ``psutil`` — silently
    disabled (stats() returns empty dict) if not installed.
    """

    def __init__(self, poll_interval: float = 5.0) -> None:
        self._poll_interval = poll_interval
        self._pids: dict[str, int] = {}        # label → PID
        self._stats: dict[str, Any] = {}       # label → stat dict
        self._thread: threading.Thread | None = None
        self._running = False
        self._lock = threading.Lock()

    # -- Registration ---------------------------------------------------------

    def register(self, label: str, pid: int) -> None:
        """Register a process for monitoring.

        Args:
            label: Human-readable name (e.g. ``"worker-0"``).
            pid:   OS process ID.
        """
        with self._lock:
            self._pids[label] = pid

    def unregister(self, label: str) -> None:
        """Stop monitoring *label*. No-op if not registered."""
        with self._lock:
            self._pids.pop(label, None)
            self._stats.pop(label, None)

    # -- Lifecycle ------------------------------------------------------------

    def start(self) -> None:
        """Start background polling thread. Idempotent."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._poll_loop, daemon=True, name="resource-monitor"
        )
        self._thread.start()
        logger.debug("ResourceMonitor started (interval=%.1fs)", self._poll_interval)

    def stop(self) -> None:
        """Signal the polling thread to stop. Non-blocking."""
        self._running = False

    # -- Query ----------------------------------------------------------------

    def stats(self) -> dict[str, Any]:
        """Return a snapshot of current stats.

        Each entry is a dict with keys:
        - ``pid``      — int
        - ``cpu_pct``  — float (% CPU since last poll)
        - ``rss_mb``   — float (resident memory in MB)
        - ``vms_mb``   — float (virtual memory in MB)
        - ``status``   — str (``"running"`` / ``"sleeping"`` / ``"zombie"`` / ...)
        - ``threads``  — int
        Or ``{"error": str}`` if the process could not be queried.
        """
        with self._lock:
            return dict(self._stats)

    def summary(self) -> str:
        """One-line text summary for all registered processes."""
        rows = []
        for label, info in sorted(self.stats().items()):
            if "error" in info:
                rows.append(f"{label}: ERR({info['error']})")
            else:
                rows.append(
                    f"{label}(pid={info['pid']}): "
                    f"cpu={info['cpu_pct']:.1f}%  "
                    f"rss={info['rss_mb']:.1f}MB"
                )
        return "  |  ".join(rows) if rows else "(no processes registered)"

    def is_running(self) -> bool:
        """True if the polling thread is active."""
        return self._running

    # -- Internal -------------------------------------------------------------

    def _poll_loop(self) -> None:
        try:
            import psutil  # type: ignore[import]
        except ImportError:
            logger.info(
                "psutil not installed — ResourceMonitor is a no-op. "
                "Install with: pip install psutil"
            )
            return

        procs: dict[str, Any] = {}
        self._sync_procs(psutil, procs)

        # Prime cpu_percent counters (first call always returns 0.0)
        for proc in procs.values():
            try:
                proc.cpu_percent(interval=None)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass

        while self._running:
            time.sleep(self._poll_interval)
            self._sync_procs(psutil, procs)
            new_stats: dict[str, Any] = {}
            for label, proc in list(procs.items()):
                try:
                    mem = proc.memory_info()
                    new_stats[label] = {
                        "pid":     proc.pid,
                        "cpu_pct": proc.cpu_percent(interval=None),
                        "rss_mb":  mem.rss / (1024 * 1024),
                        "vms_mb":  mem.vms / (1024 * 1024),
                        "status":  proc.status(),
                        "threads": proc.num_threads(),
                    }
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess) as exc:
                    new_stats[label] = {"error": str(exc)}
            with self._lock:
                self._stats = new_stats

    def _sync_procs(self, psutil_mod: Any, procs: dict[str, Any]) -> None:
        """Keep *procs* in sync with the current _pids registry."""
        with self._lock:
            current = dict(self._pids)

        for label, pid in current.items():
            existing = procs.get(label)
            if existing is None or existing.pid != pid:
                try:
                    procs[label] = psutil_mod.Process(pid)
                except (psutil_mod.NoSuchProcess, psutil_mod.AccessDenied):
                    procs.pop(label, None)

        for label in list(procs):
            if label not in current:
                del procs[label]
