"""BlackBoxRecorder — ring-buffer crash recorder for the SLAM drift watchdog.

When the drift watchdog tears down a diverged SLAM stack, the only trace left
behind today is a single SSE event with `xy` and `v`. By the time an operator
notices the crash, the only artefact is "we restarted at 13:42 because xy=8e6".
That is not enough to attribute the divergence: was it a long static stretch,
a step-pattern IMU pulse, a GNSS jump that jerked the alignment lock, or a
LiDAR drop-out?

This recorder maintains short ring buffers of the streams that flow through
the gateway (odometry, slam_diag, gnss_fusion, map_odom_tf) and dumps them as
JSONL files when the watchdog trips. The dumps go to a configurable directory
under ~/data/slam/crashes/ and are GC'd to a fixed retention so they do not
bloat the disk.

The recorder is intentionally agnostic about ROS, drivers, or transport — any
caller can `record(channel, value)` from any thread and `dump(reason)` to
flush. JSON-encoding is best-effort: numpy arrays are converted via
``.tolist()`` and non-serialisable values fall back to ``repr()``.

Configuration via environment variables (read by ``BlackBoxRecorder.from_env``):

* ``LINGTU_BLACKBOX_ENABLED``       — "0" disables; default enabled.
* ``LINGTU_BLACKBOX_DIR``           — base directory; default ``~/data/slam/crashes``.
* ``LINGTU_BLACKBOX_MAX_PER_CHANNEL``— ring-buffer depth; default 600 (≈60 s @10 Hz).
* ``LINGTU_BLACKBOX_RETENTION``     — number of dump directories to keep; default 20.
"""

from __future__ import annotations

import json
import logging
import os
import socket
import threading
import time
from collections import defaultdict, deque
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


def _json_default(obj: Any) -> Any:
    """Best-effort fallback for objects ``json.dumps`` cannot encode natively."""
    # numpy is ubiquitous in this codebase but optional in test environments
    try:
        import numpy as np  # local import — avoid forcing numpy on consumers
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.integer,)):
            return int(obj)
        if isinstance(obj, (np.floating,)):
            return float(obj)
    except ImportError:
        pass
    if hasattr(obj, "__dict__"):
        return {"__type__": type(obj).__name__, "repr": repr(obj)}
    return repr(obj)


class BlackBoxRecorder:
    """Thread-safe per-channel ring buffer with on-demand crash dumps."""

    def __init__(
        self,
        base_dir: Path,
        max_per_channel: int = 600,
        retention: int = 20,
        enabled: bool = True,
    ) -> None:
        self.base_dir = Path(base_dir)
        self.max_per_channel = int(max_per_channel)
        self.retention = int(retention)
        self.enabled = bool(enabled)
        self._buffers: dict[str, deque[tuple[float, Any]]] = defaultdict(
            lambda: deque(maxlen=self.max_per_channel)
        )
        self._lock = threading.Lock()
        if self.enabled:
            try:
                self.base_dir.mkdir(parents=True, exist_ok=True)
            except OSError as e:
                # Disable rather than crash the gateway if the path is unwritable.
                logger.warning(
                    "BlackBoxRecorder: cannot create %s (%s) — disabling",
                    self.base_dir, e,
                )
                self.enabled = False

    @classmethod
    def from_env(cls) -> BlackBoxRecorder:
        enabled = os.environ.get("LINGTU_BLACKBOX_ENABLED", "1") != "0"
        base = Path(os.environ.get(
            "LINGTU_BLACKBOX_DIR",
            str(Path.home() / "data" / "slam" / "crashes"),
        ))
        max_per_ch = int(os.environ.get("LINGTU_BLACKBOX_MAX_PER_CHANNEL", "600"))
        retention = int(os.environ.get("LINGTU_BLACKBOX_RETENTION", "20"))
        return cls(base, max_per_channel=max_per_ch, retention=retention, enabled=enabled)

    # ── recording ─────────────────────────────────────────────────────────

    def record(self, channel: str, value: Any) -> None:
        """Append ``value`` to the ``channel`` ring buffer. Thread-safe.

        Cheap path when disabled — early return to keep callback hot loops fast.
        """
        if not self.enabled:
            return
        ts = time.time()
        with self._lock:
            self._buffers[channel].append((ts, value))

    # ── dumping ───────────────────────────────────────────────────────────

    def dump(self, reason: str, metadata: dict | None = None) -> Path | None:
        """Flush all ring buffers to ``base_dir/drift_<ts>/`` as JSONL files.

        Returns the dump directory path so the caller can attach it to whatever
        crash event it emits (so operators can ``cd`` straight to the artefact).
        Returns ``None`` if disabled or on I/O failure.
        """
        if not self.enabled:
            return None
        ts = time.time()
        out = self.base_dir / f"drift_{int(ts)}"
        try:
            out.mkdir(parents=True, exist_ok=True)
            meta = {
                "timestamp": ts,
                "iso_timestamp": time.strftime("%Y-%m-%dT%H:%M:%S",
                                                time.gmtime(ts)) + "Z",
                "reason": reason,
                "hostname": socket.gethostname(),
            }
            if metadata:
                meta.update(metadata)
            (out / "metadata.json").write_text(
                json.dumps(meta, indent=2, default=_json_default))

            with self._lock:
                snapshots = {name: list(buf) for name, buf in self._buffers.items()}

            for name, buf in snapshots.items():
                lines = [
                    json.dumps({"t": t, "v": v}, default=_json_default)
                    for t, v in buf
                ]
                (out / f"{name}.jsonl").write_text("\n".join(lines) + "\n" if lines else "")
        except OSError as e:
            logger.error("BlackBoxRecorder: dump failed: %s", e)
            return None

        self._gc()
        logger.info("BlackBoxRecorder: dumped %d channels to %s",
                    len(snapshots), out)
        return out

    def _gc(self) -> None:
        """Keep only the most recent ``retention`` dump directories."""
        try:
            dirs = sorted(
                (p for p in self.base_dir.iterdir()
                 if p.is_dir() and p.name.startswith("drift_")),
                key=lambda p: p.stat().st_mtime,
                reverse=True,
            )
        except OSError:
            return
        for stale in dirs[self.retention:]:
            try:
                for f in stale.iterdir():
                    f.unlink()
                stale.rmdir()
            except OSError as e:
                logger.debug("BlackBoxRecorder: gc %s failed: %s", stale, e)

    # ── introspection (testing) ───────────────────────────────────────────

    def channels(self) -> list[str]:
        with self._lock:
            return list(self._buffers.keys())

    def buffer_size(self, channel: str) -> int:
        with self._lock:
            return len(self._buffers.get(channel, ()))
