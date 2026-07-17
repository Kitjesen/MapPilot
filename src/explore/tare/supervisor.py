"""ExplorationSupervisorModule 鈥?cross-process watchdog for TARE exploration.

Motivation
----------
`TAREExplorerModule` consumes TARE output from an endpoint-owned DDS runtime.
If that endpoint silently hangs (alive but not publishing waypoints) or the DDS
bridge loses connectivity, the rest of the stack has no way to know.
Navigation keeps waiting for a `goal_pose` that never arrives.

This supervisor consolidates TARE's `tare_stats` diagnostic stream into a
concise `supervisor_state` dict consumed by Gateway SSE / Dashboard widgets
/ operators, and fires a one-shot `exploration_ready` signal the first time
TARE reports healthy.

Supervisor modes
----------------
- ``uninit``    鈥?no tare_stats received yet
- ``starting``  鈥?TARE alive, no waypoints yet (warm-up)
- ``healthy``   鈥?waypoints flowing within timeout
- ``degraded``  鈥?waypoint silence > warn_timeout_s (TARE may be stuck)
- ``fallback``  鈥?waypoint silence > fallback_timeout_s (operator should
                   switch to wavefront backend)
- ``finished``  鈥?TARE reports exploration complete

The supervisor does NOT auto-swap backends at runtime 鈥?that requires
Blueprint rebuild and a Navigation handoff that's best done via a
controlled restart. Instead, it publishes `fallback_requested=True` so the
operator (or an external orchestrator) can decide.
"""

from __future__ import annotations

import logging
import threading
import time as _time
from typing import Any

from runtime.module import Module, skill
from runtime.registry import register
from runtime.stream import In, Out

logger = logging.getLogger(__name__)

# Supervisor mode constants
MODE_UNINIT = "uninit"
MODE_STARTING = "starting"
MODE_HEALTHY = "healthy"
MODE_DEGRADED = "degraded"
MODE_FALLBACK = "fallback"
MODE_FINISHED = "finished"


@register("exploration", "supervisor", description="TARE exploration cross-process watchdog")
class ExplorationSupervisorModule(Module, layer=5):
    """Aggregate TARE tare_stats into a concise supervisor_state stream.

    Consumers (Gateway SSE, Dashboard widgets, operator CLI) subscribe to
    ``supervisor_state`` instead of parsing raw tare_stats 鈥?so they see a
    single mode enum and reason string.

    The one-shot ``exploration_ready`` signal fires the first time TARE is
    healthy; modules that should wait for exploration to warm up can gate
    on this.
    """

    # Inputs
    tare_stats: In[dict]  # from TAREExplorerModule
    # Outputs
    supervisor_state: Out[dict]  # consolidated watchdog state
    exploration_ready: Out[bool]  # one-shot "TARE is ready" flag

    def __init__(
        self,
        warn_timeout_s: float = 20.0,
        fallback_timeout_s: float = 60.0,
        poll_hz: float = 1.0,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._warn_timeout_s = float(warn_timeout_s)
        self._fallback_timeout_s = float(fallback_timeout_s)
        self._interval = 1.0 / max(float(poll_hz), 0.5)

        self._last_stats: dict | None = None
        self._last_stats_ts: float = 0.0
        self._mode: str = MODE_UNINIT
        self._reason: str = "no tare_stats received yet"
        self._ready_fired: bool = False
        self._degraded_ticks: int = 0
        self._fallback_requested: bool = False
        self._shutdown = threading.Event()
        self._watchdog: threading.Thread | None = None

    # 鈹€鈹€ lifecycle 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def setup(self) -> None:
        self.tare_stats.subscribe(self._on_tare_stats)

    def start(self) -> None:
        super().start()
        self._shutdown.clear()
        self._watchdog = threading.Thread(target=self._loop, daemon=True, name="exploration-supervisor")
        self._watchdog.start()

    def stop(self) -> None:
        self._shutdown.set()
        if self._watchdog and self._watchdog.is_alive():
            self._watchdog.join(timeout=2.0)
        super().stop()

    # 鈹€鈹€ internals 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def _on_tare_stats(self, stats: dict) -> None:
        if not isinstance(stats, dict):
            return
        self._last_stats = stats
        self._last_stats_ts = _time.time()
        if not self._ready_fired and stats.get("alive") and stats.get("healthy"):
            self._ready_fired = True
            self.exploration_ready.publish(True)
            logger.info("ExplorationSupervisor: TARE is ready (first healthy tare_stats)")

    def _loop(self) -> None:
        while not self._shutdown.wait(self._interval):
            self._evaluate_and_publish()

    def _evaluate_and_publish(self) -> None:
        mode, reason, wp_age = self._evaluate()
        self._mode = mode
        self._reason = reason
        self.supervisor_state.publish(
            {
                "configured_backend": (
                    self._last_stats.get("configured_backend", "tare") if self._last_stats else "tare"
                ),
                "backend": (self._last_stats.get("backend", "tare") if self._last_stats else "tare"),
                "degraded": mode in (MODE_DEGRADED, MODE_FALLBACK),
                "degraded_reason": (reason if mode in (MODE_DEGRADED, MODE_FALLBACK) else ""),
                "mode": mode,
                "reason": reason,
                "waypoint_age_s": wp_age,
                "waypoint_count": (self._last_stats.get("waypoint_count", 0) if self._last_stats else 0),
                "last_runtime_ms": (self._last_stats.get("last_runtime_ms", 0.0) if self._last_stats else 0.0),
                "fallback_requested": self._fallback_requested,
                "degraded_ticks": self._degraded_ticks,
                "ready_fired": self._ready_fired,
                "ts": _time.time(),
            }
        )

    def _evaluate(self) -> tuple[str, str, float | None]:
        """Classify current state and return (mode, reason, wp_age_seconds)."""
        stats = self._last_stats
        if stats is None:
            return MODE_UNINIT, "no tare_stats received yet", None

        if stats.get("finished"):
            return MODE_FINISHED, "TARE reports exploration complete", 0.0

        if not stats.get("alive"):
            return MODE_UNINIT, "TARE bridge not alive", None

        wp_age = stats.get("waypoint_age_s")
        try:
            wp_age_f = float(wp_age) if wp_age is not None else float("inf")
        except (TypeError, ValueError):
            wp_age_f = float("inf")
        # NaN and infinity both treated as "no waypoint yet"
        no_waypoint = (wp_age_f != wp_age_f) or wp_age_f > 1e9

        if no_waypoint:
            return MODE_STARTING, "waiting for first waypoint", None

        if wp_age_f > self._fallback_timeout_s:
            if not self._fallback_requested:
                logger.error(
                    "ExplorationSupervisor: fallback requested 鈥?no waypoint "
                    "for %.1fs (>%.0fs threshold). Operator should switch to "
                    "wavefront backend or investigate TARE.",
                    wp_age_f,
                    self._fallback_timeout_s,
                )
            self._fallback_requested = True
            return (
                MODE_FALLBACK,
                f"no waypoint for {wp_age_f:.1f}s (>{self._fallback_timeout_s:.0f}s)",
                wp_age_f,
            )

        if wp_age_f > self._warn_timeout_s:
            self._degraded_ticks += 1
            return (
                MODE_DEGRADED,
                f"no waypoint for {wp_age_f:.1f}s (>{self._warn_timeout_s:.0f}s) 鈥?TARE may be stuck",
                wp_age_f,
            )

        if stats.get("healthy"):
            # Recovered 鈥?clear the fallback flag so a future stall re-raises.
            if self._fallback_requested:
                logger.info(
                    "ExplorationSupervisor: TARE recovered (wp_age=%.1fs), clearing fallback_requested",
                    wp_age_f,
                )
            self._fallback_requested = False
            return MODE_HEALTHY, "ok", wp_age_f

        # alive + not finished + has waypoint + not healthy yet
        return MODE_STARTING, "warm-up", wp_age_f

    # 鈹€鈹€ skills 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    @skill
    def get_exploration_supervisor(self) -> str:
        """Return the latest exploration supervisor state as JSON."""
        import json

        stats_age = _time.time() - self._last_stats_ts if self._last_stats_ts > 0 else None
        return json.dumps(
            {
                "configured_backend": (
                    self._last_stats.get("configured_backend", "tare") if self._last_stats else "tare"
                ),
                "backend": (self._last_stats.get("backend", "tare") if self._last_stats else "tare"),
                "degraded": self._mode in (MODE_DEGRADED, MODE_FALLBACK),
                "degraded_reason": (self._reason if self._mode in (MODE_DEGRADED, MODE_FALLBACK) else ""),
                "mode": self._mode,
                "reason": self._reason,
                "ready_fired": self._ready_fired,
                "fallback_requested": self._fallback_requested,
                "degraded_ticks": self._degraded_ticks,
                "last_stats_age_s": stats_age,
                "last_stats": self._last_stats,
            }
        )

    @skill
    def clear_exploration_fallback(self) -> str:
        """Reset the fallback_requested flag (use after operator handled)."""
        import json

        self._fallback_requested = False
        self._degraded_ticks = 0
        return json.dumps({"status": "cleared"})
