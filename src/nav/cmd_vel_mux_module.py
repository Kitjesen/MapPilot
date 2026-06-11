"""CmdVelMux — priority-based velocity command arbitration.

Multiple sources (teleop, visual servo, path follower, recovery) can publish
cmd_vel.  CmdVelMux selects the highest-priority *active* source and forwards
only that source's commands to the driver.

A source is "active" if it published within its timeout window.  When the
active source goes idle, the mux falls through to the next lower-priority
source that has recently published.

Priority (higher = wins):
    teleop          100   — human joystick override
    visual_servo     80   — close-range PD tracking
    recovery         60   — NavigationModule stuck recovery
    path_follower    40   — normal autonomy

All incoming cmd_vel arrive on separate In ports so the mux knows *which*
source sent the message.  One Out[Twist] port goes to the driver.

The mux also publishes `active_source: Out[str]` so other modules (Gateway,
SafetyRing, etc.) can display who controls the robot.
"""

from __future__ import annotations

import logging
import math
import threading
import time

from core.module import Module
from core.msgs.geometry import Twist
from core.registry import register
from core.stream import In, Out

logger = logging.getLogger(__name__)

# Source definitions: name → default priority
_DEFAULT_PRIORITIES: dict[str, int] = {
    "teleop": 100,
    "visual_servo": 80,
    "recovery": 60,
    "path_follower": 40,
}


@register("safety", "cmd_vel_mux", description="Priority-based cmd_vel arbitration")
class CmdVelMux(Module, layer=0):
    """Priority-based cmd_vel multiplexer.

    Each source has its own In port.  The mux forwards the highest-priority
    active source to driver_cmd_vel.
    """

    # -- Inputs (one per source) --
    teleop_cmd_vel:        In[Twist]
    visual_servo_cmd_vel:  In[Twist]
    recovery_cmd_vel:      In[Twist]
    path_follower_cmd_vel: In[Twist]

    # -- Outputs --
    driver_cmd_vel: Out[Twist]     # → driver
    active_source:  Out[str]       # who currently controls

    def __init__(
        self,
        source_timeout: float = 0.5,
        **kw,
    ):
        super().__init__(**kw)
        self._source_timeout = source_timeout

        # Per-source state: last twist + last publish time
        self._sources: dict[str, dict] = {}
        for name, priority in _DEFAULT_PRIORITIES.items():
            self._sources[name] = {
                "priority": priority,
                "last_time": 0.0,
                "last_twist": Twist(),
            }

        self._active: str = ""
        self._last_publish_time: float = 0.0
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._monitor_thread: threading.Thread | None = None
        self._frozen: bool = False

    def freeze(self) -> None:
        self._frozen = True
        self.driver_cmd_vel.publish(Twist.zero())
        logger.info("CmdVelMux: frozen")

    def unfreeze(self) -> None:
        self._frozen = False
        logger.info("CmdVelMux: unfrozen")

    @property
    def is_frozen(self) -> bool:
        return self._frozen

    def setup(self) -> None:
        self.teleop_cmd_vel.subscribe(
            lambda t: self._on_source("teleop", t))
        self.visual_servo_cmd_vel.subscribe(
            lambda t: self._on_source("visual_servo", t))
        self.recovery_cmd_vel.subscribe(
            lambda t: self._on_source("recovery", t))
        self.path_follower_cmd_vel.subscribe(
            lambda t: self._on_source("path_follower", t))

    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        if self._monitor_thread is None or not self._monitor_thread.is_alive():
            self._monitor_thread = threading.Thread(
                target=self._timeout_loop,
                name="cmd_vel_mux_timeout",
                daemon=True,
            )
            self._monitor_thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=1.0)
        self._monitor_thread = None
        super().stop()

    def _timeout_loop(self) -> None:
        interval = max(0.02, min(0.1, self._source_timeout / 2.0))
        while not self._stop_event.wait(interval):
            self._check_timeout()

    @staticmethod
    def _sanitize_twist(twist: Twist) -> Twist:
        values = (
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
        )
        if all(math.isfinite(float(v)) for v in values):
            return twist
        logger.warning("CmdVelMux: dropping non-finite cmd_vel")
        return Twist.zero()

    def _on_source(self, name: str, twist: Twist) -> None:
        """Handle incoming cmd_vel from a named source."""
        if self._frozen:
            return
        now = time.time()
        twist = self._sanitize_twist(twist)
        active_update: str | None = None
        driver_twist: Twist | None = None

        with self._lock:
            src = self._sources[name]
            src["last_time"] = now
            src["last_twist"] = twist

            winner = self._select_active(now)
            if winner != self._active:
                if self._active and winner:
                    logger.info("CmdVelMux: %s -> %s", self._active, winner)
                elif winner:
                    logger.info("CmdVelMux: %s active", winner)
                self._active = winner
                active_update = winner

            if name == winner:
                driver_twist = twist
                self._last_publish_time = now

        if active_update is not None:
            self.active_source.publish(active_update)
        if driver_twist is not None:
            self.driver_cmd_vel.publish(driver_twist)

    def _check_timeout(self, now: float | None = None) -> None:
        if self._frozen:
            return
        now = time.time() if now is None else now
        with self._lock:
            winner = self._select_active(now)
            if winner == self._active:
                return
            self._active = winner
            driver_twist = (
                self._sanitize_twist(self._sources[winner]["last_twist"])
                if winner
                else Twist.zero()
            )
            self._last_publish_time = now

        self.active_source.publish(winner)
        self.driver_cmd_vel.publish(driver_twist)

    def _select_active(self, now: float) -> str:
        """Return the name of the highest-priority source that is still active."""
        with self._lock:
            best_name = ""
            best_priority = -1
            for name, src in self._sources.items():
                age = now - src["last_time"]
                if age <= self._source_timeout and src["priority"] > best_priority:
                    best_name = name
                    best_priority = src["priority"]
            return best_name

    # -- health ---------------------------------------------------------

    def health(self) -> dict:
        now = time.time()
        with self._lock:
            sources = {}
            for name, src in self._sources.items():
                last_time = src["last_time"]
                age = now - last_time if last_time > 0.0 else None
                sources[name] = {
                    "priority": src["priority"],
                    "active": age is not None and age <= self._source_timeout,
                    "age_ms": round(age * 1000) if age is not None else None,
                }
            active = self._active or "none"
        return {
            "active_source": active,
            "source_timeout_s": float(self._source_timeout),
            "sources": sources,
        }
