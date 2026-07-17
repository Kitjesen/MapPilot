"""Recovery motion runtime for Navigation."""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any

from nav.model.recovery import recovery_strategy_for
from nav.model.state import MissionEvent, MissionState
from runtime.msgs.geometry import Twist, Vector3

logger = logging.getLogger(__name__)


class NavigationRecoveryMixin:
    def _request_recovery_stop(self, join_timeout: float = 0.2) -> None:
        with self._recovery_lock:
            self._recovery_stop_event.set()
            thread = self._recovery_thread

        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=join_timeout)

        with self._recovery_lock:
            if self._recovery_thread is thread and (thread is None or not thread.is_alive()):
                self._recovery_thread = None

    def _execute_recovery_motion(self, post_action: str = "replan") -> bool:
        if self._is_path_execution_state():
            self._apply_event(MissionEvent.STUCK, reason="stuck")
        klass = getattr(self, "_latest_traversability_class", "unknown")
        strat = recovery_strategy_for(klass)

        self.adapter_status.publish(
            {
                "event": "recovery_started",
                "strategy": strat["strategy"],
                "reason": klass,
                "ts": time.time(),
            }
        )
        logger.info("Recovery: strategy=%s, reason=%s", strat["strategy"], klass)
        self.clear_path.publish(True)

        self._request_recovery_stop()
        stop_event = threading.Event()
        thread = threading.Thread(
            target=self._run_recovery_motion,
            args=(strat, klass, post_action, stop_event),
            name="navigation-recovery-motion",
            daemon=True,
        )
        with self._recovery_lock:
            self._recovery_stop_event = stop_event
            self._recovery_thread = thread
        thread.start()
        return True

    def _run_recovery_motion(
        self,
        strat: dict[str, Any],
        reason: str,
        post_action: str,
        stop_event: threading.Event,
    ) -> None:
        step_hz = 10

        def _drive(linear_x: float, angular_z: float, duration: float) -> bool:
            if duration <= 0.0:
                return True
            steps = max(1, int(math.ceil(duration * step_hz)))
            for _ in range(steps):
                if stop_event.is_set() or self._get_state() != MissionState.RECOVERING:
                    return False
                self.recovery_cmd_vel.publish(
                    Twist(
                        linear=Vector3(linear_x, 0.0, 0.0),
                        angular=Vector3(0.0, 0.0, angular_z),
                    )
                )
                if stop_event.wait(1.0 / step_hz):
                    return False
            return True

        def _finish(completed: bool) -> None:
            self.recovery_cmd_vel.publish(Twist.zero())
            with self._recovery_lock:
                if self._recovery_thread is threading.current_thread():
                    self._recovery_thread = None
            if completed and not stop_event.is_set():
                self._finish_recovery_motion(post_action, reason)

        if not _drive(strat["backup_speed"], 0.0, strat["backup_duration"]):
            _finish(False)
            return

        if strat["rotate_duration"] > 0.0 and strat["rotate_speed"] != 0.0:
            direction = 1.0 if self._get_replan_count() % 2 == 1 else -1.0
            if not _drive(0.0, strat["rotate_speed"] * direction, strat["rotate_duration"]):
                _finish(False)
                return

        if not _drive(strat["forward_speed"], 0.0, strat["forward_duration"]):
            _finish(False)
            return

        _finish(True)

    def _is_recovery_active(self) -> bool:
        with self._recovery_lock:
            return bool(self._recovery_thread is not None and self._recovery_thread.is_alive())

    def _finish_recovery_motion(self, post_action: str, reason: str) -> None:
        if post_action == "none":
            return
        if self._get_state() != MissionState.RECOVERING:
            return
        if post_action == "external_strategy":
            self._apply_event(
                MissionEvent.PLAN_OK,
                reason="recovery complete; retaining external strategy path",
            )
            self._tracker.pause()
            self._republish_external_strategy_path()
            logger.warning(
                "Stuck detected, recovery motion; retaining external strategy path (%d/%d)",
                self._get_replan_count(),
                self._max_replan,
            )
            self.adapter_status.publish(
                {
                    "event": "external_strategy_path_stuck_recovery",
                    "recovery_count": self._get_replan_count(),
                    "max_recovery_count": self._max_replan,
                    "remaining_waypoints": max(
                        0,
                        self._tracker.path_length - self._tracker.wp_index,
                    ),
                    "reason": reason,
                    "ts": time.time(),
                }
            )
            return
        if post_action == "replan":
            self._apply_event(MissionEvent.RECOVERY_OK, reason="recovery complete")
            logger.warning(
                "Stuck detected, recovery motion + replan (%d/%d)",
                self._get_replan_count(),
                self._max_replan,
            )
            self._plan()
