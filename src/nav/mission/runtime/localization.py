"""Localization health reactions for Navigation."""

from __future__ import annotations

import logging
import math
import time

from nav.mission.model.state import MissionEvent, MissionState
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np

logger = logging.getLogger(__name__)

LOCALIZATION_MOTION_HOLD_SIGNALS = {"ODOM_MISSING"}
LOCALIZATION_MOTION_HOLD_ACTIONS = {"restart_localization_chain"}


class NavigationLocalizationMixin:
    def _on_localization_status(self, msg: dict) -> None:
        prev = self._loc_state
        self._loc_state = msg.get("state", "UNINIT")
        self._loc_confidence = msg.get("confidence", 0.0)
        self._degen_level = msg.get("degeneracy", "NONE")
        motion_hold_required = self._localization_status_requires_motion_hold(msg)

        if motion_hold_required:
            self._localization_recovery_motion_hold = True
            if self._is_path_execution_state():
                self._pause_for_localization(
                    reason="localization recovery requires explicit new goal"
                )
                logger.warning(
                    "Navigation PAUSED: localization recovery requires "
                    "operator-confirmed motion"
                )
            elif self._paused_for_localization:
                self._failure_reason = (
                    "localization recovered; explicit goal required"
                )

        if self._loc_state == "LOST" and prev != "LOST":
            if self._is_path_execution_state():
                self._pause_for_localization(reason="localization lost")
                logger.warning("Navigation PAUSED: localization lost")

        elif self._loc_state == "TRACKING" and self._paused_for_localization:
            if self._localization_recovery_motion_hold:
                self._failure_reason = (
                    "localization recovered; explicit goal required"
                )
                if self._get_state() != MissionState.IDLE:
                    self._apply_event(MissionEvent.STOP, reason=self._failure_reason)
                logger.warning(
                    "Navigation HOLD: localization recovered after automatic "
                    "chain restart; waiting for explicit goal"
                )
            else:
                self._paused_for_localization = False
                if self._pre_pause_state and self._goal is not None:
                    self._apply_event(
                        MissionEvent.RESUME,
                        reason="localization recovered",
                    )
                    self._pre_pause_state = None
                    logger.info("Navigation RESUMED: localization recovered")

        # Degeneracy-aware speed scaling
        self._apply_degeneracy_speed_limit()

    def _pause_for_localization(self, *, reason: str) -> None:
        if not self._paused_for_localization:
            self._pre_pause_state = self._get_state()
        self._paused_for_localization = True
        self._tracker.pause()
        self._failure_reason = reason
        if self._get_state() != MissionState.IDLE:
            self._apply_event(MissionEvent.PAUSE, reason=reason)

    def _clear_localization_pause_for_explicit_action(
        self,
        *,
        reason: str,
        clear_goal: bool,
    ) -> None:
        if clear_goal:
            self._goal = None
        if not (
            self._paused_for_localization
            or self._pre_pause_state is not None
            or self._localization_recovery_motion_hold
        ):
            return
        self._paused_for_localization = False
        self._pre_pause_state = None
        self._localization_recovery_motion_hold = False
        self._tracker.clear()
        logger.info("Navigation localization hold cleared by %s", reason)

    def _localization_status_requires_motion_hold(self, msg: dict) -> bool:
        if bool(msg.get("motion_hold_required")):
            return True
        if bool(msg.get("auto_resume_blocked")):
            return True
        if str(msg.get("recovery_signal") or "") in LOCALIZATION_MOTION_HOLD_SIGNALS:
            return True
        return (
            str(msg.get("recovery_action") or "")
            in LOCALIZATION_MOTION_HOLD_ACTIONS
        )

    def _apply_degeneracy_speed_limit(self) -> None:
        """Scale navigation speed based on SLAM health.

        FALLBACK_GNSS_ONLY -> 0.3x (cautious - SLAM has been DEGRADED for >10s
                                   with healthy GNSS; we are essentially flying
                                   on absolute fixes plus dead reckoning).
        DEGEN SEVERE       -> 0.4x
        DEGEN MILD         -> 0.7x
        otherwise          -> 1.0x

        CRITICAL is handled by the DEGRADED->LOST path above (mission pauses).
        """
        prev_scale = self._speed_scale
        reason = ""
        if self._loc_state == "FALLBACK_GNSS_ONLY":
            self._speed_scale = 0.3
            reason = "FALLBACK_GNSS_ONLY"
        elif self._degen_level == "SEVERE":
            self._speed_scale = 0.4
            reason = "degeneracy=SEVERE"
        elif self._degen_level == "MILD":
            self._speed_scale = 0.7
            reason = "degeneracy=MILD"
        else:
            self._speed_scale = 1.0
            reason = "normal"
        self._speed_policy_reason = reason

        if self._speed_scale != prev_scale and self._get_state() == MissionState.EXECUTING:
            if self._speed_scale < 1.0:
                logger.info("Navigation speed scaled to %.0f%% (%s)",
                            self._speed_scale * 100, reason)
            else:
                logger.info("Navigation speed restored to 100%%")

    def _accelerate_stuck_check_on_reverse_motion(self, odom: Odometry) -> None:
        waypoint = self._tracker.current_waypoint
        if waypoint is None:
            return
        robot = np.asarray(self._robot_pos[:2], dtype=float)
        target = np.asarray(waypoint[:2], dtype=float)
        to_target = target - robot
        distance = float(np.linalg.norm(to_target))
        if distance <= 1e-6:
            return
        vx_body = float(getattr(odom.twist.linear, "x", 0.0) or 0.0)
        vy_body = float(getattr(odom.twist.linear, "y", 0.0) or 0.0)
        if math.hypot(vx_body, vy_body) < 0.05:
            return
        cy = math.cos(self._robot_yaw)
        sy = math.sin(self._robot_yaw)
        velocity_world = np.asarray(
            [cy * vx_body - sy * vy_body, sy * vx_body + cy * vy_body],
            dtype=float,
        )
        away_speed = -float(np.dot(velocity_world, to_target / distance))
        if away_speed <= 0.05:
            return
        if self._tracker.accelerate_stuck_check(max_remaining_s=3.0):
            self.adapter_status.publish({
                "event": "reverse_motion_stuck_accelerated",
                "away_speed_mps": round(away_speed, 3),
                "waypoint_index": self._tracker.wp_index,
            })
