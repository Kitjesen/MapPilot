"""LocalizationMonitorModule - monitors localization health and outputs speed scaling.

Extracted from the former NavigationLocalizationMixin. This module owns:
  1. Localization state tracking (LOST / TRACKING / FALLBACK_GNSS_ONLY / etc.)
  2. Degeneracy-based speed scaling computation
  3. Motion-hold signal detection (published as part of state output)

NOT owned by this module (stays in Navigation core):
  - map->odom TF transform application (_apply_map_odom_tf, _odom_pose_in_planning_frame)
  - Direct odometry coordinate transformation in the odom callback
  - Pause/resume FSM calls (Navigation owns its own FSM transitions)

Navigation subscribes to `speed_scale` and `localization_state` to drive
its own pause/resume and speed-limit decisions.
"""

from __future__ import annotations

import logging
from typing import Any

from runtime.module import Module
from runtime.registry import register
from runtime.stream import In, Out

logger = logging.getLogger(__name__)

# Signals / actions in a localization status message that require the robot
# to hold motion until an operator confirms a new goal.
LOCALIZATION_MOTION_HOLD_SIGNALS = {"ODOM_MISSING"}
LOCALIZATION_MOTION_HOLD_ACTIONS = {"restart_localization_chain"}

# Degeneracy-level to speed-scale mapping.
_DEGEN_SPEED_SCALE: dict[str, float] = {
    "FALLBACK_GNSS_ONLY": 0.3,
    "SEVERE": 0.4,
    "MILD": 0.7,
}


@register("localization_monitor", "default")
class LocalizationMonitorModule(Module, layer=5):
    """Monitors localization health and publishes speed scaling factors.

    Inputs:
        localization_status: SLAM / localization health dict with keys:
            - state (str): e.g. TRACKING, LOST, FALLBACK_GNSS_ONLY, UNINIT
            - confidence (float): 0.0 - 1.0
            - degeneracy (str): NONE, MILD, SEVERE
            - motion_hold_required (bool, optional)
            - auto_resume_blocked (bool, optional)
            - recovery_signal (str, optional)
            - recovery_action (str, optional)

    Outputs:
        speed_scale: float multiplier (0.3 - 1.0) for navigation speed
        localization_state: dict with current state, degeneracy, confidence,
                            and motion_hold_required flag

    Note:
        The map->odom TF transform application is handled by the Navigation
        core module (ExecutionMixin), not by this monitor.
    """

    runtime_id = "nav.localization_monitor"

    # -- Inputs --
    localization_status: In[dict]

    # -- Outputs --
    speed_scale: Out[float]
    localization_state: Out[dict]

    def __init__(self, **kw: Any) -> None:
        super().__init__(**kw)
        # Current localization state tracking
        self._loc_state: str = "UNINIT"
        self._loc_confidence: float = 0.0
        self._degen_level: str = "NONE"
        self._speed_scale: float = 1.0
        self._speed_policy_reason: str = "normal"
        self._motion_hold_required: bool = False

    def setup(self) -> None:
        self.localization_status.subscribe(self._on_localization_status)
        # Publish initial state
        self._publish_state()

    # ------------------------------------------------------------------
    # Input handler
    # ------------------------------------------------------------------

    def _on_localization_status(self, msg: dict) -> None:
        """Process localization status update and recompute speed scale."""
        prev_state = self._loc_state
        self._loc_state = str(msg.get("state", "UNINIT"))
        self._loc_confidence = float(msg.get("confidence", 0.0))
        self._degen_level = str(msg.get("degeneracy", "NONE"))
        self._motion_hold_required = self._check_motion_hold_required(msg)

        # Log significant state transitions
        if self._loc_state != prev_state:
            logger.info(
                "Localization state: %s -> %s (confidence=%.2f, degen=%s)",
                prev_state,
                self._loc_state,
                self._loc_confidence,
                self._degen_level,
            )

        # Recompute degeneracy-based speed scaling
        self._apply_degeneracy_speed_limit()

        # Publish updated state
        self._publish_state()

    # ------------------------------------------------------------------
    # Speed scaling
    # ------------------------------------------------------------------

    def _apply_degeneracy_speed_limit(self) -> None:
        """Scale navigation speed based on SLAM health.

        FALLBACK_GNSS_ONLY -> 0.3x (cautious - SLAM has been DEGRADED for >10s
                                   with healthy GNSS; we are essentially flying
                                   on absolute fixes plus dead reckoning).
        DEGEN SEVERE       -> 0.4x
        DEGEN MILD         -> 0.7x
        otherwise          -> 1.0x

        CRITICAL is handled by the DEGRADED->LOST path in Navigation
        (mission pauses).
        """
        prev_scale = self._speed_scale
        if self._loc_state == "FALLBACK_GNSS_ONLY":
            self._speed_scale = 0.3
            reason = "FALLBACK_GNSS_ONLY"
        elif self._degen_level in _DEGEN_SPEED_SCALE:
            self._speed_scale = _DEGEN_SPEED_SCALE[self._degen_level]
            reason = f"degeneracy={self._degen_level}"
        else:
            self._speed_scale = 1.0
            reason = "normal"
        self._speed_policy_reason = reason

        if self._speed_scale != prev_scale:
            if self._speed_scale < 1.0:
                logger.info(
                    "LocalizationMonitor: speed scaled to %.0f%% (%s)",
                    self._speed_scale * 100,
                    reason,
                )
            else:
                logger.info("LocalizationMonitor: speed restored to 100%%")

    # ------------------------------------------------------------------
    # Motion-hold detection
    # ------------------------------------------------------------------

    def _check_motion_hold_required(self, msg: dict) -> bool:
        """Determine whether the localization status requires motion hold.

        Motion hold means the robot should NOT resume autonomously after
        localization recovery; an operator must provide an explicit new goal.
        """
        if bool(msg.get("motion_hold_required")):
            return True
        if bool(msg.get("auto_resume_blocked")):
            return True
        if str(msg.get("recovery_signal") or "") in LOCALIZATION_MOTION_HOLD_SIGNALS:
            return True
        return str(msg.get("recovery_action") or "") in LOCALIZATION_MOTION_HOLD_ACTIONS

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish_state(self) -> None:
        """Publish current speed_scale and localization_state."""
        self.speed_scale.publish(self._speed_scale)
        self.localization_state.publish(
            {
                "state": self._loc_state,
                "confidence": self._loc_confidence,
                "degeneracy": self._degen_level,
                "motion_hold_required": self._motion_hold_required,
                "speed_scale": self._speed_scale,
                "speed_policy_reason": self._speed_policy_reason,
            }
        )

    # ------------------------------------------------------------------
    # Health / diagnostics
    # ------------------------------------------------------------------

    def health(self) -> dict[str, Any]:
        return {
            "loc_state": self._loc_state,
            "loc_confidence": self._loc_confidence,
            "degeneracy": self._degen_level,
            "speed_scale": self._speed_scale,
            "motion_hold_required": self._motion_hold_required,
        }
