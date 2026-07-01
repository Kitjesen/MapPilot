"""SafetyRingModule — unified safety + evaluation + dialogue in one Module.

Replaces 3 separate modules (SafetyModule, EvaluatorModule, DialogueModule).
Three internal rings, one Module:

  Ring 1 (Reflex):  obstacle/link checks → stop_cmd
  Ring 2 (Cognition): cross-track error, progress rate → assessment
  Ring 3 (Dialogue):  aggregate state → user-facing status

Usage::

    bp.add(SafetyRingModule)
    # auto_wire connects odometry, path, cmd_vel, etc.
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from enum import Enum
from typing import Any

from core.module import Module, skill
from core.msgs.geometry import Twist
from core.msgs.nav import Odometry, Path
from core.msgs.numpy_compat import np
from core.msgs.semantic import ExecutionEval, SafetyState
from core.registry import register
from core.stream import In, Out

logger = logging.getLogger(__name__)


class SafetyLevel(Enum):
    """Safety severity levels for SafetyRing reflex actions."""
    SAFE = 0
    WARN = 1
    STOP = 2


class Assessment(Enum):
    """Robot motion assessment categories from the safety evaluator."""
    IDLE = "IDLE"
    ON_TRACK = "ON_TRACK"
    DRIFTING = "DRIFTING"
    STALLED = "STALLED"
    REGRESSING = "REGRESSING"


@register("safety", "ring", description="Unified safety ring (reflex + eval + dialogue)")
class SafetyRingModule(Module, layer=0):
    """Three-ring safety architecture in one Module.

    Ring 1 (reflex): obstacle detection → stop_cmd
    Ring 2 (cognition): execution quality evaluation
    Ring 3 (dialogue): user-facing state aggregation
    """

    # -- Inputs --
    odometry: In[Odometry]
    path: In[Path]
    cmd_vel: In[Twist]
    mission_status: In[dict]
    localization_status: In[dict]
    gnss_fusion_health: In[dict]

    # -- Outputs --
    stop_cmd: Out[int]           # 0=clear, 1=soft, 2=hard
    safety_state: Out[SafetyState]
    execution_eval: Out[ExecutionEval]
    dialogue_state: Out[dict]

    def __init__(
        self,
        cross_track_warn: float = 1.5,
        cross_track_danger: float = 3.0,
        stall_threshold: float = 0.05,
        progress_window_sec: float = 3.0,
        odom_timeout_ms: float = 500.0,
        cmd_vel_timeout_ms: float = 300.0,
        **kw,
    ):
        super().__init__(**kw)
        # Ring 1 config
        self._odom_timeout = odom_timeout_ms / 1000.0
        self._cmdvel_timeout = cmd_vel_timeout_ms / 1000.0

        # Ring 2 config
        self._ct_warn = cross_track_warn
        self._ct_danger = cross_track_danger
        self._stall_thr = stall_threshold
        self._progress_window = progress_window_sec

        # Ring 1 state
        self._last_odom_time = 0.0
        self._last_cmdvel_time = time.time()
        self._invalid_odom = False
        self._invalid_cmd_vel = False
        self._safety_level = SafetyLevel.SAFE
        self._publishing_stop_cmd_level: int | None = None

        # Ring 2 state
        self._robot_xy = [0.0, 0.0]
        self._robot_yaw = 0.0
        self._path_points: np.ndarray | None = None
        self._goal_xy: np.ndarray | None = None
        self._cmd_speed = 0.0
        self._actual_speed = 0.0
        self._progress_history: list[tuple[float, float]] = []
        self._last_progress_time = time.monotonic()
        self._last_distance = float("inf")
        self._assessment = Assessment.IDLE

        # Localization state
        self._loc_state: str = "UNINIT"
        self._loc_confidence: float = 0.0

        # GNSS fusion state (populated from gnss_fusion_health port)
        self._gnss_enabled: bool = False
        self._gnss_alignment_locked: bool = False
        self._gnss_last_fix_type: str = "NONE"
        self._gnss_age_s: float = float("inf")
        self._gnss_residual_m: float = 0.0
        self._gnss_relock_count: int = 0
        # Thresholds — when SLAM is DEGRADED we check these to decide whether
        # the root cause is GNSS loss ("GNSS signal lost") vs SLAM sensor
        # degeneracy ("SLAM localization weak"). Separate messages help
        # operators diagnose faster.
        self._gnss_max_age_warn_s: float = 5.0
        self._gnss_healthy_fix_types: set = {"RTK_FIXED", "RTK_FLOAT"}

        # Ring 3 state
        self._latest_mission: dict | None = None
        self._instruction = ""

        self._monitor_stop = threading.Event()
        self._monitor_thread: threading.Thread | None = None

    def start(self) -> None:
        super().start()
        self._monitor_stop.clear()
        if self._monitor_thread is None or not self._monitor_thread.is_alive():
            self._monitor_thread = threading.Thread(
                target=self._monitor_loop,
                name="safety_ring_monitor",
                daemon=True,
            )
            self._monitor_thread.start()

    def stop(self) -> None:
        self._monitor_stop.set()
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=1.0)
        self._monitor_thread = None
        super().stop()

    def _monitor_loop(self) -> None:
        interval = max(0.02, min(self._odom_timeout, self._cmdvel_timeout) / 2.0)
        while not self._monitor_stop.wait(interval):
            self._publish_safety()

    def _has_motion_intent(self) -> bool:
        mission = self._latest_mission or {}
        state = str(mission.get("state", "")).upper()
        idle_states = {"", "IDLE", "SUCCESS", "DONE", "CANCELLED", "CANCELED", "FAILED", "ABORTED"}
        return (
            state not in idle_states
            or self._path_points is not None
            or self._cmd_speed > self._stall_thr
        )

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.path.subscribe(self._on_path)
        self.cmd_vel.subscribe(self._on_cmdvel)
        self.mission_status.subscribe(self._on_mission)
        self.localization_status.subscribe(self._on_localization_status)
        self.gnss_fusion_health.subscribe(self._on_gnss_fusion_health)
        self._publish_safety()

    # -- Ring 1: Reflex Safety -----------------------------------------------

    def _check_links(self) -> SafetyLevel:
        """Check communication link health + localization status."""
        now = time.time()
        odom_alive = (now - self._last_odom_time) < self._odom_timeout
        cmd_alive = (now - self._last_cmdvel_time) < self._cmdvel_timeout

        if self._invalid_odom or self._invalid_cmd_vel:
            return SafetyLevel.STOP
        if not odom_alive:
            self._path_points = None
            self._goal_xy = None
            return SafetyLevel.STOP
        if self._loc_state == "LOST":
            return SafetyLevel.STOP
        if not cmd_alive and self._has_motion_intent():
            return SafetyLevel.WARN
        if self._loc_state == "DEGRADED":
            return SafetyLevel.WARN
        return SafetyLevel.SAFE

    def _publish_safety(self):
        level = self._check_links()
        self._safety_level = level
        if level == SafetyLevel.STOP:
            stop_level = 2
        elif level == SafetyLevel.WARN:
            stop_level = 1
        else:
            stop_level = 0

        # Avoid recursive STOP storms: the stop command clears paths and
        # publishes zero cmd_vel, which is looped back into this module for
        # auditing through CmdVelMux.driver_cmd_vel. Suppress only re-entrant
        # same-level publishes while the current stop_cmd delivery is active;
        # later safety ticks may still resend STOP for restarted downstreams.
        if stop_level == self._publishing_stop_cmd_level:
            return
        self._publishing_stop_cmd_level = stop_level
        try:
            self.stop_cmd.publish(stop_level)
        finally:
            self._publishing_stop_cmd_level = None

        self.safety_state.publish(SafetyState(
            level=level.value,
        ))

    # -- Ring 2: Execution Evaluation ----------------------------------------

    def _cross_track_error(self) -> float:
        if self._path_points is None or len(self._path_points) < 2:
            return 0.0
        p = self._robot_xy
        a = self._path_points[:-1]
        b = self._path_points[1:]
        ab = b - a
        ap = p - a
        ab_sq = np.sum(ab * ab, axis=1)
        ab_sq = np.where(ab_sq < 1e-10, 1.0, ab_sq)
        t = np.clip(np.sum(ap * ab, axis=1) / ab_sq, 0.0, 1.0)
        proj = a + t[:, np.newaxis] * ab
        dists = np.linalg.norm(p - proj, axis=1)
        return float(np.min(dists))

    def _distance_to_goal(self) -> float:
        if self._goal_xy is None:
            return float("inf")
        return float(np.linalg.norm(self._robot_xy - self._goal_xy))

    def _progress_rate(self) -> float:
        now = time.monotonic()
        dist = self._distance_to_goal()
        self._progress_history.append((now, dist))
        cutoff = now - self._progress_window
        self._progress_history = [(t, d) for t, d in self._progress_history if t > cutoff]
        if len(self._progress_history) < 2:
            return 0.0
        t0, d0 = self._progress_history[0]
        t1, d1 = self._progress_history[-1]
        dt = t1 - t0
        return (d1 - d0) / dt if dt > 0.1 else 0.0

    def _evaluate(self):
        if self._path_points is None:
            self._assessment = Assessment.IDLE
            return

        cte = self._cross_track_error()
        rate = self._progress_rate()
        now = time.monotonic()
        dist = self._distance_to_goal()

        if dist < self._last_distance - 0.1:
            self._last_progress_time = now
            self._last_distance = dist
        stall = now - self._last_progress_time

        if rate > 0.02 and stall > 3.0:
            self._assessment = Assessment.REGRESSING
        elif stall > self._progress_window:
            self._assessment = Assessment.STALLED
        elif cte > self._ct_warn:
            self._assessment = Assessment.DRIFTING
        else:
            self._assessment = Assessment.ON_TRACK

        self.execution_eval.publish(ExecutionEval(
            assessment=self._assessment.value,
            cross_track_error=round(cte, 2),
            distance_to_goal=round(dist, 1),
            progress_rate=round(rate, 3),
        ))

    # -- Ring 3: Dialogue State ----------------------------------------------

    def _publish_dialogue(self):
        mission = self._latest_mission or {}
        payload = {
            "safety": self._safety_level.name,
            "assessment": self._assessment.value,
            "mission": mission.get("state", "IDLE"),
            "instruction": self._instruction,
            "localization": self._loc_state,
            "ts": time.time(),
        }
        # Attach GNSS root-cause diagnosis when localization is impaired,
        # so operators can distinguish "sky-blocked/no-RTK" from
        # "LiDAR degeneracy in corridor".
        if self._loc_state in ("DEGRADED", "LOST"):
            payload["loc_root_cause"] = self._degraded_root_cause()
            payload["gnss_fix_type"] = self._gnss_last_fix_type
            payload["gnss_age_s"] = (
                None if not math.isfinite(self._gnss_age_s)
                else round(self._gnss_age_s, 2)
            )
        self.dialogue_state.publish(payload)

    # -- Input callbacks -----------------------------------------------------

    def _on_odom(self, odom: Odometry):
        self._last_odom_time = time.time()
        x, y = odom.x, odom.y
        self._invalid_odom = not all(
            math.isfinite(float(v))
            for v in (odom.x, odom.y, odom.yaw, odom.vx, odom.vy)
        )
        if math.isfinite(x) and math.isfinite(y):
            self._robot_xy = [x, y]
        if math.isfinite(odom.yaw):
            self._robot_yaw = odom.yaw
        self._actual_speed = (
            math.hypot(odom.vx, odom.vy)
            if math.isfinite(odom.vx) and math.isfinite(odom.vy)
            else float("inf")
        )

        # Tick all three rings
        self._publish_safety()
        self._evaluate()
        self._publish_dialogue()

    def _on_path(self, msg: Path):
        pts = []
        for ps in msg.poses:
            if math.isfinite(ps.x) and math.isfinite(ps.y):
                pts.append([ps.x, ps.y])
        if len(pts) >= 2:
            self._path_points = np.array(pts)
            self._goal_xy = self._path_points[-1].copy()
            self._progress_history.clear()
            self._last_progress_time = time.monotonic()
            self._last_distance = float("inf")
        else:
            self._path_points = None
            self._goal_xy = None

    def _on_cmdvel(self, msg: Twist):
        self._last_cmdvel_time = time.time()
        values = (
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z,
        )
        self._invalid_cmd_vel = not all(math.isfinite(float(v)) for v in values)
        self._cmd_speed = (
            math.hypot(msg.linear.x, msg.linear.y)
            if not self._invalid_cmd_vel
            else float("inf")
        )
        self._publish_safety()

    def _on_mission(self, status: dict):
        self._latest_mission = status

    def _on_localization_status(self, msg: dict) -> None:
        self._loc_state = msg.get("state", "UNINIT")
        self._loc_confidence = msg.get("confidence", 0.0)
        self._publish_safety()

    def _on_gnss_fusion_health(self, msg: dict) -> None:
        self._gnss_enabled = bool(msg.get("enabled", False))
        self._gnss_alignment_locked = bool(msg.get("alignment_locked", False))
        self._gnss_last_fix_type = str(msg.get("last_fix_type", "NONE"))
        self._gnss_age_s = float(msg.get("last_gnss_age_s", float("inf")))
        self._gnss_residual_m = float(msg.get("last_residual_m", 0.0))
        self._gnss_relock_count = int(msg.get("relock_count", 0))

    def _gnss_healthy(self) -> bool:
        """True iff GNSS is currently providing fusion-quality data."""
        if not self._gnss_enabled:
            return False
        if self._gnss_last_fix_type not in self._gnss_healthy_fix_types:
            return False
        return self._gnss_age_s <= self._gnss_max_age_warn_s

    def _degraded_root_cause(self) -> str:
        """Diagnose why localization is DEGRADED.

        Returns: 'gnss_lost' | 'slam_weak' | 'both' | 'unknown'.
        Only meaningful when _loc_state in ('DEGRADED', 'LOST').
        """
        gnss_ok = self._gnss_healthy()
        # When GNSS fusion is disabled, degradation is always SLAM-side
        if not self._gnss_enabled:
            return "slam_weak"
        if gnss_ok:
            return "slam_weak"
        # GNSS is enabled and unhealthy — is SLAM also weak?
        if self._loc_confidence < 0.3:
            return "both"
        return "gnss_lost"

    # -- AI-callable skills -----------------------------------------------

    @skill
    def get_safety_status(self) -> str:
        """Get current safety state: level, cross-track error, distance to goal, and module health."""
        return json.dumps({
            "level": self._safety_level.name,
            "cross_track_error": round(self._cross_track_error(), 3),
            "distance_to_goal": round(self._distance_to_goal(), 3),
            "modules_ok": self._check_links() != SafetyLevel.STOP,
        })

    @skill
    def emergency_stop(self) -> str:
        """Trigger an emergency stop (safety level STOP). Use for immediate halt."""
        self._safety_level = SafetyLevel.STOP
        self.stop_cmd.publish(2)
        self.safety_state.publish(SafetyState(level=SafetyLevel.STOP.value))
        return json.dumps({"status": "estop_triggered"})

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["safety_ring"] = {
            "level": self._safety_level.name,
            "assessment": self._assessment.value,
            "has_path": self._path_points is not None,
            "localization_state": self._loc_state,
            "localization_confidence": self._loc_confidence,
        }
        return info
