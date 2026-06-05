"""NavigationModule — unified path planning + tracking + mission FSM.

NAV COMPUTE CONTRACT (docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md):
  This module owns the L5 GLOBAL PLANNING layer (tomogram/saved_map -> global_path
  + waypoint) and the L2 SAFETY GATING use of `costmap`.

  `costmap` (= TraversabilityCostModule.fused_cost) is consumed ONLY for safety
  gating, never to produce trajectories:
    1. GlobalPlannerService.update_map() overlays live risk onto PCT/A* safe-goal
       and path-safety checks.
    2. Throttled replan trigger (PCT defaults OFF — it owns terrain cost via the
       offline tomogram; A* dev/sim defaults ON).
  Local trajectory generation is NOT done here — that is LocalPlannerModule (L2).

Internal services (not Modules — no ports, just algorithm logic):
  - GlobalPlannerService  (nav/global_planner_service.py) — plan() + downsample
  - WaypointTracker       (nav/waypoint_tracker.py)       — arrival + stuck detection

Ports:
  In:  goal_pose, odometry, costmap, instruction, stop_signal, patrol_goals, cancel
  Out: waypoint, global_path, planner_status, mission_status, adapter_status

Usage::

    bp.add(NavigationModule, planner="astar", tomogram="building.pickle")
    bp.add(NavigationModule, planner="pct")   # S100P with ele_planner.so
"""

from __future__ import annotations

import concurrent.futures
import json
import logging
import math
import os
import threading
import time
from typing import Any

import numpy as np

from core.module import Module, skill
from core.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from core.msgs.nav import Odometry
from core.registry import register
from core.runtime_interface import (
    map_frame_id,
)
from nav.services.frame_contract import FrameContract
from core.stream import In, Out
from nav.global_planner_service import GlobalPlannerService
from nav.waypoint_tracker import (
    EV_PATH_COMPLETE,
    EV_STUCK,
    EV_STUCK_WARN,
    EV_WAYPOINT_REACHED,
    WaypointTracker,
)

logger = logging.getLogger(__name__)


LOCALIZATION_MOTION_HOLD_SIGNALS = {"ODOM_MISSING"}
LOCALIZATION_MOTION_HOLD_ACTIONS = {"restart_localization_chain"}
PLANNING_FRAME_ID = map_frame_id()


class _PlanPreviewBusy(RuntimeError):
    """Raised when a non-motion plan preview is already running."""


class MissionState:
    """Mission FSM states for NavigationModule.

    State machine covering the full lifecycle of a navigation mission:
    IDLE → PLANNING → EXECUTING → SUCCESS | FAILED | STUCK | CANCELLED.
    PATROLLING is a persistent cyclic state. Any state can transition
    back to IDLE via stop/cancel/reset.
    """
    IDLE       = "IDLE"
    PLANNING   = "PLANNING"
    EXECUTING  = "EXECUTING"
    SUCCESS    = "SUCCESS"
    FAILED     = "FAILED"
    STUCK      = "STUCK"
    CANCELLED  = "CANCELLED"
    PATROLLING = "PATROLLING"

    # Legal state transitions. Any state → IDLE is always allowed (stop/cancel/reset).
    TRANSITIONS: dict[str, frozenset[str]] = {
        "IDLE":       frozenset(["PLANNING", "PATROLLING", "EXECUTING", "FAILED", "CANCELLED"]),
        "PLANNING":   frozenset(["EXECUTING", "PATROLLING", "FAILED", "IDLE", "CANCELLED"]),
        "EXECUTING":  frozenset([
            "SUCCESS", "FAILED", "STUCK", "IDLE", "PLANNING", "PATROLLING", "CANCELLED"
        ]),
        "PATROLLING": frozenset(["EXECUTING", "SUCCESS", "FAILED", "STUCK", "IDLE", "PLANNING", "CANCELLED"]),
        "SUCCESS":    frozenset(["IDLE", "PLANNING", "PATROLLING"]),
        "FAILED":     frozenset(["IDLE", "PLANNING", "PATROLLING", "EXECUTING"]),
        "STUCK":      frozenset(["IDLE", "PLANNING", "EXECUTING", "CANCELLED"]),
        "CANCELLED":  frozenset(["IDLE", "PLANNING", "PATROLLING", "EXECUTING"]),
    }


@register("navigation", "default", description="Unified navigation module")
class NavigationModule(Module, layer=5):
    """Unified navigation: plan → track → FSM in one Module.

    Architecture:
      NavigationModule owns the mission FSM and patrol logic.
      GlobalPlannerService owns planning algorithm + downsample.
      WaypointTracker owns arrival detection + stuck detection.
    """

    # -- Inputs --
    goal_pose:    In[PoseStamped]
    odometry:     In[Odometry]
    costmap:      In[dict]
    instruction:  In[str]
    stop_signal:  In[int]
    patrol_goals: In[list]
    cancel:       In[str]
    teleop_active: In[bool]
    localization_status: In[dict]
    traversability: In[dict]  # W2-8: terrain class from TerrainModule
    # P4: TF jump events from SlamBridgeModule. PGO loop closures and BBS3D
    # relocalisations can move map→odom by metres in a single tick. Cached
    # global path + waypoint then point at the wrong place. We force a replan
    # on the next planning tick to catch up.
    map_frame_jump_event: In[dict]

    # -- Outputs --
    waypoint:       Out[PoseStamped]
    global_path:    Out[list]
    clear_path:     Out[bool]
    planner_status: Out[str]
    mission_status: Out[dict]
    adapter_status: Out[dict]
    recovery_cmd_vel: Out[Twist]  # direct cmd_vel for backup/rotate recovery

    def __init__(
        self,
        planner: str = "astar",
        tomogram: str = "",
        obstacle_thr: float = 49.9,
        waypoint_threshold: float = 1.5,
        stuck_timeout: float = 10.0,
        stuck_dist_thre: float = 0.15,
        max_replan_count: int = 3,
        downsample_dist: float = 2.0,
        enable_ros2_bridge: bool = False,
        final_waypoint_threshold: float | None = None,
        waypoint_z_threshold: float | None = 0.25,
        allow_direct_goal_fallback: bool = False,
        direct_goal_fallback_on_planner_failure: bool = False,
        external_strategy_path_control: bool = False,
        external_strategy_start_tolerance_m: float = 1.5,
        goal_update_epsilon: float = 0.25,
        safe_goal_tolerance: float = 4.0,
        plan_safety_policy: str = "observe",
        fallback_planner_name: str = "astar",
        accept_partial_goal_progress: bool = False,
        partial_goal_repeat_ignore_window_s: float = 5.0,
        defer_empty_path_planning_failure: bool = False,
        empty_path_retry_interval_s: float = 3.0,
        empty_path_retry_timeout_s: float = 30.0,
        replan_on_costmap_update: bool | None = None,
        auto_resume_after_teleop: bool = False,
        **kw,
    ):
        super().__init__(**kw)
        planner_key = str(planner or "").strip().lower()
        self._enable_ros2_bridge = enable_ros2_bridge
        self._allow_direct_goal_fallback = allow_direct_goal_fallback
        self._direct_goal_fallback_on_planner_failure = direct_goal_fallback_on_planner_failure
        self._external_strategy_path_control = external_strategy_path_control
        self._external_strategy_start_tolerance_m = float(
            external_strategy_start_tolerance_m
        )
        self._using_external_strategy_path = False
        self._goal_update_epsilon = goal_update_epsilon
        self._safe_goal_tolerance = safe_goal_tolerance
        self._accept_partial_goal_progress = accept_partial_goal_progress
        self._partial_goal_repeat_ignore_window_s = max(
            0.0,
            float(partial_goal_repeat_ignore_window_s),
        )
        self._defer_empty_path_planning_failure = bool(
            defer_empty_path_planning_failure
        )
        self._empty_path_retry_interval_s = max(
            0.1,
            float(empty_path_retry_interval_s),
        )
        self._empty_path_retry_timeout_s = max(
            self._empty_path_retry_interval_s,
            float(empty_path_retry_timeout_s),
        )
        if replan_on_costmap_update is None:
            replan_on_costmap_update = planner_key != "pct"
        self._replan_on_costmap_update = bool(replan_on_costmap_update)
        self._direct_goal_fallback_status: dict[str, Any] | None = None
        self._deferred_empty_path_first_ts: float = 0.0
        self._deferred_empty_path_attempts: int = 0

        self._planner_svc = GlobalPlannerService(
            planner_name=planner,
            tomogram=tomogram,
            obstacle_thr=obstacle_thr,
            downsample_dist=downsample_dist,
            plan_safety_policy=plan_safety_policy,
            fallback_planner_name=fallback_planner_name,
        )
        self._tracker = WaypointTracker(
            threshold=waypoint_threshold,
            final_threshold=final_waypoint_threshold,
            z_threshold=waypoint_z_threshold,
            stuck_timeout=stuck_timeout,
            stuck_dist=stuck_dist_thre,
        )

        # Mission FSM state (protected by _nav_lock for cross-thread access)
        self._nav_lock = threading.Lock()
        self._state = MissionState.IDLE
        self._robot_pos = np.zeros(3)
        self._robot_yaw = 0.0
        self._planning_frame_id = str(kw.get("planning_frame_id", PLANNING_FRAME_ID))
        self._frame_contract = FrameContract(
            planning_frame_id=self._planning_frame_id,
            publish_adapter_status=self.adapter_status.publish,
        )
        self._odom_frame_id = "unknown"
        self._costmap_frame_id = "unknown"
        self._goal_frame_id: str | None = None
        self._goal: np.ndarray | None = None
        self._active_path_terminal_goal: np.ndarray | None = None
        self._partial_progress_completed_goal: np.ndarray | None = None
        self._partial_progress_completed_terminal: np.ndarray | None = None
        self._partial_progress_completed_ts: float = 0.0
        self._active_external_strategy_path: list[np.ndarray] = []
        self._replan_count = 0
        self._max_replan = max_replan_count
        self._failure_reason = ""
        self._mission_start_time = 0.0
        self._mission_timeout = kw.get("mission_timeout", 300.0)

        # Localization-aware pause/resume
        self._loc_state: str = "UNINIT"
        self._loc_confidence: float = 0.0
        self._degen_level: str = "NONE"
        self._paused_for_localization: bool = False
        self._pre_pause_state: str | None = None
        self._localization_recovery_motion_hold: bool = False
        self._planning_timeout = kw.get("planning_timeout", 30.0)
        self._speed_scale: float = 1.0  # degeneracy-based speed multiplier
        self._speed_policy_reason: str = "normal"
        self._preview_timeout_s = float(
            kw.get("preview_timeout", os.environ.get("LINGTU_PLAN_PREVIEW_TIMEOUT", 3.0))
        )
        self._preview_executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="lingtu-plan-preview",
        )
        self._preview_planner_lock = threading.Lock()

        # Teleop pause/resume
        self._auto_resume_after_teleop = bool(auto_resume_after_teleop)
        self._paused_for_teleop: bool = False
        self._pre_teleop_goal: np.ndarray | None = None
        self._pre_teleop_state: str | None = None
        self._recovery_lock = threading.Lock()
        self._recovery_stop_event = threading.Event()
        self._recovery_thread: threading.Thread | None = None

        # Cooldown for costmap-triggered replanning (3s minimum between replans)
        self._last_costmap_replan_time = 0.0

        # Patrol FSM state
        self._patrol_goals: list[np.ndarray] = []
        self._patrol_index = 0
        self._patrol_loop = False

        # W2-8: latest terrain class from TerrainModule — defaults to "unknown"
        # which triggers the conservative backup+rotate recovery strategy.
        self._latest_traversability_class: str = "unknown"

    def setup(self) -> None:
        self._planner_svc.setup()

        self.goal_pose.subscribe(self._on_goal)
        self.odometry.subscribe(self._on_odom)
        self.costmap.subscribe(self._on_costmap)
        self.instruction.subscribe(self._on_instruction)
        self.stop_signal.subscribe(self._on_stop)
        self.patrol_goals.subscribe(self._on_patrol_goals)
        self.cancel.subscribe(self._on_cancel)
        self.teleop_active.subscribe(self._on_teleop_active)
        self.localization_status.subscribe(self._on_localization_status)
        self.traversability.subscribe(self._on_traversability)
        self.map_frame_jump_event.subscribe(self._on_map_frame_jump)

        # DEPRECATED: ROS2 waypoint bridge is now a separate module
        # (nav.ros2_waypoint_bridge_module.ROS2WaypointBridgeModule).
        # The enable_ros2_bridge parameter is kept as a no-op for backward
        # compatibility; set it at the blueprint level instead.
        if self._enable_ros2_bridge:
            logger.info(
                "NavigationModule: enable_ros2_bridge is deprecated — "
                "use ROS2WaypointBridgeModule in the blueprint stack instead"
            )

        self._set_state(MissionState.IDLE)

    # ROS2 spin handled by shared executor (core.ros2_context)

    # ── Mission FSM ───────────────────────────────────────────────────────

    @staticmethod
    def _point_summary(point: np.ndarray | None) -> list[float] | None:
        if point is None:
            return None
        if len(point) < 2:
            return None
        return [
            float(point[0]),
            float(point[1]),
            float(point[2]) if len(point) > 2 else 0.0,
        ]

    @staticmethod
    def _safe_nonnegative_int(value: object, default: int = 0) -> int:
        try:
            parsed = int(value)  # type: ignore[arg-type]
        except (TypeError, ValueError):
            return default
        return max(0, parsed)

    @staticmethod
    def _distance_xy(a: np.ndarray | None, b: np.ndarray | None) -> float | None:
        if a is None or b is None or len(a) < 2 or len(b) < 2:
            return None
        av = np.asarray(a[:2], dtype=float)
        bv = np.asarray(b[:2], dtype=float)
        return round(float(np.linalg.norm(av - bv)), 3)

    @staticmethod
    def _distance_xyz_or_xy(a: np.ndarray, b: np.ndarray) -> float:
        av = np.asarray(a, dtype=float).reshape(-1)
        bv = np.asarray(b, dtype=float).reshape(-1)
        if (
            av.size >= 3
            and bv.size >= 3
            and np.all(np.isfinite(av[:3]))
            and np.all(np.isfinite(bv[:3]))
        ):
            return float(np.linalg.norm(av[:3] - bv[:3]))
        return float(np.linalg.norm(av[:2] - bv[:2]))

    def _set_state(self, state: str) -> None:
        # Snapshot shared state under lock for cross-thread consistency
        with self._nav_lock:
            _current_state = self._state
            _replan_count = self._replan_count
            _goal = self._goal
            _failure_reason = self._failure_reason

        allowed = MissionState.TRANSITIONS.get(_current_state, frozenset())
        if state != _current_state and state not in allowed:
            logger.warning(
                "NavigationModule: illegal state transition %s → %s (allowed: %s)",
                _current_state, state, sorted(allowed),
            )

        # Write new state under lock
        with self._nav_lock:
            self._state = state

        # Publish outside lock (I/O — should not block other state readers)
        self.planner_status.publish(state)
        current_waypoint = self._tracker.current_waypoint
        wp_index = self._safe_nonnegative_int(self._tracker.wp_index)
        wp_total = self._safe_nonnegative_int(self._tracker.path_length)
        remaining_waypoints = max(0, wp_total - wp_index)
        self.mission_status.publish({
            "state": state,
            "replan_count": _replan_count,
            "wp_index": wp_index,
            "wp_total": wp_total,
            "remaining_waypoints": remaining_waypoints,
            "frame_id": self._planning_frame_id,
            "planning_frame_id": self._planning_frame_id,
            "odom_frame_id": self._odom_frame_id,
            "costmap_frame_id": self._costmap_frame_id,
            "goal_frame_id": self._goal_frame_id,
            "goal": self._point_summary(_goal),
            "current_waypoint": self._point_summary(current_waypoint),
            "distance_to_goal_m": self._distance_xy(self._robot_pos, _goal),
            "active_waypoint_distance_m": self._distance_xy(
                self._robot_pos,
                current_waypoint,
            ),
            "speed_scale": self._speed_scale,
            "speed_policy": {
                "scale": self._speed_scale,
                "mode": (
                    "restricted" if self._speed_scale < 0.5
                    else "cautious" if self._speed_scale < 1.0
                    else "normal"
                ),
                "reason": self._speed_policy_reason,
                "source": "localization_degeneracy",
                "applied": self._speed_scale < 1.0,
            },
            "plan_safety_policy": getattr(
                self._planner_svc,
                "_plan_safety_policy",
                "observe",
            ),
            "replan_on_costmap_update": self._replan_on_costmap_update,
            "last_plan_report": self._current_plan_report(),
            "map_artifact_gate": getattr(self._planner_svc, "map_artifact_gate", {}),
            "direct_goal_fallback": self._direct_goal_fallback_status,
            "external_strategy_path_control": self._external_strategy_path_control,
            "using_external_strategy_path": self._using_external_strategy_path,
            "accept_partial_goal_progress": self._accept_partial_goal_progress,
            "degeneracy": self._degen_level,
            "failure_reason": _failure_reason,
            "localization_paused": self._paused_for_localization,
            "localization_recovery_motion_hold": (
                self._localization_recovery_motion_hold
            ),
            "ts": time.time(),
        })

    def _get_state(self) -> str:
        """Thread-safe read of current mission state.

        Acquires the nav lock to ensure a consistent snapshot of self._state.
        Use this everywhere outside _set_state() to avoid cross-thread races
        with the recovery motion thread and plan preview executor.
        """
        with self._nav_lock:
            return self._state

    def _set_state_locked(self, new_state: str) -> None:
        """Thread-safe write of mission state without publish side-effects.

        Only sets the internal _state field under lock. For full FSM transitions
        (including legality checks and port publishing), use _set_state() instead.
        """
        with self._nav_lock:
            self._state = new_state

    def _get_goal(self) -> np.ndarray | None:
        """Thread-safe read of current navigation goal."""
        with self._nav_lock:
            _g = self._goal
            return _g.copy() if _g is not None else None

    def _set_goal(self, goal: np.ndarray | None) -> None:
        """Thread-safe write of navigation goal."""
        with self._nav_lock:
            self._goal = goal

    def _get_replan_count(self) -> int:
        """Thread-safe read of replan count."""
        with self._nav_lock:
            return self._replan_count

    def _set_replan_count(self, value: int) -> None:
        """Thread-safe write of replan count."""
        with self._nav_lock:
            self._replan_count = value

    def _get_failure_reason(self) -> str:
        """Thread-safe read of failure reason."""
        with self._nav_lock:
            return self._failure_reason

    def _set_failure_reason(self, reason: str) -> None:
        """Thread-safe write of failure reason."""
        with self._nav_lock:
            self._failure_reason = reason

    # ── Input handlers ────────────────────────────────────────────────────

    def _block_for_frame_mismatch(self, source: str, frame_id: str) -> None:
        expected_frame = self._frame_contract.expected_frame_label(source)
        self._failure_reason = (
            f"unsupported {source} frame {frame_id!r}; expected "
            f"{expected_frame!r}"
        )
        self.adapter_status.publish({
            "event": "navigation_blocked",
            "reason": "frame_mismatch",
            "source": source,
            "expected_frame": expected_frame,
            "received_frame": frame_id,
            "ts": time.time(),
        })
        self._tracker.clear()
        self._active_path_terminal_goal = None
        self._active_external_strategy_path = []
        self._clear_partial_goal_progress()
        self._publish_motion_stop()
        self._set_state(MissionState.FAILED)

    def _reject_goal_frame(self, frame_id: str, *, source: str) -> None:
        self._failure_reason = (
            f"unsupported {source} frame {frame_id!r}; expected "
            f"{self._planning_frame_id!r}"
        )
        self.adapter_status.publish({
            "event": "goal_rejected",
            "reason": "unsupported_frame",
            "source": source,
            "expected_frame": self._planning_frame_id,
            "received_frame": frame_id,
            "ts": time.time(),
        })
        self._tracker.clear()
        self._goal = None
        self._goal_frame_id = None
        self._active_path_terminal_goal = None
        self._active_external_strategy_path = []
        self._clear_partial_goal_progress()
        self._clear_deferred_empty_path()
        self._publish_motion_stop()
        if self._get_state() not in (
            MissionState.IDLE,
            MissionState.SUCCESS,
            MissionState.CANCELLED,
        ):
            self._set_state(MissionState.FAILED)
        else:
            self._set_state(self._get_state())

    def _reject_invalid_goal(
        self,
        *,
        source: str,
        reason: str = "invalid_coordinates",
        details: str = "invalid coordinates",
    ) -> None:
        self._failure_reason = f"invalid {source} coordinates"
        self.adapter_status.publish({
            "event": "goal_rejected",
            "reason": reason,
            "source": source,
            "details": details,
            "ts": time.time(),
        })
        self._tracker.clear()
        self._goal = None
        self._goal_frame_id = None
        self._active_path_terminal_goal = None
        self._active_external_strategy_path = []
        self._clear_partial_goal_progress()
        self._clear_deferred_empty_path()
        self._publish_motion_stop()
        if self._get_state() not in (
            MissionState.IDLE,
            MissionState.SUCCESS,
            MissionState.CANCELLED,
        ):
            self._set_state(MissionState.FAILED)
        else:
            self._set_state(self._get_state())

    def _on_goal(self, goal: PoseStamped) -> None:
        frame_id = self._frame_contract.goal_frame(goal, self._planning_frame_id)
        if frame_id != self._planning_frame_id:
            self._reject_goal_frame(frame_id, source="goal_pose")
            return
        try:
            new_goal = np.array([
                goal.pose.position.x,
                goal.pose.position.y,
                goal.pose.position.z,
            ], dtype=float)
        except (TypeError, ValueError):
            self._reject_invalid_goal(
                source="goal_pose",
                details="coordinates must be numeric",
            )
            return
        if new_goal.shape != (3,) or not np.all(np.isfinite(new_goal)):
            self._reject_invalid_goal(
                source="goal_pose",
                details="coordinates must be finite x/y/z values",
            )
            return
        if self._should_ignore_goal_update(new_goal):
            return
        self._clear_localization_pause_for_explicit_action(
            reason="new_goal",
            clear_goal=False,
        )
        self._clear_partial_goal_progress()
        self._clear_deferred_empty_path()
        self._using_external_strategy_path = False
        self._goal = new_goal
        self._goal_frame_id = frame_id
        self._set_replan_count(0)
        self._plan()

    def _should_ignore_goal_update(self, new_goal: np.ndarray) -> bool:
        if self._should_ignore_completed_partial_goal(new_goal):
            return True
        if self._goal is None:
            return False
        if self._get_state() not in (MissionState.PLANNING, MissionState.EXECUTING, MissionState.PATROLLING):
            return False
        dist = self._distance_xyz_or_xy(new_goal, self._goal)
        if dist > self._goal_update_epsilon:
            return False
        self.adapter_status.publish({
            "event": "goal_update_ignored",
            "distance": dist,
            "goal": [float(new_goal[0]), float(new_goal[1]), float(new_goal[2])],
            "ts": time.time(),
        })
        return True

    def _should_ignore_completed_partial_goal(self, new_goal: np.ndarray) -> bool:
        if not self._accept_partial_goal_progress:
            return False
        if (
            self._partial_progress_completed_goal is None
            or self._partial_progress_completed_terminal is None
        ):
            return False
        if self._partial_goal_repeat_ignore_window_s > 0.0:
            age_s = time.time() - self._partial_progress_completed_ts
            if age_s > self._partial_goal_repeat_ignore_window_s:
                self._clear_partial_goal_progress()
                return False
        try:
            goal_delta = self._distance_xyz_or_xy(
                new_goal,
                self._partial_progress_completed_goal,
            )
            robot_terminal_delta = self._distance_xyz_or_xy(
                self._robot_pos,
                self._partial_progress_completed_terminal,
            )
        except (TypeError, ValueError):
            return False
        if goal_delta > self._goal_update_epsilon:
            return False
        final_threshold = float(getattr(self._tracker, "_final_threshold", 0.0))
        terminal_hold_radius = max(0.75, final_threshold * 2.0)
        if robot_terminal_delta > terminal_hold_radius:
            return False
        self.adapter_status.publish({
            "event": "partial_goal_repeat_ignored",
            "goal": self._point_summary(new_goal),
            "completed_goal": self._point_summary(
                self._partial_progress_completed_goal
            ),
            "completed_terminal": self._point_summary(
                self._partial_progress_completed_terminal
            ),
            "robot_terminal_delta_m": round(robot_terminal_delta, 3),
            "ts": time.time(),
        })
        return True

    def _on_instruction(self, text: str) -> None:
        logger.info("NavigationModule received instruction: %s", text[:50])

    def _on_stop(self, level: int) -> None:
        if level >= 2:
            self._clear_teleop_resume_state()
            self._tracker.clear()
            self._publish_motion_stop()
            self._using_external_strategy_path = False
            self._active_external_strategy_path = []
            self._clear_partial_goal_progress()
            self._set_state(MissionState.IDLE)
            self.adapter_status.publish({
                "event": "safety_stop",
                "level": int(level),
                "action": "mission_cleared",
                "ts": time.time(),
            })
        elif level == 1:
            self.adapter_status.publish({
                "event": "safety_soft_stop",
                "level": int(level),
                "action": "mission_held",
                "ts": time.time(),
            })
        elif level <= 0:
            self.adapter_status.publish({
                "event": "safety_clear",
                "level": int(level),
                "action": "mission_retained",
                "ts": time.time(),
            })

    def _publish_motion_stop(self) -> None:
        self._request_recovery_stop()
        self.clear_path.publish(True)
        self.recovery_cmd_vel.publish(Twist.zero())

    def _clear_teleop_resume_state(self) -> None:
        self._paused_for_teleop = False
        self._pre_teleop_goal = None
        self._pre_teleop_state = None

    def _on_teleop_active(self, active: bool) -> None:
        """Pause navigation when teleop engages, resume when released."""
        if active and not self._paused_for_teleop:
            # Save current mission state so we can resume
            if self._get_state() in (MissionState.EXECUTING, MissionState.PATROLLING):
                self._pre_teleop_goal = self._goal.copy() if self._goal is not None else None
                self._pre_teleop_state = self._get_state()
                self._tracker.clear()
                self._publish_motion_stop()
                self._set_state(MissionState.IDLE)
                logger.info("NavigationModule: paused for teleop (saved goal)")
            self._paused_for_teleop = True

        elif not active and self._paused_for_teleop:
            self._paused_for_teleop = False
            if (self._pre_teleop_goal is not None
                    and self._get_state() == MissionState.IDLE):
                if self._auto_resume_after_teleop:
                    logger.info("NavigationModule: teleop released, resuming navigation")
                    self._goal = self._pre_teleop_goal
                    self._pre_teleop_goal = None
                    self._pre_teleop_state = None
                    self._plan()
                else:
                    logger.info(
                        "NavigationModule: teleop released, explicit resume required"
                    )
                    self.adapter_status.publish({
                        "event": "teleop_release_resume_required",
                        "ts": time.time(),
                    })
                    self._pre_teleop_goal = None
                    self._pre_teleop_state = None
            else:
                logger.info("NavigationModule: teleop released, no mission to resume")
                self._pre_teleop_goal = None
                self._pre_teleop_state = None

    def _on_costmap(self, data: dict) -> None:
        self._costmap_frame_id = str(data.get("frame_id") or self._planning_frame_id)
        self._frame_contract.report_frame_mismatch("costmap", self._costmap_frame_id)
        if self._frame_contract.is_frame_mismatch(self._costmap_frame_id, source="costmap"):
            if self._frame_contract.has_motion_artifacts(
                self._get_state(), self._tracker.path_length, self._goal, self._patrol_goals,
            ):
                self._block_for_frame_mismatch("costmap", self._costmap_frame_id)
            return
        grid = data.get("grid")
        if grid is None:
            return
        if self._replan_on_costmap_update:
            self._planner_svc.update_map(
                grid,
                resolution=data.get("resolution", 0.2),
                origin=data.get("origin"),
            )
        if (self._get_state() in (MissionState.EXECUTING, MissionState.PATROLLING)
                and self._goal is not None
                and not self._using_external_strategy_path
                and self._replan_on_costmap_update
                and not self._is_recovery_active()
                and time.time() - self._last_costmap_replan_time > 3.0):
            self._last_costmap_replan_time = time.time()
            self._plan()
        elif (
            self._get_state() == MissionState.PLANNING
            and self._deferred_empty_path_first_ts > 0.0
            and self._goal is not None
            and not self._using_external_strategy_path
            and self._replan_on_costmap_update
            and time.time() - self._last_costmap_replan_time
            > self._empty_path_retry_interval_s
        ):
            self._last_costmap_replan_time = time.time()
            self._plan()

    def _on_cancel(self, msg: str) -> None:
        had_localization_pause = (
            self._paused_for_localization
            or self._pre_pause_state is not None
            or self._localization_recovery_motion_hold
        )
        self._tracker.clear()
        self._patrol_goals.clear()
        self._patrol_index = 0
        self._goal = None
        self._goal_frame_id = None
        self._clear_teleop_resume_state()
        self._active_path_terminal_goal = None
        self._using_external_strategy_path = False
        self._active_external_strategy_path = []
        self._clear_partial_goal_progress()
        self._clear_deferred_empty_path()
        self._failure_reason = f"cancelled: {msg}" if msg else "cancelled"
        self._publish_motion_stop()
        if self._get_state() in (MissionState.IDLE, MissionState.CANCELLED) \
                and not had_localization_pause:
            return
        self._clear_localization_pause_for_explicit_action(
            reason="cancel",
            clear_goal=True,
        )
        if self._get_state() in (MissionState.IDLE, MissionState.CANCELLED):
            logger.info("Mission cancelled while idle/paused: %s", msg)
            return
        self._set_state(MissionState.CANCELLED)
        logger.info("Mission cancelled: %s", msg)

    def _on_map_frame_jump(self, event: dict) -> None:
        """SlamBridge detected a map↔odom TF discontinuity (P4).

        The cached global path / waypoint were planned in the *old* map frame.
        Force an immediate replan so we don't drive the robot toward a
        coordinate that no longer matches reality. Costmap is in odom frame
        so it remains valid; ESDF / OccupancyGrid handle their own clear via
        their own subscription to this event.
        """
        if not isinstance(event, dict):
            return
        dt_m = event.get("dt_m", 0.0)
        dyaw = event.get("dyaw_deg", 0.0)
        # Only act if we have an active mission — idle robot doesn't care.
        if self._get_state() in (MissionState.EXECUTING, MissionState.PATROLLING) \
                and self._goal is not None:
            logger.warning(
                "TF jump (Δt=%.2fm Δyaw=%.1f°) → forced replan",
                dt_m, dyaw)
            self._publish_motion_stop()
            self._tracker.clear()  # invalidate current waypoint tracking
            self._plan()

    def _on_localization_status(self, msg: dict) -> None:
        prev = self._loc_state
        self._loc_state = msg.get("state", "UNINIT")
        self._loc_confidence = msg.get("confidence", 0.0)
        self._degen_level = msg.get("degeneracy", "NONE")
        motion_hold_required = self._localization_status_requires_motion_hold(msg)

        if motion_hold_required:
            self._localization_recovery_motion_hold = True
            if self._get_state() in (MissionState.EXECUTING, MissionState.PATROLLING):
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
            if self._get_state() in (MissionState.EXECUTING, MissionState.PATROLLING):
                self._pause_for_localization(reason="localization lost")
                logger.warning("Navigation PAUSED: localization lost")

        elif self._loc_state == "TRACKING" and self._paused_for_localization:
            if self._localization_recovery_motion_hold:
                self._failure_reason = (
                    "localization recovered; explicit goal required"
                )
                if self._get_state() != MissionState.IDLE:
                    self._set_state(MissionState.IDLE)
                logger.warning(
                    "Navigation HOLD: localization recovered after automatic "
                    "chain restart; waiting for explicit goal"
                )
            else:
                self._paused_for_localization = False
                if self._pre_pause_state and self._goal is not None:
                    self._set_state(self._pre_pause_state)
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
            self._set_state(MissionState.IDLE)

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

    @staticmethod
    def _localization_status_requires_motion_hold(msg: dict) -> bool:
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

        FALLBACK_GNSS_ONLY → 0.3x (cautious — SLAM has been DEGRADED for >10s
                                   with healthy GNSS; we are essentially flying
                                   on absolute fixes plus dead reckoning).
        DEGEN SEVERE       → 0.4x
        DEGEN MILD         → 0.7x
        otherwise          → 1.0x

        CRITICAL is handled by the DEGRADED→LOST path above (mission pauses).
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

    def _on_patrol_goals(self, goals: list) -> None:
        if not goals:
            return
        parsed_goals: list[np.ndarray] = []
        patrol_loop = False
        for g in goals:
            if isinstance(g, dict):
                frame_id = str(g.get("frame_id") or self._planning_frame_id)
                if frame_id != self._planning_frame_id:
                    self._reject_goal_frame(frame_id, source="patrol_goals")
                    return
                try:
                    goal = np.array([
                        g["x"],
                        g["y"],
                        g["z"] if "z" in g else self._resolve_goal_z(None),
                    ], dtype=float)
                except (KeyError, TypeError, ValueError):
                    self._reject_invalid_goal(source="patrol_goals")
                    return
                if goal.shape != (3,) or not np.all(np.isfinite(goal)):
                    self._reject_invalid_goal(source="patrol_goals")
                    return
                parsed_goals.append(goal)
                if g.get("loop"):
                    patrol_loop = True
            elif isinstance(g, (list, tuple)) and len(g) >= 2:
                try:
                    goal = np.array([
                        g[0],
                        g[1],
                        g[2] if len(g) > 2 else self._resolve_goal_z(None),
                    ], dtype=float)
                except (TypeError, ValueError):
                    self._reject_invalid_goal(source="patrol_goals")
                    return
                if goal.shape != (3,) or not np.all(np.isfinite(goal)):
                    self._reject_invalid_goal(source="patrol_goals")
                    return
                parsed_goals.append(goal)
        if parsed_goals:
            if self._same_patrol_goals(parsed_goals, loop=patrol_loop):
                return
            self._patrol_goals = parsed_goals
            self._patrol_loop = patrol_loop
            self._patrol_index = 0
            if self._external_strategy_path_control:
                self._start_external_strategy_path(parsed_goals)
                return
            self._goal = self._patrol_goals[0]
            self._goal_frame_id = self._planning_frame_id
            self._set_replan_count(0)
            self._set_state(MissionState.PATROLLING)
            logger.info(
                "Patrol started: %d goals, loop=%s",
                len(self._patrol_goals), self._patrol_loop,
            )
            self._plan()

    def _start_external_strategy_path(self, goals: list[np.ndarray]) -> None:
        """Execute an externally planned strategy path through LingTu tracking."""
        try:
            path = self._validate_planned_path(goals)
            path = self._reanchor_external_strategy_path(path)
        except RuntimeError as exc:
            self._failure_reason = f"invalid external strategy path: {exc}"
            self._tracker.clear()
            self._publish_motion_stop()
            self._set_state(MissionState.FAILED)
            return
        if len(path) < 2:
            self.adapter_status.publish({
                "event": "external_strategy_path_ignored",
                "reason": "path_too_short",
                "points": len(path),
                "ts": time.time(),
            })
            return

        self._using_external_strategy_path = True
        self._clear_partial_goal_progress()
        self._goal = np.asarray(path[-1][:3], dtype=float).copy()
        self._goal_frame_id = self._planning_frame_id
        self._active_path_terminal_goal = self._goal.copy()
        self._active_external_strategy_path = [
            np.asarray(point[:3], dtype=float).copy() for point in path
        ]
        self._failure_reason = ""
        self._direct_goal_fallback_status = None
        self._mission_start_time = time.time()
        self._last_costmap_replan_time = time.time()
        self._set_replan_count(0)

        self._tracker.reset(path, self._robot_pos, self._robot_yaw)
        self._tracker.update(self._robot_pos, self._robot_yaw)
        if self._tracker.current_waypoint is None:
            self.adapter_status.publish({
                "event": "external_strategy_path_ignored",
                "reason": "already_complete",
                "points": len(path),
                "ts": time.time(),
            })
            return

        self.global_path.publish(path)
        self._set_state(MissionState.EXECUTING)
        self.adapter_status.publish({
            "event": "external_strategy_path_control",
            "points": len(path),
            "goal": self._point_summary(self._goal),
            "current_waypoint": self._point_summary(self._tracker.current_waypoint),
            "ts": time.time(),
        })
        self._publish_waypoint()

    def _reanchor_external_strategy_path(
        self,
        path: list[np.ndarray],
    ) -> list[np.ndarray]:
        """Return the forward path segment anchored near current odometry.

        External strategy paths are generated by another ROS graph. Before
        tracking them directly, require at least one path point near the
        current robot pose and start tracking from that segment.
        """
        if len(path) < 2:
            raise RuntimeError("path_too_short")
        robot = np.asarray(self._robot_pos[:3], dtype=float)
        if not np.all(np.isfinite(robot[:2])):
            raise RuntimeError("missing_current_odometry")
        distances = [
            float(np.linalg.norm(np.asarray(point[:2], dtype=float) - robot[:2]))
            for point in path
        ]
        nearest_index = min(range(len(distances)), key=distances.__getitem__)
        nearest_distance = distances[nearest_index]
        if nearest_distance > max(0.0, self._external_strategy_start_tolerance_m):
            raise RuntimeError(
                "path_not_anchored_near_current_odom "
                f"(nearest={nearest_distance:.2f}m)"
            )
        anchored = [np.asarray(point[:3], dtype=float).copy() for point in path[nearest_index:]]
        if len(anchored) < 2:
            raise RuntimeError("path_has_no_forward_segment_after_odom_anchor")
        if float(np.linalg.norm(anchored[0][:2] - robot[:2])) > 0.05:
            anchored.insert(0, robot.copy())
        return anchored

    def _same_patrol_goals(self, goals: list[np.ndarray], *, loop: bool) -> bool:
        if self._get_state() not in (
            MissionState.PLANNING,
            MissionState.EXECUTING,
            MissionState.PATROLLING,
        ):
            return False
        if loop != self._patrol_loop or len(goals) != len(self._patrol_goals):
            return False
        if not goals:
            return False
        return all(
            np.linalg.norm(
                np.asarray(a[:3], dtype=float) - np.asarray(b[:3], dtype=float)
            ) <= 0.05
            for a, b in zip(goals, self._patrol_goals)
        )

    def _on_odom(self, odom: Odometry) -> None:
        self._odom_frame_id = str(getattr(odom, "frame_id", "") or "unknown")
        self._frame_contract.report_frame_mismatch("odometry", self._odom_frame_id)
        if self._frame_contract.is_frame_mismatch(self._odom_frame_id, source="odometry"):
            if self._frame_contract.has_motion_artifacts(
                self._get_state(), self._tracker.path_length, self._goal, self._patrol_goals,
            ):
                self._block_for_frame_mismatch("odometry", self._odom_frame_id)
            return
        self._robot_pos = np.array([
            odom.pose.position.x,
            odom.pose.position.y,
            odom.pose.position.z,
        ])
        try:
            yaw = float(odom.yaw)
            if math.isfinite(yaw):
                self._robot_yaw = yaw
        except (AttributeError, TypeError, ValueError):
            pass

        if self._paused_for_localization:
            return

        if self._get_state() not in (MissionState.EXECUTING, MissionState.PATROLLING):
            return

        status = self._tracker.update(self._robot_pos, self._robot_yaw)

        if status.event == EV_PATH_COMPLETE:
            if self._should_continue_after_partial_path():
                self.adapter_status.publish({
                    "event": "partial_path_complete_replan",
                    "goal": self._point_summary(self._goal),
                    "path_terminal_goal": self._point_summary(
                        self._active_path_terminal_goal
                    ),
                    "distance_to_goal_m": self._distance_xy(
                        self._robot_pos,
                        self._goal,
                    ),
                    "ts": time.time(),
                })
                self._plan()
                return
            partial_status = self._partial_path_terminal_status()
            if (
                self._accept_partial_goal_progress
                and partial_status.get("partial")
            ):
                self._record_partial_goal_progress_complete()
                self.adapter_status.publish({
                    "event": "partial_goal_progress_complete",
                    "original_goal": self._point_summary(self._goal),
                    "path_terminal_goal": self._point_summary(
                        self._active_path_terminal_goal
                    ),
                    "terminal_gap_m": partial_status.get("terminal_gap_m"),
                    "distance_to_goal_m": partial_status.get("robot_gap_m"),
                    "selected_planner": partial_status.get("selected_planner"),
                    "reason": partial_status.get("reason"),
                    "ts": time.time(),
                })
            if self._get_state() == MissionState.PATROLLING and self._patrol_goals:
                if self._advance_patrol():
                    return
            self._publish_motion_stop()
            self._set_state(MissionState.SUCCESS)
            return

        elif status.event == EV_WAYPOINT_REACHED:
            self.adapter_status.publish({
                "event": "waypoint_reached",
                "index": status.wp_index,
                "total": status.wp_total,
            })
            self._publish_waypoint()

        elif status.event == EV_STUCK_WARN:
            self.adapter_status.publish({
                "event": "WARN_STUCK",
                "elapsed": round(time.time() - self._mission_start_time, 1),
            })

        elif status.event == EV_STUCK:
            if self._get_replan_count() < self._max_replan:
                self._set_replan_count(self._get_replan_count() + 1)
                if self._using_external_strategy_path:
                    self._execute_recovery_motion(post_action="external_strategy")
                else:
                    self._execute_recovery_motion(post_action="replan")
                return
            else:
                self._failure_reason = "stuck after max replans"
                self._set_state(MissionState.STUCK)

        # Mission timeout — only applies while mission is active
        if (self._get_state() in (MissionState.EXECUTING, MissionState.PATROLLING)
                and self._mission_start_time > 0
                and time.time() - self._mission_start_time > self._mission_timeout):
            self._failure_reason = f"mission timeout ({self._mission_timeout}s)"
            self._set_state(MissionState.FAILED)
            logger.warning("Mission timeout after %.0fs", self._mission_timeout)

    # ── Recovery motion ───────────────────────────────────────────────────

    def _on_traversability(self, msg: dict) -> None:
        """W2-8: update latest terrain class from TerrainModule."""
        if not isinstance(msg, dict):
            return
        klass = msg.get("traversability_class") or msg.get("class")
        if klass:
            self._latest_traversability_class = str(klass)

    # Recovery strategies keyed by terrain class. Each entry specifies the
    # backup, rotate, and optional forward-nudge phases — setting a phase's
    # duration to 0 skips it. The default strategy (1.5s backup + 1.5s rotate)
    # kicks in for "unknown" or any class not listed below.
    _RECOVERY_STRATEGIES: dict[str, dict[str, Any]] = {
        "cliff": {
            "strategy": "rotate_only",
            "backup_speed": 0.0, "backup_duration": 0.0,
            "rotate_speed": 0.3, "rotate_duration": 2.5,
            "forward_speed": 0.0, "forward_duration": 0.0,
        },
        "unsafe_forward": {
            "strategy": "rotate_only",
            "backup_speed": 0.0, "backup_duration": 0.0,
            "rotate_speed": 0.3, "rotate_duration": 2.5,
            "forward_speed": 0.0, "forward_duration": 0.0,
        },
        "narrow": {
            "strategy": "short_backup_rotate",
            "backup_speed": -0.15, "backup_duration": 0.8,
            "rotate_speed": 0.5,   "rotate_duration": 0.8,
            "forward_speed": 0.0,  "forward_duration": 0.0,
        },
        "corridor": {
            "strategy": "short_backup_rotate",
            "backup_speed": -0.15, "backup_duration": 0.8,
            "rotate_speed": 0.5,   "rotate_duration": 0.8,
            "forward_speed": 0.0,  "forward_duration": 0.0,
        },
        "stuck_in_soft": {
            "strategy": "long_backup_nudge",
            "backup_speed": -0.15, "backup_duration": 2.0,
            "rotate_speed": 0.0,   "rotate_duration": 0.0,
            "forward_speed": 0.15, "forward_duration": 0.5,
        },
        "grip_loss": {
            "strategy": "long_backup_nudge",
            "backup_speed": -0.15, "backup_duration": 2.0,
            "rotate_speed": 0.0,   "rotate_duration": 0.0,
            "forward_speed": 0.15, "forward_duration": 0.5,
        },
    }

    _DEFAULT_RECOVERY_STRATEGY: dict[str, Any] = {
        "strategy": "default_backup_rotate",
        "backup_speed": -0.2, "backup_duration": 1.5,
        "rotate_speed": 0.5,  "rotate_duration": 1.5,
        "forward_speed": 0.0, "forward_duration": 0.0,
    }

    def _request_recovery_stop(self, join_timeout: float = 0.2) -> None:
        with self._recovery_lock:
            self._recovery_stop_event.set()
            thread = self._recovery_thread

        if (
            thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(timeout=join_timeout)

        with self._recovery_lock:
            if self._recovery_thread is thread and (
                thread is None or not thread.is_alive()
            ):
                self._recovery_thread = None

    def _execute_recovery_motion(self, post_action: str = "replan") -> bool:
        """W2-8: context-aware stuck recovery.

        Picks a recovery strategy based on the latest terrain class reported
        by TerrainModule. Emits adapter_status {"event":"recovery_started",
        "strategy":..., "reason":...} so operators can see which branch ran.
        """
        klass = getattr(self, "_latest_traversability_class", "unknown")
        strat = self._RECOVERY_STRATEGIES.get(klass, self._DEFAULT_RECOVERY_STRATEGY)

        self.adapter_status.publish({
            "event":    "recovery_started",
            "strategy": strat["strategy"],
            "reason":   klass,
            "ts":       time.time(),
        })
        logger.info(
            "Recovery: strategy=%s, reason=%s", strat["strategy"], klass,
        )
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
            """Publish a fixed cmd_vel for `duration` seconds at step_hz."""
            if duration <= 0.0:
                return True
            steps = max(1, int(math.ceil(duration * step_hz)))
            for _ in range(steps):
                if stop_event.is_set() or self._get_state() != MissionState.EXECUTING:
                    return False
                self.recovery_cmd_vel.publish(Twist(
                    linear=Vector3(linear_x, 0.0, 0.0),
                    angular=Vector3(0.0, 0.0, angular_z),
                ))
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

        # Backup phase
        if not _drive(strat["backup_speed"], 0.0, strat["backup_duration"]):
            _finish(False)
            return

        # Rotate phase — alternate direction per replan to avoid dead-lock
        if strat["rotate_duration"] > 0.0 and strat["rotate_speed"] != 0.0:
            # Thread-safe read: recovery thread vs main thread replan count updates
            direction = 1.0 if self._get_replan_count() % 2 == 1 else -1.0
            if not _drive(0.0, strat["rotate_speed"] * direction, strat["rotate_duration"]):
                _finish(False)
                return

        # Forward nudge phase (slip/grip terrain)
        if not _drive(strat["forward_speed"], 0.0, strat["forward_duration"]):
            _finish(False)
            return

        # Stop
        _finish(True)

    def _is_recovery_active(self) -> bool:
        """True if a recovery-motion thread is currently driving cmd_vel."""
        with self._recovery_lock:
            return bool(
                self._recovery_thread is not None
                and self._recovery_thread.is_alive()
            )

    def _finish_recovery_motion(self, post_action: str, reason: str) -> None:
        if post_action == "none":
            return
        if self._get_state() != MissionState.EXECUTING:
            return
        if post_action == "external_strategy":
            self._tracker.pause()
            self._republish_external_strategy_path()
            logger.warning(
                "Stuck detected, recovery motion; retaining external "
                "strategy path (%d/%d)",
                self._get_replan_count(),
                self._max_replan,
            )
            self.adapter_status.publish({
                "event": "external_strategy_path_stuck_recovery",
                "recovery_count": self._get_replan_count(),
                "max_recovery_count": self._max_replan,
                "remaining_waypoints": max(
                    0,
                    self._tracker.path_length - self._tracker.wp_index,
                ),
                "reason": reason,
                "ts": time.time(),
            })
            return
        if post_action == "replan":
            logger.warning(
                "Stuck detected, recovery motion + replan (%d/%d)",
                self._get_replan_count(), self._max_replan,
            )
            self._plan()

    # ── Planning ──────────────────────────────────────────────────────────

    @staticmethod
    def _preview_path_point(
        point: Any,
        *,
        frame_id: str = PLANNING_FRAME_ID,
        ts: float | None = None,
        index: int | None = None,
    ) -> dict[str, Any]:
        arr = np.asarray(point, dtype=float).reshape(-1)
        metadata: dict[str, Any] = {}
        if index is not None:
            metadata["index"] = index
        return {
            "x": float(arr[0]) if arr.size >= 1 and np.isfinite(arr[0]) else 0.0,
            "y": float(arr[1]) if arr.size >= 2 and np.isfinite(arr[1]) else 0.0,
            "z": float(arr[2]) if arr.size >= 3 and np.isfinite(arr[2]) else 0.0,
            "frame_id": frame_id,
            "ts": ts,
            "metadata": metadata,
        }

    @staticmethod
    def _preview_path_array(point: Any) -> np.ndarray | None:
        try:
            arr = np.asarray(point, dtype=float).reshape(-1)
        except (TypeError, ValueError):
            return None
        if arr.size < 2 or not np.all(np.isfinite(arr)):
            return None
        if arr.size < 3:
            return np.pad(arr[:2], (0, 1), constant_values=0.0)
        return arr[:3].copy()

    @staticmethod
    def _preview_path_distance(path: list[np.ndarray]) -> float:
        if len(path) < 2:
            return 0.0
        total = 0.0
        for prev, curr in zip(path, path[1:]):
            a = np.asarray(prev, dtype=float).reshape(-1)
            b = np.asarray(curr, dtype=float).reshape(-1)
            if a.size >= 2 and b.size >= 2:
                with np.errstate(over="ignore", invalid="ignore"):
                    total += float(np.linalg.norm(b[:2] - a[:2]))
        return total

    def preview_plan(self, x: float, y: float, z: float = 0.0) -> dict[str, Any]:
        """Return a client-facing path preview without changing mission state.

        This intentionally avoids _set_state(), tracker updates, and all output
        ports so App/Web clients can validate a candidate goal before sending a
        real navigation command.
        """
        ts = time.time()
        planner = self._planner_svc
        start = np.asarray(self._robot_pos, dtype=float).reshape(-1)[:3].copy()
        if start.size < 3:
            start = np.pad(start, (0, 3 - start.size), constant_values=0.0)
        goal = np.array([x, y, z], dtype=float)
        result: dict[str, Any] = {
            "schema_version": 1,
            "ok": True,
            "feasible": False,
            "frame_id": self._planning_frame_id,
            "start": self._preview_path_point(
                start, frame_id=self._planning_frame_id, ts=ts
            ),
            "goal": self._preview_path_point(
                goal, frame_id=self._planning_frame_id, ts=ts
            ),
            "adjusted_goal": None,
            "path": [],
            "count": 0,
            "distance_m": None,
            "plan_ms": None,
            "planner": getattr(planner, "_planner_name", None),
            "selected_planner": None,
            "plan_safety_policy": getattr(planner, "_plan_safety_policy", None),
            "path_safety": None,
            "fallback_reason": "",
            "rejected_plans": [],
            "source": "navigation_preview",
            "reasons": [],
            "error": None,
            "ts": ts,
        }

        if not np.all(np.isfinite(start)):
            result["reasons"] = ["odometry_invalid"]
            result["error"] = "current robot position is not finite"
            return result
        frame_blocker = self._frame_contract.planning_frame_blocker(
            self._odom_frame_id, self._costmap_frame_id,
        )
        if frame_blocker is not None:
            source, frame_id = frame_blocker
            result["reasons"] = ["frame_mismatch"]
            result["error"] = (
                f"unsupported {source} frame {frame_id!r}; expected "
                f"{self._planning_frame_id!r}"
            )
            return result
        if not np.all(np.isfinite(goal)):
            result["ok"] = False
            result["reasons"] = ["goal_invalid"]
            result["error"] = "goal position is not finite"
            return result
        try:
            planner_ready = bool(getattr(planner, "is_ready", False))
            planner_has_map = bool(getattr(planner, "has_map", False))
        except Exception as exc:
            result["reasons"] = ["planner_status_unavailable"]
            result["error"] = str(exc)
            return result
        if not planner_ready:
            reasons = ["planner_not_ready"]
            if not planner_has_map:
                reasons.append("map_unavailable")
            result["reasons"] = reasons
            return result

        with np.errstate(over="ignore", invalid="ignore"):
            start_goal_delta = float(np.linalg.norm(goal[:2] - start[:2]))
        if not math.isfinite(start_goal_delta):
            result["reasons"] = ["goal_invalid"]
            result["error"] = "goal distance from current position is not finite"
            return result
        if start_goal_delta <= 0.05:
            result.update({
                "feasible": True,
                "path": [
                    self._preview_path_point(
                        start,
                        frame_id=self._planning_frame_id,
                        ts=ts,
                        index=0,
                    )
                ],
                "count": 1,
                "distance_m": 0.0,
                "plan_ms": 0.0,
                "reasons": ["already_at_goal"],
            })
            return result

        try:
            path, plan_ms = self._preview_plan_with_timeout(planner, start, goal)
        except TimeoutError as exc:
            logger.warning("NavigationModule: plan preview timed out: %s", exc)
            result["reasons"] = ["planning_timeout"]
            result["error"] = str(exc)
            return result
        except _PlanPreviewBusy as exc:
            result["reasons"] = ["planning_preview_busy"]
            result["error"] = str(exc)
            return result
        except Exception as exc:
            logger.warning("NavigationModule: plan preview failed: %s", exc)
            result["reasons"] = ["planning_failed"]
            result["error"] = str(exc)
            result.update(self._preview_plan_report_fields(planner))
            return result

        try:
            path_points = list(path) if path is not None else []
        except TypeError:
            result["reasons"] = ["planner_returned_invalid_path"]
            result["error"] = "planner returned a non-iterable path"
            return result
        if not path_points:
            result["reasons"] = ["empty_path"]
            result["error"] = "planner returned empty path"
            return result

        path_arrays: list[np.ndarray] = []
        for point in path_points:
            arr = self._preview_path_array(point)
            if arr is None:
                result["reasons"] = ["planner_returned_nonfinite_path"]
                result["error"] = "planner returned a non-finite path point"
                return result
            path_arrays.append(arr)

        try:
            plan_ms_value = float(plan_ms)
        except (TypeError, ValueError):
            result["reasons"] = ["planner_returned_invalid_timing"]
            result["error"] = "planner returned an invalid plan_ms value"
            return result
        if not math.isfinite(plan_ms_value):
            result["reasons"] = ["planner_returned_invalid_timing"]
            result["error"] = "planner returned a non-finite plan_ms value"
            return result

        distance_m = self._preview_path_distance(path_arrays)
        if not math.isfinite(distance_m):
            result["reasons"] = ["planner_returned_invalid_distance"]
            result["error"] = "planner returned a path with non-finite distance"
            return result

        serialized = [
            self._preview_path_point(
                point,
                frame_id=self._planning_frame_id,
                ts=ts,
                index=index,
            )
            for index, point in enumerate(path_arrays)
        ]
        final = path_arrays[-1]
        adjusted_goal = None
        with np.errstate(over="ignore", invalid="ignore"):
            goal_delta = float(np.linalg.norm(final[:2] - goal[:2]))
        if not math.isfinite(goal_delta):
            result["reasons"] = ["planner_returned_invalid_distance"]
            result["error"] = "planner returned a path with non-finite goal distance"
            return result
        if goal_delta > 0.05:
            adjusted_goal = self._preview_path_point(
                final,
                frame_id=self._planning_frame_id,
                ts=ts,
            )

        result.update({
            "feasible": True,
            "path": serialized,
            "count": len(serialized),
            "distance_m": distance_m,
            "plan_ms": plan_ms_value,
            "adjusted_goal": adjusted_goal,
            "reasons": ["goal_adjusted"] if adjusted_goal is not None else [],
        })
        result.update(self._preview_plan_report_fields(planner))
        return result

    @staticmethod
    def _preview_plan_report_fields(planner: Any) -> dict[str, Any]:
        """Expose planner safety/selection diagnostics without changing state."""
        report = NavigationModule._planner_last_plan_report(planner)
        selected = report.get("selected_planner") or getattr(
            planner,
            "_planner_name",
            None,
        )
        return {
            "planner": selected or getattr(planner, "_planner_name", None),
            "selected_planner": selected,
            "plan_safety_policy": (
                report.get("policy")
                or getattr(planner, "_plan_safety_policy", None)
            ),
            "path_safety": report.get("selected_path_safety"),
            "fallback_reason": report.get("fallback_reason", ""),
            "rejected_plans": list(report.get("rejected_plans") or []),
        }

    @staticmethod
    def _planner_last_plan_report(planner: Any) -> dict[str, Any]:
        report = getattr(planner, "last_plan_report", {}) or {}
        if callable(report):
            report = report()
        if not isinstance(report, dict):
            return {}
        return dict(report)

    def _current_plan_report(self) -> dict[str, Any]:
        if self._using_external_strategy_path:
            return {
                "primary_planner": "tare_external",
                "selected_planner": "external_strategy_path",
                "fallback_used": False,
                "path_safety_ok": None,
                "external_strategy_path_control": True,
                "points": len(self._active_external_strategy_path),
            }
        report = self._planner_last_plan_report(self._planner_svc)
        if self._deferred_empty_path_first_ts > 0.0:
            deferred_reason = str(
                report.get("fallback_reason")
                or self._failure_reason
                or "empty path deferred"
            )
            sanitized = dict(report)
            sanitized["fallback_reason"] = ""
            sanitized["rejected_plans"] = []
            sanitized["deferred_planning"] = {
                "active": True,
                "reason": deferred_reason,
                "attempts": self._deferred_empty_path_attempts,
                "waited_s": round(time.time() - self._deferred_empty_path_first_ts, 3),
                "retry_interval_s": self._empty_path_retry_interval_s,
                "retry_timeout_s": self._empty_path_retry_timeout_s,
            }
            return sanitized
        return report

    def _republish_external_strategy_path(self) -> None:
        if self._active_external_strategy_path:
            self.global_path.publish([
                np.asarray(point[:3], dtype=float).copy()
                for point in self._active_external_strategy_path
            ])
        self._publish_waypoint()

    def _preview_plan_with_timeout(
        self,
        planner: Any,
        start: np.ndarray,
        goal: np.ndarray,
    ) -> tuple[Any, Any]:
        if not self._preview_planner_lock.acquire(blocking=False):
            raise _PlanPreviewBusy("another plan preview is already running")

        def _run() -> tuple[Any, Any]:
            try:
                return planner.plan(start, goal)
            finally:
                self._preview_planner_lock.release()

        future = self._preview_executor.submit(_run)
        try:
            return future.result(timeout=max(0.001, self._preview_timeout_s))
        except concurrent.futures.TimeoutError as exc:
            if future.cancel():
                self._preview_planner_lock.release()
            raise TimeoutError(
                f"planner preview exceeded {self._preview_timeout_s:.1f}s"
            ) from exc

    def _plan(self) -> None:
        # Snapshot goal under lock for cross-thread consistency
        # (_plan is called from both the dispatch thread and recovery thread).
        _goal = self._get_goal()
        if _goal is None:
            self._set_state(MissionState.FAILED)
            return

        frame_blocker = self._frame_contract.planning_frame_blocker(
            self._odom_frame_id, self._costmap_frame_id,
        )
        if frame_blocker is not None:
            self._block_for_frame_mismatch(*frame_blocker)
            return

        planned_state = (
            MissionState.PATROLLING
            if self._patrol_goals
            and _goal is not None
            and self._get_state() in (
                MissionState.PATROLLING,
                MissionState.PLANNING,
                MissionState.FAILED,
            )
            else MissionState.EXECUTING
        )
        self._set_state(MissionState.PLANNING)
        self._mission_start_time = time.time()
        self._last_costmap_replan_time = time.time()
        self._direct_goal_fallback_status = None
        self._using_external_strategy_path = False
        self._active_external_strategy_path = []

        path = None
        if not self._planner_svc.is_ready:
            if self._allow_direct_goal_fallback:
                path = self._direct_goal_path(reason="planner backend not ready")
            else:
                self._failure_reason = "planner backend not ready"
                self._set_state(MissionState.FAILED)
                return
        else:
            try:
                path, _ = self._planner_svc.plan(
                    self._robot_pos,
                    _goal,
                    safe_goal_tolerance=self._safe_goal_tolerance,
                )
                self._publish_plan_report()
            except Exception as exc:
                self._publish_plan_report()
                if self._should_use_direct_goal_fallback(exc):
                    path = self._direct_goal_path(reason=str(exc))
                elif self._should_defer_empty_path_failure(exc):
                    self._defer_empty_path_failure(str(exc), planned_state)
                    return
                else:
                    logger.exception("Planning failed: %s", exc)
                    self._failure_reason = str(exc)
                    self._set_state(MissionState.FAILED)
                    return

        try:
            path = self._validate_planned_path(path)
            path = self._anchor_path_start_to_robot(path)
        except RuntimeError as exc:
            if self._should_defer_empty_path_failure(exc):
                self._defer_empty_path_failure(str(exc), planned_state)
                return
            logger.error("Planning failed: %s", exc)
            self._failure_reason = str(exc)
            self._tracker.clear()
            self._publish_motion_stop()
            self._set_state(MissionState.FAILED)
            return

        self._failure_reason = ""
        self._clear_deferred_empty_path()
        self._active_path_terminal_goal = np.asarray(path[-1][:3], dtype=float).copy()
        self._tracker.reset(path, self._robot_pos, self._robot_yaw)
        # Advance past any waypoints the robot is already at (e.g. the start)
        self._tracker.update(self._robot_pos, self._robot_yaw)
        self.global_path.publish(path)
        self._set_state(planned_state)
        self._publish_waypoint()

    def _publish_plan_report(self) -> None:
        report = self._planner_last_plan_report(self._planner_svc)
        if not report:
            return
        selected = report.get("selected_planner")
        rejected = report.get("rejected_plans") or []
        planner_name = getattr(self._planner_svc, "_planner_name", selected)
        if selected != planner_name or rejected:
            payload = {
                "event": "global_plan_selection",
                "selected_planner": selected,
                "fallback_reason": report.get("fallback_reason", ""),
                "rejected_plans": rejected,
                "policy": report.get("policy", ""),
                "ts": time.time(),
            }
            self.adapter_status.publish(payload)

    def _should_use_direct_goal_fallback(self, exc: Exception) -> bool:
        if not self._allow_direct_goal_fallback:
            return False
        if not self._planner_svc.has_map:
            return True
        if not self._direct_goal_fallback_on_planner_failure:
            return False
        text = str(exc).lower()
        return "empty path" in text or "no path" in text

    def _should_defer_empty_path_failure(self, exc: Exception) -> bool:
        if not self._defer_empty_path_planning_failure:
            return False
        text = str(exc).lower()
        return (
            "empty path" in text
            or "no path" in text
            or "no reachable free cell" in text
        )

    def _clear_deferred_empty_path(self) -> None:
        self._deferred_empty_path_first_ts = 0.0
        self._deferred_empty_path_attempts = 0

    def _defer_empty_path_failure(
        self,
        reason: str,
        planned_state: str,
    ) -> None:
        now = time.time()
        if self._deferred_empty_path_first_ts <= 0.0:
            self._deferred_empty_path_first_ts = now
            self._deferred_empty_path_attempts = 0
        self._deferred_empty_path_attempts += 1
        waited_s = now - self._deferred_empty_path_first_ts
        if waited_s > self._empty_path_retry_timeout_s:
            logger.error("Planning failed after deferred retries: %s", reason)
            self._failure_reason = reason
            self._tracker.clear()
            self._publish_motion_stop()
            self._clear_deferred_empty_path()
            self._set_state(MissionState.FAILED)
            return

        self._failure_reason = ""
        self._tracker.clear()
        self._publish_motion_stop()
        self.adapter_status.publish({
            "event": "planning_deferred_empty_path",
            "reason": reason,
            "planned_state": planned_state,
            "attempt": self._deferred_empty_path_attempts,
            "waited_s": round(waited_s, 3),
            "retry_interval_s": self._empty_path_retry_interval_s,
            "retry_timeout_s": self._empty_path_retry_timeout_s,
            "goal": self._point_summary(self._goal),
            "ts": now,
        })
        self._set_state(MissionState.PLANNING)

    def _direct_goal_path(self, reason: str = "") -> list[np.ndarray]:
        _goal = self._get_goal()
        if _goal is None:
            raise RuntimeError("direct_goal_fallback called with no goal set")
        logger.warning(
            "NavigationModule: using direct-goal fallback waypoint (reason=%s)",
            reason or "planner unavailable",
        )
        self._direct_goal_fallback_status = {
            "used": True,
            "reason": reason or "planner unavailable",
            "goal": [float(_goal[0]), float(_goal[1]), float(_goal[2])],
            "ts": time.time(),
        }
        self.adapter_status.publish({
            "event": "direct_goal_fallback",
            **self._direct_goal_fallback_status,
        })
        return [_goal.copy()]

    def _should_continue_after_partial_path(self) -> bool:
        status = self._partial_path_terminal_status()
        if not status.get("partial"):
            return False
        if self._accept_partial_goal_progress:
            return False
        return True

    def _partial_path_terminal_status(self) -> dict[str, Any]:
        if self._using_external_strategy_path:
            return {"partial": False}
        if self._goal is None or self._active_path_terminal_goal is None:
            return {"partial": False}
        try:
            goal = np.asarray(self._goal[:2], dtype=float)
            terminal = np.asarray(self._active_path_terminal_goal[:2], dtype=float)
            robot = np.asarray(self._robot_pos[:2], dtype=float)
        except (TypeError, ValueError):
            return {"partial": False}
        if not (
            np.all(np.isfinite(goal))
            and np.all(np.isfinite(terminal))
            and np.all(np.isfinite(robot))
        ):
            return {"partial": False}
        final_threshold = float(getattr(self._tracker, "_final_threshold", 0.0))
        threshold = max(final_threshold, 0.05)
        terminal_gap = float(np.linalg.norm(terminal - goal))
        robot_gap = float(np.linalg.norm(robot - goal))
        if terminal_gap <= threshold or robot_gap <= threshold:
            return {
                "partial": False,
                "terminal_gap_m": round(terminal_gap, 3),
                "robot_gap_m": round(robot_gap, 3),
            }
        report = self._planner_last_plan_report(self._planner_svc)
        primary_replan = report.get("primary_replan")
        partial = bool(primary_replan) or report.get("reached_goal") is False
        reason = ""
        if bool(primary_replan):
            reason = "primary_replan"
        elif report.get("reached_goal") is False:
            reason = "planner_partial_path"
        return {
            "partial": partial,
            "terminal_gap_m": round(terminal_gap, 3),
            "robot_gap_m": round(robot_gap, 3),
            "selected_planner": report.get("selected_planner"),
            "reason": reason,
        }

    def _record_partial_goal_progress_complete(self) -> None:
        if self._goal is None or self._active_path_terminal_goal is None:
            return
        self._partial_progress_completed_goal = np.asarray(
            self._goal[:3], dtype=float
        ).copy()
        self._partial_progress_completed_terminal = np.asarray(
            self._active_path_terminal_goal[:3], dtype=float
        ).copy()
        self._partial_progress_completed_ts = time.time()

    def _clear_partial_goal_progress(self) -> None:
        self._partial_progress_completed_goal = None
        self._partial_progress_completed_terminal = None
        self._partial_progress_completed_ts = 0.0

    def _validate_planned_path(self, path: Any) -> list[np.ndarray]:
        try:
            points = list(path) if path is not None else []
        except TypeError as exc:
            raise RuntimeError("planner returned a non-iterable path") from exc
        if not points:
            raise RuntimeError("planner returned empty path")

        validated: list[np.ndarray] = []
        for point in points:
            try:
                arr = np.asarray(point, dtype=float).reshape(-1)
            except (TypeError, ValueError) as exc:
                raise RuntimeError("planner returned an invalid path point") from exc
            if arr.size < 2:
                raise RuntimeError("planner returned a path point with fewer than 2 coordinates")
            if not np.all(np.isfinite(arr)):
                raise RuntimeError("planner returned a non-finite path point")
            if arr.size < 3:
                arr = np.pad(arr[:2], (0, 1), constant_values=0.0)
            else:
                arr = arr[:3].copy()
            validated.append(arr)
        return validated

    def _anchor_path_start_to_robot(self, path: list[np.ndarray]) -> list[np.ndarray]:
        if not path:
            return path
        robot = np.asarray(self._robot_pos[:3], dtype=float)
        first = np.asarray(path[0][:3], dtype=float)
        if not (np.all(np.isfinite(robot)) and np.all(np.isfinite(first))):
            return path
        if float(np.linalg.norm(first[:2] - robot[:2])) > 0.5:
            return path
        anchored = [point.copy() for point in path]
        original_z = np.array([point[2] for point in anchored], dtype=float)
        planar_zero_height = bool(
            original_z.size > 0
            and np.all(np.isfinite(original_z))
            and float(np.ptp(original_z)) <= 1e-6
            and abs(float(original_z[0])) <= 1e-6
        )
        if planar_zero_height:
            for point in anchored:
                point[2] = robot[2]
        anchored[0] = robot.copy()
        return anchored

    def _advance_patrol(self) -> bool:
        """Advance to next patrol goal. Returns True if more goals remain."""
        self._patrol_index += 1
        if self._patrol_index >= len(self._patrol_goals):
            if self._patrol_loop:
                self._patrol_index = 0
            else:
                return False
        self._goal = self._patrol_goals[self._patrol_index]
        self._set_replan_count(0)
        logger.info(
            "Patrol advancing to goal %d/%d",
            self._patrol_index + 1, len(self._patrol_goals),
        )
        self._plan()
        return True

    def _publish_waypoint(self) -> None:
        wp = self._tracker.current_waypoint
        if wp is None:
            return
        pose = PoseStamped(
            pose=Pose(
                position=Vector3(
                    float(wp[0]), float(wp[1]),
                    float(wp[2]) if len(wp) > 2 else 0.0,
                ),
                orientation=Quaternion(0, 0, 0, 1),
            ),
            frame_id=self._planning_frame_id,
            ts=time.time(),
        )
        self.waypoint.publish(pose)
        # ROS2 waypoint publishing moved to ROS2WaypointBridgeModule
        # (nav.ros2_waypoint_bridge_module).  Subscribe to self.waypoint.

    # ── Skills (auto-discovered by MCPServerModule) ───────────────────────

    @skill
    def navigate_to(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        z: float | None = None,
    ) -> str:
        """Navigate to map coordinates.

        Args:
            x: X coordinate in meters (map frame)
            y: Y coordinate in meters (map frame)
            z: Z coordinate in meters (map frame). Defaults to current odometry z.
            yaw: Heading in radians (default 0)
        """
        try:
            goal_x = float(x)
            goal_y = float(y)
            goal_yaw = float(yaw)
            goal_z = self._resolve_goal_z(z)
        except (TypeError, ValueError) as exc:
            return json.dumps({
                "status": "rejected",
                "reason": "invalid_coordinates",
                "error": str(exc),
                "frame_id": self._planning_frame_id,
            })
        if not all(math.isfinite(v) for v in (goal_x, goal_y, goal_yaw, goal_z)):
            return json.dumps({
                "status": "rejected",
                "reason": "invalid_coordinates",
                "error": "x, y, z, and yaw must be finite",
                "frame_id": self._planning_frame_id,
            })
        q_w = math.cos(goal_yaw / 2)
        q_z = math.sin(goal_yaw / 2)
        self._on_goal(PoseStamped(
            pose=Pose(
                position=Vector3(goal_x, goal_y, goal_z),
                orientation=Quaternion(0.0, 0.0, q_z, q_w),
            ),
            frame_id=self._planning_frame_id, ts=0.0,
        ))
        return json.dumps({
            "status": "navigating",
            "goal": [goal_x, goal_y, goal_z],
            "yaw": goal_yaw,
            "frame_id": self._planning_frame_id,
        })

    def _resolve_goal_z(self, z: float | None) -> float:
        if z is not None:
            try:
                z_value = float(z)
                if math.isfinite(z_value):
                    return z_value
            except (TypeError, ValueError):
                pass
            raise ValueError("z must be finite")
        try:
            current_z = float(self._robot_pos[2])
            if math.isfinite(current_z):
                return current_z
        except (AttributeError, IndexError, TypeError, ValueError):
            pass
        return 0.0

    @skill
    def stop_navigation(self) -> str:
        """Immediately stop all robot motion and cancel current mission."""
        self._on_stop(2)
        return json.dumps({"status": "stopped"})

    @skill
    def cancel_mission(self, reason: str = "user_cancel") -> str:
        """Cancel current navigation mission gracefully."""
        self._on_cancel(reason)
        return json.dumps({"status": "cancelled", "reason": reason})

    @skill
    def get_navigation_status(self) -> str:
        """Get current navigation state, position, and mission progress."""
        pos = {"x": round(float(self._robot_pos[0]), 3),
               "y": round(float(self._robot_pos[1]), 3),
               "yaw": round(float(self._robot_yaw), 3)}
        _status_goal = self._get_goal()
        goal = ({"x": round(float(_status_goal[0]), 3),
                 "y": round(float(_status_goal[1]), 3)}
                if _status_goal is not None else None)
        return json.dumps({
            "state": self._get_state(),
            "frame_id": self._planning_frame_id,
            "planning_frame_id": self._planning_frame_id,
            "odom_frame_id": self._odom_frame_id,
            "costmap_frame_id": self._costmap_frame_id,
            "position": pos,
            "goal": goal,
            "path_length": self._tracker.path_length,
            "waypoint_index": self._tracker.wp_index,
            "replan_count": self._get_replan_count(),
            "failure_reason": self._get_failure_reason(),
            "plan_safety_policy": getattr(
                self._planner_svc,
                "_plan_safety_policy",
                "observe",
            ),
            "replan_on_costmap_update": self._replan_on_costmap_update,
            "last_plan_report": self._current_plan_report(),
            "map_artifact_gate": getattr(self._planner_svc, "map_artifact_gate", {}),
            "direct_goal_fallback": self._direct_goal_fallback_status,
        })

    @skill
    def start_patrol(self, waypoints_json: str) -> str:
        """Start patrol mode — navigate a list of waypoints sequentially.

        Args:
            waypoints_json: JSON array of [x, y] pairs,
                            e.g. "[[1.0, 2.0], [3.0, 4.0]]"
        """
        try:
            goals = json.loads(waypoints_json)
            self._on_patrol_goals(goals)
            return json.dumps({"status": "patrolling", "goals": len(goals)})
        except Exception as e:
            return json.dumps({"error": str(e)})

    # ── Lifecycle ──────────────────────────────────────────────────────────

    def stop(self) -> None:
        self._request_recovery_stop()
        self._preview_executor.shutdown(wait=False, cancel_futures=True)
        super().stop()

    def planner_backend_status(self) -> dict[str, Any]:
        backend_status = getattr(self._planner_svc, "backend_status", None)
        if callable(backend_status):
            return backend_status()
        report = self._planner_last_plan_report(self._planner_svc)
        selected = str(
            report.get("selected_planner")
            or getattr(self._planner_svc, "_planner_name", "unknown")
        )
        configured = str(getattr(self._planner_svc, "_planner_name", selected))
        fallback_reason = str(report.get("fallback_reason") or "")
        return {
            "configured_backend": configured,
            "backend": selected,
            "fallback_backend": getattr(
                self._planner_svc,
                "_fallback_planner_name",
                "",
            ),
            "degraded": bool(fallback_reason) or selected != configured,
            "degraded_reason": fallback_reason,
        }

    def reload_planner_tomogram(self, tomogram: str) -> dict[str, Any]:
        reload_tomogram = getattr(self._planner_svc, "reload_tomogram", None)
        if not callable(reload_tomogram):
            return {
                "ok": False,
                "reason": "planner_reload_unsupported",
            }
        return reload_tomogram(tomogram)

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["planner_backend"] = self.planner_backend_status()
        info["navigation"] = {
            "planner": getattr(self._planner_svc, "_planner_name", None),
            "plan_safety_policy": getattr(
                self._planner_svc,
                "_plan_safety_policy",
                "observe",
            ),
            "last_plan_report": self._current_plan_report(),
            "map_artifact_gate": getattr(self._planner_svc, "map_artifact_gate", {}),
            "state": self._get_state(),
            "wp_index": self._tracker.wp_index,
            "wp_total": self._tracker.path_length,
            "replan_count": self._get_replan_count(),
            "failure_reason": self._get_failure_reason(),
            "replan_on_costmap_update": self._replan_on_costmap_update,
            "direct_goal_fallback": self._direct_goal_fallback_status,
            "patrol_index": self._patrol_index if self._patrol_goals else -1,
            "patrol_total": len(self._patrol_goals),
        }
        return info
