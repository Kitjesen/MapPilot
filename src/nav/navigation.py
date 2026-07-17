"""Navigation mission runtime: goal lifecycle, planning handoff, waypoint tracking.

NAV COMPUTE CONTRACT (docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md):
  This module owns the mission lifecycle around L5 global planning
  (saved map artifact -> global_path + waypoint) and the L2 safety-gating use
  of `costmap`.

  `costmap` (= TraversabilityCostModule.fused_cost) is consumed ONLY for safety
  gating, never to produce trajectories:
    1. Map-backed planner service overlays live risk onto OctoPlanner3D/A*
       safe-goal and path-safety checks.
    2. Throttled replan trigger. OctoPlanner3D consumes saved-map artifacts;
       A* dev/sim defaults can replan from a live grid.
  Local trajectory generation is NOT done here - that is LocalPlanner (L2).

Boundary:
  - GoalService/PatrolService parse external commands and publish typed inputs.
  - Navigation owns mission state, current goal/route, replans, recovery, and
    waypoint dispatch.
  - PlannerService owns path calculation and planner backend diagnostics.
  - LocalPlanner/PathFollower own local obstacle avoidance and velocity output.

Internal services (not Modules - no ports, just algorithm logic):
  - Planner service       (nav/services/plan/factory.py) - plan() + downsample
  - WaypointTracker       (nav/mission/tracking/waypoint_tracker.py) - arrival + stuck detection

Ports:
  In:  goal_pose, odometry, costmap, stop_signal, patrol_goals, cancel,
       speed_scale, localization_state, map_odom_tf, map_frame_jump_event
  Out: waypoint, global_path, planner_status, mission_status, adapter_status

Usage::

    bp.add(Navigation, planner="octoplanner3d", map_path="building.pcd")
"""

from __future__ import annotations

import logging
import math
import os
import threading
from typing import Any

from nav.model.frame_contract import FrameContract
from nav.model.state import (
    MissionEvent,
    MissionMode,
    MissionState,
    MissionStateName,
    MissionTransitionRejected,
)
from nav.model.status import (
    MissionStatus,
    build_navigation_health,
)
from nav.runtime.control import NavigationControlMixin
from nav.runtime.execution import NavigationExecutionMixin
from nav.runtime.fsm import NavigationFsmMixin
from nav.runtime.planning import NavigationPlanningMixin
from nav.runtime.recovery import NavigationRecoveryMixin
from nav.services.frame_transforms import transform_xyz_yaw_with_map_odom
from nav.services.plan.contracts import PlannerService
from nav.services.plan.factory import create_planner_service
from nav.services.plan.preview import (
    PlanPreviewService,
    planner_backend_status,
)
from nav.tracking.waypoint_tracker import WaypointTracker
from runtime.module import Module
from runtime.msgs.geometry import PoseStamped, Twist
from runtime.msgs.nav import Odometry, Path
from runtime.msgs.numpy_compat import np
from runtime.registry import register
from runtime.runtime_interface import (
    map_frame_id,
    normalize_frame_id,
)
from runtime.stream import In, Out

logger = logging.getLogger(__name__)


PLANNING_FRAME_ID = map_frame_id()


@register("navigation", "default", description="Unified navigation module")
class Navigation(
    Module,
    NavigationFsmMixin,
    NavigationControlMixin,
    NavigationExecutionMixin,
    NavigationPlanningMixin,
    NavigationRecoveryMixin,
    layer=5,
):
    """Unified navigation: plan -> track -> FSM in one Module.

    Architecture:
      Navigation owns the mission FSM and patrol logic.
      The selected planner service owns planning algorithm + downsample.
      WaypointTracker owns arrival detection + stuck detection.
    """

    runtime_id = "nav.mission"

    # -- Inputs --
    goal_pose: In[PoseStamped]
    odometry: In[Odometry]
    costmap: In[dict]
    stop_signal: In[int]
    patrol_goals: In[list]
    cancel: In[str]
    teleop_active: In[bool]
    speed_scale: In[float]
    localization_state: In[dict]
    traversability: In[dict]  # W2-8: terrain class from Terrain
    map_odom_tf: In[dict]
    # P4: TF jump events from the selected SLAM implementation. PGO loop closures and BBS3D
    # relocalisations can move map->odom by metres in a single tick. Cached
    # global path + waypoint then point at the wrong place. We force a replan
    # on the next planning tick to catch up.
    map_frame_jump_event: In[dict]

    # -- Outputs --
    waypoint: Out[PoseStamped]
    global_path: Out[Path]
    clear_path: Out[bool]
    planner_status: Out[MissionStateName]
    mission_status: Out[MissionStatus]
    adapter_status: Out[dict]
    map_health_event: Out[dict]
    recovery_cmd_vel: Out[Twist]  # direct cmd_vel for backup/rotate recovery

    def __init__(
        self,
        planner: str = "octoplanner3d",
        map_path: str = "",
        obstacle_thr: float = 49.9,
        waypoint_threshold: float = 1.5,
        stuck_timeout: float = 10.0,
        stuck_dist_thre: float = 0.15,
        max_replan_count: int = 3,
        downsample_dist: float = 2.0,
        final_waypoint_threshold: float | None = None,
        complete_path_on_goal_proximity: bool = False,
        goal_proximity_completion_threshold: float | None = None,
        waypoint_z_threshold: float | None = 0.25,
        allow_direct_goal_fallback: bool = False,
        direct_goal_fallback_on_planner_failure: bool = False,
        external_strategy_path_control: bool = False,
        external_strategy_start_tolerance_m: float = 1.5,
        goal_update_epsilon: float = 0.25,
        safe_goal_tolerance: float = 4.0,
        plan_safety_policy: str = "observe",
        fallback_planner_name: str = "",
        octoplanner3d_robot_radius: float | None = None,
        octoplanner3d_max_iterations: int | None = None,
        octoplanner3d_snap_search_radius_cells: int | None = None,
        octoplanner3d_require_ground_support: bool | None = None,
        octoplanner3d_strict_direct_ground_support: bool | None = None,
        octoplanner3d_ground_support_xy_radius_cells: int | None = None,
        octoplanner3d_ground_support_depth_cells: int | None = None,
        octoplanner3d_enable_preblocked_costmap: bool | None = None,
        octoplanner3d_preblocked_costmap_radius_cells: int | None = None,
        octoplanner3d_preblocked_costmap_weight: float | None = None,
        octoplanner3d_lowest_traversable_only: bool | None = None,
        octoplanner3d_floor_change_penalty: float | None = None,
        octoplanner3d_max_step_height: float | None = None,
        octoplanner3d_max_slope: float | None = None,
        octoplanner3d_same_floor_preference: bool | None = None,
        octoplanner3d_same_floor_z_tolerance: float | None = None,
        octoplanner3d_max_same_floor_z_excursion: float | None = None,
        octoplanner3d_obstacle_clearance_radius_cells: int | None = None,
        octoplanner3d_obstacle_clearance_weight: float | None = None,
        accept_partial_goal_progress: bool = False,
        partial_goal_repeat_ignore_window_s: float = 5.0,
        defer_empty_path_planning_failure: bool = False,
        empty_path_retry_interval_s: float = 3.0,
        empty_path_retry_timeout_s: float = 30.0,
        replan_on_costmap_update: bool | None = None,
        auto_resume_after_teleop: bool = False,
        allow_path_start_insert: bool = False,
        octoplanner3d_timeout_s: float | None = None,
        **kw,
    ):
        if "enable_ros2_bridge" in kw:
            raise TypeError(
                "Navigation no longer accepts enable_ros2_bridge; "
                "configure navigation IO adapters in the blueprint stack"
            )
        super().__init__(**kw)
        self._allow_direct_goal_fallback = allow_direct_goal_fallback
        self._direct_goal_fallback_on_planner_failure = direct_goal_fallback_on_planner_failure
        self._external_strategy_path_control = external_strategy_path_control
        self._external_strategy_start_tolerance_m = float(external_strategy_start_tolerance_m)
        self._using_external_strategy_path = False
        self._goal_update_epsilon = goal_update_epsilon
        self._safe_goal_tolerance = safe_goal_tolerance
        self._complete_path_on_goal_proximity = bool(complete_path_on_goal_proximity)
        self._goal_proximity_completion_threshold = self._normalise_goal_proximity_completion_threshold(
            goal_proximity_completion_threshold,
            final_waypoint_threshold=final_waypoint_threshold,
            waypoint_threshold=waypoint_threshold,
        )
        self._accept_partial_goal_progress = accept_partial_goal_progress
        self._partial_goal_repeat_ignore_window_s = max(
            0.0,
            float(partial_goal_repeat_ignore_window_s),
        )
        self._defer_empty_path_planning_failure = bool(defer_empty_path_planning_failure)
        self._empty_path_retry_interval_s = max(
            0.1,
            float(empty_path_retry_interval_s),
        )
        self._empty_path_retry_timeout_s = max(
            self._empty_path_retry_interval_s,
            float(empty_path_retry_timeout_s),
        )
        if replan_on_costmap_update is None:
            replan_on_costmap_update = str(planner).strip().lower() != "pct"
        self._replan_on_costmap_update = bool(replan_on_costmap_update)
        self._direct_goal_fallback_status: dict[str, Any] | None = None
        self._allow_path_start_insert = bool(allow_path_start_insert)
        self._path_start_anchor_status: dict[str, Any] = {}
        self._deferred_empty_path_first_ts: float = 0.0
        self._deferred_empty_path_attempts: int = 0
        planning_frame_id = str(kw.get("planning_frame_id", PLANNING_FRAME_ID))
        expected_saved_map_frame_id = str(kw.get("expected_saved_map_frame_id", planning_frame_id) or planning_frame_id)
        map_artifact_gate_required = kw.get("map_artifact_gate_required")
        octoplanner3d_constraints = {
            "robot_radius": octoplanner3d_robot_radius,
            "max_iterations": octoplanner3d_max_iterations,
            "snap_search_radius_cells": octoplanner3d_snap_search_radius_cells,
            "require_ground_support": octoplanner3d_require_ground_support,
            "strict_direct_ground_support": octoplanner3d_strict_direct_ground_support,
            "ground_support_xy_radius_cells": octoplanner3d_ground_support_xy_radius_cells,
            "ground_support_depth_cells": octoplanner3d_ground_support_depth_cells,
            "enable_preblocked_costmap": octoplanner3d_enable_preblocked_costmap,
            "preblocked_costmap_radius_cells": (octoplanner3d_preblocked_costmap_radius_cells),
            "preblocked_costmap_weight": octoplanner3d_preblocked_costmap_weight,
            "lowest_traversable_only": octoplanner3d_lowest_traversable_only,
            "floor_change_penalty": octoplanner3d_floor_change_penalty,
            "max_step_height": octoplanner3d_max_step_height,
            "max_slope": octoplanner3d_max_slope,
            "same_floor_preference": octoplanner3d_same_floor_preference,
            "same_floor_z_tolerance": octoplanner3d_same_floor_z_tolerance,
            "max_same_floor_z_excursion": octoplanner3d_max_same_floor_z_excursion,
            "obstacle_clearance_radius_cells": (octoplanner3d_obstacle_clearance_radius_cells),
            "obstacle_clearance_weight": octoplanner3d_obstacle_clearance_weight,
        }

        self._planner_svc: PlannerService = create_planner_service(
            planner_name=planner,
            map_path=map_path,
            obstacle_thr=obstacle_thr,
            downsample_dist=downsample_dist,
            plan_safety_policy=plan_safety_policy,
            fallback_planner_name=fallback_planner_name,
            expected_saved_map_frame_id=expected_saved_map_frame_id,
            map_artifact_gate_required=map_artifact_gate_required,
            octoplanner3d_constraints=octoplanner3d_constraints,
            octoplanner3d_timeout_s=octoplanner3d_timeout_s,
        )
        self._tracker = WaypointTracker(
            threshold=waypoint_threshold,
            final_threshold=final_waypoint_threshold,
            z_threshold=waypoint_z_threshold,
            stuck_timeout=stuck_timeout,
            stuck_dist=stuck_dist_thre,
        )

        # Mission FSM
        self._nav_lock = threading.Lock()
        self._state: MissionState = MissionState.IDLE
        self._mission_mode: MissionMode = MissionMode.NONE
        self._previous_state: MissionState | None = None
        self._phase_reason = "initialized"
        self._last_rejected_transition: MissionTransitionRejected | None = None
        self._robot_pos = [0.0, 0.0, 0.0]
        self._robot_yaw = 0.0
        self._planning_frame_id = planning_frame_id
        self._frame_contract = FrameContract(
            planning_frame_id=self._planning_frame_id,
            publish_adapter_status=self.adapter_status.publish,
        )
        self._odom_frame_id = "unknown"
        self._costmap_frame_id = "unknown"
        self._map_odom_tf: dict[str, Any] | None = None
        self._goal_frame_id: str | None = None
        self._goal: np.ndarray | None = None
        self._active_path_terminal_goal: np.ndarray | None = None
        self._last_global_plan: dict[str, Any] | None = None
        self._last_map_health_plan_signature: tuple[Any, ...] | None = None
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
        self._pre_pause_state: MissionState | None = None
        self._localization_recovery_motion_hold: bool = False
        self._planning_timeout = kw.get("planning_timeout", 30.0)
        self._speed_scale: float = 1.0  # degeneracy-based speed multiplier
        self._speed_policy_reason: str = "normal"
        self._plan_preview = PlanPreviewService(
            timeout_s=float(
                kw.get(
                    "preview_timeout",
                    os.environ.get("LINGTU_PLAN_PREVIEW_TIMEOUT", 3.0),
                )
            )
        )

        # Teleop pause/resume
        self._auto_resume_after_teleop = bool(auto_resume_after_teleop)
        self._paused_for_teleop: bool = False
        self._pre_teleop_goal: np.ndarray | None = None
        self._pre_teleop_state: MissionState | None = None
        self._recovery_lock = threading.Lock()
        self._recovery_stop_event = threading.Event()
        self._recovery_thread: threading.Thread | None = None

        # Cooldown for costmap-triggered replanning (3s minimum between replans)
        self._last_costmap_replan_time = 0.0

        # Patrol FSM state
        self._patrol_goals: list[np.ndarray] = []
        self._patrol_index = 0
        self._patrol_loop = False

        # W2-8: latest terrain class from Terrain - defaults to "unknown"
        # which triggers the conservative backup+rotate recovery strategy.
        self._latest_traversability_class: str = "unknown"

    def setup(self) -> None:
        self._planner_svc.setup()

        self.goal_pose.subscribe(self._on_goal)
        self.odometry.subscribe(self._on_odom)
        self.costmap.subscribe(self._on_costmap)
        self.stop_signal.subscribe(self._on_stop)
        self.patrol_goals.subscribe(self._on_patrol_goals)
        self.cancel.subscribe(self._on_cancel)
        self.teleop_active.subscribe(self._on_teleop_active)
        self.speed_scale.subscribe(self._on_speed_scale)
        self.localization_state.subscribe(self._on_localization_state)
        self.traversability.subscribe(self._on_traversability)
        self.map_odom_tf.subscribe(self._on_map_odom_tf)
        self.map_frame_jump_event.subscribe(self._on_map_frame_jump)

        self._set_state(MissionState.IDLE, reason="module setup complete")

    # External endpoint spinning is owned by the selected compat adapter.

    # Mission FSM

    # Input handlers

    # Recovery motion

    # Planning

    # Endpoint waypoint publishing moved to the optional endpoint bridge.

    # Localization TF and pause/resume

    def _on_map_odom_tf(self, msg: dict) -> None:
        """Receive and cache the map->odom TF transform."""
        self._map_odom_tf = None
        if isinstance(msg, dict) and msg.get("valid") is not False:
            self._map_odom_tf = dict(msg)
        self._frame_contract.set_map_odom_tf(self._map_odom_tf)

    def _odom_pose_in_planning_frame(
        self,
        odom: Odometry,
        raw_yaw: float | None,
    ) -> tuple[list[float], float | None]:
        """Convert odometry pose to planning frame using map->odom TF."""
        source_frame = normalize_frame_id(getattr(odom, "frame_id", None))
        pos, yaw, _out_frame, _transformed, _reason = transform_xyz_yaw_with_map_odom(
            [odom.pose.position.x, odom.pose.position.y, odom.pose.position.z],
            raw_yaw,
            source_frame=source_frame,
            target_frame=self._planning_frame_id,
            map_odom_tf=self._map_odom_tf,
        )
        return pos, yaw

    def _accelerate_stuck_check_on_reverse_motion(self, odom: Odometry) -> None:
        """Accelerate stuck detection when the robot moves away from the waypoint.

        Checks body-frame velocity against the direction to the current
        waypoint.  If the robot is moving *away* from the waypoint
        (reverse / backing up), reduce the effective stuck timeout so we
        trigger recovery faster.
        """
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
            self.adapter_status.publish(
                {
                    "event": "reverse_motion_stuck_accelerated",
                    "away_speed_mps": round(away_speed, 3),
                    "waypoint_index": self._tracker.wp_index,
                }
            )

    def _pause_for_localization(self, *, reason: str = "localization lost") -> None:
        """Pause the mission when localization is LOST."""
        if not self._paused_for_localization:
            self._pre_pause_state = self._get_state()
        self._paused_for_localization = True
        self._tracker.pause()
        self._failure_reason = reason
        if self._get_state() != MissionState.IDLE:
            self._apply_event(MissionEvent.PAUSE, reason=reason)
        logger.warning("Navigation PAUSED: %s", reason)

    def _clear_localization_pause_for_explicit_action(
        self,
        *,
        reason: str,
        clear_goal: bool = False,
    ) -> None:
        """Clear a localization pause when the user issues an explicit action."""
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

    def _localization_status_requires_motion_hold(self, state: dict) -> bool:
        """Check whether the localization state requires the robot to hold motion."""
        return bool(state.get("motion_hold_required", False))

    def _on_speed_scale(self, value: float) -> None:
        """Receive speed multiplier from LocalizationMonitorModule."""
        self._speed_scale = max(0.0, min(1.0, float(value)))

    def _on_localization_state(self, state: dict) -> None:
        """Receive localization health state from LocalizationMonitorModule.

        This replaces the old _on_localization_status handler from the deleted
        NavigationLocalizationMixin.  Speed scaling and degeneracy detection
        are now computed by LocalizationMonitorModule and arrive pre-processed
        via the speed_scale port / state dict.
        """
        prev = self._loc_state
        self._loc_state = str(state.get("state", "UNINIT"))
        self._loc_confidence = float(state.get("confidence", 0.0))
        self._degen_level = str(state.get("degeneracy", "NONE"))
        motion_hold_required = bool(state.get("motion_hold_required", False))

        # Update speed scale from the monitor (authoritative)
        scale = state.get("speed_scale")
        if scale is not None:
            self._speed_scale = float(scale)
        reason = state.get("speed_policy_reason")
        if reason:
            self._speed_policy_reason = str(reason)

        if motion_hold_required:
            self._localization_recovery_motion_hold = True
            if self._is_path_execution_state():
                self._pause_for_localization(reason="localization recovery requires explicit new goal")
            elif self._paused_for_localization:
                self._failure_reason = "localization recovered; explicit goal required"

        # Pause mission when localization transitions to LOST
        if self._loc_state == "LOST" and prev != "LOST":
            if self._is_path_execution_state():
                self._pause_for_localization(reason="localization lost")

        elif self._loc_state == "TRACKING" and self._paused_for_localization:
            if self._localization_recovery_motion_hold:
                self._failure_reason = "localization recovered; explicit goal required"
                if self._get_state() != MissionState.IDLE:
                    self._apply_event(MissionEvent.STOP, reason=self._failure_reason)
            else:
                self._paused_for_localization = False
                if self._pre_pause_state and self._goal is not None:
                    self._apply_event(
                        MissionEvent.RESUME,
                        reason="localization recovered",
                    )
                    self._pre_pause_state = None

    # Patrol (internal lightweight handler)

    def _on_patrol_goals(self, goals: list) -> None:
        """Parse incoming patrol goal lists into the internal patrol route."""
        if not goals:
            return
        parsed: list[np.ndarray] = []
        patrol_loop = False
        for item in goals:
            if isinstance(item, dict):
                frame_id = str(item.get("frame_id") or self._planning_frame_id)
                if frame_id != self._planning_frame_id:
                    continue
                raw = [item.get("x"), item.get("y"), item.get("z", 0.0)]
            elif isinstance(item, (list, tuple)) and len(item) >= 2:
                raw = [item[0], item[1], item[2] if len(item) > 2 else 0.0]
            else:
                continue
            try:
                g = np.array(raw, dtype=float)
            except (TypeError, ValueError):
                continue
            if g.shape == (3,) and np.all(np.isfinite(g)):
                parsed.append(g)
                if isinstance(item, dict) and item.get("loop"):
                    patrol_loop = True
        if parsed:
            self._patrol_goals = parsed
            self._patrol_loop = patrol_loop
            self._patrol_index = 0
            if self._external_strategy_path_control:
                self._start_external_strategy_path(parsed)
            else:
                self._start_goal(
                    self._patrol_goals[0],
                    frame_id=self._planning_frame_id,
                    event=MissionEvent.PATROL_ACCEPTED,
                    reason="patrol accepted",
                )

    def _advance_patrol(self) -> bool:
        """Advance to the next patrol waypoint; return False when done."""
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
            self._patrol_index + 1,
            len(self._patrol_goals),
        )
        self._plan()
        return True

    # Lifecycle

    def stop(self) -> None:
        self._request_recovery_stop()
        self._plan_preview.shutdown()
        super().stop()

    def planner_backend_status(self) -> dict[str, Any]:
        return planner_backend_status(self._planner_svc)

    def reload_planner_map(self, map_path: str = "") -> dict[str, Any]:
        reload_map = getattr(self._planner_svc, "reload_map", None)
        if callable(reload_map):
            return reload_map(map_path)
        return {
            "ok": False,
            "reason": "planner_reload_unsupported",
        }

    def health(self) -> dict[str, Any]:
        return build_navigation_health(self, super().port_summary())
