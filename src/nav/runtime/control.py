"""Mission control handlers for Navigation."""

from __future__ import annotations

import logging
import time

from nav.model.geometry import distance_xyz_or_xy, point_summary
from nav.model.state import MissionEvent, MissionState
from nav.services.frame_transforms import is_map_frame_jump_event
from runtime.msgs.geometry import PoseStamped, Twist
from runtime.msgs.numpy_compat import np

logger = logging.getLogger(__name__)


class NavigationControlMixin:
    def _start_goal(
        self,
        goal: np.ndarray,
        *,
        frame_id: str,
        event: MissionEvent,
        reason: str,
    ) -> None:
        self._clear_localization_pause_for_explicit_action(
            reason=reason,
            clear_goal=False,
        )
        self._clear_partial_goal_progress()
        self._clear_deferred_empty_path()
        self._using_external_strategy_path = False
        self._active_external_strategy_path = []
        self._goal = np.asarray(goal[:3], dtype=float).copy()
        self._goal_frame_id = frame_id
        self._set_replan_count(0)
        self._apply_event(event, reason=reason)
        self._plan()

    def _block_for_frame_mismatch(self, source: str, frame_id: str) -> None:
        expected_frame = self._frame_contract.expected_frame_label(source)
        self._failure_reason = f"unsupported {source} frame {frame_id!r}; expected {expected_frame!r}"
        self.adapter_status.publish(
            {
                "event": "navigation_blocked",
                "reason": "frame_mismatch",
                "source": source,
                "expected_frame": expected_frame,
                "received_frame": frame_id,
                "ts": time.time(),
            }
        )
        self._tracker.clear()
        self._active_path_terminal_goal = None
        self._active_external_strategy_path = []
        self._clear_partial_goal_progress()
        self._publish_motion_stop()
        self._apply_event(MissionEvent.FRAME_ERROR, reason=self._failure_reason)

    def _reject_goal_frame(self, frame_id: str, *, source: str) -> None:
        self._failure_reason = f"unsupported {source} frame {frame_id!r}; expected {self._planning_frame_id!r}"
        self.adapter_status.publish(
            {
                "event": "goal_rejected",
                "reason": "unsupported_frame",
                "source": source,
                "expected_frame": self._planning_frame_id,
                "received_frame": frame_id,
                "ts": time.time(),
            }
        )
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
            self._apply_event(MissionEvent.FRAME_ERROR, reason=self._failure_reason)
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
        self.adapter_status.publish(
            {
                "event": "goal_rejected",
                "reason": reason,
                "source": source,
                "details": details,
                "ts": time.time(),
            }
        )
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
            self._apply_event(MissionEvent.PLAN_FAILED, reason=self._failure_reason)
        else:
            self._set_state(self._get_state())

    def _on_goal(self, goal: PoseStamped) -> None:
        frame_id = self._frame_contract.goal_frame(goal, self._planning_frame_id)
        if frame_id != self._planning_frame_id:
            self._reject_goal_frame(frame_id, source="goal_pose")
            return
        try:
            new_goal = np.array(
                [
                    goal.pose.position.x,
                    goal.pose.position.y,
                    goal.pose.position.z,
                ],
                dtype=float,
            )
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
        self._start_goal(
            new_goal,
            frame_id=frame_id,
            event=MissionEvent.GOAL_ACCEPTED,
            reason="goal accepted",
        )

    def _should_ignore_goal_update(self, new_goal: np.ndarray) -> bool:
        if self._should_ignore_completed_partial_goal(new_goal):
            return True
        if self._goal is None:
            return False
        if not self._is_planning_or_path_state():
            return False
        dist = distance_xyz_or_xy(new_goal, self._goal)
        if dist > self._goal_update_epsilon:
            return False
        self.adapter_status.publish(
            {
                "event": "goal_update_ignored",
                "distance": dist,
                "goal": [float(new_goal[0]), float(new_goal[1]), float(new_goal[2])],
                "ts": time.time(),
            }
        )
        return True

    def _should_ignore_completed_partial_goal(self, new_goal: np.ndarray) -> bool:
        if not self._accept_partial_goal_progress:
            return False
        if self._partial_progress_completed_goal is None or self._partial_progress_completed_terminal is None:
            return False
        if self._partial_goal_repeat_ignore_window_s > 0.0:
            age_s = time.time() - self._partial_progress_completed_ts
            if age_s > self._partial_goal_repeat_ignore_window_s:
                self._clear_partial_goal_progress()
                return False
        try:
            goal_delta = distance_xyz_or_xy(
                new_goal,
                self._partial_progress_completed_goal,
            )
            robot_terminal_delta = distance_xyz_or_xy(
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
        self.adapter_status.publish(
            {
                "event": "partial_goal_repeat_ignored",
                "goal": point_summary(new_goal),
                "completed_goal": point_summary(self._partial_progress_completed_goal),
                "completed_terminal": point_summary(self._partial_progress_completed_terminal),
                "robot_terminal_delta_m": round(robot_terminal_delta, 3),
                "ts": time.time(),
            }
        )
        return True

    def _on_stop(self, level: int) -> None:
        if level >= 2:
            self._clear_teleop_resume_state()
            self._clear_active_mission(clear_goal=True)
            self._publish_motion_stop()
            self._apply_event(MissionEvent.STOP, reason="safety stop")
            self.adapter_status.publish(
                {
                    "event": "safety_stop",
                    "level": int(level),
                    "action": "mission_cleared",
                    "ts": time.time(),
                }
            )
        elif level == 1:
            self.adapter_status.publish(
                {
                    "event": "safety_soft_stop",
                    "level": int(level),
                    "action": "mission_held",
                    "ts": time.time(),
                }
            )
        elif level <= 0:
            self.adapter_status.publish(
                {
                    "event": "safety_clear",
                    "level": int(level),
                    "action": "mission_retained",
                    "ts": time.time(),
                }
            )

    def _publish_motion_stop(self) -> None:
        self._request_recovery_stop()
        self.clear_path.publish(True)
        self.recovery_cmd_vel.publish(Twist.zero())

    def _clear_teleop_resume_state(self) -> None:
        self._paused_for_teleop = False
        self._pre_teleop_goal = None
        self._pre_teleop_state = None

    def _clear_active_mission(self, *, clear_goal: bool) -> None:
        self._tracker.clear()
        self._patrol_goals.clear()
        self._patrol_index = 0
        self._patrol_loop = False
        if clear_goal:
            self._goal = None
            self._goal_frame_id = None
        self._active_path_terminal_goal = None
        self._using_external_strategy_path = False
        self._active_external_strategy_path = []
        self._clear_partial_goal_progress()
        self._clear_deferred_empty_path()

    def _on_teleop_active(self, active: bool) -> None:
        """Pause navigation when teleop engages, resume when released."""
        if active and not self._paused_for_teleop:
            # Save current mission state so we can resume
            if self._is_path_execution_state():
                self._pre_teleop_goal = self._goal.copy() if self._goal is not None else None
                self._pre_teleop_state = self._get_state()
                self._tracker.clear()
                self._publish_motion_stop()
                self._apply_event(MissionEvent.PAUSE, reason="teleop active")
                logger.info("Navigation: paused for teleop (saved goal)")
            self._paused_for_teleop = True

        elif not active and self._paused_for_teleop:
            self._paused_for_teleop = False
            if self._pre_teleop_goal is not None and self._get_state() == MissionState.PAUSED:
                if self._auto_resume_after_teleop:
                    logger.info("Navigation: teleop released, resuming navigation")
                    self._goal = self._pre_teleop_goal
                    self._pre_teleop_goal = None
                    self._pre_teleop_state = None
                    self._apply_event(MissionEvent.RESUME, reason="teleop released")
                    self._plan()
                else:
                    logger.info("Navigation: teleop released, explicit resume required")
                    self.adapter_status.publish(
                        {
                            "event": "teleop_release_resume_required",
                            "ts": time.time(),
                        }
                    )
                    self._pre_teleop_goal = None
                    self._pre_teleop_state = None
            else:
                logger.info("Navigation: teleop released, no mission to resume")
                self._pre_teleop_goal = None
                self._pre_teleop_state = None

    def _on_costmap(self, data: dict) -> None:
        self._costmap_frame_id = str(data.get("frame_id") or self._planning_frame_id)
        self._frame_contract.report_frame_mismatch("costmap", self._costmap_frame_id)
        if self._frame_contract.is_frame_mismatch(self._costmap_frame_id, source="costmap"):
            if self._frame_contract.has_motion_artifacts(
                self._get_state(),
                self._tracker.path_length,
                self._goal,
                self._patrol_goals,
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
        if (
            self._is_path_execution_state()
            and self._goal is not None
            and not self._using_external_strategy_path
            and self._replan_on_costmap_update
            and not self._is_recovery_active()
            and time.time() - self._last_costmap_replan_time > 3.0
        ):
            self._last_costmap_replan_time = time.time()
            self._plan()
        elif (
            self._get_state() == MissionState.PLANNING
            and self._deferred_empty_path_first_ts > 0.0
            and self._goal is not None
            and not self._using_external_strategy_path
            and self._replan_on_costmap_update
            and time.time() - self._last_costmap_replan_time > self._empty_path_retry_interval_s
        ):
            self._last_costmap_replan_time = time.time()
            self._plan()

    def _on_cancel(self, msg: str) -> None:
        had_localization_pause = (
            self._paused_for_localization
            or self._pre_pause_state is not None
            or self._localization_recovery_motion_hold
        )
        self._clear_active_mission(clear_goal=True)
        self._clear_teleop_resume_state()
        self._failure_reason = f"cancelled: {msg}" if msg else "cancelled"
        self._publish_motion_stop()
        if self._get_state() in (MissionState.IDLE, MissionState.CANCELLED) and not had_localization_pause:
            return
        self._clear_localization_pause_for_explicit_action(
            reason="cancel",
            clear_goal=True,
        )
        if self._get_state() in (MissionState.IDLE, MissionState.CANCELLED):
            logger.info("Mission cancelled while idle/paused: %s", msg)
            return
        cancel_reason = f"cancelled: {msg}" if msg else "cancelled"
        self._apply_event(MissionEvent.CANCEL, reason=cancel_reason)
        logger.info("Mission cancelled: %s", msg)

    def _on_map_frame_jump(self, event: dict) -> None:
        """SlamBridge detected a map->odom TF discontinuity (P4).

        The cached global path / waypoint were planned in the *old* map frame.
        Force an immediate replan so we don't drive the robot toward a
        coordinate that no longer matches reality. Costmap is in odom frame
        so it remains valid; ESDF / OccupancyGrid handle their own clear via
        their own subscription to this event.
        """
        if not is_map_frame_jump_event(event):
            return
        dt_m = event.get("dt_m", 0.0)
        dyaw = event.get("dyaw_deg", 0.0)
        # Only act if we have an active mission - idle robot doesn't care.
        if self._is_path_execution_state() and self._goal is not None:
            logger.warning("TF jump (dt=%.2fm dyaw=%.1fdeg) -> forced replan", dt_m, dyaw)
            self._publish_motion_stop()
            self._tracker.clear()  # invalidate current waypoint tracking
            self._plan()
