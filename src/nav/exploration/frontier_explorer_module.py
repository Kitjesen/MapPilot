"""WavefrontFrontierExplorer 鈥?autonomous frontier-based exploration planner.

Finds boundaries between known-free and unknown space (frontiers) via BFS,
clusters them, scores each cluster, and publishes the best goal for exploration.

Grid cell conventions (standard ROS2 OccupancyGrid):
  FREE     =  0
  OCCUPIED = 100
  UNKNOWN  = -1

Scoring weights:
  30% information gain  (cluster size 鈫?proxy for how much unknown area opens up)
  30% novelty           (distance from previously visited goals)
  20% distance          (sweet spot 3-8 m from robot)
  15% safety            (clearance from nearest obstacle)
   5% momentum          (prefer continuing in current heading)

Usage::

    bp.add(WavefrontFrontierExplorer, min_frontier_size=5, explore_rate=2.0)
    # auto_wire connects costmap + odometry
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from typing import Any

from runtime.module import Module, skill
from runtime.msgs.geometry import Pose, PoseStamped
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import is_numpy_array, np
from runtime.registry import register
from runtime.runtime_interface import map_frame_id
from runtime.stream import In, Out
from nav.mission.model.status import MissionStatus

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Grid constants
# ---------------------------------------------------------------------------

FREE     =  0
OCCUPIED = 100
UNKNOWN  = -1
FRONTIER_MAP_FRAME_ID = map_frame_id()

# Frontier scoring constants
_NOVELTY_SATURATE_DIST = 10.0     # m 鈥?novelty score saturates at this distance from visited goals
_FRONTIER_DIST_SWEET_LOW = 3.0    # m 鈥?lower bound of distance sweet-spot for frontier scoring
_FRONTIER_DIST_SWEET_HIGH = 8.0   # m 鈥?upper bound of distance sweet-spot for frontier scoring
_FRONTIER_INFO_GAIN_WEIGHT = 0.30   # scoring weight: information gain (cluster size)
_FRONTIER_NOVELTY_WEIGHT = 0.30     # scoring weight: distance from visited goals
_FRONTIER_DIST_WEIGHT = 0.20        # scoring weight: distance sweet-spot
_FRONTIER_SAFETY_WEIGHT = 0.15      # scoring weight: clearance from obstacles
_FRONTIER_MOMENTUM_WEIGHT = 0.05    # scoring weight: heading alignment

# 4-connectivity neighbours (row_delta, col_delta)
_NEIGH4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]

# 8-connectivity (used when expanding BFS wavefront)
_NEIGH8 = [
    (-1, -1), (-1, 0), (-1, 1),
    ( 0, -1),          ( 0, 1),
    ( 1, -1), ( 1, 0), ( 1, 1),
]


# ---------------------------------------------------------------------------
# Module
# ---------------------------------------------------------------------------

@register("exploration", "wavefront_frontier",
          description="Wavefront frontier exploration")
class WavefrontFrontierExplorer(Module, layer=2):
    """Autonomous exploration: BFS wavefront 鈫?frontier clusters 鈫?scored goal.

    Ports in:
        costmap      鈥?dict from OccupancyGridModule with keys:
                        'grid'       np.ndarray (H脳W, dtype int8/int16)
                        'resolution' float  (m/cell)
                        'origin_x'   float  (world x of cell [0,0])
                        'origin_y'   float  (world y of cell [0,0])
                        'width'      int
                        'height'     int
        odometry     鈥?Odometry (robot world pose)
        goal_reached 鈥?bool  (True when Navigation signals arrival)

    Ports out:
        exploration_goal 鈥?PoseStamped  (next frontier centroid in map frame)
        frontiers        鈥?list of dicts [{cx, cy, size, score}, ...]
        exploring        鈥?bool  (True while exploration thread is running)
    """

    # -- Inputs --
    costmap:      In[dict]
    exploration_grid: In[dict]
    odometry:     In[Odometry]
    goal_reached: In[bool]
    navigation_status: In[MissionStatus]

    # -- Outputs --
    exploration_goal: Out[PoseStamped]
    frontiers:        Out[list]
    exploring:        Out[bool]

    def __init__(
        self,
        min_frontier_size: int   = 5,
        safe_distance: float     = 1.0,    # m 鈥?minimum clearance from obstacles
        lookahead_distance: float = 5.0,   # m 鈥?max BFS expansion radius
        max_explored_distance: float = 15.0,  # m 鈥?frontier beyond this is ignored
        info_gain_threshold: float = 0.03,  # stop if all frontiers below this gain
        goal_timeout: float      = 30.0,   # s 鈥?give up on a goal after this
        explore_rate: float      = 2.0,    # Hz 鈥?how often to re-evaluate frontiers
        blocked_goal_radius: float = 1.0,
        blocked_goal_ttl: float = 120.0,
        **kw,
    ):
        super().__init__(**kw)

        # Parameters
        self._min_size       = min_frontier_size
        self._safe_dist      = safe_distance
        self._lookahead      = lookahead_distance
        self._max_dist       = max_explored_distance
        self._info_gain_thr  = info_gain_threshold
        self._goal_timeout   = goal_timeout
        self._rate           = explore_rate
        self._blocked_goal_radius = max(0.0, float(blocked_goal_radius))
        self._blocked_goal_ttl = max(0.0, float(blocked_goal_ttl))
        self._reachable_goal_radius = max(
            0.0,
            float(kw.get("reachable_goal_radius", 0.8)),
        )
        self._cost_obstacle_threshold = float(kw.get("cost_obstacle_threshold", 49.9))
        self._navigation_failure_grace_s = max(
            0.0,
            float(kw.get("navigation_failure_grace_s", 2.0)),
        )
        self._approach_standoff_m = max(
            0.0,
            float(kw.get("approach_standoff_m", 0.8)),
        )
        self._approach_max_target_distance_m = max(
            self._approach_standoff_m,
            float(kw.get(
                "approach_max_target_distance_m",
                max(1.5, self._approach_standoff_m + 0.7),
            )),
        )
        self._approach_goal_max_distance_m = max(
            self._approach_standoff_m,
            float(kw.get("approach_goal_max_distance_m", 3.0)),
        )

        # Costmap freshness
        self._costmap_ts: float = 0.0
        self._exploration_grid_ts: float = 0.0
        self._costmap_max_age: float = kw.get("costmap_max_age", 30.0)

        # Sensor state (updated by subscribers, read by exploration thread)
        self._costmap_data: dict | None = None
        self._exploration_grid_data: dict | None = None
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_yaw: float = 0.0
        self._state_lock = threading.Lock()

        # Goal-reached event (set by subscriber, cleared by exploration thread)
        self._goal_reached_event = threading.Event()

        # Exploration thread
        self._explore_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        # Visited goals: list of (x, y) to avoid revisiting
        self._visited_goals: list[tuple[float, float]] = []
        self._blocked_goals: list[dict[str, Any]] = []
        self._current_goal: tuple[float, float] | None = None
        self._current_goal_result: str | None = None
        self._pending_goal_failure: dict[str, Any] | None = None
        self._consecutive_low_gain: int = 0
        self._max_low_gain: int = 3  # stop after N consecutive low-gain attempts
        self._frontier_debug: dict[str, Any] = {
            "frontier_grid_source": "missing",
            "raw_cluster_count": 0,
            "frontier_cell_count": 0,
            "clusters_after_min_size": 0,
            "scored_frontier_count": 0,
            "frontiers_after_blocked_filter": 0,
            "frontiers_after_reachability_filter": 0,
            "reachable_component_size": 0,
            "reachable_direct_count": 0,
            "reachable_approach_count": 0,
            "reachable_rejected_count": 0,
            "last_empty_reason": "not_started",
        }

        # Current heading (radians) for momentum scoring
        self._current_heading: float = 0.0

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------

    def setup(self) -> None:
        self.costmap.subscribe(self._on_costmap)
        self.exploration_grid.subscribe(self._on_exploration_grid)
        self.odometry.subscribe(self._on_odometry)
        self.goal_reached.subscribe(self._on_goal_reached)
        self.navigation_status.subscribe(self._on_navigation_status)

    def start(self) -> None:
        super().start()
        self.exploring.publish(False)

    def stop(self) -> None:
        self._stop_event.set()
        self._goal_reached_event.set()
        if self._explore_thread and self._explore_thread.is_alive():
            self._explore_thread.join(timeout=3.0)
        super().stop()

    # -------------------------------------------------------------------------
    # Subscribers
    # -------------------------------------------------------------------------

    def _on_costmap(self, data: dict) -> None:
        with self._state_lock:
            self._costmap_data = data
            self._costmap_ts = time.time()

    def _on_exploration_grid(self, data: dict) -> None:
        with self._state_lock:
            self._exploration_grid_data = data
            self._exploration_grid_ts = time.time()

    def _on_odometry(self, odom: Odometry) -> None:
        with self._state_lock:
            self._robot_x   = odom.x
            self._robot_y   = odom.y
            self._robot_yaw = odom.yaw

    def _on_goal_reached(self, reached: bool) -> None:
        if reached:
            with self._state_lock:
                has_current_goal = self._current_goal is not None
            if has_current_goal:
                self._mark_current_goal_result("reached")
            else:
                self._goal_reached_event.set()

    def _on_navigation_status(self, status: MissionStatus) -> None:
        if not isinstance(status, dict):
            return
        state = str(status.get("state") or "").upper()
        if state in {
            "PLANNING",
            "EXECUTING",
            "PAUSED",
            "RECOVERING",
            "ACTIVE",
            "RUNNING",
        }:
            if self._navigation_status_matches_current_goal(
                status,
                allow_missing_goal=True,
            ):
                with self._state_lock:
                    self._pending_goal_failure = None
            return
        if state == "SUCCESS":
            if not self._navigation_status_matches_current_goal(status):
                return
        elif state in {"FAILED", "STUCK", "CANCELLED"}:
            if not self._navigation_status_matches_current_goal(
                status,
                allow_missing_goal=True,
            ):
                return
        if state == "SUCCESS":
            self._mark_current_goal_result("reached")
        elif state in {"FAILED", "STUCK", "CANCELLED"}:
            reason = str(
                status.get("failure_reason")
                or status.get("reason")
                or state.lower()
            )
            self._record_pending_goal_failure(reason)

    def _navigation_status_matches_current_goal(
        self,
        status: dict,
        *,
        allow_missing_goal: bool = False,
    ) -> bool:
        goal = status.get("goal")
        with self._state_lock:
            current = self._current_goal
        if current is None:
            return False
        if isinstance(goal, dict):
            try:
                gx = float(goal.get("x"))
                gy = float(goal.get("y"))
            except (TypeError, ValueError):
                return bool(allow_missing_goal)
        elif isinstance(goal, (list, tuple)) and len(goal) >= 2:
            try:
                gx = float(goal[0])
                gy = float(goal[1])
            except (TypeError, ValueError):
                return bool(allow_missing_goal)
        else:
            return bool(allow_missing_goal)
        tolerance = max(self._blocked_goal_radius, 0.5)
        return math.hypot(float(current[0]) - gx, float(current[1]) - gy) <= tolerance

    # -------------------------------------------------------------------------
    # Skills
    # -------------------------------------------------------------------------

    @skill
    def begin_exploration(self) -> str:
        """Start autonomous frontier exploration. Returns status string."""
        if self._explore_thread and self._explore_thread.is_alive():
            return "already_running"
        self._stop_event.clear()
        self._goal_reached_event.clear()
        self._visited_goals.clear()
        self._blocked_goals.clear()
        self._current_goal = None
        self._current_goal_result = None
        self._pending_goal_failure = None
        self._consecutive_low_gain = 0
        self._reset_frontier_debug()
        self._explore_thread = threading.Thread(
            target=self._exploration_loop,
            name="frontier_explorer",
            daemon=True,
        )
        self._explore_thread.start()
        self.exploring.publish(True)
        logger.info("WavefrontFrontierExplorer: exploration started")
        return "started"

    @skill
    def end_exploration(self) -> str:
        """Stop autonomous frontier exploration. Returns status string."""
        self._stop_event.set()
        self._goal_reached_event.set()
        if self._explore_thread and self._explore_thread.is_alive():
            self._explore_thread.join(timeout=3.0)
        self.exploring.publish(False)
        logger.info("WavefrontFrontierExplorer: exploration stopped")
        return "stopped"

    def _compute_frontier_clusters(self) -> list:
        """Internal: compute and return frontier clusters as structured data."""
        with self._state_lock:
            costmap_data = self._frontier_grid_data_unlocked()
            rx, ry, ryaw = self._robot_x, self._robot_y, self._robot_yaw

        if costmap_data is None:
            self._update_frontier_debug(last_empty_reason="missing_frontier_grid")
            return []

        grid, meta = self._parse_costmap(costmap_data)
        if grid is None:
            self._update_frontier_debug(last_empty_reason="frontier_grid_parse_failed")
            return []

        clusters = self._find_frontier_clusters(grid, meta, rx, ry)
        clusters = self._score_clusters(clusters, rx, ry, ryaw, meta)
        output = []
        for c in clusters:
            item = {
                "cx": float(c["cx"]),
                "cy": float(c["cy"]),
                "size": int(c["size"]),
                "score": float(c.get("score", 0.0)),
            }
            if "distance" in c:
                item["distance"] = float(c.get("distance", 0.0))
            if c.get("approach_goal"):
                item["approach_goal"] = True
            frontier_target = c.get("frontier_target")
            if (
                isinstance(frontier_target, (list, tuple))
                and len(frontier_target) >= 2
            ):
                item["frontier_target"] = [
                    float(frontier_target[0]),
                    float(frontier_target[1]),
                ]
            output.append(item)
        return output

    @skill
    def get_frontiers(self) -> str:
        """Return the most recently computed frontier cluster list."""
        import json
        return json.dumps(self._compute_frontier_clusters())

    @skill
    def clear_frontier_blocks(self) -> str:
        """Clear navigation-failed frontier goals from the local block list."""
        with self._state_lock:
            self._blocked_goals.clear()
        return "cleared"

    # -------------------------------------------------------------------------
    # Exploration loop (runs in background thread)
    # -------------------------------------------------------------------------

    def _exploration_loop(self) -> None:
        period = 1.0 / max(self._rate, 0.1)

        while not self._stop_event.is_set():
            loop_start = time.monotonic()

            # Snapshot sensor state
            with self._state_lock:
                costmap_data = self._frontier_grid_data_unlocked()
                costmap_ts = self._frontier_grid_ts_unlocked()
                rx, ry, ryaw = self._robot_x, self._robot_y, self._robot_yaw

            # Costmap freshness check
            if costmap_ts > 0 and time.time() - costmap_ts > self._costmap_max_age:
                logger.warning(
                    "FrontierExplorer: costmap stale (%.1fs > %.1fs) 鈥?skipping cycle",
                    time.time() - costmap_ts, self._costmap_max_age,
                )
                self._stop_event.wait(timeout=period)
                continue

            if costmap_data is None:
                logger.debug("FrontierExplorer: waiting for costmap...")
                self._update_frontier_debug(last_empty_reason="missing_frontier_grid")
                self._stop_event.wait(timeout=period)
                continue

            grid, meta = self._parse_costmap(costmap_data)
            if grid is None:
                self._update_frontier_debug(last_empty_reason="frontier_grid_parse_failed")
                self._stop_event.wait(timeout=period)
                continue

            # Detect and score frontiers
            clusters = self._find_frontier_clusters(grid, meta, rx, ry)
            scored   = self._score_clusters(clusters, rx, ry, ryaw, meta)

            # Publish frontiers for visualization (regardless of selection)
            self.frontiers.publish(scored)

            if not scored:
                logger.info("FrontierExplorer: no frontiers found 鈥?exploration complete")
                break

            best = scored[0]

            # Check information gain threshold
            max_gain = best.get("info_gain_norm", 0.0)
            if max_gain < self._info_gain_thr:
                self._consecutive_low_gain += 1
                logger.info(
                    "FrontierExplorer: low info gain (%.3f < %.3f), consecutive=%d",
                    max_gain, self._info_gain_thr, self._consecutive_low_gain,
                )
                if self._consecutive_low_gain >= self._max_low_gain:
                    logger.info("FrontierExplorer: info gain persistently low 鈥?stopping")
                    break
            else:
                self._consecutive_low_gain = 0

            # Publish exploration goal
            goal = self._make_pose_stamped(
                best["cx"],
                best["cy"],
                frame_id=str(meta.get("frame_id") or FRONTIER_MAP_FRAME_ID),
            )
            with self._state_lock:
                self._current_goal = (float(best["cx"]), float(best["cy"]))
                self._current_goal_result = None
                self._pending_goal_failure = None
            self._goal_reached_event.clear()
            self.exploration_goal.publish(goal)
            self._visited_goals.append((best["cx"], best["cy"]))
            logger.debug(
                "FrontierExplorer: goal (%.2f, %.2f) score=%.3f size=%d",
                best["cx"], best["cy"], best["score"], best["size"],
            )

            # Wait for goal completion. Navigation can emit transient FAILED
            # during replanning; only block after a grace period without
            # PLANNING/EXECUTING recovery for the same target.
            reached = self._wait_for_goal_result(timeout=self._goal_timeout)
            if not reached:
                self._mark_current_goal_result(
                    "timeout",
                    reason=f"goal_timeout_{self._goal_timeout:.1f}s",
                )
                logger.debug(
                    "FrontierExplorer: goal timeout after %.1fs 鈥?selecting next frontier",
                    self._goal_timeout,
                )

            # Sleep remaining period time (avoids tight loop on fast goal_reached)
            elapsed = time.monotonic() - loop_start
            remaining = period - elapsed
            if remaining > 0:
                self._stop_event.wait(timeout=remaining)

        self.exploring.publish(False)
        logger.info("WavefrontFrontierExplorer: exploration loop exited")

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        exploring = self._explore_thread is not None and self._explore_thread.is_alive()
        info["exploration_state"] = "exploring" if exploring else "idle"
        info["frontier_count"] = len(self._visited_goals)
        with self._state_lock:
            self._prune_blocked_goals_unlocked(time.time())
            current_goal = self._current_goal
            info["blocked_goal_count"] = len(self._blocked_goals)
            info["current_goal"] = (
                None
                if current_goal is None
                else [float(current_goal[0]), float(current_goal[1])]
            )
            info["current_goal_result"] = self._current_goal_result
            pending = self._pending_goal_failure
            info["pending_goal_failure"] = (
                None
                if pending is None
                else {
                    "reason": str(pending.get("reason") or ""),
                    "age_s": max(
                        0.0,
                        time.monotonic() - float(pending.get("ts") or 0.0),
                    ),
                }
            )
            info["frontier_grid_source"] = self._frontier_grid_source_unlocked()
            info["costmap_source"] = self._grid_source_name(self._costmap_data)
            info["exploration_grid_source"] = self._grid_source_name(
                self._exploration_grid_data
            )
            info["visited_goal_count"] = len(self._visited_goals)
            info["frontier_grid_counts"] = self._grid_counts_from_data(
                self._frontier_grid_data_unlocked()
            )
            info["costmap_counts"] = self._grid_counts_from_data(self._costmap_data)
            info["exploration_grid_counts"] = self._grid_counts_from_data(
                self._exploration_grid_data
            )
            info["frontier_debug"] = dict(self._frontier_debug)
            info["current_frontier_candidates"] = int(
                self._frontier_debug.get("scored_frontier_count") or 0
            )
            info["current_frontier_raw_clusters"] = int(
                self._frontier_debug.get("raw_cluster_count") or 0
            )
        return info

    def _wait_for_goal_result(self, *, timeout: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout))
        while not self._stop_event.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            if self._goal_reached_event.wait(timeout=min(0.2, remaining)):
                return True
            pending_reason = self._pending_goal_failure_ready()
            if pending_reason is not None:
                self._mark_current_goal_result("failed", reason=pending_reason)
                return True
        return True

    def _record_pending_goal_failure(self, reason: str) -> None:
        if self._navigation_failure_grace_s <= 0.0:
            self._mark_current_goal_result("failed", reason=reason)
            return
        with self._state_lock:
            if self._current_goal is None or self._current_goal_result is not None:
                return
            goal = self._current_goal
            self._pending_goal_failure = {
                "x": float(goal[0]),
                "y": float(goal[1]),
                "reason": reason or "navigation_failed",
                "ts": time.monotonic(),
            }

    def _pending_goal_failure_ready(self) -> str | None:
        with self._state_lock:
            pending = self._pending_goal_failure
            if pending is None:
                return None
            age = time.monotonic() - float(pending.get("ts") or 0.0)
            if age < self._navigation_failure_grace_s:
                return None
            return str(pending.get("reason") or "navigation_failed")

    def _mark_current_goal_result(self, result: str, *, reason: str = "") -> None:
        block_goal = result in {"failed", "timeout"}
        with self._state_lock:
            if self._current_goal is None:
                return
            if self._current_goal_result is not None:
                return
            goal = self._current_goal
            self._current_goal_result = result
            self._pending_goal_failure = None
            if block_goal:
                self._blocked_goals.append({
                    "x": float(goal[0]),
                    "y": float(goal[1]),
                    "reason": reason or result,
                    "ts": time.time(),
                })
        self._goal_reached_event.set()

    def _blocked_goal_snapshot(self) -> list[dict[str, Any]]:
        with self._state_lock:
            self._prune_blocked_goals_unlocked(time.time())
            return [dict(item) for item in self._blocked_goals]

    def _prune_blocked_goals_unlocked(self, now: float) -> None:
        ttl = self._blocked_goal_ttl
        if ttl <= 0.0:
            return
        self._blocked_goals = [
            item
            for item in self._blocked_goals
            if now - float(item.get("ts") or 0.0) <= ttl
        ]

    def _is_blocked_goal(
        self,
        x: float,
        y: float,
        blocked_goals: list[dict[str, Any]] | None = None,
    ) -> bool:
        radius = self._blocked_goal_radius
        if radius <= 0.0:
            return False
        blocked = blocked_goals if blocked_goals is not None else self._blocked_goal_snapshot()
        for item in blocked:
            bx = float(item.get("x") or 0.0)
            by = float(item.get("y") or 0.0)
            if math.hypot(float(x) - bx, float(y) - by) <= radius:
                return True
        return False

    # -------------------------------------------------------------------------
    # Costmap parsing
    # -------------------------------------------------------------------------

    def _parse_costmap(
        self, data: dict
    ) -> tuple[np.ndarray | None, dict | None]:
        """Extract grid array and geometry metadata from costmap dict.

        Returns (grid_2d, meta) where meta has resolution/origin_x/origin_y,
        or (None, None) on error.
        """
        try:
            grid = data["grid"]
            if not is_numpy_array(grid):
                grid = np.array(grid, dtype=np.int16)
            else:
                grid = grid.astype(np.int16)

            h = int(data.get("height", grid.shape[0]))
            w = int(data.get("width", grid.shape[1] if grid.ndim == 2 else 0))

            if grid.ndim == 1:
                grid = grid.reshape(h, w)

            meta = {
                "resolution": float(data.get("resolution", 0.05)),
                "origin_x":   float(data.get("origin_x", 0.0)),
                "origin_y":   float(data.get("origin_y", 0.0)),
                "width":      w,
                "height":     h,
                "frame_id":   str(data.get("frame_id") or FRONTIER_MAP_FRAME_ID),
            }
            return grid, meta

        except Exception as exc:
            logger.warning("FrontierExplorer: costmap parse error: %s", exc)
            return None, None

    def _frontier_grid_data_unlocked(self) -> dict | None:
        return self._exploration_grid_data or self._costmap_data

    def _frontier_grid_ts_unlocked(self) -> float:
        return self._exploration_grid_ts or self._costmap_ts

    def _frontier_grid_source_unlocked(self) -> str:
        if self._exploration_grid_data is not None:
            return "exploration_grid"
        if self._costmap_data is not None:
            return "costmap"
        return "missing"

    @staticmethod
    def _grid_source_name(data: dict | None) -> str:
        if not isinstance(data, dict):
            return "missing"
        source = data.get("source") or data.get("frame_id") or "present"
        return str(source)

    @staticmethod
    def _grid_counts_from_data(data: dict | None) -> dict[str, int]:
        if not isinstance(data, dict):
            return {}
        counts = data.get("counts")
        if isinstance(counts, dict):
            return {
                "unknown": int(counts.get("unknown") or 0),
                "free": int(counts.get("free") or 0),
                "occupied": int(counts.get("occupied") or 0),
            }
        grid = data.get("grid")
        if grid is None:
            return {}
        arr = np.asarray(grid)
        if arr.size == 0:
            return {"unknown": 0, "free": 0, "occupied": 0}
        return {
            "unknown": int((arr < 0).sum()),
            "free": int((arr == FREE).sum()),
            "occupied": int((arr >= OCCUPIED).sum()),
        }

    def _reset_frontier_debug(self) -> None:
        with self._state_lock:
            self._frontier_debug = {
                "frontier_grid_source": self._frontier_grid_source_unlocked(),
                "raw_cluster_count": 0,
                "frontier_cell_count": 0,
                "clusters_after_min_size": 0,
                "scored_frontier_count": 0,
                "frontiers_after_blocked_filter": 0,
                "frontiers_after_reachability_filter": 0,
                "reachable_component_size": 0,
                "reachable_direct_count": 0,
                "reachable_approach_count": 0,
                "reachable_rejected_count": 0,
                "last_empty_reason": "reset",
            }

    def _update_frontier_debug(self, **values: Any) -> None:
        cleaned: dict[str, Any] = {}
        for key, value in values.items():
            if isinstance(value, np.generic):
                value = value.item()
            elif isinstance(value, tuple):
                value = list(value)
            cleaned[key] = value
        with self._state_lock:
            self._frontier_debug.update(cleaned)

    # -------------------------------------------------------------------------
    # Wavefront BFS frontier detection
    # -------------------------------------------------------------------------

    def _find_frontier_clusters(
        self,
        grid: np.ndarray,
        meta: dict,
        robot_x: float,
        robot_y: float,
    ) -> list[dict]:
        """BFS from robot position to find frontier cells, then cluster them.

        A frontier cell is a FREE cell that has at least one UNKNOWN neighbour.
        Returns a list of cluster dicts: {cx, cy, size, cells}.
        """
        res      = meta["resolution"]
        origin_x = meta["origin_x"]
        origin_y = meta["origin_y"]
        h, w     = grid.shape

        # Convert robot world position to grid cell
        start_col = int((robot_x - origin_x) / res)
        start_row = int((robot_y - origin_y) / res)

        if not (0 <= start_row < h and 0 <= start_col < w):
            logger.debug("FrontierExplorer: robot outside grid bounds")
            self._update_frontier_debug(
                frontier_grid_source=self._frontier_grid_source_unlocked(),
                grid_shape=[int(h), int(w)],
                grid_resolution=float(res),
                grid_origin=[float(origin_x), float(origin_y)],
                robot_xy=[float(robot_x), float(robot_y)],
                robot_cell=[int(start_row), int(start_col)],
                robot_cell_in_bounds=False,
                raw_cluster_count=0,
                frontier_cell_count=0,
                clusters_after_min_size=0,
                scored_frontier_count=0,
                last_empty_reason="robot_outside_frontier_grid",
            )
            return []

        # BFS max radius in cells
        max_radius_cells = int(self._max_dist / res)

        # BFS through free cells, collect frontier cells
        visited     = np.zeros((h, w), dtype=bool)
        frontier_mask = np.zeros((h, w), dtype=bool)

        queue = deque()
        queue.append((start_row, start_col, 0))
        visited[start_row, start_col] = True

        while queue:
            r, c, depth = queue.popleft()

            if depth > max_radius_cells:
                continue

            cell_val = int(grid[r, c])

            # Only propagate through known free cells. UNKNOWN cells are the
            # frontier boundary and must not be traversed as usable space.
            if cell_val < 0 or cell_val == OCCUPIED or cell_val > 50:
                continue

            # Check if this free cell borders an unknown cell 鈫?frontier
            is_frontier = False
            for dr, dc in _NEIGH4:
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w:
                    nval = int(grid[nr, nc])
                    if nval == UNKNOWN or nval < 0:
                        is_frontier = True
                        break

            if is_frontier:
                frontier_mask[r, c] = True

            # Expand neighbours
            for dr, dc in _NEIGH8:
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w and not visited[nr, nc]:
                    nval = int(grid[nr, nc])
                    # Traverse free or mildly inflated known cells (< 50).
                    if 0 <= nval <= 50 and nval != OCCUPIED:
                        visited[nr, nc] = True
                        queue.append((nr, nc, depth + 1))

        # Connected-component labelling on frontier_mask
        clusters: list[dict] = []
        cluster_visited = np.zeros((h, w), dtype=bool)

        frontier_cell_count = int(frontier_mask.sum())
        rejected_min_size = 0
        frontier_rows, frontier_cols = np.where(frontier_mask)

        for r0, c0 in zip(frontier_rows.tolist(), frontier_cols.tolist()):
            if cluster_visited[r0, c0]:
                continue

            # Flood-fill this cluster with 8-connectivity
            cells: list[tuple[int, int]] = []
            cq = deque()
            cq.append((r0, c0))
            cluster_visited[r0, c0] = True

            while cq:
                cr, cc = cq.popleft()
                cells.append((cr, cc))
                for dr, dc in _NEIGH8:
                    nr, nc = cr + dr, cc + dc
                    if (0 <= nr < h and 0 <= nc < w
                            and not cluster_visited[nr, nc]
                            and frontier_mask[nr, nc]):
                        cluster_visited[nr, nc] = True
                        cq.append((nr, nc))

            if len(cells) < self._min_size:
                rejected_min_size += 1
                continue

            # Use the frontier cell closest to the cluster centroid as the goal.
            # A concave or ring-shaped frontier can have its geometric centroid
            # in already-known free space, even at the robot pose. The goal must
            # remain on the frontier boundary so navigation makes progress.
            mean_row = sum(r for r, _ in cells) / len(cells)
            mean_col = sum(c for _, c in cells) / len(cells)
            goal_row, goal_col = min(
                cells,
                key=lambda cell: (
                    (cell[0] - mean_row) * (cell[0] - mean_row)
                    + (cell[1] - mean_col) * (cell[1] - mean_col)
                ),
            )
            cx = origin_x + goal_col * res
            cy = origin_y + goal_row * res

            clusters.append({
                "cx":    cx,
                "cy":    cy,
                "size":  len(cells),
                "cells": cells,
                "centroid": [origin_x + mean_col * res, origin_y + mean_row * res],
            })

        self._update_frontier_debug(
            frontier_grid_source=self._frontier_grid_source_unlocked(),
            grid_shape=[int(h), int(w)],
            grid_resolution=float(res),
            grid_origin=[float(origin_x), float(origin_y)],
            robot_xy=[float(robot_x), float(robot_y)],
            robot_cell=[int(start_row), int(start_col)],
            robot_cell_in_bounds=True,
            robot_cell_value=int(grid[start_row, start_col]),
            frontier_cell_count=frontier_cell_count,
            raw_cluster_count=len(clusters),
            clusters_after_min_size=len(clusters),
            clusters_rejected_min_size=rejected_min_size,
            last_empty_reason="no_frontier_cells"
            if frontier_cell_count <= 0
            else (
                "all_frontier_clusters_below_min_size"
                if len(clusters) <= 0
                else "frontier_clusters_detected"
            ),
        )
        return clusters

    # -------------------------------------------------------------------------
    # Cluster scoring
    # -------------------------------------------------------------------------

    def _score_clusters(
        self,
        clusters: list[dict],
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        meta: dict,
    ) -> list[dict]:
        """Score and sort clusters (highest first).

        Weights:
          0.30 鈥?information gain  (normalised cluster size)
          0.30 鈥?novelty           (distance from nearest visited goal)
          0.20 鈥?distance          (sweet-spot 3-8 m)
          0.15 鈥?safety            (clearance from obstacles, approx via costmap)
          0.05 鈥?momentum          (alignment with current heading)
        """
        self._update_frontier_debug(
            raw_cluster_count=len(clusters),
            frontiers_after_blocked_filter=0,
            frontiers_after_reachability_filter=0,
            scored_frontier_count=0,
            last_empty_reason="no_raw_frontier_clusters"
            if not clusters
            else "scoring_frontiers",
        )
        if not clusters:
            return []

        blocked_goals = self._blocked_goal_snapshot()
        if blocked_goals:
            before_blocked = len(clusters)
            clusters = [
                c for c in clusters
                if not self._is_blocked_goal(c["cx"], c["cy"], blocked_goals)
            ]
            self._update_frontier_debug(
                frontiers_after_blocked_filter=len(clusters),
                blocked_filter_rejected_count=before_blocked - len(clusters),
            )
            if not clusters:
                self._update_frontier_debug(last_empty_reason="all_frontiers_blocked")
                return []
        else:
            self._update_frontier_debug(
                frontiers_after_blocked_filter=len(clusters),
                blocked_filter_rejected_count=0,
            )

        clusters = self._filter_reachable_clusters(clusters, robot_x, robot_y)
        if not clusters:
            self._update_frontier_debug(
                frontiers_after_reachability_filter=0,
                scored_frontier_count=0,
                last_empty_reason="all_frontiers_unreachable",
            )
            return []
        self._update_frontier_debug(frontiers_after_reachability_filter=len(clusters))

        max_size = max(c["size"] for c in clusters) or 1

        # Compute raw scores
        for c in clusters:
            dx = c["cx"] - robot_x
            dy = c["cy"] - robot_y
            dist = math.hypot(dx, dy)

            # Information gain: normalised cluster size
            info_gain_norm = c["size"] / max_size
            c["info_gain_norm"] = info_gain_norm

            # Novelty: minimum distance from all previously visited goals
            if self._visited_goals:
                novelty_dist = min(
                    math.hypot(c["cx"] - vx, c["cy"] - vy)
                    for vx, vy in self._visited_goals
                )
                novelty_score = min(novelty_dist / _NOVELTY_SATURATE_DIST, 1.0)
            else:
                novelty_score = 1.0

            # Distance: sweet spot _FRONTIER_DIST_SWEET_LOW-_FRONTIER_DIST_SWEET_HIGH m
            sweet_mid = (_FRONTIER_DIST_SWEET_LOW + _FRONTIER_DIST_SWEET_HIGH) / 2.0
            dist_score = max(0.0, 1.0 - abs(dist - sweet_mid) / sweet_mid)

            # Safety: approximate using distance to nearest OCCUPIED cell
            # We use the robot鈫抍entroid midpoint check (cheap proxy)
            safety_score = self._approx_safety(
                c["cx"], c["cy"], robot_x, robot_y, meta
            )

            # Momentum: dot-product alignment with robot heading
            if dist > 0.01:
                goal_angle = math.atan2(dy, dx)
                angle_diff = abs(_wrap_angle(goal_angle - robot_yaw))
                momentum_score = max(0.0, 1.0 - angle_diff / math.pi)
            else:
                momentum_score = 0.0

            # Weighted sum (weights from module docstring)
            score = (
                _FRONTIER_INFO_GAIN_WEIGHT * info_gain_norm
                + _FRONTIER_NOVELTY_WEIGHT * novelty_score
                + _FRONTIER_DIST_WEIGHT * dist_score
                + _FRONTIER_SAFETY_WEIGHT * safety_score
                + _FRONTIER_MOMENTUM_WEIGHT * momentum_score
            )

            c["score"]          = score
            c["info_gain_norm"] = info_gain_norm
            c["novelty_score"]  = novelty_score
            c["dist_score"]     = dist_score
            c["safety_score"]   = safety_score
            c["momentum_score"] = momentum_score
            c["distance"]       = dist

        # Sort descending by score; strip heavy 'cells' list for output
        clusters.sort(key=lambda c: c["score"], reverse=True)

        output = []
        for c in clusters:
            output.append({
                "cx":             c["cx"],
                "cy":             c["cy"],
                "size":           c["size"],
                "score":          c["score"],
                "info_gain_norm": c["info_gain_norm"],
                "novelty_score":  c["novelty_score"],
                "dist_score":     c["dist_score"],
                "safety_score":   c["safety_score"],
                "momentum_score": c["momentum_score"],
                "distance":       c["distance"],
            })
        self._update_frontier_debug(
            scored_frontier_count=len(output),
            selected_frontier=[
                float(output[0]["cx"]),
                float(output[0]["cy"]),
            ] if output else None,
            selected_frontier_score=float(output[0]["score"]) if output else 0.0,
            selected_frontier_approach=bool(clusters[0].get("approach_goal", False))
            if clusters
            else False,
            last_empty_reason="scored_frontiers_ready" if output else "no_scored_frontiers",
        )
        return output

    def _filter_reachable_clusters(
        self,
        clusters: list[dict],
        robot_x: float,
        robot_y: float,
    ) -> list[dict]:
        with self._state_lock:
            costmap_data = self._costmap_data
        if costmap_data is None:
            self._update_frontier_debug(
                reachability_filter_applied=False,
                reachability_filter_reason="missing_costmap",
                frontiers_after_reachability_filter=len(clusters),
                reachable_component_size=0,
            )
            return clusters

        grid, meta = self._parse_costmap(costmap_data)
        if grid is None or meta is None:
            self._update_frontier_debug(
                reachability_filter_applied=False,
                reachability_filter_reason="costmap_parse_failed",
                frontiers_after_reachability_filter=len(clusters),
                reachable_component_size=0,
            )
            return clusters

        component = self._reachable_costmap_component(grid, meta, robot_x, robot_y)
        if not component:
            self._update_frontier_debug(
                reachability_filter_applied=True,
                reachability_filter_reason="empty_reachable_component",
                frontiers_after_reachability_filter=0,
                reachable_component_size=0,
                reachable_direct_count=0,
                reachable_approach_count=0,
                reachable_rejected_count=len(clusters),
            )
            return []

        approach: list[dict] = []
        direct: list[dict] = []
        for cluster in clusters:
            has_frontier_cells = bool(cluster.get("cells"))
            goal_cell = self._world_to_costmap_cell(
                float(cluster["cx"]),
                float(cluster["cy"]),
                float(meta["origin_x"]),
                float(meta["origin_y"]),
                float(meta["resolution"]),
            )
            candidate = self._best_reachable_approach_cell(
                component,
                meta,
                robot_x,
                robot_y,
                float(cluster["cx"]),
                float(cluster["cy"]),
                require_target_near=True,
            )
            if has_frontier_cells and candidate is None:
                continue
            if candidate is not None and (
                has_frontier_cells
                or goal_cell is None
                or goal_cell not in component
            ):
                row, col = candidate
                wx, wy = self._costmap_cell_to_world(
                    row,
                    col,
                    float(meta["origin_x"]),
                    float(meta["origin_y"]),
                    float(meta["resolution"]),
                )
                adjusted = dict(cluster)
                adjusted["frontier_target"] = [float(cluster["cx"]), float(cluster["cy"])]
                adjusted["approach_goal"] = True
                adjusted["cx"] = wx
                adjusted["cy"] = wy
                approach.append(adjusted)
                continue
            if goal_cell is not None and goal_cell in component:
                direct.append(cluster)
                continue
        selected = approach if approach else direct
        self._update_frontier_debug(
            reachability_filter_applied=True,
            reachability_filter_reason="approach" if approach else "direct" if direct else "none",
            frontiers_after_reachability_filter=len(selected),
            reachable_component_size=len(component),
            reachable_direct_count=len(direct),
            reachable_approach_count=len(approach),
            reachable_rejected_count=max(0, len(clusters) - len(direct) - len(approach)),
        )
        return selected

    def _is_costmap_reachable(
        self,
        grid: np.ndarray,
        meta: dict,
        robot_x: float,
        robot_y: float,
        goal_x: float,
        goal_y: float,
    ) -> bool:
        res = float(meta["resolution"])
        if res <= 0.0:
            return False
        h, w = grid.shape
        origin_x = float(meta["origin_x"])
        origin_y = float(meta["origin_y"])
        start = self._world_to_costmap_cell(robot_x, robot_y, origin_x, origin_y, res)
        goal = self._world_to_costmap_cell(goal_x, goal_y, origin_x, origin_y, res)
        if start is None or goal is None:
            return False
        sr, sc = start
        gr, gc = goal
        if not (0 <= sr < h and 0 <= sc < w and 0 <= gr < h and 0 <= gc < w):
            return False
        start_cell = (sr, sc)
        if not self._is_traversable_cost_cell(grid[sr, sc]):
            nearest_start = self._nearest_traversable_cell(grid, sr, sc, res)
            if nearest_start is None:
                return False
            start_cell = nearest_start

        goal_cell = (gr, gc)
        if not self._is_traversable_cost_cell(grid[gr, gc]):
            goal_cell = self._nearest_traversable_cell(grid, gr, gc, res)
            if goal_cell is None:
                return False

        return goal_cell in self._reachable_costmap_component_from_cell(
            grid,
            start_cell,
            res,
        )

    def _reachable_costmap_component(
        self,
        grid: np.ndarray,
        meta: dict,
        robot_x: float,
        robot_y: float,
    ) -> set[tuple[int, int]]:
        res = float(meta["resolution"])
        start = self._world_to_costmap_cell(
            robot_x,
            robot_y,
            float(meta["origin_x"]),
            float(meta["origin_y"]),
            res,
        )
        if start is None:
            return set()
        h, w = grid.shape
        sr, sc = start
        if not (0 <= sr < h and 0 <= sc < w):
            return set()
        start_cell = (sr, sc)
        if not self._is_traversable_cost_cell(grid[sr, sc]):
            nearest_start = self._nearest_traversable_cell(grid, sr, sc, res)
            if nearest_start is None:
                return set()
            start_cell = nearest_start
        return self._reachable_costmap_component_from_cell(grid, start_cell, res)

    def _reachable_costmap_component_from_cell(
        self,
        grid: np.ndarray,
        start_cell: tuple[int, int],
        resolution: float,
    ) -> set[tuple[int, int]]:
        h, w = grid.shape
        sr, sc = start_cell
        if not (0 <= sr < h and 0 <= sc < w):
            return set()
        if not self._is_traversable_cost_cell(grid[sr, sc]):
            return set()
        max_depth = int(max(self._max_dist, self._lookahead, 1.0) / resolution) + 2
        visited = np.zeros((h, w), dtype=bool)
        component: set[tuple[int, int]] = set()
        queue = deque()
        queue.append((sr, sc, 0))
        visited[sr, sc] = True
        while queue:
            r, c, depth = queue.popleft()
            component.add((r, c))
            if depth >= max_depth:
                continue
            for dr, dc in _NEIGH8:
                nr, nc = r + dr, c + dc
                if dr and dc and (
                    not self._costmap_cell_free(grid, r, c + dc)
                    or not self._costmap_cell_free(grid, r + dr, c)
                ):
                    continue
                if (
                    0 <= nr < h
                    and 0 <= nc < w
                    and not visited[nr, nc]
                    and self._costmap_cell_free(grid, nr, nc)
                ):
                    visited[nr, nc] = True
                    queue.append((nr, nc, depth + 1))
        return component

    def _best_reachable_approach_cell(
        self,
        component: set[tuple[int, int]],
        meta: dict,
        robot_x: float,
        robot_y: float,
        target_x: float,
        target_y: float,
        *,
        require_target_near: bool = True,
    ) -> tuple[int, int] | None:
        if not component:
            return None
        origin_x = float(meta["origin_x"])
        origin_y = float(meta["origin_y"])
        res = float(meta["resolution"])
        robot = np.array([float(robot_x), float(robot_y)], dtype=float)
        target = np.array([float(target_x), float(target_y)], dtype=float)
        if not np.isfinite(robot).all() or not np.isfinite(target).all():
            logger.warning("Non-finite robot/target in goal selection, returning None")
            return None
        target_vec = target - robot
        target_dist = float(np.linalg.norm(target_vec))
        if target_dist <= 1e-6:
            return None
        direction = target_vec / target_dist
        max_goal_dist = min(
            max(self._lookahead, self._max_dist, 1.0),
            self._approach_goal_max_distance_m,
        )
        min_goal_dist = min(0.6, max_goal_dist)
        min_target_standoff = min(
            max(self._approach_standoff_m, res),
            max(target_dist - res, res),
        )
        best: tuple[float, int, int] | None = None
        fallback: tuple[float, int, int] | None = None
        for row, col in component:
            wx, wy = self._costmap_cell_to_world(row, col, origin_x, origin_y, res)
            pos = np.array([wx, wy], dtype=float)
            if not np.isfinite(pos).all():
                continue
            delta = pos - robot
            if not np.isfinite(delta).all():
                continue
            dist_from_robot = float(np.linalg.norm(delta))
            if dist_from_robot < max(min_goal_dist, res):
                continue
            if dist_from_robot > max_goal_dist + res:
                continue
            target_delta = pos - target
            if not np.isfinite(target_delta).all():
                continue
            dist_to_target = float(np.linalg.norm(target_delta))
            if dist_to_target < min_target_standoff:
                continue
            progress = float(np.dot(delta, direction))
            fallback_score = dist_to_target - 0.05 * dist_from_robot
            if fallback is None or fallback_score < fallback[0]:
                fallback = (fallback_score, row, col)
            if progress <= 0.0:
                continue
            lateral_vec = delta - progress * direction
            if not np.isfinite(lateral_vec).all():
                continue
            lateral = float(np.linalg.norm(lateral_vec))
            score = dist_to_target + 0.25 * lateral - 0.1 * progress
            if best is None or score < best[0]:
                best = (score, row, col)
        selected = best or fallback
        if selected is None:
            return None
        wx, wy = self._costmap_cell_to_world(
            selected[1],
            selected[2],
            origin_x,
            origin_y,
            res,
        )
        if (
            require_target_near
            and math.hypot(wx - target_x, wy - target_y) > self._approach_max_target_distance_m
        ):
            return None
        return selected[1], selected[2]

    def _nearest_traversable_cell(
        self,
        grid: np.ndarray,
        row: int,
        col: int,
        resolution: float,
    ) -> tuple[int, int] | None:
        radius_cells = int(math.ceil(self._reachable_goal_radius / max(resolution, 1e-6)))
        if radius_cells <= 0:
            return None
        h, w = grid.shape
        best: tuple[float, int, int] | None = None
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                nr, nc = row + dr, col + dc
                if 0 <= nr < h and 0 <= nc < w and self._is_traversable_cost_cell(grid[nr, nc]):
                    d2 = float(dr * dr + dc * dc)
                    if best is None or d2 < best[0]:
                        best = (d2, nr, nc)
        if best is None:
            return None
        return best[1], best[2]

    @staticmethod
    def _world_to_costmap_cell(
        x: float,
        y: float,
        origin_x: float,
        origin_y: float,
        resolution: float,
    ) -> tuple[int, int] | None:
        if resolution <= 0.0:
            return None
        col = int(round((float(x) - origin_x) / resolution))
        row = int(round((float(y) - origin_y) / resolution))
        return row, col

    @staticmethod
    def _costmap_cell_to_world(
        row: int,
        col: int,
        origin_x: float,
        origin_y: float,
        resolution: float,
    ) -> tuple[float, float]:
        return (
            float(origin_x) + float(col) * float(resolution),
            float(origin_y) + float(row) * float(resolution),
        )

    def _is_traversable_cost_cell(self, value: Any) -> bool:
        try:
            v = float(value)
        except (TypeError, ValueError):
            return False
        return math.isfinite(v) and 0.0 <= v < self._cost_obstacle_threshold

    def _costmap_cell_free(self, grid: np.ndarray, row: int, col: int) -> bool:
        h, w = grid.shape
        return (
            0 <= row < h
            and 0 <= col < w
            and self._is_traversable_cost_cell(grid[row, col])
        )

    def _approx_safety(
        self,
        cx: float,
        cy: float,
        robot_x: float,
        robot_y: float,
        meta: dict,
    ) -> float:
        """Estimate clearance at the frontier centroid.

        Samples cells in a small radius around the centroid and counts
        obstacle-free cells as a fraction 鈫?safety score in [0, 1].
        """
        with self._state_lock:
            costmap_data = self._costmap_data
        if costmap_data is None:
            return 0.5

        try:
            grid, _ = self._parse_costmap(costmap_data)
            if grid is None:
                return 0.5

            res      = meta["resolution"]
            origin_x = meta["origin_x"]
            origin_y = meta["origin_y"]
            h, w     = grid.shape

            safe_cells_r = int(self._safe_dist / res)
            cr = int((cy - origin_y) / res)
            cc = int((cx - origin_x) / res)

            total = 0
            free  = 0
            for dr in range(-safe_cells_r, safe_cells_r + 1):
                for dc in range(-safe_cells_r, safe_cells_r + 1):
                    nr, nc = cr + dr, cc + dc
                    if 0 <= nr < h and 0 <= nc < w:
                        total += 1
                        val = int(grid[nr, nc])
                        if 0 <= val <= 50 and val != OCCUPIED:
                            free += 1

            return (free / total) if total > 0 else 0.5

        except Exception:
            return 0.5

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------

    def _make_pose_stamped(
        self,
        x: float,
        y: float,
        *,
        frame_id: str = FRONTIER_MAP_FRAME_ID,
    ) -> PoseStamped:
        """Create a PoseStamped in the map frame at (x, y, 0)."""
        return PoseStamped(
            pose=Pose(float(x), float(y), 0.0),
            frame_id=frame_id,
            ts=time.time(),
        )


# ---------------------------------------------------------------------------
# Angle utility
# ---------------------------------------------------------------------------

def _wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi
