"""TAREExplorerModule - connects TARE exploration to LingTu navigation.

Architecture
------------
Default production compatibility can still subscribe to endpoint-owned DDS TARE
output. The preferred LingTu path is ``transport_mode="in_process"``, where the
module runs ``PortableTAREPolicy`` directly from ``exploration_grid`` and
odometry, then publishes the same framework ports.

Port contract
-------------
Output port ``exploration_goal: Out[PoseStamped]`` is the shared Host
exploration goal contract consumed by navigation adapters.

DDS topic contract is centralized in ``topics`` and ``runtime.runtime_interface``.

Start control: ``start_tare_exploration`` enables goal emission. In DDS mode it
also publishes the configured external start topic.
"""

from __future__ import annotations

import logging
import math
import os
import threading
import time as _time
from typing import Any

from runtime.backend_status import BackendStatus
from runtime.module import skill
from runtime.msgs.nav import Odometry
from runtime.registry import register
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.stream import Out

from ..base import ExploreModule
from .policy import PortableTAREPolicy
from .topics import (
    EXPLORATION_FINISH,
    EXPLORATION_LOCAL_PATH,
    EXPLORATION_RUNTIME,
    EXPLORATION_START,
)

logger = logging.getLogger(__name__)

_DEFAULT_GOAL_FRAME_ID = topic_default_frame_id(TOPICS.exploration_way_point)


@register("exploration", "tare", description="CMU TARE hierarchical exploration bridge")
class TAREExplorerModule(ExploreModule, layer=5):
    """Standardized ExploreModule bridge for the TARE planner.

    Shared exploration ports (``exploration_goal``/``exploration_path``/
    ``exploring``/``alive``/``odometry``/``exploration_grid``/
    ``navigation_status``) are inherited from :class:`ExploreModule`. The TARE
    algorithm lives entirely in C++ (``lingtu_explore_kernel``); in-process
    planning delegates to :class:`PortableTAREPolicy`, which wraps the nanobind
    backend.
    """

    tare_stats: Out[dict]  # per-cycle runtime + health

    def __init__(
        self,
        way_point_topic: str = TOPICS.exploration_way_point,
        path_topic: str = EXPLORATION_LOCAL_PATH,
        runtime_topic: str = EXPLORATION_RUNTIME,
        finish_topic: str = EXPLORATION_FINISH,
        start_topic: str = EXPLORATION_START,
        goal_frame_id: str = "",
        way_point_timeout_s: float = 15.0,
        auto_start: bool = True,
        hold_active_goal_until_terminal: bool = False,
        prefer_path_strategy: bool = False,
        path_goal_min_distance_m: float = 1.0,
        path_goal_spacing_m: float = 0.75,
        path_start_tolerance_m: float = 1.5,
        path_max_goal_count: int = 12,
        path_strategy_timeout_s: float = 4.0,
        path_strategy_fallback_to_waypoint: bool = True,
        navigation_goal_match_tolerance_m: float = 1.0,
        max_waypoint_distance_m: float = 0.0,
        waypoint_odometry_timeout_s: float = 5.0,
        transport_mode: str = "in_process",
        policy_rate_hz: float = 1.0,
        configured_backend: str = "tare",
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._backend_status = BackendStatus.configured_as(configured_backend)
        self._transport_mode = str(transport_mode or "in_process").strip().lower()
        if self._transport_mode not in {"dds", "in_process"}:
            raise ValueError("transport_mode must be 'dds' or 'in_process'")
        self._way_point_topic = way_point_topic
        self._path_topic = path_topic
        self._runtime_topic = runtime_topic
        self._finish_topic = finish_topic
        self._start_topic = start_topic
        self._goal_frame_id = str(goal_frame_id or "")
        self._way_point_timeout_s = way_point_timeout_s
        self._auto_start = auto_start
        self._hold_active_goal_until_terminal = bool(hold_active_goal_until_terminal)
        self._prefer_path_strategy = prefer_path_strategy
        self._path_goal_min_distance_m = float(path_goal_min_distance_m)
        self._path_goal_spacing_m = float(path_goal_spacing_m)
        self._path_start_tolerance_m = float(path_start_tolerance_m)
        self._path_max_goal_count = max(2, int(path_max_goal_count))
        self._path_strategy_timeout_s = float(path_strategy_timeout_s)
        self._path_strategy_fallback_to_waypoint = bool(path_strategy_fallback_to_waypoint)
        self._navigation_goal_match_tolerance_m = max(
            0.1,
            float(navigation_goal_match_tolerance_m),
        )
        self._max_waypoint_distance_m = max(0.0, float(max_waypoint_distance_m))
        self._waypoint_odometry_timeout_s = max(
            0.0,
            float(waypoint_odometry_timeout_s),
        )

        self._dds_transport = None  # C++ TareDdsTransport (DDS mode)
        self._dds_spin_thread: threading.Thread | None = None

        # Runtime state
        self._last_waypoint_ts: float = 0.0
        self._last_runtime_ms: float = 0.0
        self._last_finish: bool = False
        self._waypoint_count: int = 0
        self._path_count: int = 0
        self._strategy_path_count: int = 0
        self._last_strategy_path_ts: float = 0.0
        self._last_strategy_goal_count: int = 0
        self._last_strategy_path_reject_reason: str = ""
        self._last_goal_candidates: list[tuple[float, float]] = []
        self._active_goal_xy: tuple[float, float] | None = None
        self._suppressed_waypoint_count: int = 0
        self._suppressed_far_waypoint_count: int = 0
        self._last_waypoint_reject_reason: str = ""
        self._navigation_terminal_count: int = 0
        self._navigation_success_count: int = 0
        self._navigation_failure_count: int = 0
        self._navigation_terminal_goal_states: dict[tuple[float, float], str] = {}
        self._last_navigation_status: dict[str, Any] = {}
        self._robot_pos: tuple[float, float, float] | None = None
        self._robot_yaw: float = 0.0
        self._last_odom_ts: float = 0.0
        self._exploration_grid_data: dict[str, Any] | None = None
        self._started_exploration: bool = False
        self._shutdown = threading.Event()
        self._state_lock = threading.Lock()  # protects cross-thread scalar state
        self._watchdog_thread: threading.Thread | None = None
        self._policy_thread: threading.Thread | None = None
        self._policy = PortableTAREPolicy()
        self._policy_period_s = 1.0 / max(0.1, float(policy_rate_hz))

    # 鈹€鈹€ lifecycle 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def preflight(self) -> str | None:
        """Verify external DDS mode: requires the native C++ kernel with DDS."""
        if self._transport_mode == "in_process":
            return None
        try:
            import lingtu_explore_kernel as _k

            if not getattr(_k, "HAS_DDS", False):
                return (
                    "TAREExplorerModule requires lingtu_explore_kernel compiled "
                    "with DDS support (LINGTU_EXPLORE_CPP_WITH_DDS=ON). "
                    "Use transport_mode='in_process' as a fallback."
                )
        except ImportError:
            return (
                "TAREExplorerModule requires lingtu_explore_kernel for DDS bridge "
                "support. Install the native kernel or use transport_mode='in_process'."
            )
        return None

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.exploration_grid.subscribe(self._on_exploration_grid)
        self.navigation_status.subscribe(self._on_navigation_status)
        if self._transport_mode == "in_process":
            return
        if self._try_cyclonedds():
            return
        logger.warning(
            "TAREExplorerModule: cyclonedds unavailable or failed to initialize; no waypoints will be produced."
        )

    def start(self) -> None:
        super().start()
        self.alive.publish(self._dds_transport is not None or self._transport_mode == "in_process")
        self._shutdown.clear()
        # Watchdog: detect TARE silence and publish health state
        self._watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True, name="tare-watchdog")
        self._watchdog_thread.start()
        if self._dds_transport is not None:
            self._dds_spin_thread = threading.Thread(target=self._dds_spin_loop, daemon=True, name="tare-dds-spin")
            self._dds_spin_thread.start()
        if self._transport_mode == "in_process":
            self._policy_thread = threading.Thread(target=self._policy_loop, daemon=True, name="tare-policy")
            self._policy_thread.start()
        if self._auto_start:
            self._publish_start_signal(True)
            self._started_exploration = True
        self.exploring.publish(self._started_exploration)

    def stop(self) -> None:
        self._shutdown.set()
        if self._watchdog_thread and self._watchdog_thread.is_alive():
            self._watchdog_thread.join(timeout=2.0)
        if self._policy_thread and self._policy_thread.is_alive():
            self._policy_thread.join(timeout=2.0)
        if self._dds_spin_thread and self._dds_spin_thread.is_alive():
            self._dds_spin_thread.join(timeout=2.0)
        # Signal TARE to stop before we tear down
        try:
            self._publish_start_signal(False)
        except Exception:
            pass
        if self._dds_transport is not None:
            try:
                self._dds_transport.cleanup()
            except Exception:
                pass
            self._dds_transport = None
        super().stop()

    # ── DDS transports ──────────────────────────────────────────────────

    def _try_cyclonedds(self) -> bool:
        """Preferred path: delegate DDS to the native C++ TareDdsTransport."""
        try:
            import lingtu_explore_kernel as kernel

            if not getattr(kernel, "HAS_DDS", False):
                logger.debug("TARE: lingtu_explore_kernel compiled without DDS support")
                return False

            domain_id = self._ros_domain_id()
            transport = kernel.TareDdsTransport(domain_id)
            self._dds_transport = transport
            logger.info("TAREExplorerModule: using native C++ DDS domain_id=%s", domain_id)
            return True
        except ImportError:
            return False
        except RuntimeError as exc:
            logger.warning("TAREExplorerModule: C++ DDS init failed: %s", exc)
            return False
        except Exception as exc:
            logger.warning("TAREExplorerModule: DDS transport init failed: %s", exc)
            return False

    @staticmethod
    def _ros_domain_id() -> int:
        raw = os.environ.get("ROS_DOMAIN_ID", "0") or "0"
        try:
            return int(raw)
        except ValueError:
            logger.warning("Invalid ROS_DOMAIN_ID=%r; falling back to domain 0", raw)
            return 0

    def _dds_spin_loop(self) -> None:
        """Periodic DDS take loop: drains readers and dispatches to framework."""
        period_s = 0.1  # 10 Hz spin
        while not self._shutdown.wait(period_s):
            transport = self._dds_transport
            if transport is None:
                break
            try:
                transport.spin_once()
            except Exception as exc:
                logger.debug("TARE DDS spin_once error: %s", exc)
                continue
            self._process_dds_waypoint(transport.last_way_point)
            self._process_dds_path(transport.last_path)
            self._process_dds_runtime(transport.last_runtime)
            self._process_dds_finish(transport.last_finish)

    # ── DDS result processors ──────────────────────────────────────────

    def _process_dds_waypoint(self, wp) -> None:
        if not getattr(wp, "valid", False):
            return
        try:
            frame = getattr(wp, "frame_id", "") or _DEFAULT_GOAL_FRAME_ID
            self._emit_waypoint(float(wp.x), float(wp.y), float(wp.z), frame_id=frame)
        except Exception as e:
            logger.debug("TARE dds waypoint error: %s", e)

    def _process_dds_path(self, path) -> None:
        if not getattr(path, "valid", False):
            return
        try:
            frame = getattr(path, "frame_id", "") or _DEFAULT_GOAL_FRAME_ID
            poses = getattr(path, "poses", None) or []
            pts = [{"x": float(p.x), "y": float(p.y), "z": float(p.z), "frame_id": frame} for p in poses]
            self._path_count += 1
            self._publish_strategy_path_if_ready(pts)
        except Exception as e:
            logger.debug("TARE dds path error: %s", e)

    def _process_dds_runtime(self, rt) -> None:
        if not getattr(rt, "valid", False):
            return
        try:
            with self._state_lock:
                self._last_runtime_ms = float(rt.data)
        except Exception:
            pass

    def _process_dds_finish(self, fin) -> None:
        if not getattr(fin, "valid", False):
            return
        try:
            with self._state_lock:
                self._last_finish = bool(fin.data)
        except Exception:
            pass

    def _strategy_goals_from_path(self, pts: list[dict]) -> list[dict]:
        if not self._prefer_path_strategy or len(pts) < 2:
            return pts
        parsed: list[tuple[float, float, float, str]] = []
        for point in pts:
            try:
                frame_id = str(point.get("frame_id") or _DEFAULT_GOAL_FRAME_ID)
                if frame_id != _DEFAULT_GOAL_FRAME_ID:
                    self._last_strategy_path_reject_reason = "path_frame_not_map"
                    return []
                current = (
                    float(point["x"]),
                    float(point["y"]),
                    float(point.get("z", 0.0)),
                    frame_id,
                )
            except (KeyError, TypeError, ValueError):
                continue
            if not all(math.isfinite(v) for v in current[:3]):
                continue
            parsed.append(current)
        if len(parsed) < 2:
            self._last_strategy_path_reject_reason = "path_too_short"
            return []
        if self._robot_pos is None:
            self._last_strategy_path_reject_reason = "missing_odom"
            return []

        robot = self._robot_pos
        distances = [math.hypot(point[0] - robot[0], point[1] - robot[1]) for point in parsed]
        nearest_index = min(range(len(distances)), key=distances.__getitem__)
        nearest_distance = distances[nearest_index]
        if nearest_distance > self._path_start_tolerance_m:
            self._last_strategy_path_reject_reason = "path_not_near_odom"
            return []

        # TARE local_path is a strategy loop/sequence, not a command path.
        # Re-anchor it at the pose nearest to current odom, then expose a
        # short forward subgoal list for LingTu to re-plan safely point by point.
        ordered = parsed[nearest_index:]
        if len(parsed) > 2 and math.hypot(parsed[0][0] - parsed[-1][0], parsed[0][1] - parsed[-1][1]) <= max(
            self._path_goal_spacing_m, 0.25
        ):
            ordered = parsed[nearest_index:-1] + parsed[:nearest_index]

        min_distance = max(0.0, self._path_goal_min_distance_m)
        min_spacing = max(0.0, self._path_goal_spacing_m)
        selected: list[dict] = []
        last_selected: tuple[float, float, float] | None = None
        for x, y, z, frame_id in ordered:
            current = (x, y, z)
            from_robot = math.hypot(current[0] - robot[0], current[1] - robot[1])
            if from_robot < min_distance:
                continue
            if last_selected is not None:
                spacing = math.hypot(
                    current[0] - last_selected[0],
                    current[1] - last_selected[1],
                )
                if spacing < min_spacing:
                    continue
            selected.append({"x": x, "y": y, "z": z, "frame_id": frame_id})
            last_selected = current
            if len(selected) >= self._path_max_goal_count:
                break
        if len(selected) < 2:
            self._last_strategy_path_reject_reason = "strategy_goals_too_short"
            return []
        self._last_strategy_path_reject_reason = ""
        return selected

    def _publish_strategy_path_if_ready(self, pts: list[dict]) -> None:
        if not self._prefer_path_strategy:
            return
        strategy = self._strategy_goals_from_path(pts)
        if len(strategy) < 2:
            return
        self._strategy_path_count += 1
        self._last_strategy_path_ts = _time.time()
        self._last_strategy_goal_count = len(strategy)
        candidates = [(float(point["x"]), float(point["y"])) for point in strategy]
        if candidates != self._last_goal_candidates:
            self._navigation_terminal_goal_states.clear()
        self._last_goal_candidates = candidates
        self.exploration_path.publish(strategy)

    def _on_odom(self, odom: Odometry) -> None:
        try:
            pos = odom.pose.position
            xyz = (float(pos.x), float(pos.y), float(getattr(pos, "z", 0.0)))
            yaw = float(getattr(odom, "yaw", 0.0))
        except Exception:
            return
        if not all(math.isfinite(value) for value in xyz):
            return
        self._robot_pos = xyz
        self._robot_yaw = yaw if math.isfinite(yaw) else 0.0
        self._last_odom_ts = _time.time()

    def _on_exploration_grid(self, data: dict) -> None:
        if isinstance(data, dict):
            self._exploration_grid_data = data

    def _policy_loop(self) -> None:
        while not self._shutdown.wait(self._policy_period_s):
            self._run_policy_once()

    def _run_policy_once(self) -> bool:
        if not self._started_exploration or self._robot_pos is None:
            return False
        if self._hold_active_goal_until_terminal and self._active_goal_xy is not None:
            return False
        if self._exploration_grid_data is None:
            self._last_waypoint_reject_reason = "missing_exploration_grid"
            return False
        started = _time.time()
        visited = [(float(x), float(y)) for x, y in self._navigation_terminal_goal_states.keys()]
        try:
            decision = self._policy.select(
                grid_payload=self._exploration_grid_data,
                robot_xy=(self._robot_pos[0], self._robot_pos[1]),
                robot_yaw=self._robot_yaw,
                visited_goals=visited,
            )
        except Exception as e:
            # Kernel not built or backend unavailable: log once and bail out
            # without crashing the policy loop. The module stays in
            # ``configured`` state so tests/DDS paths remain usable.
            self._last_waypoint_reject_reason = f"explore_kernel_error: {type(e).__name__}: {e}"
            logger.warning("TARE policy select failed: %s", e)
            return False
        with self._state_lock:
            self._last_runtime_ms = (_time.time() - started) * 1000.0
            self._last_finish = bool(decision.done)
        if decision.goal is None:
            self._last_waypoint_reject_reason = decision.reason
            return False
        if len(decision.path) >= 2:
            self.exploration_path.publish(
                [
                    {
                        "x": float(x),
                        "y": float(y),
                        "z": float(z),
                        "frame_id": self._goal_frame_id
                        or str(self._exploration_grid_data.get("frame_id") or _DEFAULT_GOAL_FRAME_ID),
                    }
                    for x, y, z in decision.path
                ]
            )
        self._emit_waypoint(
            decision.goal[0],
            decision.goal[1],
            decision.goal[2],
            frame_id=str(self._exploration_grid_data.get("frame_id") or _DEFAULT_GOAL_FRAME_ID),
        )
        return True

    def _on_navigation_status(self, status: dict) -> None:
        if not isinstance(status, dict):
            return
        state = str(status.get("state") or "").upper()
        if state not in {"SUCCESS", "FAILED", "STUCK", "CANCELLED"}:
            return
        goal_xy = self._status_goal_xy(status)
        if goal_xy is None:
            if self._hold_active_goal_until_terminal:
                self._active_goal_xy = None
            return
        if not self._matches_tare_goal(goal_xy):
            return
        goal_key = (round(float(goal_xy[0]), 2), round(float(goal_xy[1]), 2))
        previous_state = self._navigation_terminal_goal_states.get(goal_key)
        if previous_state == state or previous_state == "SUCCESS":
            return
        if previous_state and state != "SUCCESS":
            return
        self._navigation_terminal_goal_states[goal_key] = state
        self._active_goal_xy = None

        if previous_state and state == "SUCCESS":
            self._navigation_success_count += 1
            self._navigation_failure_count = max(0, self._navigation_failure_count - 1)
        else:
            self._navigation_terminal_count += 1
            if state == "SUCCESS":
                self._navigation_success_count += 1
            else:
                self._navigation_failure_count += 1
        self._last_navigation_status = {
            "state": state,
            "goal": [float(goal_xy[0]), float(goal_xy[1])],
            "failure_reason": str(status.get("failure_reason") or status.get("reason") or ""),
            "ts": float(status.get("ts") or _time.time()),
        }

    @staticmethod
    def _status_goal_xy(status: dict) -> tuple[float, float] | None:
        goal = status.get("goal")
        if isinstance(goal, dict):
            raw = (goal.get("x"), goal.get("y"))
        elif isinstance(goal, (list, tuple)) and len(goal) >= 2:
            raw = (goal[0], goal[1])
        else:
            return None
        try:
            xy = (float(raw[0]), float(raw[1]))
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in xy):
            return None
        return xy

    def _matches_tare_goal(self, goal_xy: tuple[float, float]) -> bool:
        tol = self._navigation_goal_match_tolerance_m
        return any(
            math.hypot(float(goal_xy[0]) - x, float(goal_xy[1]) - y) <= tol for x, y in self._last_goal_candidates
        )

    # 鈹€鈹€ Emit 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def _emit_waypoint(self, x: float, y: float, z: float, frame_id: str = _DEFAULT_GOAL_FRAME_ID) -> None:
        """Convert a TARE PointStamped to LingTu PoseStamped and publish."""
        with self._state_lock:
            self._last_waypoint_ts = _time.time()
            self._waypoint_count += 1
        if self._prefer_path_strategy:
            strategy_age = self._last_waypoint_ts - self._last_strategy_path_ts
            if 0.0 <= strategy_age <= self._path_strategy_timeout_s:
                return
            if not self._path_strategy_fallback_to_waypoint:
                return
        candidates = [(float(x), float(y))]
        if self._should_reject_far_waypoint(candidates[0]):
            return
        if self._hold_active_goal_until_terminal and self._active_goal_xy is not None:
            self._suppressed_waypoint_count += 1
            return
        if candidates != self._last_goal_candidates:
            self._navigation_terminal_goal_states.clear()
        self._last_goal_candidates = candidates
        self._active_goal_xy = candidates[0]
        output_frame_id = self._goal_frame_id or frame_id
        pose = self._build_pose_stamped(x, y, z, frame_id=output_frame_id)
        self.exploration_goal.publish(pose)

    def _should_reject_far_waypoint(self, goal_xy: tuple[float, float]) -> bool:
        if self._max_waypoint_distance_m <= 0.0:
            self._last_waypoint_reject_reason = ""
            return False
        now = _time.time()
        if self._robot_pos is None or (
            self._waypoint_odometry_timeout_s > 0.0 and now - self._last_odom_ts > self._waypoint_odometry_timeout_s
        ):
            self._suppressed_far_waypoint_count += 1
            self._last_waypoint_reject_reason = "no_fresh_odometry"
            return True
        distance_m = math.hypot(
            float(goal_xy[0]) - float(self._robot_pos[0]),
            float(goal_xy[1]) - float(self._robot_pos[1]),
        )
        if distance_m > self._max_waypoint_distance_m:
            self._suppressed_far_waypoint_count += 1
            self._last_waypoint_reject_reason = (
                f"waypoint_distance {distance_m:.2f}m exceeds {self._max_waypoint_distance_m:.2f}m"
            )
            return True
        self._last_waypoint_reject_reason = ""
        return False

    def _publish_start_signal(self, enable: bool) -> None:
        """Send the bool start/stop signal to the TARE node via C++ DDS."""
        if self._dds_transport is not None:
            try:
                self._dds_transport.publish_start(bool(enable))
            except Exception as e:
                logger.debug("TARE start signal publish failed: %s", e)

    # 鈹€鈹€ Watchdog / diagnostics 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def _watchdog_loop(self) -> None:
        while not self._shutdown.wait(1.0):
            self._publish_stats()

    def _publish_stats(self) -> None:
        now = _time.time()
        with self._state_lock:
            last_wp_ts = self._last_waypoint_ts
            last_rt_ms = self._last_runtime_ms
            last_finish = self._last_finish
            wp_count = self._waypoint_count
            pth_count = self._path_count
        wp_age = now - last_wp_ts if last_wp_ts > 0 else float("inf")
        alive = self._dds_transport is not None or self._transport_mode == "in_process"
        healthy = wp_age < self._way_point_timeout_s and alive
        self.tare_stats.publish(
            {
                **self._backend_status.as_health_fields(),
                "alive": alive,
                "transport_mode": self._transport_mode,
                "started": self._started_exploration,
                "healthy": healthy,
                "waypoint_count": wp_count,
                "path_count": pth_count,
                "strategy_path_count": self._strategy_path_count,
                "last_strategy_goal_count": self._last_strategy_goal_count,
                "last_strategy_path_age_s": (
                    now - self._last_strategy_path_ts if self._last_strategy_path_ts > 0 else float("inf")
                ),
                "last_strategy_path_reject_reason": (self._last_strategy_path_reject_reason),
                "prefer_path_strategy": self._prefer_path_strategy,
                "hold_active_goal_until_terminal": self._hold_active_goal_until_terminal,
                "active_goal_xy": list(self._active_goal_xy) if self._active_goal_xy else [],
                "suppressed_waypoint_count": self._suppressed_waypoint_count,
                "suppressed_far_waypoint_count": self._suppressed_far_waypoint_count,
                "last_waypoint_reject_reason": self._last_waypoint_reject_reason,
                "waypoint_age_s": wp_age,
                "last_runtime_ms": last_rt_ms,
                "finished": last_finish,
                "navigation_terminal_count": self._navigation_terminal_count,
                "navigation_success_count": self._navigation_success_count,
                "navigation_failure_count": self._navigation_failure_count,
                "last_navigation_status": self._last_navigation_status,
            }
        )

    # 鈹€鈹€ Skills 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    @skill
    def start_tare_exploration(self) -> str:
        """Send the start signal to the TARE planner. Use this to resume
        exploration after calling ``stop_tare_exploration``. Re-issues are
        idempotent."""
        import json

        self._publish_start_signal(True)
        self._started_exploration = True
        self.exploring.publish(True)
        return json.dumps({"status": "started"})

    @skill
    def stop_tare_exploration(self) -> str:
        """Pause TARE exploration. The planner keeps running but stops
        emitting new waypoints until ``start_tare_exploration`` is called."""
        import json

        self._publish_start_signal(False)
        self._started_exploration = False
        self.exploring.publish(False)
        return json.dumps({"status": "stopped"})

    @skill
    def get_tare_status(self) -> str:
        """Return TARE exploration state: waypoint count, last waypoint age,
        per-cycle runtime, and whether exploration is marked finished."""
        import json

        now = _time.time()
        with self._state_lock:
            last_wp_ts = self._last_waypoint_ts
            last_rt_ms = self._last_runtime_ms
            last_finish = self._last_finish
            wp_count = self._waypoint_count
            pth_count = self._path_count
        return json.dumps(
            {
                **self._backend_status.as_health_fields(),
                "alive": self._dds_transport is not None or self._transport_mode == "in_process",
                "transport_mode": self._transport_mode,
                "started": self._started_exploration,
                "waypoint_count": wp_count,
                "path_count": pth_count,
                "prefer_path_strategy": self._prefer_path_strategy,
                "hold_active_goal_until_terminal": self._hold_active_goal_until_terminal,
                "active_goal_xy": list(self._active_goal_xy) if self._active_goal_xy else [],
                "suppressed_waypoint_count": self._suppressed_waypoint_count,
                "suppressed_far_waypoint_count": self._suppressed_far_waypoint_count,
                "last_waypoint_reject_reason": self._last_waypoint_reject_reason,
                "waypoint_age_s": (now - last_wp_ts if last_wp_ts > 0 else None),
                "last_runtime_ms": last_rt_ms,
                "finished": last_finish,
                "navigation_terminal_count": self._navigation_terminal_count,
                "navigation_success_count": self._navigation_success_count,
                "navigation_failure_count": self._navigation_failure_count,
                "last_navigation_status": self._last_navigation_status,
            }
        )
