"""PathFollowerModule - path tracking as a pluggable Module.

Takes local path + odometry, produces cmd_vel to follow the path.

Backends:
  "nav_core"     - C++ compute_control via nanobind (Pure Pursuit, adaptive lookahead,
                   two-way drive, omni-dir near-goal, acceleration limiting) [DEFAULT]
  "pure_pursuit" - C++ path_follower (NativeModule, Pure Pursuit)
  "pid"          - Python PID controller (testing/simple fallback)

Usage::

    bp.add(PathFollowerModule, backend="nav_core")
"""

from __future__ import annotations

import logging
import math
import time
from typing import Any

from base_autonomy.modules._nav_core_loader import nav_core_build_hint, try_import_nav_core
from core.backend_status import BackendStatus, require_backend
from core.msgs.numpy_compat import np
from core.module import Module
from core.msgs.geometry import Twist, Vector3
from core.msgs.nav import Odometry, Path
from core.registry import register
from core.runtime_interface import (
    map_frame_id,
    normalize_frame_id,
    runtime_fixed_path_frame_ids,
)
from core.stream import In, Out

logger = logging.getLogger(__name__)


_AVAILABLE_PATH_FOLLOWER_BACKENDS = ("nav_core", "pure_pursuit", "pid")


@register("path_follower", "pid",
          description="Python adaptive pure-pursuit fallback")
@register("path_follower", "pure_pursuit",
          description="C++ path follower via NativeModule subprocess")
@register("path_follower", "nav_core",
          description="C++ compute_control via nanobind (Pure Pursuit + adaptive lookahead)")
class PathFollowerModule(Module, layer=2):
    """Path following - tracks local path and outputs cmd_vel.

    "nav_core"     backend: C++ compute_control via nanobind [default]
    "pure_pursuit" backend: C++ path_follower NativeModule
    "pid"          backend: Python PID controller (testing/fallback)
    """

    # -- Inputs --
    odometry: In[Odometry]
    local_path: In[Path]
    control_hint: In[dict]
    map_frame_jump_event: In[dict]

    # -- Outputs --
    cmd_vel: Out[Twist]
    alive: Out[bool]

    def __init__(self, backend: str = "nav_core",
                 max_speed: float = 0.4, lookahead: float = 1.5,
                 goal_tolerance: float = 0.2,
                 min_speed: float = 0.15,
                 max_yaw_rate: float | None = None,
                 turn_speed_yaw_rate_start: float = 0.0,
                 turn_speed_min_scale: float = 1.0,
                 yaw_rate_gain: float = 7.5,
                 stop_yaw_rate_gain: float = 7.5,
                 dir_diff_thre: float = 0.1,
                 **kw):
        super().__init__(**kw)
        require_backend("path_follower", backend, _AVAILABLE_PATH_FOLLOWER_BACKENDS)
        self._backend_status = BackendStatus.configured_as(backend)
        self._backend = backend
        self._max_speed = max_speed
        self._lookahead = lookahead
        self._goal_tolerance = goal_tolerance
        self._min_speed = min_speed
        self._max_yaw_rate = max_yaw_rate
        self._turn_speed_yaw_rate_start = max(0.0, float(turn_speed_yaw_rate_start or 0.0))
        self._turn_speed_min_scale = max(0.0, min(1.0, float(turn_speed_min_scale)))
        self._yaw_rate_gain = max(0.0, float(yaw_rate_gain))
        self._stop_yaw_rate_gain = max(0.0, float(stop_yaw_rate_gain))
        self._dir_diff_thre = max(0.0, float(dir_diff_thre))
        self._two_way_drive = bool(kw.get("two_way_drive", True))
        self._node = None

        # Current robot pose (odom frame)
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0

        # Path state for pid backend
        self._path_points = None

        # Output smoothing (shared across backends)
        self._smooth_vx = 0.0
        self._smooth_wz = 0.0

        # nav_core backend state
        self._nc = None           # _nav_core module reference
        self._nc_params = None    # PathFollowerParams
        self._nc_state = None     # PathFollowerState (persistent across calls)
        self._nc_path: list = []  # list of _nav_core.Vec3 (path reference frame)

        # Recorded pose at path receipt time (for vehicleRel transform)
        self._x_rec = 0.0
        self._y_rec = 0.0
        self._yaw_rec = 0.0
        self._cos_yaw_rec = 1.0
        self._sin_yaw_rec = 0.0
        self._odom_frame_id = map_frame_id()
        self._last_odom_ts = 0.0
        self._control_hint_timeout = float(kw.get("control_hint_timeout", 0.75))
        self._control_hint_ts = 0.0
        self._control_slow_down = 0
        self._control_safety_stop = False
        self._control_hint_reason = ""

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.local_path.subscribe(self._on_path)
        self.control_hint.subscribe(self._on_control_hint)
        self.map_frame_jump_event.subscribe(self._on_map_frame_jump)

        if self._backend == "nav_core":
            self._setup_nav_core()
        elif self._backend == "pure_pursuit":
            self._setup_native()
        else:
            self._setup_pid()

    # -- pid backend setup (W2-4 adaptive Pure Pursuit) --

    def _setup_pid(self) -> None:
        """W2-4: configure adaptive Pure Pursuit for the pid fallback backend.

        Parameters (config-overridable via robot_config.yaml path_follower):
          k_v   - variable-lookahead slope (m per m/s of speed)
          L_min - minimum lookahead distance (m)
          L_max - maximum lookahead distance (m)
          a_max - max acceleration for v_cmd ramp (m/s^2)
          v_max - clamp ceiling for v_cmd (m/s), defaults to self._max_speed
        """
        self._pp_k_v: float = 0.5
        self._pp_l_min: float = 0.5
        self._pp_l_max: float = 2.0
        self._pp_a_max: float = 1.0
        self._pp_v_max: float = float(self._max_speed)

        try:
            from core.config import get_config
            cfg = get_config()
            pf = cfg.raw.get("path_follower", {}) if hasattr(cfg, "raw") else {}
            if pf:
                self._pp_k_v = float(pf.get("k_v", self._pp_k_v))
                self._pp_l_min = float(pf.get("L_min", self._pp_l_min))
                self._pp_l_max = float(pf.get("L_max", self._pp_l_max))
                self._pp_a_max = float(pf.get("a_max", self._pp_a_max))
                self._pp_v_max = float(pf.get("v_max", self._pp_v_max))
                logger.info(
                    "PathFollowerModule [pid]: loaded adaptive PP params "
                    "(k_v=%.2f, L=[%.2f,%.2f], a_max=%.2f, v_max=%.2f)",
                    self._pp_k_v, self._pp_l_min, self._pp_l_max,
                    self._pp_a_max, self._pp_v_max,
                )
        except (ImportError, AttributeError, KeyError):
            logger.info(
                "PathFollowerModule [pid]: using default adaptive PP params "
                "(k_v=0.5, L=[0.5,2.0], a_max=1.0) - override via robot_config.yaml"
            )

        # Previous-tick velocity for acceleration ramp
        self._pp_v_prev: float = 0.0

    # -- nav_core backend setup --

    def _setup_nav_core(self):
        """Import _nav_core and create PathFollowerParams/State."""
        _nav_core = try_import_nav_core(("PathFollowerParams", "PathFollowerState", "compute_control"))
        if _nav_core is None:
            logger.info(
                "PathFollowerModule: _nav_core.so not found - using pid backend.\n"
                "  To enable C++ path follower:\n  %s", nav_core_build_hint()
            )
            self._backend_status.use("pid", reason="compatible _nav_core missing")
            self._backend = "pid"
            self._nc = None
            return
        try:
            self._nc = _nav_core

            params = _nav_core.PathFollowerParams()
            params.max_speed = self._max_speed
            params.stop_dis_thre = self._goal_tolerance
            # compute_control compares the selected lookahead point distance
            # against stop_dis_thre. Dense local-planner paths can otherwise
            # select a near-start point inside the stop band and never ramp up.
            min_lookahead = max(0.2, params.stop_dis_thre + 0.05)
            params.base_look_ahead_dis = max(self._lookahead * 0.2, min_lookahead)
            params.min_look_ahead_dis = min_lookahead
            params.max_look_ahead_dis = max(min(self._lookahead, 2.0), min_lookahead)
            params.look_ahead_ratio = 0.5
            params.yaw_rate_gain = self._yaw_rate_gain
            params.stop_yaw_rate_gain = self._stop_yaw_rate_gain
            params.max_yaw_rate = (
                math.degrees(float(self._max_yaw_rate))
                if self._max_yaw_rate is not None
                else 45.0
            )
            params.max_accel = 1.0
            if hasattr(params, "turn_speed_yaw_rate_start"):
                params.turn_speed_yaw_rate_start = self._turn_speed_yaw_rate_start
            if hasattr(params, "turn_speed_min_scale"):
                params.turn_speed_min_scale = self._turn_speed_min_scale
            params.switch_time_thre = 1.0
            params.dir_diff_thre = self._dir_diff_thre
            params.omni_dir_goal_thre = 1.0
            params.omni_dir_diff_thre = 1.5
            params.slow_dwn_dis_thre = 1.0
            params.two_way_drive = self._two_way_drive
            params.no_rot_at_goal = True

            self._nc_params = params
            self._nc_state = _nav_core.PathFollowerState()

            logger.info("PathFollowerModule [nav_core]: C++ compute_control loaded")
        except Exception as e:
            logger.warning("PathFollowerModule: _nav_core error: %s - using pid backend", e)
            self._backend_status.use("pid", reason=f"nav_core init failed: {e}")
            self._backend = "pid"
            self._nc = None

    # -- NativeModule (pure_pursuit) backend setup --

    def _setup_native(self):
        try:
            from base_autonomy.native_factories import path_follower
            from core.config import get_config

            cfg = get_config()
            self._node = path_follower(cfg)
            try:
                self._node.setup()
            except (FileNotFoundError, PermissionError) as e:
                logger.warning("PathFollowerModule: setup failed: %s", e)
                self._backend_status.mark_degraded(f"pure_pursuit setup failed: {e}")
                self._node = None
        except ImportError as e:
            logger.warning("PathFollowerModule: C++ backend not available: %s", e)
            self._backend_status.mark_degraded(f"pure_pursuit backend not available: {e}")

    # -- Module lifecycle --

    def start(self):
        super().start()
        if self._node:
            try:
                self._node.start()
                logger.info("PathFollowerModule [pure_pursuit]: C++ node started")
            except Exception as e:
                logger.error("PathFollowerModule: start failed: %s", e)
        self.alive.publish(True)

    def stop(self):
        if self._node:
            try:
                self._node.stop()
            except Exception as e:
                logger.warning("PathFollowerModule stop: %s", e)
            self._node = None
        self.alive.publish(False)
        super().stop()

    # -- Callbacks --

    def _reset_nav_core_state(self) -> None:
        if self._backend != "nav_core" or self._nc is None:
            return
        try:
            self._nc_state = self._nc.PathFollowerState()
            return
        except (AttributeError, TypeError):
            pass
        state = self._nc_state
        if state is None:
            return
        for attr in ("vehicle_speed", "vehicleSpeed", "nav_fwd", "pathPointID", "path_point_id"):
            if hasattr(state, attr):
                try:
                    setattr(state, attr, 0)
                except (AttributeError, TypeError):
                    pass

    def _publish_zero(self, *, reset_nav_core_state: bool = False) -> None:
        self._smooth_vx = 0.0
        self._smooth_wz = 0.0
        if hasattr(self, "_pp_v_prev"):
            self._pp_v_prev = 0.0
        if reset_nav_core_state:
            self._reset_nav_core_state()
        self.cmd_vel.publish(Twist())

    def _reset_path_tracking(self, *, reset_nav_core_state: bool = True) -> None:
        self._nc_path = []
        self._path_points = None
        self._x_rec = self._robot_x
        self._y_rec = self._robot_y
        self._yaw_rec = self._robot_yaw
        self._cos_yaw_rec = math.cos(self._yaw_rec)
        self._sin_yaw_rec = math.sin(self._yaw_rec)
        self._publish_zero(reset_nav_core_state=reset_nav_core_state)

    def _on_control_hint(self, hint: dict) -> None:
        if not isinstance(hint, dict):
            return
        try:
            slow_down = int(hint.get("slow_down", 0) or 0)
        except (TypeError, ValueError):
            slow_down = 0
        self._control_slow_down = max(0, min(3, slow_down))
        if "safety_stop" in hint:
            self._control_safety_stop = bool(hint.get("safety_stop"))
        else:
            self._control_safety_stop = bool(hint.get("near_field_stop"))
        self._control_hint_reason = str(hint.get("reason") or "")
        self._control_hint_ts = float(hint.get("ts") or time.time())
        if self._control_safety_stop:
            self._publish_zero(reset_nav_core_state=False)

    def _active_control_guard(self) -> tuple[float, bool]:
        if self._control_hint_ts <= 0:
            return 1.0, False
        if time.time() - self._control_hint_ts > self._control_hint_timeout:
            self._control_slow_down = 0
            self._control_safety_stop = False
            self._control_hint_reason = ""
            return 1.0, False
        slow_factor_by_level = {
            0: 1.0,
            1: 0.65,
            2: 0.40,
            3: 0.20,
        }
        return slow_factor_by_level.get(self._control_slow_down, 1.0), self._control_safety_stop

    def _turn_speed_scale(self, yaw_rate_abs: float, yaw_limit: float) -> float:
        if (
            self._turn_speed_yaw_rate_start <= 0.0
            or self._turn_speed_min_scale >= 1.0
            or yaw_limit <= self._turn_speed_yaw_rate_start
        ):
            return 1.0
        ratio = (
            (max(0.0, float(yaw_rate_abs)) - self._turn_speed_yaw_rate_start)
            / (yaw_limit - self._turn_speed_yaw_rate_start)
        )
        ratio = max(0.0, min(1.0, ratio))
        return 1.0 - (1.0 - self._turn_speed_min_scale) * ratio

    def _on_map_frame_jump(self, event: dict) -> None:
        if isinstance(event, dict):
            self._reset_path_tracking(reset_nav_core_state=True)

    def _on_odom(self, odom: Odometry):
        _prev_x, _prev_y = self._robot_x, self._robot_y
        self._robot_x = odom.pose.position.x
        self._robot_y = odom.pose.position.y
        if getattr(odom, "frame_id", None):
            self._odom_frame_id = str(odom.frame_id)
        self._last_odom_ts = float(getattr(odom, "ts", 0.0) or time.time())

        # Extract yaw from quaternion (reliable, works at all speeds)
        self._robot_yaw = odom.yaw

        if self._backend == "nav_core" and self._nc_path:
            self._nav_core_step(odom.ts)
        elif self._backend == "pid" and self._path_points is not None:
            # Guard against synchronous cmd_vel->odom->pid_step recursion
            # in callback transport (no issue with DDS/shm transport).
            if not getattr(self, '_in_pid', False):
                self._in_pid = True
                self._pid_step()
                self._in_pid = False

    def _on_path(self, path: Path):
        if self._backend == "nav_core":
            # Record robot pose at path receipt - defines the reference frame
            frame_id = normalize_frame_id(getattr(path, "frame_id", None))
            fixed_frame_path = frame_id in runtime_fixed_path_frame_ids(
                self._odom_frame_id,
            )
            if fixed_frame_path:
                self._x_rec = 0.0
                self._y_rec = 0.0
                self._yaw_rec = 0.0
            else:
                # Legacy local/body-frame paths use robot pose at receipt as
                # the reference frame, matching the original pathFollower API.
                self._x_rec = self._robot_x
                self._y_rec = self._robot_y
                self._yaw_rec = self._robot_yaw
            self._cos_yaw_rec = math.cos(self._yaw_rec)
            self._sin_yaw_rec = math.sin(self._yaw_rec)

            # nav_core expects path points and vehicleRel in the same frame.
            nc = self._nc
            pts = []
            for ps in path.poses:
                x_rel, y_rel = self._to_path_reference_frame(
                    ps.pose.position.x,
                    ps.pose.position.y,
                )
                v = nc.Vec3(
                    x_rel,
                    y_rel,
                    ps.pose.position.z,
                )
                pts.append(v)
            if len(pts) < 2:
                self._reset_path_tracking(reset_nav_core_state=True)
                return
            self._reset_nav_core_state()
            self._nc_path = pts
            if self._last_odom_ts > 0:
                self._nav_core_step(self._last_odom_ts)
            # Each fresh local path has its own waypoint index and direction.
            # Reusing the previous nav_core state can keep the follower pointed
            # at a stale path point after a replan or patrol checkpoint switch.

        elif self._backend == "pid":
            pts = []
            for ps in path.poses:
                pts.append([ps.pose.position.x, ps.pose.position.y])
            if len(pts) < 2:
                self._reset_path_tracking(reset_nav_core_state=False)
                return
            self._path_points = np.array(pts)
            # Kick off the cmd_vel -> odom loop immediately
            if self._path_points is not None:
                self._pid_step()

    # -- nav_core control step --

    def _to_path_reference_frame(self, x: float, y: float) -> tuple[float, float]:
        dx = x - self._x_rec
        dy = y - self._y_rec
        return (
            self._cos_yaw_rec * dx + self._sin_yaw_rec * dy,
            -self._sin_yaw_rec * dx + self._cos_yaw_rec * dy,
        )

    def _nav_core_step(self, current_time: float):
        """Compute cmd_vel using C++ compute_control."""
        if not self._nc_path:
            self._publish_zero()
            return

        nc = self._nc

        # Transform robot position into path reference frame
        # (rotation by -yaw_rec, then translate by -(x_rec, y_rec))
        vehicle_x_rel, vehicle_y_rel = self._to_path_reference_frame(
            self._robot_x,
            self._robot_y,
        )
        vehicle_rel = nc.Vec3(vehicle_x_rel, vehicle_y_rel, 0.0)

        # Yaw change since path was received
        vehicle_yaw_diff = self._robot_yaw - self._yaw_rec

        # joy_speed normalized [0, 1] - use 1.0 (max_speed already in params)
        joy_speed = 1.0
        slow_factor, safety_stop_active = self._active_control_guard()
        safety_stop = 1 if safety_stop_active else 0
        if safety_stop_active:
            self._publish_zero(reset_nav_core_state=False)
            return

        out = nc.compute_control(
            vehicle_rel,
            vehicle_yaw_diff,
            self._nc_path,
            joy_speed,
            current_time,
            slow_factor,
            safety_stop,
            self._nc_params,
            self._nc_state,
        )

        # Apply exponential smoothing (alpha=0.3, same as pid backend)
        alpha = 0.3
        self._smooth_vx = (1 - alpha) * self._smooth_vx + alpha * out.cmd.vx
        self._smooth_wz = (1 - alpha) * self._smooth_wz + alpha * out.cmd.wz

        self.cmd_vel.publish(Twist(
            linear=Vector3(self._smooth_vx, out.cmd.vy, 0.0),
            angular=Vector3(0.0, 0.0, self._smooth_wz),
        ))

    # -- Python PID (fallback) --

    def _pid_step(self):
        """Simple Pure Pursuit in Python for testing."""
        if self._path_points is None or len(self._path_points) < 2:
            self._publish_zero()
            return
        slow_factor, safety_stop_active = self._active_control_guard()
        if safety_stop_active:
            self._publish_zero(reset_nav_core_state=False)
            return

        robot = np.array([self._robot_x, self._robot_y])
        dists = np.linalg.norm(self._path_points - robot, axis=1)
        # Find lookahead point: search forward from the closest path point so
        # far-away points *behind* the robot (e.g. near the path start once the
        # robot approaches the end) are never selected as the target. Without
        # this, the follower targets early path points forever and never
        # reaches the goal-tolerance stop branch below.
        nearest = int(np.argmin(dists))
        beyond = np.where(dists[nearest:] > self._lookahead)[0]
        if len(beyond) == 0:
            target = self._path_points[-1]
        else:
            target = self._path_points[nearest + beyond[0]]

        dx = target[0] - self._robot_x
        dy = target[1] - self._robot_y
        dist = math.hypot(dx, dy)

        if dist < self._goal_tolerance:
            self._smooth_vx *= 0.5  # decay to zero
            self._smooth_wz *= 0.5
            if abs(self._smooth_vx) < 0.01:
                self._smooth_vx = 0.0
                self._smooth_wz = 0.0
            self.cmd_vel.publish(Twist(
                linear=Vector3(self._smooth_vx, 0.0, 0.0),
                angular=Vector3(0.0, 0.0, self._smooth_wz),
            ))
            return

        desired_yaw = math.atan2(dy, dx)
        yaw_err = desired_yaw - self._robot_yaw
        # Normalize to [-pi, pi]
        while yaw_err > math.pi:
            yaw_err -= 2 * math.pi
        while yaw_err < -math.pi:
            yaw_err += 2 * math.pi

        # P controller with cos coupling - Go1 has low yaw drift (about 2 deg/8s)
        yaw_limit = float(self._max_yaw_rate) if self._max_yaw_rate is not None else 0.8
        wz = max(-yaw_limit, min(yaw_limit, yaw_err * 0.5))
        turn_factor = max(0.2, math.cos(yaw_err))
        vx = min(self._max_speed, max(self._min_speed, dist * 0.25)) * turn_factor
        vx *= slow_factor
        vx *= self._turn_speed_scale(abs(wz), yaw_limit)

        alpha = 0.2
        self._smooth_vx = (1 - alpha) * self._smooth_vx + alpha * vx
        self._smooth_wz = (1 - alpha) * self._smooth_wz + alpha * wz

        self.cmd_vel.publish(Twist(
            linear=Vector3(self._smooth_vx, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, self._smooth_wz),
        ))

    # -- Health --

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        h = {
            **self._backend_status.as_health_fields(),
            "has_path": bool(self._nc_path) if self._backend == "nav_core"
                        else self._path_points is not None,
            "max_yaw_rate": self._max_yaw_rate,
            "turn_speed_yaw_rate_start": self._turn_speed_yaw_rate_start,
            "turn_speed_min_scale": self._turn_speed_min_scale,
            "yaw_rate_gain": self._yaw_rate_gain,
            "stop_yaw_rate_gain": self._stop_yaw_rate_gain,
            "dir_diff_thre": self._dir_diff_thre,
            "control_hint": {
                "slow_down": self._control_slow_down,
                "safety_stop": self._control_safety_stop,
                "reason": self._control_hint_reason,
                "age_ms": (
                    round((time.time() - self._control_hint_ts) * 1000)
                    if self._control_hint_ts > 0
                    else None
                ),
            },
        }
        if self._backend == "nav_core" and self._nc_state is not None:
            h["vehicle_speed"] = self._nc_state.vehicle_speed
            h["nav_fwd"] = self._nc_state.nav_fwd
        info["path_follower"] = h
        return info
