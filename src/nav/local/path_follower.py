"""PathFollower - path tracking as a pluggable Module.

Takes local path + odometry, produces cmd_vel to follow the path.

Backends:
  "nav_kernel"     - C++ compute_control via nanobind (Pure Pursuit, adaptive lookahead,
                   two-way drive, omni-dir near-goal, acceleration limiting) [DEFAULT]
  "pid"          - Python PID controller (testing/simple fallback)

Usage::

    bp.add(PathFollower, backend="nav_kernel")
"""

from __future__ import annotations

import logging
import math
import time
from typing import Any

from nav.local.contracts import (
    PATH_FOLLOWER_BACKENDS,
    require_path_follower_backend,
)
from nav.local.path_follower_runtime import (
    PathFollowerTuning,
    PathFollowerRuntime,
    setup_path_follower_runtime,
)
from nav.services.frame_transforms import (
    is_map_frame_jump_event,
    transform_xyz_yaw_with_map_odom,
)
from runtime.backend_status import BackendStatus
from runtime.module import Module
from runtime.msgs.geometry import Twist, Vector3
from runtime.msgs.nav import Odometry, Path
from runtime.msgs.numpy_compat import np
from runtime.registry import register
from runtime.runtime_interface import (
    map_frame_id,
    normalize_frame_id,
    runtime_fixed_path_frame_ids,
)
from runtime.stream import In, Out

logger = logging.getLogger(__name__)
_AVAILABLE_PATH_FOLLOWER_BACKENDS = PATH_FOLLOWER_BACKENDS


@register("path_follower", "pid",
          description="Python adaptive pure-pursuit fallback")
@register("path_follower", "nav_kernel",
          description="C++ compute_control via nanobind (Pure Pursuit + adaptive lookahead)")
class PathFollower(Module, layer=2):
    """Path following - tracks local path and outputs cmd_vel.

    "nav_kernel"     backend: C++ compute_control via nanobind [default]
    "pid"          backend: Python PID controller (testing/fallback)
    """

    runtime_id = "nav.path_follower"

    # -- Inputs --
    odometry: In[Odometry]
    local_path: In[Path]
    control_hint: In[dict]
    map_odom_tf: In[dict]
    map_frame_jump_event: In[dict]

    # -- Outputs --
    cmd_vel: Out[Twist]
    alive: Out[bool]

    def __init__(self, backend: str = "nav_kernel",
                 max_speed: float = 0.4, lookahead: float = 1.5,
                 goal_tolerance: float = 0.2,
                 min_speed: float = 0.15,
                 max_yaw_rate: float | None = None,
                 turn_speed_yaw_rate_start: float = 0.0,
                 turn_speed_min_scale: float = 1.0,
                 native_max_accel: float = 1.0,
                 yaw_rate_gain: float = 7.5,
                 stop_yaw_rate_gain: float = 7.5,
                 dir_diff_thre: float = 0.1,
                 use_incl_rate_to_slow: bool = False,
                 incl_rate_thre: float = 120.0,
                 slow_rate_1: float = 0.25,
                 slow_rate_2: float = 0.5,
                 slow_rate_3: float = 0.75,
                 slow_time_1: float = 2.0,
                 slow_time_2: float = 2.0,
                 use_incl_to_stop: bool = False,
                 incl_thre: float = 45.0,
                 stop_time: float = 5.0,
                 **kw):
        super().__init__(**kw)
        require_path_follower_backend(backend)
        self._backend_status = BackendStatus.configured_as(backend)
        self._backend = backend
        self._max_speed = max_speed
        self._lookahead = lookahead
        self._goal_tolerance = goal_tolerance
        self._min_speed = min_speed
        self._max_yaw_rate = max_yaw_rate
        self._turn_speed_yaw_rate_start = max(0.0, float(turn_speed_yaw_rate_start or 0.0))
        self._turn_speed_min_scale = max(0.0, min(1.0, float(turn_speed_min_scale)))
        self._native_max_accel = max(0.01, float(native_max_accel or 1.0))
        self._yaw_rate_gain = max(0.0, float(yaw_rate_gain))
        self._stop_yaw_rate_gain = max(0.0, float(stop_yaw_rate_gain))
        self._dir_diff_thre = max(0.0, float(dir_diff_thre))
        self._two_way_drive = bool(kw.get("two_way_drive", True))
        self._use_incl_rate_to_slow = bool(use_incl_rate_to_slow)
        self._incl_rate_thre = math.radians(max(0.0, float(incl_rate_thre)))
        self._slow_rate_1 = max(0.0, min(1.0, float(slow_rate_1)))
        self._slow_rate_2 = max(0.0, min(1.0, float(slow_rate_2)))
        self._slow_rate_3 = max(0.0, min(1.0, float(slow_rate_3)))
        self._slow_time_1 = max(0.0, float(slow_time_1))
        self._slow_time_2 = max(0.0, float(slow_time_2))
        self._use_incl_to_stop = bool(use_incl_to_stop)
        self._incl_thre = math.radians(max(0.0, float(incl_thre)))
        self._stop_time = max(0.0, float(stop_time))

        # Current robot pose (odom frame)
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0

        # Path state for pid backend
        self._path_points = None

        # Output smoothing (shared across backends)
        self._smooth_vx = 0.0
        self._smooth_wz = 0.0

        # nav_kernel backend state
        self._nc = None           # lingtu_nav_kernel module reference
        self._nc_params = None    # PathFollowerParams
        self._nc_state = None     # PathFollowerState (persistent across calls)
        self._nc_path: list = []  # list of lingtu_nav_kernel.Vec3 (path reference frame)

        # Recorded pose at path receipt time (for vehicleRel transform)
        self._x_rec = 0.0
        self._y_rec = 0.0
        self._yaw_rec = 0.0
        self._cos_yaw_rec = 1.0
        self._sin_yaw_rec = 0.0
        self._odom_frame_id = map_frame_id()
        self._path_frame_id = map_frame_id()
        self._robot_pose_frame_id = map_frame_id()
        self._map_odom_tf: dict[str, Any] | None = None
        self._last_odometry_transform_status: dict[str, Any] = {}
        self._last_odom_ts = 0.0
        self._control_hint_timeout = float(kw.get("control_hint_timeout", 0.75))
        self._control_hint_ts = 0.0
        self._control_slow_down = 0
        self._control_safety_stop = False
        self._control_safety_stop_level = 0
        self._control_safety_stop_clear_count = 0
        self._control_hint_reason = ""
        self._incline_slow_start_ts = 0.0
        self._incline_stop_until_ts = 0.0

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.local_path.subscribe(self._on_path)
        self.control_hint.subscribe(self._on_control_hint)
        self.map_odom_tf.subscribe(self._on_map_odom_tf)
        self.map_frame_jump_event.subscribe(self._on_map_frame_jump)

        runtime = self._create_runtime(self._backend)
        self._apply_runtime(runtime)

    def _path_follower_tuning(self) -> PathFollowerTuning:
        return PathFollowerTuning(
            max_speed=self._max_speed,
            lookahead=self._lookahead,
            goal_tolerance=self._goal_tolerance,
            max_yaw_rate=self._max_yaw_rate,
            turn_speed_yaw_rate_start=self._turn_speed_yaw_rate_start,
            turn_speed_min_scale=self._turn_speed_min_scale,
            yaw_rate_gain=self._yaw_rate_gain,
            stop_yaw_rate_gain=self._stop_yaw_rate_gain,
            dir_diff_thre=self._dir_diff_thre,
            two_way_drive=self._two_way_drive,
            native_max_accel=self._native_max_accel,
        )

    def _create_runtime(self, backend: str) -> PathFollowerRuntime:
        return setup_path_follower_runtime(
            backend,
            status=self._backend_status,
            tuning=self._path_follower_tuning(),
        )

    def _setup_native_kernel(self) -> None:
        """Compatibility hook for tests and field diagnostics."""

        self._apply_runtime(self._create_runtime("nav_kernel"))

    def _apply_runtime(self, runtime: PathFollowerRuntime) -> None:
        if runtime.pid_params is None:
            if hasattr(self, "_pp_v_prev"):
                delattr(self, "_pp_v_prev")
        self._backend = runtime.backend
        self._backend_status = runtime.status
        self._nc = runtime.nav_kernel
        self._nc_params = runtime.nav_kernel_params
        self._nc_state = runtime.nav_kernel_state
        if runtime.pid_params is not None:
            self._pp_k_v = runtime.pid_params.k_v
            self._pp_l_min = runtime.pid_params.l_min
            self._pp_l_max = runtime.pid_params.l_max
            self._pp_a_max = runtime.pid_params.a_max
            self._pp_v_max = runtime.pid_params.v_max
            self._pp_v_prev = 0.0

    # -- Module lifecycle --

    def start(self):
        super().start()
        self.alive.publish(True)

    def stop(self):
        self.alive.publish(False)
        super().stop()

    # -- Callbacks --

    def _reset_native_kernel_state(self) -> None:
        if self._backend != "nav_kernel" or self._nc is None:
            return
        try:
            self._nc_state = self._nc.PathFollowerState()
            return
        except (AttributeError, TypeError):
            pass
        state = self._nc_state
        if state is None:
            return
        for attr in (
            "vehicle_speed",
            "vehicleSpeed",
            "nav_fwd",
            "pathPointID",
            "path_point_id",
            "lastPathPointID",
            "last_path_point_id",
            "lastPathSize",
            "last_path_size",
            "switchTime",
            "switch_time",
        ):
            if hasattr(state, attr):
                try:
                    setattr(state, attr, 0)
                except (AttributeError, TypeError):
                    pass

    def _reset_native_kernel_path_progress(self) -> None:
        if self._backend != "nav_kernel":
            return
        state = self._nc_state
        if state is None:
            return
        for attr in (
            "nav_fwd",
            "pathPointID",
            "path_point_id",
            "lastPathPointID",
            "last_path_point_id",
            "lastPathSize",
            "last_path_size",
            "switchTime",
            "switch_time",
        ):
            if hasattr(state, attr):
                try:
                    setattr(state, attr, 0)
                except (AttributeError, TypeError):
                    pass

    def _publish_zero(self, *, reset_native_kernel_state: bool = False) -> None:
        self._smooth_vx = 0.0
        self._smooth_wz = 0.0
        if hasattr(self, "_pp_v_prev"):
            self._pp_v_prev = 0.0
        if reset_native_kernel_state:
            self._reset_native_kernel_state()
        self.cmd_vel.publish(Twist())

    def _reset_path_tracking(self, *, reset_native_kernel_state: bool = True) -> None:
        self._nc_path = []
        self._path_points = None
        self._x_rec = self._robot_x
        self._y_rec = self._robot_y
        self._yaw_rec = self._robot_yaw
        self._cos_yaw_rec = math.cos(self._yaw_rec)
        self._sin_yaw_rec = math.sin(self._yaw_rec)
        self._publish_zero(reset_native_kernel_state=reset_native_kernel_state)

    def _on_control_hint(self, hint: dict) -> None:
        if not isinstance(hint, dict):
            return
        try:
            slow_down = int(hint.get("slow_down", 0) or 0)
        except (TypeError, ValueError):
            slow_down = 0
        self._control_slow_down = max(0, min(3, slow_down))
        has_stop_hint = any(
            key in hint for key in ("safety_stop", "safety_stop_level", "near_field_stop")
        )
        if has_stop_hint:
            stop_active = (
                bool(hint.get("safety_stop"))
                if "safety_stop" in hint
                else bool(hint.get("near_field_stop"))
            )
            try:
                level = max(
                    0,
                    min(
                        2,
                        int(hint.get("safety_stop_level", 2 if stop_active else 0) or 0),
                    ),
                )
            except (TypeError, ValueError):
                level = 2 if stop_active else 0
            self._apply_safety_stop_level(level)
        self._control_hint_reason = str(hint.get("reason") or "")
        self._control_hint_ts = float(hint.get("ts") or time.time())
        if self._control_safety_stop_level >= 2:
            self._publish_zero(reset_native_kernel_state=False)

    def _apply_safety_stop_level(self, level: int) -> None:
        if level > self._control_safety_stop_level:
            self._control_safety_stop_level = level
            self._control_safety_stop_clear_count = 0
        elif level == 0 and self._control_safety_stop_level > 0:
            self._control_safety_stop_clear_count += 1
            if self._control_safety_stop_clear_count >= 3:
                self._control_safety_stop_level = 0
                self._control_safety_stop_clear_count = 0
        elif level > 0:
            self._control_safety_stop_clear_count = 0
        self._control_safety_stop = self._control_safety_stop_level > 0

    def _update_incline_guard(self, odom: Odometry) -> None:
        now = time.time()
        if self._use_incl_to_stop:
            rpy = odom.pose.orientation.to_euler()
            if abs(rpy.x) > self._incl_thre or abs(rpy.y) > self._incl_thre:
                self._incline_stop_until_ts = max(
                    self._incline_stop_until_ts,
                    now + self._stop_time,
                )
        if self._use_incl_rate_to_slow:
            angular = odom.twist.angular
            if abs(angular.x) > self._incl_rate_thre or abs(angular.y) > self._incl_rate_thre:
                self._incline_slow_start_ts = now

    def _active_control_guard(self) -> tuple[float, int]:
        now = time.time()
        if (
            self._control_hint_ts > 0
            and now - self._control_hint_ts > self._control_hint_timeout
        ):
            self._control_slow_down = 0
        slow_factor_by_level = {
            0: 1.0,
            1: self._slow_rate_1,
            2: self._slow_rate_2,
            3: self._slow_rate_3,
        }
        slow_factor = slow_factor_by_level.get(self._control_slow_down, 1.0)
        if self._incline_slow_start_ts > 0:
            elapsed = now - self._incline_slow_start_ts
            if elapsed < self._slow_time_1:
                slow_factor = min(slow_factor, self._slow_rate_1)
            elif elapsed < self._slow_time_1 + self._slow_time_2:
                slow_factor = min(slow_factor, self._slow_rate_2)
            else:
                self._incline_slow_start_ts = 0.0
        level = self._control_safety_stop_level if self._control_safety_stop else 0
        if now < self._incline_stop_until_ts:
            level = max(level, 2)
        elif self._incline_stop_until_ts > 0:
            self._incline_stop_until_ts = 0.0
        return slow_factor, level

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
        if is_map_frame_jump_event(event):
            self._reset_path_tracking(reset_native_kernel_state=True)

    def _on_map_odom_tf(self, msg: dict) -> None:
        if isinstance(msg, dict) and msg.get("valid") is not False:
            self._map_odom_tf = dict(msg)
        else:
            self._map_odom_tf = None

    def _on_odom(self, odom: Odometry):
        _prev_x, _prev_y = self._robot_x, self._robot_y
        source_frame = normalize_frame_id(getattr(odom, "frame_id", None))
        if source_frame:
            self._odom_frame_id = source_frame
        target_frame = self._path_frame_id
        pos, yaw, output_frame, transformed, reason = transform_xyz_yaw_with_map_odom(
            [odom.pose.position.x, odom.pose.position.y, odom.pose.position.z],
            odom.yaw,
            source_frame=source_frame,
            target_frame=target_frame,
            map_odom_tf=self._map_odom_tf,
        )
        self._robot_x = pos[0]
        self._robot_y = pos[1]
        self._robot_pose_frame_id = output_frame or source_frame or self._odom_frame_id
        self._last_odometry_transform_status = {
            "source_frame": source_frame,
            "target_frame": target_frame,
            "output_frame": self._robot_pose_frame_id,
            "transformed": transformed,
            "reason": reason,
        }
        self._last_odom_ts = float(getattr(odom, "ts", 0.0) or time.time())

        # Extract yaw from quaternion (reliable, works at all speeds)
        self._robot_yaw = float(yaw if yaw is not None else odom.yaw)
        self._update_incline_guard(odom)

        if self._backend == "nav_kernel" and self._nc_path:
            self._native_kernel_step(odom.ts)
        elif self._backend == "pid" and self._path_points is not None:
            # Guard against synchronous cmd_vel->odom->pid_step recursion
            # in callback transport (no issue with DDS/shm transport).
            if not getattr(self, '_in_pid', False):
                self._in_pid = True
                self._pid_step()
                self._in_pid = False

    def _on_path(self, path: Path):
        if self._backend == "nav_kernel":
            # Record robot pose at path receipt - defines the reference frame
            frame_id = normalize_frame_id(getattr(path, "frame_id", None))
            self._path_frame_id = frame_id or self._odom_frame_id
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

            # nav_kernel expects path points and vehicleRel in the same frame.
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
                self._reset_path_tracking(reset_native_kernel_state=True)
                return
            self._reset_native_kernel_path_progress()
            self._nc_path = pts
            if self._last_odom_ts > 0:
                self._native_kernel_step(self._last_odom_ts)
            # Each fresh local path has its own waypoint index and direction.
            # Reusing the previous nav_kernel state can keep the follower pointed
            # at a stale path point after a replan or patrol checkpoint switch.

        elif self._backend == "pid":
            self._path_frame_id = (
                normalize_frame_id(getattr(path, "frame_id", None))
                or self._odom_frame_id
            )
            pts = []
            for ps in path.poses:
                pts.append([ps.pose.position.x, ps.pose.position.y])
            if len(pts) < 2:
                self._reset_path_tracking(reset_native_kernel_state=False)
                return
            self._path_points = np.array(pts)
            # Kick off the cmd_vel -> odom loop immediately
            if self._path_points is not None:
                self._pid_step()

    # -- nav_kernel control step --

    def _to_path_reference_frame(self, x: float, y: float) -> tuple[float, float]:
        dx = x - self._x_rec
        dy = y - self._y_rec
        return (
            self._cos_yaw_rec * dx + self._sin_yaw_rec * dy,
            -self._sin_yaw_rec * dx + self._cos_yaw_rec * dy,
        )

    def _native_kernel_step(self, current_time: float):
        """Compute cmd_vel using C++ compute_control."""
        if not self._nc_path:
            self._publish_zero()
            return
        if self._robot_pose_frame_id != self._path_frame_id:
            self._publish_zero(reset_native_kernel_state=False)
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
        slow_factor, safety_stop = self._active_control_guard()
        if safety_stop >= 2:
            self._publish_zero(reset_native_kernel_state=False)
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

        self._smooth_vx = out.cmd.vx
        self._smooth_wz = out.cmd.wz

        self.cmd_vel.publish(Twist(
            linear=Vector3(out.cmd.vx, out.cmd.vy, 0.0),
            angular=Vector3(0.0, 0.0, out.cmd.wz),
        ))

    # -- Python PID (fallback) --

    def _pid_step(self):
        """Simple Pure Pursuit in Python for testing."""
        if self._path_points is None or len(self._path_points) < 2:
            self._publish_zero()
            return
        if self._robot_pose_frame_id != self._path_frame_id:
            self._publish_zero(reset_native_kernel_state=False)
            return
        slow_factor, safety_stop = self._active_control_guard()
        if safety_stop >= 1:
            self._publish_zero(reset_native_kernel_state=False)
            return

        robot = np.array([self._robot_x, self._robot_y])
        dists = np.linalg.norm(self._path_points - robot, axis=1)
        goal_dist = float(dists[-1])
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

        if goal_dist < self._goal_tolerance or dist < self._goal_tolerance:
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

    @staticmethod
    def _native_state_attr(state: Any, *names: str, default: Any = None) -> Any:
        for name in names:
            if hasattr(state, name):
                return getattr(state, name)
        return default

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        h = {
            **self._backend_status.as_health_fields(),
            "has_path": bool(self._nc_path) if self._backend == "nav_kernel"
                        else self._path_points is not None,
            "max_yaw_rate": self._max_yaw_rate,
            "turn_speed_yaw_rate_start": self._turn_speed_yaw_rate_start,
            "turn_speed_min_scale": self._turn_speed_min_scale,
            "native_max_accel": self._native_max_accel,
            "yaw_rate_gain": self._yaw_rate_gain,
            "stop_yaw_rate_gain": self._stop_yaw_rate_gain,
            "dir_diff_thre": self._dir_diff_thre,
            "control_hint": {
                "slow_down": self._control_slow_down,
                "safety_stop": self._control_safety_stop,
                "safety_stop_level": self._control_safety_stop_level,
                "safety_stop_clear_count": self._control_safety_stop_clear_count,
                "reason": self._control_hint_reason,
                "age_ms": (
                    round((time.time() - self._control_hint_ts) * 1000)
                    if self._control_hint_ts > 0
                    else None
                ),
            },
            "frames": {
                "path_frame": self._path_frame_id,
                "odom_frame": self._odom_frame_id,
                "robot_pose_frame": self._robot_pose_frame_id,
                "odometry_transform": dict(self._last_odometry_transform_status),
            },
            "incline_guard": {
                "slow_active": self._incline_slow_start_ts > 0,
                "stop_active": time.time() < self._incline_stop_until_ts,
            },
        }
        if self._backend == "nav_kernel" and self._nc_state is not None:
            state = self._nc_state
            native_state = {
                "vehicle_speed": self._native_state_attr(
                    state,
                    "vehicle_speed",
                    "vehicleSpeed",
                    default=0.0,
                ),
                "path_point_id": self._native_state_attr(
                    state,
                    "path_point_id",
                    "pathPointID",
                    default=0,
                ),
                "last_path_point_id": self._native_state_attr(
                    state,
                    "last_path_point_id",
                    "lastPathPointID",
                    default=0,
                ),
                "last_path_size": self._native_state_attr(
                    state,
                    "last_path_size",
                    "lastPathSize",
                    default=0,
                ),
                "nav_fwd": self._native_state_attr(
                    state,
                    "nav_fwd",
                    "navFwd",
                    default=True,
                ),
                "switch_time": self._native_state_attr(
                    state,
                    "switch_time",
                    "switchTime",
                    default=0.0,
                ),
            }
            h["native_state"] = native_state
            h["vehicle_speed"] = native_state["vehicle_speed"]
            h["nav_fwd"] = native_state["nav_fwd"]
        info["path_follower"] = h
        return info
