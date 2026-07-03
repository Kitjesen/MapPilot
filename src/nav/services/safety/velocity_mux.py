"""VelocityMux -priority-based velocity command arbitration.

Multiple sources (teleop, visual servo, path follower, recovery) can publish
cmd_vel.  VelocityMux selects the highest-priority *active* source and forwards
only that source's commands to the driver.

A source is "active" if it published within its timeout window.  When the
active source goes idle, the mux falls through to the next lower-priority
source that has recently published.

Priority (higher = wins):
    teleop          100   -human joystick override
    visual_servo     80   -close-range PD tracking
    recovery         60   -Navigation stuck recovery
    path_follower    40   -normal autonomy

All incoming cmd_vel arrive on separate In ports so the mux knows *which*
source sent the message.  One Out[Twist] port goes to the driver.

The mux also publishes `active_source: Out[str]` so other modules (Gateway,
SafetyRing, etc.) can display who controls the robot.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any

from runtime.module import Module
from runtime.msgs.geometry import Twist
from runtime.msgs.nav import Odometry
from runtime.registry import register
from runtime.stream import In, Out

logger = logging.getLogger(__name__)

# Source definitions: name ->default priority
_DEFAULT_PRIORITIES: dict[str, int] = {
    "teleop": 100,
    "visual_servo": 80,
    "recovery": 60,
    "path_follower": 40,
}

_NATIVE_COLLISION_SYMBOLS = (
    "Grid2D",
    "CmdVelCollisionParams",
    "project_cmd_vel_collision",
)

_NATIVE_COLLISION_ACTIONS = {
    0: "pass",
    1: "slowdown",
    2: "stop",
}

_NATIVE_COLLISION_REASONS = {
    0: "clear",
    1: "projected_near_obstacle",
    2: "projected_collision",
    3: "projected_outside_costmap",
    4: "nonfinite_projection",
}


@register("safety", "cmd_vel_mux", description="Priority-based cmd_vel arbitration")
class VelocityMux(Module, layer=0):
    """Priority-based cmd_vel multiplexer.

    Each source has its own In port.  The mux forwards the highest-priority
    active source to driver_cmd_vel.
    """

    runtime_id = "nav.velocity_mux"

    # -- Inputs (one per source) --
    teleop_cmd_vel:        In[Twist]
    visual_servo_cmd_vel:  In[Twist]
    recovery_cmd_vel:      In[Twist]
    path_follower_cmd_vel: In[Twist]
    collision_odometry:    In[Odometry]
    collision_costmap:     In[dict]

    # -- Outputs --
    driver_cmd_vel: Out[Twist]     # ->driver
    active_source:  Out[str]       # who currently controls

    def __init__(
        self,
        source_timeout: float = 0.5,
        enable_collision_monitor: bool = False,
        collision_monitor_timeout_s: float = 1.0,
        collision_monitor_horizon_s: float = 1.0,
        collision_monitor_step_s: float = 0.1,
        collision_monitor_stop_cost: float = 99.0,
        collision_monitor_slow_cost: float = 60.0,
        collision_monitor_slowdown_scale: float = 0.35,
        **kw,
    ):
        super().__init__(**kw)
        self._source_timeout = source_timeout
        self._collision_monitor_enabled = bool(enable_collision_monitor)
        self._collision_timeout_s = max(0.05, float(collision_monitor_timeout_s))
        self._collision_horizon_s = max(0.05, float(collision_monitor_horizon_s))
        self._collision_step_s = max(0.02, float(collision_monitor_step_s))
        self._collision_stop_cost = float(collision_monitor_stop_cost)
        self._collision_slow_cost = float(collision_monitor_slow_cost)
        self._collision_slowdown_scale = min(
            1.0,
            max(0.0, float(collision_monitor_slowdown_scale)),
        )
        self._collision_native_runtime = (
            self._load_collision_native()
            if self._collision_monitor_enabled
            else None
        )
        self._collision_native_params = self._make_collision_native_params()

        # Per-source state: last twist + last publish time
        self._sources: dict[str, dict] = {}
        for name, priority in _DEFAULT_PRIORITIES.items():
            self._sources[name] = {
                "priority": priority,
                "last_time": 0.0,
                "last_twist": Twist(),
            }

        self._active: str = ""
        self._last_publish_time: float = 0.0
        self._last_driver_twist: Twist = Twist.zero()
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._monitor_thread: threading.Thread | None = None
        self._frozen: bool = False
        self._latest_costmap: dict[str, Any] | None = None
        self._latest_native_costmap: Any | None = None
        self._latest_costmap_time: float = 0.0
        self._latest_odom: Odometry | None = None
        self._latest_odom_time: float = 0.0
        self._collision_state: dict[str, Any] = {
            "enabled": self._collision_monitor_enabled,
            "action": "disabled" if not self._collision_monitor_enabled else "waiting",
            "reason": (
                "disabled"
                if not self._collision_monitor_enabled
                else "waiting_for_costmap"
            ),
            "last_cost": None,
            "costmap_age_ms": None,
            "odometry_age_ms": None,
            "horizon_s": self._collision_horizon_s,
            "stop_cost": self._collision_stop_cost,
            "slow_cost": self._collision_slow_cost,
            "slowdown_scale": self._collision_slowdown_scale,
            "backend": self._collision_backend_name(),
            "ts": None,
        }

    @staticmethod
    def _load_collision_native() -> Any | None:
        try:
            from nav.kernel import try_import_nav_kernel
        except Exception as exc:  # pragma: no cover - import path failures are platform-specific.
            logger.debug("VelocityMux: nav kernel loader unavailable: %s", exc)
            return None
        return try_import_nav_kernel(_NATIVE_COLLISION_SYMBOLS)

    def _make_collision_native_params(self) -> Any | None:
        runtime = self._collision_native_runtime
        if runtime is None:
            return None
        params = runtime.CmdVelCollisionParams()
        params.horizon_s = self._collision_horizon_s
        params.step_s = self._collision_step_s
        params.stop_cost = self._collision_stop_cost
        params.slow_cost = self._collision_slow_cost
        return params

    def _collision_backend_name(self) -> str | None:
        if not self._collision_monitor_enabled:
            return None
        return "native" if self._collision_native_runtime is not None else "python"

    def freeze(self) -> None:
        self._frozen = True
        zero = Twist.zero()
        with self._lock:
            self._last_driver_twist = zero
            self._last_publish_time = time.time()
        self.driver_cmd_vel.publish(zero)
        logger.info("VelocityMux: frozen")

    def unfreeze(self) -> None:
        self._frozen = False
        logger.info("VelocityMux: unfrozen")

    @property
    def is_frozen(self) -> bool:
        return self._frozen

    def setup(self) -> None:
        self.teleop_cmd_vel.subscribe(
            lambda t: self._on_source("teleop", t))
        self.visual_servo_cmd_vel.subscribe(
            lambda t: self._on_source("visual_servo", t))
        self.recovery_cmd_vel.subscribe(
            lambda t: self._on_source("recovery", t))
        self.path_follower_cmd_vel.subscribe(
            lambda t: self._on_source("path_follower", t))
        self.collision_odometry.subscribe(self._on_odometry)
        self.collision_odometry.set_policy("latest")
        self.collision_costmap.subscribe(self._on_costmap)
        self.collision_costmap.set_policy("latest")

    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        if self._monitor_thread is None or not self._monitor_thread.is_alive():
            self._monitor_thread = threading.Thread(
                target=self._timeout_loop,
                name="cmd_vel_mux_timeout",
                daemon=True,
            )
            self._monitor_thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=1.0)
        self._monitor_thread = None
        super().stop()

    def _timeout_loop(self) -> None:
        interval = max(0.02, min(0.1, self._source_timeout / 2.0))
        while not self._stop_event.wait(interval):
            self._check_timeout()

    @staticmethod
    def _sanitize_twist(twist: Twist) -> Twist:
        values = (
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
        )
        if all(math.isfinite(float(v)) for v in values):
            return twist
        logger.warning("VelocityMux: dropping non-finite cmd_vel")
        return Twist.zero()

    @staticmethod
    def _is_zero_twist(twist: Twist) -> bool:
        values = (
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
        )
        return all(abs(float(v)) <= 1e-9 for v in values)

    def _on_source(self, name: str, twist: Twist) -> None:
        """Handle incoming cmd_vel from a named source."""
        if self._frozen:
            return
        now = time.time()
        twist = self._sanitize_twist(twist)
        release_source = name != "teleop" and self._is_zero_twist(twist)
        active_update: str | None = None
        driver_twist: Twist | None = None

        with self._lock:
            src = self._sources[name]
            src["last_time"] = 0.0 if release_source else now
            src["last_twist"] = twist

            winner = self._select_active(now)
            previous_active = self._active
            if winner != self._active:
                if self._active and winner:
                    logger.info("VelocityMux: %s -> %s", self._active, winner)
                elif winner:
                    logger.info("VelocityMux: %s active", winner)
                self._active = winner
                active_update = winner

            if active_update is not None:
                driver_twist = (
                    self._apply_collision_monitor(
                        self._sanitize_twist(self._sources[winner]["last_twist"]),
                        now,
                    )
                    if winner
                    else Twist.zero()
                )
            elif name == winner:
                driver_twist = self._apply_collision_monitor(twist, now)
            elif release_source and previous_active == name:
                driver_twist = Twist.zero()

            if driver_twist is not None:
                self._last_driver_twist = driver_twist
                self._last_publish_time = now

        if active_update is not None:
            self.active_source.publish(active_update)
        if driver_twist is not None:
            self.driver_cmd_vel.publish(driver_twist)

    def _check_timeout(self, now: float | None = None) -> None:
        if self._frozen:
            return
        now = time.time() if now is None else now
        with self._lock:
            winner = self._select_active(now)
            if winner == self._active:
                return
            self._active = winner
            driver_twist = (
                self._sanitize_twist(self._sources[winner]["last_twist"])
                if winner
                else Twist.zero()
            )
            if winner:
                driver_twist = self._apply_collision_monitor(driver_twist, now)
            self._last_driver_twist = driver_twist
            self._last_publish_time = now

        self.active_source.publish(winner)
        self.driver_cmd_vel.publish(driver_twist)

    def _select_active(self, now: float) -> str:
        """Return the name of the highest-priority source that is still active."""
        with self._lock:
            best_name = ""
            best_priority = -1
            for name, src in self._sources.items():
                age = now - src["last_time"]
                if age <= self._source_timeout and src["priority"] > best_priority:
                    best_name = name
                    best_priority = src["priority"]
            return best_name

    def _on_odometry(self, odom: Odometry) -> None:
        now = time.time()
        with self._lock:
            self._latest_odom = odom
            self._latest_odom_time = now
        self._reevaluate_active_collision(now)

    def _on_costmap(self, costmap: dict) -> None:
        now = time.time()
        native_costmap = self._make_native_collision_costmap(costmap)
        with self._lock:
            self._latest_costmap = costmap
            self._latest_native_costmap = native_costmap
            self._latest_costmap_time = now
        self._reevaluate_active_collision(now)

    def _reevaluate_active_collision(self, now: float) -> None:
        if not self._collision_monitor_enabled or self._frozen:
            return
        with self._lock:
            winner = self._select_active(now)
            if not winner or winner != self._active:
                return
            twist = self._sanitize_twist(self._sources[winner]["last_twist"])
            driver_twist = self._apply_collision_monitor(twist, now)
            self._last_driver_twist = driver_twist
            self._last_publish_time = now
        self.driver_cmd_vel.publish(driver_twist)

    def _apply_collision_monitor(self, twist: Twist, now: float) -> Twist:
        if not self._collision_monitor_enabled:
            return twist
        if twist.is_zero():
            self._set_collision_state(
                "pass",
                "zero_cmd",
                0.0,
                now,
                backend=self._collision_backend_name(),
            )
            return twist

        with self._lock:
            costmap = self._latest_costmap
            native_costmap = self._latest_native_costmap
            costmap_time = self._latest_costmap_time
            odom = self._latest_odom
            odom_time = self._latest_odom_time

        if costmap is None:
            self._set_collision_state(
                "stop",
                "costmap_missing",
                None,
                now,
                backend=self._collision_backend_name(),
            )
            return Twist.zero()
        if now - costmap_time > self._collision_timeout_s:
            self._set_collision_state(
                "stop",
                "costmap_stale",
                None,
                now,
                backend=self._collision_backend_name(),
            )
            return Twist.zero()
        if odom is None:
            self._set_collision_state(
                "stop",
                "odometry_missing",
                None,
                now,
                backend=self._collision_backend_name(),
            )
            return Twist.zero()
        if now - odom_time > self._collision_timeout_s:
            self._set_collision_state(
                "stop",
                "odometry_stale",
                None,
                now,
                backend=self._collision_backend_name(),
            )
            return Twist.zero()

        action, reason, cost, backend = self._collision_action(
            twist,
            costmap,
            odom,
            native_costmap,
        )
        self._set_collision_state(action, reason, cost, now, backend=backend)
        if action == "stop":
            return Twist.zero()
        if action == "slowdown":
            return self._scale_twist(twist, self._collision_slowdown_scale)
        return twist

    def _collision_action(
        self,
        twist: Twist,
        costmap: dict[str, Any],
        odom: Odometry,
        native_costmap: Any | None,
    ) -> tuple[str, str, float | None, str]:
        native_result = self._collision_action_native(twist, odom, native_costmap)
        if native_result is not None:
            return (*native_result, "native")
        action, reason, cost = self._collision_action_python(twist, costmap, odom)
        return action, reason, cost, "python"

    def _collision_action_native(
        self,
        twist: Twist,
        odom: Odometry,
        native_costmap: Any | None,
    ) -> tuple[str, str, float | None] | None:
        runtime = self._collision_native_runtime
        params = self._collision_native_params
        if runtime is None or params is None or native_costmap is None:
            return None
        try:
            result = runtime.project_cmd_vel_collision(
                native_costmap,
                float(odom.x),
                float(odom.y),
                float(odom.yaw),
                float(twist.linear.x),
                float(twist.linear.y),
                float(twist.angular.z),
                params,
            )
        except Exception as exc:
            logger.debug("VelocityMux: native collision projection failed: %s", exc)
            return None
        action = _NATIVE_COLLISION_ACTIONS.get(int(result.action), "stop")
        reason = _NATIVE_COLLISION_REASONS.get(int(result.reason), "nonfinite_projection")
        try:
            cost = float(result.max_cost)
        except (TypeError, ValueError):
            cost = None
        if cost is not None and not math.isfinite(cost):
            cost = None
        return action, reason, cost

    def _collision_action_python(
        self,
        twist: Twist,
        costmap: dict[str, Any],
        odom: Odometry,
    ) -> tuple[str, str, float | None]:
        grid = costmap.get("grid") if isinstance(costmap, dict) else None
        if grid is None:
            return "stop", "costmap_grid_missing", None
        shape = self._grid_shape(grid)
        if shape is None:
            return "stop", "costmap_grid_invalid", None
        height, width = shape
        resolution = self._costmap_resolution(costmap)
        origin = self._costmap_origin(costmap)
        if resolution is None or origin is None or height <= 0 or width <= 0:
            return "stop", "costmap_geometry_invalid", None

        x = float(odom.x)
        y = float(odom.y)
        yaw = float(odom.yaw)
        vx = float(twist.linear.x)
        vy = float(twist.linear.y)
        wz = float(twist.angular.z)
        if not all(math.isfinite(v) for v in (x, y, yaw, vx, vy, wz)):
            return "stop", "nonfinite_projection", None

        max_cost = 0.0
        steps = max(
            1,
            int(math.ceil(self._collision_horizon_s / self._collision_step_s)),
        )
        for step in range(steps + 1):
            if step > 0:
                dt = self._collision_step_s
                c = math.cos(yaw)
                s = math.sin(yaw)
                x += (c * vx - s * vy) * dt
                y += (s * vx + c * vy) * dt
                yaw += wz * dt

            cost = self._sample_cost(grid, height, width, resolution, origin, x, y)
            if cost is None:
                return "stop", "projected_outside_costmap", max_cost
            max_cost = max(max_cost, cost)
            if cost >= self._collision_stop_cost:
                return "stop", "projected_collision", cost

        if max_cost >= self._collision_slow_cost:
            return "slowdown", "projected_near_obstacle", max_cost
        return "pass", "clear", max_cost

    def _set_collision_state(
        self,
        action: str,
        reason: str,
        cost: float | None,
        now: float,
        *,
        backend: str | None = None,
    ) -> None:
        with self._lock:
            costmap_age = (
                now - self._latest_costmap_time
                if self._latest_costmap_time > 0.0
                else None
            )
            odom_age = (
                now - self._latest_odom_time
                if self._latest_odom_time > 0.0
                else None
            )
            self._collision_state.update(
                {
                    "enabled": self._collision_monitor_enabled,
                    "action": action,
                    "reason": reason,
                    "last_cost": None if cost is None else round(float(cost), 3),
                    "costmap_age_ms": (
                        None if costmap_age is None else round(costmap_age * 1000)
                    ),
                    "odometry_age_ms": (
                        None if odom_age is None else round(odom_age * 1000)
                    ),
                    "backend": backend,
                    "ts": now,
                }
            )

    @staticmethod
    def _scale_twist(twist: Twist, scale: float) -> Twist:
        return Twist(
            linear=twist.linear * scale,
            angular=twist.angular * scale,
        )

    @staticmethod
    def _grid_shape(grid: Any) -> tuple[int, int] | None:
        shape = getattr(grid, "shape", None)
        if shape is not None and len(shape) >= 2:
            return int(shape[0]), int(shape[1])
        try:
            height = len(grid)
            width = len(grid[0]) if height else 0
            return int(height), int(width)
        except (TypeError, IndexError):
            return None

    @staticmethod
    def _costmap_resolution(costmap: dict[str, Any]) -> float | None:
        try:
            resolution = float(costmap.get("resolution", 0.0))
        except (TypeError, ValueError):
            return None
        return resolution if math.isfinite(resolution) and resolution > 0.0 else None

    @staticmethod
    def _costmap_origin(costmap: dict[str, Any]) -> tuple[float, float] | None:
        origin = costmap.get("origin")
        if origin is None:
            origin = [costmap.get("origin_x"), costmap.get("origin_y")]
        try:
            if isinstance(origin, dict):
                x = float(origin.get("x", origin.get("origin_x")))
                y = float(origin.get("y", origin.get("origin_y")))
            else:
                x = float(origin[0])
                y = float(origin[1])
        except (TypeError, ValueError, IndexError, KeyError):
            return None
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        return x, y

    @staticmethod
    def _coerce_cost_value(value: Any) -> float:
        try:
            cost = float(value)
        except (TypeError, ValueError):
            return 100.0
        return cost if math.isfinite(cost) else 100.0

    def _make_native_collision_costmap(self, costmap: dict[str, Any]) -> Any | None:
        runtime = self._collision_native_runtime
        if runtime is None or not isinstance(costmap, dict):
            return None
        grid = costmap.get("grid")
        if grid is None:
            return None
        shape = self._grid_shape(grid)
        resolution = self._costmap_resolution(costmap)
        origin = self._costmap_origin(costmap)
        if shape is None or resolution is None or origin is None:
            return None
        height, width = shape
        if height <= 0 or width <= 0:
            return None

        try:
            native = runtime.Grid2D()
            native.rows = int(height)
            native.cols = int(width)
            native.resolution = float(resolution)
            native.origin_x = float(origin[0])
            native.origin_y = float(origin[1])
            ravel = getattr(grid, "ravel", None)
            if callable(ravel):
                values = [self._coerce_cost_value(value) for value in ravel()]
            else:
                values = [
                    self._coerce_cost_value(grid[row][col])
                    for row in range(height)
                    for col in range(width)
                ]
            if len(values) != height * width:
                return None
            native.data = values
            return native
        except Exception as exc:
            logger.debug("VelocityMux: native costmap conversion failed: %s", exc)
            return None

    @staticmethod
    def _sample_cost(
        grid: Any,
        height: int,
        width: int,
        resolution: float,
        origin: tuple[float, float],
        x: float,
        y: float,
    ) -> float | None:
        col = int(math.floor((x - origin[0]) / resolution))
        row = int(math.floor((y - origin[1]) / resolution))
        if row < 0 or row >= height or col < 0 or col >= width:
            return None
        try:
            value = grid[row, col]
        except (TypeError, KeyError):
            value = grid[row][col]
        try:
            cost = float(value)
        except (TypeError, ValueError):
            return 100.0
        return cost if math.isfinite(cost) else 100.0

    # -- health ---------------------------------------------------------

    def health(self) -> dict:
        now = time.time()
        with self._lock:
            sources = {}
            for name, src in self._sources.items():
                last_time = src["last_time"]
                age = now - last_time if last_time > 0.0 else None
                sources[name] = {
                    "priority": src["priority"],
                    "active": age is not None and age <= self._source_timeout,
                    "age_ms": round(age * 1000) if age is not None else None,
                }
            active = self._active or "none"
            twist = self._last_driver_twist
            last_publish_time = self._last_publish_time
            collision_state = dict(self._collision_state)
        return {
            "active_source": active,
            "source_timeout_s": float(self._source_timeout),
            "sources": sources,
            "collision_monitor": collision_state,
            "last_driver_cmd_vel": {
                "linear": {
                    "x": float(twist.linear.x),
                    "y": float(twist.linear.y),
                    "z": float(twist.linear.z),
                },
                "angular": {
                    "x": float(twist.angular.x),
                    "y": float(twist.angular.y),
                    "z": float(twist.angular.z),
                },
                "ts": last_publish_time if last_publish_time > 0.0 else None,
                "active_source": active,
            },
        }
