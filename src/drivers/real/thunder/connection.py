# DEPRECATED: Use ThunderDriver from han_dog_module.py instead.
# This file is kept for backward compatibility with legacy blueprints.
# TODO: Remove once all consumers are migrated.
"""
NovaDogConnection — Quadruped robot gRPC bridge module.

DEPRECATED: This module is superseded by ThunderDriver in han_dog_module.py,
which is the canonical driver registered as ("driver", "thunder") in the
registry and used by all current blueprints. ThunderDriver includes
thread-safe cmd_vel handling and brainstem reachability checks that this
module lacks.

Ports:
  In:  cmd_vel      (Twist)       — velocity command → gRPC Walk()
       stop_signal  (int)         — 0=normal, 1=soft_stop, 2=hard_stop
       slam_odom    (Odometry)    — SLAM position reset (prevent integration drift)
  Out: odometry     (Odometry)    — IMU orientation + cmd_vel position integration
       alive        (bool)        — gRPC connection status

Usage::

    # Prefer ThunderDriver instead:
    from drivers.real.thunder.han_dog_module import ThunderDriver

    # Legacy usage (deprecated):
    from drivers.real.thunder.connection import NovaDogConnection
"""

from __future__ import annotations

import asyncio
import logging
import math
import os
import sys
import threading
import time
from typing import Any

try:
    from core.module import Module
    from core.msgs.geometry import Quaternion, Twist, Vector3
    from core.msgs.nav import Odometry
    from core.registry import register
    from core.runtime_interface import body_frame_id, odom_frame_id
    from core.stream import In, Out
except ModuleNotFoundError as exc:
    if exc.name != "core":
        raise
    _src = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    _known_paths = {os.path.normcase(os.path.abspath(path or ".")) for path in sys.path}
    if os.path.normcase(_src) not in _known_paths:
        sys.path.insert(0, _src)
    from core.module import Module
    from core.msgs.geometry import Quaternion, Twist, Vector3
    from core.msgs.nav import Odometry
    from core.registry import register
    from core.runtime_interface import body_frame_id, odom_frame_id
    from core.stream import In, Out

logger = logging.getLogger(__name__)
NOVA_DOG_ODOM_FRAME_ID = odom_frame_id()
NOVA_DOG_BODY_FRAME_ID = body_frame_id()


@register("driver", "nova_dog", priority=10, platforms={"aarch64"},
          description="NOVA quadruped via brainstem gRPC")
# Deliberately NOT registering as "driver_protocol" — ThunderDriver in
# han_dog_module.py is the canonical grpc_brainstem implementation.
# This deprecated module only carries @register("driver", "nova_dog") for
# backward compatibility with legacy blueprints.
# See han_dog_module.py for the canonical @register("driver_protocol", "grpc_brainstem").
class NovaDogConnection(Module, layer=1):
    """Quadruped robot gRPC bridge module.

    In:  cmd_vel → normalize to [-1,1] → gRPC Walk()
    Out: odometry ← IMU orientation + dead-reckoning position integration
    """

    # -- Ports --
    cmd_vel: In[Twist]
    stop_signal: In[int]
    slam_odom: In[Odometry]
    odometry: Out[Odometry]
    alive: Out[bool]

    def __init__(
        self,
        dog_host: str = "127.0.0.1",
        dog_port: int = 13145,
        max_linear_speed: float = 1.0,
        max_angular_speed: float = 1.0,
        cmd_vel_timeout_ms: float = 200.0,
        control_rate: float = 50.0,
        auto_enable: bool = True,
        auto_standup: bool = True,
        reconnect_interval: float = 3.0,
        odom_pub_rate: float = 10.0,
        slam_reset_interval: float = 5.0,
        **kw,
    ):
        super().__init__(**kw)
        self._dog_host = dog_host
        self._dog_port = dog_port
        self._max_linear = max_linear_speed
        self._max_angular = max_angular_speed
        self._watchdog_timeout = cmd_vel_timeout_ms / 1000.0
        self._control_rate = control_rate
        self._auto_enable = auto_enable
        self._auto_standup = auto_standup
        self._reconnect_interval = reconnect_interval
        self._odom_skip = max(1, int(control_rate / odom_pub_rate))
        self._slam_reset_interval = slam_reset_interval

        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0
        self._last_cmd_time = time.time()
        self._watchdog_triggered = False
        self._connected = False
        self._standing = False
        self._enabled = False
        self._shutdown = False

        # gRPC (initialized in asyncio thread)
        self._channel = None
        self._stub = None

        # Dead-reckoning position
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._last_odom_time: float | None = None
        self._last_slam_reset = time.time()

        # IMU cache
        self._latest_quat = (0.0, 0.0, 0.0, 1.0)  # x, y, z, w
        self._latest_gyro = (0.0, 0.0, 0.0)

        # asyncio
        self._loop: asyncio.AbstractEventLoop | None = None
        self._grpc_thread: threading.Thread | None = None

    # -- Lifecycle --

    def setup(self) -> None:
        self.cmd_vel.subscribe(self._on_cmd_vel)
        self.stop_signal.subscribe(self._on_stop)
        self.slam_odom.subscribe(self._on_slam_odom)

    def start(self):
        super().start()
        self._shutdown = False

        self._loop = asyncio.new_event_loop()
        self._grpc_thread = threading.Thread(
            target=self._run_async_loop, daemon=True)
        self._grpc_thread.start()

        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, daemon=True)
        self._watchdog_thread.start()

        logger.info(
            "NovaDogConnection started → %s:%d (max_lin=%.1f, max_ang=%.1f)",
            self._dog_host, self._dog_port, self._max_linear, self._max_angular,
        )

    def stop(self):
        self._shutdown = True
        loop = self._loop
        if self._connected and self._standing and loop:
            try:
                fut = asyncio.run_coroutine_threadsafe(
                    self._safe_shutdown(), loop)
                fut.result(timeout=5.0)
            except Exception as e:
                logger.warning("Shutdown cleanup failed: %s", e)
        if loop:
            loop.call_soon_threadsafe(loop.stop)
        if self._grpc_thread:
            self._grpc_thread.join(timeout=3.0)
        if loop:
            try:
                loop.close()
            except RuntimeError:
                logger.debug("connection: loop close error", exc_info=True)
        self._loop = None
        self.alive.publish(False)
        super().stop()

    # -- Port callbacks --

    def _on_cmd_vel(self, twist: Twist):
        self._last_cmd_time = time.time()
        self._cmd_vx = twist.linear.x
        self._cmd_vy = twist.linear.y
        self._cmd_wz = twist.angular.z

        if self._watchdog_triggered:
            self._watchdog_triggered = False
            logger.info("Watchdog cleared: cmd_vel resumed")

        if self._connected and self._standing:
            walk = self._twist_to_walk(self._cmd_vx, self._cmd_vy, self._cmd_wz)
            asyncio.run_coroutine_threadsafe(
                self._send_walk(walk), self._loop)

    def _on_stop(self, level: int):
        if level == 2 and self._loop:
            logger.warning("Hard stop → SitDown")
            asyncio.run_coroutine_threadsafe(self._sit_down(), self._loop)
        elif level == 1 and self._loop:
            logger.info("Soft stop → zero velocity")
            asyncio.run_coroutine_threadsafe(
                self._send_walk_zero(), self._loop)

    def _on_slam_odom(self, odom: Odometry):
        """Reset dead-reckoning position from SLAM to prevent drift accumulation."""
        now = time.time()
        if now - self._last_slam_reset >= self._slam_reset_interval:
            sx, sy = odom.x, odom.y
            if math.isfinite(sx) and math.isfinite(sy):
                self._pos_x = sx
                self._pos_y = sy
                self._last_slam_reset = now

    # -- Twist normalization --

    def _twist_to_walk(self, vx: float, vy: float, wz: float) -> tuple:
        """Convert m/s, rad/s → [-1,1] normalized Walk command."""
        nx = max(-1.0, min(1.0, vx / self._max_linear)) if self._max_linear > 0 else 0.0
        ny = max(-1.0, min(1.0, vy / self._max_linear)) if self._max_linear > 0 else 0.0
        nz = max(-1.0, min(1.0, wz / self._max_angular)) if self._max_angular > 0 else 0.0
        return (nx, ny, nz)

    # -- Odometry synthesis --

    def _publish_odometry(self):
        """Fuse IMU orientation + cmd_vel dead-reckoning → Odometry message."""
        from core.msgs.geometry import Pose

        now = time.time()
        if self._last_odom_time is not None:
            dt = now - self._last_odom_time
            if 0.0 < dt < 1.0:
                qx, qy, qz, qw = self._latest_quat
                yaw = math.atan2(
                    2.0 * (qw * qz + qx * qy),
                    1.0 - 2.0 * (qy * qy + qz * qz))
                self._pos_x += (self._cmd_vx * math.cos(yaw) - self._cmd_vy * math.sin(yaw)) * dt
                self._pos_y += (self._cmd_vx * math.sin(yaw) + self._cmd_vy * math.cos(yaw)) * dt
                if not math.isfinite(self._pos_x):
                    self._pos_x = 0.0
                if not math.isfinite(self._pos_y):
                    self._pos_y = 0.0
        self._last_odom_time = now

        qx, qy, qz, qw = self._latest_quat
        gx, gy, gz = self._latest_gyro

        odom = Odometry(
            pose=Pose(
                position=Vector3(self._pos_x, self._pos_y, 0.0),
                orientation=Quaternion(qx, qy, qz, qw),
            ),
            twist=Twist(
                linear=Vector3(self._cmd_vx, self._cmd_vy, 0.0),
                angular=Vector3(gx, gy, gz),
            ),
            ts=now,
            frame_id=NOVA_DOG_ODOM_FRAME_ID,
            child_frame_id=NOVA_DOG_BODY_FRAME_ID,
        )
        self.odometry.publish(odom)

    # -- Watchdog --

    def _watchdog_loop(self):
        """Control-rate watchdog: zero velocity on cmd_vel timeout, periodic odom publish."""
        odom_counter = 0
        while not self._shutdown:
            elapsed = time.time() - self._last_cmd_time
            if elapsed > self._watchdog_timeout:
                if not self._watchdog_triggered:
                    self._watchdog_triggered = True
                    logger.warning(
                        "WATCHDOG: No cmd_vel for %.0fms → zero velocity",
                        elapsed * 1000)
                if self._connected and self._standing and self._loop:
                    asyncio.run_coroutine_threadsafe(
                        self._send_walk_zero(), self._loop)

            odom_counter += 1
            if odom_counter >= self._odom_skip:
                odom_counter = 0
                self._publish_odometry()

            time.sleep(1.0 / self._control_rate)

    # -- gRPC async logic --

    def _run_async_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._async_main())

    async def _async_main(self):
        import random
        backoff = self._reconnect_interval
        max_backoff = 30.0
        consecutive_failures = 0
        while not self._shutdown:
            try:
                await self._connect_and_run()
                # Connection was live and then dropped — reset backoff
                backoff = self._reconnect_interval
                consecutive_failures = 0
            except Exception as e:
                self._connected = False
                self.alive.publish(False)
                consecutive_failures += 1
                # Exponential backoff with jitter
                jitter = random.uniform(0, backoff * 0.3)
                wait = min(backoff + jitter, max_backoff)
                logger.error(
                    "gRPC connection lost: %s. Reconnecting in %.1fs "
                    "(attempt %d)...", e, wait, consecutive_failures)
                await asyncio.sleep(wait)
                backoff = min(backoff * 2, max_backoff)

    async def _connect_and_run(self):
        try:
            import brainstem_api as dog_msg
            import grpc.aio as grpc_aio
        except ImportError:
            logger.error("grpc/brainstem_api not available — running in stub mode")
            self._connected = False
            self.alive.publish(False)
            await asyncio.sleep(999999)
            return

        addr = f"{self._dog_host}:{self._dog_port}"
        logger.info("Connecting to brainstem CMS at %s...", addr)

        async with grpc_aio.insecure_channel(addr) as channel:
            self._stub = dog_msg.RobotControlStub(channel)
            self._connected = True
            self.alive.publish(True)
            logger.info("Connected to brainstem CMS at %s", addr)

            if self._auto_enable:
                try:
                    await self._stub.Enable(dog_msg.Empty())
                    self._enabled = True
                    logger.info("Motors ENABLED")
                except Exception as e:
                    logger.error("Enable failed: %s", e)

            if self._auto_standup:
                for attempt in range(1, 4):
                    try:
                        await self._stub.StandUp(dog_msg.Empty())
                        self._standing = True
                        logger.info("Robot STANDING UP")
                        break
                    except Exception as e:
                        logger.error("StandUp attempt %d/3 failed: %s", attempt, e)
                        if attempt < 3:
                            await asyncio.sleep(1.0)
                else:
                    raise RuntimeError("StandUp failed after 3 attempts")

            await asyncio.gather(
                self._listen_imu(dog_msg),
                return_exceptions=True,
            )

    async def _listen_imu(self, dog_msg):
        """Subscribe to IMU data stream and update internal state."""
        async for imu in self._stub.ListenImu(dog_msg.Empty()):
            gx = float(imu.gyroscope.x) if math.isfinite(imu.gyroscope.x) else 0.0
            gy = float(imu.gyroscope.y) if math.isfinite(imu.gyroscope.y) else 0.0
            gz = float(imu.gyroscope.z) if math.isfinite(imu.gyroscope.z) else 0.0
            self._latest_gyro = (gx, gy, gz)

            qx, qy, qz, qw = (
                imu.quaternion.x, imu.quaternion.y,
                imu.quaternion.z, imu.quaternion.w)
            norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
            if norm > 1e-6:
                self._latest_quat = (qx/norm, qy/norm, qz/norm, qw/norm)

    async def _send_walk(self, vec: tuple):
        try:
            import brainstem_api as dog_msg
            await self._stub.Walk(dog_msg.Vector3(x=vec[0], y=vec[1], z=vec[2]))
        except Exception as e:
            logger.warning("Walk failed: %s", e)

    async def _send_walk_zero(self):
        await self._send_walk((0.0, 0.0, 0.0))

    async def _sit_down(self):
        try:
            import brainstem_api as dog_msg
            await self._send_walk_zero()
            await self._stub.SitDown(dog_msg.Empty())
            self._standing = False
            logger.info("Robot SITTING DOWN")
        except Exception as e:
            logger.error("SitDown failed: %s", e)

    async def _safe_shutdown(self):
        try:
            import brainstem_api as dog_msg
            await self._send_walk_zero()
            await self._stub.SitDown(dog_msg.Empty())
            await self._stub.Disable(dog_msg.Empty())
            logger.info("Safe shutdown complete")
        except Exception:
            logger.warning("Connection safe shutdown error", exc_info=True)

    # -- Health --

    def health(self) -> dict[str, Any]:
        stats = super().port_summary()
        stats["grpc"] = {
            "connected": self._connected,
            "standing": self._standing,
            "enabled": self._enabled,
            "watchdog": self._watchdog_triggered,
            "host": f"{self._dog_host}:{self._dog_port}",
        }
        return stats
