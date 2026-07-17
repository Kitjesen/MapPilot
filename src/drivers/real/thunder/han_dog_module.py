"""ThunderDriver — quadruped robot driver module (gRPC → brainstem CMS).

Ports:
  In:  cmd_vel      (Twist)       — velocity command → gRPC Walk()
       stop_signal  (int)         — 0=normal, 1=soft_stop, 2=hard_stop
       slam_odom    (Odometry)    — SLAM position reset (anti-drift)
  Out: odometry     (Odometry)    — IMU orientation + cmd_vel dead-reckoning
       alive        (bool)        — gRPC connection status

Usage::

    python lingtu.py thunder-nav --dog-host 192.168.66.190
"""

from __future__ import annotations

import asyncio
import logging
import math
import threading
import time
from typing import Any

from runtime.module import Module
from runtime.msgs.geometry import Quaternion, Twist, Vector3
from runtime.msgs.nav import Odometry
from runtime.registry import register
from runtime.runtime_interface import body_frame_id, odom_frame_id
from runtime.stream import In, Out

logger = logging.getLogger(__name__)
THUNDER_ODOM_FRAME_ID = odom_frame_id()
THUNDER_BODY_FRAME_ID = body_frame_id()


@register("driver", "thunder", priority=10, platforms={"aarch64"},
          description="Thunder quadruped via brainstem gRPC")
@register("driver_protocol", "grpc_brainstem", priority=10, platforms={"aarch64"},
          description="Brainstem gRPC protocol for Thunder quadruped")
class ThunderDriver(Module, layer=1):
    """Thunder quadruped robot driver.

    In:  cmd_vel → normalize to [-1,1] → gRPC Walk()
    Out: odometry ← IMU orientation + dead-reckoning position
    """

    # ── 端口声明 ──
    cmd_vel: In[Twist]
    stop_signal: In[int]
    slam_odom: In[Odometry]
    odometry: Out[Odometry]
    alive: Out[bool]
    robot_state: Out[dict]

    def __init__(
        self,
        dog_host: str = "127.0.0.1",
        dog_port: int = 13145,
        max_linear_speed: float = 1.0,
        max_angular_speed: float = 1.0,
        cmd_vel_timeout_ms: float = 200.0,
        control_rate: float = 50.0,
        auto_enable: bool = False,
        auto_standup: bool = False,
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

        # 内部状态
        self._cmd_lock = threading.Lock()
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0
        self._last_cmd_time = time.time()
        self._watchdog_triggered = False
        self._connected = False
        self._standing = False
        self._enabled = False
        self._shutdown = False
        self._emergency_stop = False
        self._battery_voltage = 0.0   # TODO: read from brainstem battery stream
        self._battery_soc = 0.0       # TODO: read from brainstem battery stream
        self._gait_type = "unknown"   # TODO: track gait type from brainstem

        # gRPC (在 asyncio 线程初始化)
        self._channel = None
        self._stub = None

        # 位置积分
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._last_odom_time: float | None = None
        self._last_slam_reset = time.time()

        # IMU 缓存
        self._latest_quat = (0.0, 0.0, 0.0, 1.0)  # x, y, z, w
        self._latest_gyro = (0.0, 0.0, 0.0)

        # asyncio
        self._loop: asyncio.AbstractEventLoop | None = None
        self._grpc_thread: threading.Thread | None = None

    # ── Brainstem auto-start ──

    def _check_brainstem(self) -> bool:
        """Check if brainstem gRPC server is reachable. Log warning if not."""
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(1)
        try:
            s.connect((self._dog_host, self._dog_port))
            logger.info("brainstem reachable at %s:%d", self._dog_host, self._dog_port)
            return True
        except (ConnectionRefusedError, OSError):
            logger.warning(
                "brainstem not reachable at %s:%d — "
                "ensure brainstem systemd service is running: "
                "sudo systemctl start brainstem",
                self._dog_host, self._dog_port,
            )
            return False
        finally:
            s.close()

    # ── 生命周期 ──

    def setup(self) -> None:
        """注册端口回调。"""
        self.cmd_vel.subscribe(self._on_cmd_vel)
        self.stop_signal.subscribe(self._on_stop)
        self.slam_odom.subscribe(self._on_slam_odom)

    def start(self):
        """启动 gRPC 连接 + 看门狗。brainstem 应由 systemd 独立管理。"""
        super().start()
        self._shutdown = False

        # 启动 asyncio 事件循环 (独立线程)
        self._loop = asyncio.new_event_loop()
        self._grpc_thread = threading.Thread(
            target=self._run_async_loop, daemon=True)
        self._grpc_thread.start()

        # 看门狗线程
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, daemon=True)
        self._watchdog_thread.start()

        logger.info(
            "HanDogModule started → %s:%d (max_lin=%.1f, max_ang=%.1f)",
            self._dog_host, self._dog_port, self._max_linear, self._max_angular,
        )
        self._publish_robot_state()

    def stop(self):
        """安全关机：零速 → 坐下 → 断开。"""
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
            loop.call_soon_threadsafe(self._cancel_loop_tasks, loop)
        if self._grpc_thread:
            self._grpc_thread.join(timeout=3.0)
        if self._watchdog_thread:
            self._watchdog_thread.join(timeout=1.0)
        if loop:
            try:
                loop.close()
            except RuntimeError:
                logger.debug("han_dog: loop close error", exc_info=True)
        self._loop = None
        self.alive.publish(False)
        self._publish_robot_state()
        super().stop()

    # ── 端口回调 ──

    def _on_cmd_vel(self, twist: Twist):
        """收到速度指令 → 缓存 + 异步发送 Walk。"""
        self._last_cmd_time = time.time()
        with self._cmd_lock:
            self._cmd_vx = twist.linear.x
            self._cmd_vy = twist.linear.y
            self._cmd_wz = twist.angular.z

        if self._watchdog_triggered:
            self._watchdog_triggered = False
            logger.info("Watchdog cleared: cmd_vel resumed")

        if self._connected and self._standing:
            with self._cmd_lock:
                walk = self._twist_to_walk(self._cmd_vx, self._cmd_vy, self._cmd_wz)
            asyncio.run_coroutine_threadsafe(
                self._send_walk(walk), self._loop)

        self._publish_robot_state()

    def _on_stop(self, level: int):
        """收到停止信号。"""
        if level == 2 and self._loop:
            logger.warning("Hard stop → SitDown")
            asyncio.run_coroutine_threadsafe(self._sit_down(), self._loop)
        elif level == 1 and self._loop:
            logger.info("Soft stop → zero velocity")
            asyncio.run_coroutine_threadsafe(
                self._send_walk_zero(), self._loop)

    def _on_slam_odom(self, odom: Odometry):
        """SLAM 位置重置（防积分漂移）。"""
        now = time.time()
        if now - self._last_slam_reset >= self._slam_reset_interval:
            sx, sy = odom.x, odom.y
            if math.isfinite(sx) and math.isfinite(sy):
                self._pos_x = sx
                self._pos_y = sy
                self._last_slam_reset = now

    # ── Twist → Walk 转换 ──

    def _twist_to_walk(self, vx: float, vy: float, wz: float) -> tuple:
        """m/s, rad/s → [-1,1] 归一化 Walk 指令。"""
        nx = max(-1.0, min(1.0, vx / self._max_linear)) if self._max_linear > 0 else 0.0
        ny = max(-1.0, min(1.0, vy / self._max_linear)) if self._max_linear > 0 else 0.0
        nz = max(-1.0, min(1.0, wz / self._max_angular)) if self._max_angular > 0 else 0.0
        return (nx, ny, nz)

    # ── 里程计合成 ──

    def _publish_odometry(self):
        """IMU 姿态 + cmd_vel 位置积分 → Odometry 消息。"""
        from runtime.msgs.geometry import Pose

        now = time.time()
        with self._cmd_lock:
            cmd_vx = self._cmd_vx
            cmd_vy = self._cmd_vy

        if self._last_odom_time is not None:
            dt = now - self._last_odom_time
            if 0.0 < dt < 1.0:
                qx, qy, qz, qw = self._latest_quat
                yaw = math.atan2(
                    2.0 * (qw * qz + qx * qy),
                    1.0 - 2.0 * (qy * qy + qz * qz))
                self._pos_x += (cmd_vx * math.cos(yaw) - cmd_vy * math.sin(yaw)) * dt
                self._pos_y += (cmd_vx * math.sin(yaw) + cmd_vy * math.cos(yaw)) * dt
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
                linear=Vector3(cmd_vx, cmd_vy, 0.0),
                angular=Vector3(gx, gy, gz),
            ),
            ts=now,
            frame_id=THUNDER_ODOM_FRAME_ID,
            child_frame_id=THUNDER_BODY_FRAME_ID,
        )
        self.odometry.publish(odom)

    # ── 机器人状态发布 ──

    def _build_robot_state(self) -> dict:
        """Assemble operational robot state from internal fields."""
        return {
            "standing": self._standing,
            "enabled": self._enabled,
            "emergency": self._emergency_stop or (not self._connected),
            "connected": self._connected,
            "battery_voltage": self._battery_voltage,
            "battery_soc": self._battery_soc,
            "current_gait": self._gait_type,
            "timestamp": time.time(),
        }

    def _publish_robot_state(self):
        """Build and publish the current robot state to the robot_state port."""
        self.robot_state.publish(self._build_robot_state())

    # ── 看门狗 ──

    def _watchdog_loop(self):
        """50Hz 看门狗循环。"""
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

    # ── gRPC 异步逻辑 ──

    def _run_async_loop(self):
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._async_main())
        except asyncio.CancelledError:
            pass

    @staticmethod
    def _cancel_loop_tasks(loop: asyncio.AbstractEventLoop) -> None:
        for task in asyncio.all_tasks(loop):
            task.cancel()

    async def _async_main(self):
        while not self._shutdown:
            try:
                await self._connect_and_run()
            except Exception as e:
                self._connected = False
                self.alive.publish(False)
                logger.error(
                    "gRPC connection lost: %s. Reconnecting in %.0fs...",
                    e, self._reconnect_interval)
                await asyncio.sleep(self._reconnect_interval)

    async def _connect_and_run(self):
        try:
            import brainstem_api as dog_msg
            import grpc.aio as grpc_aio
        except ImportError:
            logger.error("grpc/brainstem_api not available — running in stub mode")
            self._connected = False
            self.alive.publish(False)
            await asyncio.sleep(999999)  # block forever in stub mode
            return

        addr = f"{self._dog_host}:{self._dog_port}"
        logger.info("Connecting to Han Dog CMS at %s...", addr)

        async with grpc_aio.insecure_channel(addr) as channel:
            self._stub = dog_msg.RobotControlStub(channel)
            self._connected = True
            self.alive.publish(True)
            logger.info("Connected to Han Dog CMS at %s", addr)

            if self._auto_enable:
                try:
                    await self._stub.Enable(dog_msg.Empty())
                    self._enabled = True
                    logger.info("Dog motors ENABLED")
                    self._publish_robot_state()
                except Exception as e:
                    logger.error("Enable failed: %s", e)

            if self._auto_standup:
                for attempt in range(1, 4):
                    try:
                        await self._stub.StandUp(dog_msg.Empty())
                        self._standing = True
                        logger.info("Dog STANDING UP")
                        self._publish_robot_state()
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
        """订阅 IMU 数据流 → 更新内部状态。"""
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
            logger.info("Dog SITTING DOWN")
            self._publish_robot_state()
        except Exception as e:
            logger.error("SitDown failed: %s", e)

    async def _safe_shutdown(self):
        try:
            import brainstem_api as dog_msg
            await self._send_walk_zero()
            await self._stub.SitDown(dog_msg.Empty())
            await self._stub.Disable(dog_msg.Empty())
            self._enabled = False
            self._standing = False
            logger.info("Safe shutdown complete")
            self._publish_robot_state()
        except Exception:
            logger.warning("HanDog safe shutdown error", exc_info=True)

    # ── 健康报告 ──

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


# Backward-compat alias
HanDogModule = ThunderDriver
