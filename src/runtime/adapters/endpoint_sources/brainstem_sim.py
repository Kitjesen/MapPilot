"""Brainstem-compatible simulation source for endpoint closed-loop checks."""

from __future__ import annotations

import asyncio
import logging
import math
import os
import threading
import time
from collections import Counter
from collections.abc import Mapping
from typing import Any

from runtime.msgs.geometry import Pose, Quaternion, Twist, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import Imu, PointCloud2
from runtime.runtime_interface import FRAMES, TOPICS, body_frame_id, odom_frame_id

logger = logging.getLogger(__name__)


class BrainstemSimSource:
    """Simulate the Brainstem gRPC receiver and feed odometry back to LingTu.

    This source is meant for field-runtime closed-loop validation:

    LingTu navigation -> endpoint cmd_vel -> ThunderBrainstemSource -> gRPC Walk
    -> this source -> simulated odometry/clouds -> LingTu navigation.

    MuJoCo is used only when explicitly requested and available. The default
    kinematic backend keeps the source usable on S100P without ROS or MuJoCo.
    """

    name = "brainstem_sim"

    def __init__(
        self,
        *,
        mode: str = "kinematic",
        host: str = "127.0.0.1",
        port: int = 13145,
        start_grpc: bool = True,
        require_sdk: bool | None = None,
        direct_cmd_vel: bool = False,
        publish_rate: float = 20.0,
        cmd_timeout_sec: float = 0.3,
        max_linear_speed: float = 1.0,
        max_angular_speed: float = 1.0,
        frame_id: str = odom_frame_id(),
        body_frame: str = body_frame_id(),
        cloud_points: int = 64,
        mujoco_world: str = "building_scene",
        mujoco_drive_mode: str = "kinematic",
        mujoco_fallback_kinematic: bool = True,
        run_background: bool = True,
    ) -> None:
        self._mode = str(mode or "kinematic").strip().lower()
        if self._mode not in {"kinematic", "mujoco"}:
            raise ValueError(f"Unsupported brainstem sim mode: {mode}")
        self._host = str(host or "127.0.0.1")
        self._port = int(port)
        self._start_grpc = bool(start_grpc)
        self._require_sdk = self._start_grpc if require_sdk is None else bool(require_sdk)
        self._direct_cmd_vel = bool(direct_cmd_vel)
        self._period = 1.0 / max(1.0, float(publish_rate))
        self._cmd_timeout = max(0.02, float(cmd_timeout_sec))
        self._max_linear = max(0.0, float(max_linear_speed))
        self._max_angular = max(0.0, float(max_angular_speed))
        self._frame_id = str(frame_id or odom_frame_id())
        self._body_frame = str(body_frame or body_frame_id())
        self._cloud_points = max(4, int(cloud_points))
        self._mujoco_world = str(mujoco_world or "building_scene")
        self._mujoco_drive_mode = str(mujoco_drive_mode or "kinematic").strip().lower()
        self._mujoco_fallback_kinematic = bool(mujoco_fallback_kinematic)
        self._run_background = bool(run_background)

        self._service: Any | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

        self._grpc_loop: asyncio.AbstractEventLoop | None = None
        self._grpc_server: Any | None = None
        self._grpc_thread: threading.Thread | None = None
        self._grpc_ready = threading.Event()
        self._dog_msg: Any | None = None
        self._grpc_aio: Any | None = None

        self._engine: Any | None = None
        self._mujoco_adapter: Any | None = None
        self._backend = "kinematic"
        self._backend_error = ""
        self._grpc_error = ""

        self._started = False
        self._grpc_started = False
        self._enabled = False
        self._standing = False
        self._speed_mode_request: Any | None = None
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._z = 0.45
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0
        self._last_cmd_ts = 0.0
        self._last_publish_ts = 0.0
        self._last_walk: tuple[float, float, float] | None = None
        self._distance_m = 0.0

        self._published: Counter[str] = Counter()
        self._received: Counter[str] = Counter()
        self._grpc_calls: Counter[str] = Counter()
        self._errors: Counter[str] = Counter()

    def start(self, service: Any) -> None:
        if self._started:
            return
        self._service = service
        self._stop.clear()
        self._setup_backend()
        self._start_grpc_server()
        self._started = True
        if self._run_background:
            self._thread = threading.Thread(
                target=self._run_loop,
                name="lingtu-brainstem-sim-source",
                daemon=True,
            )
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = None
        self._stop_grpc_server()
        if self._engine is not None:
            close = getattr(self._engine, "close", None)
            if callable(close):
                try:
                    close()
                except Exception:
                    logger.debug("Brainstem sim MuJoCo close failed", exc_info=True)
        self._engine = None
        self._mujoco_adapter = None
        self._service = None
        self._started = False

    def on_lingtu_message(self, event: Any) -> None:
        self._received[event.topic] += 1
        if event.topic != TOPICS.cmd_vel or not self._direct_cmd_vel:
            return
        vx, vy, wz = _twist_components(event.message)
        self._set_real_command(vx, vy, wz, walk=None)

    def step_once(self, dt: float | None = None) -> None:
        """Advance one simulation sample and publish feedback.

        Normal deployments use the background loop. Tests and small gates can
        call this directly to avoid timing-sensitive sleeps.
        """

        self._advance_and_publish(self._period if dt is None else float(dt))

    def health(self) -> Mapping[str, Any]:
        with self._lock:
            cmd = (self._cmd_vx, self._cmd_vy, self._cmd_wz)
            pose = (self._x, self._y, self._yaw)
            last_walk = self._last_walk
            last_cmd_ts = self._last_cmd_ts
        return {
            "name": self.name,
            "hardware": False,
            "role": "brainstem_grpc_sim_feedback",
            "started": self._started,
            "mode": self._mode,
            "backend": self._backend,
            "backend_error": self._backend_error,
            "grpc_enabled": self._start_grpc,
            "grpc_started": self._grpc_started,
            "grpc_host": f"{self._host}:{self._port}",
            "grpc_error": self._grpc_error,
            "direct_cmd_vel": self._direct_cmd_vel,
            "run_background": self._run_background,
            "enabled": self._enabled,
            "standing": self._standing,
            "cmd_timeout_sec": self._cmd_timeout,
            "last_walk": list(last_walk) if last_walk is not None else None,
            "last_command": {"vx": cmd[0], "vy": cmd[1], "wz": cmd[2]},
            "last_cmd_age_sec": max(0.0, time.time() - last_cmd_ts) if last_cmd_ts else None,
            "pose": {"x": pose[0], "y": pose[1], "yaw": pose[2]},
            "distance_m": self._distance_m,
            "published": dict(self._published),
            "received": dict(self._received),
            "grpc_calls": dict(self._grpc_calls),
            "errors": dict(self._errors),
            "last_publish_ts": self._last_publish_ts,
        }

    def _setup_backend(self) -> None:
        self._backend = "kinematic"
        self._backend_error = ""
        if self._mode != "mujoco":
            return
        self._backend_error = (
            "MuJoCo backend is not loaded from runtime.adapters; use the sim driver "
            "stack for MuJoCo-backed endpoint tests"
        )
        self._errors["mujoco_setup"] += 1
        if not self._mujoco_fallback_kinematic:
            raise RuntimeError(self._backend_error)
        logger.warning("Brainstem sim falling back to kinematic backend: %s", self._backend_error)

    def _start_grpc_server(self) -> None:
        if not self._start_grpc:
            return
        try:
            import brainstem_api as dog_msg
            import grpc.aio as grpc_aio
        except ImportError as exc:
            self._grpc_error = str(exc)
            self._errors["grpc_import"] += 1
            if self._require_sdk:
                raise RuntimeError(
                    "brainstem_sim gRPC mode requires brainstem_api and grpc.aio; "
                    "set LINGTU_BRAINSTEM_SIM_START_GRPC=0 for direct kinematic smoke tests"
                ) from exc
            return

        self._dog_msg = dog_msg
        self._grpc_aio = grpc_aio
        self._grpc_ready.clear()
        self._grpc_thread = threading.Thread(
            target=self._run_grpc_server,
            name="lingtu-brainstem-sim-grpc",
            daemon=True,
        )
        self._grpc_thread.start()
        if not self._grpc_ready.wait(timeout=3.0):
            self._errors["grpc_start_timeout"] += 1
            raise RuntimeError("brainstem_sim gRPC server did not become ready")

    def _stop_grpc_server(self) -> None:
        server = self._grpc_server
        loop = self._grpc_loop
        if server is not None and loop is not None:
            future = asyncio.run_coroutine_threadsafe(server.stop(0), loop)
            try:
                future.result(timeout=2.0)
            except Exception:
                logger.debug("Brainstem sim gRPC stop failed", exc_info=True)
        if self._grpc_thread is not None and self._grpc_thread.is_alive():
            self._grpc_thread.join(timeout=2.0)
        self._grpc_thread = None
        self._grpc_loop = None
        self._grpc_server = None
        self._grpc_started = False

    def _run_grpc_server(self) -> None:
        loop = asyncio.new_event_loop()
        self._grpc_loop = loop
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._serve_grpc())
        except Exception as exc:
            self._grpc_error = str(exc)
            self._errors[type(exc).__name__] += 1
            self._grpc_ready.set()
            logger.exception("Brainstem sim gRPC server failed")
        finally:
            try:
                loop.close()
            finally:
                self._grpc_loop = None

    async def _serve_grpc(self) -> None:
        assert self._grpc_aio is not None
        assert self._dog_msg is not None
        server = self._grpc_aio.server()
        self._dog_msg.add_RobotControlServicer_to_server(
            _BrainstemSimServicer(self, self._dog_msg),
            server,
        )
        bound = server.add_insecure_port(f"{self._host}:{self._port}")
        if int(bound) <= 0:
            raise RuntimeError(f"failed to bind brainstem_sim gRPC on {self._host}:{self._port}")
        self._grpc_server = server
        await server.start()
        self._grpc_started = True
        self._grpc_error = ""
        self._grpc_ready.set()
        await server.wait_for_termination()

    def _run_loop(self) -> None:
        last = time.monotonic()
        self._publish_snapshot(0.0)
        while not self._stop.is_set():
            now = time.monotonic()
            dt = max(0.0, min(0.2, now - last))
            last = now
            self._advance_and_publish(dt)
            elapsed = time.monotonic() - now
            self._stop.wait(max(0.0, self._period - elapsed))

    def _advance_and_publish(self, dt: float) -> None:
        vx, vy, wz = self._active_command()
        if self._backend == "mujoco" and self._engine is not None:
            self._advance_mujoco(vx, vy, wz)
            return
        self._advance_kinematic(vx, vy, wz, dt)
        self._publish_snapshot(dt)

    def _advance_mujoco(self, vx: float, vy: float, wz: float) -> None:
        try:
            from sim.engine.core.engine import VelocityCommand

            state = self._engine.step(VelocityCommand(vx, vy, wz))
            adapter = self._mujoco_adapter
            if adapter is None:
                return
            frame = adapter.sensor_frame_from_state(state)
            self._publish_frame(
                odometry=frame.odometry,
                imu=frame.imu,
                registered_cloud=frame.lidar_cloud,
                map_cloud=frame.map_cloud,
                backend="mujoco",
            )
            if frame.odometry is not None:
                with self._lock:
                    self._x = frame.odometry.x
                    self._y = frame.odometry.y
                    self._z = frame.odometry.z
                    self._yaw = frame.odometry.yaw
        except Exception as exc:
            self._backend_error = str(exc)
            self._errors["mujoco_step"] += 1
            logger.debug("Brainstem sim MuJoCo step failed", exc_info=True)

    def _advance_kinematic(self, vx: float, vy: float, wz: float, dt: float) -> None:
        dt = max(0.0, float(dt))
        with self._lock:
            c, s = math.cos(self._yaw), math.sin(self._yaw)
            dx = (vx * c - vy * s) * dt
            dy = (vx * s + vy * c) * dt
            self._x += dx
            self._y += dy
            self._yaw = _wrap_pi(self._yaw + wz * dt)
            self._distance_m += math.hypot(dx, dy)

    def _publish_snapshot(self, dt: float) -> None:
        del dt
        now = time.time()
        with self._lock:
            x, y, z, yaw = self._x, self._y, self._z, self._yaw
            vx, vy, wz = self._cmd_vx, self._cmd_vy, self._cmd_wz
        quat = Quaternion.from_yaw(yaw)
        odom = Odometry(
            pose=Pose(position=Vector3(x, y, z), orientation=quat),
            twist=Twist(
                linear=Vector3(vx, vy, 0.0),
                angular=Vector3(0.0, 0.0, wz),
            ),
            ts=now,
            frame_id=self._frame_id,
            child_frame_id=self._body_frame,
        )
        imu = Imu(
            orientation=quat,
            angular_velocity=Vector3(0.0, 0.0, wz),
            linear_acceleration=Vector3(0.0, 0.0, 9.81),
            ts=now,
            frame_id=FRAMES.lidar,
        )
        map_cloud, registered_cloud = self._cloud_pair(x, y, yaw, now)
        self._publish_frame(
            odometry=odom,
            imu=imu,
            registered_cloud=registered_cloud,
            map_cloud=map_cloud,
            backend="kinematic",
        )

    def _publish_frame(
        self,
        *,
        odometry: Odometry | None,
        imu: Imu | None,
        registered_cloud: PointCloud2 | None,
        map_cloud: PointCloud2 | None,
        backend: str,
    ) -> None:
        service = self._service
        if service is None:
            return
        localization_health = {
            "state": "TRACKING",
            "quality": 1.0,
            "source": self.name,
            "backend": backend,
            "grpc_started": self._grpc_started,
            "direct_cmd_vel": self._direct_cmd_vel,
        }
        try:
            count = service.publish_sensor_snapshot(imu=imu)
            count += service.publish_localization_snapshot(
                odometry=odometry,
                registered_cloud=registered_cloud,
                map_cloud=map_cloud,
                localization_health=localization_health,
                localization_quality=1.0,
            )
        except Exception:
            self._errors["publish"] += 1
            logger.debug("Brainstem sim publish failed", exc_info=True)
            return
        self._published["messages"] += count
        for topic, msg in (
            (TOPICS.imu, imu),
            (TOPICS.odometry, odometry),
            (TOPICS.registered_cloud, registered_cloud),
            (TOPICS.map_cloud, map_cloud),
            (TOPICS.localization_health, localization_health),
            (TOPICS.localization_quality, 1.0),
        ):
            if msg is not None:
                self._published[topic] += 1
        self._last_publish_ts = time.time()

    def _cloud_pair(self, x: float, y: float, yaw: float, ts: float) -> tuple[PointCloud2, PointCloud2]:
        angles = np.linspace(0.0, 2.0 * math.pi, self._cloud_points, endpoint=False)
        radius = np.where(np.arange(self._cloud_points) % 2 == 0, 4.0, 6.0)
        z = np.zeros_like(angles)
        world = np.stack(
            [
                x + radius * np.cos(angles),
                y + radius * np.sin(angles),
                z,
                np.ones_like(angles),
            ],
            axis=1,
        ).astype(np.float32)
        c, s = math.cos(yaw), math.sin(yaw)
        rel_x = world[:, 0] - x
        rel_y = world[:, 1] - y
        body = world.copy()
        body[:, 0] = rel_x * c + rel_y * s
        body[:, 1] = -rel_x * s + rel_y * c
        return (
            PointCloud2(points=world, frame_id=self._frame_id, ts=ts),
            PointCloud2(points=body, frame_id=self._body_frame, ts=ts),
        )

    def _active_command(self) -> tuple[float, float, float]:
        now = time.time()
        with self._lock:
            if not self._last_cmd_ts or now - self._last_cmd_ts > self._cmd_timeout:
                self._cmd_vx = 0.0
                self._cmd_vy = 0.0
                self._cmd_wz = 0.0
            return self._cmd_vx, self._cmd_vy, self._cmd_wz

    def _set_walk(self, walk: tuple[float, float, float]) -> None:
        vx = _clamp(float(walk[0]), -1.0, 1.0) * self._max_linear
        vy = _clamp(float(walk[1]), -1.0, 1.0) * self._max_linear
        wz = _clamp(float(walk[2]), -1.0, 1.0) * self._max_angular
        self._set_real_command(vx, vy, wz, walk=walk)

    def _set_real_command(
        self,
        vx: float,
        vy: float,
        wz: float,
        *,
        walk: tuple[float, float, float] | None,
    ) -> None:
        with self._lock:
            self._cmd_vx = _finite(vx)
            self._cmd_vy = _finite(vy)
            self._cmd_wz = _finite(wz)
            self._last_cmd_ts = time.time()
            if walk is not None:
                self._last_walk = tuple(_finite(v) for v in walk)


class _BrainstemSimServicer:
    def __init__(self, owner: BrainstemSimSource, dog_msg: Any) -> None:
        self._owner = owner
        self._dog_msg = dog_msg

    async def Enable(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["Enable"] += 1
        self._owner._enabled = True
        return self._dog_msg.Empty()

    async def Disable(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["Disable"] += 1
        self._owner._enabled = False
        return self._dog_msg.Empty()

    async def StandUp(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["StandUp"] += 1
        self._owner._standing = True
        return self._dog_msg.Empty()

    async def SitDown(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["SitDown"] += 1
        self._owner._standing = False
        return self._dog_msg.Empty()

    async def Walk(self, request: Any, context: Any) -> Any:
        del context
        walk = (
            _finite(getattr(request, "x", 0.0)),
            _finite(getattr(request, "y", 0.0)),
            _finite(getattr(request, "z", 0.0)),
        )
        self._owner._grpc_calls["Walk"] += 1
        self._owner._set_walk(walk)
        return self._dog_msg.Empty()

    async def Tick(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["Tick"] += 1
        return self._dog_msg.History()

    async def Step(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["Step"] += 1
        return self._dog_msg.Empty()

    async def SetSpeedMode(self, request: Any, context: Any) -> Any:
        del context
        self._owner._grpc_calls["SetSpeedMode"] += 1
        self._owner._speed_mode_request = request
        return self._dog_msg.Empty()

    async def GetSpeedMode(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["GetSpeedMode"] += 1
        return self._owner._speed_mode_request or self._dog_msg.SpeedModeRequest()

    async def GetStartTime(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["GetStartTime"] += 1
        return self._dog_msg.Timestamp()

    async def GetCmsState(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["GetCmsState"] += 1
        return self._dog_msg.CmsState()

    async def ListenHistory(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["ListenHistory"] += 1
        if False:
            yield self._dog_msg.History()

    async def ListenCmsState(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["ListenCmsState"] += 1
        if False:
            yield self._dog_msg.CmsState()

    async def ListenImu(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["ListenImu"] += 1
        if False:
            yield self._dog_msg.Imu()

    async def ListenJoint(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["ListenJoint"] += 1
        if False:
            yield self._dog_msg.Joint()

    async def GetParams(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["GetParams"] += 1
        return self._dog_msg.Params()

    async def SwitchProfile(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["SwitchProfile"] += 1
        return self._dog_msg.ProfileInfo()

    async def GetProfile(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["GetProfile"] += 1
        return self._dog_msg.ProfileInfo()

    async def PlayGesture(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["PlayGesture"] += 1
        return self._dog_msg.Empty()

    async def ListGestures(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["ListGestures"] += 1
        return self._dog_msg.GestureList()

    async def GetVoltage(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["GetVoltage"] += 1
        return self._dog_msg.Voltage()

    async def GetMotorStatus(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["GetMotorStatus"] += 1
        return self._dog_msg.MotorStatusResponse()

    async def ClearMotorFault(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["ClearMotorFault"] += 1
        return self._dog_msg.Empty()

    async def SetZero(self, request: Any, context: Any) -> Any:
        del request, context
        self._owner._grpc_calls["SetZero"] += 1
        return self._dog_msg.Empty()


def create(**overrides: Any) -> BrainstemSimSource:
    values = {
        "mode": _env_str("LINGTU_BRAINSTEM_SIM_MODE", "kinematic"),
        "host": _env_str("LINGTU_BRAINSTEM_SIM_HOST", "127.0.0.1"),
        "port": _env_int(
            "LINGTU_BRAINSTEM_SIM_PORT",
            _env_int("LINGTU_BRAINSTEM_PORT", _env_int("LINGTU_DOG_PORT", 13145)),
        ),
        "start_grpc": _env_bool("LINGTU_BRAINSTEM_SIM_START_GRPC", True),
        "direct_cmd_vel": _env_bool("LINGTU_BRAINSTEM_SIM_DIRECT_CMD_VEL", False),
        "publish_rate": _env_float("LINGTU_BRAINSTEM_SIM_RATE", 20.0),
        "cmd_timeout_sec": _env_float(
            "LINGTU_BRAINSTEM_SIM_CMD_TIMEOUT_SEC",
            _env_float("LINGTU_BRAINSTEM_CMD_TIMEOUT_MS", 300.0) / 1000.0,
        ),
        "max_linear_speed": _env_float(
            "LINGTU_BRAINSTEM_SIM_MAX_LINEAR",
            _env_float("LINGTU_BRAINSTEM_MAX_LINEAR", _env_float("LINGTU_MAX_LINEAR_SPEED", 1.0)),
        ),
        "max_angular_speed": _env_float(
            "LINGTU_BRAINSTEM_SIM_MAX_ANGULAR",
            _env_float("LINGTU_BRAINSTEM_MAX_ANGULAR", _env_float("LINGTU_MAX_ANGULAR_SPEED", 1.0)),
        ),
        "mujoco_world": _env_str("LINGTU_BRAINSTEM_SIM_MUJOCO_WORLD", "building_scene"),
        "mujoco_drive_mode": _env_str("LINGTU_BRAINSTEM_SIM_MUJOCO_DRIVE_MODE", "kinematic"),
        "mujoco_fallback_kinematic": _env_bool("LINGTU_BRAINSTEM_SIM_MUJOCO_FALLBACK", True),
        "run_background": _env_bool("LINGTU_BRAINSTEM_SIM_BACKGROUND", True),
    }
    if os.getenv("LINGTU_BRAINSTEM_SIM_REQUIRE_SDK") not in (None, ""):
        values["require_sdk"] = _env_bool("LINGTU_BRAINSTEM_SIM_REQUIRE_SDK", True)
    values.update(overrides)
    return BrainstemSimSource(**values)


def _twist_components(message: Any) -> tuple[float, float, float]:
    if isinstance(message, Twist):
        return (
            _finite(message.linear.x),
            _finite(message.linear.y),
            _finite(message.angular.z),
        )
    linear = getattr(message, "linear", None)
    angular = getattr(message, "angular", None)
    if isinstance(message, Mapping):
        linear = message.get("linear", {})
        angular = message.get("angular", {})
        return (
            _finite(getattr(linear, "get", lambda key, default=0.0: default)("x", message.get("linear_x", 0.0))),
            _finite(getattr(linear, "get", lambda key, default=0.0: default)("y", message.get("linear_y", 0.0))),
            _finite(getattr(angular, "get", lambda key, default=0.0: default)("z", message.get("angular_z", 0.0))),
        )
    return (
        _finite(getattr(linear, "x", 0.0)),
        _finite(getattr(linear, "y", 0.0)),
        _finite(getattr(angular, "z", 0.0)),
    )


def _finite(value: Any) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    return number if math.isfinite(number) else 0.0


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _wrap_pi(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _env_str(name: str, default: str) -> str:
    value = os.getenv(name)
    return default if value in (None, "") else value


def _env_int(name: str, default: int) -> int:
    value = os.getenv(name)
    if value in (None, ""):
        return int(default)
    try:
        return int(value)
    except ValueError:
        return int(default)


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    if value in (None, ""):
        return float(default)
    try:
        return float(value)
    except ValueError:
        return float(default)


def _env_bool(name: str, default: bool) -> bool:
    value = os.getenv(name)
    if value in (None, ""):
        return bool(default)
    return value.strip().lower() in {"1", "true", "yes", "on"}
