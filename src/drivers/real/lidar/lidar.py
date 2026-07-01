"""Lidar - Livox MID-360 stream interface.

Consumes an existing Livox DDS stream and delivers NumPy point clouds. Native
driver process startup is an explicit compatibility option.

Usage::

    from drivers.real.lidar import Lidar

    # connect + callback
    lidar = Lidar()
    lidar.connect("192.168.1.115")      # subscribe only; driver is external
    lidar.on_cloud(lambda pts: print(pts.shape))

    # polling
    cloud = lidar.wait_for_cloud()          # numpy (N, 4) x,y,z,intensity
    imu   = lidar.get_imu()                 # runtime.msgs.sensor.Imu

    # health
    print(lidar.health)                     # LidarHealth(fps=10.0, frames=42, ...)
    print(lidar.state)                      # LidarState.CONNECTED

    lidar.disconnect()

    # Context manager
    with Lidar("192.168.1.115") as lidar:
        cloud = lidar.wait_for_cloud()
"""

from __future__ import annotations

import enum
import logging
import threading
import time
from collections.abc import Callable
from copy import deepcopy
from dataclasses import dataclass
from typing import Any

from runtime.msgs.numpy_compat import np
from runtime.runtime_interface import TOPICS

from .dds_adapter import LivoxDdsAdapter
from .frame_stream import LidarFrameStream

logger = logging.getLogger(__name__)


# State machine


class LidarState(enum.Enum):
    """Connection lifecycle states."""
    DISCONNECTED = "disconnected"
    CONNECTING   = "connecting"
    CONNECTED    = "connected"
    ERROR        = "error"


# Health metrics


@dataclass
class LidarHealth:
    """Observable health metrics - updated every frame."""
    state: LidarState = LidarState.DISCONNECTED
    ip: str = ""
    fps: float = 0.0
    total_frames: int = 0
    total_points: int = 0
    last_frame_time: float = 0.0
    last_frame_points: int = 0
    uptime_s: float = 0.0
    driver_pid: int | None = None
    driver_restarts: int = 0
    last_error: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "state": self.state.value,
            "ip": self.ip,
            "fps": round(self.fps, 1),
            "total_frames": self.total_frames,
            "total_points": self.total_points,
            "last_frame_points": self.last_frame_points,
            "uptime_s": round(self.uptime_s, 1),
            "driver_pid": self.driver_pid,
            "driver_restarts": self.driver_restarts,
            "last_error": self.last_error,
        }

# Main class


class Lidar:
    """Livox MID-360 LiDAR - connect, stream, disconnect.

    Under the hood:
    1. Optionally launches ``livox_ros_driver2_node`` only when
       ``start_driver=True`` is passed.
    2. Subscribes to canonical ``/lidar/raw_frame`` and ``/imu/raw``
       via lightweight cyclonedds - no rclpy required.
    3. Converts Livox frames to numpy (N, 4) and delivers via callback or poll.

    The IP given to :meth:`connect` overrides ``config/robot_config.yaml``
    for this session only - no file modification.
    """

    def __init__(
        self,
        ip: str | None = None,
        scan_topic: str = TOPICS.lidar_scan,
        imu_topic: str = TOPICS.imu,
        start_driver: bool = False,
    ):
        self._ip = ip
        self._scan_topic = scan_topic
        self._imu_topic = imu_topic
        self._start_driver = bool(start_driver)

        # State
        self._state = LidarState.DISCONNECTED
        self._state_lock = threading.Lock()

        # Native driver
        self._native = None

        # Data/callback state is pure Python so non-ROS adapters can reuse it.
        self._frames = LidarFrameStream()

        # DDS bridge
        self._dds_adapter = LivoxDdsAdapter(
            scan_topic=scan_topic,
            imu_topic=imu_topic,
            frames=self._frames,
        )
        self._dds = None

        # Health
        self._health = LidarHealth()
        self._connect_time: float = 0.0

    # Public API

    def connect(self, ip: str | None = None) -> Lidar:
        """Start the LiDAR driver and begin streaming.

        Args:
            ip: LiDAR IP address (e.g. ``"192.168.1.115"``).
                Overrides constructor IP and robot_config.yaml.
                Falls back to constructor IP -> config file -> default.

        Returns:
            self - for chaining.

        Raises:
            RuntimeError: If already connected.
        """
        if self._state == LidarState.CONNECTED:
            logger.warning("Lidar already connected at %s", self._ip)
            return self

        if ip:
            self._ip = ip

        self._set_state(LidarState.CONNECTING)
        self._health.ip = self._ip or ""

        try:
            cfg = self._build_config()
            self._health.ip = cfg.lidar.lidar_ip

            # 1) Optional compatibility path: start local Livox ROS2 driver.
            if self._start_driver:
                self._start_native_driver(cfg)
            else:
                logger.info(
                    "Lidar native driver start skipped; expecting external "
                    "stream on %s/%s",
                    self._scan_topic,
                    self._imu_topic,
                )

            # 2) Start DDS bridge (subscribe to scan + imu topics)
            self._start_dds_bridge()

            self._connect_time = time.monotonic()
            self._set_state(LidarState.CONNECTED)
            logger.info(
                "Lidar connected - ip=%s, scan=%s, imu=%s",
                cfg.lidar.lidar_ip, self._scan_topic, self._imu_topic,
            )
        except Exception as e:
            self._health.last_error = str(e)
            self._set_state(LidarState.ERROR)
            logger.error("Lidar connect failed: %s", e)
            raise

        return self

    def disconnect(self) -> None:
        """Stop the LiDAR driver and DDS bridge. Safe to call multiple times."""
        if self._state == LidarState.DISCONNECTED:
            return

        # Stop DDS first (no more callbacks during driver shutdown)
        try:
            self._dds_adapter.stop()
        except Exception as e:
            logger.warning("Lidar DDS stop: %s", e)
        self._dds = None

        # Stop native driver
        if self._native:
            try:
                self._native.stop()
            except Exception as e:
                logger.warning("Lidar native stop: %s", e)
            self._native = None

        self._set_state(LidarState.DISCONNECTED)
        self._health.driver_pid = None
        logger.info("Lidar disconnected")

    # Data access

    def on_cloud(self, callback: Callable[[np.ndarray], None]) -> Lidar:
        """Register a point cloud callback: ``fn(numpy_Nx4)``.

        Callbacks fire on the DDS reader thread. Keep them fast (<10ms)
        or offload to your own thread/queue.

        Returns:
            self - for chaining.
        """
        self._frames.on_cloud(callback)
        return self

    def on_raw_cloud(self, callback: Callable) -> Lidar:
        """Register a lossless Livox point-frame callback."""

        self._frames.on_raw_cloud(callback)
        return self

    def on_imu(self, callback: Callable) -> Lidar:
        """Register an IMU callback: ``fn(runtime.msgs.sensor.Imu)``.

        Returns:
            self - for chaining.
        """
        self._frames.on_imu(callback)
        return self

    def get_cloud(self) -> np.ndarray | None:
        """Return the latest point cloud as numpy (N, 4): x, y, z, intensity.

        Non-blocking. Returns ``None`` if no data has arrived yet.
        """
        return self._frames.get_cloud()

    def get_raw_cloud(self):
        """Return the latest lossless Livox point frame."""

        return self._frames.get_raw_cloud()

    def get_imu(self):
        """Return the latest IMU reading (``runtime.msgs.sensor.Imu``).

        Non-blocking. Returns ``None`` if no data yet.
        """
        return self._frames.get_imu()

    def wait_for_cloud(self, timeout: float = 5.0) -> np.ndarray | None:
        """Block until the first point cloud arrives.

        Args:
            timeout: Seconds to wait before returning ``None``.

        Returns:
            numpy (N, 4) or ``None`` on timeout.
        """
        return self._frames.wait_for_cloud(timeout=timeout)

    # Health and introspection

    @property
    def state(self) -> LidarState:
        return self._state

    @property
    def is_connected(self) -> bool:
        return self._state == LidarState.CONNECTED

    @property
    def ip(self) -> str | None:
        return self._ip

    @property
    def fps(self) -> float:
        """Current frame rate (Hz)."""
        return self._frames.fps

    @property
    def health(self) -> LidarHealth:
        """Snapshot of current health metrics."""
        h = self._health
        h.state = self._state
        h.fps = self._frames.fps
        metrics = self._frames.metrics
        h.total_frames = metrics.total_frames
        h.total_points = metrics.total_points
        h.last_frame_points = metrics.last_frame_points
        h.last_frame_time = metrics.last_frame_time
        if self._connect_time > 0 and self._state == LidarState.CONNECTED:
            h.uptime_s = time.monotonic() - self._connect_time
        # Update driver PID
        if self._native and self._native._process:
            proc = self._native._process
            h.driver_pid = proc.pid if proc.poll() is None else None
            h.driver_restarts = self._native._restart_count
        return h

    # Context manager

    def __enter__(self) -> Lidar:
        if self._state != LidarState.CONNECTED:
            self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.disconnect()

    def __repr__(self) -> str:
        return (
            f"Lidar(ip={self._ip!r}, state={self._state.value}, "
            f"fps={self.fps:.1f})"
        )

    # Internal

    def _set_state(self, new: LidarState) -> None:
        with self._state_lock:
            old = self._state
            self._state = new
        if old != new:
            logger.debug("Lidar state: %s -> %s", old.value, new.value)

    def _build_config(self):
        """Return a RobotConfig, overriding lidar_ip if self._ip is set."""
        from runtime.config import get_config

        cfg = get_config()
        if self._ip and self._ip != cfg.lidar.lidar_ip:
            cfg = deepcopy(cfg)
            cfg.lidar.lidar_ip = self._ip
        return cfg

    def _start_native_driver(self, cfg) -> None:
        """Launch livox_ros_driver2 as a managed subprocess.

        The NativeModule provides:
        - Process spawn with correct ROS args and DDS env
        - Watchdog thread: detects crashes, auto-restarts (up to 3x)
        - SIGTERM -> SIGKILL graceful shutdown
        - Subprocess log piping to Python logger
        """
        from .native_factory import livox_driver_process

        self._native = livox_driver_process(cfg)
        try:
            self._native.setup()   # validates executable exists
        except (FileNotFoundError, PermissionError) as e:
            raise RuntimeError(
                f"Livox driver binary not found. "
                f"Build with: source /opt/ros/humble/setup.bash && make build\n"
                f"Detail: {e}"
            ) from e
        self._native.start()       # spawns process + watchdog

    def _start_dds_bridge(self) -> None:
        """Subscribe to LiDAR scan and IMU topics via cyclonedds.

        No rclpy needed - cyclonedds talks DDS directly.
        """
        self._dds_adapter.start()
        self._dds = self._dds_adapter.dds

    # DDS callbacks kept for compatibility with older tests/debug hooks.

    def _on_scan(self, msg) -> None:
        """Delegate LivoxCustomMsg handling to the DDS Adapter."""
        self._dds_adapter.on_scan(msg)

    def _on_imu(self, msg) -> None:
        """Delegate DDS_Imu handling to the DDS Adapter."""
        self._dds_adapter.on_imu(msg)
