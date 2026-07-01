"""ROS-free sensor, command, and planning contracts.

These dataclasses are the bottom-layer contract between LingTu core and external
callers/adapters:

- MuJoCo translates simulator state into :class:`PortableSensorFrame`.
- JSONL/replay translates recorded frames into :class:`PortableSensorFrame`.
- ROS2 translates ROS messages into these frames only inside ``ros-compat``.
- Hardware adapters translate vendor data/commands at the boundary.

The module may depend on LingTu-owned ``runtime.msgs`` types, but it must not import
MuJoCo, ROS2, PCL, Gateway, ML/LLM, LCM, or vendor robot libraries.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from runtime.msgs.geometry import Twist
from runtime.msgs.nav import Odometry, Path
from runtime.msgs.sensor import CameraIntrinsics, Image, Imu, PointCloud2


@dataclass(frozen=True)
class PortableSensorFrame:
    """A single adapter-neutral sensor sample.

    All fields are optional so low-rate command/status adapters and partial
    sensor sources can produce only the data they own.  Consumers should use
    field presence, not source-specific message classes, to decide what changed.
    """

    odometry: Odometry | None = None
    lidar_cloud: PointCloud2 | None = None
    map_cloud: PointCloud2 | None = None
    imu: Imu | None = None
    camera_image: Image | None = None
    depth_image: Image | None = None
    camera_info: CameraIntrinsics | None = None
    timestamp_s: float | None = None
    source: str = "unknown"

    @property
    def has_point_cloud(self) -> bool:
        return self.lidar_cloud is not None or self.map_cloud is not None

    @property
    def has_camera_bundle(self) -> bool:
        return (
            self.camera_image is not None
            or self.depth_image is not None
            or self.camera_info is not None
        )


@dataclass(frozen=True)
class PortableCommandFrame:
    """Adapter-neutral command sink payload.

    The authoritative motion path should still pass through the LingTu safety and
    mux chain before a hardware adapter applies this frame.
    """

    cmd_vel: Twist | None = None
    stop_signal: int | None = None
    timestamp_s: float | None = None
    sink: str = "unknown"
    safety_owner: str = "cmd_vel_mux"

    @property
    def requests_stop(self) -> bool:
        return bool(self.stop_signal)


@dataclass(frozen=True)
class PortablePlanningFrame:
    """Adapter-neutral planning input/output snapshot."""

    odometry: Odometry | None = None
    map_cloud: PointCloud2 | None = None
    goal_path: Path | None = None
    global_path: Path | None = None
    local_path: Path | None = None
    timestamp_s: float | None = None
    source: str = "unknown"

    @property
    def has_plan(self) -> bool:
        return self.global_path is not None or self.local_path is not None


class SensorSource(Protocol):
    """Source adapter that produces portable sensor frames."""

    def poll(self) -> PortableSensorFrame:
        """Return the latest available sensor frame."""
        ...


class CommandSink(Protocol):
    """Sink adapter that consumes portable command frames."""

    def apply(self, command: PortableCommandFrame) -> None:
        """Apply a command after LingTu safety/mux arbitration."""
        ...
