"""Production endpoint factories keyed by compiled SensorPlan contracts."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .contracts import SensorRoute, SensorStreamPlan
from .dds_adapter import ImuDdsAdapter, Mid360DdsAdapter, TruthOdometryDdsAdapter
from .mid360 import Mid360PatternCursor, Mid360RaycastExtractor
from .session import SensorEndpoint, SensorEndpointFactory

_TRUTH_ODOMETRY_ROUTE = SensorRoute(
    owner="physics",
    source="mujoco_truth",
    transport="typed_dds",
)
_IMU_ROUTE = SensorRoute(owner="physics", source="mujoco_sensor", transport="typed_dds")
_MID360_ROUTE = SensorRoute(owner="physics", source="mujoco_livox_model", transport="typed_dds")


class SensorEndpointRoutingError(RuntimeError):
    """Raised when more than one runtime owns the same compiled stream."""


@dataclass(frozen=True)
class SensorEndpointRouter:
    """Compose endpoint factories while preserving one owner per stream.

    A stream may remain intentionally unbound when no factory matches.  More
    than one match is an architecture error because it would create duplicate
    publishers for one stable SensorPlan stream.
    """

    factories: tuple[SensorEndpointFactory, ...]

    def __post_init__(self) -> None:
        if not self.factories or any(not callable(factory) for factory in self.factories):
            raise ValueError("sensor endpoint router requires at least one callable factory")

    def __call__(
        self,
        stream: SensorStreamPlan,
        allocation: Any,
    ) -> SensorEndpoint | None:
        """Return the sole matching endpoint and reject duplicate ownership."""

        matches = tuple(
            endpoint
            for factory in self.factories
            if (endpoint := factory(stream, allocation)) is not None
        )
        if len(matches) > 1:
            raise SensorEndpointRoutingError(
                f"sensor stream {stream.sensor_id!r} matched more than one endpoint factory"
            )
        return matches[0] if matches else None


@dataclass(frozen=True)
class TruthOdometryEndpointFactory:
    """Create the native DDS endpoint for exact truth-odometry declarations."""

    executable: Path
    parent_frame: str
    source_id: str = "truth-odom-dds"

    def __post_init__(self) -> None:
        object.__setattr__(self, "executable", Path(self.executable).resolve())
        for field, value in (
            ("parent_frame", self.parent_frame),
            ("source_id", self.source_id),
        ):
            if (
                not isinstance(value, str)
                or not value
                or value != value.strip()
                or any(character.isspace() for character in value)
            ):
                raise ValueError(f"{field} must be one non-empty frame/source token")

    def __call__(
        self,
        stream: SensorStreamPlan,
        allocation: Any,
    ) -> SensorEndpoint | None:
        """Return an endpoint only for the exact v1 truth-odometry contract."""

        if (
            stream.stream_kind != "truth_odom"
            or stream.message_type != "lingtu.dds.Odometry"
            or stream.route != _TRUTH_ODOMETRY_ROUTE
        ):
            return None
        return SensorEndpoint.truth_odometry(
            source_id=self.source_id,
            sink=TruthOdometryDdsAdapter(
                self.executable,
                allocation=allocation,
                parent_frame=self.parent_frame,
                child_frame=stream.frame_id,
            ),
        )


@dataclass(frozen=True)
class ImuEndpointFactory:
    """Create a native DDS endpoint only for the exact simulation IMU route."""

    executable: Path
    source_id: str = "mujoco-imu-dds"

    def __post_init__(self) -> None:
        object.__setattr__(self, "executable", Path(self.executable).resolve())
        if (
            not isinstance(self.source_id, str)
            or not self.source_id
            or self.source_id != self.source_id.strip()
            or any(character.isspace() for character in self.source_id)
        ):
            raise ValueError("source_id must be one non-empty source token")

    def __call__(self, stream: SensorStreamPlan, allocation: Any) -> SensorEndpoint | None:
        """Return an endpoint only for the exact IMU route contract."""
        if (
            stream.stream_kind != "imu"
            or stream.message_type != "lingtu.dds.Imu"
            or stream.route != _IMU_ROUTE
        ):
            return None
        return SensorEndpoint.imu(
            source_id=self.source_id,
            sink=ImuDdsAdapter(
                self.executable,
                allocation=allocation,
            ),
        )


@dataclass(frozen=True)
class Mid360EndpointFactory:
    """Create a sim-only MID-360 DDS endpoint for the new physics raycast route."""

    executable: Path
    process: Any
    source_id: str = "mujoco-mid360-dds"
    cursor_factory: Callable[[], Any] = Mid360PatternCursor

    def __post_init__(self) -> None:
        object.__setattr__(self, "executable", Path(self.executable).resolve())
        if not callable(getattr(self.process, "raycast", None)):
            raise ValueError("process must provide raycast()")
        if not callable(self.cursor_factory):
            raise ValueError("cursor_factory must be callable")
        if (
            not isinstance(self.source_id, str)
            or not self.source_id
            or self.source_id != self.source_id.strip()
            or any(character.isspace() for character in self.source_id)
        ):
            raise ValueError("source_id must be one non-empty source token")

    def __call__(self, stream: SensorStreamPlan, allocation: Any) -> SensorEndpoint | None:
        """Return an endpoint only for the exact Mid360 route contract."""

        if (
            stream.stream_kind != "mid360"
            or stream.message_type != "lingtu.dds.LivoxFrame"
            or stream.route != _MID360_ROUTE
        ):
            return None
        if stream.raycast_frame_stable_id is None:
            raise ValueError(
                "compiled Mid360 stream requires raycast_frame_stable_id"
            )
        return SensorEndpoint(
            source_id=self.source_id,
            sink=Mid360DdsAdapter(
                self.executable,
                allocation=allocation,
                frame_id=stream.frame_id,
            ),
            extractor=Mid360RaycastExtractor(
                process=self.process,
                sensor_frame_id=stream.raycast_frame_stable_id,
                cursor=self.cursor_factory(),
            ),
        )


__all__ = [
    "ImuEndpointFactory",
    "Mid360EndpointFactory",
    "SensorEndpointRouter",
    "SensorEndpointRoutingError",
    "TruthOdometryEndpointFactory",
]
