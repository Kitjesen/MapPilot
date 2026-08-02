"""Single-robot Open-RMF to LingTu mission translation."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Protocol

from nav.building import (
    ActiveFloor,
    BuildingMissionPort,
    BuildingMissionRequest,
    PoseTarget,
)
from nav.building import (
    NativeGoalCompletionGate as NativeGoalCompletionGate,
)
from runtime.runtime_interface import map_frame_id


@dataclass(frozen=True)
class FloorBinding:
    """Bind one Open-RMF map name to a LingTu building-floor map."""

    building_id: str
    floor_id: str
    map_id: str
    frame_id: str = field(default_factory=map_frame_id)


@dataclass(frozen=True)
class RmfDestination:
    """High-level destination received from an Open-RMF fleet adapter."""

    map_name: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class MissionSubmission:
    """Observable admission result returned to the Open-RMF adapter."""

    request_id: str
    accepted: bool
    reason: str


@dataclass(frozen=True)
class RobotSnapshot:
    """Floor-verified LingTu robot state used for RMF reporting."""

    building_id: str
    floor_id: str
    map_id: str
    x: float
    y: float
    z: float
    yaw: float
    battery_soc: float
    navigation_state: str
    stamp_s: float
    native_request_id: str = ""
    native_command_kind: str = ""
    native_command_accepted: bool = False
    native_status_stamp_s: float = 0.0
    native_control_mode: str = ""
    native_estop_latched: bool = False
    native_operator_takeover_latched: bool = False
    native_plan_seen: bool = False
    native_plan_accepted: bool = False
    native_plan_reason: str = ""
    native_plan_goal: tuple[float, float, float] | None = None
    native_local_active: bool = False
    native_goal_reached: bool = False


@dataclass(frozen=True)
class RmfRobotState:
    """Open-RMF state projection for one LingTu robot."""

    connected: bool
    map_name: str
    position: tuple[float, float, float]
    battery_soc: float
    navigation_state: str
    stamp_s: float


class NavigationGoalPort(Protocol):
    """Safety-aware native navigation capability used by the mission adapter."""

    def autonomy_ready(self) -> tuple[bool, str]:
        """Require exclusive, fresh native autonomy ownership."""

    def send_goal(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        *,
        request_id: str | None = None,
    ) -> None:
        """Send one map-frame goal and wait for its business ACK."""


class NativeSingleFloorMissionPort:
    """Execute same-floor building missions through native navigation."""

    def __init__(
        self,
        *,
        navigation_client: NavigationGoalPort,
        active_floor: ActiveFloor | Callable[[], ActiveFloor],
    ) -> None:
        self._navigation_client = navigation_client
        if callable(active_floor):
            self._active_floor_provider = active_floor
        else:
            self._active_floor_provider = lambda: active_floor

    def submit(self, request: BuildingMissionRequest) -> tuple[bool, str]:
        """Dispatch a same-floor goal or fail closed for a floor transition."""

        try:
            ready, readiness_reason = self._navigation_client.autonomy_ready()
        except Exception:
            return False, "native_autonomy_status_error"
        if ready is not True:
            return False, f"autonomy_not_ready:{readiness_reason or 'unknown'}"
        try:
            active_floor = self._active_floor_provider()
        except Exception:
            return False, "active_floor_unavailable"
        if not isinstance(active_floor, ActiveFloor):
            return False, "active_floor_invalid"
        if (
            request.building_id != active_floor.building_id
            or request.floor_id != active_floor.floor_id
            or request.map_id != active_floor.map_id
        ):
            return False, "floor_transition_executor_unavailable"
        if request.target.frame_id != map_frame_id():
            return False, "unsupported_navigation_frame"
        if not all(
            math.isfinite(value)
            for value in (
                request.target.x,
                request.target.y,
                request.target.z,
                request.target.yaw,
            )
        ):
            return False, "invalid_navigation_target"
        try:
            self._navigation_client.send_goal(
                request.target.x,
                request.target.y,
                request.target.z,
                request.target.yaw,
                request_id=request.request_id,
            )
        except Exception:
            return False, "native_navigation_goal_error"
        return True, "native_navigation_goal_accepted"


class SingleRobotRmfBridge:
    """Translate Open-RMF destinations into LingTu building missions."""

    def __init__(
        self,
        *,
        fleet_name: str,
        robot_name: str,
        mission_port: BuildingMissionPort,
        floor_bindings: dict[str, FloorBinding],
    ) -> None:
        self._fleet_name = str(fleet_name).strip()
        self._robot_name = str(robot_name).strip()
        if not self._fleet_name or not self._robot_name:
            raise ValueError("fleet_name and robot_name are required")
        self._mission_port = mission_port
        self._floor_bindings = dict(floor_bindings)
        identities = {(item.building_id, item.floor_id, item.map_id) for item in self._floor_bindings.values()}
        if len(identities) != len(self._floor_bindings):
            raise ValueError("Open-RMF floor bindings must be one-to-one")

    def navigate(
        self,
        destination: RmfDestination,
        *,
        request_id: str,
    ) -> MissionSubmission:
        """Submit one Open-RMF destination as a floor-aware LingTu mission."""

        request_id = str(request_id).strip()
        if not request_id:
            return MissionSubmission(
                request_id="",
                accepted=False,
                reason="request_id_required",
            )
        if not all(math.isfinite(value) for value in (destination.x, destination.y, destination.yaw)):
            return MissionSubmission(
                request_id=request_id,
                accepted=False,
                reason="invalid_destination_coordinates",
            )
        binding = self._floor_bindings.get(destination.map_name)
        if binding is None:
            return MissionSubmission(
                request_id=request_id,
                accepted=False,
                reason=f"unknown Open-RMF map: {destination.map_name}",
            )
        request = BuildingMissionRequest(
            request_id=request_id,
            source="open_rmf",
            fleet_name=self._fleet_name,
            robot_name=self._robot_name,
            building_id=binding.building_id,
            floor_id=binding.floor_id,
            map_id=binding.map_id,
            target=PoseTarget(
                frame_id=binding.frame_id,
                x=float(destination.x),
                y=float(destination.y),
                z=0.0,
                yaw=float(destination.yaw),
            ),
        )
        accepted, reason = self._mission_port.submit(request)
        return MissionSubmission(
            request_id=request_id,
            accepted=accepted is True,
            reason=str(reason),
        )

    def report_state(self, snapshot: RobotSnapshot) -> RmfRobotState:
        """Project verified LingTu floor identity into Open-RMF robot state."""

        map_name = ""
        for candidate, binding in self._floor_bindings.items():
            if (
                binding.building_id == snapshot.building_id
                and binding.floor_id == snapshot.floor_id
                and binding.map_id == snapshot.map_id
            ):
                map_name = candidate
                break
        return RmfRobotState(
            connected=bool(map_name),
            map_name=map_name,
            position=(float(snapshot.x), float(snapshot.y), float(snapshot.yaw)),
            battery_soc=max(0.0, min(1.0, float(snapshot.battery_soc))),
            navigation_state=str(snapshot.navigation_state),
            stamp_s=float(snapshot.stamp_s),
        )
