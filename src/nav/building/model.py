"""Floor-aware navigation mission domain types."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from runtime.contracts.building import LiftState


@dataclass(frozen=True)
class ActiveFloor:
    """Building-floor identity currently owned by localization."""

    building_id: str
    floor_id: str
    map_id: str


@dataclass(frozen=True)
class PoseTarget:
    """Metric target in one floor map."""

    frame_id: str
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class BuildingMissionRequest:
    """One floor-aware mission accepted by LingTu orchestration."""

    request_id: str
    source: str
    fleet_name: str
    robot_name: str
    building_id: str
    floor_id: str
    map_id: str
    target: PoseTarget

    @property
    def target_floor(self) -> ActiveFloor:
        """Return the immutable building/floor/map identity of the goal."""

        return ActiveFloor(
            building_id=self.building_id,
            floor_id=self.floor_id,
            map_id=self.map_id,
        )


class GoalProgress(str, Enum):
    """Correlated native goal state observed by building orchestration."""

    PENDING = "pending"
    EXECUTING = "executing"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    BLOCKED = "blocked"


class BuildingMissionPhase(str, Enum):
    """Small public lifecycle for one building-scale mission."""

    IDLE = "IDLE"
    FLOOR_TRANSITION = "FLOOR_TRANSITION"
    TARGET_NAVIGATION = "TARGET_NAVIGATION"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


@dataclass(frozen=True)
class BuildingMissionStatus:
    """Observable building mission state without exposing transport details."""

    phase: BuildingMissionPhase
    request_id: str = ""
    reason: str = ""
    target_floor: ActiveFloor | None = None
    transition_phase: str = ""

    @property
    def active(self) -> bool:
        """Report whether the mission still owns an executable step."""

        return self.phase in {
            BuildingMissionPhase.FLOOR_TRANSITION,
            BuildingMissionPhase.TARGET_NAVIGATION,
        }


@dataclass(frozen=True)
class LiftTransitionPlan:
    """Configured connector geometry for one directed floor transition."""

    lift_id: str
    source_floor: ActiveFloor
    target_floor: ActiveFloor
    source_lobby: PoseTarget
    source_cabin: PoseTarget
    target_cabin: PoseTarget
    target_lobby: PoseTarget


class LiftTransitionPhase(str, Enum):
    """Safety-critical steps of a single lift transition."""

    IDLE = "IDLE"
    APPROACH_SOURCE = "APPROACH_SOURCE"
    WAIT_SOURCE_DOOR = "WAIT_SOURCE_DOOR"
    ENTER_CABIN = "ENTER_CABIN"
    RIDE = "RIDE"
    VERIFY_TARGET_LOCALIZATION = "VERIFY_TARGET_LOCALIZATION"
    EXIT_CABIN = "EXIT_CABIN"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


@dataclass(frozen=True)
class LiftTransitionStatus:
    """Observable lift transition state."""

    phase: LiftTransitionPhase
    request_id: str = ""
    session_id: str = ""
    reason: str = ""
    release_pending: bool = False

    @property
    def active(self) -> bool:
        """Report whether the lift transition still owns an executable step."""

        return self.phase in {
            LiftTransitionPhase.APPROACH_SOURCE,
            LiftTransitionPhase.WAIT_SOURCE_DOOR,
            LiftTransitionPhase.ENTER_CABIN,
            LiftTransitionPhase.RIDE,
            LiftTransitionPhase.VERIFY_TARGET_LOCALIZATION,
            LiftTransitionPhase.EXIT_CABIN,
        }
