"""Built-in routing from deterministic Scenario state into MuJoCo truth."""

from __future__ import annotations

from typing import Any, Mapping, Protocol

from .runtime import ScenarioPlanError, ScenarioSnapshot


class KinematicPoseSink(Protocol):
    """Physics host seam for one atomic scenario-owned mocap update."""

    def apply_kinematic_poses(self, snapshot: ScenarioSnapshot) -> Mapping[str, Any]:
        """Apply every kinematic entity in the stamped snapshot."""


class MujocoScenarioDispatcher:
    """Fail-closed dispatcher that makes MuJoCo the dynamic pose truth source."""

    def __init__(self, sink: KinematicPoseSink) -> None:
        self._sink = sink

    def dispatch(self, snapshot: ScenarioSnapshot) -> None:
        """Route one complete physical entity set without filtering or re-timing it."""

        if not snapshot.entities:
            raise ScenarioPlanError("MuJoCo kinematic dispatch requires at least one entity")
        for entity in snapshot.entities:
            if (
                entity.authority != "scenario"
                or entity.physics_proxy_mode != "kinematic"
                or entity.body_stable_id is None
            ):
                raise ScenarioPlanError(
                    f"{entity.entity_id}: MuJoCo dispatch requires an explicit kinematic physics proxy"
                )
        self._sink.apply_kinematic_poses(snapshot)
