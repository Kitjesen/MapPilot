from __future__ import annotations

# ruff: noqa: S101
from dataclasses import replace

import pytest

from sim.runtime.scenario import (
    EntitySnapshot,
    MujocoScenarioDispatcher,
    ScenarioPlanError,
    ScenarioSnapshot,
    Transform,
)


def _snapshot() -> ScenarioSnapshot:
    return ScenarioSnapshot(
        session_id="a" * 64,
        model_generation=0,
        reset_generation=0,
        sequence=2,
        sim_time_ns=4_000_000,
        entities=(
            EntitySnapshot(
                entity_id="pedestrian_01",
                transform=Transform((1.0, 2.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
                authority="scenario",
                source_epoch=0,
                semantic_class="person",
                motion_state="active",
                physics_proxy_mode="kinematic",
                body_stable_id="pedestrian_01/proxy_root",
            ),
        ),
    )


class RecordingSink:
    def __init__(self) -> None:
        self.snapshots: list[ScenarioSnapshot] = []

    def apply_kinematic_poses(self, snapshot: ScenarioSnapshot) -> dict[str, object]:
        self.snapshots.append(snapshot)
        return {"result": "applied"}


def test_dispatcher_passes_one_complete_physical_batch_to_mujoco() -> None:
    sink = RecordingSink()
    dispatcher = MujocoScenarioDispatcher(sink)
    snapshot = _snapshot()

    dispatcher.dispatch(snapshot)

    assert sink.snapshots == [snapshot]


def test_dispatcher_refuses_to_silently_drop_a_nonphysical_entity() -> None:
    sink = RecordingSink()
    dispatcher = MujocoScenarioDispatcher(sink)
    invalid = replace(
        _snapshot().entities[0],
        physics_proxy_mode="none",
        body_stable_id=None,
    )

    with pytest.raises(ScenarioPlanError, match="kinematic"):
        dispatcher.dispatch(replace(_snapshot(), entities=(invalid,)))

    assert sink.snapshots == []
