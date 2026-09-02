from __future__ import annotations

# ruff: noqa: S101
from dataclasses import replace
from pathlib import Path

import pytest

from sim.runtime.coordinator import (
    CoordinatorError,
    PhysicsGlobalPolicy,
    PhysicsKinematicEntityPlan,
    PhysicsPlan,
)
from sim.runtime.coordinator.coordinator import PhysicsPayloadPlan
from sim.runtime.coordinator.mujoco_process import MujocoProcess
from sim.runtime.scenario import EntitySnapshot, ScenarioSnapshot, Transform

SESSION_ID = "scenario-test"


def _plan() -> PhysicsPlan:
    return PhysicsPlan(
        session_id=SESSION_ID,
        model_generation=3,
        reset_generation=0,
        repo_root=Path("D:/repo"),
        world_model_path=Path("D:/repo/world.xml"),
        global_policy=PhysicsGlobalPolicy(
            timestep_s=0.002,
            integrator="rk4",
            solver="newton",
            iterations=100,
            gravity_mps2=(0.0, 0.0, -9.81),
        ),
        robots=(),
        kinematic_entities=(
            PhysicsKinematicEntityPlan(
                entity_id="pedestrian_01",
                model_path=Path("D:/repo/pedestrian.xml"),
                attach_root="proxy_root",
                position_m=(1.0, 2.0, 0.0),
                quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
            ),
        ),
    )


def _snapshot() -> ScenarioSnapshot:
    return ScenarioSnapshot(
        session_id=SESSION_ID,
        model_generation=3,
        reset_generation=2,
        sequence=9,
        sim_time_ns=18_000_000,
        entities=(
            EntitySnapshot(
                entity_id="pedestrian_01",
                transform=Transform(
                    position_m=(4.0, -1.0, 0.0),
                    quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
                ),
                authority="scenario",
                source_epoch=0,
                semantic_class="person",
                motion_state="active",
                physics_proxy_mode="kinematic",
                body_stable_id="pedestrian_01/proxy_root",
            ),
        ),
    )


def test_command_stages_package_declared_kinematic_entity_in_shared_scene() -> None:
    command = MujocoProcess(Path("mujoco.exe"))._command(_plan())

    marker = command.index("--kinematic-entity")
    assert command[marker : marker + 11] == [
        "--kinematic-entity",
        "pedestrian_01",
        str(Path("D:/repo/pedestrian.xml")),
        "proxy_root",
        "1",
        "2",
        "0",
        "1",
        "0",
        "0",
        "0",
    ]


def test_payload_argv_uses_identity_and_paths() -> None:
    payload = PhysicsPayloadPlan(
        instance_id="rws_01",
        namespace="rws_01",
        robot_instance_id="thunder_01",
        package={
            "id": "fictional_rws_01",
            "version": "1.0.0",
            "kind": "payload",
            "manifest": "payload.package.yaml",
        },
        parent_frame="payload_top",
        parent_body="base_link",
        model_path=Path("D:/repo/payload.xml"),
        attach_root="payload_base",
        position_m=(0.0, 0.0, 0.14),
        quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
        authority="mujoco",
        collision_representation="primitive_proxy",
        frames=(
            {"name": "payload_base", "role": "payload_root"},
            {
                "name": "yaw_frame",
                "role": "articulation",
                "parent_frame": "payload_base",
            },
        ),
    )

    command = MujocoProcess(Path("mujoco.exe"))._command(
        replace(_plan(), payloads=(payload,))
    )

    marker = command.index("--payload")
    next_marker = command.index("--kinematic-entity")
    assert command[marker:next_marker] == [
        "--payload",
        "rws_01",
        "rws_01",
        "thunder_01",
        "fictional_rws_01",
        "1.0.0",
        "payload",
        "payload.package.yaml",
        "payload_top",
        "base_link",
        "0",
        "0",
        "0.14000000000000001",
        "1",
        "0",
        "0",
        "0",
        str(Path("D:/repo/payload.xml")),
        "payload_base",
        "mujoco",
        "primitive_proxy",
        "2",
        "payload_base",
        "payload_root",
        "-",
        "yaw_frame",
        "articulation",
        "payload_base",
    ]


def test_apply_kinematic_poses_serializes_and_validates_the_stamped_batch() -> None:
    process = MujocoProcess(Path("unused.exe"))
    requests: list[str] = []

    def request(line: str) -> dict[str, object]:
        requests.append(line)
        return {
            "event": "kinematic_poses",
            "session_id": SESSION_ID,
            "model_generation": 3,
            "reset_generation": 2,
            "sequence": 9,
            "sim_time_ns": 18_000_000,
            "entity_count": 1,
            "result": "applied",
        }

    process._request = request  # type: ignore[method-assign]
    event = process.apply_kinematic_poses(_snapshot())

    assert event["result"] == "applied"
    assert requests == [
        "kinematic-poses "
        f"{SESSION_ID} 3 2 9 18000000 1 "
        "pedestrian_01 pedestrian_01/proxy_root 4 -1 0 1 0 0 0"
    ]


def test_apply_kinematic_poses_rejects_a_non_applied_host_result() -> None:
    process = MujocoProcess(Path("unused.exe"))
    process._request = lambda _line: {  # type: ignore[method-assign]
        "event": "kinematic_poses",
        "session_id": SESSION_ID,
        "model_generation": 3,
        "reset_generation": 2,
        "sequence": 9,
        "sim_time_ns": 18_000_000,
        "entity_count": 1,
        "result": "rejected_entity_set",
    }

    with pytest.raises(CoordinatorError, match="rejected_entity_set"):
        process.apply_kinematic_poses(_snapshot())


def test_apply_kinematic_poses_rejects_invalid_session_before_protocol_write() -> None:
    process = MujocoProcess(Path("unused.exe"))
    process._request = lambda _line: pytest.fail("invalid snapshot reached the host")  # type: ignore[method-assign]

    with pytest.raises(CoordinatorError, match=r"session_id"):
        process.apply_kinematic_poses(
            replace(_snapshot(), session_id=f"{SESSION_ID}\nstop")
        )


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("model_generation", -1),
        ("reset_generation", 2**32),
        ("sequence", True),
        ("sim_time_ns", -1),
    ),
)
def test_apply_kinematic_poses_rejects_invalid_stamp_before_protocol_write(
    field: str, value: object
) -> None:
    process = MujocoProcess(Path("unused.exe"))
    process._request = lambda _line: pytest.fail("invalid snapshot reached the host")  # type: ignore[method-assign]

    with pytest.raises(CoordinatorError, match=field):
        process.apply_kinematic_poses(replace(_snapshot(), **{field: value}))
