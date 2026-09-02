"""Contracts for session-bound simulation runtime JSON documents."""

# ruff: noqa: S101

from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from sim.runtime.coordinator.live_snapshot import truth_snapshot_document
from sim.runtime.replay.visual import _runtime_manifest as replay_runtime_manifest
from sim.tests.test_sim_plan_schemas import SchemaError, _validate

REPO_ROOT = Path(__file__).resolve().parents[2]
SCHEMA_ROOT = REPO_ROOT / "sim" / "contracts" / "schemas"
MUJOCO_SNAPSHOT_SOURCE = (
    REPO_ROOT / "sim" / "runtime" / "physics" / "apps" / "mujoco_snapshot.cpp"
)


def _schema(filename: str) -> dict[str, Any]:
    return json.loads((SCHEMA_ROOT / filename).read_text(encoding="utf-8"))


def _assert_valid(document: dict[str, Any], schema: dict[str, Any]) -> None:
    _validate(document, schema, schema)


def _assert_invalid(document: dict[str, Any], schema: dict[str, Any]) -> None:
    with pytest.raises(SchemaError):
        _validate(document, schema, schema)


def _snapshot_event() -> dict[str, Any]:
    return {
        "event": "snapshot",
        "session_id": "session-a",
        "model_generation": 3,
        "reset_generation": 2,
        "sequence": 11,
        "physics_step": 88,
        "sim_time_ns": 22_000_000,
        "bodies": [
            {
                "stable_id": "thunder_01/base_link",
                "instance_id": "thunder_01",
                "frame_id": "base_link",
                "position_m": [1.0, 2.0, 0.4],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                "linear_velocity_mps": [0.1, 0.0, 0.0],
                "angular_velocity_rps": [0.0, 0.0, 0.2],
            }
        ],
        "joints": [
            {
                "stable_id": "thunder_01/front_left_hip",
                "instance_id": "thunder_01",
                "position_rad": [0.1],
                "velocity_rps": [0.2],
            }
        ],
        "actuators": [],
        "sensors": [
            {
                "source_stable_id": "thunder_01/imu",
                "sensor_type": "gyro",
                "values": [0.0, 0.0, 0.2],
            }
        ],
    }


def _runtime_manifest() -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.session-runtime.v1",
        "run_id": "run-a",
        "session_id": "session-a",
        "model_generation": 3,
        "reset_generation": 2,
        "state": "RUNNING",
        "bindings": {
            "physics": {
                "required": True,
                "state": "ACTIVE",
                "source_id": "mujoco-headless",
                "failure_reason": None,
                "model_generation": 3,
                "reset_generation": 2,
            },
            "visual": {
                "required": False,
                "state": "UNBOUND",
                "source_id": None,
                "failure_reason": None,
                "model_generation": 3,
                "reset_generation": 2,
            },
            "sensors": {
                "required": False,
                "state": "UNBOUND",
                "source_id": None,
                "failure_reason": None,
                "model_generation": 3,
                "reset_generation": 2,
            },
            "control": {
                "required": False,
                "state": "UNBOUND",
                "source_id": None,
                "failure_reason": None,
                "model_generation": 3,
                "reset_generation": 2,
            },
        },
        "sensor_streams": None,
        "bundle_dir": "D:/runs/bundle",
        "allocation": {
            "run_dir": "D:/runs/run-a",
            "log_dir": "D:/runs/run-a/logs",
            "boot_id": "boot-a",
            "physics_pid": 4242,
            "dds_domain": 17,
            "ports": {"truth_snapshot": 32001},
            "shm": {"front_camera": "lingtu-camera-a"},
            "shared_memory": {"front_camera": "lingtu-camera-a"},
        },
        "clock": {"sequence": 11, "physics_step": 88, "sim_time_ns": 22_000_000},
    }


def _visual_replay_runtime_manifest(
    *,
    state: str = "STOPPED",
    visual_state: str = "ACTIVE",
    visual_pid: int | None = 4242,
    failure_reason: str | None = None,
) -> dict[str, Any]:
    allocation = SimpleNamespace(
        run_id="replay-run-a",
        run_dir=Path("D:/runs/replay-run-a"),
        log_dir=Path("D:/runs/replay-run-a/logs"),
        boot_id="boot-replay-a",
        dds_domain=17,
        ports={"visual_snapshot_udp": 25124},
        shm={},
        shared_memory={},
    )
    replay = SimpleNamespace(
        session_id="session-a",
        model_generation=3,
        end_reset_generation=2,
        run_id="source-run-a",
        root=Path("D:/recordings/source-run-a"),
        frame_count=2,
        frames=(
            SimpleNamespace(snapshot={"sequence": 10}),
            SimpleNamespace(
                snapshot={
                    "sequence": 11,
                    "physics_step": 88,
                    "sim_time_ns": 22_000_000,
                }
            ),
        ),
    )
    return replay_runtime_manifest(
        allocation=allocation,
        bundle_dir=Path("D:/runs/bundle"),
        replay=replay,
        state=state,
        visual_state=visual_state,
        visual_pid=visual_pid,
        failure_reason=failure_reason,
    )


def test_truth_snapshot_schema_accepts_the_coordinator_wire_document() -> None:
    schema = _schema("truth-snapshot.v1.json")
    document = truth_snapshot_document(_snapshot_event())

    assert schema["$id"] == "lingtu.sim.truth-snapshot.v1"
    _assert_valid(document, schema)


@pytest.mark.parametrize("field", ["session_id", "physics_step"])
def test_truth_snapshot_schema_requires_session_bound_clock_fields(field: str) -> None:
    schema = _schema("truth-snapshot.v1.json")
    document = truth_snapshot_document(_snapshot_event())
    document.pop(field)

    _assert_invalid(document, schema)


def test_truth_snapshot_schema_rejects_the_unscoped_mujoco_cli_shape() -> None:
    schema = _schema("truth-snapshot.v1.json")
    document = {
        "schema": "lingtu.sim.truth-snapshot.v1",
        "model_generation": 0,
        "reset_generation": 0,
        "sequence": 0,
        "sim_time_ns": 0,
        "bodies": [
            {
                "body_id": 1,
                "name": "base",
                "position_m": [0.0, 0.0, 0.0],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                "linear_velocity_mps": [0.0, 0.0, 0.0],
                "angular_velocity_rps": [0.0, 0.0, 0.0],
            }
        ],
    }

    _assert_invalid(document, schema)


def test_mujoco_snapshot_cli_uses_its_offline_document_contract() -> None:
    schema = _schema("mujoco-snapshot.v1.json")
    source = MUJOCO_SNAPSHOT_SOURCE.read_text(encoding="utf-8")
    document = {
        "schema": "lingtu.sim.mujoco-snapshot.v1",
        "model_generation": 0,
        "reset_generation": 0,
        "sequence": 0,
        "sim_time_ns": 0,
        "bodies": [
            {
                "body_id": 1,
                "name": "base",
                "position_m": [0.0, 0.0, 0.0],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                "linear_velocity_mps": [0.0, 0.0, 0.0],
                "angular_velocity_rps": [0.0, 0.0, 0.0],
            }
        ],
    }

    assert schema["$id"] == "lingtu.sim.mujoco-snapshot.v1"
    assert "lingtu.sim.mujoco-snapshot.v1" in source
    assert "lingtu.sim.truth-snapshot.v1" not in source
    _assert_valid(document, schema)


def test_session_runtime_schema_accepts_the_coordinator_manifest() -> None:
    schema = _schema("session-runtime.v1.json")

    assert schema["$id"] == "lingtu.sim.session-runtime.v1"
    _assert_valid(_runtime_manifest(), schema)


@pytest.mark.parametrize(
    ("state", "visual_state", "visual_pid", "failure_reason"),
    [
        ("PREPARING", "PREPARING", None, None),
        ("STOPPED", "ACTIVE", 4242, None),
        ("FAILED", "FAILED", 4242, "visual process exited"),
    ],
)
def test_session_runtime_schema_accepts_the_visual_replay_manifest(
    state: str,
    visual_state: str,
    visual_pid: int | None,
    failure_reason: str | None,
) -> None:
    schema = _schema("session-runtime.v1.json")

    document = _visual_replay_runtime_manifest(
        state=state,
        visual_state=visual_state,
        visual_pid=visual_pid,
        failure_reason=failure_reason,
    )

    _assert_valid(document, schema)


def test_session_runtime_schema_rejects_a_live_replay_hybrid() -> None:
    schema = _schema("session-runtime.v1.json")
    document = _runtime_manifest()
    replay = _visual_replay_runtime_manifest()
    document["failure_reason"] = None
    document["replay"] = replay["replay"]

    _assert_invalid(document, schema)


def test_session_runtime_schema_rejects_live_binding_fields_in_replay() -> None:
    schema = _schema("session-runtime.v1.json")
    document = _visual_replay_runtime_manifest()
    document["bindings"]["visual"]["model_generation"] = 3

    _assert_invalid(document, schema)


def test_session_runtime_schema_accepts_sensor_readiness_manifest() -> None:
    schema = _schema("session-runtime.v1.json")
    document = _runtime_manifest()
    document["sensor_streams"] = {
        "model_generation": 3,
        "reset_generation": 2,
        "is_ready": True,
        "required_stream_ids": ["thunder_01.imu"],
        "blocking_reasons": {},
        "failures": {},
        "streams": {
            "thunder_01.imu": {
                "stream_id": "thunder_01.imu",
                "stream_kind": "imu",
                "required": True,
                "state": "ACTIVE",
                "source": "mujoco_truth",
                "owner": "physics",
                "transport": "dds",
                "model_generation": 3,
                "reset_generation": 2,
                "failure_reason": None,
                "runtime_source_id": "mujoco-headless",
            }
        },
        "summary": {"required": 1, "active": 1},
    }

    _assert_valid(document, schema)


def test_runtime_document_schemas_reject_unknown_root_fields() -> None:
    for filename, document in (
        ("truth-snapshot.v1.json", truth_snapshot_document(_snapshot_event())),
        ("session-runtime.v1.json", _runtime_manifest()),
        ("session-runtime.v1.json", _visual_replay_runtime_manifest()),
    ):
        invalid = deepcopy(document)
        invalid["unexpected"] = True
        _assert_invalid(invalid, _schema(filename))


def test_simulation_contract_index_lists_every_schema_file_and_id() -> None:
    index = (SCHEMA_ROOT / "README.md").read_text(encoding="utf-8")

    for path in SCHEMA_ROOT.glob("*.json"):
        schema = json.loads(path.read_text(encoding="utf-8"))
        assert f"`{path.name}`" in index
        assert f"`{schema['$id']}`" in index
