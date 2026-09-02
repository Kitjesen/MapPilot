# ruff: noqa: S101

from __future__ import annotations

from pathlib import Path

import pytest

from sim.runtime.coordinator import (
    CoordinatorError,
    PhysicsGlobalPolicy,
    PhysicsPlan,
    PhysicsRobotPlan,
)
from sim.runtime.coordinator.mujoco_process import MujocoProcess
from sim.runtime.coordinator.run_allocation import RunAllocation

REPO_ROOT = Path(__file__).resolve().parents[2]
MUJOCO_HEADLESS = (
    REPO_ROOT
    / "build"
    / "mujoco-runtime-physics-win"
    / "Release"
    / "lingtu_mujoco_headless.exe"
)
SESSION_ID = "f" * 64


def _ray_hit(**overrides: object) -> dict[str, object]:
    hit: dict[str, object] = {
        "xyz_sensor": [0.0, 0.0, -1.0],
        "origin_world_m": [0.0, 0.0, 1.0],
        "direction_world": [0.0, 0.0, -1.0],
        "position_world_m": [0.0, 0.0, 0.0],
        "distance_m": 1.0,
        "body_stable_id": "world/floor",
        "entity_id": "world",
        "offset_time_ns": 0,
        "reflectivity": 15,
        "tag": 0,
        "line": 3,
    }
    hit.update(overrides)
    return hit


def _require_headless() -> Path:
    if not MUJOCO_HEADLESS.is_file():
        pytest.skip(f"MuJoCo headless executable is missing: {MUJOCO_HEADLESS}")
    return MUJOCO_HEADLESS


def _allocation(tmp_path: Path) -> RunAllocation:
    run_dir = tmp_path / "run-raycast"
    log_dir = run_dir / "logs"
    log_dir.mkdir(parents=True)
    return RunAllocation(
        run_id="run-raycast",
        run_dir=run_dir,
        artifact_root=REPO_ROOT,
        log_dir=log_dir,
        ports={},
        shm={},
        session_id=SESSION_ID,
        boot_id="boot-raycast",
        dds_domain=0,
    )


def _plan() -> PhysicsPlan:
    return PhysicsPlan(
        session_id=SESSION_ID,
        model_generation=11,
        reset_generation=0,
        repo_root=REPO_ROOT,
        world_model_path=REPO_ROOT / "tests/sim/physics/fixtures/mid360_raycast.xml",
        global_policy=PhysicsGlobalPolicy(
            timestep_s=0.002,
            integrator="rk4",
            solver="newton",
            iterations=100,
            gravity_mps2=(0.0, 0.0, -9.81),
        ),
        robots=(
            PhysicsRobotPlan(
                instance_id="robot",
                model_path=REPO_ROOT / "tests/sim/physics/fixtures/free_body.xml",
                attach_root="test_body",
                initial_keyframe=None,
                position_m=(2.0, 0.0, 1.0),
                quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
            ),
        ),
    )


def test_mujoco_process_serializes_mid360_raycast_batch(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    process = MujocoProcess(Path("unused.exe"))
    requests: list[str] = []

    def request(line: str) -> dict[str, object]:
        requests.append(line)
        return {
            "event": "raycast",
            "sensor_frame_id": "lidar_site",
            "session_id": SESSION_ID,
            "model_generation": 11,
            "reset_generation": 2,
            "sequence": 7,
            "sim_time_ns": 4_000_000,
            "hit_count": 1,
            "hits": [_ray_hit()],
        }

    monkeypatch.setattr(process, "_request", request)

    event = process.raycast(
        sensor_frame_id="lidar_site",
        directions_sensor=((0.0, 0.0, -1.0), (1.0, 0.0, 0.0)),
        offsets_time_ns=(0, 2_000_000),
        session_id=SESSION_ID,
        model_generation=11,
        reset_generation=2,
        sequence=7,
        sim_time_ns=4_000_000,
        range_min_m=0.1,
        range_max_m=40.0,
        reflectivity_proxy=15,
        unknown_line=3,
    )

    assert event["hit_count"] == 1
    assert requests == [
        "raycast lidar_site "
        "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff "
        "11 2 7 4000000 0.10000000000000001 40 15 3 2 "
        "0 0 -1 0 1 0 0 2000000"
    ]


def test_mujoco_process_rejects_raycast_generation_mismatch(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    process = MujocoProcess(Path("unused.exe"))
    monkeypatch.setattr(
        process,
        "_request",
        lambda line: {
            "event": "raycast",
            "sensor_frame_id": "lidar_site",
            "session_id": SESSION_ID,
            "model_generation": 10,
            "reset_generation": 0,
            "sequence": 0,
            "sim_time_ns": 0,
            "hit_count": 0,
            "hits": [],
        },
    )

    with pytest.raises(CoordinatorError, match="model_generation"):
        process.raycast(
            sensor_frame_id="lidar_site",
            directions_sensor=((0.0, 0.0, -1.0),),
            session_id=SESSION_ID,
            model_generation=11,
            reset_generation=0,
            sequence=0,
            sim_time_ns=0,
        )


def test_mujoco_process_rejects_invalid_raycast_payload_before_host(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    process = MujocoProcess(Path("unused.exe"))
    monkeypatch.setattr(
        process,
        "_request",
        lambda line: pytest.fail(f"unexpected request: {line}"),
    )

    with pytest.raises(CoordinatorError, match=r"directions_sensor\[0\]"):
        process.raycast(
            sensor_frame_id="lidar_site",
            directions_sensor=((0.0, 0.0, 0.0),),
            session_id=SESSION_ID,
            model_generation=11,
            reset_generation=0,
            sequence=0,
            sim_time_ns=0,
        )


def test_mujoco_process_rejects_raycast_hit_without_body_identity(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    process = MujocoProcess(Path("unused.exe"))
    hit = _ray_hit()
    del hit["body_stable_id"]
    monkeypatch.setattr(
        process,
        "_request",
        lambda line: {
            "event": "raycast",
            "sensor_frame_id": "lidar_site",
            "session_id": SESSION_ID,
            "model_generation": 11,
            "reset_generation": 0,
            "sequence": 0,
            "sim_time_ns": 0,
            "hit_count": 1,
            "hits": [hit],
        },
    )

    with pytest.raises(CoordinatorError, match="body_stable_id"):
        process.raycast(
            sensor_frame_id="lidar_site",
            directions_sensor=((0.0, 0.0, -1.0),),
            session_id=SESSION_ID,
            model_generation=11,
            reset_generation=0,
            sequence=0,
            sim_time_ns=0,
        )


def test_mujoco_process_rejects_raycast_sequence_mismatch(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    process = MujocoProcess(Path("unused.exe"))
    monkeypatch.setattr(
        process,
        "_request",
        lambda line: {
            "event": "raycast",
            "sensor_frame_id": "lidar_site",
            "session_id": SESSION_ID,
            "model_generation": 11,
            "reset_generation": 0,
            "sequence": 6,
            "sim_time_ns": 0,
            "hit_count": 0,
            "hits": [],
        },
    )

    with pytest.raises(CoordinatorError, match="sequence"):
        process.raycast(
            sensor_frame_id="lidar_site",
            directions_sensor=((0.0, 0.0, -1.0),),
            session_id=SESSION_ID,
            model_generation=11,
            reset_generation=0,
            sequence=7,
            sim_time_ns=0,
        )


def test_mujoco_process_mid360_raycast_e2e(tmp_path: Path) -> None:
    process = MujocoProcess(_require_headless(), timeout_s=10.0)
    temp_dir = tmp_path / "mujoco_raycast"
    try:
        ready = process.prepare(_plan(), _allocation(temp_dir))
        assert ready["event"] == "ready"
        assert ready["model_generation"] == 11
        assert ready["reset_generation"] == 0

        frame = process.raycast(
            sensor_frame_id="lidar_site",
            directions_sensor=((0.0, 0.0, -1.0), (1.0, 0.0, 0.0)),
            offsets_time_ns=(0, 2_000_000),
            session_id=SESSION_ID,
            model_generation=11,
            reset_generation=0,
            sequence=0,
            sim_time_ns=0,
            range_min_m=0.1,
            range_max_m=40.0,
            reflectivity_proxy=15,
            unknown_line=0,
        )

        assert frame["event"] == "raycast"
        assert frame["sensor_frame_id"] == "lidar_site"
        assert frame["session_id"] == SESSION_ID
        assert frame["model_generation"] == 11
        assert frame["reset_generation"] == 0
        assert frame["sequence"] == 0
        assert frame["sim_time_ns"] == 0
        assert frame["hit_count"] == 1
        hit = frame["hits"][0]
        assert hit["xyz_sensor"] == pytest.approx([0.0, 0.0, -1.0], abs=1e-6)
        assert hit["origin_world_m"] == pytest.approx([0.0, 0.0, 1.0], abs=1e-6)
        assert hit["direction_world"] == pytest.approx([0.0, 0.0, -1.0], abs=1e-6)
        assert hit["position_world_m"] == pytest.approx([0.0, 0.0, 0.0], abs=1e-6)
        assert hit["distance_m"] == pytest.approx(1.0, abs=1e-6)
        assert isinstance(hit["body_stable_id"], str) and hit["body_stable_id"]
        assert isinstance(hit["entity_id"], str) and hit["entity_id"]
        assert hit["offset_time_ns"] == 0
        assert hit["reflectivity"] == 15
        assert hit["tag"] == 0
        assert hit["line"] == 0

        reset = process.reset()
        assert reset["reset_generation"] == 1
        with pytest.raises(CoordinatorError, match="reset_generation"):
            process.raycast(
                sensor_frame_id="lidar_site",
                directions_sensor=((0.0, 0.0, -1.0),),
                session_id=SESSION_ID,
                model_generation=11,
                reset_generation=0,
                sequence=0,
                sim_time_ns=0,
            )
        refreshed = process.raycast(
            sensor_frame_id="lidar_site",
            directions_sensor=((0.0, 0.0, -1.0),),
            session_id=SESSION_ID,
            model_generation=11,
            reset_generation=1,
            sequence=0,
            sim_time_ns=0,
        )
        assert refreshed["hit_count"] == 1
    finally:
        process.stop()
