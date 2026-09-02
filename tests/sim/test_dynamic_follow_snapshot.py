from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from lingtu.assembly.compiler import compile_run_plan
from perception.detection.sim_scene_observer import SimSceneObserver
from sim.compat.engine.mujoco.engine import MuJoCoEngine
from sim.runtime.scenario.runtime import ScenarioClock, ScenarioRuntime
from sim.scripts.mujoco import tracking_native_acceptance as tracking
from sim.scripts.mujoco.formal_feeder import (
    _KinematicEntityConfig,
    _parser,
    _run_plan_runtime_config,
    _ScenarioFeed,
    _Services,
)

ROOT = Path(__file__).resolve().parents[2]


def test_tracking_run_plan_exposes_one_dynamic_mujoco_person() -> None:
    plan = compile_run_plan(
        "tracking",
        "sim",
        robot="doso/thunder_v4",
        env_config={"backend": "mujoco"},
    )
    config = _run_plan_runtime_config(_parser().parse_args([]), plan)
    runtime = ScenarioRuntime.from_plan(config.scenario_plan)
    startup_budget_s = sum(
        max(process.timeout_s for process in plan.processes if process.order == order)
        for order in {process.order for process in plan.processes}
    )
    initial = runtime.snapshot(
        ScenarioClock(
            session_id=config.session_id,
            model_generation=0,
            reset_generation=0,
            sim_time_ns=int(startup_budget_s * 1_000_000_000),
        )
    )
    moved = runtime.snapshot(
        ScenarioClock(
            session_id=config.session_id,
            model_generation=0,
            reset_generation=0,
            sim_time_ns=int((startup_budget_s + 30.0) * 1_000_000_000),
        )
    )

    assert [entity.entity_id for entity in config.kinematic_entities] == ["person_01"]
    assert moved.entities[-1].transform.position_m[0] - initial.entities[-1].transform.position_m[0] >= 0.5


def _scenario_plan() -> dict[str, object]:
    return {
        "schema": "lingtu.sim.scenario-plan.v1",
        "session_id": "dynamic-follow",
        "env": "sim",
        "backend": "mujoco",
        "package": {
            "id": "industrial_park_walking_person",
            "version": "1.0.0",
            "kind": "scenario",
            "manifest": "scenario.package.yaml",
        },
        "model_generation": 0,
        "reset_generation": 0,
        "seed": 1,
        "clock": {"unit": "ns", "source": "mujoco_sim_time", "sim_time_ns": 0},
        "authority_policy": {
            "robot_physics_owner": "mujoco",
            "dynamic_behavior_owner": "scenario",
            "visual_animation_owner": "ue_animation",
        },
        "entities": [
            {
                "entity_id": "person_01",
                "entity_type": "pedestrian",
                "authority": "scenario",
                "source_epoch": 0,
                "initial_transform": {
                    "position_m": [6.0, 4.0, 0.0],
                    "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                },
                "physics_proxy": {
                    "mode": "kinematic",
                    "body_stable_id": "person_01/proxy_root",
                },
                "semantic_class": "person",
                "behavior": {
                    "profile": "linear_crossing",
                    "seed": 1,
                    "parameters": {
                        "start_time_s": 0.0,
                        "duration_s": 10.0,
                        "speed_mps": 0.2,
                        "end_position_m": [8.0, 4.0, 0.0],
                    },
                },
            }
        ],
    }


def test_one_scenario_snapshot_moves_mujoco_and_sim_perception(tmp_path: Path) -> None:
    world = ROOT / "sim" / "packages" / "worlds" / "open_field" / "1.1.0" / "physics" / "open_field.xml"
    proxy = (
        ROOT
        / "sim"
        / "packages"
        / "scenarios"
        / "industrial_park_walking_person"
        / "physics"
        / "person_capsule.xml"
    )
    staged_world = tmp_path / "world.xml"
    staged_world.write_bytes(world.read_bytes())
    entity = _KinematicEntityConfig(
        entity_id="person_01",
        package_root="unused",
        model="unused",
        attach_root="proxy_root",
        position_m=(6.0, 4.0, 0.0),
        quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
    )
    composed = _Services.compose_scenario_world(staged_world, ((entity, proxy),))

    engine = MuJoCoEngine(drive_mode="kinematic")
    engine._model = mujoco.MjModel.from_xml_path(str(composed))
    engine._data = mujoco.MjData(engine._model)
    runtime = ScenarioRuntime.from_plan(_scenario_plan())
    snapshot = runtime.snapshot(
        ScenarioClock(
            session_id="dynamic-follow",
            model_generation=0,
            reset_generation=0,
            sim_time_ns=5_000_000_000,
        )
    )

    assert engine.apply_scenario_snapshot(snapshot) == 1
    body_id = mujoco.mj_name2id(engine.model, mujoco.mjtObj.mjOBJ_BODY, "person_01__proxy_root")
    mocap_id = int(engine.model.body_mocapid[body_id])
    assert engine.data.mocap_pos[mocap_id].tolist() == [7.0, 4.0, 0.0]

    snapshot_path = tmp_path / "scenario.current.json"
    snapshot_path.write_text(json.dumps(snapshot.to_dict()), encoding="utf-8")
    observer = SimSceneObserver(
        scenario_entities=[
            {
                "entity_id": "person_01",
                "entity_type": "pedestrian",
                "semantic_class": "person",
                "initial_transform": {"position_m": [6.0, 4.0, 0.0]},
            }
        ],
        scenario_snapshot=snapshot_path,
    )
    camera_to_world = np.eye(4, dtype=np.float64)
    camera_to_world[:3, 3] = [3.0, 4.0, 1.0]
    observed = observer.observe(
        camera_to_world,
        SimpleNamespace(fx=300.0, fy=300.0, cx=320.0, cy=240.0, width=640, height=480),
    )

    person = next(item for item in observed if item.track_id == "person_01")
    assert person.position.tolist() == [7.0, 4.0, 0.0]


def test_scenario_feed_publishes_the_snapshot_applied_to_physics(tmp_path: Path) -> None:
    runtime = ScenarioRuntime.from_plan(_scenario_plan())
    applied = []
    published = []
    engine = SimpleNamespace(
        sim_time=0.0,
        apply_scenario_snapshot=lambda snapshot: applied.append(snapshot),
    )
    services = SimpleNamespace(
        publish_scenario=lambda root, payload: published.append((root, payload)),
    )
    feed = _ScenarioFeed(
        engine=engine,
        runtime=runtime,
        services=services,
        session_root=tmp_path,
        config=SimpleNamespace(
            session_id="dynamic-follow",
            model_generation=0,
            reset_generation=0,
        ),
    )

    feed.start()
    engine.sim_time = 4.95
    feed.before_step(0.05)
    feed.before_step(0.05)

    assert published[-1][1] == applied[-1].to_dict()
    assert applied[-1].entities[0].transform.position_m == (7.0, 4.0, 0.0)


def test_tracking_acceptance_requires_motion_with_active_native_navigation(
    monkeypatch,
) -> None:
    person_id = "track_person_01"
    initial = {
        "state": "following",
        "target_visible": True,
        "navigation_state": "path_active",
        "navigation_task_id": "task-1",
        "robot_position": [3.0, 4.0, 0.0],
        "goal_position": [4.5, 4.0, 0.0],
        "person": {"id": person_id, "position": [6.0, 4.0, 0.0]},
    }
    states = iter(
        [
            {
                "state": "following",
                "target_visible": True,
                "navigation_state": "planning",
                "navigation_task_id": "task-1",
                "robot_position": [3.05, 4.0, 0.0],
                "goal_position": [4.6, 4.0, 0.0],
                "person": {"id": person_id, "position": [6.1, 4.0, 0.0]},
            },
            {
                "state": "following",
                "target_visible": True,
                "navigation_state": "path_active",
                "navigation_task_id": "task-2",
                "robot_position": [3.3, 4.0, 0.0],
                "goal_position": [4.8, 4.0, 0.0],
                "person": {"id": person_id, "position": [6.3, 4.0, 0.0]},
            },
        ]
    )
    client = SimpleNamespace(
        _get=lambda _path: {"visual_servo": next(states)},
    )
    monkeypatch.setattr(tracking.time, "sleep", lambda _seconds: None)

    result = tracking._poll_person_motion(
        client,
        person_id,
        initial,
        timeout_s=1.0,
        interval_s=0.01,
        minimum_motion_m=0.2,
        minimum_robot_motion_m=0.2,
        minimum_goal_motion_m=0.2,
        maximum_distance_growth_m=0.1,
    )

    assert result["person_motion_m"] == pytest.approx(0.3)
    assert result["robot_motion_m"] == pytest.approx(0.3)
    assert result["goal_motion_m"] == pytest.approx(0.3)
    assert result["end_distance_m"] == pytest.approx(3.0)
    assert result["navigation_tasks_observed"] == 2
    assert result["navigation_state"] == "path_active"
