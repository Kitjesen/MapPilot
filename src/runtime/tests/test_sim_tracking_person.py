from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
from sim.runtime.scenario.runtime import parse_scenario_plan

from decision.modules.visual_servo import VisualServoModule
from lingtu.assembly.compiler import compile_run_plan
from perception.detection.sim_scene_observer import SimSceneObserver
from perception.services import DetectionService
from runtime.msgs.geometry import Pose, PoseStamped, Vector3


def test_tracking_sim_uses_walking_person_scenario() -> None:
    plan = compile_run_plan(
        "tracking",
        "sim",
        robot="doso/thunder_v4",
        env_config={"backend": "mujoco"},
    )

    assert plan.simulation["session_source"] == "sim/presets/doso/thunder_v4/tracking.yaml"
    assert plan.simulation["session"]["world"] == "industrial_park@1.0.0"
    assert plan.simulation["session"]["scenario"] == "industrial_park_walking_person@1.0.0"
    assert [
        entity["entity_id"] for entity in plan.host_config["scenario_entities"]
    ] == ["person_01"]
    assert any(
        entity["entity_id"] == "person_01"
        and entity["semantic_class"] == "person"
        for entity in plan.host_config["scenario_entities"]
    )
    scenario = parse_scenario_plan(plan.simulation["scenario_plan"])
    assert scenario.mujoco_kinematic_bindings == (("person_01", "person_01/proxy_root"),)


def test_sim_person_can_be_selected_by_id_and_publish_a_goal() -> None:
    plan = compile_run_plan(
        "tracking",
        "sim",
        robot="doso/thunder_v4",
        env_config={"backend": "mujoco"},
    )
    observer = SimSceneObserver(
        world=Path(plan.simulation["physics_plan"]["world"]["mjcf"]).name,
        scenario_entities=plan.host_config["scenario_entities"],
    )
    intrinsics = SimpleNamespace(
        fx=300.0,
        fy=300.0,
        cx=320.0,
        cy=240.0,
        width=640,
        height=480,
    )
    camera_to_world = np.eye(4, dtype=np.float64)
    camera_to_world[:3, 3] = [3.0, 4.0, 1.0]

    observed = observer.observe(camera_to_world, intrinsics)
    person = next(item for item in observed if item.label == "person")
    assert person.track_id == "person_01"

    detections = DetectionService().convert_to_core_detections(
        [person],
        source_ts=42.0,
    )
    assert detections[0].id == "track_person_01"
    assert detections[0].ts == 42.0

    servo = VisualServoModule()
    servo.setup()
    goals = []
    servo.goal_pose._add_callback(goals.append)
    servo.robot_pose._deliver(
        PoseStamped(
            pose=Pose(position=Vector3(3.0, 4.0, 0.0)),
            ts=42.0,
            frame_id="map",
        )
    )
    servo._on_servo_target("follow_id:track_person_01")
    servo.detections_3d._deliver(detections)

    assert len(goals) == 1
    assert goals[0].frame_id == "map"
    status = servo.get_servo_status()
    assert status["target_id"] == "track_person_01"
    assert status["person"]["id"] == "track_person_01"
