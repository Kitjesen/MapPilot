from __future__ import annotations

import json

import pytest

pytest.importorskip("mujoco")

from sim.scripts.mujoco.lift_transition_acceptance import (
    run_lift_transition_acceptance,
)


def test_ros_free_mujoco_lift_acceptance_runs_full_building_mission(tmp_path) -> None:
    report = run_lift_transition_acceptance(
        output_dir=tmp_path / "lift_acceptance",
        step_s=0.10,
        timeout_s=45.0,
    )

    assert report["passed"] is True, report
    assert report["ros_required"] is False
    assert report["open_rmf_required"] is False
    assert report["native_dds_contract_preserved"] is True
    assert report["thunder_robot_model_loaded"] is True
    assert report["mujoco_lift_joint_motion_verified"] is True
    assert report["kinematic_rider_coupling"] is True
    assert report["physical_gait_verified"] is False
    assert report["real_localization_verified"] is False
    assert report["final_mission_phase"] == "SUCCEEDED"
    assert report["final_floor"] == {
        "building_id": "factory-a",
        "floor_id": "floor-2",
        "map_id": "factory-a-floor-2",
    }
    assert report["lift_released"] is True
    assert report["floor_switch_count"] == 1
    assert report["goal_request_ids"] == [
        "mujoco-lift-acceptance:lift:approach",
        "mujoco-lift-acceptance:lift:enter",
        "mujoco-lift-acceptance:lift:exit",
        "mujoco-lift-acceptance:goal",
    ]
    assert {
        "APPROACH_SOURCE",
        "WAIT_SOURCE_DOOR",
        "ENTER_CABIN",
        "RIDE",
        "VERIFY_TARGET_LOCALIZATION",
        "EXIT_CABIN",
        "SUCCEEDED",
    } <= set(report["lift_phases_seen"])

    persisted = json.loads((tmp_path / "lift_acceptance" / "report.json").read_text())
    assert persisted["passed"] is True


def test_ros_free_mujoco_lift_acceptance_runs_reverse_building_mission(tmp_path) -> None:
    report = run_lift_transition_acceptance(
        output_dir=tmp_path / "lift_acceptance_reverse",
        direction="down",
        step_s=0.10,
        timeout_s=45.0,
    )

    assert report["passed"] is True, report
    assert report["direction"] == "down"
    assert report["source_floor"]["floor_id"] == "floor-2"
    assert report["final_floor"] == {
        "building_id": "factory-a",
        "floor_id": "floor-1",
        "map_id": "factory-a-floor-1",
    }
    assert report["final_lift_floor_id"] == "floor-1"
    assert report["mujoco_lift_joint_motion_verified"] is True
    assert report["kinematic_rider_coupling"] is True
    assert report["lift_released"] is True
    assert report["floor_switch_count"] == 1
    assert report["final_mission_phase"] == "SUCCEEDED"
