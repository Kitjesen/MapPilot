from __future__ import annotations

import math
import re
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from sim.scripts.mujoco.teleop_avoid_native_acceptance import (
    FIELD_TELEOP_AVOID_PROFILE,
    SIMULATION_POSTURE_GATE,
    ResilientTeleopProcess,
    _binary_source_provenance,
    _requested_scenarios,
    _wait_for_policy_driving,
    assign_motion_phases,
    build_execution_plan,
    build_odom_prior_diagnostic_config,
    build_parser,
    build_scene_variant,
    cleanup_owned_pid_file,
    continuous_teleop_exit_blocker,
    evaluate_case,
    evaluate_simulation_posture,
    prepare_runtime,
    project_motion_timestamp,
    reclaim_prior_case_processes,
    reset_case_artifacts,
    run,
    signal_managed_process,
    typed_teleop_delivery_blocker,
)


def test_binary_provenance_includes_native_client_shared_library(tmp_path) -> None:
    control = tmp_path / "lingtu_nav_control"
    client_library = tmp_path / "liblingtu_nav_client.so"
    control.write_bytes(b"control")
    client_library.write_bytes(b"client")

    provenance, blockers = _binary_source_provenance({"navigation_control": control})

    assert blockers == []
    dependency = provenance["navigation_control"]["runtime_dependencies"]
    assert dependency["lingtu_nav_client"]["path"] == str(client_library)
    assert len(dependency["lingtu_nav_client"]["sha256"]) == 64


def test_harness_field_profile_matches_deployed_service_contract() -> None:
    nav_service = Path("scripts/deploy/thunder/lingtu-nav-dds.service").read_text(encoding="utf-8")
    terrain_service = Path("scripts/deploy/thunder/lingtu-traversability-dds.service").read_text(encoding="utf-8")
    expected_nav = {
        "LINGTU_NAV_ODOM_MAX_AGE_S": "odom_max_age_s",
        "LINGTU_NAV_TF_MAX_AGE_S": "tf_max_age_s",
        "LINGTU_NAV_CLOUD_MAX_AGE_S": "cloud_max_age_s",
        "LINGTU_NAV_CLOUD_POSE_MAX_GAP_S": "cloud_pose_max_gap_s",
        "LINGTU_NAV_LOCALIZATION_HEALTH_MAX_AGE_S": "localization_health_max_age_s",
        "LINGTU_NAV_INPUT_RECOVERY_FRAMES": "input_recovery_frames",
    }
    for environment_name, profile_name in expected_nav.items():
        match = re.search(rf"Environment={environment_name}=([^\r\n]+)", nav_service)
        assert match is not None
        assert float(match.group(1)) == pytest.approx(float(FIELD_TELEOP_AVOID_PROFILE[profile_name]))
    expected_terrain = {
        "LINGTU_TRAVERSABILITY_PUBLISH_HZ": "traversability_publish_hz",
        "LINGTU_TRAVERSABILITY_TERRAIN_MAP_HZ": "traversability_slow_hz",
        "LINGTU_TRAVERSABILITY_TICK_HZ": "traversability_tick_hz",
        "LINGTU_TRAVERSABILITY_CLOUD_POSE_MAX_GAP_S": ("traversability_cloud_pose_max_gap_s"),
        "LINGTU_TRAVERSABILITY_OBSERVED_FREE_TTL_S": "observed_free_ttl_s",
    }
    for environment_name, profile_name in expected_terrain.items():
        match = re.search(rf"Environment={environment_name}=([^\r\n]+)", terrain_service)
        assert match is not None
        assert float(match.group(1)) == pytest.approx(float(FIELD_TELEOP_AVOID_PROFILE[profile_name]))


def _nav_sample(
    *,
    phase: str,
    reason: str,
    output_vx: float,
    gate_ready: bool = True,
    gate_reason: str = "ready",
    command_count: int = 1,
) -> dict:
    return {
        "phase": phase,
        "wall_s": float(command_count),
        "nav": {
            "control_mode": "teleop_avoid",
            "input_gate": {
                "ready": gate_ready,
                "recovering": gate_reason == "recovering",
                "reason": gate_reason,
            },
            "teleop": {
                "seen": True,
                "reason": reason,
                "request": {"vx": 0.18, "vy": 0.0, "wz": 0.0},
                "output": {"vx": output_vx, "vy": 0.0, "wz": 0.0},
            },
            "final_cmd_vel": {"vx": output_vx, "vy": 0.0, "wz": 0.0},
            "counters": {"teleop_cmd": command_count},
        },
    }


def _motion_posture_sample(
    *,
    z: float = 0.48,
    roll_rad: float = 0.0,
    pitch_rad: float = 0.0,
    phase: str = "steady",
) -> dict:
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    # Quaternion for yaw=0 using MuJoCo qpos order qw, qx, qy, qz.
    quaternion = [cr * cp, sr * cp, cr * sp, -sr * sp]
    return {
        "phase": phase,
        "driving": True,
        "qpos": [0.0, 0.0, z, *quaternion],
        "cmd": [0.0, 0.0, 0.0],
        "x": 0.0,
        "y": 0.0,
    }


def test_simulation_posture_gate_uses_motion_qpos() -> None:
    samples = [_motion_posture_sample(z=0.46, roll_rad=0.10, pitch_rad=-0.12) for _ in range(4)]

    result = evaluate_simulation_posture(samples)

    assert result["ok"] is True
    assert result["evaluated"] is True
    assert result["evidence_origin"] == "motion"
    assert result["sources"] == ["motion.qpos"]
    assert result["min_base_z_m"] == pytest.approx(0.46)
    assert result["thresholds"] == SIMULATION_POSTURE_GATE


def test_simulation_posture_gate_falls_back_to_timeline_posture() -> None:
    timeline = [
        {
            "phase": "steady",
            "nav": {
                "simulation_posture": {
                    "base_z_m": 0.48,
                    "roll_rad": 0.0,
                    "pitch_rad": 0.90,
                }
            },
        }
        for _ in range(3)
    ]

    result = evaluate_simulation_posture([], timeline)

    assert result["ok"] is False
    assert result["reason"] == "simulation_posture_invalid"
    assert result["evidence_origin"] == "timeline"
    assert result["longest_invalid_run"] == 3
    assert result["violations"] == ["pitch_exceeds_limit"]


def test_invalid_simulation_posture_preempts_terrain_attribution() -> None:
    nav_samples = [
        _nav_sample(
            phase="steady",
            reason="terrain_stop",
            output_vx=0.0,
            command_count=index,
        )
        for index in range(1, 6)
    ]
    motion_samples = [
        _motion_posture_sample(),
        *[_motion_posture_sample(z=0.22) for _ in range(3)],
    ]

    result = evaluate_case(
        "terrain_hard",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
    )

    assert result["ok"] is False
    assert result["blockers"] == ["simulation_posture_invalid"]
    assert result["failure"]["reason"] == "simulation_posture_invalid"
    assert result["policy_attribution"] == {
        "evaluated": False,
        "reason": "simulation_posture_invalid",
    }
    assert result["metrics"]["teleop_reasons"] == []
    assert result["simulation_posture"]["fall_detected"] is True
    assert "base_z_below_min" in result["simulation_posture"]["violations"]


def test_free_case_proves_ready_native_teleop_and_policy_motion() -> None:
    nav_samples = [
        _nav_sample(
            phase="steady",
            reason="accepted",
            output_vx=0.18,
            command_count=index,
        )
        for index in range(1, 6)
    ]
    motion_samples = [
        {
            "phase": "steady",
            "driving": True,
            "cmd": [0.18, 0.0, 0.0],
            "x": index * 0.04,
            "y": 0.0,
        }
        for index in range(6)
    ]

    result = evaluate_case(
        "free",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
    )

    assert result["ok"] is True
    assert result["blockers"] == []
    assert result["metrics"]["policy_motion_xy_m"] >= 0.20
    assert result["metrics"]["steady_nonzero_cmd_samples"] == 6


def test_free_case_rejects_pulsed_motion_and_any_false_hazard_reason() -> None:
    nav_samples = [
        _nav_sample(
            phase="steady",
            reason="accepted" if index < 2 else "terrain_stop",
            output_vx=0.18 if index < 2 else 0.0,
            command_count=index,
        )
        for index in range(1, 7)
    ]
    motion_samples = [
        {
            "phase": "steady",
            "driving": True,
            "cmd": [0.18 if index < 2 else 0.0, 0.0, 0.0],
            "x": index * 0.03,
            "y": 0.0,
        }
        for index in range(6)
    ]

    result = evaluate_case(
        "free",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
    )

    assert result["ok"] is False
    assert "free_hazard_decision_observed" in result["blockers"]
    assert "free_accepted_ratio_too_low" in result["blockers"]
    assert "free_output_scale_out_of_range" in result["blockers"]
    assert "free_nonzero_command_ratio_too_low" in result["blockers"]


def test_mode_evidence_is_independent_from_input_gate_readiness() -> None:
    nav_samples = [
        _nav_sample(
            phase="steady",
            reason="odom_stale",
            output_vx=0.0,
            gate_ready=False,
            gate_reason="odom_stale",
            command_count=index,
        )
        for index in range(1, 4)
    ]
    motion_samples = [_motion_posture_sample() for _ in range(3)]

    result = evaluate_case(
        "free",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
    )

    assert "input_gate_ready_missing" in result["blockers"]
    assert "teleop_avoid_mode_missing" not in result["blockers"]


def test_obstacle_slow_requires_auditable_scale_on_final_policy_command() -> None:
    nav_samples = [
        _nav_sample(
            phase="steady",
            reason="obstacle_slow",
            output_vx=0.063,
            command_count=index,
        )
        for index in range(1, 6)
    ]
    for sample in nav_samples:
        sample["nav"]["teleop"]["slowed"] = True
        sample["nav"]["teleop"]["obstacle_distance_m"] = 0.90
    motion_samples = [
        {
            "phase": "steady",
            "driving": True,
            "cmd": [0.063, 0.0, 0.0],
            "x": index * 0.02,
            "y": 0.0,
        }
        for index in range(6)
    ]

    result = evaluate_case(
        "obstacle_slow",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
    )

    assert result["ok"] is True
    assert result["metrics"]["median_policy_cmd_vx"] == 0.063
    assert result["metrics"]["median_output_scale"] == 0.35
    assert result["metrics"]["obstacle_distance_m"] == 0.90


def test_obstacle_stop_proves_zero_at_policy_while_teleop_requests_continue() -> None:
    nav_samples = [
        _nav_sample(
            phase="steady",
            reason="obstacle_stop",
            output_vx=0.0,
            command_count=index,
        )
        for index in range(1, 7)
    ]
    for sample in nav_samples:
        sample["nav"]["teleop"]["stopped"] = True
        sample["nav"]["teleop"]["obstacle_distance_m"] = 0.42
    motion_samples = [
        {
            "phase": "steady",
            "driving": True,
            "cmd": [0.0, 0.0, 0.0],
            "x": 0.002,
            "y": -0.001,
        }
        for _ in range(8)
    ]

    result = evaluate_case(
        "obstacle_stop",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
    )

    assert result["ok"] is True
    assert result["metrics"]["steady_zero_cmd_samples"] == 8
    assert result["metrics"]["teleop_command_count_delta"] == 5
    assert result["metrics"]["policy_motion_xy_m"] == 0.0


def test_terrain_soft_is_labeled_as_injected_dds_consumer_evidence() -> None:
    nav_samples = [
        _nav_sample(
            phase="steady",
            reason="terrain_slow",
            output_vx=0.063,
            command_count=index,
        )
        for index in range(1, 6)
    ]
    for sample in nav_samples:
        sample["nav"]["teleop"]["slowed"] = True
        sample["nav"]["teleop"]["traversability_cost"] = 50.0
    motion_samples = [
        {
            "phase": "steady",
            "driving": True,
            "cmd": [0.063, 0.0, 0.0],
            "x": index * 0.02,
            "y": 0.0,
        }
        for index in range(6)
    ]

    result = evaluate_case(
        "terrain_soft",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
        injected=True,
    )

    assert result["ok"] is True
    assert result["evidence_scope"] == "dds_consumer_contract_injected"
    assert result["producer_e2e"] is False
    assert result["metrics"]["traversability_cost"] == 50.0


def test_terrain_hard_injected_contract_requires_zero_policy_command() -> None:
    nav_samples = [
        _nav_sample(
            phase="steady",
            reason="terrain_stop",
            output_vx=0.0,
            command_count=index,
        )
        for index in range(1, 6)
    ]
    for sample in nav_samples:
        sample["nav"]["teleop"]["stopped"] = True
        sample["nav"]["teleop"]["traversability_cost"] = 100.0
    motion_samples = [
        {
            "phase": "steady",
            "driving": True,
            "cmd": [0.0, 0.0, 0.0],
            "x": 0.0,
            "y": 0.0,
        }
        for _ in range(6)
    ]

    result = evaluate_case(
        "terrain_hard",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
        injected=True,
    )

    assert result["ok"] is True
    assert result["evidence_scope"] == "dds_consumer_contract_injected"
    assert result["metrics"]["traversability_cost"] == 100.0
    assert result["metrics"]["steady_zero_cmd_samples"] == 6


def test_terrain_hard_rejects_a_nonzero_policy_command() -> None:
    nav_samples = [
        _nav_sample(
            phase="steady",
            reason="terrain_stop",
            output_vx=0.0,
            command_count=index,
        )
        for index in range(1, 5)
    ]
    for sample in nav_samples:
        sample["nav"]["teleop"]["traversability_cost"] = 100.0
    motion_samples = [
        {
            "phase": "steady",
            "driving": True,
            "cmd": [0.10, 0.0, 0.0],
            "x": index * 0.03,
            "y": 0.0,
        }
        for index in range(5)
    ]

    result = evaluate_case(
        "terrain_hard",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
        injected=True,
    )

    assert result["ok"] is False
    assert "terrain_hard_zero_command_missing" in result["blockers"]


def test_traversability_dropout_recovers_only_after_zero_and_recovery_hysteresis() -> None:
    nav_samples = [
        _nav_sample(
            phase="baseline",
            reason="accepted",
            output_vx=0.18,
            command_count=1,
        ),
        _nav_sample(
            phase="baseline",
            reason="accepted",
            output_vx=0.18,
            command_count=2,
        ),
        _nav_sample(
            phase="dropout_grace",
            reason="accepted",
            output_vx=0.18,
            command_count=3,
        ),
        _nav_sample(
            phase="dropout",
            reason="traversability_stale",
            output_vx=0.0,
            gate_ready=False,
            gate_reason="traversability_stale",
            command_count=4,
        ),
        _nav_sample(
            phase="dropout",
            reason="traversability_stale",
            output_vx=0.0,
            gate_ready=False,
            gate_reason="traversability_stale",
            command_count=6,
        ),
        _nav_sample(
            phase="recovery",
            reason="recovering",
            output_vx=0.0,
            gate_ready=False,
            gate_reason="recovering",
            command_count=7,
        ),
        _nav_sample(
            phase="recovery",
            reason="accepted",
            output_vx=0.18,
            gate_ready=True,
            gate_reason="ready",
            command_count=9,
        ),
    ]
    motion_samples = [
        {"phase": "baseline", "driving": True, "cmd": [0.18, 0.0, 0.0], "x": 0.00, "y": 0.0},
        {"phase": "baseline", "driving": True, "cmd": [0.18, 0.0, 0.0], "x": 0.04, "y": 0.0},
        {"phase": "dropout_grace", "driving": True, "cmd": [0.18, 0.0, 0.0], "x": 0.07, "y": 0.0},
        {"phase": "dropout", "driving": True, "cmd": [0.0, 0.0, 0.0], "x": 0.04, "y": 0.0},
        {"phase": "dropout", "driving": True, "cmd": [0.0, 0.0, 0.0], "x": 0.04, "y": 0.0},
        {"phase": "recovery", "driving": True, "cmd": [0.0, 0.0, 0.0], "x": 0.04, "y": 0.0},
        {"phase": "recovery", "driving": True, "cmd": [0.18, 0.0, 0.0], "x": 0.08, "y": 0.0},
    ]

    result = evaluate_case(
        "traversability_dropout_recovery",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
    )

    assert result["ok"] is True
    assert result["metrics"]["dropout_reason"] == "traversability_stale"
    assert result["metrics"]["dropout_zero_cmd_samples"] == 2
    assert result["metrics"]["recovery_sequence"] == ["recovering", "ready"]
    assert result["metrics"]["teleop_command_count_delta"] == 8
    assert result["metrics"]["dropout_teleop_command_count_delta"] == 2


def test_correlated_slam_dropout_proves_all_streams_stale_and_recovers() -> None:
    nav_samples = [
        _nav_sample(
            phase="baseline",
            reason="accepted",
            output_vx=0.18,
            command_count=1,
        ),
        _nav_sample(
            phase="baseline",
            reason="accepted",
            output_vx=0.18,
            command_count=2,
        ),
        _nav_sample(
            phase="dropout",
            reason="odom_stale",
            output_vx=0.0,
            gate_ready=False,
            gate_reason="odom_stale",
            command_count=4,
        ),
        _nav_sample(
            phase="dropout",
            reason="odom_stale",
            output_vx=0.0,
            gate_ready=False,
            gate_reason="odom_stale",
            command_count=6,
        ),
        _nav_sample(
            phase="recovery",
            reason="recovering",
            output_vx=0.0,
            gate_ready=False,
            gate_reason="recovering",
            command_count=7,
        ),
        _nav_sample(
            phase="recovery",
            reason="accepted",
            output_vx=0.18,
            gate_ready=True,
            gate_reason="ready",
            command_count=9,
        ),
    ]
    for sample in nav_samples:
        sample["nav"]["counters"].update({"odom": 20, "tf": 21, "registered_clouds": 19})
    for sample in nav_samples[2:4]:
        sample["nav"]["input_gate"].update(
            {
                "odom_age_s": 0.8,
                "odom_max_age_s": 0.25,
                "tf_age_s": 0.8,
                "tf_max_age_s": 0.25,
                "cloud_age_s": 0.8,
                "cloud_max_age_s": 0.35,
                "localization_health_age_s": 0.8,
                "localization_health_max_age_s": 0.5,
            }
        )
    motion_samples = [
        {"phase": "baseline", "driving": True, "cmd": [0.18, 0.0, 0.0]},
        {"phase": "baseline", "driving": True, "cmd": [0.18, 0.0, 0.0]},
        {"phase": "dropout", "driving": True, "cmd": [0.0, 0.0, 0.0]},
        {"phase": "dropout", "driving": True, "cmd": [0.0, 0.0, 0.0]},
        {"phase": "recovery", "driving": True, "cmd": [0.0, 0.0, 0.0]},
        {"phase": "recovery", "driving": True, "cmd": [0.18, 0.0, 0.0]},
    ]

    result = evaluate_case(
        "slam_inputs_dropout_recovery",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
    )

    assert result["ok"] is True
    assert result["metrics"]["dropout_reason"] == "odom_stale"
    assert result["metrics"]["correlated_slam_streams_stale"] == [
        "odom",
        "tf",
        "registered_cloud",
        "localization_health",
    ]
    assert result["metrics"]["correlated_slam_counter_deltas"] == {
        "odom": 0,
        "tf": 0,
        "registered_cloud": 0,
    }
    assert result["metrics"]["input_gate_generation_recovery_proven"] is True


def test_correlated_slam_dropout_rejects_an_advancing_stream() -> None:
    def sample(command_count: int, odom_count: int) -> dict:
        value = _nav_sample(
            phase="dropout",
            reason="odom_stale",
            output_vx=0.0,
            gate_ready=False,
            gate_reason="odom_stale",
            command_count=command_count,
        )
        value["nav"]["counters"].update({"odom": odom_count, "tf": 4, "registered_clouds": 5})
        value["nav"]["input_gate"].update(
            {
                "odom_age_s": 0.8,
                "odom_max_age_s": 0.25,
                "tf_age_s": 0.8,
                "tf_max_age_s": 0.25,
                "cloud_age_s": 0.8,
                "cloud_max_age_s": 0.35,
                "localization_health_age_s": 0.8,
                "localization_health_max_age_s": 0.5,
            }
        )
        return value

    nav_samples = [sample(2, 7), sample(4, 8)]
    result = evaluate_case(
        "slam_inputs_dropout_recovery",
        nav_samples=nav_samples,
        motion_samples=[
            {"phase": "dropout", "driving": True, "cmd": [0.0, 0.0, 0.0]},
            {"phase": "dropout", "driving": True, "cmd": [0.0, 0.0, 0.0]},
        ],
        command_vx=0.18,
    )

    assert "slam_dropout_stream_advanced:odom" in result["blockers"]


def test_dropout_rejects_nonzero_final_command_during_recovery_hysteresis() -> None:
    nav_samples = [
        _nav_sample(
            phase="baseline",
            reason="accepted",
            output_vx=0.18,
            command_count=1,
        ),
        _nav_sample(
            phase="dropout",
            reason="traversability_stale",
            output_vx=0.0,
            gate_ready=False,
            gate_reason="traversability_stale",
            command_count=3,
        ),
        _nav_sample(
            phase="dropout",
            reason="traversability_stale",
            output_vx=0.0,
            gate_ready=False,
            gate_reason="traversability_stale",
            command_count=5,
        ),
        _nav_sample(
            phase="recovery",
            reason="recovering",
            output_vx=0.05,
            gate_ready=False,
            gate_reason="recovering",
            command_count=6,
        ),
        _nav_sample(
            phase="recovery",
            reason="accepted",
            output_vx=0.18,
            gate_ready=True,
            gate_reason="ready",
            command_count=8,
        ),
    ]
    motion_samples = [
        {"phase": "baseline", "driving": True, "cmd": [0.18, 0.0, 0.0]},
        {"phase": "dropout", "driving": True, "cmd": [0.0, 0.0, 0.0]},
        {"phase": "dropout", "driving": True, "cmd": [0.0, 0.0, 0.0]},
        {"phase": "recovery", "driving": True, "cmd": [0.0, 0.0, 0.0]},
        {"phase": "recovery", "driving": True, "cmd": [0.18, 0.0, 0.0]},
    ]

    result = evaluate_case(
        "traversability_dropout_recovery",
        nav_samples=nav_samples,
        motion_samples=motion_samples,
        command_vx=0.18,
    )

    assert result["ok"] is False
    assert "dropout_recovery_nonzero_before_ready" in result["blockers"]


def test_terrain_producer_scene_variants_use_the_declared_low_steps(tmp_path: Path) -> None:
    base = tmp_path / "base.xml"
    base.write_text(
        '<mujoco model="base"><worldbody><geom name="floor" type="plane"/></worldbody></mujoco>',
        encoding="utf-8",
    )

    soft = build_scene_variant(base, tmp_path / "soft.xml", "terrain_soft")
    hard = build_scene_variant(base, tmp_path / "hard.xml", "terrain_hard")

    soft_geom = ET.parse(soft).getroot().find("./worldbody/geom[@name='acceptance_terrain_soft']")
    hard_geom = ET.parse(hard).getroot().find("./worldbody/geom[@name='acceptance_terrain_hard']")
    assert soft_geom is not None
    assert soft_geom.attrib["pos"] == "1.55 0 0.05"
    assert soft_geom.attrib["size"] == "0.65 0.72 0.05"
    assert hard_geom is not None
    assert hard_geom.attrib["pos"] == "1.55 0 0.12"
    assert hard_geom.attrib["size"] == "0.65 0.72 0.12"


def test_interactive_obstacle_stop_scene_has_demo_clearance_without_leaving_path(tmp_path: Path) -> None:
    base = tmp_path / "base.xml"
    base.write_text(
        '<mujoco model="base"><worldbody><geom name="floor" type="plane"/></worldbody></mujoco>',
        encoding="utf-8",
    )

    scene = build_scene_variant(base, tmp_path / "demo_stop.xml", "obstacle_stop_demo")
    obstacle = ET.parse(scene).getroot().find("./worldbody/geom[@name='acceptance_obstacle_stop_demo']")

    assert obstacle is not None
    pos = [float(value) for value in obstacle.attrib["pos"].split()]
    size = [float(value) for value in obstacle.attrib["size"].split()]
    conservative_robot_front_x_m = 0.50
    minimum_demo_clearance_m = 1.10
    obstacle_near_face_x_m = pos[0] - size[0]

    assert obstacle_near_face_x_m - conservative_robot_front_x_m >= minimum_demo_clearance_m
    assert abs(pos[1]) <= size[1]


def test_odom_prior_diagnostic_derives_config_without_mutating_product_default(
    tmp_path: Path,
) -> None:
    base = tmp_path / "slam.yaml"
    base.write_text(
        "backend: fastlio2\nodom_prior_enabled: false\n",
        encoding="utf-8",
    )

    derived = build_odom_prior_diagnostic_config(
        base,
        tmp_path / "diagnostic" / "slam.yaml",
    )

    assert "odom_prior_enabled: false" in base.read_text(encoding="utf-8")
    assert "odom_prior_enabled: true" in derived.read_text(encoding="utf-8")


def test_execution_plan_is_the_real_native_teleop_avoid_policy_chain(tmp_path: Path) -> None:
    binaries = {
        name: tmp_path / name
        for name in (
            "slam",
            "traversability",
            "navigation",
            "navigation_control",
            "sensor_publisher",
            "cmd_vel_tap",
        )
    }
    paths = {
        "slam": tmp_path / "map.pcd",
        "slam_config": tmp_path / "slam.yaml",
        "policy": tmp_path / "policy.onnx",
        "sensor_runner": tmp_path / "native_dds_sensors.py",
        "world": tmp_path / "scene.xml",
    }

    plan = build_execution_plan(
        scenario="free",
        domain_id=226,
        binaries=binaries,
        paths=paths,
        case_dir=tmp_path / "case",
        duration_s=8.0,
        warmup_s=4.0,
        command_vx=0.18,
        manifest={},
    )

    by_name = {item["name"]: item for item in plan["processes"]}
    assert list(by_name) == ["slam", "traversability", "navigation", "sensor"]
    assert "--control-mode" in by_name["navigation"]["command"]
    assert "teleop_avoid" in by_name["navigation"]["command"]
    assert "--command-source" in by_name["sensor"]["command"]
    assert "dds" in by_name["sensor"]["command"]
    assert "--drive-mode" in by_name["sensor"]["command"]
    assert "policy" in by_name["sensor"]["command"]
    assert "--terrain-soft-height-m" in by_name["traversability"]["command"]
    assert "0.08" in by_name["traversability"]["command"]
    assert "--terrain-hard-height-m" in by_name["traversability"]["command"]
    assert "0.20" in by_name["traversability"]["command"]
    slow_index = by_name["traversability"]["command"].index("--slow-hz")
    assert by_name["traversability"]["command"][slow_index + 1] == "5"
    terrain_tick_index = by_name["traversability"]["command"].index("--tick-hz")
    assert by_name["traversability"]["command"][terrain_tick_index + 1] == "50"
    track_index = by_name["slam"]["command"].index("--track-against-map-period-s")
    assert by_name["slam"]["command"][track_index + 1] == "5.0"
    nav_command = by_name["navigation"]["command"]
    for option, expected in {
        "--odom-max-age-s": "0.25",
        "--tf-max-age-s": "0.25",
        "--cloud-max-age-s": "0.35",
        "--cloud-pose-max-gap-s": "0.1",
        "--localization-health-max-age-s": "0.5",
        "--input-recovery-frames": "3",
    }.items():
        option_index = nav_command.index(option)
        assert nav_command[option_index + 1] == expected
    assert "teleop" in plan["teleop_command"]
    assert "--duration-s" in plan["teleop_command"]
    teleop_timeout_index = plan["teleop_command"].index("--timeout-ms")
    assert plan["teleop_command"][teleop_timeout_index + 1] == "3000"
    teleop_duration_index = plan["teleop_command"].index("--duration-s")
    assert float(plan["teleop_command"][teleop_duration_index + 1]) >= 60.0
    assert plan["artifacts"]["motion_log"].endswith("motion.jsonl")
    assert plan["terrain_producer_contract"]["soft_cost"] == 40.0
    assert plan["terrain_producer_contract"]["hard_cost"] == 100.0
    assert plan["functional_scope"]["traversability_cost_in_decision"] is True

    obstacle_plan = build_execution_plan(
        scenario="obstacle_slow",
        domain_id=227,
        binaries=binaries,
        paths=paths,
        case_dir=tmp_path / "obstacle_case",
        duration_s=8.0,
        warmup_s=4.0,
        command_vx=0.18,
        manifest={},
    )
    obstacle_nav = next(item for item in obstacle_plan["processes"] if item["name"] == "navigation")["command"]
    use_cost_index = obstacle_nav.index("--use-traversability-cost")
    assert obstacle_nav[use_cost_index + 1] == "true"
    obstacle_traversability = next(item for item in obstacle_plan["processes"] if item["name"] == "traversability")[
        "command"
    ]
    obstacle_min_z_index = obstacle_traversability.index("--obstacle-min-z")
    soft_height_index = obstacle_traversability.index("--terrain-soft-height-m")
    soft_slope_index = obstacle_traversability.index("--terrain-soft-slope-deg")
    assert obstacle_traversability[obstacle_min_z_index + 1] == "2.00"
    assert obstacle_traversability[soft_height_index + 1] == "2.00"
    assert obstacle_traversability[soft_slope_index + 1] == "100.0"
    assert obstacle_plan["functional_scope"]["isolation"] == (
        "live_obstacle_decision_with_free_cost_producer_thresholds"
    )

    diagnostic_plan = build_execution_plan(
        scenario="free",
        domain_id=228,
        binaries=binaries,
        paths=paths,
        case_dir=tmp_path / "diagnostic_case",
        duration_s=8.0,
        warmup_s=4.0,
        command_vx=0.18,
        manifest={
            "_odom_prior_diagnostic": True,
            "sensor_runtime": {"publish_odom_prior": True},
        },
    )
    diagnostic_sensor = next(item for item in diagnostic_plan["processes"] if item["name"] == "sensor")["command"]
    assert "--publish-odom-prior" in diagnostic_sensor
    assert "--allow-kinematic-fastlio-acceptance" in diagnostic_sensor


def test_preflight_only_never_starts_a_scenario(tmp_path: Path) -> None:
    args = build_parser().parse_args(
        [
            "--scenario",
            "free",
            "--artifact-dir",
            str(tmp_path),
            "--preflight-only",
        ]
    )

    def prepare(_args: object) -> dict:
        return {
            "ok": True,
            "blockers": [],
            "manifest": {},
            "binaries": {},
            "paths": {},
            "details": {"source": "test"},
        }

    def execute(**_kwargs: object) -> dict:
        raise AssertionError("preflight-only must not execute a scenario")

    report = run(args, prepare_runtime_fn=prepare, execute_case_fn=execute)

    assert report["ok"] is True
    assert report["preflight"]["ok"] is True
    assert report["cases"] == []


def test_teleop_preflight_does_not_require_autonomy_planner_artifacts(
    tmp_path: Path,
    monkeypatch,
) -> None:
    from sim.scripts.mujoco import native_navigation_acceptance as native

    world = tmp_path / "scene.xml"
    world.write_text("<mujoco><worldbody/></mujoco>", encoding="utf-8")
    map_dir = tmp_path / "map"
    map_dir.mkdir()
    (map_dir / "map.pcd").write_text("pcd", encoding="utf-8")
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text("{}", encoding="utf-8")
    manifest = {
        "world": str(world),
        "map_dir": str(map_dir),
        "map_files": {"slam": "map.pcd"},
        "binaries": {},
    }
    binaries = {
        name: tmp_path / name
        for name in (
            "sensor_publisher",
            "slam",
            "traversability",
            "navigation",
            "navigation_control",
            "cmd_vel_tap",
        )
    }
    paths = {
        "world": world,
        "slam": map_dir / "map.pcd",
        "slam_config": tmp_path / "slam.yaml",
        "sensor_runner": tmp_path / "sensor.py",
        "policy": tmp_path / "policy.onnx",
    }
    monkeypatch.setattr(native, "_load_manifest", lambda _path: dict(manifest))
    monkeypatch.setattr(
        native,
        "_preflight",
        lambda _manifest: (
            binaries,
            paths,
            [
                "runtime_path_missing:path_library:/unused/paths",
                "map_artifact_missing:planner:/unused/octomap.ot",
                "map_artifact_missing:metadata:/unused/metadata.json",
                "native_binary_missing:autonomy_only_tool",
            ],
            {},
        ),
    )

    args = build_parser().parse_args(
        [
            "--manifest",
            str(manifest_path),
            "--artifact-dir",
            str(tmp_path / "artifacts"),
            "--preflight-only",
        ]
    )
    prepared = prepare_runtime(args)

    assert prepared["ok"] is True
    assert prepared["blockers"] == []
    assert len(prepared["details"]["out_of_scope_preflight_findings"]) == 4


def test_motion_samples_are_correlated_with_fault_phase_events() -> None:
    samples = [
        {"t": 99.0, "cmd": [0.0, 0.0, 0.0]},
        {"t": 100.5, "cmd": [0.18, 0.0, 0.0]},
        {"t": 102.5, "cmd": [0.0, 0.0, 0.0]},
        {"t": 104.5, "cmd": [0.18, 0.0, 0.0]},
    ]
    events = [
        {"phase": "baseline", "wall_s": 100.0},
        {"phase": "dropout", "wall_s": 102.0},
        {"phase": "recovery", "wall_s": 104.0},
    ]

    assigned = assign_motion_phases(samples, events)

    assert [sample["phase"] for sample in assigned] == [
        "warmup",
        "baseline",
        "dropout",
        "recovery",
    ]


def test_motion_phase_projection_uses_sim_hardware_rate_not_wall_clock() -> None:
    projected = project_motion_timestamp(
        last_sensor_timestamp_s=100.0,
        last_write_wall_s=200.0,
        event_wall_s=202.0,
        realtime_factor=0.5,
    )
    assigned = assign_motion_phases(
        [
            {"t": 100.9, "cmd": [0.18, 0.0, 0.0]},
            {"t": 101.1, "cmd": [0.0, 0.0, 0.0]},
        ],
        [{"phase": "dropout", "wall_s": 202.0, "motion_s": projected}],
    )

    assert projected == 101.0
    assert [sample["phase"] for sample in assigned] == ["warmup", "dropout"]


def test_managed_process_signal_supports_stop_and_cleanup_cont(monkeypatch) -> None:
    sent: list[int] = []
    monkeypatch.setattr(signal, "SIGSTOP", 19, raising=False)
    monkeypatch.setattr(signal, "SIGCONT", 18, raising=False)

    class Child:
        def poll(self) -> None:
            return None

        def send_signal(self, signum: int) -> None:
            sent.append(signum)

    class Managed:
        linux_pid = None
        pid_path = None
        process = Child()

    assert signal_managed_process(Managed(), "STOP") is True
    assert signal_managed_process(Managed(), "CONT") is True
    assert sent == [signal.SIGSTOP, signal.SIGCONT]


def test_fault_phase_waits_until_policy_driving_and_teleop_is_alive(
    tmp_path: Path,
) -> None:
    motion_log = tmp_path / "motion.jsonl"
    motion_log.write_text(
        '{"t":1.0,"driving":false}\n{"t":2.0,"driving":true}\n',
        encoding="utf-8",
    )

    class Alive:
        def poll(self) -> None:
            return None

    ready, reason = _wait_for_policy_driving(
        sensor=Alive(),
        teleop=Alive(),
        motion_log=motion_log,
        timeout_s=0.1,
    )

    assert (ready, reason) == (True, "driving")


def test_continuous_teleop_early_exit_is_a_hard_blocker() -> None:
    assert continuous_teleop_exit_blocker(None) == ""
    assert continuous_teleop_exit_blocker(1) == "continuous_teleop_exited_early:1"


def test_product_gate_rejects_retried_teleop_but_diagnostic_reports_it() -> None:
    delivery = {
        "retry_count": 1,
        "failure_reason_counts": {"teleop_source_stamp_stale": 1},
    }

    assert (
        typed_teleop_delivery_blocker(
            delivery,
            product_gate_eligible=True,
        )
        == "typed_teleop_delivery_unstable"
    )
    assert (
        typed_teleop_delivery_blocker(
            delivery,
            product_gate_eligible=False,
        )
        == ""
    )


def test_resilient_teleop_restarts_after_ack_timeout_and_stays_live(
    tmp_path: Path,
) -> None:
    created: list[object] = []

    class Attempt:
        def __init__(self, returncode: int | None, output: str) -> None:
            self.returncode = returncode
            self.output = output
            self.cleanup: dict = {}

        def start(self) -> None:
            return None

        def poll(self) -> int | None:
            return self.returncode

        def stop(self) -> None:
            self.cleanup = {"clean": True}

        def tail(self) -> str:
            return self.output

    def factory(_name: str, _command: list[str], _log_path: Path) -> Attempt:
        attempt = Attempt(1, "dds_wait_for_acks(nav_command_request): Timeout") if not created else Attempt(None, "")
        created.append(attempt)
        return attempt

    publisher = ResilientTeleopProcess(
        ["lingtu_nav_control", "teleop", "0.18", "0", "0"],
        tmp_path / "teleop.log",
        managed_process_factory=factory,
        retry_delay_s=0.0,
    )
    publisher.start()
    deadline = time.monotonic() + 1.0
    while len(created) < 2 and time.monotonic() < deadline:
        time.sleep(0.01)

    assert len(created) >= 2
    assert publisher.poll() is None
    publisher.stop()

    snapshot = publisher.snapshot()
    assert snapshot["attempt_count"] == 2
    assert snapshot["ack_timeout_count"] == 1
    assert snapshot["retry_count"] == 1
    assert publisher.cleanup["clean"] is True


def test_case_artifact_reset_removes_stale_slam_snapshots(tmp_path: Path) -> None:
    snapshots = tmp_path / "slam_clouds"
    nested = snapshots / "old"
    nested.mkdir(parents=True)
    (nested / "stale.pcd").write_text("stale", encoding="utf-8")
    status = tmp_path / "status.json"
    status.write_text("stale", encoding="utf-8")

    reset_case_artifacts(
        {
            "slam_cloud_dir": snapshots,
            "status": status,
        }
    )

    assert snapshots.is_dir()
    assert list(snapshots.iterdir()) == []
    assert not status.exists()


def test_run_assigns_isolated_domains_and_aggregates_case_failures(tmp_path: Path) -> None:
    args = build_parser().parse_args(
        [
            "--scenario",
            "free",
            "--scenario",
            "obstacle_slow",
            "--domain-base",
            "226",
            "--artifact-dir",
            str(tmp_path),
        ]
    )

    def prepare(_args: object) -> dict:
        return {
            "ok": True,
            "blockers": [],
            "manifest": {},
            "binaries": {},
            "paths": {},
            "details": {},
        }

    observed: list[tuple[str, int]] = []

    def execute(*, scenario: str, domain_id: int, **_kwargs: object) -> dict:
        observed.append((scenario, domain_id))
        blockers = ["slow_not_observed"] if scenario == "obstacle_slow" else []
        return {"scenario": scenario, "ok": not blockers, "blockers": blockers}

    report = run(args, prepare_runtime_fn=prepare, execute_case_fn=execute)

    assert observed == [("free", 226), ("obstacle_slow", 227)]
    assert report["ok"] is False
    assert report["blockers"] == ["slow_not_observed"]
    assert report["terrain_producer_contract"]["soft_height_m"] == 0.08


def test_slam_inputs_dropout_is_selectable_but_not_in_default_matrix() -> None:
    selected = build_parser().parse_args(["--scenario", "slam_inputs_dropout_recovery"])

    assert selected.scenario == ["slam_inputs_dropout_recovery"]
    assert "slam_inputs_dropout_recovery" not in _requested_scenarios(None)


def test_cli_entrypoint_loads_the_evaluator_before_main_runs() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "sim/scripts/mujoco/teleop_avoid_native_acceptance.py",
            "--help",
        ],
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0
    assert "--preflight-only" in completed.stdout


def test_direct_script_contract_adds_repo_root_before_runtime_import(
    tmp_path: Path,
) -> None:
    script = Path(__file__).resolve().parents[1] / "scripts" / "mujoco" / "teleop_avoid_native_acceptance.py"
    code = (
        "import importlib, runpy; "
        f"runpy.run_path({str(script)!r}, run_name='teleop_acceptance_import'); "
        "importlib.import_module('sim.scripts.mujoco.native_navigation_acceptance')"
    )

    completed = subprocess.run(
        [sys.executable, "-c", code],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr


def test_owned_pid_cleanup_waits_for_late_pidfile_and_audits_term(
    tmp_path: Path,
    monkeypatch,
) -> None:
    from sim.scripts.mujoco import native_navigation_acceptance as native

    pid_path = tmp_path / "late-child.pid"
    calls: list[tuple] = []

    def read_pid(path: Path, timeout_s: float = 0.0) -> int:
        calls.append(("read", path, timeout_s))
        return 495

    alive_answers = iter((True, False))

    def alive(pid: int | None) -> bool:
        calls.append(("alive", pid))
        return next(alive_answers)

    def signal(pid: int | None, signal_name: str) -> bool:
        calls.append(("signal", pid, signal_name))
        return True

    def wait(pid: int | None, timeout_s: float) -> bool:
        calls.append(("wait", pid, timeout_s))
        return True

    monkeypatch.setattr(native, "_read_linux_pid", read_pid)
    monkeypatch.setattr(native, "_wsl_pid_alive", alive)
    monkeypatch.setattr(native, "_signal_wsl_pid", signal)
    monkeypatch.setattr(native, "_wait_wsl_pid_exit", wait)

    result = cleanup_owned_pid_file("late_child", pid_path, pid_wait_s=1.5)

    assert calls[0] == ("read", pid_path, 1.5)
    assert ("signal", 495, "TERM") in calls
    assert result == {
        "name": "late_child",
        "linux_pid": 495,
        "pid_file": str(pid_path),
        "pid_wait_s": 1.5,
        "alive_before_cleanup": True,
        "term_sent": True,
        "kill_sent": False,
        "alive_after_cleanup": False,
        "clean": True,
    }


def test_owned_pid_cleanup_uses_posix_kill_outside_wsl(
    tmp_path: Path,
    monkeypatch,
) -> None:
    from sim.scripts.mujoco import native_navigation_acceptance as native
    from sim.scripts.mujoco import teleop_avoid_native_acceptance as acceptance

    pid_path = tmp_path / "native-child.pid"
    pid_path.write_text("812\n", encoding="ascii")
    alive = True
    calls: list[tuple[int, int]] = []

    def kill(pid: int, signum: int) -> None:
        nonlocal alive
        calls.append((pid, signum))
        if signum == 0 and not alive:
            raise ProcessLookupError(pid)
        if signum == signal.SIGTERM:
            alive = False

    monkeypatch.setattr(acceptance.os, "name", "posix")
    monkeypatch.setattr(acceptance.os, "kill", kill)
    monkeypatch.setattr(native, "_read_linux_pid", lambda *_args, **_kwargs: 812)

    result = cleanup_owned_pid_file("native_child", pid_path)

    assert (812, signal.SIGTERM) in calls
    assert result["alive_before_cleanup"] is True
    assert result["term_sent"] is True
    assert result["clean"] is True


def test_prior_owned_pidfiles_are_reclaimed_before_artifact_reset(
    tmp_path: Path,
    monkeypatch,
) -> None:
    from sim.scripts.mujoco import teleop_avoid_native_acceptance as acceptance

    case_dir = tmp_path / "case"
    case_dir.mkdir()
    traversability_pid = case_dir / "traversability.pid"
    tap_pid = case_dir / "cmd_vel_tap.pid"
    traversability_pid.write_text("101\n", encoding="ascii")
    tap_pid.write_text("102\n", encoding="ascii")
    observed: list[tuple[str, Path, float]] = []

    def cleanup(name: str, path: Path, *, pid_wait_s: float = 2.0, **_kwargs) -> dict:
        observed.append((name, path, pid_wait_s))
        return {"name": name, "pid_file": str(path), "clean": True}

    monkeypatch.setattr(acceptance, "cleanup_owned_pid_file", cleanup)
    monkeypatch.setattr(
        acceptance,
        "_inspect_pid_command",
        lambda pid: {
            "alive": True,
            "readable": True,
            "command": (
                "lingtu_traversability_dds --domain-id 226"
                if pid == 101
                else "lingtu_mujoco_cmd_vel_tap --domain-id 226"
            ),
        },
    )

    result = reclaim_prior_case_processes(
        case_dir,
        {
            "sensor_publisher_pid": case_dir / "sensor_publisher.pid",
            "cmd_vel_tap_pid": tap_pid,
        },
        {
            "prior_traversability": [
                "lingtu_traversability_dds",
                "--domain-id",
                "226",
            ],
            "prior_cmd_vel_tap": [
                "lingtu_mujoco_cmd_vel_tap",
                "--domain-id",
                "226",
            ],
        },
    )

    assert observed == [
        ("prior_traversability", traversability_pid, 0.1),
        ("prior_cmd_vel_tap", tap_pid, 0.1),
    ]
    assert all(item["ownership_handle_reclaimed"] is True for item in result)


def test_prior_reclaim_never_signals_a_reused_unrelated_pid(
    tmp_path: Path,
    monkeypatch,
) -> None:
    from sim.scripts.mujoco import native_navigation_acceptance as native
    from sim.scripts.mujoco import teleop_avoid_native_acceptance as acceptance

    case_dir = tmp_path / "case"
    case_dir.mkdir()
    pid_path = case_dir / "navigation.pid"
    pid_path.write_text("777\n", encoding="ascii")
    monkeypatch.setattr(native, "_read_linux_pid", lambda *_args, **_kwargs: 777)
    monkeypatch.setattr(
        acceptance,
        "_inspect_pid_command",
        lambda _pid: {
            "alive": True,
            "readable": True,
            "command": "unrelated_database_worker --domain-id 226",
        },
    )
    monkeypatch.setattr(
        acceptance,
        "cleanup_owned_pid_file",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("a reused PID must never be signalled")),
    )

    result = reclaim_prior_case_processes(
        case_dir,
        {
            "sensor_publisher_pid": case_dir / "sensor_publisher.pid",
            "cmd_vel_tap_pid": case_dir / "cmd_vel_tap.pid",
        },
        {
            "prior_navigation": [
                "navd",
                "--domain-id",
                "226",
            ]
        },
    )

    assert result[0]["action"] == "stale_reused_pid_handle_removed"
    assert result[0]["clean"] is True
    assert not pid_path.exists()
