from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

import pytest

from sim.scripts.mujoco import native_control_mode_acceptance as acceptance

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "config" / "runtime_graph" / "endpoints" / "mujoco_native_control_mode_acceptance.json"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _mapping_sha256(value: dict) -> str:
    payload = json.dumps(value, ensure_ascii=True, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _endpoint_observations(control_mode: str, nonzero_samples: int) -> dict:
    selected = "goal" if control_mode == "autonomy" else "teleop"
    forbidden = "teleop" if control_mode == "autonomy" else "goal"
    return {
        "control_mode": control_mode,
        "command_transport": "typed_dds_request_ack",
        "command_events": [
            {
                "request_id": "selected-1",
                "kind": selected,
                "accepted": True,
                "acked": True,
            },
            {
                "request_id": "forbidden-1",
                "kind": forbidden,
                "accepted": False,
                "acked": True,
            },
            {
                "request_id": "stop-1",
                "kind": "stop",
                "accepted": True,
                "acked": True,
                "post_stop_zero_observed": True,
                "post_stop_nonzero_samples": 0,
            },
        ],
        "final_cmd_topic": "rt/nav/cmd_vel",
        "final_cmd_writer": "lingtu_nav_native_endpoint",
        "nonzero_cmd_samples": nonzero_samples,
    }


def test_each_control_mode_has_an_exclusive_product_execution_plan() -> None:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))

    autonomy = acceptance.build_execution_plan(manifest, "autonomy")
    teleop = acceptance.build_execution_plan(manifest, "teleop")
    teleop_avoid = acceptance.build_execution_plan(manifest, "teleop_avoid")

    assert autonomy["endpoint_control_mode"] == "autonomy"
    assert autonomy["command_kind"] == "goal"
    assert set(autonomy["required_processes"]) == {
        "slam",
        "traversability",
        "navigation",
        "mujoco_sensor_policy",
    }
    assert autonomy["runner"]["kind"] == "native_navigation_acceptance"

    assert teleop["endpoint_control_mode"] == "teleop"
    assert teleop["command_kind"] == "teleop"
    assert set(teleop["required_processes"]) == {
        "navigation",
        "mujoco_sensor_policy",
    }
    assert teleop["required_inputs"] == ["rt/nav/command/request"]
    assert teleop["runner"]["kind"] == "unavailable"

    assert teleop_avoid["endpoint_control_mode"] == "teleop_avoid"
    assert teleop_avoid["command_kind"] == "teleop"
    assert set(teleop_avoid["required_processes"]) == {
        "slam",
        "traversability",
        "navigation",
        "mujoco_sensor_policy",
    }
    assert "rt/slam/localization_health" in teleop_avoid["required_inputs"]
    assert "rt/nav/traversability" in teleop_avoid["required_inputs"]
    assert teleop_avoid["runner"]["kind"] == "teleop_avoid_geometry_mirror"

    with pytest.raises(ValueError, match="unsupported control mode"):
        acceptance.build_execution_plan(manifest, "both")


def test_pure_teleop_passes_without_slam_and_marks_quality_not_applicable() -> None:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    evidence = {
        "endpoint": _endpoint_observations("teleop", 12),
        "runtime": {
            "processes": ["navigation", "mujoco_sensor_policy"],
            "python_cmd_vel_mux_active": False,
            "python_planner_used": False,
            "command_source": "dds",
            "policy_loaded": True,
            "motion_m": 0.42,
            "cleanup_clean": True,
            "harness_report_ok": True,
        },
    }

    report = acceptance.evaluate_observations(manifest, "teleop", evidence)

    assert report["ok"] is True
    assert report["promotion_eligible"] is False
    assert report["assessment_kind"] == "diagnostic_observations"
    assert report["gates"]["control_chain"]["status"] == "passed"
    assert report["gates"]["product_integration"]["status"] == "passed"
    assert report["gates"]["slam_map_quality"] == {
        "status": "not_applicable",
        "ok": True,
        "blockers": [],
    }


def test_teleop_avoid_fails_closed_when_traversability_is_stale() -> None:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    evidence = {
        "endpoint": _endpoint_observations("teleop_avoid", 8),
        "runtime": {
            "processes": [
                "slam",
                "traversability",
                "navigation",
                "mujoco_sensor_policy",
            ],
            "python_cmd_vel_mux_active": False,
            "python_planner_used": False,
            "command_source": "dds",
            "policy_loaded": True,
            "motion_m": 0.35,
            "cleanup_clean": True,
            "harness_report_ok": True,
        },
        "slam_map": {
            "slam_state": "TRACKING",
            "localization_health_fresh": True,
            "registered_cloud_fresh": True,
            "traversability_fresh": False,
            "map_xy_error_m": 0.12,
        },
    }

    report = acceptance.evaluate_observations(manifest, "teleop_avoid", evidence)

    assert report["gates"]["control_chain"]["ok"] is True
    assert report["gates"]["product_integration"]["ok"] is True
    assert report["gates"]["slam_map_quality"]["status"] == "failed"
    assert "traversability_stale" in report["gates"]["slam_map_quality"]["blockers"]
    assert report["ok"] is False


def test_autonomy_quality_gate_rejects_scale_and_map_match_regression() -> None:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    evidence = {
        "endpoint": _endpoint_observations("autonomy", 121),
        "runtime": {
            "processes": [
                "slam",
                "traversability",
                "navigation",
                "mujoco_sensor_policy",
            ],
            "python_cmd_vel_mux_active": False,
            "python_planner_used": False,
            "command_source": "dds",
            "policy_loaded": True,
            "motion_m": 1.121,
            "cleanup_clean": True,
            "harness_report_ok": True,
        },
        "slam_map": {
            "slam_state": "TRACKING",
            "localization_health_fresh": True,
            "registered_cloud_fresh": True,
            "traversability_fresh": True,
            "ate_rmse_m": 0.268,
            "trajectory_scale_ratio": 6.22,
            "map_near_field_match_rate": 0.708,
        },
    }

    report = acceptance.evaluate_observations(manifest, "autonomy", evidence)

    assert report["gates"]["control_chain"]["ok"] is True
    assert report["gates"]["product_integration"]["ok"] is True
    assert report["gates"]["slam_map_quality"]["status"] == "failed"
    assert set(report["gates"]["slam_map_quality"]["blockers"]) == {
        "trajectory_scale_ratio_out_of_range",
        "map_near_field_match_rate_below_threshold",
    }
    assert report["ok"] is False


def test_pure_teleop_rejects_slam_or_traversability_processes() -> None:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    evidence = {
        "endpoint": _endpoint_observations("teleop", 5),
        "runtime": {
            "processes": [
                "slam",
                "traversability",
                "navigation",
                "mujoco_sensor_policy",
            ],
            "python_cmd_vel_mux_active": False,
            "python_planner_used": False,
            "command_source": "dds",
            "policy_loaded": True,
            "motion_m": 0.25,
            "cleanup_clean": True,
            "harness_report_ok": True,
        },
    }

    report = acceptance.evaluate_observations(manifest, "teleop", evidence)

    assert set(report["gates"]["product_integration"]["blockers"]) >= {
        "forbidden_process_active:slam",
        "forbidden_process_active:traversability",
    }
    assert report["ok"] is False


def test_cli_requires_one_mode_and_only_evaluates_runner_artifacts(
    tmp_path: Path,
) -> None:
    parser = acceptance.build_parser()
    with pytest.raises(SystemExit):
        parser.parse_args([])
    with pytest.raises(SystemExit):
        parser.parse_args(["--control-mode", "both"])

    report_path = tmp_path / "report.json"
    args = parser.parse_args(
        [
            "--control-mode",
            "teleop",
            "--action",
            "evaluate",
            "--json-out",
            str(report_path),
        ]
    )

    report = acceptance.run(args)

    assert report["ok"] is False
    assert report["promotion_eligible"] is False
    assert report["blockers"] == ["runner_artifact_not_supplied"]
    assert set(report["gates"]) == set(acceptance.GATE_LAYERS)
    assert json.loads(report_path.read_text(encoding="utf-8")) == report


def test_promotion_rejects_handwritten_boolean_evidence(tmp_path: Path) -> None:
    """A hand-authored summary is diagnostic input, never promotion evidence."""

    raw_evidence = tmp_path / "handwritten.json"
    raw_evidence.write_text(
        json.dumps(
            {
                "endpoint": {
                    "control_mode": "teleop",
                    "forbidden_mode_rejection_proven": True,
                    "stop_proven": True,
                },
                "runtime": {
                    "policy_loaded": True,
                    "cleanup_clean": True,
                },
            }
        ),
        encoding="utf-8",
    )

    args = acceptance.build_parser().parse_args(
        [
            "--control-mode",
            "teleop",
            "--action",
            "evaluate",
            "--runner-artifact",
            str(raw_evidence),
        ]
    )
    report = acceptance.run(args)

    assert report["ok"] is False
    assert report["promotion_eligible"] is False
    assert "runner_artifact_schema_mismatch" in report["blockers"]


def test_run_mode_fails_honestly_when_full_mode_harness_is_unavailable(
    tmp_path: Path,
) -> None:
    args = acceptance.build_parser().parse_args(
        [
            "--control-mode",
            "teleop",
            "--action",
            "run",
            "--artifact-dir",
            str(tmp_path),
        ]
    )

    report = acceptance.run(args)
    artifact = json.loads((tmp_path / "runner_artifact.json").read_text(encoding="utf-8"))

    assert report["ok"] is False
    assert report["promotion_eligible"] is False
    assert "runner_unavailable:full_mujoco_native_teleop_harness_not_implemented" in report["blockers"]
    assert artifact["schema_version"] == acceptance.RUNNER_ARTIFACT_SCHEMA
    assert artifact["producer"]["name"] == "native_control_mode_acceptance"
    assert artifact["source_report"] is None


def test_run_mode_executes_geometry_harness_but_does_not_promote_it(
    tmp_path: Path,
) -> None:
    stale_report = tmp_path / "harness" / "report.json"
    stale_report.parent.mkdir()
    stale_report.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.mujoco.teleop_avoid_gate.v1",
                "ok": True,
                "blockers": [],
                "cases": [],
            }
        ),
        encoding="utf-8",
    )
    args = acceptance.build_parser().parse_args(
        [
            "--control-mode",
            "teleop_avoid",
            "--action",
            "run",
            "--artifact-dir",
            str(tmp_path),
        ]
    )

    report = acceptance.run(args)
    artifact = json.loads((tmp_path / "runner_artifact.json").read_text(encoding="utf-8"))

    assert artifact["runner_blockers"] == []
    assert artifact["source_report"]["schema_version"] == ("lingtu.mujoco.teleop_avoid_gate.v1")
    assert artifact["source_report"]["sha256"]
    assert Path(artifact["source_report"]["path"]) != stale_report
    assert stale_report.is_file()
    assert artifact["observations"]["geometry_mirror"]["ok"] is True
    assert artifact["observations_sha256"]
    assert report["promotion_eligible"] is False
    assert report["supplemental_observations"]["geometry_mirror"]["ok"] is True
    assert "control_chain:typed_command_observations_missing" in report["blockers"]

    invalid_time = copy.deepcopy(artifact)
    invalid_time["finished_at"] = "2026-07-12T00:00:00"
    invalid_time_report = acceptance.evaluate_runner_artifact(
        json.loads(MANIFEST.read_text(encoding="utf-8")),
        "teleop_avoid",
        invalid_time,
    )
    assert invalid_time_report["promotion_eligible"] is False
    assert "runner_timestamp_invalid" in invalid_time_report["blockers"]

    nonzero_exit = copy.deepcopy(artifact)
    nonzero_exit["execution"]["returncode"] = 2
    nonzero_exit_report = acceptance.evaluate_runner_artifact(
        json.loads(MANIFEST.read_text(encoding="utf-8")),
        "teleop_avoid",
        nonzero_exit,
    )
    assert nonzero_exit_report["promotion_eligible"] is False
    assert "runner_execution_nonzero" in nonzero_exit_report["blockers"]

    source_path = Path(artifact["source_report"]["path"])
    source_path.write_text("{}", encoding="utf-8")
    tampered = acceptance.evaluate_runner_artifact(
        json.loads(MANIFEST.read_text(encoding="utf-8")),
        "teleop_avoid",
        artifact,
    )
    assert tampered["promotion_eligible"] is False
    assert "runner_source_report_digest_mismatch" in tampered["blockers"]


def test_autonomy_adapter_recomputes_proofs_from_harness_report(
    tmp_path: Path,
) -> None:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    run_id = "00000000-0000-4000-8000-000000000001"
    run_dir = tmp_path / "runs" / run_id
    harness_dir = run_dir / "harness"
    harness_dir.mkdir(parents=True)
    source_path = harness_dir / "report.json"
    source_path.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.mujoco.native_navigation.acceptance.v1",
                "phases": {
                    "motion": {
                        "evidence": {
                            "last_nav": {
                                "control_mode": "autonomy",
                                "command_boundary": {
                                    "received": 1,
                                    "ack_sent": 1,
                                    "last_kind": "goal",
                                    "last_accepted": True,
                                    "last_request_id": "goal-1",
                                },
                            },
                            "last_slam": {"state": "TRACKING"},
                            "max_registered_clouds": 5,
                            "max_traversability_published": 5,
                        },
                        "post_safety_command": {
                            "dds_topic": "rt/nav/cmd_vel",
                            "tap_nonzero_samples": 4,
                        },
                        "sensor_report": {
                            "command_source": "dds",
                            "policy_loaded": True,
                            "motion": {"sim_path_length_xy_m": 0.42},
                        },
                        "processes": [
                            {"name": "slam"},
                            {"name": "traversability"},
                            {"name": "navigation"},
                            {"name": "sensor"},
                        ],
                        "process_cleanup": {"zero_leftovers": True},
                        "python_planner_used": False,
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    script_path = ROOT / "sim/scripts/mujoco/native_navigation_acceptance.py"
    source_report = json.loads(source_path.read_text(encoding="utf-8"))
    observations = acceptance.extract_runner_observations("native_navigation_acceptance", source_report)
    artifact = {
        "schema_version": acceptance.RUNNER_ARTIFACT_SCHEMA,
        "control_mode": "autonomy",
        "run_id": run_id,
        "started_at": "2026-07-12T00:00:00+00:00",
        "finished_at": "2026-07-12T00:00:01+00:00",
        "producer": {
            "name": "native_control_mode_acceptance",
            "schema_version": acceptance.ACCEPTANCE_SCHEMA,
        },
        "manifest_sha256": _mapping_sha256(manifest),
        "artifact_dir": str(tmp_path),
        "run_dir": str(run_dir),
        "runner_kind": "native_navigation_acceptance",
        "execution": {
            "command": [
                "python",
                str(script_path),
                "--mode",
                "motion",
                "--out-dir",
                str(harness_dir),
            ],
            "returncode": 0,
        },
        "runner_blockers": [],
        "observations": observations,
        "observations_sha256": _mapping_sha256(observations),
        "source_report": {
            "path": str(source_path),
            "sha256": _sha256(source_path),
            "schema_version": "lingtu.mujoco.native_navigation.acceptance.v1",
            "script_path": str(script_path),
            "script_sha256": _sha256(script_path),
        },
        # These summaries must be ignored; the harness did not run either proof.
        "endpoint": {
            "forbidden_mode_rejection_proven": True,
            "stop_proven": True,
        },
    }

    report = acceptance.evaluate_runner_artifact(manifest, "autonomy", artifact)

    assert report["promotion_eligible"] is False
    assert "control_chain:mode_mutual_exclusion_observation_missing" in report["blockers"]
    assert "control_chain:native_stop_observation_missing" in report["blockers"]
