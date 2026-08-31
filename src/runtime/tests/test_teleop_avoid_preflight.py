from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from diagnostics.field.teleop_avoid_preflight import evaluate_teleop_avoid_preflight
from lingtu.run_plan import CURRENT_RUN_SCHEMA, RUN_PLAN_SCHEMA, RunPlan

ROOT = Path(__file__).resolve().parents[3]
COLLECTOR = ROOT / "scripts" / "gates" / "thunder_service_readiness_collect.py"
LINGTU_CLI = ROOT / "scripts" / "lingtu"
PRODUCT_SESSION_ID = "a" * 32


def _load_collector():
    spec = importlib.util.spec_from_file_location("teleop_preflight_collector", COLLECTOR)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _zero() -> dict[str, float]:
    return {"vx": 0.0, "vy": 0.0, "wz": 0.0}


def _snapshot() -> dict:
    required_capabilities = {
        "operator_motion_typed_dds_interface",
        "native_operator_motion_authority",
        "registered_cloud_collision_check",
        "traversability_costmap",
        "local_planner_collision_and_traversability_scoring",
        "path_follower_pre_command_output",
        "operator_assisted_local_planner_control",
        "final_cmd_vel_single_writer",
    }
    required_topics = {
        "/nav/operator_motion/control",
        "/nav/operator_motion/sample",
        "/nav/operator_motion/ack",
        "/nav/operator_motion/status",
        "/slam/odometry",
        "/slam/registered_cloud",
        "/slam/map_observation",
        "/maps/state",
        "/maps/scene",
        "/nav/traversability",
        "/nav/local_path",
        "/nav/cmd_vel",
    }
    native_nav = {
        "control_mode": "teleop_avoid",
        "publish_cmd_vel": True,
        "check_obstacle": True,
        "use_traversability_cost": True,
        "allow_teleop_takeover": False,
        "teleop_local_planner": True,
    }
    nav = {
        "schema_version": "lingtu.nav.endpoint.status.v1",
        "control_mode": "teleop_avoid",
        "native_product": {
            "product": "teleop_avoid",
        },
        **native_nav,
        "operator_motion": {
            "interface_enabled": True,
            "authority_owner": "native_endpoint",
            "control_mode": "teleop_avoid",
            "control_ack_scope": "claim_hold_release",
            "sample_evidence": "status_sequences",
            "ack_publish_failed": 0,
            "status_publish_failed": 0,
            "last_ack": {
                "observed": False,
                "published": False,
                "accepted": False,
                "accepted_sequence": 0,
                "final_output_sequence": 0,
            },
            "status": {
                "observed": True,
                "published": True,
                "has_active_authority": False,
                "has_active_sample": False,
                "admitted_sequence": 0,
                "final_output_sequence": 0,
                "input_gate_reason": "ready",
                "teleop_output": _zero(),
                "final_cmd_vel": _zero(),
            },
        },
        "has_odom": True,
        "has_traversability": True,
        "input_gate": {
            "ready": True,
            "reason": "ready",
            "require_odom": True,
            "require_cloud": True,
            "require_traversability": True,
            "require_driver_control": True,
            "require_localization_health": True,
            "localization_healthy": True,
            "driver_control_ready": True,
            "driver_control_reason": "ready",
        },
        "control_loop_health": {"ready": True, "healthy": True, "reason": "ready"},
        "counters": {"odom": 10, "registered_clouds": 10, "traversability": 10},
        "final_cmd_vel": _zero(),
        "final_output": {
            "published": True,
            "producer_boot_id": "nav-boot-1",
            "output_sequence": 17,
            "driver_delivery_accepted": True,
        },
        "driver_control": {
            "received": True,
            "ready": True,
            "reason": "ready",
            "last_command_accepted": True,
            "accepted_producer_boot_id": "nav-boot-1",
            "accepted_output_sequence": 17,
        },
        "teleop": {
            "seen": False,
            "published": False,
            "reason": "idle",
            "request": _zero(),
            "output": _zero(),
        },
        "local_path_points": 0,
        "last_local": {"seen": False, "final_safety": {"applied": False}},
    }
    traversability = {
        "schema_version": "lingtu.traversability.status.v2",
        "endpoint": "lingtu_traversability_dds",
        "has_odom": True,
        "counters": {"odom": 10, "registered_clouds": 10, "published": 10},
    }
    maps = {
        "schema_version": "lingtu.maps.runtime.v1",
        "process": "mapd",
        "status": "ready",
        "ready": True,
        "running": True,
        "live": True,
        "generation": 12,
        "accepted_observations": 10,
        "processed_observations": 10,
        "dds_received": 10,
        "dds_decoded": 10,
        "dds_rejected": 0,
        "dds_write_attempts": 40,
        "dds_write_failures": 0,
        "dds_serialization_rejections": 0,
        "dds_scene_oversize_rejections": 0,
        "dds_unhealthy_writers": 0,
        "required_publications_ready": True,
        "current_generation_published": True,
        "state_published_generation": 12,
        "realtime_clouds_published_generation": 12,
        "map_layers_published_generation": 12,
        "scene_published_generation": 12,
        "voxel_snapshot_omitted_cells": 0,
        "capacity_limited": False,
        "voxel_capacity_rejections": 0,
        "accumulated_capacity_rejections": 0,
    }
    control = {
        "control_assured": True,
        "motors_enabled": True,
        "critical_fault": False,
        "lease_valid": True,
        "initial_zero_acknowledged": True,
        "fsm": "standing",
    }
    driver = {
        "schema_version": "lingtu.driver.status.v2",
        "role": "driver",
        "backend": "doso",
        "connected": True,
        "ready": True,
        "dds": {
            "topic": "/nav/cmd_vel",
            "wire_topic": "rt/nav/cmd_vel",
            "cmd_vel_writer_ready": True,
            "matched_cmd_vel_writers": 1,
        },
        "adapter": {
            "protocol": "brainstem_grpc",
            "target": "192.168.66.12:13145",
            "control_owner": "grpc",
            "control_owner_id": "lingtu-driver@robot",
        },
        "control": control,
        "last_velocity": {"vx_mps": 0.0, "vy_mps": 0.0, "yaw_rps": 0.0},
        "output_ack": {
            "accepted": True,
            "producer_boot_id": "nav-boot-1",
            "output_sequence": 17,
        },
    }
    return {
        "current_run": {
            "exists": True,
            "state": {
                "schema_version": CURRENT_RUN_SCHEMA,
                "product": "teleop_avoid",
                "env": "real",
                "product_session_id": PRODUCT_SESSION_ID,
                "run_plan_path": "/run/lingtu/plans/plan-session.json",
            },
            "plan_verified": True,
            "run_plan": {
                "identity": {
                    "schema": RUN_PLAN_SCHEMA,
                    "product": "teleop_avoid",
                    "env": "real",
                },
                "launch": {
                    "native_process_environment": {
                        "LINGTU_DRIVER_TARGET": "192.168.66.12:13145",
                        "LINGTU_DRIVER_NETWORK_INTERFACE": "",
                    }
                },
            },
            "verified_contract": {
                "product": "teleop_avoid",
                "env": "real",
                "required_capabilities": sorted(required_capabilities),
                "required_topics": sorted(required_topics),
                "native_nav": native_nav,
                "selected_roles": [
                    "driver",
                    "host",
                    "lidar",
                    "maps",
                    "nav",
                    "slam",
                    "traversability",
                ],
            },
        },
        "status_files": {
            "nav": {
                "path": "/dev/shm/lingtu/nav_endpoint_status.json",  # noqa: S108
                "exists": True,
                "age_s": 0.1,
                "json": nav,
            },
            "traversability": {
                "path": "/dev/shm/lingtu/traversability_status.json",  # noqa: S108
                "exists": True,
                "age_s": 0.1,
                "json": traversability,
            },
            "maps": {
                "path": "/dev/shm/lingtu/mapd_status.json",  # noqa: S108
                "exists": True,
                "age_s": 0.1,
                "json": maps,
            },
            "driver": {
                "path": "/dev/shm/lingtu/driver_status.json",  # noqa: S108
                "exists": True,
                "age_s": 0.1,
                "json": driver,
            },
        },
        "driver_readiness": {"ok": True, "blockers": []},
    }


def _go2_snapshot() -> dict:
    snapshot = _snapshot()
    driver = snapshot["status_files"]["driver"]["json"]
    driver["backend"] = "go2"
    driver["adapter"] = {
        "protocol": "unitree_sdk2",
        "target": "dds://eth0/rt/api/sport/request",
        "control_owner": "none",
        "control_owner_id": "",
    }
    driver["control"]["lease_valid"] = False
    environment = snapshot["current_run"]["run_plan"]["launch"]["native_process_environment"]
    environment["LINGTU_DRIVER_TARGET"] = ""
    environment["LINGTU_DRIVER_NETWORK_INTERFACE"] = "eth0"
    return snapshot


def test_contract_stage_accepts_verified_runtime_contract_without_motion() -> None:
    result = evaluate_teleop_avoid_preflight(_snapshot(), stage="contract")

    assert result["ok"] is True
    assert result["read_only"] is True
    assert result["authority_acquired"] is False
    assert result["command_published"] is False
    assert result["nonzero_motion_allowed"] is False
    assert result["nonzero_motion_blockers"] == ["motion_stage_not_evaluated"]
    assert not any(check["id"].startswith("motion.") for check in result["checks"])


def test_contract_stage_rejects_current_product_mismatch() -> None:
    snapshot = _snapshot()
    snapshot["current_run"]["state"]["product"] = "nav"

    result = evaluate_teleop_avoid_preflight(snapshot, stage="contract")

    assert "product.identity" in result["blockers"]


def test_contract_stage_requires_native_mapd_topics() -> None:
    snapshot = _snapshot()
    required_topics = snapshot["current_run"]["verified_contract"]["required_topics"]
    required_topics.remove("/slam/map_observation")

    result = evaluate_teleop_avoid_preflight(snapshot, stage="contract")

    assert "product.required_topics" in result["blockers"]


@pytest.mark.parametrize(
    ("path", "value", "blocker"),
    [
        (("age_s",), 3.1, "status.maps.fresh"),
        (("json", "schema_version"), "wrong", "status.maps.schema"),
        (("json", "process"), "python_maps", "maps.runtime"),
        (("json", "status"), "waiting_for_observation", "maps.runtime"),
        (("json", "ready"), False, "maps.runtime"),
        (("json", "running"), False, "maps.runtime"),
        (("json", "live"), False, "maps.runtime"),
        (("json", "accepted_observations"), 0, "maps.observation_pipeline"),
        (("json", "processed_observations"), 0, "maps.observation_pipeline"),
        (("json", "generation"), 0, "maps.observation_pipeline"),
        (("json", "dds_received"), 0, "maps.observation_pipeline"),
        (("json", "dds_decoded"), 0, "maps.observation_pipeline"),
        (("json", "dds_write_attempts"), 0, "maps.dds_health"),
        (("json", "dds_rejected"), 1, "maps.dds_health"),
        (("json", "dds_write_failures"), 1, "maps.dds_health"),
        (("json", "dds_serialization_rejections"), 1, "maps.dds_health"),
        (("json", "dds_scene_oversize_rejections"), 1, "maps.dds_health"),
        (("json", "dds_unhealthy_writers"), 1, "maps.dds_health"),
        (
            ("json", "voxel_snapshot_omitted_cells"),
            -1,
            "maps.resource_accounting",
        ),
        (("json", "capacity_limited"), True, "maps.resource_capacity"),
        (("json", "capacity_limited"), 0, "maps.resource_capacity"),
        (("json", "voxel_capacity_rejections"), 1, "maps.resource_capacity"),
        (("json", "voxel_capacity_rejections"), False, "maps.resource_capacity"),
        (("json", "accumulated_capacity_rejections"), 1, "maps.resource_capacity"),
        (("json", "accumulated_capacity_rejections"), False, "maps.resource_capacity"),
        (
            ("json", "required_publications_ready"),
            False,
            "maps.required_publications",
        ),
        (
            ("json", "current_generation_published"),
            False,
            "maps.required_publications",
        ),
        (
            ("json", "scene_published_generation"),
            11,
            "maps.required_publications",
        ),
    ],
)
def test_contract_stage_rejects_unready_or_degraded_mapd(path: tuple[str, ...], value: object, blocker: str) -> None:
    snapshot = _snapshot()
    target = snapshot["status_files"]["maps"]
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value

    result = evaluate_teleop_avoid_preflight(snapshot, stage="contract")

    assert blocker in result["blockers"]


def test_contract_stage_allows_bounded_voxel_snapshot_omissions() -> None:
    snapshot = _snapshot()
    snapshot["status_files"]["maps"]["json"]["voxel_snapshot_omitted_cells"] = 42

    result = evaluate_teleop_avoid_preflight(snapshot, stage="contract")

    assert result["ok"] is True


def test_contract_stage_accepts_traversability_five_second_status_cadence() -> None:
    snapshot = _snapshot()
    snapshot["status_files"]["traversability"]["age_s"] = 5.2

    result = evaluate_teleop_avoid_preflight(snapshot, stage="contract", status_max_age_s=3.0)

    assert result["ok"] is True


def test_contract_stage_rejects_traversability_status_older_than_its_cadence() -> None:
    snapshot = _snapshot()
    snapshot["status_files"]["traversability"]["age_s"] = 6.1

    result = evaluate_teleop_avoid_preflight(snapshot, stage="contract", status_max_age_s=3.0)

    assert "status.traversability.fresh" in result["blockers"]


def test_motion_stage_keeps_nav_traversability_as_distinct_safety_authority() -> None:
    snapshot = _snapshot()
    snapshot["current_run"]["verified_contract"]["required_topics"].remove("/nav/traversability")

    result = evaluate_teleop_avoid_preflight(snapshot, stage="motion")

    checks = {check["id"]: check for check in result["checks"]}
    assert "product.required_topics" in result["blockers"]
    assert "/nav/traversability" in checks["product.required_topics"]["expected"]
    assert "/maps/traversability" not in checks["product.required_topics"]["expected"]


def test_motion_stage_rejects_degraded_mapd() -> None:
    snapshot = _snapshot()
    snapshot["status_files"]["maps"]["json"]["live"] = False

    result = evaluate_teleop_avoid_preflight(snapshot, stage="motion")

    assert "maps.runtime" in result["blockers"]


def test_motion_stage_accepts_idle_correlated_zero_ack_without_active_sample() -> None:
    result = evaluate_teleop_avoid_preflight(_snapshot(), stage="motion")

    assert result["ok"] is True
    assert result["nonzero_motion_allowed"] is True
    assert result["nonzero_motion_blockers"] == []
    checks = {check["id"]: check for check in result["checks"]}
    assert checks["motion.operator_sample_correlation"]["ok"] is True
    assert checks["motion.idle_zero"]["ok"] is True
    assert checks["motion.correlated_driver_ack"]["ok"] is True


def test_motion_stage_accepts_one_tick_nav_ack_lag() -> None:
    snapshot = _go2_snapshot()
    nav = snapshot["status_files"]["nav"]["json"]
    nav["final_output"]["driver_delivery_accepted"] = False
    nav["driver_control"]["accepted_output_sequence"] = 16

    result = evaluate_teleop_avoid_preflight(snapshot, stage="motion")

    assert result["ok"] is True
    checks = {check["id"]: check for check in result["checks"]}
    assert checks["motion.correlated_driver_ack"]["ok"] is True


def test_motion_stage_accepts_lagging_driver_status_snapshot() -> None:
    snapshot = _go2_snapshot()
    snapshot["status_files"]["driver"]["json"]["output_ack"]["output_sequence"] = 3

    result = evaluate_teleop_avoid_preflight(snapshot, stage="motion")

    assert result["ok"] is True


def test_motion_stage_rejects_stale_nav_ack() -> None:
    snapshot = _go2_snapshot()
    snapshot["status_files"]["nav"]["json"]["driver_control"]["accepted_output_sequence"] = 14

    result = evaluate_teleop_avoid_preflight(snapshot, stage="motion")

    assert "motion.correlated_driver_ack" in result["blockers"]


def test_motion_stage_rejects_driver_ack_too_far_ahead() -> None:
    snapshot = _go2_snapshot()
    snapshot["status_files"]["driver"]["json"]["output_ack"]["output_sequence"] = 20

    result = evaluate_teleop_avoid_preflight(snapshot, stage="motion")

    assert "motion.correlated_driver_ack" in result["blockers"]


def test_motion_stage_accepts_go2_sdk2_control() -> None:
    result = evaluate_teleop_avoid_preflight(_go2_snapshot(), stage="motion")

    assert result["ok"] is True
    checks = {check["id"]: check for check in result["checks"]}
    assert checks["motion.go2_control"]["ok"] is True


def test_motion_stage_rejects_go2_with_fabricated_sdk2_owner() -> None:
    snapshot = _go2_snapshot()
    snapshot["status_files"]["driver"]["json"]["adapter"]["control_owner"] = "sdk2"

    result = evaluate_teleop_avoid_preflight(snapshot, stage="motion")

    assert "motion.go2_control" in result["blockers"]


def test_motion_stage_rejects_unrelated_operator_output_sequence() -> None:
    snapshot = _snapshot()
    nav = snapshot["status_files"]["nav"]["json"]
    nav["operator_motion"]["status"].update(
        {
            "has_active_sample": True,
            "admitted_sequence": 8,
            "final_output_sequence": 16,
        }
    )

    result = evaluate_teleop_avoid_preflight(snapshot, stage="motion")

    assert "motion.operator_sample_correlation" in result["blockers"]


def test_motion_stage_rejects_loopback_brainstem() -> None:
    snapshot = _snapshot()
    snapshot["status_files"]["driver"]["json"]["adapter"]["target"] = "127.0.0.1:13145"

    result = evaluate_teleop_avoid_preflight(snapshot, stage="motion")

    assert "motion.brainstem_control" in result["blockers"]
    assert result["nonzero_motion_allowed"] is False
    assert "motion.brainstem_control" in result["nonzero_motion_blockers"]


def test_motion_stage_rejects_driver_target_that_differs_from_run_plan() -> None:
    snapshot = _snapshot()
    snapshot["current_run"]["run_plan"]["launch"]["native_process_environment"][
        "LINGTU_DRIVER_TARGET"
    ] = "192.168.66.99:13145"

    result = evaluate_teleop_avoid_preflight(snapshot, stage="motion")

    assert "motion.brainstem_control" in result["blockers"]


def test_current_run_collector_loads_current_run_plan(monkeypatch, tmp_path: Path) -> None:
    module = _load_collector()
    run_plan = RunPlan.create(
        product="teleop_avoid",
        env="real",
        robot="unitree/go2",
        process_control="systemd",
        modules=(),
        processes=(),
        available_processes=(),
        stop_before_start=(),
        contracts=("lingtu.product.teleop_avoid.v1",),
        critical_modules=(),
        native_nav={
            "control_mode": "teleop_avoid",
            "publish_cmd_vel": True,
            "check_obstacle": True,
            "use_traversability_cost": True,
            "allow_teleop_takeover": False,
            "teleop_local_planner": True,
            "global_planner": "octoplanner3d",
        },
        route_contract="robot",
        host_config={},
        lifecycle={"product": "teleop_avoid"},
    )
    run_plan_path = tmp_path / "runtime.json"
    run_plan.write(run_plan_path)
    active_path = tmp_path / "active-runtime.json"
    active_path.write_text(
        json.dumps(
            {
                "schema_version": CURRENT_RUN_SCHEMA,
                "product": "teleop_avoid",
                "env": "real",
                "run_plan_path": str(run_plan_path),
                "product_session_id": PRODUCT_SESSION_ID,
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_CURRENT_FILE", str(active_path))

    result = module.collect_current_run()

    assert result["plan_verified"] is True
    assert result["state"]["product_session_id"] == PRODUCT_SESSION_ID
    assert result["run_plan"]["identity"] == run_plan.as_dict()["identity"]
    assert result["verified_contract"]["native_nav"]["control_mode"] == "teleop_avoid"


def test_collector_strict_uses_selected_preflight_not_general_summary(monkeypatch, capsys) -> None:
    module = _load_collector()
    monkeypatch.setattr(
        module,
        "parse_args",
        lambda: SimpleNamespace(
            gateway_url="http://127.0.0.1:5050",
            dds_seconds=0.0,
            dds_domain=0,
            teleop_stage=None,
            teleop_avoid_stage="contract",
            status_max_age_s=3.0,
            json_out=None,
            pretty=False,
            strict=True,
        ),
    )
    monkeypatch.setattr(
        module,
        "build_report",
        lambda **_: {
            "summary": {"ok": True},
            "teleop_avoid_preflight": {"ok": False, "blockers": ["product.identity"]},
        },
    )

    assert module.main() == 1
    assert "product.identity" in capsys.readouterr().out


def test_lingtu_cli_exposes_thin_read_only_preflight_adapter() -> None:
    source = LINGTU_CLI.read_text(encoding="utf-8")

    assert '-m lingtu.control "$@"' in source
    assert "thunder_service_readiness_collect.py" not in source
    assert "operator_motion" not in source
    assert "curl" not in source
