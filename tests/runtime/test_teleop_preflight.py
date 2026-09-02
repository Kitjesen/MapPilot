from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

from diagnostics.field.teleop_preflight import evaluate_teleop_preflight
from lingtu.run_plan import CURRENT_RUN_SCHEMA, RUN_PLAN_SCHEMA

ROOT = Path(__file__).resolve().parents[2]
COLLECTOR = ROOT / "src" / "diagnostics" / "field" / "service_readiness.py"
LINGTU_CLI = ROOT / "scripts" / "lingtu"
NAV_PRODUCER_BOOT_ID = "nav:boot:1"
IDLE_ZERO_OUTPUT_SEQUENCE = 7


def _load_collector():
    spec = importlib.util.spec_from_file_location("teleop_collector", COLLECTOR)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _zero() -> dict[str, float]:
    return {"vx": 0.0, "vy": 0.0, "wz": 0.0}


def _snapshot() -> dict:
    native_nav = {
        "control_mode": "teleop",
        "publish_cmd_vel": True,
        "check_obstacle": False,
        "use_traversability_cost": False,
        "allow_teleop_takeover": False,
        "teleop_local_planner": False,
    }
    required_topics = [
        "/nav/command/request",
        "/nav/command/ack",
        "/nav/state",
        "/nav/operator_motion/control",
        "/nav/operator_motion/sample",
        "/nav/operator_motion/ack",
        "/nav/operator_motion/status",
        "/nav/cmd_vel",
    ]
    required_capabilities = [
        "teleop",
        "operator_motion_typed_dds_interface",
        "native_operator_motion_authority",
        "command_staleness_gate",
        "velocity_limit",
        "final_cmd_vel_single_writer",
    ]
    nav = {
        "schema_version": "lingtu.nav.endpoint.status.v1",
        "native_product": {
            "product": "teleop",
        },
        **native_nav,
        "stop_confirmation_evidence": "driver_ack",
        "operator_motion": {
            "interface_enabled": True,
            "authority_owner": "native_endpoint",
            "control_mode": "teleop",
            "control_ack_scope": "claim_hold_release",
            "sample_evidence": "status_sequences",
            "ack_publish_failed": 0,
            "status_publish_failed": 0,
            "status": {
                "has_active_authority": False,
                "has_active_sample": False,
                "final_cmd_vel": _zero(),
            },
        },
        "input_gate": {
            "ready": True,
            "driver_control_ready": True,
            "require_driver_control": True,
            "require_odom": False,
            "require_cloud": False,
            "require_traversability": False,
            "require_localization_health": False,
        },
        "final_cmd_vel": _zero(),
        "final_output": {
            "published": True,
            "producer_boot_id": NAV_PRODUCER_BOOT_ID,
            "output_sequence": IDLE_ZERO_OUTPUT_SEQUENCE,
            "driver_delivery_accepted": True,
        },
        "driver_control": {
            "received": True,
            "ready": True,
            "last_command_accepted": True,
            "accepted_producer_boot_id": NAV_PRODUCER_BOOT_ID,
            "accepted_output_sequence": IDLE_ZERO_OUTPUT_SEQUENCE,
        },
    }
    control = {
        "control_assured": True,
        "initial_zero_acknowledged": True,
        "motors_enabled": True,
        "critical_fault": False,
        "lease_valid": True,
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
        "output_ack": {
            "producer_boot_id": NAV_PRODUCER_BOOT_ID,
            "output_sequence": IDLE_ZERO_OUTPUT_SEQUENCE,
            "accepted": True,
        },
    }
    return {
        "current_run": {
            "exists": True,
            "state": {
                "schema_version": CURRENT_RUN_SCHEMA,
                "product": "teleop",
                "env": "real",
            },
            "plan_verified": True,
            "run_plan": {
                "identity": {
                    "schema": RUN_PLAN_SCHEMA,
                    "product": "teleop",
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
                "product": "teleop",
                "env": "real",
                "required_topics": required_topics,
                "required_capabilities": required_capabilities,
                "native_nav": native_nav,
                "selected_roles": ["driver", "host", "nav"],
            },
        },
        "status_files": {
            "nav": {
                "path": "/dev/shm/lingtu/nav_endpoint_status.json",  # noqa: S108
                "exists": True,
                "age_s": 0.1,
                "json": nav,
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


def test_contract_stage_accepts_map_free_verified_product_without_motion() -> None:
    result = evaluate_teleop_preflight(_snapshot(), stage="contract")

    assert result["ok"] is True
    assert result["read_only"] is True
    assert result["command_published"] is False
    assert result["authority_acquired"] is False
    assert result["nonzero_motion_allowed"] is False


def test_motion_stage_requires_remote_brainstem_and_checked_zero() -> None:
    result = evaluate_teleop_preflight(_snapshot(), stage="motion")

    assert result["ok"] is True
    assert result["nonzero_motion_allowed"] is True


def test_motion_stage_accepts_go2_sdk2_control_on_configured_interface() -> None:
    result = evaluate_teleop_preflight(_go2_snapshot(), stage="motion")

    assert result["ok"] is True
    assert result["nonzero_motion_allowed"] is True
    checks = {check["id"]: check for check in result["checks"]}
    assert checks["motion.go2_control"]["ok"] is True


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("target", "dds:///rt/api/sport/request"),
        ("target", "dds://eth1/rt/api/sport/request"),
        ("control_owner", "sdk2"),
    ],
)
def test_motion_stage_rejects_invalid_go2_control(field: str, value: str) -> None:
    snapshot = _go2_snapshot()
    snapshot["status_files"]["driver"]["json"]["adapter"][field] = value

    result = evaluate_teleop_preflight(snapshot, stage="motion")

    assert result["ok"] is False
    assert "motion.go2_control" in result["blockers"]


@pytest.mark.parametrize(
    ("target", "initial_zero_acknowledged"),
    [
        ("127.0.0.1:13145", True),
        ("localhost:13145", True),
        ("192.168.66.13:13145", True),
        ("192.168.66.12:13145", False),
    ],
)
def test_motion_stage_rejects_loopback_or_unconfirmed_initial_zero(
    target: str, initial_zero_acknowledged: bool
) -> None:
    snapshot = _snapshot()
    driver = snapshot["status_files"]["driver"]["json"]
    driver["adapter"]["target"] = target
    driver["control"]["initial_zero_acknowledged"] = initial_zero_acknowledged

    result = evaluate_teleop_preflight(snapshot, stage="motion")

    assert result["ok"] is False
    assert "motion.brainstem_control" in result["blockers"]


def test_motion_stage_requires_exactly_one_cmd_vel_writer() -> None:
    snapshot = _snapshot()
    snapshot["status_files"]["driver"]["json"]["dds"]["matched_cmd_vel_writers"] = 2

    result = evaluate_teleop_preflight(snapshot, stage="motion")

    assert result["ok"] is False
    assert "motion.driver_ready" in result["blockers"]


@pytest.mark.parametrize("surface", ["missing", "driver_identity", "nav_identity"])
def test_motion_stage_requires_exact_idle_zero_driver_ack(surface: str) -> None:
    snapshot = _snapshot()
    if surface == "missing":
        snapshot["status_files"]["driver"]["json"].pop("output_ack")
    elif surface == "driver_identity":
        snapshot["status_files"]["driver"]["json"]["output_ack"]["output_sequence"] += 1
    else:
        snapshot["status_files"]["nav"]["json"]["driver_control"]["accepted_producer_boot_id"] = "stale:nav:boot"

    result = evaluate_teleop_preflight(snapshot, stage="motion")

    assert result["ok"] is False
    assert result["nonzero_motion_allowed"] is False
    assert "motion.exact_idle_zero_ack" in result["blockers"]


@pytest.mark.parametrize("extra_role", ["slam", "maps", "traversability", "lidar"])
def test_contract_rejects_non_teleop_native_roles(extra_role: str) -> None:
    snapshot = _snapshot()
    snapshot["current_run"]["verified_contract"]["selected_roles"].append(extra_role)

    result = evaluate_teleop_preflight(snapshot, stage="contract")

    assert result["ok"] is False
    assert "product.roles" in result["blockers"]


@pytest.mark.parametrize("extra_topic", ["/slam/odometry", "/maps/scene", "/nav/traversability"])
def test_contract_rejects_map_or_local_avoidance_topics(extra_topic: str) -> None:
    snapshot = _snapshot()
    snapshot["current_run"]["verified_contract"]["required_topics"].append(extra_topic)

    result = evaluate_teleop_preflight(snapshot, stage="contract")

    assert result["ok"] is False
    assert "product.topics" in result["blockers"]


def test_contract_requires_driver_ack_only_stop_policy() -> None:
    snapshot = _snapshot()
    snapshot["status_files"]["nav"]["json"]["stop_confirmation_evidence"] = "driver_ack_and_odometry"

    result = evaluate_teleop_preflight(snapshot, stage="contract")

    assert result["ok"] is False
    assert "nav.runtime_policy" in result["blockers"]


def test_contract_requires_idle_zero_and_no_authority() -> None:
    snapshot = _snapshot()
    operator_status = snapshot["status_files"]["nav"]["json"]["operator_motion"]["status"]
    operator_status["has_active_authority"] = True
    operator_status["final_cmd_vel"]["vx"] = 0.1

    result = evaluate_teleop_preflight(snapshot, stage="contract")

    assert result["ok"] is False
    assert "nav.idle_zero" in result["blockers"]


def test_collector_rejects_both_teleop_gates() -> None:
    module = _load_collector()

    with pytest.raises(ValueError, match="mutually exclusive"):
        module.build_report(
            gateway_url="http://127.0.0.1:5050",
            teleop_stage="contract",
            teleop_avoid_stage="contract",
        )


def test_collector_strict_uses_pure_teleop_gate(monkeypatch, capsys) -> None:
    module = _load_collector()
    monkeypatch.setattr(
        module,
        "parse_args",
        lambda: SimpleNamespace(
            gateway_url="http://127.0.0.1:5050",
            dds_seconds=0.0,
            dds_domain=0,
            teleop_stage="contract",
            teleop_avoid_stage=None,
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
            "teleop_preflight": {"ok": False, "blockers": ["motion.driver_ready"]},
        },
    )

    assert module.main() == 1
    assert "motion.driver_ready" in capsys.readouterr().out


def test_lingtu_cli_keeps_teleop_modes_separate_and_read_only() -> None:
    source = LINGTU_CLI.read_text(encoding="utf-8")

    assert '-m lingtu.control "$@"' in source
    assert "teleop-preflight" not in source
    assert "teleop-avoid-preflight" not in source
    assert "curl" not in source
