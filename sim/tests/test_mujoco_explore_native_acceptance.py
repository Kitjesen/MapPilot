# ruff: noqa: S101

from __future__ import annotations

from pathlib import Path

from sim.scripts.mujoco import explore_native_acceptance as acceptance
from sim.scripts.mujoco import native_navigation_acceptance as native

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "config" / "runtime_graph" / "acceptance" / "mujoco_explore_native_acceptance.json"


def _fake_paths(tmp_path: Path) -> dict[str, Path]:
    return {
        "world": tmp_path / "world.xml",
        "slam_config": ROOT / "src" / "localization" / "fastlio2" / "config" / "mid360_mujoco_native_dds.yaml",
        "path_library": ROOT / "src" / "nav" / "local" / "paths",
        "sensor_runner": ROOT / "sim" / "scripts" / "mujoco" / "native_dds_sensors.py",
        "policy": tmp_path / "policy.onnx",
    }


def _fake_binaries(tmp_path: Path) -> dict[str, Path]:
    return {
        name: tmp_path / name
        for name in (
            "sensor_publisher",
            "slam",
            "mapd",
            "traversability",
            "navigation",
            "explore",
            "navigation_control",
            "cmd_vel_tap",
        )
    }


def _option(command: list[str], name: str) -> str:
    return command[command.index(name) + 1]


def test_manifest_matches_map_free_explore_product() -> None:
    manifest = native._load_manifest(MANIFEST)
    evidence = acceptance.product_contract_evidence(manifest)

    assert evidence["ok"] is True
    assert evidence["blockers"] == []
    assert manifest["product_contract"]["route"] == "live"
    assert manifest["product_contract"]["requires_map"] is False


def test_plan_uses_live_segment_without_saved_map(tmp_path: Path) -> None:
    manifest = native._load_manifest(MANIFEST)
    plan = acceptance.build_execution_plan(
        domain_id=236,
        binaries=_fake_binaries(tmp_path),
        paths=_fake_paths(tmp_path),
        case_dir=tmp_path / "case",
        duration_s=5.0,
        warmup_s=1.0,
        manifest=manifest,
    )
    processes = {item["name"]: item["command"] for item in plan["processes"]}

    assert _option(processes["navigation"], "--control-mode") == "autonomy"
    assert _option(processes["navigation"], "--teleop-local-planner") == "false"
    assert acceptance._forbidden_arguments(processes["navigation"]) == []
    assert _option(processes["explore"], "--route") == "live"
    assert "--map" not in processes["explore"]
    assert "explore" in plan["control_command"]
    assert "start" in plan["control_command"]
    assert plan["product_contract"]["requires_map"] is False


def test_evaluation_requires_segment_ack_status_path_and_zero_cleanup() -> None:
    good = {
        "startup_ok": True,
        "control": {"returncode": 0, "stdout": "accepted explore start"},
        "timeline": [
            {
                "route": "live",
                "pending_goal": None,
                "counters": {
                    "plans": 1,
                    "segment_requests": 1,
                    "segment_ack_messages": 1,
                    "segment_status_messages": 1,
                },
            }
        ],
        "nav": {"counters": {"paths": 1, "cmd_vel_published": 4}},
        "stop_zero": True,
        "cleanup_ok": True,
    }
    assert acceptance.evaluate_case(good)["ok"] is True

    bad = dict(good)
    bad["timeline"] = [{"route": "map", "pending_goal": {"request_id": "wrong"}, "counters": {}}]
    result = acceptance.evaluate_case(bad)
    assert result["ok"] is False
    assert "explore_route_not_live" in result["blockers"]
    assert "live_route_used_generic_goal" in result["blockers"]
    assert "explore_evidence_missing:segment_requests" in result["blockers"]
