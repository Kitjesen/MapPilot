# ruff: noqa: S101

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from sim.scripts.mujoco import explore_native_acceptance as acceptance
from sim.scripts.mujoco import native_navigation_acceptance as native

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "config" / "runtime_graph" / "acceptance" / "mujoco_explore_native_acceptance.json"
MAP_MANIFEST = (
    ROOT
    / "config"
    / "runtime_graph"
    / "acceptance"
    / "mujoco_explore_map_native_acceptance.json"
)


def _fake_paths(tmp_path: Path) -> dict[str, Path]:
    return {
        "world": tmp_path / "world.xml",
        "slam_config": ROOT / "src" / "localization" / "fastlio2" / "config" / "sim_mid360_slam.yaml",
        "path_library": ROOT / "src" / "nav" / "local" / "paths",
        "sensor_runner": ROOT / "sim" / "scripts" / "mujoco" / "native_dds_sensors.py",
        "policy": tmp_path / "policy.onnx",
    }


def _fake_map_paths(tmp_path: Path) -> dict[str, Path]:
    map_dir = tmp_path / "maps" / "saved-map"
    return {
        **_fake_paths(tmp_path),
        "map_dir": map_dir,
        "slam": map_dir / "map.pcd",
        "planner": map_dir / "octomap.ot",
        "metadata": map_dir / "metadata.json",
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
            "driver_bridge",
        )
    }


def _option(command: list[str], name: str) -> str:
    return command[command.index(name) + 1]


def _process_environment(process: dict[str, object]) -> dict[str, str]:
    environment = dict(process.get("env") or {})
    command = list(process["command"])
    start = 3 if len(command) >= 3 and command[1:3] == ["-e", "env"] else 1
    if command and (command[0] == "env" or start == 3):
        for argument in command[start:]:
            if "=" not in str(argument):
                break
            name, value = str(argument).split("=", 1)
            environment[name] = value
    return environment


def test_manifest_matches_map_free_explore_product() -> None:
    manifest = native._load_manifest(MANIFEST)
    evidence = acceptance.product_contract_evidence(manifest)

    assert evidence["ok"] is True
    assert evidence["blockers"] == []
    assert manifest["product_contract"]["route"] == "live"
    assert manifest["product_contract"]["requires_map"] is False
    assert manifest["binaries"]["driver_bridge"]["env"] == (
        "LINGTU_MUJOCO_DRIVER_BRIDGE_BIN"
    )
    assert "cmd_vel_tap" not in manifest["binaries"]


def test_map_manifest_matches_saved_map_explore_product() -> None:
    manifest = native._load_manifest(MAP_MANIFEST)
    evidence = acceptance.product_contract_evidence(manifest)

    assert evidence["ok"] is True
    assert evidence["blockers"] == []
    assert manifest["product_contract"]["route"] == "map"
    assert manifest["product_contract"]["slam_mode"] == "localization"
    assert manifest["product_contract"]["requires_map"] is True
    assert manifest["asset_builder"]["kind"] == "saved_map_plan_gate"


def test_component_manifests_claim_terminal_correlation_only_when_verified() -> None:
    acceptance_dir = ROOT / "config" / "runtime_graph" / "acceptance"
    manifests = {
        name: native._load_manifest(acceptance_dir / name)
        for name in (
            "mujoco_native_navigation_acceptance.json",
            "mujoco_tracking_native_acceptance.json",
            "mujoco_explore_native_acceptance.json",
            "mujoco_explore_map_native_acceptance.json",
        )
    }
    for name in (
        "mujoco_native_navigation_acceptance.json",
        "mujoco_tracking_native_acceptance.json",
    ):
        claims = manifests[name]["acceptance_scope"]["claims"]
        assert any("terminal exact Driver stop" in claim for claim in claims)
    for name in (
        "mujoco_explore_native_acceptance.json",
        "mujoco_explore_map_native_acceptance.json",
    ):
        claims = manifests[name]["acceptance_scope"]["claims"]
        assert all("terminal" not in claim for claim in claims)
    assert "required_terminal_state" not in manifests[
        "mujoco_tracking_native_acceptance.json"
    ]["acceptance_scope"]


def test_report_keeps_component_evidence_outside_product_pass(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manifest = native._load_manifest(MANIFEST)
    monkeypatch.setattr(
        acceptance,
        "prepare_runtime",
        lambda _args: {
            "ok": True,
            "blockers": [],
            "manifest": manifest,
            "details": {},
        },
    )
    monkeypatch.setattr(
        acceptance,
        "execute_case",
        lambda _prepared, _args: {"evaluation": {"ok": True, "blockers": []}},
    )
    report_path = tmp_path / "report.json"
    args = acceptance.build_parser().parse_args(
        ["--artifact-dir", str(tmp_path), "--json-out", str(report_path)]
    )
    args.run_plan_verified = True

    report = acceptance.run(args)

    assert json.loads(report_path.read_text(encoding="utf-8"))[
        "product_acceptance_passed"
    ] is False
    assert report["acceptance_scope"]["coverage"] == "component"
    assert report["acceptance_evaluated"] is True
    assert report["evidence_scope"] == "component_e2e"
    assert report["product_acceptance_passed"] is False


def test_dispatcher_shape_derives_artifacts_from_exact_run_plan_session(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    session_root = tmp_path / "session"
    run_plan = session_root / "plan.json"
    observed: dict[str, object] = {}
    monkeypatch.setattr(
        acceptance,
        "validate_runner_plan",
        lambda *_args, **_kwargs: SimpleNamespace(acceptance={"roles": ["explore_runtime"]}),
    )

    def fake_run(args: object) -> dict[str, object]:
        observed["artifact_dir"] = args.artifact_dir
        return {
            "ok": True,
            "blockers": [],
            "report_path": str(Path(args.artifact_dir) / "report.json"),
        }

    monkeypatch.setattr(acceptance, "run", fake_run)

    result = acceptance.main(
        [
            "--run-plan",
            str(run_plan),
            "--manifest",
            str(MANIFEST),
            "--strict",
        ]
    )

    assert result == 0
    assert observed["artifact_dir"] == session_root / "acceptance" / "explore"


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
    sensor = processes["sensor"]
    assert _option(sensor, "--driver-bridge-bin") == str(
        _fake_binaries(tmp_path)["driver_bridge"]
    )
    assert _option(sensor, "--driver-expected-host-boot-id") == plan["host_boot_id"]
    assert plan["native_control_env"]["LINGTU_HOST_BOOT_ID"] == plan["host_boot_id"]
    assert "--cmd-vel-tap-bin" not in sensor
    assert plan["driver_sample_semantics"] == (
        "successful_mujoco_step_then_LT_DRIVER_APPLIED_V2"
    )
    assert plan["product_session_id"].startswith("product-")
    assert plan["product_session_id"] == plan["session_id"]
    assert plan["product_session_id"] in plan["control_command"]
    assert "teleop-avoid" not in json.dumps(plan)
    for process in plan["processes"]:
        environment = _process_environment(process)
        assert environment["LINGTU_ENV"] == "sim"
        assert environment["LINGTU_PRODUCT"] == "explore"
        assert environment["LINGTU_PRODUCT_SESSION_ID"] == plan["product_session_id"]
    assert plan["native_control_env"]["LINGTU_PRODUCT_SESSION_ID"] == plan["product_session_id"]


def test_managed_processes_preserve_direct_windows_process_environment(tmp_path: Path) -> None:
    process = {
        "name": "explore",
        "command": [str(tmp_path / "explore.exe")],
        "log": str(tmp_path / "explore.log"),
        "env": {"LINGTU_PRODUCT": "explore", "LINGTU_PRODUCT_SESSION_ID": "a" * 32},
    }

    managed = acceptance._managed_processes([process])

    assert managed[0].env == process["env"]


def test_plan_uses_saved_map_goal_route_with_exact_map_inputs(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setenv("LINGTU_MAP_CONTENT_EPOCH", "7")
    manifest = native._load_manifest(MAP_MANIFEST)
    paths = _fake_map_paths(tmp_path)
    paths["map_dir"].mkdir(parents=True)
    paths["slam"].write_text("pcd\n", encoding="utf-8")
    paths["planner"].write_text("octomap\n", encoding="utf-8")
    paths["metadata"].write_text("{}\n", encoding="utf-8")

    plan = acceptance.build_execution_plan(
        domain_id=237,
        binaries=_fake_binaries(tmp_path),
        paths=paths,
        case_dir=tmp_path / "case-map",
        duration_s=5.0,
        warmup_s=1.0,
        manifest=manifest,
    )
    processes = {item["name"]: item for item in plan["processes"]}
    slam = processes["slam"]["command"]
    navigation = processes["navigation"]["command"]
    explore = processes["explore"]["command"]

    assert _option(slam, "--mode") == "localization"
    assert _option(slam, "--map").endswith("map.pcd")
    assert _option(navigation, "--teleop-local-planner") == "true"
    assert _option(navigation, "--allow-teleop-takeover") == "true"
    assert _option(navigation, "--map").endswith("octomap.ot")
    assert _option(navigation, "--map-root").endswith("maps")
    assert _option(explore, "--route") == "map"
    assert _option(explore, "--map-root").endswith("maps")
    assert plan["product_contract"] == {
        "product": "explore",
        "native_control_mode": "autonomy",
        "slam_mode": "localization",
        "requires_map": True,
        "route": "map",
    }
    assert plan["native_control_env"]["LINGTU_EXPLORE_ROUTE"] == "map"
    assert plan["native_control_env"]["LINGTU_MAP_ID"] == "saved-map"
    identity = plan["saved_map_identity"]
    assert identity == {
        "map_id": "saved-map",
        "content_epoch": 7,
    }
    assert plan["saved_map_paths"] == {
        "map_dir": str(paths["map_dir"].resolve()),
        "slam": str(paths["slam"].resolve()),
        "planner": str(paths["planner"].resolve()),
        "metadata": str(paths["metadata"].resolve()),
    }
    for name in ("slam", "traversability", "navigation", "explore"):
        environment = _process_environment(processes[name])
        assert environment["LINGTU_MAP_ID"] == identity["map_id"]
        assert environment["LINGTU_MAP_CONTENT_EPOCH"] == str(identity["content_epoch"])


def test_map_evaluation_requires_goal_flow_and_forbids_live_segments() -> None:
    session_id = "a" * 32
    identity = {
        "map_id": "saved-map",
        "content_epoch": 1,
    }
    identity_environment = {
        "LINGTU_ENV": "sim",
        "LINGTU_PRODUCT": "explore",
        "LINGTU_PRODUCT_SESSION_ID": session_id,
        "LINGTU_MAP_ID": "saved-map",
        "LINGTU_MAP_CONTENT_EPOCH": "1",
    }
    slam_path = "C:/maps/saved-map/map.pcd"
    planner_path = "C:/maps/saved-map/octomap.ot"
    map_dir = "C:/maps/saved-map"
    map_root = "C:/maps"
    good = {
        "startup_ok": True,
        "control": {"returncode": 0, "stdout": "accepted explore start"},
        "product_contract": {"route": "map"},
        "timeline": [
            {
                "route": "map",
                "map": {
                    "map_id": "saved-map",
                    "map_content_epoch": 1,
                    "live": False,
                },
                "counters": {
                    "plans": 1,
                    "goals_accepted": 1,
                    "goal_status_messages": 1,
                    "segment_requests": 0,
                },
            }
        ],
        "nav": {
            "counters": {
                "paths": 1,
                "global_path_points": 2,
                "cmd_vel_published": 4,
            }
        },
        "plan": {
            "host_boot_id": "host-a",
            "product_contract": {"route": "map"},
            "product_session_id": session_id,
            "saved_map_identity": identity,
            "saved_map_paths": {
                "map_dir": map_dir,
                "slam": slam_path,
                "planner": planner_path,
                "metadata": "C:/maps/saved-map/metadata.json",
            },
            "thresholds": {"min_global_path_points": 2},
            "native_control_env": identity_environment,
            "processes": [
                {
                    "name": name,
                    "command": (
                        [name, "--map", slam_path]
                        if name == "slam"
                        else [name, "--map", planner_path, "--map-root", map_root]
                        if name == "navigation"
                        else [name, "--map-root", map_root]
                        if name == "explore"
                        else [name]
                    ),
                    "env": identity_environment,
                }
                for name in ("slam", "traversability", "navigation", "explore")
            ],
        },
        "slam_readiness": {
            "ready": True,
            "product_session_id": session_id,
            "product": "explore",
            "process": "slam_runtime",
            "details": {
                "mode": "localization",
                "product_session_id": session_id,
                "map_loaded": True,
                "saved_map_points": 10,
                "map_odom_tf_valid": True,
            },
        },
        "traversability_readiness": {
            "ready": True,
            "product_session_id": session_id,
            "product": "explore",
            "process": "traversability_runtime",
        },
        "explore_readiness": {
            "ready": True,
            "product_session_id": session_id,
            "product": "explore",
            "process": "explore_runtime",
            "details": {
                "route": "map",
                "map": {
                    "map_id": "saved-map",
                    "map_content_epoch": 1,
                    "live": False,
                },
            },
        },
        "sensor_report": {
            "ok": True,
            "command_source": "dds",
            "remaining_gaps": [],
            "cmd_vel": {
                "driver_ready": False,
                "driver_ready_observed": True,
                "accepted_sequence": 0,
                "accepted_producer_boot_id": "",
                "accepted_output_sequence": 0,
                "samples": 4,
                "nonzero_samples": 2,
                "observed_output_ack": {
                    "producer_boot_id": "host-a:1234:567890",
                    "output_sequence": 9,
                },
                "stopped_evidence": {"kind": "deactivate_zero"},
            },
        },
        "stop_zero": True,
        "cleanup_ok": True,
    }
    good_result = acceptance.evaluate_case(good)
    assert good_result["ok"] is True, good_result["blockers"]

    bad = dict(good)
    bad["timeline"] = [
        {
            "route": "map",
            "counters": {
                "plans": 1,
                "goals_accepted": 0,
                "goal_status_messages": 0,
                "segment_requests": 1,
            },
        }
    ]
    result = acceptance.evaluate_case(bad)
    assert result["ok"] is False
    assert "explore_evidence_missing:goals_accepted" in result["blockers"]
    assert "explore_evidence_missing:goal_status_messages" in result["blockers"]
    assert "map_route_used_live_segment" in result["blockers"]

    no_global_path = dict(good)
    no_global_path["nav"] = {
        "counters": {"paths": 1, "global_path_points": 1, "cmd_vel_published": 4}
    }
    result = acceptance.evaluate_case(no_global_path)
    assert "native_global_path_points_missing" in result["blockers"]

    wrong_map = dict(good)
    wrong_map["timeline"] = [
        {
            **good["timeline"][0],
            "map": {**good["timeline"][0]["map"], "map_content_epoch": 2},
        }
    ]
    result = acceptance.evaluate_case(wrong_map)
    assert "saved_map_identity_mismatch:explore_timeline" in result["blockers"]

    missing_slam = dict(good)
    missing_slam.pop("slam_readiness")
    result = acceptance.evaluate_case(missing_slam)
    assert "saved_map_slam_readiness_missing" in result["blockers"]


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
        "plan": {"host_boot_id": "host-a"},
        "sensor_report": {
            "ok": True,
            "command_source": "dds",
            "remaining_gaps": [],
            "cmd_vel": {
                "driver_ready": False,
                "driver_ready_observed": True,
                "accepted_sequence": 0,
                "accepted_producer_boot_id": "",
                "accepted_output_sequence": 0,
                "samples": 4,
                "nonzero_samples": 2,
                "observed_output_ack": {
                    "accepted_sequence": 6,
                    "producer_boot_id": "host-a:1234:567890",
                    "output_sequence": 9,
                },
                "stopped_evidence": {"kind": "deactivate_zero"},
            },
        },
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

    no_physical_apply = dict(good)
    no_physical_apply["sensor_report"] = {
        "ok": False,
        "command_source": "dds",
        "remaining_gaps": ["native_driver_bridge_ready_never_observed"],
        "cmd_vel": {
            "driver_ready": False,
            "driver_ready_observed": False,
            "samples": 0,
            "nonzero_samples": 0,
            "accepted_producer_boot_id": "",
            "accepted_output_sequence": 0,
            "observed_output_ack": {},
            "stopped_evidence": {},
        },
    }
    result = acceptance.evaluate_case(no_physical_apply)
    assert result["ok"] is False
    assert "native_driver_physical_applied_samples_empty" in result["blockers"]
    assert "native_driver_bridge_ready_never_observed" in result["blockers"]

    wrong_identity = dict(good)
    wrong_identity["sensor_report"] = {
        **good["sensor_report"],
        "cmd_vel": {
            **good["sensor_report"]["cmd_vel"],
            "observed_output_ack": {
                **good["sensor_report"]["cmd_vel"]["observed_output_ack"],
                "producer_boot_id": "other-host:1234:567890",
            },
        },
    }
    result = acceptance.evaluate_case(wrong_identity)
    assert result["ok"] is False
    assert "native_driver_output_ack_identity_mismatch" in result["blockers"]
