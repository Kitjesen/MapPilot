#!/usr/bin/env python3
"""Strict live/map MuJoCo component acceptance for the native Explore Product."""

from __future__ import annotations

import argparse
import json
import os
import secrets
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from sim.scripts.mujoco.product_acceptance import classify_evidence  # noqa: E402

from lingtu.sim.acceptance import validate_runner_plan  # noqa: E402
from lingtu.switch_contracts import new_product_session_id  # noqa: E402

DEFAULT_MANIFEST = (
    ROOT / "config" / "runtime_graph" / "acceptance" / "mujoco_explore_native_acceptance.json"
)
REPORT_SCHEMA = "lingtu.mujoco.explore_native_acceptance.report.v1"


def _write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=True, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _set_option(command: Sequence[str], option: str, value: str) -> list[str]:
    result = list(command)
    if option in result:
        index = result.index(option)
        if index + 1 >= len(result):
            raise ValueError(f"missing value after {option}")
        result[index + 1] = value
    else:
        result.extend([option, value])
    return result


def _option_value(command: Sequence[str], option: str) -> str:
    try:
        index = list(command).index(option)
    except ValueError:
        return ""
    return str(command[index + 1]) if index + 1 < len(command) else ""


def _matches_generated_path(argument: str, path: str) -> bool:
    from sim.scripts.mujoco import native_navigation_acceptance as native

    expected = Path(path)
    return (
        argument == native._linux_arg(expected)
        or Path(argument).expanduser().resolve() == expected.expanduser().resolve()
    )


def _forbidden_arguments(command: Sequence[str]) -> list[str]:
    forbidden = {"--map", "--map-root", "--planner-map", "--track-against-map"}
    return sorted(forbidden.intersection(command))


def _route_contract(manifest: Mapping[str, Any]) -> tuple[str, str, bool]:
    contract = dict(manifest.get("product_contract") or {})
    route = str(contract.get("route") or "").strip().lower()
    if route not in {"live", "map"}:
        raise ValueError("explore product contract route must be live or map")
    slam_mode = str(contract.get("slam_mode") or "").strip().lower()
    requires_map = contract.get("requires_map")
    expected = ("mapping", False) if route == "live" else ("localization", True)
    if (slam_mode, requires_map) != expected:
        raise ValueError(
            f"explore {route} requires slam_mode={expected[0]} and "
            f"requires_map={str(expected[1]).lower()}"
        )
    return route, slam_mode, bool(requires_map)


def _merge_process_environment(
    process: dict[str, Any], values: Mapping[str, str]
) -> None:
    """Replace exact process-local variables without changing the parent Host."""

    command = list(process["command"])
    prefix_length = 0
    if os.name == "nt" and len(command) >= 3 and command[1] == "-e":
        if command[2] != "env":
            command = [*command[:2], "env", *command[2:]]
        prefix_length = 3
    elif os.name != "nt":
        if not command or command[0] != "env":
            command = ["env", *command]
        prefix_length = 1
    if prefix_length:
        assignments: dict[str, str] = {}
        tail_index = prefix_length
        while tail_index < len(command) and "=" in command[tail_index]:
            name, value = command[tail_index].split("=", 1)
            assignments[name] = value
            tail_index += 1
        assignments.update({name: str(value) for name, value in values.items()})
        process["command"] = [
            *command[:prefix_length],
            *(f"{name}={value}" for name, value in assignments.items()),
            *command[tail_index:],
        ]
        process["env"] = {}
        return
    process["command"] = command
    process["env"] = {
        **dict(process.get("env") or {}),
        **{name: str(value) for name, value in values.items()},
    }


def _effective_process_environment(process: Mapping[str, Any]) -> dict[str, str]:
    environment = {name: str(value) for name, value in dict(process.get("env") or {}).items()}
    command = list(process.get("command") or ())
    start = 3 if len(command) >= 3 and command[1:3] == ["-e", "env"] else 1
    if command and (command[0] == "env" or start == 3):
        for argument in command[start:]:
            if "=" not in str(argument):
                break
            name, value = str(argument).split("=", 1)
            environment[name] = value
    return environment


def _status_map_matches(value: object, expected: Mapping[str, Any]) -> bool:
    if not isinstance(value, Mapping):
        return False
    return all(value.get(name) == expected_value for name, expected_value in expected.items())


def _managed_processes(processes: Sequence[Mapping[str, Any]]) -> list[Any]:
    from sim.scripts.mujoco import native_navigation_acceptance as native

    return [
        native.ManagedProcess(
            str(item["name"]),
            list(item["command"]),
            Path(item["log"]),
            env=dict(item.get("env") or {}) or None,
        )
        for item in processes
    ]


def product_contract_evidence(manifest: Mapping[str, Any]) -> dict[str, Any]:
    """Validate that the manifest names one exact Explore Product variant."""

    contract = dict(manifest.get("product_contract") or {})
    blockers: list[str] = []
    try:
        route, slam_mode, requires_map = _route_contract(manifest)
    except ValueError as exc:
        route, slam_mode, requires_map = "", "", False
        blockers.append(f"product_contract_invalid:{exc}")
    expected = {
        "product": "explore",
        "native_control_mode": "autonomy",
        "slam_mode": slam_mode,
        "requires_map": requires_map,
        "route": route,
    }
    for field, expected_value in expected.items():
        if contract.get(field) != expected_value:
            blockers.append(f"product_contract_mismatch:{field}")

    return {
        "ok": not blockers,
        "blockers": blockers,
        "manifest": contract,
    }


def build_execution_plan(
    *,
    domain_id: int,
    binaries: Mapping[str, Path],
    paths: Mapping[str, Path],
    case_dir: Path,
    duration_s: float,
    warmup_s: float,
    manifest: Mapping[str, Any],
) -> dict[str, Any]:
    """Build the exact native process and control commands for one route."""

    from sim.scripts.mujoco import native_navigation_acceptance as native
    from sim.scripts.mujoco import teleop_avoid_native_acceptance as teleop

    route, slam_mode, requires_map = _route_contract(manifest)
    if requires_map:
        missing = sorted(
            name for name in ("map_dir", "slam", "planner", "metadata") if name not in paths
        )
        if missing:
            raise ValueError(f"saved-map explore inputs are missing: {missing}")
    case_dir = case_dir.resolve()
    case_dir.mkdir(parents=True, exist_ok=True)
    host_boot_id = secrets.token_hex(16)
    external_arm_token = secrets.token_hex(16)
    external_arm_timeout = max(60.0, float(duration_s) + float(warmup_s) + 30.0)
    artifacts = {
        "scene": str(case_dir / "scene.xml"),
        "slam_status": str(case_dir / "slam_status.json"),
        "slam_cloud_dir": str(case_dir / "slam_clouds"),
        "mapd_status": str(case_dir / "mapd_status.json"),
        "traversability_status": str(case_dir / "traversability_status.json"),
        "nav_status": str(case_dir / "nav_status.json"),
        "sensor_report": str(case_dir / "sensor_report.json"),
        "parent_sensor_diagnostics": str(case_dir / "parent_sensor_diagnostics.json"),
        "sensor_arm": str(case_dir / "sensor_arm.json"),
        "sensor_arm_status": str(case_dir / "sensor_arm_status.json"),
        "motion_log": str(case_dir / "motion.jsonl"),
        "nav_timeline": str(case_dir / "nav_timeline.jsonl"),
        "sensor_publisher_pid": str(case_dir / "sensor_publisher.pid"),
        "driver_bridge_pid": str(case_dir / "driver_bridge.pid"),
    }
    Path(artifacts["slam_cloud_dir"]).mkdir(parents=True, exist_ok=True)

    start = [float(value) for value in manifest.get("start") or [0.0, 0.0, 0.48, 0.0]]
    tolerances = dict(manifest.get("runtime_tolerances") or {})
    slam_runtime = dict(manifest.get("slam_runtime") or {})
    state_provider = str(slam_runtime.get("provider") or "fastlio2").strip().lower()
    if state_provider != "fastlio2":
        raise ValueError(f"unsupported explore slam provider: {state_provider}")

    slam_command = native._native_command(
        Path(binaries["slam"]),
        "--backend",
        "fastlio2",
        "--mode",
        "mapping",
        "--config",
        native._native_path_arg(Path(binaries["slam"]), Path(paths["slam_config"])),
        "--domain-id",
        str(domain_id),
        "--tick-hz",
        "50",
        "--status-json",
        native._native_path_arg(Path(binaries["slam"]), Path(artifacts["slam_status"])),
        "--status-json-hz",
        str(float(slam_runtime.get("status_json_hz") or 10.0)),
        "--cloud-snapshot-dir",
        native._native_path_arg(Path(binaries["slam"]), Path(artifacts["slam_cloud_dir"])),
        "--cloud-snapshot-hz",
        str(float(slam_runtime.get("cloud_snapshot_hz") or 2.0)),
    )

    map_root = (
        Path(paths["map_dir"]).resolve().parent
        if requires_map
        else case_dir / "maps"
    )
    mapd_command, mapd_environment = native._native_mapd_launch(
        binary=Path(binaries["mapd"]),
        domain_id=domain_id,
        status_file=Path(artifacts["mapd_status"]),
        map_root=map_root,
        runtime=dict(manifest.get("mapd_runtime") or {}),
        environment={},
    )

    traversability_command = native._native_command(
        Path(binaries["traversability"]),
        "--domain-id",
        str(domain_id),
        "--publish-hz",
        "10",
        "--slow-hz",
        "5",
        "--tick-hz",
        "20",
        "--resolution",
        "0.2",
        "--radius",
        "6",
        "--max-points",
        "5000",
        "--status-file",
        native._native_path_arg(
            Path(binaries["traversability"]), Path(artifacts["traversability_status"])
        ),
    )

    navigation_binary = Path(binaries["navigation"])
    local_planner = native._local_planner_backend(dict(manifest))
    navigation_args = [
        "--control-mode",
        "autonomy",
        "--local-planner",
        local_planner,
        "--domain-id",
        str(domain_id),
        "--tick-hz",
        "20",
        "--publish-cmd-vel",
        "true",
        "--teleop-local-planner",
        "false",
        "--allow-teleop-takeover",
        "false",
        "--check-obstacle",
        "true",
        "--use-traversability-cost",
        "true",
        "--status-file",
        native._native_path_arg(navigation_binary, Path(artifacts["nav_status"])),
        "--status-s",
        "0.1",
    ]
    if local_planner == "cmu":
        navigation_args.extend(
            [
                "--path-library",
                native._native_path_arg(navigation_binary, Path(paths["path_library"])),
            ]
        )
    navigation_command = native._native_command(navigation_binary, *navigation_args)

    sensor_args = [
        sys.executable,
        str(Path(paths["sensor_runner"])),
        "--world",
        str(Path(paths["world"])),
        "--start",
        ",".join(str(value) for value in start[:3]),
        "--start-anchor",
        "warmup",
        "--duration",
        str(max(1.0, float(duration_s))),
        "--settle-s",
        "1.0",
        "--warmup-s",
        str(max(0.0, float(warmup_s))),
        "--drive-ramp-s",
        "0",
        "--drive-mode",
        "policy",
        "--policy-path",
        str(Path(paths["policy"])),
        "--command-source",
        "dds",
        "--driver-bridge-bin",
        str(Path(binaries["driver_bridge"])),
        "--driver-expected-host-boot-id",
        host_boot_id,
        "--driver-max-linear-mps",
        str(float((manifest.get("driver_bridge") or {}).get("max_linear_mps") or 1.0)),
        "--driver-max-angular-rps",
        str(float((manifest.get("driver_bridge") or {}).get("max_angular_rps") or 1.0)),
        "--driver-command-timeout-ms",
        str(int((manifest.get("driver_bridge") or {}).get("command_timeout_ms") or 200)),
        "--driver-heartbeat-timeout-ms",
        str(int((manifest.get("driver_bridge") or {}).get("heartbeat_timeout_ms") or 500)),
        "--driver-apply-timeout-ms",
        str(int((manifest.get("driver_bridge") or {}).get("apply_timeout_ms") or 500)),
        "--publisher-bin",
        str(Path(binaries["sensor_publisher"])),
        "--publisher-pid-file",
        str(Path(artifacts["sensor_publisher_pid"])),
        "--domain-id",
        str(domain_id),
        "--slam-status-json",
        str(Path(artifacts["slam_status"])),
        "--require-slam-output",
        "--nav-status-json",
        str(Path(artifacts["nav_status"])),
        "--external-arm-file",
        str(Path(artifacts["sensor_arm"])),
        "--external-arm-token",
        external_arm_token,
        "--external-arm-timeout-s",
        f"{external_arm_timeout:g}",
        "--external-arm-status-json",
        str(Path(artifacts["sensor_arm_status"])),
        "--external-arm-scenario",
        "free",
        "--json-out",
        str(Path(artifacts["sensor_report"])),
        *native._parent_sensor_diagnostics_args(
            Path(artifacts["parent_sensor_diagnostics"]),
            tolerances,
        ),
        *native._sensor_runtime_args(dict(manifest)),
    ]
    sensor_command = sensor_args

    plan = {
        "scenario": route,
        "scene_variant": "free",
        "domain_id": int(domain_id),
        "start": start,
        "processes": [
            {
                "name": "slam",
                "command": slam_command,
                "log": str(case_dir / "slam.log"),
            },
            {
                "name": "mapd",
                "command": mapd_command,
                "env": dict(mapd_environment),
                "log": str(case_dir / "mapd.log"),
            },
            {
                "name": "traversability",
                "command": traversability_command,
                "log": str(case_dir / "traversability.log"),
            },
            {
                "name": "navigation",
                "command": navigation_command,
                "log": str(case_dir / "navigation.log"),
            },
            {
                "name": "sensor",
                "command": sensor_command,
                "log": str(case_dir / "sensor.log"),
            },
        ],
        "artifacts": artifacts,
        "external_arm": {
            "required": True,
            "token": external_arm_token,
            "timeout_s": external_arm_timeout,
        },
        "host_boot_id": host_boot_id,
        "driver_sample_semantics": teleop.DRIVER_SAMPLE_SEMANTICS,
        "native_control_env": {"LINGTU_HOST_BOOT_ID": host_boot_id},
    }
    product_session_id = new_product_session_id()
    identity_environment = {
        "LINGTU_ENV": "sim",
        "LINGTU_PRODUCT": "explore",
        "LINGTU_PRODUCT_SESSION_ID": product_session_id,
    }
    artifacts["explore_status"] = str(case_dir / "explore_status.json")
    artifacts["explore_control"] = str(case_dir / "explore_control.log")

    map_environment: dict[str, str] = {}
    map_root: Path | None = None
    if requires_map:
        map_root = Path(paths["map_dir"]).resolve().parent
        map_environment = native._native_map_identity(
            paths={name: Path(value) for name, value in paths.items()},
            phase="explore-map",
            domain_id=domain_id,
            session_root=map_root,
        )
        map_id = Path(paths["map_dir"]).name
        map_environment.update(
            {
                "LINGTU_EXPLORE_ROUTE": "map",
                "LINGTU_PRODUCT_SESSION_ID": product_session_id,
                "LINGTU_MAP_ID": map_id,
            }
        )

    processes: list[dict[str, Any]] = []
    for item in plan["processes"]:
        process = dict(item)
        process["command"] = list(item["command"])
        if process["name"] == "slam" and requires_map:
            command = _set_option(process["command"], "--mode", "localization")
            command = _set_option(
                command,
                "--map",
                native._native_path_arg(Path(binaries["slam"]), Path(paths["slam"])),
            )
            command = _set_option(command, "--track-against-map-period-s", "1.0")
            process["command"] = command
        if process["name"] == "traversability" and requires_map:
            _merge_process_environment(process, map_environment)
        if process["name"] == "navigation":
            command = _set_option(process["command"], "--control-mode", "autonomy")
            command = _set_option(
                command, "--teleop-local-planner", "true" if requires_map else "false"
            )
            command = _set_option(
                command, "--allow-teleop-takeover", "true" if requires_map else "false"
            )
            if requires_map:
                command = _set_option(
                    command,
                    "--map",
                    native._native_path_arg(
                        Path(binaries["navigation"]), Path(paths["planner"])
                    ),
                )
                command = _set_option(
                    command,
                    "--map-root",
                    native._native_path_arg(Path(binaries["navigation"]), map_root),
                )
            else:
                invalid = _forbidden_arguments(command)
                if invalid:
                    raise ValueError(f"map-free navigation command contains {invalid}")
            process["command"] = command
        if process["name"] == "sensor":
            explore_command = native._native_command(
                Path(binaries["explore"]),
                "--domain",
                str(domain_id),
                "--route",
                route,
                "--tick-hz",
                "2",
                "--status-file",
                native._linux_arg(Path(artifacts["explore_status"])),
            )
            if requires_map:
                explore_command = _set_option(
                    explore_command,
                    "--map-root",
                    native._native_path_arg(Path(binaries["explore"]), map_root),
                )
            explore_process = {
                "name": "explore",
                "command": explore_command,
                "log": str(case_dir / "explore.log"),
            }
            if requires_map:
                _merge_process_environment(explore_process, map_environment)
            processes.append(explore_process)
        processes.append(process)

    for process in processes:
        _merge_process_environment(process, {**identity_environment, **map_environment})

    request_id = f"mujoco-explore-{route}-start-{domain_id}"
    control = native._native_command(
        Path(binaries["navigation_control"]),
        "explore",
        "start",
        product_session_id,
        f"mujoco_{route}_acceptance",
        "--request-id",
        request_id,
        "--domain-id",
        str(domain_id),
        "--timeout-ms",
        "5000",
    )
    return {
        **plan,
        "scenario": route,
        "product_contract": {
            "product": "explore",
            "native_control_mode": "autonomy",
            "slam_mode": slam_mode,
            "requires_map": requires_map,
            "route": route,
        },
        "processes": processes,
        "control_command": control,
        "session_id": product_session_id,
        "product_session_id": product_session_id,
        "request_id": request_id,
        "thresholds": dict(manifest.get("thresholds") or {}),
        "artifacts": artifacts,
        "native_control_env": {
            **dict(plan.get("native_control_env") or {}),
            **identity_environment,
            **map_environment,
        },
        "saved_map_identity": (
            {
                "map_id": map_environment["LINGTU_MAP_ID"],
                "content_epoch": int(map_environment["LINGTU_MAP_CONTENT_EPOCH"]),
            }
            if requires_map
            else None
        ),
        "saved_map_paths": (
            {
                name: str(Path(paths[name]).resolve())
                for name in ("map_dir", "slam", "planner", "metadata")
            }
            if requires_map
            else None
        ),
    }


def prepare_runtime(args: argparse.Namespace) -> dict[str, Any]:
    """Resolve scene, binaries, source freshness, and exact route inputs."""

    from sim.scripts.mujoco import native_navigation_acceptance as native
    from sim.scripts.mujoco import teleop_avoid_native_acceptance as teleop

    artifact_dir = Path(args.artifact_dir).expanduser().resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = Path(args.manifest).expanduser().resolve()
    manifest = native._load_manifest(manifest_path)
    product_contract = product_contract_evidence(manifest)
    try:
        route, _slam_mode, requires_map = _route_contract(manifest)
    except ValueError:
        route, requires_map = "", False
    if requires_map:
        asset_preparation = native._prepare_acceptance_assets(manifest, artifact_dir)
        binaries, paths, blockers, provenance = native._preflight(manifest)
    else:
        asset_preparation = teleop._prepare_teleop_scene_asset(manifest, artifact_dir)
        binaries, paths, blockers, provenance = native._preflight_map_free(manifest)
    required = {
        "sensor_publisher",
        "slam",
        "mapd",
        "traversability",
        "navigation",
        "explore",
        "navigation_control",
        "driver_bridge",
    }
    blockers.extend(product_contract["blockers"])
    blockers.extend(
        f"native_binary_missing:{name}" for name in sorted(required.difference(binaries))
    )
    native_clock_platform = ""
    if {"navigation", "driver_bridge"} <= set(binaries):
        try:
            native_clock_platform = teleop._validated_driver_bridge_clock_platform(
                binaries
            )
        except ValueError:
            blockers.append("native_driver_clock_platform_mismatch")
    binary_provenance, stale = teleop._binary_source_provenance(binaries)
    blockers.extend(stale)
    policy = teleop._policy_runtime_evidence(required=Path(paths.get("policy") or "").is_file())
    blockers.extend(policy["blockers"])
    if asset_preparation.get("ok") is not True:
        blockers.append(str(asset_preparation.get("reason") or "asset_preparation_failed"))
    map_paths = sorted(
        name for name in paths if name in {"map_dir", "planner", "metadata", "slam"}
    )
    if requires_map:
        missing_map_paths = sorted(
            {"map_dir", "planner", "metadata", "slam"}.difference(map_paths)
        )
        blockers.extend(
            f"saved_map_preflight_missing:{name}" for name in missing_map_paths
        )
    elif map_paths:
        blockers.append("map_free_preflight_resolved_saved_map")
    if os.name != "nt":
        blockers.append("host_contract_requires_windows_wsl2")
    blockers = list(dict.fromkeys(str(value) for value in blockers))
    return {
        "ok": not blockers,
        "blockers": blockers,
        "manifest": manifest,
        "binaries": binaries,
        "paths": paths,
        "details": {
            "manifest": str(manifest_path),
            "route": route,
            "product_contract": product_contract,
            "asset_preparation": asset_preparation,
            "runtime_provenance": provenance,
            "binary_provenance": binary_provenance,
            "native_clock_platform": native_clock_platform,
            "policy_runtime": policy,
            "paths": {name: str(path) for name, path in paths.items()},
            "binaries": {name: str(path) for name, path in binaries.items()},
        },
    }


def _physical_driver_bridge_evidence(evidence: Mapping[str, Any]) -> dict[str, Any]:
    from sim.scripts.mujoco.native_dds_sensors import (
        _driver_producer_matches_host,
    )

    blockers: list[str] = []
    report_value = evidence.get("sensor_report")
    report = dict(report_value) if isinstance(report_value, Mapping) else {}
    command_value = report.get("cmd_vel")
    command = dict(command_value) if isinstance(command_value, Mapping) else {}
    plan_value = evidence.get("plan")
    plan = dict(plan_value) if isinstance(plan_value, Mapping) else {}
    host_boot_id = str(plan.get("host_boot_id") or "")

    if report.get("ok") is not True:
        blockers.append("native_driver_sensor_report_failed")
    if report.get("command_source") != "dds":
        blockers.append("native_driver_command_source_not_dds")
    if command.get("driver_ready_observed") is not True:
        blockers.append("native_driver_bridge_ready_never_observed")
    if not command.get("stopped_evidence"):
        blockers.append("native_driver_bridge_stopped_evidence_missing")
    if (
        command.get("driver_ready") is not False
        or int(command.get("accepted_sequence") or 0) != 0
        or str(command.get("accepted_producer_boot_id") or "")
        or int(command.get("accepted_output_sequence") or 0) != 0
    ):
        blockers.append("native_driver_bridge_terminal_authority_not_cleared")

    samples = command.get("samples")
    if isinstance(samples, bool) or not isinstance(samples, int) or samples <= 0:
        blockers.append("native_driver_physical_applied_samples_empty")
    nonzero_samples = command.get("nonzero_samples")
    if (
        isinstance(nonzero_samples, bool)
        or not isinstance(nonzero_samples, int)
        or nonzero_samples <= 0
    ):
        blockers.append("native_driver_physical_nonzero_samples_empty")

    observed_output_ack_value = command.get("observed_output_ack")
    observed_output_ack = (
        dict(observed_output_ack_value)
        if isinstance(observed_output_ack_value, Mapping)
        else {}
    )
    producer = str(observed_output_ack.get("producer_boot_id") or "")
    output_sequence = observed_output_ack.get("output_sequence")
    output_valid = (
        not isinstance(output_sequence, bool)
        and isinstance(output_sequence, int)
        and output_sequence > 0
    )
    if (
        not host_boot_id
        or not _driver_producer_matches_host(producer, host_boot_id)
        or not output_valid
    ):
        blockers.append("native_driver_output_ack_identity_mismatch")

    raw_gaps = report.get("remaining_gaps")
    driver_gaps: list[str] = []
    if not isinstance(raw_gaps, list) or any(
        not isinstance(item, str) for item in raw_gaps
    ):
        blockers.append("native_driver_sensor_gaps_invalid")
    else:
        driver_gaps = [
            item for item in raw_gaps if item.startswith("native_driver_bridge_")
        ]
        blockers.extend(driver_gaps)
    blockers = list(dict.fromkeys(blockers))
    return {
        "ok": not blockers,
        "blockers": blockers,
        "samples": samples,
        "nonzero_samples": nonzero_samples,
        "accepted_producer_boot_id": producer,
        "accepted_output_sequence": output_sequence,
        "remaining_driver_gaps": driver_gaps,
    }


def evaluate_case(evidence: Mapping[str, Any]) -> dict[str, Any]:
    """Apply strict route, motion, stop, and cleanup evidence gates."""

    blockers: list[str] = []
    if evidence.get("startup_ok") is not True:
        blockers.append(str(evidence.get("startup_reason") or "runtime_not_ready"))
    control = dict(evidence.get("control") or {})
    if control.get("returncode") != 0 or "accepted explore start" not in str(control.get("stdout") or ""):
        blockers.append("explore_start_not_accepted")
    timeline = list(evidence.get("timeline") or ())
    if not timeline:
        blockers.append("explore_status_missing")
    contract_value = evidence.get("product_contract")
    if not isinstance(contract_value, Mapping):
        plan_value = evidence.get("plan")
        plan = dict(plan_value) if isinstance(plan_value, Mapping) else {}
        contract_value = plan.get("product_contract")
    contract = dict(contract_value) if isinstance(contract_value, Mapping) else {}
    route = str(contract.get("route") or "live")
    if any(str(item.get("route") or "") != route for item in timeline):
        blockers.append(f"explore_route_not_{route}")
    route_counters = (
        ("segment_requests", "segment_ack_messages", "segment_status_messages")
        if route == "live"
        else ("goals_accepted", "goal_status_messages")
    )
    maxima = {
        name: max(
            (int((item.get("counters") or {}).get(name) or 0) for item in timeline),
            default=0,
        )
        for name in ("plans", *route_counters)
    }
    if route == "live" and any(item.get("pending_goal") is not None for item in timeline):
        blockers.append("live_route_used_generic_goal")
    if route == "map":
        segment_requests = max(
            (
                int((item.get("counters") or {}).get("segment_requests") or 0)
                for item in timeline
            ),
            default=0,
        )
        if segment_requests > 0:
            blockers.append("map_route_used_live_segment")
    for name, value in maxima.items():
        if value <= 0:
            blockers.append(f"explore_evidence_missing:{name}")
    nav = dict(evidence.get("nav") or {})
    nav_counters = dict(nav.get("counters") or {})
    if int(nav_counters.get("paths") or 0) <= 0:
        blockers.append("native_local_path_missing")
    if route == "map":
        plan_value = evidence.get("plan")
        plan = dict(plan_value) if isinstance(plan_value, Mapping) else {}
        min_global_path_points = int(
            dict(plan.get("thresholds") or {}).get("min_global_path_points") or 1
        )
        if int(nav_counters.get("global_path_points") or 0) < min_global_path_points:
            blockers.append("native_global_path_points_missing")
    if int(nav_counters.get("cmd_vel_published") or 0) <= 0:
        blockers.append("native_cmd_vel_missing")
    if route == "map":
        plan_value = evidence.get("plan")
        plan = dict(plan_value) if isinstance(plan_value, Mapping) else {}
        product_session_id = str(plan.get("product_session_id") or "")
        saved_map_identity = dict(plan.get("saved_map_identity") or {})

        readiness_contracts = (
            ("slam_readiness", "slam_runtime"),
            ("traversability_readiness", "traversability_runtime"),
            ("explore_readiness", "explore_runtime"),
        )
        for field, process_name in readiness_contracts:
            summary = evidence.get(field)
            if not isinstance(summary, Mapping):
                blockers.append(f"saved_map_{field.removesuffix('_readiness')}_readiness_missing")
                continue
            if (
                summary.get("ready") is not True
                or summary.get("product_session_id") != product_session_id
                or summary.get("product") != "explore"
                or summary.get("process") != process_name
            ):
                blockers.append(f"saved_map_{field.removesuffix('_readiness')}_identity_mismatch")

        slam_summary = dict(evidence.get("slam_readiness") or {})
        slam_details = dict(slam_summary.get("details") or {})
        if slam_summary and (
            slam_details.get("mode") != "localization"
            or slam_details.get("product_session_id") != product_session_id
            or slam_details.get("map_loaded") is not True
            or int(slam_details.get("saved_map_points") or 0) <= 0
            or slam_details.get("map_odom_tf_valid") is not True
        ):
            blockers.append("saved_map_slam_localization_evidence_invalid")

        expected_status_map = {
            "map_id": saved_map_identity.get("map_id"),
            "map_content_epoch": saved_map_identity.get("content_epoch"),
            "live": False,
        }
        explore_summary = dict(evidence.get("explore_readiness") or {})
        explore_details = dict(explore_summary.get("details") or {})
        if explore_summary and not _status_map_matches(
            explore_details.get("map"), expected_status_map
        ):
            blockers.append("saved_map_identity_mismatch:explore_readiness")
        if any(not _status_map_matches(item.get("map"), expected_status_map) for item in timeline):
            blockers.append("saved_map_identity_mismatch:explore_timeline")

        identity_environment = {
            "LINGTU_ENV": "sim",
            "LINGTU_PRODUCT": "explore",
            "LINGTU_PRODUCT_SESSION_ID": product_session_id,
            "LINGTU_MAP_ID": str(saved_map_identity.get("map_id") or ""),
            "LINGTU_MAP_CONTENT_EPOCH": str(saved_map_identity.get("content_epoch") or ""),
        }
        native_control_env = {
            name: str(value) for name, value in dict(plan.get("native_control_env") or {}).items()
        }
        if any(native_control_env.get(name) != value for name, value in identity_environment.items()):
            blockers.append("saved_map_identity_mismatch:native_control_env")
        processes = {
            str(item.get("name")): item
            for item in plan.get("processes") or ()
            if isinstance(item, Mapping)
        }
        for process_name in ("slam", "traversability", "navigation", "explore"):
            process = processes.get(process_name)
            if process is None or any(
                _effective_process_environment(process).get(name) != value
                for name, value in identity_environment.items()
            ):
                blockers.append(f"saved_map_identity_mismatch:{process_name}_env")
        saved_map_paths = dict(plan.get("saved_map_paths") or {})
        map_root = str(Path(str(saved_map_paths.get("map_dir") or "")).parent)
        path_contracts = (
            ("slam", "--map", "slam"),
            ("navigation", "--map", "planner"),
            ("navigation", "--map-root", "map_root"),
            ("explore", "--map-root", "map_root"),
        )
        expected_paths = {**saved_map_paths, "map_root": map_root}
        for process_name, option, path_name in path_contracts:
            process = processes.get(process_name)
            if (
                not saved_map_paths
                or process is None
                or not _matches_generated_path(
                    _option_value(list(process.get("command") or ()), option),
                    str(expected_paths.get(path_name) or ""),
                )
            ):
                blockers.append(f"saved_map_identity_mismatch:{process_name}_{path_name}_path")
    physical_driver = _physical_driver_bridge_evidence(evidence)
    blockers.extend(physical_driver["blockers"])
    if evidence.get("stop_zero") is not True:
        blockers.append("explore_stop_zero_not_proven")
    if evidence.get("cleanup_ok") is not True:
        blockers.append("owned_process_cleanup_failed")
    blockers = list(dict.fromkeys(blockers))
    return {
        "ok": not blockers,
        "blockers": blockers,
        "route": route,
        "maxima": maxima,
        "physical_driver_bridge": physical_driver,
    }


def _is_zero_output(nav: Mapping[str, Any]) -> bool:
    twist = dict(nav.get("final_cmd_vel") or {})
    return all(abs(float(twist.get(name) or 0.0)) <= 1e-4 for name in ("vx", "vy", "wz"))


def execute_case(prepared: Mapping[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    """Run one isolated native exploration route and collect bounded evidence."""

    from sim.scripts.mujoco import teleop_avoid_native_acceptance as teleop

    from lingtu.sim.readiness import (
        SimReadinessError,
        load_typed_readiness,
        readiness_expectation_for_process,
    )

    domain_id = int(args.domain_id)
    case_dir = Path(args.artifact_dir).expanduser().resolve() / "case"
    binaries = {name: Path(value) for name, value in prepared["binaries"].items()}
    paths = {name: Path(value) for name, value in prepared["paths"].items()}
    plan = build_execution_plan(
        domain_id=domain_id,
        binaries=binaries,
        paths=paths,
        case_dir=case_dir,
        duration_s=float(args.duration_s),
        warmup_s=float(args.warmup_s),
        manifest=prepared["manifest"],
    )
    route = str(plan["product_contract"]["route"])
    artifacts = {name: Path(value) for name, value in plan["artifacts"].items()}
    for path in artifacts.values():
        if path.is_file():
            path.unlink()
    teleop.build_scene_variant(paths["world"], artifacts["scene"], "free")
    processes = _managed_processes(plan["processes"])
    by_name = {process.name: process for process in processes}
    timeline: list[dict[str, Any]] = []
    control: dict[str, Any] = {}
    stop: dict[str, Any] = {}
    startup_ok = False
    startup_reason = "not_started"
    stop_zero = False
    cleanup: list[dict[str, Any]] = []
    started_wall_ns = time.time_ns()
    mapd_not_before_ns = started_wall_ns
    slam_readiness: Mapping[str, Any] = {}
    traversability_readiness: Mapping[str, Any] = {}
    explore_readiness: Mapping[str, Any] = {}
    try:
        for process in processes:
            process.start()
        deadline = time.monotonic() + max(1.0, float(args.startup_timeout_s))
        while time.monotonic() < deadline:
            if by_name["sensor"].poll() is not None:
                startup_reason = "sensor_exited_before_ready"
                break
            slam = _load_json(artifacts["slam_status"])
            nav = _load_json(artifacts["nav_status"])
            terrain = _load_json(artifacts["traversability_status"])
            explore = _load_json(artifacts["explore_status"])
            explore_map = dict(explore.get("map") or {})
            mapd = teleop.mapd_status_evidence(
                artifacts["mapd_status"], required=True, not_before_ns=mapd_not_before_ns
            )
            typed_ready = route != "map"
            if route == "map":
                try:
                    slam_expectation = readiness_expectation_for_process(
                        "slam_runtime", "slam.status.json"
                    )
                    traversability_expectation = readiness_expectation_for_process(
                        "traversability_runtime", "traversability.status.json"
                    )
                    explore_expectation = readiness_expectation_for_process(
                        "explore_runtime", "explore.status.json"
                    )
                    if any(
                        expectation is None
                        for expectation in (
                            slam_expectation,
                            traversability_expectation,
                            explore_expectation,
                        )
                    ):
                        raise RuntimeError("typed explore readiness adapter is unavailable")
                    slam_readiness = load_typed_readiness(
                        artifacts["slam_status"],
                        expectation=slam_expectation,
                        product_session_id=str(plan["product_session_id"]),
                        product="explore",
                        process="slam_runtime",
                        started_wall_ns=started_wall_ns,
                        expected_slam_mode="localization",
                    )
                    traversability_readiness = load_typed_readiness(
                        artifacts["traversability_status"],
                        expectation=traversability_expectation,
                        product_session_id=str(plan["product_session_id"]),
                        product="explore",
                        process="traversability_runtime",
                        started_wall_ns=started_wall_ns,
                    )
                    explore_readiness = load_typed_readiness(
                        artifacts["explore_status"],
                        expectation=explore_expectation,
                        product_session_id=str(plan["product_session_id"]),
                        product="explore",
                        process="explore_runtime",
                        started_wall_ns=started_wall_ns,
                        expected_explore_route="map",
                    )
                    typed_ready = True
                except SimReadinessError as exc:
                    startup_reason = f"typed_readiness:{exc}"
            if (
                typed_ready
                and str(slam.get("state") or "").upper() == "TRACKING"
                and str(nav.get("control_mode") or "") == "autonomy"
                and bool((nav.get("input_gate") or {}).get("ready"))
                and int((terrain.get("counters") or {}).get("published") or 0) > 0
                and mapd.get("ok") is True
                and explore.get("ready") is True
                and explore.get("route") == route
                and (
                    route != "map"
                    or (
                        bool(explore_map.get("map_id"))
                        and int(explore_map.get("map_content_epoch") or 0) > 0
                        and explore_map.get("live") is False
                    )
                )
            ):
                startup_ok = True
                startup_reason = "ready"
                break
            time.sleep(0.1)
        if startup_ok:
            arm = dict(plan["external_arm"])
            teleop.trigger_external_arm(
                artifacts["sensor_arm"],
                token=str(arm["token"]),
                domain_id=domain_id,
                scenario="free",
            )
            control_command, control_env = teleop._with_native_env(
                plan["control_command"], **dict(plan["native_control_env"])
            )
            popen_options: dict[str, Any] = {}
            if control_env:
                inherited = dict(os.environ)
                inherited.update(control_env)
                popen_options["env"] = inherited
            completed = subprocess.run(  # noqa: S603 - command is provenance-checked native code.
                control_command,
                cwd=ROOT,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=10.0,
                check=False,
                **popen_options,
            )
            control = {
                "returncode": int(completed.returncode),
                "stdout": (completed.stdout or "")[-4000:],
                "stderr": (completed.stderr or "")[-4000:],
            }
            run_deadline = time.monotonic() + max(1.0, float(args.duration_s))
            while time.monotonic() < run_deadline and by_name["sensor"].poll() is None:
                status = _load_json(artifacts["explore_status"])
                if status:
                    timeline.append(status)
                time.sleep(0.2)
            stop = teleop._run_control(
                binaries["navigation_control"],
                ["explore", "stop", "mujoco_acceptance_complete", "--request-id", f"stop-{domain_id}"],
                domain_id=domain_id,
                env=plan["native_control_env"],
            )
            zero_deadline = time.monotonic() + 3.0
            while time.monotonic() < zero_deadline:
                if _is_zero_output(_load_json(artifacts["nav_status"])):
                    stop_zero = True
                    break
                time.sleep(0.05)
    except (OSError, subprocess.SubprocessError, ValueError) as exc:
        startup_reason = f"runtime_exception:{type(exc).__name__}:{exc}"
    finally:
        for process in reversed(processes):
            process.stop()
            cleanup.append(process.cleanup)

    evidence = {
        "startup_ok": startup_ok,
        "startup_reason": startup_reason,
        "control": control,
        "stop": stop,
        "stop_zero": stop_zero,
        "timeline": timeline,
        "nav": _load_json(artifacts["nav_status"]),
        "slam": _load_json(artifacts["slam_status"]),
        "mapd": _load_json(artifacts["mapd_status"]),
        "traversability": _load_json(artifacts["traversability_status"]),
        "slam_readiness": dict(slam_readiness),
        "traversability_readiness": dict(traversability_readiness),
        "explore_readiness": dict(explore_readiness),
        "sensor_report": _load_json(artifacts["sensor_report"]),
        "cleanup": cleanup,
        "cleanup_ok": all(item.get("clean") is True for item in cleanup),
        "product_contract": dict(plan["product_contract"]),
        "plan": plan,
    }
    return {**evidence, "evaluation": evaluate_case(evidence)}


def build_parser() -> argparse.ArgumentParser:
    """Create the command-line parser."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--run-plan", type=Path)
    parser.add_argument("--artifact-dir", type=Path)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--domain-id", type=int, default=236)
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--warmup-s", type=float, default=8.0)
    parser.add_argument("--startup-timeout-s", type=float, default=45.0)
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--strict", action="store_true")
    return parser


def run(args: argparse.Namespace) -> dict[str, Any]:
    """Run preflight and, when requested, one strict native exploration case."""

    prepared = prepare_runtime(args)
    case = None
    if prepared["ok"] and not args.preflight_only:
        case = execute_case(prepared, args)
    case_blockers = list((case or {}).get("evaluation", {}).get("blockers") or ())
    blockers = [*prepared["blockers"], *case_blockers]
    ok = bool(prepared["ok"]) and (
        bool(args.preflight_only) or bool((case or {}).get("evaluation", {}).get("ok"))
    )
    preflight = dict(prepared["details"])
    preflight["ok"] = bool(prepared["ok"])
    preflight["blockers"] = list(prepared["blockers"])
    acceptance_scope = dict(prepared["manifest"].get("acceptance_scope") or {})
    report = {
        "schema_version": REPORT_SCHEMA,
        "ok": ok,
        "strict": bool(args.strict),
        "preflight_only": bool(args.preflight_only),
        "acceptance_scope": acceptance_scope,
        "preflight": preflight,
        "blockers": blockers,
        "case": case,
    }
    report.update(
        classify_evidence(
            acceptance_scope,
            run_plan_verified=bool(getattr(args, "run_plan_verified", False)),
            acceptance_evaluated=case is not None,
            ok=ok,
        )
    )
    output = Path(args.json_out or (Path(args.artifact_dir) / "report.json")).resolve()
    _write_json(output, report)
    report["report_path"] = str(output)
    return report


def main(argv: Sequence[str] | None = None) -> int:
    """Run the acceptance command."""

    args = build_parser().parse_args(argv)
    if args.run_plan is not None:
        plan = validate_runner_plan(
            ROOT,
            args.run_plan,
            args.manifest,
            expected_products=("explore",),
        )
        args.run_plan_roles = tuple(
            sorted({role for process in plan.processes for role in process.provides})
        )
        args.run_plan_verified = True
        if args.artifact_dir is None:
            args.artifact_dir = (
                args.run_plan.expanduser().resolve().parent / "acceptance" / "explore"
            )
    elif args.artifact_dir is None:
        args.artifact_dir = ROOT / "artifacts" / "mujoco_explore_native"
    report = run(args)
    print(json.dumps({"ok": report["ok"], "blockers": report["blockers"], "report": report["report_path"]}))
    return 1 if args.strict and not report["ok"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
