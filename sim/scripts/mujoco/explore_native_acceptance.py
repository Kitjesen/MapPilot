#!/usr/bin/env python3
"""Strict map-free MuJoCo acceptance for the native Explore Product."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

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


def _forbidden_arguments(command: Sequence[str]) -> list[str]:
    forbidden = {"--map", "--map-root", "--planner-map", "--track-against-map"}
    return sorted(forbidden.intersection(command))


def product_contract_evidence(manifest: Mapping[str, Any]) -> dict[str, Any]:
    """Validate that the acceptance manifest names the real map-free Product."""

    contract = dict(manifest.get("product_contract") or {})
    blockers: list[str] = []
    expected = {
        "product": "explore",
        "native_control_mode": "autonomy",
        "slam_mode": "mapping",
        "requires_map": False,
        "route": "live",
    }
    for field, expected_value in expected.items():
        if contract.get(field) != expected_value:
            blockers.append(f"product_contract_mismatch:{field}")

    source = ROOT / str(contract.get("source") or "")
    try:
        product = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        product = {}
        blockers.append("explore_product_source_unreadable")
    if product:
        if product.get("slam_mode") != "mapping" or product.get("requires_map") is not False:
            blockers.append("explore_product_not_map_free_mapping")
        if product.get("native_control_mode") != "autonomy":
            blockers.append("explore_product_not_autonomy")
        capabilities = set(product.get("required_capabilities") or ())
        if "rolling_map_segment_execution" not in capabilities:
            blockers.append("explore_product_live_segment_missing")
        if "octoplanner3d_global_planning" in capabilities:
            blockers.append("explore_product_static_planner_forbidden")
    return {
        "ok": not blockers,
        "blockers": blockers,
        "manifest": contract,
        "source": str(source),
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
    """Build the exact native process and control commands for one Live run."""

    from sim.scripts.mujoco import native_navigation_acceptance as native
    from sim.scripts.mujoco import teleop_avoid_native_acceptance as teleop

    plan = teleop.build_execution_plan(
        scenario="free",
        domain_id=domain_id,
        binaries=binaries,
        paths=paths,
        case_dir=case_dir,
        duration_s=duration_s,
        warmup_s=warmup_s,
        command_vx=0.0,
        manifest=manifest,
        external_arm_timeout_s=max(60.0, duration_s + warmup_s + 30.0),
    )
    artifacts = dict(plan["artifacts"])
    artifacts["explore_status"] = str(case_dir / "explore_status.json")
    artifacts["explore_control"] = str(case_dir / "explore_control.log")

    processes: list[dict[str, Any]] = []
    for item in plan["processes"]:
        process = dict(item)
        if process["name"] == "navigation":
            command = _set_option(process["command"], "--control-mode", "autonomy")
            command = _set_option(command, "--teleop-local-planner", "false")
            command = _set_option(command, "--allow-teleop-takeover", "false")
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
                "live",
                "--tick-hz",
                "2",
                "--status-file",
                native._linux_arg(Path(artifacts["explore_status"])),
            )
            processes.append(
                {
                    "name": "explore",
                    "command": explore_command,
                    "log": str(case_dir / "explore.log"),
                }
            )
        processes.append(process)

    session_id = f"mujoco-explore-{domain_id}"
    request_id = f"mujoco-explore-start-{domain_id}"
    control = native._native_command(
        Path(binaries["navigation_control"]),
        "explore",
        "start",
        session_id,
        "mujoco_live_acceptance",
        "--request-id",
        request_id,
        "--domain-id",
        str(domain_id),
        "--timeout-ms",
        "5000",
    )
    return {
        **plan,
        "scenario": "live",
        "product_contract": {
            "product": "explore",
            "native_control_mode": "autonomy",
            "slam_mode": "mapping",
            "requires_map": False,
            "route": "live",
        },
        "processes": processes,
        "control_command": control,
        "session_id": session_id,
        "request_id": request_id,
        "artifacts": artifacts,
    }


def prepare_runtime(args: argparse.Namespace) -> dict[str, Any]:
    """Resolve scene, binaries, source freshness, and map-free Product inputs."""

    from sim.scripts.mujoco import native_navigation_acceptance as native
    from sim.scripts.mujoco import teleop_avoid_native_acceptance as teleop

    artifact_dir = Path(args.artifact_dir).expanduser().resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = Path(args.manifest).expanduser().resolve()
    manifest = native._load_manifest(manifest_path)
    product_contract = product_contract_evidence(manifest)
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
        "cmd_vel_tap",
    }
    blockers.extend(product_contract["blockers"])
    blockers.extend(
        f"native_binary_missing:{name}" for name in sorted(required.difference(binaries))
    )
    binary_provenance, stale = teleop._binary_source_provenance(binaries)
    blockers.extend(stale)
    policy = teleop._policy_runtime_evidence(required=Path(paths.get("policy") or "").is_file())
    blockers.extend(policy["blockers"])
    if asset_preparation.get("ok") is not True:
        blockers.append(str(asset_preparation.get("reason") or "asset_preparation_failed"))
    map_paths = sorted(
        name for name in paths if name in {"map_dir", "planner", "metadata", "slam_map"}
    )
    if map_paths:
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
            "product_contract": product_contract,
            "asset_preparation": asset_preparation,
            "runtime_provenance": provenance,
            "binary_provenance": binary_provenance,
            "policy_runtime": policy,
            "paths": {name: str(path) for name, path in paths.items()},
            "binaries": {name: str(path) for name, path in binaries.items()},
        },
    }


def evaluate_case(evidence: Mapping[str, Any]) -> dict[str, Any]:
    """Apply strict Live-segment, motion, stop, and cleanup evidence gates."""

    blockers: list[str] = []
    if evidence.get("startup_ok") is not True:
        blockers.append(str(evidence.get("startup_reason") or "runtime_not_ready"))
    control = dict(evidence.get("control") or {})
    if control.get("returncode") != 0 or "accepted explore start" not in str(control.get("stdout") or ""):
        blockers.append("explore_start_not_accepted")
    timeline = list(evidence.get("timeline") or ())
    if not timeline:
        blockers.append("explore_status_missing")
    if any(str(item.get("route") or "") != "live" for item in timeline):
        blockers.append("explore_route_not_live")
    if any(item.get("pending_goal") is not None for item in timeline):
        blockers.append("live_route_used_generic_goal")
    maxima = {
        name: max((int((item.get("counters") or {}).get(name) or 0) for item in timeline), default=0)
        for name in (
            "plans",
            "segment_requests",
            "segment_ack_messages",
            "segment_status_messages",
        )
    }
    for name, value in maxima.items():
        if value <= 0:
            blockers.append(f"explore_evidence_missing:{name}")
    nav = dict(evidence.get("nav") or {})
    nav_counters = dict(nav.get("counters") or {})
    if int(nav_counters.get("paths") or 0) <= 0:
        blockers.append("native_local_path_missing")
    if int(nav_counters.get("cmd_vel_published") or 0) <= 0:
        blockers.append("native_cmd_vel_missing")
    if evidence.get("stop_zero") is not True:
        blockers.append("explore_stop_zero_not_proven")
    if evidence.get("cleanup_ok") is not True:
        blockers.append("owned_process_cleanup_failed")
    return {"ok": not blockers, "blockers": blockers, "maxima": maxima}


def _is_zero_output(nav: Mapping[str, Any]) -> bool:
    twist = dict(nav.get("final_cmd_vel") or {})
    return all(abs(float(twist.get(name) or 0.0)) <= 1e-4 for name in ("vx", "vy", "wz"))


def execute_case(prepared: Mapping[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    """Run one isolated native Live exploration case and collect bounded evidence."""

    from sim.scripts.mujoco import native_navigation_acceptance as native
    from sim.scripts.mujoco import teleop_avoid_native_acceptance as teleop

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
    artifacts = {name: Path(value) for name, value in plan["artifacts"].items()}
    for path in artifacts.values():
        if path.is_file():
            path.unlink()
    teleop.build_scene_variant(paths["world"], artifacts["scene"], "free")
    processes = [
        native.ManagedProcess(str(item["name"]), list(item["command"]), Path(item["log"]))
        for item in plan["processes"]
    ]
    by_name = {process.name: process for process in processes}
    timeline: list[dict[str, Any]] = []
    control: dict[str, Any] = {}
    stop: dict[str, Any] = {}
    startup_ok = False
    startup_reason = "not_started"
    stop_zero = False
    cleanup: list[dict[str, Any]] = []
    mapd_not_before_ns = time.time_ns()
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
            mapd = teleop.mapd_status_evidence(
                artifacts["mapd_status"], required=True, not_before_ns=mapd_not_before_ns
            )
            if (
                str(slam.get("state") or "").upper() == "TRACKING"
                and str(nav.get("control_mode") or "") == "autonomy"
                and bool((nav.get("input_gate") or {}).get("ready"))
                and int((terrain.get("counters") or {}).get("published") or 0) > 0
                and mapd.get("ok") is True
                and explore.get("ready") is True
                and explore.get("route") == "live"
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
            completed = subprocess.run(  # noqa: S603 - command is provenance-checked native code.
                plan["control_command"],
                cwd=ROOT,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=10.0,
                check=False,
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
        "cleanup": cleanup,
        "cleanup_ok": all(item.get("clean") is True for item in cleanup),
        "plan": plan,
    }
    return {**evidence, "evaluation": evaluate_case(evidence)}


def build_parser() -> argparse.ArgumentParser:
    """Create the command-line parser."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--artifact-dir", type=Path, required=True)
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
    report = {
        "schema_version": REPORT_SCHEMA,
        "ok": ok,
        "strict": bool(args.strict),
        "preflight_only": bool(args.preflight_only),
        "preflight": preflight,
        "blockers": blockers,
        "case": case,
    }
    output = Path(args.json_out or (Path(args.artifact_dir) / "report.json")).resolve()
    _write_json(output, report)
    report["report_path"] = str(output)
    return report


def main(argv: Sequence[str] | None = None) -> int:
    """Run the acceptance command."""

    args = build_parser().parse_args(argv)
    report = run(args)
    print(json.dumps({"ok": report["ok"], "blockers": report["blockers"], "report": report["report_path"]}))
    return 1 if args.strict and not report["ok"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
