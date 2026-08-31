"""Product-scoped MuJoCo acceptance for native map-free teleoperation.

This runner proves the complete typed command path through navd and the native
MuJoCo driver bridge. It deliberately does not start SLAM, mapd,
traversability, a planner, or a Python velocity mux.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import uuid
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"

if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from sim.scripts.mujoco import native_navigation_acceptance as native
from sim.scripts.mujoco import teleop_avoid_native_acceptance as shared


SCHEMA_VERSION = "lingtu.mujoco.teleop_native_acceptance.v1"
DEFAULT_MANIFEST = ROOT / "config" / "runtime_graph" / "acceptance" / "mujoco_teleop_native_acceptance.json"
REQUIRED_BINARIES = (
    "sensor_publisher",
    "navigation",
    "navigation_control",
    "driver_bridge",
)
PRODUCT_PROCESSES = ("navigation", "driver")
TEST_FIXTURE_PROCESSES = ("mujoco_sensor_policy", "sensor_publisher")
POST_STOP_SAMPLES = 4
_MAX_ATTACHED_STATUS_PERIOD_S = 5.0


def _dds_domain_id(value: str) -> int:
    try:
        domain_id = int(value)
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError("DDS domain ID must be an integer") from exc
    if not 0 <= domain_id <= 232:
        raise argparse.ArgumentTypeError("DDS domain ID must be in [0, 232]")
    return domain_id


def _positive_finite_float(value: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError("value must be a number") from exc
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and greater than zero")
    return parsed


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _contract_evidence(manifest: Mapping[str, Any]) -> dict[str, Any]:
    from lingtu.products import product_lifecycle

    lifecycle = product_lifecycle("teleop")
    expected = {
        "product": "teleop",
        "source": "config/runtime_graph/products/teleop.yaml",
        "native_control_mode": lifecycle.native_control_mode,
        "slam_mode": lifecycle.slam_mode,
        "requires_map": lifecycle.requires_map,
    }
    declared_raw = manifest.get("product_contract")
    declared = dict(declared_raw) if isinstance(declared_raw, Mapping) else {}
    blockers = [
        f"teleop_product_contract_mismatch:{name}:expected={expected_value}:actual={declared.get(name)}"
        for name, expected_value in expected.items()
        if declared.get(name) != expected_value
    ]
    if str(manifest.get("extends") or ""):
        blockers.append("teleop_manifest_inheritance_forbidden")
    if manifest.get("goal") or manifest.get("map_dir") or manifest.get("map_files"):
        blockers.append("teleop_navigation_asset_forbidden")
    if manifest.get("slam_runtime") or manifest.get("traversability_runtime"):
        blockers.append("teleop_localization_or_traversability_runtime_forbidden")
    return {
        "ok": not blockers,
        "expected": expected,
        "declared": declared,
        "blockers": blockers,
    }


def _native_library_evidence(binaries: Mapping[str, Path]) -> dict[str, Any]:
    if os.name != "nt":
        return {"ok": True, "required": False, "directories": [], "blockers": []}
    directories = list(dict.fromkeys(str(Path(path).resolve().parent) for path in binaries.values()))
    runtime_dlls: dict[str, str] = {}
    blockers: list[str] = []
    for name, binary in binaries.items():
        resolved = Path(binary).resolve()
        if resolved.suffix.lower() != ".exe":
            continue
        runtime = resolved.parent / "ddsc.dll"
        if not runtime.is_file():
            blockers.append(f"native_runtime_library_missing:{name}:ddsc.dll")
            continue
        runtime_dlls[str(name)] = str(runtime)
    return {
        "ok": not blockers,
        "required": True,
        "directories": directories,
        "runtime_dlls": runtime_dlls,
        "blockers": blockers,
    }


def _process_environment(values: Mapping[str, str], binary: Path) -> dict[str, str]:
    environment = {name: str(value) for name, value in values.items()}
    if os.name == "nt":
        directory = str(Path(binary).resolve().parent)
        environment["PATH"] = os.pathsep.join([directory, str(os.environ.get("PATH") or "")])
    return environment


def prepare_runtime(args: argparse.Namespace) -> dict[str, Any]:
    """Validate pinned assets and binaries before a teleop scenario."""

    artifact_dir = Path(args.artifact_dir).expanduser().resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = Path(args.manifest).expanduser().resolve()
    manifest = native._load_manifest(manifest_path)
    contract = _contract_evidence(manifest)
    asset = shared._prepare_teleop_scene_asset(manifest, artifact_dir)
    binaries, paths, blockers, runtime_provenance = native._preflight_map_free(manifest)
    blockers = [*contract["blockers"], *blockers]
    for name in REQUIRED_BINARIES:
        if name not in binaries:
            blockers.append(f"native_binary_missing:{name}")
    unexpected = sorted(set(binaries).difference(REQUIRED_BINARIES))
    if unexpected:
        blockers.extend(f"teleop_unexpected_native_binary:{name}" for name in unexpected)
    policy = shared._policy_runtime_evidence(required=Path(paths.get("policy") or "").is_file())
    blockers.extend(str(value) for value in policy.get("blockers") or ())
    binary_provenance, stale = shared._binary_source_provenance(binaries)
    blockers.extend(stale)
    native_libraries = _native_library_evidence(binaries)
    blockers.extend(native_libraries["blockers"])
    if asset.get("ok") is not True:
        blockers.extend(str(value) for value in asset.get("blockers") or ())
        blockers.append(str(asset.get("reason") or "teleop_scene_preparation_failed"))
    clock_platform = ""
    if {"navigation", "driver_bridge"} <= set(binaries):
        try:
            clock_platform = shared._validated_driver_bridge_clock_platform(binaries)
        except ValueError:
            blockers.append("native_driver_clock_platform_mismatch")
    blockers = list(dict.fromkeys(str(value) for value in blockers))
    return {
        "ok": not blockers,
        "blockers": blockers,
        "manifest": manifest,
        "binaries": binaries,
        "paths": paths,
        "details": {
            "manifest": str(manifest_path),
            "product_contract": contract,
            "asset_preparation": asset,
            "runtime_provenance": runtime_provenance,
            "binary_provenance": binary_provenance,
            "native_libraries": native_libraries,
            "policy_runtime": policy,
            "native_clock_platform": clock_platform,
            "binaries": {name: str(path) for name, path in binaries.items()},
            "paths": {name: str(path) for name, path in paths.items()},
        },
    }


def _native_environment(host_boot_id: str, session_id: str) -> dict[str, str]:
    return {
        "LINGTU_HOST_BOOT_ID": host_boot_id,
        "LINGTU_PRODUCT_SESSION_ID": session_id,
    }


def _run_native_command(
    command: Sequence[str],
    environment: Mapping[str, str],
    *,
    timeout_s: float,
) -> dict[str, Any]:
    import subprocess

    inherited = dict(os.environ)
    inherited.update(environment)
    try:
        completed = subprocess.run(  # noqa: S603 - command is manifest-pinned
            list(command),
            cwd=ROOT,
            env=inherited,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=max(1.0, timeout_s),
            check=False,
        )
        stdout = completed.stdout or ""
        stderr = completed.stderr or ""
        events = shared.parse_operator_motion_events(f"{stdout}\n{stderr}")

        def bounded(value: str, limit: int = 8000) -> str:
            if len(value) <= limit:
                return value
            head = limit // 4
            return value[:head] + "\n... output truncated ...\n" + value[-(limit - head) :]

        return {
            "command": shared._redact_command_args(command),
            "returncode": int(completed.returncode),
            "stdout": bounded(stdout),
            "stderr": bounded(stderr),
            "events": events,
            "output_bytes": len(stdout.encode("utf-8")) + len(stderr.encode("utf-8")),
            "output_truncated": len(stdout) > 8000 or len(stderr) > 8000,
        }
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {
            "command": shared._redact_command_args(command),
            "returncode": None,
            "stdout": "",
            "stderr": f"{type(exc).__name__}:{exc}",
        }


def _post_stop_samples(
    path: Path,
    *,
    after_stamp_s: float,
    timeout_s: float = 1.5,
) -> list[dict[str, Any]]:
    deadline = time.monotonic() + timeout_s
    samples: list[dict[str, Any]] = []
    last_stamp = float(after_stamp_s)
    while time.monotonic() < deadline and len(samples) < POST_STOP_SAMPLES:
        status = _read_json(path)
        stamp = shared._nav_status_stamp_s(status)
        if stamp is not None and stamp > last_stamp:
            samples.append(status)
            last_stamp = stamp
        time.sleep(0.05)
    return samples


def _attached_post_stop_timeout_s(plan: Any) -> float:
    selected = tuple(
        process for process in getattr(plan, "processes", ()) if getattr(process, "name", None) == "nav_runtime"
    )
    if len(selected) != 1 or getattr(selected[0], "command", None) is None:
        raise ValueError("attach-only teleop must select exactly one nav_runtime command")
    argv = tuple(selected[0].command.argv)
    positions = tuple(index for index, value in enumerate(argv) if value == "--status-s")
    if len(positions) != 1 or positions[0] + 1 >= len(argv):
        raise ValueError("attach-only teleop nav_runtime must declare --status-s exactly once")
    try:
        period_s = float(argv[positions[0] + 1])
    except (TypeError, ValueError) as exc:
        raise ValueError("attach-only teleop nav_runtime status period is invalid") from exc
    if not math.isfinite(period_s) or not 0.0 < period_s <= _MAX_ATTACHED_STATUS_PERIOD_S:
        raise ValueError("attach-only teleop nav_runtime status period is invalid")
    return (POST_STOP_SAMPLES + 1) * period_s + 0.5


def run_attached(
    *,
    plan: Any,
    run_plan_path: Path,
    product_session_id: str,
    prepared: Mapping[str, Any],
    args: argparse.Namespace,
) -> dict[str, Any]:
    """Exercise teleop through DDS without starting any Product process."""

    if getattr(plan, "product", None) != "teleop":
        raise ValueError("attach-only teleop requires the teleop RunPlan")
    exact_path = run_plan_path.expanduser().resolve()
    binaries = {name: Path(path) for name, path in (prepared.get("binaries") or {}).items()}
    control_binary = binaries.get("navigation_control")
    if control_binary is None:
        raise ValueError("attach-only teleop navigation_control is unavailable")
    domain_id = int(args.domain_id)
    inherited = dict(getattr(plan, "native_process_environment", {}) or {})
    host_boot_id = str(inherited.get("LINGTU_HOST_BOOT_ID") or os.environ.get("LINGTU_HOST_BOOT_ID") or uuid.uuid4())
    environment = _native_environment(host_boot_id, product_session_id)
    forbidden_goal = shared._run_control(
        control_binary,
        ("goal", "1", "0", "0", "0", "--timeout-ms", "3000"),
        domain_id=domain_id,
        env=environment,
        timeout_s=6.0,
    )
    operator = native._native_command(
        control_binary,
        "operator-motion",
        str(float(args.command_vx)),
        "0",
        "0",
        "--duration-s",
        str(float(args.duration_s)),
        "--rate-hz",
        "10",
        "--source-id",
        f"mujoco-exact-teleop-{domain_id}",
        "--lease-ttl-ms",
        "2000",
        "--freshness-budget-ms",
        "350",
        "--cleanup-settle-ms",
        "300",
        "--timeout-ms",
        "3000",
        "--domain-id",
        str(domain_id),
    )
    operator, operator_env = shared._with_native_env(operator, **environment)
    operator_motion = _run_native_command(
        operator,
        _process_environment(operator_env, control_binary),
        timeout_s=max(12.0, float(args.duration_s) + 6.0),
    )
    nav_status_path = exact_path.parent / "nav.status.json"
    pre_stop_status_stamp_s = shared._nav_status_stamp_s(_read_json(nav_status_path))
    stop = shared._run_control(
        control_binary,
        ("stop", "teleop_exact_acceptance_stop", "--timeout-ms", "7000"),
        domain_id=domain_id,
        env=environment,
        timeout_s=9.0,
    )
    stop_ack_wall_s = time.time()
    post_stop_not_before_stamp_s = max(
        stop_ack_wall_s,
        pre_stop_status_stamp_s if pre_stop_status_stamp_s is not None else stop_ack_wall_s,
    )
    post_stop = _post_stop_samples(
        nav_status_path,
        after_stamp_s=post_stop_not_before_stamp_s,
        timeout_s=_attached_post_stop_timeout_s(plan),
    )
    case = {
        "ok": False,
        "mode": "attach_only",
        "run_plan": str(exact_path),
        "product_session_id": product_session_id,
        "min_path_length_m": float(args.min_motion_m),
        "distance_claim": {
            "path_smoke_only": True,
            "net_displacement_verified": False,
            "long_distance_verified": False,
        },
        "forbidden_goal": forbidden_goal,
        "operator_motion": operator_motion,
        "stop": stop,
        "pre_stop_status_stamp_s": pre_stop_status_stamp_s,
        "stop_ack_wall_s": stop_ack_wall_s,
        "post_stop_not_before_stamp_s": post_stop_not_before_stamp_s,
        "post_stop_samples": post_stop,
    }
    blockers = _evaluate_attached_case(case)
    case["blockers"] = blockers
    case["ok"] = not blockers
    return case


def _fresh_attached_post_stop_samples(case: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    pre_stop_stamp = case.get("pre_stop_status_stamp_s")
    stop_ack_wall_s = case.get("stop_ack_wall_s")
    if not (shared._finite_number(pre_stop_stamp) and shared._finite_number(stop_ack_wall_s)):
        return []
    last_stamp = max(float(pre_stop_stamp), float(stop_ack_wall_s))
    samples: list[Mapping[str, Any]] = []
    for item in case.get("post_stop_samples") or ():
        if not isinstance(item, Mapping):
            continue
        stamp = shared._nav_status_stamp_s(item)
        if stamp is None or stamp <= last_stamp:
            continue
        samples.append(item)
        last_stamp = stamp
    return samples


def _evaluate_attached_case(case: Mapping[str, Any]) -> list[str]:
    blockers: list[str] = []
    forbidden = case.get("forbidden_goal")
    forbidden = forbidden if isinstance(forbidden, Mapping) else {}
    if not (forbidden.get("returncode") not in (None, 0) and "goal rejected:" in str(forbidden.get("stderr") or "")):
        blockers.append("teleop_goal_mutual_exclusion_not_proven")
    motion = case.get("operator_motion")
    motion = motion if isinstance(motion, Mapping) else {}
    actions = {
        str(item.get("action") or "")
        for item in motion.get("events") or ()
        if isinstance(item, Mapping) and item.get("accepted") is True
    }
    if motion.get("returncode") != 0 or actions < {"claim", "sample", "hold", "release"}:
        blockers.append("typed_operator_motion_lifecycle_incomplete")
    stop = case.get("stop")
    stop = stop if isinstance(stop, Mapping) else {}
    if not (stop.get("returncode") == 0 and shared._native_stop_accepted(str(stop.get("stdout") or ""))):
        blockers.append("native_stop_ack_missing")
    samples = _fresh_attached_post_stop_samples(case)
    if len(samples) < POST_STOP_SAMPLES:
        blockers.append("post_stop_status_samples_missing")
    elif not all(shared._twist_is_zero(shared._final_cmd_vel_from_nav_status(item)) for item in samples):
        blockers.append("post_stop_zero_barrier_failed")
    return blockers


def build_parser() -> argparse.ArgumentParser:
    """Build the teleop acceptance command-line parser."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--run-plan", type=Path)
    parser.add_argument(
        "--artifact-dir",
        type=Path,
        default=ROOT / "artifacts" / "mujoco_teleop_native_acceptance",
    )
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--domain-id", type=_dds_domain_id, default=225)
    parser.add_argument("--duration-s", type=float, default=16.0)
    parser.add_argument("--warmup-s", type=float, default=2.0)
    parser.add_argument("--startup-timeout-s", type=float, default=20.0)
    parser.add_argument("--command-vx", type=float, default=0.18)
    parser.add_argument("--min-motion-m", type=_positive_finite_float, default=0.15)
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--attach-only", action="store_true")
    parser.add_argument("--strict", action="store_true")
    return parser
