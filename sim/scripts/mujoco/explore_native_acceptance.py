#!/usr/bin/env python3
"""Exercise the active native Explore Product in MuJoCo."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from sim.scripts.mujoco.product_acceptance import classify_evidence  # noqa: E402

from lingtu.sim.acceptance import validate_runner_plan  # noqa: E402

DEFAULT_MANIFEST = ROOT / "config" / "acceptance" / "mujoco" / "explore.json"
REPORT_SCHEMA = "lingtu.mujoco.explore_native_acceptance.report.v1"
_REQUIRED_PROCESSES = frozenset(
    {
        "driver_bridge",
        "imu_publisher",
        "lidar_publisher",
        "map_runtime",
        "mujoco_feeder",
        "nav_runtime",
        "slam_runtime",
        "traversability_runtime",
        "explore_runtime",
    }
)


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
    return {"ok": not blockers, "blockers": blockers, "manifest": contract}


def _plan_route(plan: Any) -> tuple[str, str, bool]:
    if getattr(plan, "product", None) != "explore":
        raise ValueError("Explore acceptance requires an Explore RunPlan")
    lifecycle = dict(getattr(plan, "lifecycle", {}) or {})
    requires_map = lifecycle.get("requires_map")
    slam_mode = str(lifecycle.get("slam_mode") or "").strip().lower()
    route = str(getattr(plan, "product_variant", None) or "").strip().lower()
    if route not in {"live", "map"}:
        route = "map" if requires_map is True else "live"
    expected = ("mapping", False) if route == "live" else ("localization", True)
    if (slam_mode, requires_map) != expected:
        raise ValueError("Explore RunPlan lifecycle does not match its route")
    return route, slam_mode, bool(requires_map)


def _require_manifest_matches_plan(plan: Any, manifest: Mapping[str, Any]) -> str:
    manifest_route, manifest_slam, manifest_map = _route_contract(manifest)
    route, slam_mode, requires_map = _plan_route(plan)
    if (manifest_route, manifest_slam, manifest_map) != (
        route,
        slam_mode,
        requires_map,
    ):
        raise ValueError("Explore manifest does not match the committed RunPlan")
    names = {str(item.name) for item in getattr(plan, "processes", ())}
    missing = sorted(_REQUIRED_PROCESSES.difference(names))
    if missing:
        raise ValueError(f"Explore RunPlan is missing processes: {missing}")
    return route


def prepare_runtime(args: argparse.Namespace) -> dict[str, Any]:
    """Resolve only the command client needed to inspect an active Product."""

    from sim.scripts.mujoco import native_navigation_acceptance as native

    artifact_dir = Path(args.artifact_dir).expanduser().resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = Path(args.manifest).expanduser().resolve()
    manifest = native._load_manifest(manifest_path)
    contract = product_contract_evidence(manifest)
    blockers = list(contract["blockers"])
    binary_specs = manifest.get("binaries")
    binary_specs = binary_specs if isinstance(binary_specs, Mapping) else {}
    navigation_control_spec = binary_specs.get("navigation_control")
    navigation_control = (
        native._resolve_binary(dict(navigation_control_spec))
        if isinstance(navigation_control_spec, Mapping)
        else None
    )
    if navigation_control is None:
        blockers.append("native_binary_missing:navigation_control")
    return {
        "ok": not blockers,
        "blockers": list(dict.fromkeys(blockers)),
        "manifest": manifest,
        "binaries": (
            {"navigation_control": navigation_control}
            if navigation_control is not None
            else {}
        ),
        "details": {
            "manifest": str(manifest_path),
            "product_contract": contract,
            "navigation_control": (
                str(navigation_control) if navigation_control is not None else None
            ),
        },
    }


def _is_zero_output(nav: Mapping[str, Any]) -> bool:
    twist = nav.get("final_cmd_vel")
    if not isinstance(twist, Mapping):
        return False
    return all(abs(float(twist.get(name) or 0.0)) <= 1e-4 for name in ("vx", "vy", "wz"))


def _counter_max(timeline: Sequence[Mapping[str, Any]], name: str) -> int:
    return max(
        (int((item.get("counters") or {}).get(name) or 0) for item in timeline),
        default=0,
    )


def evaluate_case(evidence: Mapping[str, Any]) -> dict[str, Any]:
    """Evaluate Explore behavior; ProductControl owns lifecycle evidence."""

    blockers: list[str] = []
    control = evidence.get("control")
    control = control if isinstance(control, Mapping) else {}
    if control.get("returncode") != 0 or "accepted explore start" not in str(
        control.get("stdout") or ""
    ):
        blockers.append("explore_start_not_accepted")

    contract = evidence.get("product_contract")
    contract = contract if isinstance(contract, Mapping) else {}
    route = str(contract.get("route") or "live")
    timeline = [
        item for item in evidence.get("timeline") or () if isinstance(item, Mapping)
    ]
    if not timeline:
        blockers.append("explore_status_missing")
    if any(str(item.get("route") or "") != route for item in timeline):
        blockers.append(f"explore_route_not_{route}")

    required = (
        ("segment_requests", "segment_ack_messages", "segment_status_messages")
        if route == "live"
        else ("goals_accepted", "goal_status_messages")
    )
    maxima = {name: _counter_max(timeline, name) for name in ("plans", *required)}
    for name, value in maxima.items():
        if value <= 0:
            blockers.append(f"explore_evidence_missing:{name}")
    if route == "live" and any(item.get("pending_goal") is not None for item in timeline):
        blockers.append("live_route_used_generic_goal")
    if route == "map" and _counter_max(timeline, "segment_requests") > 0:
        blockers.append("map_route_used_live_segment")

    nav = evidence.get("nav")
    nav = nav if isinstance(nav, Mapping) else {}
    counters = nav.get("counters")
    counters = counters if isinstance(counters, Mapping) else {}
    if int(counters.get("paths") or 0) <= 0:
        blockers.append("native_local_path_missing")
    if int(counters.get("cmd_vel_published") or 0) <= 0:
        blockers.append("native_cmd_vel_missing")
    if route == "map":
        threshold = int(evidence.get("min_global_path_points") or 1)
        if int(counters.get("global_path_points") or 0) < threshold:
            blockers.append("native_global_path_points_missing")

    stop = evidence.get("stop")
    stop = stop if isinstance(stop, Mapping) else {}
    if stop.get("returncode") != 0:
        blockers.append("explore_stop_not_accepted")
    if evidence.get("stop_zero") is not True:
        blockers.append("explore_stop_zero_not_proven")
    blockers = list(dict.fromkeys(blockers))
    return {"ok": not blockers, "blockers": blockers, "route": route, "maxima": maxima}


def run_attached(
    *,
    plan: Any,
    run_plan_path: Path,
    product_session_id: str,
    prepared: Mapping[str, Any],
    args: argparse.Namespace,
) -> dict[str, Any]:
    """Send Explore commands to the Product already owned by ProductControl."""

    from sim.scripts.mujoco import teleop_avoid_native_acceptance as teleop

    manifest = prepared.get("manifest")
    manifest = manifest if isinstance(manifest, Mapping) else {}
    route = _require_manifest_matches_plan(plan, manifest)
    root = run_plan_path.expanduser().resolve().parent
    nav_status = teleop._ready_path(plan, "nav_runtime", root)
    explore_status = teleop._ready_path(plan, "explore_runtime", root)
    environment = dict(getattr(plan, "native_process_environment", {}) or {})
    environment["LINGTU_PRODUCT_SESSION_ID"] = product_session_id
    domain_id = int(args.domain_id)
    control_binary = Path((prepared.get("binaries") or {})["navigation_control"])
    request_id = f"mujoco-explore-{route}-start-{domain_id}"
    control = teleop._run_control(
        control_binary,
        (
            "explore",
            "start",
            product_session_id,
            f"mujoco_{route}_acceptance",
            "--request-id",
            request_id,
        ),
        domain_id=domain_id,
        env=environment,
        timeout_s=10.0,
    )

    timeline: list[dict[str, Any]] = []
    deadline = time.monotonic() + max(1.0, float(args.duration_s))
    while time.monotonic() < deadline:
        status = _load_json(explore_status)
        if status:
            timeline.append(status)
        time.sleep(0.2)

    stop = teleop._run_control(
        control_binary,
        (
            "explore",
            "stop",
            "mujoco_acceptance_complete",
            "--request-id",
            f"mujoco-explore-{route}-stop-{domain_id}",
        ),
        domain_id=domain_id,
        env=environment,
        timeout_s=10.0,
    )
    stop_zero = False
    zero_deadline = time.monotonic() + 3.0
    while time.monotonic() < zero_deadline:
        if _is_zero_output(_load_json(nav_status)):
            stop_zero = True
            break
        time.sleep(0.05)

    contract = dict(manifest.get("product_contract") or {})
    thresholds = manifest.get("thresholds")
    thresholds = thresholds if isinstance(thresholds, Mapping) else {}
    evidence = {
        "control": control,
        "stop": stop,
        "stop_zero": stop_zero,
        "timeline": timeline,
        "nav": _load_json(nav_status),
        "product_contract": contract,
        "min_global_path_points": int(thresholds.get("min_global_path_points") or 1),
    }
    evaluation = evaluate_case(evidence)
    return {
        "ok": evaluation["ok"],
        "mode": "attach_only",
        "run_plan": str(run_plan_path.expanduser().resolve()),
        "product_session_id": product_session_id,
        "request_id": request_id,
        "route": route,
        "control": control,
        "stop": stop,
        "timeline_samples": len(timeline),
        "evaluation": evaluation,
        "min_path_length_m": float(thresholds.get("min_motion_distance_m") or 0.05),
        "blockers": list(evaluation["blockers"]),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--run-plan", type=Path)
    parser.add_argument("--product-session-id")
    parser.add_argument("--artifact-dir", type=Path)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--domain-id", type=int, default=236)
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--strict", action="store_true")
    return parser


def run(args: argparse.Namespace) -> dict[str, Any]:
    prepared = prepare_runtime(args)
    blockers = list(prepared["blockers"])
    case: Mapping[str, Any] | None = None
    if prepared["ok"] and not args.preflight_only:
        if args.run_plan is None or not str(args.product_session_id or "").strip():
            blockers.append("active_product_identity_required")
        else:
            plan = validate_runner_plan(
                ROOT,
                args.run_plan,
                args.manifest,
                expected_products=("explore",),
            )
            case = run_attached(
                plan=plan,
                run_plan_path=args.run_plan,
                product_session_id=str(args.product_session_id),
                prepared=prepared,
                args=args,
            )
            blockers.extend(str(value) for value in case.get("blockers") or ())
    ok = not blockers and (bool(args.preflight_only) or case is not None)
    scope = dict(prepared["manifest"].get("acceptance_scope") or {})
    report = {
        "schema_version": REPORT_SCHEMA,
        "ok": ok,
        "strict": bool(args.strict),
        "preflight_only": bool(args.preflight_only),
        "acceptance_scope": scope,
        "preflight": prepared["details"],
        "blockers": blockers,
        "case": case,
    }
    report.update(
        classify_evidence(
            scope,
            run_plan_verified=case is not None,
            acceptance_evaluated=case is not None,
            ok=ok,
        )
    )
    output = Path(args.json_out or (Path(args.artifact_dir) / "report.json")).resolve()
    _write_json(output, report)
    report["report_path"] = str(output)
    return report


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.artifact_dir is None:
        args.artifact_dir = ROOT / "artifacts" / "mujoco_explore_native"
    report = run(args)
    print(
        json.dumps(
            {
                "ok": report["ok"],
                "blockers": report["blockers"],
                "report": report["report_path"],
            }
        )
    )
    return 1 if args.strict and not report["ok"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
