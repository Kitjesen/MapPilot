#!/usr/bin/env python3
"""Generate a no-motion blocked-route replanning artifact through Gateway preview."""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


SCHEMA_VERSION = "lingtu.blocked_route_replan_gate.v1"


class _FakeBlockedRouteNav:
    def __init__(self, *, planner: str) -> None:
        self.planner = planner
        self.phase = "baseline"
        self.calls: list[dict[str, Any]] = []

    def preview_plan(self, x: float, y: float, z: float) -> dict[str, Any]:
        ts = time.time()
        self.calls.append({"phase": self.phase, "x": float(x), "y": float(y), "z": float(z)})
        if self.phase == "blocked_candidate":
            path = [
                _point(0.0, 0.0, ts),
                _point(0.6, -0.55, ts),
                _point(1.35, -0.55, ts),
                _point(float(x), float(y), ts),
            ]
            reasons = ["blocked_route_replan", "synthetic_corridor_obstruction"]
            min_clearance = 0.4
        else:
            path = [
                _point(0.0, 0.0, ts),
                _point(float(x) / 2.0, float(y) / 2.0, ts),
                _point(float(x), float(y), ts),
            ]
            reasons = []
            min_clearance = 0.0
        return {
            "schema_version": 1,
            "ok": True,
            "feasible": True,
            "frame_id": "map",
            "start": _point(0.0, 0.0, ts),
            "goal": _point(float(x), float(y), ts),
            "path": path,
            "count": len(path),
            "distance_m": round(_path_length(_path_xy(path)), 4),
            "plan_ms": 2.5,
            "planner": self.planner,
            "selected_planner": self.planner,
            "plan_safety_policy": "blocked_route_replan_preflight",
            "path_safety": {
                "ok": True,
                "blocked_sample_count": 0,
                "min_clearance_m": min_clearance,
            },
            "fallback_reason": "",
            "rejected_plans": [],
            "source": "blocked_route_replan_preflight",
            "reasons": reasons,
            "error": None,
            "ts": ts,
            "phase": self.phase,
            "blocked_route_replanned": self.phase == "blocked_candidate",
        }


def _point(x: float, y: float, ts: float) -> dict[str, Any]:
    return {"x": float(x), "y": float(y), "z": 0.0, "frame_id": "map", "ts": ts}


def _xy(value: Any) -> tuple[float, float] | None:
    if isinstance(value, dict) and "x" in value and "y" in value:
        try:
            return float(value["x"]), float(value["y"])
        except Exception:
            return None
    return None


def _path_xy(path: list[Any]) -> list[tuple[float, float]]:
    return [point for item in path if (point := _xy(item)) is not None]


def _path_length(path: list[tuple[float, float]]) -> float:
    total = 0.0
    previous = path[0] if path else (0.0, 0.0)
    for point in path[1:]:
        total += math.hypot(point[0] - previous[0], point[1] - previous[1])
        previous = point
    return total


def _segment_samples(path: list[tuple[float, float]], samples_per_segment: int = 40) -> list[tuple[float, float]]:
    samples: list[tuple[float, float]] = []
    for start, end in zip(path, path[1:]):
        for index in range(samples_per_segment + 1):
            ratio = index / samples_per_segment
            samples.append(
                (
                    start[0] + (end[0] - start[0]) * ratio,
                    start[1] + (end[1] - start[1]) * ratio,
                )
            )
    return samples


def _inside_rect(point: tuple[float, float], block: dict[str, float]) -> bool:
    return (
        block["x_min"] <= point[0] <= block["x_max"]
        and block["y_min"] <= point[1] <= block["y_max"]
    )


def _rect_clearance(point: tuple[float, float], block: dict[str, float]) -> float:
    dx = max(block["x_min"] - point[0], 0.0, point[0] - block["x_max"])
    dy = max(block["y_min"] - point[1], 0.0, point[1] - block["y_max"])
    return math.hypot(dx, dy)


def _block_metrics(plan: dict[str, Any], block: dict[str, float]) -> dict[str, Any]:
    path = _path_xy(list(plan.get("path") or []))
    samples = _segment_samples(path)
    intersects = any(_inside_rect(sample, block) for sample in samples)
    min_clearance = min((_rect_clearance(sample, block) for sample in samples), default=0.0)
    max_lateral = max((abs(point[1]) for point in path), default=0.0)
    return {
        "path_count": len(path),
        "path_length_m": round(_path_length(path), 4),
        "intersects_block": bool(intersects),
        "min_block_clearance_m": round(min_clearance, 4),
        "max_lateral_offset_m": round(max_lateral, 4),
    }


def _endpoint(gateway: Any, path: str):
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def _mark_navigation_ready(gateway: Any, *, map_name: str) -> None:
    gateway._session_mode = "navigating"
    gateway._active_map = map_name
    gateway._icp_quality = 0.03
    odom = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "ts": time.time(), "frame_id": "map"}
    mission = {"state": "IDLE"}
    localization_status = {
        "state": "TRACKING",
        "confidence": 0.92,
        "degeneracy": "NONE",
        "odom_age_ms": 80.0,
        "localizer_health": "LOCKED",
    }
    lock = getattr(gateway, "_state_lock", None)
    if lock is not None:
        with lock:
            gateway._odom = odom
            gateway._mission = mission
            gateway._localization_status = localization_status
            gateway._safety = {"level": "ok", "stop": False}
    else:
        gateway._latest_odom = odom
        gateway._latest_mission = mission
        gateway._latest_safety = {"level": "ok", "stop": False}


def _active_cmd_source(status: dict[str, Any]) -> str:
    control = status.get("control") or {}
    value = control.get("active_cmd_source")
    if value is None:
        value = control.get("command_owner")
    source = str(value or "none").strip().lower()
    return "none" if source in {"", "unknown", "none", "null", "-"} else source


def _write_json(path: Path, payload: dict[str, Any]) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return str(path)


def _phase_summary(
    *,
    phase: str,
    status: dict[str, Any],
    plan: dict[str, Any],
    block: dict[str, float],
) -> dict[str, Any]:
    readiness = status.get("readiness") or {}
    metrics = _block_metrics(plan, block)
    return {
        "schema_version": 1,
        "phase": phase,
        "non_motion": True,
        "navigation_state": status.get("state"),
        "can_accept_goal": bool(readiness.get("can_accept_goal", status.get("can_accept_goal", False))),
        "active_cmd_source_before": _active_cmd_source(status),
        "feasible": bool(plan.get("feasible")),
        "count": int(plan.get("count") or len(plan.get("path") or [])),
        "selected_planner": plan.get("selected_planner") or plan.get("planner"),
        "blocked_route_replanned": plan.get("blocked_route_replanned") is True,
        "reasons": list(plan.get("reasons") or []),
        **metrics,
    }


def _run_preview(
    *,
    gateway: Any,
    nav: _FakeBlockedRouteNav,
    phase: str,
    x: float,
    y: float,
    client_id: str,
    artifact_dir: Path,
    block: dict[str, float],
) -> tuple[dict[str, Any], dict[str, str]]:
    from gateway.schemas import PlanPreviewRequest
    from gateway.services.runtime_status import build_navigation_status

    nav.phase = phase
    status_before = build_navigation_status(gateway)
    preview_endpoint = _endpoint(gateway, "/api/v1/navigation/plan")
    plan = asyncio.run(
        preview_endpoint(
            PlanPreviewRequest(
                x=float(x),
                y=float(y),
                z=0.0,
                frame_id="map",
                client_id=client_id,
            )
        )
    )
    status_after = build_navigation_status(gateway)
    phase_dir = artifact_dir / phase
    artifacts = {
        "navigation_before": _write_json(phase_dir / "navigation_before.json", status_before),
        "plan": _write_json(phase_dir / "plan.json", plan),
        "navigation_after": _write_json(phase_dir / "navigation_after.json", status_after),
        "plan_summary": "",
    }
    summary = _phase_summary(phase=phase, status=status_before, plan=plan, block=block)
    artifacts["plan_summary"] = _write_json(phase_dir / "plan_summary.json", summary)
    return summary, artifacts


def run_gate(
    *,
    map_name: str,
    x: float,
    y: float,
    planner: str,
    json_out: Path,
    client_id: str,
    min_block_clearance_m: float = 0.25,
    min_lateral_change_m: float = 0.30,
) -> dict[str, Any]:
    artifact_dir = json_out.parent
    block = {"x_min": 0.85, "x_max": 1.15, "y_min": -0.15, "y_max": 0.15}
    try:
        from gateway.gateway_module import GatewayModule
        from gateway.services.runtime_status import build_navigation_status
    except Exception as exc:
        return {
            "schema_version": SCHEMA_VERSION,
            "ok": False,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "mode": "blocked_route_replan_non_motion",
            "errors": [f"gateway imports unavailable: {exc}"],
        }

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakeBlockedRouteNav(planner=planner)
    gateway.on_system_modules({"NavigationModule": nav})
    _mark_navigation_ready(gateway, map_name=map_name)

    sent_goals: list[Any] = []
    sent_cmd_vel: list[Any] = []
    sent_stops: list[Any] = []
    gateway.goal_pose._add_callback(sent_goals.append)
    gateway.cmd_vel._add_callback(sent_cmd_vel.append)
    gateway.stop_cmd._add_callback(sent_stops.append)

    baseline, baseline_artifacts = _run_preview(
        gateway=gateway,
        nav=nav,
        phase="baseline",
        x=x,
        y=y,
        client_id=client_id,
        artifact_dir=artifact_dir,
        block=block,
    )
    candidate, candidate_artifacts = _run_preview(
        gateway=gateway,
        nav=nav,
        phase="blocked_candidate",
        x=x,
        y=y,
        client_id=client_id,
        artifact_dir=artifact_dir,
        block=block,
    )
    after_rollback = build_navigation_status(gateway)
    rollback_path = _write_json(artifact_dir / "after_rollback" / "navigation.json", after_rollback)

    published = {
        "goal_pose": len(sent_goals),
        "cmd_vel": len(sent_cmd_vel),
        "stop_cmd": len(sent_stops),
    }
    no_motion = published == {"goal_pose": 0, "cmd_vel": 0, "stop_cmd": 0}
    baseline_intersects = baseline["intersects_block"] is True
    candidate_avoids = (
        candidate["intersects_block"] is False
        and float(candidate["min_block_clearance_m"]) >= min_block_clearance_m
    )
    candidate_changed = (
        float(candidate["max_lateral_offset_m"]) - float(baseline["max_lateral_offset_m"])
        >= min_lateral_change_m
    )
    phases_ok = all(
        phase.get("non_motion") is True
        and phase.get("can_accept_goal") is True
        and phase.get("active_cmd_source_before") == "none"
        and phase.get("feasible") is True
        and int(phase.get("count") or 0) >= 2
        and bool(phase.get("selected_planner"))
        for phase in (baseline, candidate)
    )
    errors: list[str] = []
    if not no_motion:
        errors.append("preview published motion commands")
    if not phases_ok:
        errors.append("baseline or candidate phase did not satisfy preview readiness")
    if not baseline_intersects:
        errors.append("baseline path does not intersect the synthetic block")
    if not candidate_avoids:
        errors.append("blocked candidate path does not avoid the synthetic block")
    if not candidate_changed:
        errors.append("candidate route did not change enough after blocking")
    if candidate.get("blocked_route_replanned") is not True:
        errors.append("candidate did not mark blocked_route_replanned")
    if "blocked_route_replan" not in candidate.get("reasons", []):
        errors.append("candidate missing blocked_route_replan reason")

    ok = not errors
    return {
        "schema_version": SCHEMA_VERSION,
        "ok": ok,
        "mode": "blocked_route_replan_non_motion",
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "gateway_used": True,
        "driver_used": False,
        "map": map_name,
        "goal": {"x": float(x), "y": float(y), "yaw": 0.0},
        "synthetic_block": block,
        "thresholds": {
            "min_block_clearance_m": float(min_block_clearance_m),
            "min_lateral_change_m": float(min_lateral_change_m),
        },
        "baseline_path_intersects_block": baseline_intersects,
        "candidate_path_avoids_block": candidate_avoids,
        "candidate_route_changed": candidate_changed,
        "blocked_route_replanned": candidate.get("blocked_route_replanned") is True,
        "phases": {"baseline": baseline, "blocked_candidate": candidate},
        "published": published,
        "nav_preview_calls": nav.calls,
        "artifacts": {
            "baseline": str(artifact_dir / "baseline"),
            "blocked_candidate": str(artifact_dir / "blocked_candidate"),
            "after_rollback": str(artifact_dir / "after_rollback"),
            "baseline_files": baseline_artifacts,
            "blocked_candidate_files": candidate_artifacts,
            "after_rollback_navigation": rollback_path,
        },
        "errors": errors,
        "ts": time.time(),
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", dest="map_name", default="server_sim_demo")
    parser.add_argument("--goal-x", type=float, default=2.0)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--planner", default="pct")
    parser.add_argument("--client-id", default="server-sim-blocked-route")
    parser.add_argument("--min-block-clearance-m", type=float, default=0.25)
    parser.add_argument("--min-lateral-change-m", type=float, default=0.30)
    parser.add_argument(
        "--json-out",
        type=Path,
        default=ROOT / "artifacts/server_sim_closure/blocked_route_replan/report.json",
    )
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    report = run_gate(
        map_name=args.map_name,
        x=args.goal_x,
        y=args.goal_y,
        planner=args.planner,
        json_out=args.json_out,
        client_id=args.client_id,
        min_block_clearance_m=args.min_block_clearance_m,
        min_lateral_change_m=args.min_lateral_change_m,
    )
    text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    print(text)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    return 0 if report.get("ok") is True or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())
