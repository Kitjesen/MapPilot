#!/usr/bin/env python3
"""Generate a server-side, no-motion route preflight artifact.

This gate exercises Gateway's navigation plan-preview route in process. It does
not start a server, attach a driver, publish a goal, or publish cmd_vel.
"""

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


class _FakeRoutecheckNav:
    def __init__(self, *, planner: str) -> None:
        self.planner = planner
        self.phase = "baseline"
        self.calls: list[dict[str, Any]] = []

    def preview_plan(self, x: float, y: float, z: float) -> dict[str, Any]:
        self.calls.append(
            {
                "phase": self.phase,
                "x": float(x),
                "y": float(y),
                "z": float(z),
            }
        )
        ts = time.time()
        midpoint = {
            "x": float(x) / 2.0,
            "y": float(y) / 2.0,
            "z": float(z),
            "frame_id": "map",
        }
        return {
            "schema_version": 1,
            "ok": True,
            "feasible": True,
            "frame_id": "map",
            "start": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "frame_id": "map",
                "ts": ts,
            },
            "goal": {
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "frame_id": "map",
                "ts": ts,
            },
            "path": [
                {"x": 0.0, "y": 0.0, "z": 0.0, "frame_id": "map", "ts": ts},
                midpoint,
                {"x": float(x), "y": float(y), "z": float(z), "frame_id": "map", "ts": ts},
            ],
            "count": 3,
            "distance_m": math.hypot(float(x), float(y)),
            "plan_ms": 2.5,
            "planner": self.planner,
            "selected_planner": self.planner,
            "plan_safety_policy": "fallback_astar",
            "path_safety": {
                "ok": True,
                "blocked_sample_count": 0,
                "min_clearance_m": 0.8,
            },
            "fallback_reason": "",
            "rejected_plans": [],
            "source": "server_sim_routecheck_preflight",
            "reasons": [],
            "error": None,
            "ts": ts,
            "phase": self.phase,
        }


def _endpoint(gateway: Any, path: str):
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def _mark_navigation_ready(gateway: Any, *, map_name: str) -> None:
    gateway._session_mode = "navigating"
    gateway._active_map = map_name
    gateway._icp_quality = 0.03
    odom = {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "yaw": 0.0,
        "ts": time.time(),
        "frame_id": "map",
    }
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
    if source in {"", "unknown", "none", "null", "-"}:
        return "none"
    return source


def _phase_summary(*, phase: str, status: dict[str, Any], plan: dict[str, Any]) -> dict[str, Any]:
    readiness = status.get("readiness") or {}
    path_safety = plan.get("path_safety") if isinstance(plan.get("path_safety"), dict) else {}
    rejected = plan.get("rejected_plans") if isinstance(plan.get("rejected_plans"), list) else []
    return {
        "schema_version": 1,
        "phase": phase,
        "non_motion": True,
        "navigation_state": status.get("state"),
        "can_accept_goal": bool(readiness.get("can_accept_goal", status.get("can_accept_goal", False))),
        "active_cmd_source_before": _active_cmd_source(status),
        "feasible": bool(plan.get("feasible")),
        "count": int(plan.get("count") or len(plan.get("path") or [])),
        "planner": plan.get("planner"),
        "selected_planner": plan.get("selected_planner") or plan.get("planner"),
        "plan_safety_policy": plan.get("plan_safety_policy"),
        "path_safety_ok": path_safety.get("ok") is True,
        "fallback_reason": str(plan.get("fallback_reason") or ""),
        "rejected_plan_count": len(rejected),
        "reasons": list(plan.get("reasons") or []),
    }


def _write_json(path: Path, payload: dict[str, Any]) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return str(path)


def _run_preview(
    *,
    gateway: Any,
    nav: _FakeRoutecheckNav,
    phase: str,
    x: float,
    y: float,
    z: float,
    client_id: str,
    artifact_dir: Path,
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
                z=float(z),
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
    summary = _phase_summary(phase=phase, status=status_before, plan=plan)
    artifacts["plan_summary"] = _write_json(phase_dir / "plan_summary.json", summary)
    return summary, artifacts


def run_gate(
    *,
    map_name: str,
    x: float,
    y: float,
    yaw: float,
    planner: str,
    json_out: Path,
    client_id: str,
) -> dict[str, Any]:
    artifact_dir = json_out.parent
    try:
        from gateway.gateway_module import GatewayModule
        from gateway.services.runtime_status import build_navigation_status
    except Exception as exc:
        return {
            "schema_version": 1,
            "mode": "routecheck_non_motion",
            "outcome": "fail",
            "exit_status": 2,
            "non_motion": True,
            "map": map_name,
            "goal": {"x": float(x), "y": float(y), "yaw": float(yaw)},
            "phases": {},
            "artifacts": {},
            "error": f"gateway imports unavailable: {exc}",
        }

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakeRoutecheckNav(planner=planner)
    gateway.on_system_modules({"nav.mission": nav})
    _mark_navigation_ready(gateway, map_name=map_name)

    sent_goals = []
    sent_cmd_vel = []
    sent_stops = []
    gateway.goal_pose._add_callback(sent_goals.append)
    gateway.cmd_vel._add_callback(sent_cmd_vel.append)
    gateway.stop_cmd._add_callback(sent_stops.append)

    baseline, baseline_artifacts = _run_preview(
        gateway=gateway,
        nav=nav,
        phase="baseline",
        x=x,
        y=y,
        z=0.0,
        client_id=client_id,
        artifact_dir=artifact_dir,
    )
    candidate, candidate_artifacts = _run_preview(
        gateway=gateway,
        nav=nav,
        phase="candidate",
        x=x,
        y=y,
        z=0.0,
        client_id=client_id,
        artifact_dir=artifact_dir,
    )
    after_rollback = build_navigation_status(gateway)
    rollback_path = _write_json(artifact_dir / "after_rollback" / "navigation.json", after_rollback)

    published = {
        "goal_pose": len(sent_goals),
        "cmd_vel": len(sent_cmd_vel),
        "stop_cmd": len(sent_stops),
    }
    no_motion = published == {"goal_pose": 0, "cmd_vel": 0, "stop_cmd": 0}
    phase_values = {"baseline": baseline, "candidate": candidate}
    phases_ok = all(
        bool(item.get("non_motion"))
        and bool(item.get("can_accept_goal"))
        and str(item.get("active_cmd_source_before") or "none").lower() in {"none", "null", "-", ""}
        and bool(item.get("feasible"))
        and int(item.get("count") or 0) >= 2
        and bool(item.get("selected_planner") or item.get("planner"))
        and item.get("path_safety_ok") is True
        for item in phase_values.values()
    )
    ok = bool(no_motion and phases_ok)
    return {
        "schema_version": 1,
        "mode": "routecheck_non_motion",
        "outcome": "pass" if ok else "fail",
        "exit_status": 0 if ok else 1,
        "non_motion": True,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "gateway_used": True,
        "driver_used": False,
        "map": map_name,
        "goal": {"x": float(x), "y": float(y), "yaw": float(yaw)},
        "phases": phase_values,
        "published": published,
        "nav_preview_calls": nav.calls,
        "artifacts": {
            "baseline": str(artifact_dir / "baseline"),
            "candidate": str(artifact_dir / "candidate"),
            "after_rollback": str(artifact_dir / "after_rollback"),
            "baseline_files": baseline_artifacts,
            "candidate_files": candidate_artifacts,
            "after_rollback_navigation": rollback_path,
        },
        "ts": time.time(),
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", dest="map_name", default="server_sim_demo")
    parser.add_argument("--goal-x", type=float, default=1.0)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--planner", default="pct")
    parser.add_argument("--client-id", default="server-sim-routecheck")
    parser.add_argument("--json-out", type=Path, default=ROOT / "artifacts/server_sim_closure/routecheck/summary.json")
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    report = run_gate(
        map_name=args.map_name,
        x=args.goal_x,
        y=args.goal_y,
        yaw=args.goal_yaw,
        planner=args.planner,
        json_out=args.json_out,
        client_id=args.client_id,
    )
    text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    print(text)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    return 0 if report.get("exit_status") == 0 or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())
