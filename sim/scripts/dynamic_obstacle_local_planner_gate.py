#!/usr/bin/env python3
"""Verify local planner replanning against changing obstacle inputs."""

from __future__ import annotations

import argparse
import json
import platform
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

np: Any = None
LocalPlanner: Any = None
Pose: Any = None
PoseStamped: Any = None
Quaternion: Any = None
Vector3: Any = None
Odometry: Any = None
NavPath: Any = None
PointCloud2: Any = None


@dataclass(frozen=True)
class PhaseSpec:
    name: str
    obstacle_center_y: float | None
    expected: str


PHASES = (
    PhaseSpec("clear_initial", None, "straight"),
    PhaseSpec("obstacle_left", 0.35, "right_detour"),
    PhaseSpec("obstacle_right", -0.35, "left_detour"),
    PhaseSpec("obstacle_center", 0.0, "detour"),
    PhaseSpec("clear_recovered", None, "straight"),
)


def _load_runtime() -> None:
    global LocalPlanner
    global NavPath
    global Odometry
    global PointCloud2
    global Pose
    global PoseStamped
    global Quaternion
    global Vector3
    global np

    if np is None:
        import numpy as _np

        np = _np
    if LocalPlanner is None:
        from nav.services.plan.local_planner.service import (
            LocalPlanner as _LocalPlanner,
        )

        LocalPlanner = _LocalPlanner
    if Pose is None:
        from runtime.msgs.geometry import (
            Pose as _Pose,
            PoseStamped as _PoseStamped,
            Quaternion as _Quaternion,
            Vector3 as _Vector3,
        )
        from runtime.msgs.nav import Odometry as _Odometry
        from runtime.msgs.nav import Path as _NavPath
        from runtime.msgs.sensor import PointCloud2 as _PointCloud2

        Pose = _Pose
        PoseStamped = _PoseStamped
        Quaternion = _Quaternion
        Vector3 = _Vector3
        Odometry = _Odometry
        NavPath = _NavPath
        PointCloud2 = _PointCloud2


def _base_report(
    *,
    backend: str,
    backend_actual: str = "",
    native_backend_used: bool = False,
    local_planner_evidence: dict[str, Any] | None = None,
    execution_mode: str = "runtime_gate",
    environment: dict[str, Any] | None = None,
    errors: list[str] | None = None,
) -> dict[str, Any]:
    return {
        "schema_version": "lingtu.dynamic_obstacle_local_planner.v1",
        "ok": False,
        "execution_mode": execution_mode,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "backend_requested": backend,
        "backend_actual": backend_actual,
        "native_backend_used": native_backend_used,
        "algorithm_backends": {
            "local_planner": local_planner_evidence
            or {
                "requested": backend,
                "configured_backend": backend,
                "backend_actual": backend_actual,
                "degraded": False,
                "degraded_reason": "",
                "native_backend_used": native_backend_used,
                "exercised_by": "dynamic_obstacle",
            },
            "path_follower": {
                "status": "not_exercised",
                "exercised_by": "not_exercised",
            },
        },
        "dynamic_replan_verified": False,
        "obstacle_response_verified": False,
        "clear_path_recovery_verified": False,
        "min_clearance_m": None,
        "phases": [],
        "frames": {
            "cmd_vel": "not_published",
        },
        "environment": environment or {},
        "errors": errors or [],
    }


def _make_obstacle(center_y: float, *, half_width: float, spacing_x: int, spacing_y: int) -> np.ndarray:
    points: list[list[float]] = []
    for x in np.linspace(0.8, 2.2, spacing_x):
        for y in np.linspace(center_y - half_width, center_y + half_width, spacing_y):
            points.append([float(x), float(y), 0.0, 200.0])
    return np.asarray(points, dtype=np.float32)


def _path_xy(path: NavPath) -> np.ndarray:
    return np.asarray(
        [[pose.pose.position.x, pose.pose.position.y] for pose in path.poses],
        dtype=float,
    )


def _min_clearance(path_xy: np.ndarray, obstacles: np.ndarray) -> float | None:
    if path_xy.size == 0 or obstacles.size == 0:
        return None
    dists = np.linalg.norm(path_xy[:, None, :] - obstacles[None, :, :2].astype(float), axis=2)
    return float(np.min(dists))


def _phase_report(
    *,
    name: str,
    path: NavPath,
    obstacles: np.ndarray,
    expected: str,
) -> dict[str, Any]:
    xy = _path_xy(path)
    mean_y = float(np.mean(xy[:, 1])) if xy.size else 0.0
    max_abs_y = float(np.max(np.abs(xy[:, 1]))) if xy.size else 0.0
    clearance = _min_clearance(xy, obstacles)
    if mean_y > 0.03:
        side = "left"
    elif mean_y < -0.03:
        side = "right"
    else:
        side = "straight"
    return {
        "name": name,
        "expected": expected,
        "path_count": int(len(path.poses)),
        "path_frame_id": path.frame_id,
        "mean_y_m": round(mean_y, 4),
        "max_abs_y_m": round(max_abs_y, 4),
        "avoidance_side": side,
        "obstacle_count": int(len(obstacles)),
        "min_obstacle_clearance_m": None if clearance is None else round(clearance, 4),
    }


def _run_local_planner_phase(
    module: LocalPlanner,
    *,
    backend: str,
    obstacles: np.ndarray,
    timestamp: float,
) -> NavPath:
    published: list[NavPath] = []
    module.local_path._add_callback(published.append)
    module._on_added_obstacles(PointCloud2(points=obstacles, frame_id="map", ts=timestamp))
    if backend == "nanobind":
        module._run_nanobind(timestamp)
    elif backend == "cmu_py":
        module._last_cmu_py_time = 0.0
        module._run_cmu_py()
    else:
        raise ValueError(f"unsupported backend for dynamic obstacle gate: {backend}")
    if not published:
        return NavPath(poses=[], frame_id="map", ts=timestamp)
    return published[-1]


def _backend_evidence(module: LocalPlanner, *, requested: str, exercised_by: str) -> dict[str, Any]:
    status = getattr(module, "_backend_status", None)
    actual = str(getattr(module, "_backend", "") or requested)
    configured = str(getattr(status, "configured", "") or requested)
    degraded = bool(getattr(status, "degraded", False) or actual != requested)
    degraded_reason = str(
        getattr(status, "degraded_reason", "") or getattr(status, "reason", "") or ""
    )
    if degraded and not degraded_reason and actual != requested:
        degraded_reason = f"effective backend changed from {requested} to {actual}"
    return {
        "requested": requested,
        "configured_backend": configured,
        "backend_actual": actual,
        "degraded": degraded,
        "degraded_reason": degraded_reason,
        "native_backend_used": actual == "nanobind",
        "exercised_by": exercised_by,
    }


def run_gate(
    *,
    backend: str = "nanobind",
    obstacle_half_width: float = 0.12,
    min_clearance_m: float = 0.25,
    min_path_points: int = 20,
    min_lateral_response_m: float = 0.08,
    platform_system: str | None = None,
    allow_unstable_windows_numpy: bool = False,
) -> dict[str, Any]:
    errors: list[str] = []
    resolved_platform = platform_system or platform.system()
    runtime_injected = LocalPlanner is not None
    if (
        resolved_platform.lower() == "windows"
        and not allow_unstable_windows_numpy
        and not runtime_injected
    ):
        blocker = (
            "Windows/MINGW NumPy local-planner runtime is not accepted; run this "
            "gate on Linux or pass --allow-unstable-windows-numpy for manual diagnosis"
        )
        return _base_report(
            backend=backend,
            execution_mode="host_guard",
            environment={
                "platform_system": resolved_platform,
                "accepted_host": False,
                "accepted_platforms": ["Linux"],
                "blocked_reason": "windows_mingw_numpy_not_accepted",
                "blockers": [blocker],
                "manual_diagnosis_flag": "--allow-unstable-windows-numpy",
                "claim_boundary": "environment_blocked_no_algorithm_claim",
            },
            errors=[blocker],
        )

    try:
        _load_runtime()
    except Exception as exc:
        blocker = f"dynamic obstacle gate runtime unavailable: {type(exc).__name__}: {exc}"
        return _base_report(
            backend=backend,
            execution_mode="runtime_unavailable",
            environment={
                "platform_system": resolved_platform,
                "accepted_host": resolved_platform.lower() != "windows"
                or allow_unstable_windows_numpy
                or runtime_injected,
                "blocked_reason": "runtime_import_failed",
                "blockers": [blocker],
                "claim_boundary": "environment_blocked_no_algorithm_claim",
            },
            errors=[blocker],
        )

    module = LocalPlanner(backend=backend)
    started = False
    backend_evidence: dict[str, Any] | None = None
    try:
        module.setup()
        started = True
        backend_evidence = _backend_evidence(
            module,
            requested=backend,
            exercised_by="dynamic_obstacle",
        )
        backend_actual = str(backend_evidence["backend_actual"])
        module._on_odom(
            Odometry(
                pose=Pose(
                    position=Vector3(0.0, 0.0, 0.0),
                    orientation=Quaternion.from_yaw(0.0),
                ),
                frame_id="map",
                child_frame_id="base_link",
                ts=1.0,
            )
        )
        module._on_waypoint(
            PoseStamped(
                pose=Pose(
                    position=Vector3(4.0, 0.0, 0.0),
                    orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                ),
                frame_id="map",
                ts=1.0,
            )
        )

        phase_reports: list[dict[str, Any]] = []
        for idx, phase in enumerate(PHASES):
            ts = 2.0 + idx
            obstacles = (
                np.zeros((0, 4), dtype=np.float32)
                if phase.obstacle_center_y is None
                else _make_obstacle(
                    phase.obstacle_center_y,
                    half_width=obstacle_half_width,
                    spacing_x=12,
                    spacing_y=5,
                )
            )
            path = _run_local_planner_phase(
                module,
                backend=backend_actual,
                obstacles=obstacles,
                timestamp=ts,
            )
            phase_reports.append(
                _phase_report(
                    name=phase.name,
                    path=path,
                    obstacles=obstacles,
                    expected=phase.expected,
                )
            )

        by_name = {item["name"]: item for item in phase_reports}
        clear_initial = by_name["clear_initial"]
        clear_recovered = by_name["clear_recovered"]
        obstacle_left = by_name["obstacle_left"]
        obstacle_right = by_name["obstacle_right"]
        obstacle_center = by_name["obstacle_center"]

        for item in phase_reports:
            if int(item["path_count"]) < min_path_points:
                errors.append(f"{item['name']} path_count < {min_path_points}")
            if item["path_frame_id"] != "map":
                errors.append(f"{item['name']} local path frame is not map")

        for item in (clear_initial, clear_recovered):
            if abs(float(item["mean_y_m"])) > 0.03 or float(item["max_abs_y_m"]) > 0.05:
                errors.append(f"{item['name']} did not recover a straight path")

        if float(obstacle_left["mean_y_m"]) > -min_lateral_response_m:
            errors.append("left obstacle did not push path to the right")
        if float(obstacle_right["mean_y_m"]) < min_lateral_response_m:
            errors.append("right obstacle did not push path to the left")
        if float(obstacle_center["max_abs_y_m"]) < 0.30:
            errors.append("center obstacle did not produce a visible detour")

        obstacle_clearances = [
            float(item["min_obstacle_clearance_m"])
            for item in (obstacle_left, obstacle_right, obstacle_center)
            if item["min_obstacle_clearance_m"] is not None
        ]
        if not obstacle_clearances:
            errors.append("obstacle clearances missing")
        elif min(obstacle_clearances) < min_clearance_m:
            errors.append(f"min obstacle clearance < {min_clearance_m:.2f}m")

        dynamic_replan_verified = (
            obstacle_left["avoidance_side"] == "right"
            and obstacle_right["avoidance_side"] == "left"
            and clear_recovered["avoidance_side"] == "straight"
        )
        if not dynamic_replan_verified:
            errors.append("dynamic replan direction sequence was not verified")

        return {
            "schema_version": "lingtu.dynamic_obstacle_local_planner.v1",
            "ok": not errors,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "backend_requested": backend,
            "backend_actual": backend_actual,
            "native_backend_used": backend_actual == "nanobind",
            "execution_mode": "runtime_gate",
            "algorithm_backends": {
                "local_planner": backend_evidence,
                "path_follower": {
                    "status": "not_exercised",
                    "exercised_by": "not_exercised",
                },
            },
            "dynamic_replan_verified": dynamic_replan_verified,
            "obstacle_response_verified": not any("obstacle did not" in item for item in errors),
            "clear_path_recovery_verified": not any("straight path" in item for item in errors),
            "min_clearance_m": None if not obstacle_clearances else round(min(obstacle_clearances), 4),
            "phases": phase_reports,
            "frames": {
                "odometry": "map",
                "waypoint": "map",
                "added_obstacles": "map",
                "local_path": "map",
                "cmd_vel": "not_published",
            },
            "environment": {
                "platform_system": resolved_platform,
                "accepted_host": True,
                "blocked_reason": "",
                "blockers": [],
                "claim_boundary": "dynamic_obstacle_algorithm_gate",
            },
            "errors": errors,
        }
    except Exception as exc:
        return _base_report(
            backend=backend,
            backend_actual=(
                str(backend_evidence["backend_actual"])
                if backend_evidence is not None
                else (backend if started else "")
            ),
            native_backend_used=(
                bool(backend_evidence["native_backend_used"])
                if backend_evidence is not None
                else False
            ),
            local_planner_evidence=backend_evidence,
            execution_mode="runtime_error",
            environment={
                "platform_system": resolved_platform,
                "accepted_host": True,
                "blocked_reason": "runtime_error",
                "blockers": [str(exc)],
                "claim_boundary": "runtime_error_no_algorithm_claim",
            },
            errors=[str(exc)],
        )
    finally:
        if started:
            module.stop()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("nanobind", "cmu_py"), default="nanobind")
    parser.add_argument("--obstacle-half-width", type=float, default=0.12)
    parser.add_argument("--min-clearance-m", type=float, default=0.25)
    parser.add_argument("--min-path-points", type=int, default=20)
    parser.add_argument("--min-lateral-response-m", type=float, default=0.08)
    parser.add_argument(
        "--allow-unstable-windows-numpy",
        action="store_true",
        help="Manual diagnosis only: bypass the Windows/MINGW NumPy guard.",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=ROOT / "artifacts/server_sim_closure/dynamic_obstacle_local_planner/report.json",
    )
    args = parser.parse_args()

    report = run_gate(
        backend=args.backend,
        obstacle_half_width=args.obstacle_half_width,
        min_clearance_m=args.min_clearance_m,
        min_path_points=args.min_path_points,
        min_lateral_response_m=args.min_lateral_response_m,
        allow_unstable_windows_numpy=args.allow_unstable_windows_numpy,
    )
    text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    print(text)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    return 0 if report.get("ok") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
