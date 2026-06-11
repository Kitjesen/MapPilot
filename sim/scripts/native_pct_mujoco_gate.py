#!/usr/bin/env python3
"""Validate native PCT through ROS2 native local planning into MuJoCo.

This gate is intentionally simulation-only. It launches the production ROS2
`localPlanner` and `pathFollower` executables in an isolated ROS domain, feeds
them a native PCT route from a previous multifloor validation report, applies
the resulting `/cmd_vel` to a kinematic MuJoCo robot, and writes a JSON proof.

It never starts LingTu Gateway, robot drivers, systemd services, or hardware
bridges. Before launching `pathFollower`, it checks that the selected ROS
domain has no existing `/cmd_vel` or `/nav/cmd_vel` subscribers, so an already
running robot driver cannot accidentally consume the simulated command stream.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import signal
import shutil
import subprocess
import sys
import time
import traceback
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from core.msgs.numpy_compat import np, numpy_import_is_safe

DEFAULT_MID360_PATTERN = ROOT / "sim/assets/livox/mid360.npy"
DEFAULT_MID360_SAMPLES_PER_FRAME = 24000


@dataclass(frozen=True)
class PctRoute:
    route: str
    source_report: Path
    scene_xml: Path
    start: list[float]
    goal: list[float]
    path: list[list[float]]
    plan: dict[str, Any]
    case: dict[str, Any]


@dataclass
class _VelocityCommand:
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class _Vector2:
    x: float
    y: float

    def __iter__(self):
        yield self.x
        yield self.y

    def __len__(self) -> int:
        return 2

    def __getitem__(self, index: int | slice) -> Any:
        values = [self.x, self.y]
        return values[index]

    def __add__(self, other: Any) -> "_Vector2":
        other_vec = _as_vec2(other)
        return _Vector2(self.x + other_vec.x, self.y + other_vec.y)

    def __sub__(self, other: Any) -> "_Vector2":
        other_vec = _as_vec2(other)
        return _Vector2(self.x - other_vec.x, self.y - other_vec.y)

    def __mul__(self, scalar: float) -> "_Vector2":
        return _Vector2(self.x * float(scalar), self.y * float(scalar))

    __rmul__ = __mul__

    def tolist(self) -> list[float]:
        return [self.x, self.y]

    def astype(self, _dtype: Any) -> "_Vector2":
        return self


class _ArrayColumn:
    def __init__(self, values: list[float]) -> None:
        self._values = values

    def tolist(self) -> list[float]:
        return list(self._values)


class _Array2D:
    def __init__(self, rows: list[list[float]]) -> None:
        self._rows = rows

    @property
    def shape(self) -> tuple[int, int]:
        return (len(self._rows), len(self._rows[0]) if self._rows else 0)

    def __len__(self) -> int:
        return len(self._rows)

    def __getitem__(self, key: Any) -> Any:
        if not isinstance(key, tuple):
            return self._rows[key]
        row_key, col_key = key
        rows = self._rows[row_key]
        if rows and not isinstance(rows[0], list):
            rows = [rows]
        if isinstance(col_key, int):
            return _ArrayColumn([float(row[col_key]) for row in rows])
        return _Array2D([row[col_key] for row in rows])

    def tolist(self) -> list[list[float]]:
        return [list(row) for row in self._rows]

    def copy(self) -> "_Array2D":
        return _Array2D(self.tolist())


def _as_vec2(value: Any) -> _Vector2:
    if isinstance(value, _Vector2):
        return value
    return _Vector2(float(value[0]), float(value[1]))


def _clip(value: float, low: float, high: float) -> float:
    return max(float(low), min(float(high), float(value)))


def _xy_points(points: Any) -> list[_Vector2]:
    return [_Vector2(float(point[0]), float(point[1])) for point in points]


def _percentile(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * float(q) / 100.0
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return ordered[lower]
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _tail(path: Path, limit: int = 1200) -> str:
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8", errors="replace")[-limit:]


def _start_native_node(
    cmd: list[str],
    *,
    env: dict[str, str],
    log_fh: Any,
) -> subprocess.Popen[bytes]:
    kwargs: dict[str, Any] = {
        "cwd": str(ROOT),
        "env": env,
        "stdout": log_fh,
        "stderr": subprocess.STDOUT,
    }
    if os.name == "posix":
        kwargs["start_new_session"] = True
    elif os.name == "nt":
        kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
    return subprocess.Popen(cmd, **kwargs)


def _stop_native_node(proc: subprocess.Popen[bytes], *, timeout_s: float = 2.0) -> None:
    if proc.poll() is not None:
        return
    try:
        if os.name == "posix":
            os.killpg(proc.pid, signal.SIGTERM)
        else:
            proc.terminate()
    except ProcessLookupError:
        return
    except Exception:
        proc.terminate()
    try:
        proc.wait(timeout=timeout_s)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        if os.name == "posix":
            os.killpg(proc.pid, signal.SIGKILL)
        else:
            proc.kill()
    except ProcessLookupError:
        return
    except Exception:
        proc.kill()
    try:
        proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        pass


def _as_xyz(value: Any, *, field: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        raise ValueError(f"{field} must contain at least x/y")
    z = value[2] if len(value) > 2 else 0.0
    return [float(value[0]), float(value[1]), float(z)]


def _select_case(report: dict[str, Any], route: str) -> dict[str, Any]:
    cases = report.get("cases")
    if isinstance(cases, list):
        for case in cases:
            if isinstance(case, dict) and case.get("route") == route:
                return case
        raise ValueError(f"route {route!r} not found in source report cases")
    if report.get("route") == route:
        return report
    raise ValueError(f"source report is not for route {route!r}")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _selection_value(case: dict[str, Any], key: str, default: Any = "") -> Any:
    selection = case.get("selection")
    if isinstance(selection, dict) and key in selection:
        return selection.get(key)
    return case.get(key, default)


def _planner_contract(route: "PctRoute", planner: str) -> dict[str, Any]:
    assets = route.case.get("assets") or {}
    selection = route.case.get("selection") if isinstance(route.case.get("selection"), dict) else {}
    tomogram_raw = str(assets.get("tomogram") or "")
    tomogram = Path(tomogram_raw) if tomogram_raw else None
    safety = route.plan.get("path_safety")
    if safety is None and str(_selection_value(route.case, "primary_planner", "")).lower() == planner:
        safety = route.case.get("path_safety")
    if not isinstance(safety, dict):
        safety = {}
    return {
        "source_report": str(route.source_report),
        "route": route.route,
        "source_validation_level": str((json.loads(route.source_report.read_text(encoding="utf-8")) or {}).get("validation_level", "")),
        "tomogram": tomogram_raw,
        "tomogram_exists": bool(tomogram is not None and tomogram.exists()),
        "tomogram_sha256": _sha256(tomogram) if tomogram is not None and tomogram.exists() else "",
        "primary_planner": str(_selection_value(route.case, "primary_planner", route.plan.get("planner", ""))).lower(),
        "selected_planner": str(_selection_value(route.case, "selected_planner", route.plan.get("planner", ""))).lower(),
        "fallback_used": bool(_selection_value(route.case, "fallback_used", False)),
        "selection_policy": str(selection.get("policy") or ""),
        "selected_route_ok": bool(selection.get("selected_route_ok", route.plan.get("route_ok", True))),
        "path_safety_ok": bool(safety.get("ok", True)),
        "path_safety": safety,
        "plan_backend_class": route.plan.get("backend_class"),
        "native_backend_used": bool(route.plan.get("native_backend_used")),
        "native_runtime_ok": bool((route.plan.get("native_runtime") or {}).get("ok", True)),
        "pct_optimizer_enabled": route.plan.get("pct_optimizer_enabled"),
        "pct_optimizer_attempted": route.plan.get("pct_optimizer_attempted"),
        "pct_optimizer_accepted": route.plan.get("pct_optimizer_accepted"),
        "pct_optimizer_reject_reason": str(
            route.plan.get("pct_optimizer_reject_reason") or ""
        ),
        "pct_optimizer_blocked_sample_count": int(
            route.plan.get("pct_optimizer_blocked_sample_count") or 0
        ),
        "pct_planner_path_mode": str(route.plan.get("pct_planner_path_mode") or ""),
    }


def _source_map_artifacts(route: "PctRoute") -> dict[str, Any]:
    existing = route.case.get("map_artifacts")
    if isinstance(existing, dict):
        return dict(existing)

    assets = route.case.get("assets") if isinstance(route.case.get("assets"), dict) else {}
    metadata_raw = str(assets.get("map_metadata") or assets.get("metadata") or "")
    metadata_path = Path(metadata_raw) if metadata_raw else None
    metadata: dict[str, Any] = {}
    if metadata_path is not None and metadata_path.exists():
        try:
            loaded = json.loads(metadata_path.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                metadata = loaded
        except Exception:
            metadata = {}

    metadata_assets = (
        metadata.get("artifacts")
        if isinstance(metadata.get("artifacts"), dict)
        else {}
    )
    map_pcd = (
        dict(metadata_assets.get("map_pcd"))
        if isinstance(metadata_assets.get("map_pcd"), dict)
        else {}
    )
    tomogram = (
        dict(metadata_assets.get("tomogram"))
        if isinstance(metadata_assets.get("tomogram"), dict)
        else {}
    )
    map_sha = str(map_pcd.get("sha256") or "")
    tomogram_sha = str(tomogram.get("sha256") or "")
    tomogram_source_sha = str(tomogram.get("source_map_sha256") or "")
    same_source_pcd = bool(map_sha and int(map_pcd.get("point_count") or 0) > 0)
    same_source_tomogram = bool(
        tomogram_sha
        and tomogram_source_sha
        and map_sha
        and tomogram_source_sha == map_sha
    )
    return {
        "ok": same_source_pcd and same_source_tomogram,
        "source_contract": {
            "same_source_pcd": same_source_pcd,
            "same_source_tomogram": same_source_tomogram,
        },
        "metadata": {
            "path": str(metadata_path) if metadata_path is not None else "",
            "schema_version": metadata.get("schema_version") or "",
            "source_profile": metadata.get("source_profile") or "",
            "data_source": metadata.get("data_source") or "",
            "mapping_source": metadata.get("mapping_source") or "",
            "frame_id": metadata.get("frame_id") or "",
        },
        "assets": {
            "map_pcd": map_pcd,
            "tomogram": tomogram,
        },
    }


def _source_map_artifact_blockers(map_artifacts: dict[str, Any]) -> list[str]:
    source_contract = (
        map_artifacts.get("source_contract")
        if isinstance(map_artifacts.get("source_contract"), dict)
        else {}
    )
    assets = (
        map_artifacts.get("assets")
        if isinstance(map_artifacts.get("assets"), dict)
        else {}
    )
    map_pcd = assets.get("map_pcd") if isinstance(assets.get("map_pcd"), dict) else {}
    tomogram = (
        assets.get("tomogram") if isinstance(assets.get("tomogram"), dict) else {}
    )
    map_sha = str(map_pcd.get("sha256") or "")
    try:
        map_point_count = int(map_pcd.get("point_count") or 0)
    except (TypeError, ValueError):
        map_point_count = 0
    tomogram_sha = str(tomogram.get("sha256") or "")
    tomogram_source_sha = str(
        tomogram.get("source_map_sha256")
        or tomogram.get("input_pcd_sha256")
        or ""
    )

    blockers: list[str] = []
    if map_artifacts.get("ok") is not True:
        blockers.append("source report lacks same-source map/tomogram artifact proof")
    if source_contract.get("same_source_pcd") is not True:
        blockers.append("source report same_source_pcd is not true")
    if source_contract.get("same_source_tomogram") is not True:
        blockers.append("source report same_source_tomogram is not true")
    if not map_sha:
        blockers.append("source report map_pcd.sha256 missing")
    if map_point_count <= 0:
        blockers.append("source report map_pcd.point_count missing")
    if not tomogram_sha:
        blockers.append("source report tomogram.sha256 missing")
    if not tomogram_source_sha:
        blockers.append("source report tomogram.source_map_sha256 missing")
    if map_sha and tomogram_source_sha and tomogram_source_sha != map_sha:
        blockers.append("source report tomogram.source_map_sha256 does not match map_pcd.sha256")
    return blockers


def _write_report_if_requested(args: argparse.Namespace, report: dict[str, Any]) -> None:
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )


def _contract_only_report(args: argparse.Namespace) -> dict[str, Any]:
    if args.generate_source_report or args.force_generate_source_report:
        _generate_source_report(args)

    planner_name = str(getattr(args, "planner", "pct")).lower().strip()
    report: dict[str, Any] = {
        "schema_version": "lingtu.native_pct_mujoco_gate.v1",
        "execution_mode": "contract_only",
        "validation_only": True,
        "ok": False,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "driver_used": False,
        "gateway_used": False,
        "physical_gait_verified": False,
        "slam_verified": False,
        "real_lidar_verified": False,
        "planner": planner_name,
        "source_report": str(args.source_report),
        "route": str(args.route),
        "claim_boundary": "contract_only_no_ros_mujoco_motion",
        "validation_limitations": [
            "Validates the PCT source-report contract only.",
            "Does not launch ROS2 localPlanner/pathFollower.",
            "Does not create a MuJoCo engine or prove cmd_vel-driven motion.",
            "Does not prove Fast-LIO2 localization or hardware command safety.",
        ],
        "blockers": [],
    }

    try:
        route = _load_pct_route(args.source_report, route=args.route, planner=planner_name)
        source_contract = _planner_contract(route, planner_name)
        map_artifacts = _source_map_artifacts(route)
        command_generation = _command_generation_report(args, route=route)
        blockers: list[str] = []
        if planner_name == "pct" and source_contract["primary_planner"] != "pct":
            blockers.append("source report primary_planner is not pct")
        if planner_name == "pct" and source_contract["selected_planner"] != "pct":
            blockers.append("source report selected_planner is not pct")
        if source_contract["fallback_used"]:
            blockers.append("source report used planner fallback")
        if not source_contract["selected_route_ok"]:
            blockers.append("source report selected_route_ok is false")
        if not source_contract["path_safety_ok"]:
            blockers.append("source report path_safety is not ok")
        if planner_name == "pct" and not source_contract["native_backend_used"]:
            blockers.append("source report did not use PCT native backend")
        if planner_name == "pct" and not source_contract["native_runtime_ok"]:
            blockers.append("source report PCT native_runtime is not ok")
        if planner_name == "pct" and not source_contract["tomogram_exists"]:
            blockers.append("source report PCT tomogram is missing")
        if not map_artifacts.get("ok"):
            blockers.append("source report lacks same-source map/tomogram artifact proof")

        report.update(
            {
                "ok": not blockers,
                "blockers": blockers,
                "source_report": str(route.source_report),
                "scene_xml": str(route.scene_xml),
                "source_planning_contract": source_contract,
                "source_tomogram": source_contract["tomogram"],
                "source_tomogram_sha256": source_contract["tomogram_sha256"],
                "map_artifacts": map_artifacts,
                "command_generation": command_generation,
                "deliverable_contract": {
                    "checks": {
                        "same_source_map_artifact": map_artifacts.get("ok") is True,
                    }
                },
                "pct_backend_class": route.plan.get("backend_class"),
                "pct_native_backend_used": (
                    bool(route.plan.get("native_backend_used"))
                    if planner_name == "pct"
                    else False
                ),
                "pct_runtime_ok": bool(
                    (route.plan.get("native_runtime") or {}).get("ok", True)
                ),
                "pct_path_count": len(route.path),
                "pct_optimizer_enabled": source_contract["pct_optimizer_enabled"],
                "pct_optimizer_attempted": source_contract["pct_optimizer_attempted"],
                "pct_optimizer_accepted": source_contract["pct_optimizer_accepted"],
                "pct_optimizer_reject_reason": source_contract[
                    "pct_optimizer_reject_reason"
                ],
                "pct_optimizer_blocked_sample_count": source_contract[
                    "pct_optimizer_blocked_sample_count"
                ],
                "pct_planner_path_mode": source_contract["pct_planner_path_mode"],
                "pct_plan_ms": route.plan.get("plan_ms"),
                "pct_path": route.path,
                "start_xy": route.start[:2],
                "final_goal_xy": route.goal[:2],
                "contract_checks": {
                    "source_report_loads": True,
                    "planner_no_fallback": not source_contract["fallback_used"],
                    "pct_native_backend_used": (
                        source_contract["native_backend_used"]
                        if planner_name == "pct"
                        else None
                    ),
                    "pct_native_runtime_ok": (
                        source_contract["native_runtime_ok"]
                        if planner_name == "pct"
                        else None
                    ),
                    "pct_optimizer_mode_recorded": (
                        source_contract["pct_optimizer_enabled"] in (True, False)
                        if planner_name == "pct"
                        else None
                    ),
                    "pct_path_mode_supported": (
                        source_contract["pct_planner_path_mode"]
                        in {"native_astar_raw_path", "optimized_trajectory"}
                        if planner_name == "pct"
                        else None
                    ),
                    "pct_path_mode_matches_optimizer": (
                        _pct_path_mode_allowed_by_optimizer_contract(
                            source_contract["pct_optimizer_enabled"],
                            source_contract["pct_planner_path_mode"],
                            source_contract,
                        )
                        if planner_name == "pct"
                        else None
                    ),
                    "pct_optimizer_rejection_recorded": (
                        _pct_optimizer_rejection_recorded(source_contract)
                        if planner_name == "pct"
                        else None
                    ),
                    "tomogram_exists": source_contract["tomogram_exists"],
                    "path_safety_ok": source_contract["path_safety_ok"],
                    "same_source_map_artifact": map_artifacts.get("ok") is True,
                },
            }
        )
    except Exception as exc:
        report["ok"] = False
        report["error"] = str(exc)
        report["blockers"] = [str(exc)]
        report["contract_checks"] = {
            "source_report_loads": False,
            "planner_no_fallback": False,
            "pct_native_backend_used": None,
            "pct_native_runtime_ok": None,
            "pct_optimizer_mode_recorded": None,
            "pct_path_mode_supported": None,
            "pct_path_mode_matches_optimizer": None,
            "pct_optimizer_rejection_recorded": None,
            "tomogram_exists": False,
            "path_safety_ok": False,
            "same_source_map_artifact": False,
        }

    _write_report_if_requested(args, report)
    return report


def _pct_optimizer_rejection_recorded(plan: dict[str, Any]) -> bool:
    return bool(
        plan.get("pct_optimizer_attempted") is True
        and plan.get("pct_optimizer_accepted") is False
        and str(plan.get("pct_optimizer_reject_reason") or "").strip()
    )


def _pct_path_mode_allowed_by_optimizer_contract(
    optimizer_enabled: Any,
    path_mode: str,
    plan: dict[str, Any],
) -> bool:
    if optimizer_enabled not in (True, False):
        return False
    if path_mode not in {"native_astar_raw_path", "optimized_trajectory"}:
        return False
    if optimizer_enabled is False:
        return path_mode == "native_astar_raw_path"
    if path_mode == "optimized_trajectory":
        return plan.get("pct_optimizer_accepted") is not False
    return _pct_optimizer_rejection_recorded(plan)


def _load_pct_route(source_report: Path, *, route: str, planner: str = "pct") -> PctRoute:
    data = json.loads(source_report.read_text(encoding="utf-8"))
    case = _select_case(data, route)
    assets = case.get("assets") or {}
    planning = case.get("planning") or []
    if not isinstance(planning, list):
        raise ValueError("source report planning must be a list")

    selected = None
    for plan in planning:
        if not isinstance(plan, dict):
            continue
        if str(plan.get("planner", "")).lower() != planner:
            continue
        selected = plan
        break
    if selected is None:
        raise ValueError(f"planner {planner!r} not found in source report")

    if planner == "pct":
        selection = case.get("selection") if isinstance(case.get("selection"), dict) else {}
        selected_planner = str(selection.get("selected_planner") or selected.get("planner", "")).lower()
        primary_planner = str(selection.get("primary_planner") or case.get("primary_planner") or selected.get("planner", "")).lower()
        if primary_planner and primary_planner != "pct":
            raise ValueError(f"source report primary planner is {primary_planner!r}, not pct")
        if selected_planner and selected_planner != "pct":
            raise ValueError(
                f"source report selected planner is {selected_planner!r}; refusing PCT no-fallback gate"
            )
        if selection.get("fallback_used") is True:
            raise ValueError("source report used planner fallback; refusing PCT no-fallback gate")
        if selection.get("selected_route_ok") is False:
            raise ValueError("source report selected PCT route is not route_ok")
        if not selected.get("native_backend_used"):
            raise ValueError("PCT plan did not use native backend")
        runtime = selected.get("native_runtime") or {}
        if runtime and not runtime.get("ok"):
            raise ValueError("PCT native runtime is not healthy")
        optimizer_enabled = selected.get("pct_optimizer_enabled")
        if optimizer_enabled not in (True, False):
            raise ValueError("PCT plan did not record optimizer enabled/disabled mode")
        path_mode = str(selected.get("pct_planner_path_mode") or "").strip()
        if path_mode not in {"native_astar_raw_path", "optimized_trajectory"}:
            raise ValueError(
                "PCT plan did not record a supported path mode "
                "(native_astar_raw_path or optimized_trajectory)"
            )
        if not _pct_path_mode_allowed_by_optimizer_contract(
            optimizer_enabled,
            path_mode,
            selected,
        ):
            raise ValueError(
                "PCT plan optimizer mode/path mode lacks accepted trajectory "
                "or recorded optimizer rejection: "
                f"optimizer_enabled={optimizer_enabled!r}, "
                f"pct_planner_path_mode={path_mode!r}, "
                f"pct_optimizer_attempted={selected.get('pct_optimizer_attempted')!r}, "
                f"pct_optimizer_accepted={selected.get('pct_optimizer_accepted')!r}, "
                f"pct_optimizer_reject_reason={selected.get('pct_optimizer_reject_reason')!r}"
            )

    selected_safety = selected.get("path_safety")
    if selected_safety is None and str(case.get("primary_planner", "")).lower() == planner:
        selected_safety = case.get("path_safety")
    if isinstance(selected_safety, dict) and selected_safety and not selected_safety.get("ok", False):
        blocked_count = selected_safety.get("blocked_sample_count", "?")
        raise ValueError(
            f"selected {planner} source path failed path_safety "
            f"({blocked_count} blocked samples); refusing to run trajectory gate"
        )

    path = [_as_xyz(point, field="planning.path[]") for point in selected.get("path") or []]
    if len(path) < 2:
        raise ValueError("PCT path must contain at least two points")

    scene_xml_raw = assets.get("scene_xml")
    if not scene_xml_raw:
        raise ValueError("source report assets.scene_xml is required")
    scene_xml = Path(scene_xml_raw)
    if not scene_xml.exists():
        raise FileNotFoundError(f"MuJoCo scene XML is missing: {scene_xml}")
    if planner == "pct":
        tomogram_raw = assets.get("tomogram")
        if not tomogram_raw:
            raise ValueError("source report assets.tomogram is required for PCT no-fallback gate")
        tomogram = Path(tomogram_raw)
        if not tomogram.exists():
            raise FileNotFoundError(f"PCT source tomogram is missing: {tomogram}")

    start = _as_xyz(assets.get("start") or selected.get("start") or path[0], field="start")
    start[2] = 0.55
    goal = _as_xyz(selected.get("goal") or assets.get("goal") or path[-1], field="goal")

    return PctRoute(
        route=route,
        source_report=source_report,
        scene_xml=scene_xml,
        start=start,
        goal=goal,
        path=path,
        plan=selected,
        case=case,
    )


def _native_node_commands(
    *,
    path_folder: Path,
    max_speed: float,
    autonomy_speed: float,
    goal_clear_range: float,
    near_field_stop_distance: float,
    lookahead: float,
    stop_distance: float,
    obstacle_aware: bool,
    check_rot_obstacle: bool,
) -> tuple[list[str], list[str]]:
    local_planner_cmd = [
        "ros2",
        "run",
        "local_planner",
        "localPlanner",
        "--ros-args",
        "-p",
        f"pathFolder:={path_folder}",
        "-p",
        "autonomyMode:=true",
        "-p",
        "useTerrainAnalysis:=false",
        "-p",
        f"checkObstacle:={'true' if obstacle_aware else 'false'}",
        "-p",
        f"checkRotObstacle:={'true' if check_rot_obstacle else 'false'}",
        "-p",
        "pathCropByGoal:=true",
        "-p",
        f"maxSpeed:={float(max_speed)}",
        "-p",
        f"goalClearRange:={float(goal_clear_range)}",
        "-p",
        f"nearFieldStopDis:={float(near_field_stop_distance)}",
    ]
    path_follower_cmd = [
        "ros2",
        "run",
        "local_planner",
        "pathFollower",
        "--ros-args",
        "-p",
        "autonomyMode:=true",
        "-p",
        f"autonomySpeed:={float(autonomy_speed)}",
        "-p",
        f"maxSpeed:={float(max_speed)}",
        "-p",
        "maxAccel:=1.2",
        "-p",
        f"lookAheadDis:={float(lookahead)}",
        "-p",
        f"stopDisThre:={float(stop_distance)}",
        "-p",
        "noRotAtGoal:=true",
    ]
    return local_planner_cmd, path_follower_cmd


def _source_report_fingerprint(source_report: Path) -> dict[str, Any]:
    resolved = source_report.resolve() if source_report.exists() else source_report
    return {
        "path": str(source_report),
        "resolved_path": str(resolved),
        "exists": source_report.is_file(),
        "sha256": _sha256(source_report) if source_report.is_file() else "",
    }


def _command_generation_report(
    args: argparse.Namespace,
    *,
    route: PctRoute,
    path_folder: Path | None = None,
) -> dict[str, Any]:
    resolved_path_folder = path_folder or getattr(args, "path_folder", None)
    if resolved_path_folder is None:
        resolved_path_folder = _default_local_planner_path_folder()
    obstacle_aware = not bool(getattr(args, "disable_obstacle_check", False))
    local_planner_cmd, path_follower_cmd = _native_node_commands(
        path_folder=resolved_path_folder,
        max_speed=float(getattr(args, "max_speed", 0.4)),
        autonomy_speed=float(getattr(args, "autonomy_speed", 0.35)),
        goal_clear_range=float(getattr(args, "goal_clear_range", 0.45)),
        near_field_stop_distance=float(getattr(args, "near_field_stop_distance", 0.5)),
        lookahead=float(getattr(args, "lookahead", 0.55)),
        stop_distance=float(getattr(args, "stop_distance", 0.30)),
        obstacle_aware=obstacle_aware,
        check_rot_obstacle=bool(getattr(args, "check_rot_obstacle", False)),
    )
    return {
        "ok": True,
        "claim_boundary": "command_generation_only_no_process_launch",
        "source_report_fingerprint": _source_report_fingerprint(route.source_report),
        "route": route.route,
        "scene_xml": str(route.scene_xml),
        "path_folder": str(resolved_path_folder),
        "path_folder_exists": resolved_path_folder.is_dir(),
        "ros2_executable": shutil.which("ros2") or "",
        "ros_distro": os.environ.get("ROS_DISTRO", ""),
        "ros_domain_id": str(getattr(args, "ros_domain_id", os.environ.get("ROS_DOMAIN_ID", ""))),
        "python_executable": sys.executable,
        "pythonpath_prefix": [str(SRC), str(ROOT)],
        "commands": {
            "localPlanner": local_planner_cmd,
            "pathFollower": path_follower_cmd,
        },
        "resolved_executables": {
            "localPlanner": "ros2 run local_planner localPlanner",
            "pathFollower": "ros2 run local_planner pathFollower",
        },
        "parameters": {
            "max_speed": float(getattr(args, "max_speed", 0.4)),
            "autonomy_speed": float(getattr(args, "autonomy_speed", 0.35)),
            "goal_clear_range": float(getattr(args, "goal_clear_range", 0.45)),
            "near_field_stop_distance": float(getattr(args, "near_field_stop_distance", 0.5)),
            "lookahead": float(getattr(args, "lookahead", 0.55)),
            "stop_distance": float(getattr(args, "stop_distance", 0.30)),
            "obstacle_aware": obstacle_aware,
            "check_rot_obstacle": bool(getattr(args, "check_rot_obstacle", False)),
        },
        "safety_boundary": {
            "starts_gateway": False,
            "starts_driver": False,
            "starts_systemd": False,
            "requires_isolated_ros_domain_for_launch": True,
        },
    }


def _default_local_planner_path_folder() -> Path:
    candidates = (
        ROOT / "install/local_planner/share/local_planner/paths",
        ROOT / "install/share/local_planner/paths",
        ROOT / "src/base_autonomy/local_planner/paths",
    )
    for candidate in candidates:
        if (candidate / "startPaths.ply").is_file() and (candidate / "paths.ply").is_file():
            return candidate
    return candidates[0]


def _load_ros_modules() -> dict[str, Any]:
    if shutil.which("ros2") is None:
        raise RuntimeError("ros2 CLI is unavailable; source ROS2 and install/setup.bash first")
    try:
        import rclpy  # type: ignore
        from geometry_msgs.msg import PointStamped, TwistStamped  # type: ignore
        from nav_msgs.msg import Odometry
        from nav_msgs.msg import Path as RosPath
        from sensor_msgs.msg import PointCloud2, PointField  # type: ignore
        from sensor_msgs_py import point_cloud2  # type: ignore
        from std_msgs.msg import Float32, Header, Int8, String  # type: ignore
    except Exception as exc:  # pragma: no cover - depends on ROS2 env
        raise RuntimeError(
            "ROS2 Python modules are unavailable. Source /opt/ros/humble/setup.bash "
            "and the LingTu install/setup.bash before running this gate."
        ) from exc
    return {
        "rclpy": rclpy,
        "PointStamped": PointStamped,
        "TwistStamped": TwistStamped,
        "Odometry": Odometry,
        "RosPath": RosPath,
        "PointCloud2": PointCloud2,
        "PointField": PointField,
        "point_cloud2": point_cloud2,
        "Float32": Float32,
        "Header": Header,
        "Int8": Int8,
        "String": String,
    }


def _generate_source_report(args: argparse.Namespace) -> None:
    if args.source_report.exists() and not args.force_generate_source_report:
        return
    args.source_report.parent.mkdir(parents=True, exist_ok=True)
    output_dir = args.source_report.parent / "multifloor_source"
    cmd = [
        sys.executable,
        str(ROOT / "sim/scripts/multifloor_nav_validation.py"),
        "--output-dir",
        str(output_dir),
        "--route",
        args.route,
        "--planners",
        "pct,astar",
        "--strict",
        "--json-out",
        str(args.source_report),
    ]
    subprocess.run(cmd, cwd=str(ROOT), check=True)


def _spin_for(rclpy: Any, node: Any, seconds: float) -> None:
    deadline = time.time() + seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def _cmd_vel_subscriber_counts(rclpy: Any, node: Any, *, spin_s: float = 1.0) -> dict[str, int]:
    _spin_for(rclpy, node, spin_s)
    return {
        "/cmd_vel": int(node.count_subscribers("/cmd_vel")),
        "/nav/cmd_vel": int(node.count_subscribers("/nav/cmd_vel")),
    }


def _make_fields(PointField: Any) -> list[Any]:
    return [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]


def _flat_terrain_points(x: float, y: float, *, x_count: int = 13, y_count: int = 11) -> list[tuple[float, float, float, float]]:
    points: list[tuple[float, float, float, float]] = []
    for dx in np.linspace(-2.0, 3.0, x_count):
        for dy in np.linspace(-2.0, 2.0, y_count):
            points.append((x + float(dx), y + float(dy), 0.0, 1.0))
    return points


def _obstacle_boxes(route: PctRoute) -> list[dict[str, Any]]:
    assets = route.case.get("assets") or {}
    metadata_path = assets.get("metadata")
    if not metadata_path:
        return []
    path = Path(metadata_path)
    if not path.exists():
        return []
    try:
        metadata = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return []
    boxes = []
    for box in metadata.get("obstacles") or []:
        if not isinstance(box, dict):
            continue
        navigation_blocking = box.get("navigation_blocking")
        if navigation_blocking is False:
            continue
        kind = str(box.get("kind") or "obstacle").strip().lower()
        if navigation_blocking is None and kind not in {"obstacle", "boundary"}:
            continue
        name = str(box.get("name", ""))
        if "floor_plate" in name:
            continue
        floor_id = box.get("floor_id")
        if floor_id not in (0, None):
            continue
        half_size = box.get("half_size") or []
        position = box.get("position") or []
        if len(half_size) < 3 or len(position) < 3:
            continue
        if float(half_size[2]) < 0.1:
            continue
        boxes.append(box)
    return boxes


def _box_surface_points(
    box: dict[str, Any],
    *,
    spacing: float,
    intensity: float,
) -> list[tuple[float, float, float, float]]:
    cx, cy, cz = [float(v) for v in box["position"][:3]]
    hx, hy, hz = [float(v) for v in box["half_size"][:3]]
    z = max(0.15, min(cz + hz * 0.35, 0.65))
    xs = np.arange(cx - hx, cx + hx + spacing * 0.5, spacing)
    ys = np.arange(cy - hy, cy + hy + spacing * 0.5, spacing)
    points: list[tuple[float, float, float, float]] = []
    thin_x = hx <= spacing * 0.75
    thin_y = hy <= spacing * 0.75
    if thin_x or thin_y:
        for x in xs:
            for y in ys:
                points.append((float(x), float(y), float(z), float(intensity)))
        return points
    for x in xs:
        points.append((float(x), float(cy - hy), float(z), float(intensity)))
        points.append((float(x), float(cy + hy), float(z), float(intensity)))
    for y in ys:
        points.append((float(cx - hx), float(y), float(z), float(intensity)))
        points.append((float(cx + hx), float(y), float(z), float(intensity)))
    return points


def _obstacle_points_from_metadata(
    route: PctRoute,
    *,
    spacing: float,
    intensity: float = 200.0,
) -> tuple[list[tuple[float, float, float, float]], list[str]]:
    points: list[tuple[float, float, float, float]] = []
    names: list[str] = []
    for box in _obstacle_boxes(route):
        names.append(str(box.get("name", "")))
        points.extend(_box_surface_points(box, spacing=spacing, intensity=intensity))
    return points, names


def _point_box_clearance(point: tuple[float, float], box: dict[str, Any]) -> float:
    cx, cy = [float(v) for v in box["position"][:2]]
    hx, hy = [float(v) for v in box["half_size"][:2]]
    dx = abs(float(point[0]) - cx) - hx
    dy = abs(float(point[1]) - cy) - hy
    outside_x = max(dx, 0.0)
    outside_y = max(dy, 0.0)
    if dx <= 0.0 and dy <= 0.0:
        return -min(-dx, -dy)
    return math.hypot(outside_x, outside_y)


def _trail_obstacle_clearance(
    trail: list[tuple[float, float]],
    route: PctRoute,
    *,
    robot_radius: float,
) -> dict[str, Any]:
    boxes = _obstacle_boxes(route)
    if not trail or not boxes:
        return {
            "checked": bool(trail),
            "box_count": len(boxes),
            "min_clearance_m": None,
            "min_clearance_minus_robot_radius_m": None,
            "collision": False,
            "nearest_obstacle": None,
        }
    best = None
    best_name = None
    for point in trail:
        for box in boxes:
            clearance = _point_box_clearance(point, box)
            if best is None or clearance < best:
                best = clearance
                best_name = box.get("name")
    assert best is not None
    margin = best - float(robot_radius)
    return {
        "checked": True,
        "box_count": len(boxes),
        "robot_radius_m": float(robot_radius),
        "min_clearance_m": round(float(best), 4),
        "min_clearance_minus_robot_radius_m": round(float(margin), 4),
        "collision": bool(margin < 0.0),
        "nearest_obstacle": best_name,
    }


def _path_length_xy(points_xy: Any) -> float:
    points = _xy_points(points_xy)
    if len(points) < 2:
        return 0.0
    return float(
        sum(
            math.hypot(points[idx].x - points[idx - 1].x, points[idx].y - points[idx - 1].y)
            for idx in range(1, len(points))
        )
    )


def _path_length_xy_numpy(points_xy: Any) -> float:
    if len(points_xy) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(points_xy, axis=0), axis=1)))


def _waypoint_goal_preservation(
    original: list[list[float]],
    filtered: list[list[float]],
    *,
    tolerance_m: float = 0.25,
) -> dict[str, Any]:
    original_goal = original[-1][:2] if original else []
    follow_goal = filtered[-1][:2] if filtered else []
    distance = (
        float(math.dist(original_goal, follow_goal))
        if len(original_goal) == 2 and len(follow_goal) == 2
        else float("inf")
    )
    return {
        "goal_preserved": bool(distance <= float(tolerance_m)),
        "goal_preservation_tolerance_m": float(tolerance_m),
        "source_path_goal_xy": original_goal,
        "follow_path_goal_xy": follow_goal,
        "goal_shift_m": None if not math.isfinite(distance) else round(distance, 4),
    }


def _local_path_obstacle_evidence(
    *,
    local_path: list[list[float]],
    local_path_frame_id: str,
    state: Any,
    route: PctRoute,
    robot_radius: float,
    sample_step_m: float,
    extra_boxes: list[dict[str, Any]] | None = None,
    include_route_boxes: bool = True,
) -> dict[str, Any]:
    boxes = (_obstacle_boxes(route) if include_route_boxes else []) + list(extra_boxes or [])
    robot_frame = _path_is_robot_frame(local_path_frame_id)
    if len(local_path) < 2:
        return {
            "ok": False,
            "reason": "local_path_too_short",
            "frame_id": str(local_path_frame_id or ""),
            "robot_frame": robot_frame,
            "point_count": len(local_path),
            "box_count": len(boxes),
            "min_clearance_m": None,
            "min_clearance_minus_robot_radius_m": None,
            "collision": False,
            "points_into_obstacle": False,
        }

    path_xy = _xy_points(local_path)
    if robot_frame:
        local_xy = path_xy
        world_xy = _xy_points(_robot_xy_to_world_xy(path_xy, state))
    else:
        world_xy = path_xy
        local_xy = _xy_points(_world_xy_to_robot_xy(path_xy, state))

    forward_count = sum(1 for point in local_xy if point.x >= -0.05)
    behind_count = int(len(local_xy) - forward_count)
    best = None
    best_name = None
    segment_hits: list[dict[str, Any]] = []
    for idx, point in enumerate(world_xy):
        for box in boxes:
            clearance = _point_box_clearance((point.x, point.y), box)
            if best is None or clearance < best:
                best = clearance
                best_name = box.get("name")
        if idx == 0:
            continue
        a = [world_xy[idx - 1].x, world_xy[idx - 1].y, 0.0]
        b = [world_xy[idx].x, world_xy[idx].y, 0.0]
        for box in boxes:
            if _segment_intersects_expanded_box(
                a,
                b,
                box,
                clearance=float(robot_radius),
                sample_step_m=sample_step_m,
            ):
                segment_hits.append(
                    {
                        "segment_index": idx - 1,
                        "obstacle": box.get("name"),
                    }
                )
                break

    margin = None if best is None else best - float(robot_radius)
    points_into_obstacle = bool(segment_hits)
    collision = bool((margin is not None and margin < 0.0) or points_into_obstacle)
    return {
        "ok": not collision and not points_into_obstacle and forward_count > 0,
        "reason": "" if forward_count > 0 else "local_path_not_forward",
        "frame_id": str(local_path_frame_id or ""),
        "robot_frame": robot_frame,
        "point_count": int(len(local_path)),
        "forward_point_count": forward_count,
        "behind_point_count": behind_count,
        "box_count": len(boxes),
        "path_length_m": round(_path_length_xy(world_xy), 4),
        "max_abs_lateral_m": round(max(abs(point.y) for point in local_xy), 4),
        "min_clearance_m": None if best is None else round(float(best), 4),
        "min_clearance_minus_robot_radius_m": (
            None if margin is None else round(float(margin), 4)
        ),
        "collision": collision,
        "points_into_obstacle": points_into_obstacle,
        "nearest_obstacle": best_name,
        "segment_hit_count": len(segment_hits),
        "segment_hits": segment_hits[:8],
    }


def _summarize_local_path_obstacle_evidence(
    *,
    samples: list[dict[str, Any]],
    path_count: int,
    stop_samples: list[int],
    slow_down_samples: list[int],
    obstacle_aware: bool,
) -> dict[str, Any]:
    valid_margins = [
        float(sample["min_clearance_minus_robot_radius_m"])
        for sample in samples
        if sample.get("min_clearance_minus_robot_radius_m") is not None
    ]
    collisions = [sample for sample in samples if sample.get("collision")]
    point_hits = [sample for sample in samples if sample.get("points_into_obstacle")]
    stopped = [value for value in stop_samples if int(value) > 0]
    slowed = [value for value in slow_down_samples if int(value) > 0]
    has_local_path = int(path_count) > 0 and bool(samples)
    ok = has_local_path and (not obstacle_aware or (not collisions and not point_hits and not stopped))
    reasons: list[str] = []
    if not has_local_path:
        reasons.append("local_path_missing")
    if collisions:
        reasons.append("local_path_collision_margin")
    if point_hits:
        reasons.append("local_path_points_into_obstacle")
    if stopped:
        reasons.append("near_field_stop")
    return {
        "schema_version": "lingtu.native_local_path_obstacle_evidence.v1",
        "ok": bool(ok),
        "obstacle_aware": bool(obstacle_aware),
        "path_update_count": int(path_count),
        "sample_count": len(samples),
        "min_clearance_minus_robot_radius_m": (
            None if not valid_margins else round(min(valid_margins), 4)
        ),
        "collision_sample_count": len(collisions),
        "points_into_obstacle_sample_count": len(point_hits),
        "near_field_stop_count": len(stopped),
        "slow_down_count": len(slowed),
        "stop_samples": stop_samples[:12],
        "slow_down_samples": slow_down_samples[:12],
        "reasons": reasons,
        "worst_sample": (
            None
            if not samples
            else min(
                samples,
                key=lambda item: (
                    float("inf")
                    if item.get("min_clearance_minus_robot_radius_m") is None
                    else float(item["min_clearance_minus_robot_radius_m"])
                ),
            )
        ),
    }


def _polyline_progress_and_error(point: tuple[float, float], reference: Any, cumulative: Any) -> tuple[float, float]:
    best_distance = float("inf")
    best_progress = 0.0
    px, py = float(point[0]), float(point[1])
    for idx in range(len(reference) - 1):
        ax, ay = reference[idx]
        bx, by = reference[idx + 1]
        vx = float(bx - ax)
        vy = float(by - ay)
        seg_len_sq = vx * vx + vy * vy
        if seg_len_sq <= 1e-12:
            t = 0.0
        else:
            t = max(0.0, min(1.0, ((px - float(ax)) * vx + (py - float(ay)) * vy) / seg_len_sq))
        proj_x = float(ax) + t * vx
        proj_y = float(ay) + t * vy
        distance = math.hypot(px - proj_x, py - proj_y)
        if distance < best_distance:
            best_distance = distance
            best_progress = float(cumulative[idx]) + math.sqrt(seg_len_sq) * t
    return best_progress, best_distance


def _trajectory_correctness(
    *,
    trail: list[tuple[float, float]],
    reference_path: list[list[float]],
    max_p95_error_m: float,
    max_progress_regressions: int,
    min_route_progress_ratio: float,
    reached_goal: bool = False,
) -> dict[str, Any]:
    if len(trail) < 2 or len(reference_path) < 2:
        return {
            "ok": False,
            "reference": "safe_follow_path",
            "reason": "not enough trail/reference points",
            "trail_sample_count": len(trail),
            "reference_point_count": len(reference_path),
        }

    reference = _xy_points(reference_path)
    segment_lengths = [
        math.hypot(reference[idx].x - reference[idx - 1].x, reference[idx].y - reference[idx - 1].y)
        for idx in range(1, len(reference))
    ]
    cumulative = [0.0]
    for length in segment_lengths:
        cumulative.append(cumulative[-1] + length)
    total = float(cumulative[-1])
    if total <= 1e-6:
        return {
            "ok": False,
            "reference": "safe_follow_path",
            "reason": "reference path length is zero",
            "trail_sample_count": len(trail),
            "reference_point_count": len(reference_path),
        }

    progresses: list[float] = []
    errors: list[float] = []
    for point in trail:
        progress, error = _polyline_progress_and_error(point, reference, cumulative)
        progresses.append(progress)
        errors.append(error)

    regressions = 0
    max_regression = 0.0
    prev = progresses[0]
    for progress in progresses[1:]:
        regression = prev - progress
        if regression > 0.10:
            regressions += 1
            max_regression = max(max_regression, regression)
        prev = max(prev, progress)

    p95 = _percentile(errors, 95)
    mean = float(sum(errors) / len(errors))
    max_error = float(max(errors))
    final_ratio = float(progresses[-1] / total)
    progress_ok = (
        final_ratio >= float(min_route_progress_ratio)
        or bool(reached_goal)
    )
    ok = (
        p95 <= float(max_p95_error_m)
        and regressions <= int(max_progress_regressions)
        and progress_ok
    )
    return {
        "ok": bool(ok),
        "reference": "safe_follow_path",
        "trail_sample_count": len(trail),
        "reference_point_count": len(reference_path),
        "reference_length_m": round(total, 4),
        "final_progress_m": round(float(progresses[-1]), 4),
        "final_progress_ratio": round(final_ratio, 4),
        "route_progress_ok": bool(progress_ok),
        "goal_reached_override": bool(
            reached_goal and final_ratio < float(min_route_progress_ratio)
        ),
        "mean_lateral_error_m": round(mean, 4),
        "p95_lateral_error_m": round(p95, 4),
        "max_lateral_error_m": round(max_error, 4),
        "progress_regression_count": int(regressions),
        "max_progress_regression_m": round(float(max_regression), 4),
        "thresholds": {
            "max_p95_error_m": float(max_p95_error_m),
            "max_progress_regressions": int(max_progress_regressions),
            "min_route_progress_ratio": float(min_route_progress_ratio),
        },
    }


def _point_inside_expanded_box(point: tuple[float, float], box: dict[str, Any], clearance: float) -> bool:
    cx, cy = [float(v) for v in box["position"][:2]]
    hx, hy = [float(v) for v in box["half_size"][:2]]
    return abs(float(point[0]) - cx) <= hx + clearance and abs(float(point[1]) - cy) <= hy + clearance


def _segment_intersects_expanded_box(
    a: list[float],
    b: list[float],
    box: dict[str, Any],
    *,
    clearance: float,
    sample_step_m: float,
) -> bool:
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    distance = max(math.hypot(bx - ax, by - ay), sample_step_m)
    steps = max(2, int(math.ceil(distance / max(sample_step_m, 0.02))))
    for i in range(steps + 1):
        t = i / steps
        if _point_inside_expanded_box((ax + (bx - ax) * t, ay + (by - ay) * t), box, clearance):
            return True
    return False


def _detour_for_box(
    a: list[float],
    b: list[float],
    box: dict[str, Any],
    *,
    clearance: float,
    extra_margin: float,
) -> list[list[float]]:
    candidates = _detour_candidates_for_box(
        a,
        b,
        box,
        clearance=clearance,
        extra_margin=extra_margin,
    )
    return candidates[0] if candidates else []


def _detour_candidates_for_box(
    a: list[float],
    b: list[float],
    box: dict[str, Any],
    *,
    clearance: float,
    extra_margin: float,
) -> list[list[list[float]]]:
    cx, cy = [float(v) for v in box["position"][:2]]
    hx, hy = [float(v) for v in box["half_size"][:2]]
    z = float(b[2]) if len(b) > 2 else 0.0
    planner_padding = 0.05
    expanded_x = hx + clearance + extra_margin + planner_padding
    expanded_y = hy + clearance + extra_margin + planner_padding
    x_before = cx - expanded_x
    x_after = cx + expanded_x
    if float(b[0]) < float(a[0]):
        x_before, x_after = x_after, x_before

    candidates: list[list[list[float]]] = []
    for y in (cy - expanded_y, cy + expanded_y):
        candidates.append(
            [
                [float(a[0]), float(y), z],
                [float(b[0]), float(y), z],
                [float(b[0]), float(b[1]), z],
            ]
        )
        candidates.append(
            [
                [float(x_before), float(y), z],
                [float(x_after), float(y), z],
                [float(x_after), float(b[1]), z],
            ]
        )
    for x in (cx - expanded_x, cx + expanded_x):
        candidates.append(
            [
                [float(x), float(a[1]), z],
                [float(x), float(b[1]), z],
                [float(b[0]), float(b[1]), z],
            ]
        )

    def cost(detour: list[list[float]]) -> float:
        points = [a, *detour, b]
        return sum(
            math.hypot(
                float(points[idx][0]) - float(points[idx - 1][0]),
                float(points[idx][1]) - float(points[idx - 1][1]),
            )
            for idx in range(1, len(points))
        )

    unique: dict[tuple[tuple[float, float], ...], list[list[float]]] = {}
    for detour in candidates:
        key = tuple((round(float(point[0]), 4), round(float(point[1]), 4)) for point in detour)
        unique.setdefault(key, detour)
    return sorted(unique.values(), key=cost)


def _detour_is_locally_progressive(a: list[float], b: list[float], detour: list[list[float]]) -> bool:
    if not detour:
        return False
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    dx = bx - ax
    dy = by - ay
    direct = max(math.hypot(dx, dy), 1e-6)
    first = detour[0]
    first_dx = float(first[0]) - ax
    first_dy = float(first[1]) - ay
    first_dist = math.hypot(first_dx, first_dy)
    projection = (first_dx * dx + first_dy * dy) / direct
    if first_dist > max(1.0, direct * 2.5):
        return False
    return projection >= -0.10


def _dedupe_path(path: list[list[float]], *, min_step_m: float = 0.08) -> list[list[float]]:
    deduped: list[list[float]] = []
    for point in path:
        if not deduped:
            deduped.append(point)
            continue
        prev = deduped[-1]
        if math.hypot(float(point[0]) - float(prev[0]), float(point[1]) - float(prev[1])) >= min_step_m:
            deduped.append(point)
    return deduped


def _detour_is_safe(
    a: list[float],
    b: list[float],
    detour: list[list[float]],
    boxes: list[dict[str, Any]],
    *,
    clearance: float,
    sample_step_m: float,
) -> bool:
    if not detour or not _detour_is_locally_progressive(a, b, detour):
        return False
    candidate = [a, *detour, b]
    for point in detour:
        if any(
            _point_inside_expanded_box((float(point[0]), float(point[1])), box, clearance)
            for box in boxes
        ):
            return False
    for start, end in zip(candidate, candidate[1:]):
        if any(
            _segment_intersects_expanded_box(
                start,
                end,
                box,
                clearance=clearance,
                sample_step_m=sample_step_m,
            )
            for box in boxes
        ):
            return False
    return True


def _build_safe_follow_path(
    route: PctRoute,
    *,
    enabled: bool,
    clearance: float,
    extra_margin: float,
    sample_step_m: float,
) -> tuple[list[list[float]], dict[str, Any]]:
    original = [[float(v) for v in point[:3]] for point in route.path]
    if not enabled:
        return original, {
            "enabled": False,
            "inserted_count": 0,
            "path_count": len(original),
            **_waypoint_goal_preservation(original, original),
        }

    boxes = _obstacle_boxes(route)
    avoidance_clearance = float(clearance) + float(extra_margin)
    path: list[list[float]] = [original[0]]
    insertions: list[dict[str, Any]] = []
    rejected_detours = 0
    skipped_unsafe_waypoints = 0
    for b in original[1:]:
        a = path[-1]
        unsafe_box = next(
            (
                box
                for box in boxes
                if _point_inside_expanded_box(
                    (float(b[0]), float(b[1])),
                    box,
                    avoidance_clearance,
                )
            ),
            None,
        )
        inserted_for_segment: list[list[float]] = []
        hit_box = next(
            (
                box
                for box in boxes
                if _segment_intersects_expanded_box(
                    a,
                    b,
                    box,
                    clearance=avoidance_clearance,
                    sample_step_m=sample_step_m,
                )
            ),
            None,
        )
        if hit_box is not None and unsafe_box is None:
            detour_candidates = _detour_candidates_for_box(
                a,
                b,
                hit_box,
                clearance=float(clearance),
                extra_margin=float(extra_margin),
            )
            safe_detour = next(
                (
                    detour
                    for detour in detour_candidates
                    if _detour_is_safe(
                        a,
                        b,
                        detour,
                        boxes,
                        clearance=avoidance_clearance,
                        sample_step_m=sample_step_m,
                    )
                ),
                None,
            )
            if safe_detour is not None:
                inserted_for_segment = safe_detour
                insertions.append(
                    {
                        "obstacle": str(hit_box.get("name") or ""),
                        "from": a,
                        "to": b,
                        "detour": safe_detour,
                    }
                )
            else:
                rejected_detours += 1
        path.extend(inserted_for_segment)
        if unsafe_box is not None:
            skipped_unsafe_waypoints += 1
            continue
        path.append(b)

    filtered = _dedupe_path(path)
    return filtered, {
        "enabled": True,
        "box_count": len(boxes),
        "clearance_m": float(clearance),
        "extra_margin_m": float(extra_margin),
        "avoidance_clearance_m": avoidance_clearance,
        "original_count": len(original),
        "path_count": len(filtered),
        "inserted_count": sum(len(item["detour"]) for item in insertions),
        "auto_detour_enabled": True,
        "rejected_detour_count": rejected_detours,
        "skipped_unsafe_waypoint_count": skipped_unsafe_waypoints,
        "insertions": insertions,
        **_waypoint_goal_preservation(original, filtered),
    }


def _route_frame_at_ratio(route: PctRoute, ratio: float) -> tuple[_Vector2, _Vector2, _Vector2]:
    path = _xy_points(route.path)
    if len(path) < 2:
        anchor = _as_vec2(route.goal[:2])
        tangent = _Vector2(1.0, 0.0)
        return anchor, tangent, _Vector2(0.0, 1.0)
    segments = [path[idx + 1] - path[idx] for idx in range(len(path) - 1)]
    lengths = [math.hypot(segment.x, segment.y) for segment in segments]
    total = float(sum(lengths))
    if total < 1e-6:
        anchor = path[0]
        tangent = _Vector2(1.0, 0.0)
        return anchor, tangent, _Vector2(0.0, 1.0)
    target = _clip(float(ratio), 0.0, 1.0) * total
    accum = 0.0
    anchor = path[-1]
    tangent = _Vector2(1.0, 0.0)
    for idx, (segment, length) in enumerate(zip(segments, lengths)):
        seg_len = float(length)
        if seg_len < 1e-6:
            continue
        if target <= accum + seg_len or idx == len(segments) - 1:
            local_t = _clip((target - accum) / seg_len, 0.0, 1.0)
            anchor = path[idx] + segment * local_t
            tangent = _Vector2(segment.x / seg_len, segment.y / seg_len)
            break
        accum += seg_len
    normal = _Vector2(-tangent.y, tangent.x)
    return anchor, tangent, normal


def _moving_obstacle_boxes(route: PctRoute, args: argparse.Namespace, elapsed_s: float) -> list[dict[str, Any]]:
    mode = str(getattr(args, "moving_obstacle_mode", "none") or "none")
    if mode == "none":
        return []
    start_s = float(getattr(args, "moving_obstacle_start_s", 0.0))
    duration_s = float(getattr(args, "moving_obstacle_duration_s", 0.0))
    if elapsed_s < start_s:
        return []
    if duration_s > 0.0 and elapsed_s > start_s + duration_s:
        return []

    base_ratio = float(getattr(args, "moving_obstacle_route_ratio", 0.5))
    count = int(max(1, min(32, int(getattr(args, "moving_obstacle_count", 1)))))
    ratio_step = max(0.0, float(getattr(args, "moving_obstacle_route_ratio_step", 0.0)))
    phase_step = float(getattr(args, "moving_obstacle_lateral_phase_step_rad", 0.0))
    t = elapsed_s - start_s
    period = max(0.1, float(getattr(args, "moving_obstacle_period_s", 10.0)))
    base_phase = 2.0 * math.pi * t / period
    lateral_amp = float(getattr(args, "moving_obstacle_lateral_amplitude_m", 0.75))
    along_amp = float(getattr(args, "moving_obstacle_along_amplitude_m", 0.25))
    radius = max(0.02, float(getattr(args, "moving_obstacle_radius_m", 0.22)))
    height = max(0.05, float(getattr(args, "moving_obstacle_height_m", 0.7)))

    boxes: list[dict[str, Any]] = []
    for obstacle_idx in range(count):
        route_offset = (float(obstacle_idx) - (float(count) - 1.0) * 0.5) * ratio_step
        anchor, tangent, normal = _route_frame_at_ratio(route, base_ratio + route_offset)
        phase = base_phase + float(obstacle_idx) * phase_step
        lateral = lateral_amp * math.sin(phase)
        along = along_amp * math.sin(phase * 0.5)
        center = anchor + normal * lateral + tangent * along
        boxes.append(
            {
                "name": "moving_crossing_obstacle" if count == 1 else f"moving_crossing_obstacle_{obstacle_idx}",
                "floor_id": 0,
                "position": [float(center[0]), float(center[1]), height * 0.5],
                "half_size": [radius, radius, height * 0.5],
                "elapsed_s": float(elapsed_s),
            }
        )
    return boxes


def _moving_obstacle_points(
    boxes: list[dict[str, Any]],
    *,
    spacing: float,
    intensity: float,
) -> list[tuple[float, float, float, float]]:
    points: list[tuple[float, float, float, float]] = []
    for box in boxes:
        points.extend(_box_surface_points(box, spacing=spacing, intensity=intensity))
    return points


def _moving_obstacle_trail_clearance(
    *,
    timed_trail: list[tuple[float, float, float]],
    route: PctRoute,
    args: argparse.Namespace,
    robot_radius: float,
) -> dict[str, Any]:
    if str(getattr(args, "moving_obstacle_mode", "none") or "none") == "none":
        return {"checked": False, "collision": False}
    best: float | None = None
    best_sample: dict[str, Any] | None = None
    active_count = 0
    for elapsed_s, x, y in timed_trail:
        boxes = _moving_obstacle_boxes(route, args, elapsed_s)
        if not boxes:
            continue
        active_count += 1
        for box in boxes:
            clearance = _point_box_clearance((x, y), box)
            if best is None or clearance < best:
                best = clearance
                best_sample = {
                    "elapsed_s": round(float(elapsed_s), 3),
                    "robot_xy": [round(float(x), 4), round(float(y), 4)],
                    "obstacle": box.get("name"),
                    "obstacle_xy": [
                        round(float(box["position"][0]), 4),
                        round(float(box["position"][1]), 4),
                    ],
                }
    margin = None if best is None else float(best) - float(robot_radius)
    return {
        "checked": True,
        "active_trail_sample_count": int(active_count),
        "min_clearance_m": None if best is None else round(float(best), 4),
        "min_clearance_minus_robot_radius_m": None if margin is None else round(float(margin), 4),
        "collision": bool(margin is not None and margin < 0.0),
        "nearest_sample": best_sample,
    }


def _append_lidar_points(engine: Any, points: list[tuple[float, float, float, float]], limit: int) -> None:
    if limit <= 0:
        return
    lidar = engine.get_lidar_points()
    if lidar is None or len(lidar) == 0:
        return
    for row in lidar[: min(limit, len(lidar))]:
        intensity = float(row[3]) if len(row) > 3 else 1.0
        points.append((float(row[0]), float(row[1]), float(row[2]), intensity))


def _empty_lidar_points() -> Any:
    if numpy_import_is_safe():
        return np.zeros((0, 4), dtype=np.float32)
    return _Array2D([])


def _sample_lidar_points(engine: Any, limit: int) -> Any:
    if limit <= 0:
        return _empty_lidar_points()
    lidar = engine.get_lidar_points()
    if lidar is None:
        return _empty_lidar_points()
    if not numpy_import_is_safe():
        rows: list[list[float]] = []
        for row in lidar:
            values = [float(row[0]), float(row[1]), float(row[2])]
            values.append(float(row[3]) if len(row) > 3 else 1.0)
            rows.append(values[:4])
        if not rows:
            return _empty_lidar_points()
        if len(rows) > limit:
            step = max(1, int(math.ceil(len(rows) / float(limit))))
            rows = rows[::step][:limit]
        return _Array2D(rows)
    pts = np.asarray(lidar, dtype=np.float32)
    if pts.size == 0:
        return _empty_lidar_points()
    if pts.ndim != 2:
        pts = pts.reshape((-1, pts.shape[-1]))
    if pts.shape[1] == 3:
        pts = np.hstack([pts, np.ones((len(pts), 1), dtype=np.float32)])
    if len(pts) <= limit:
        return pts[:, :4].copy()
    step = max(1, int(math.ceil(len(pts) / float(limit))))
    return pts[::step, :4][:limit].copy()


def _yaw_from_quat_xyzw(quat: Any) -> float:
    x, y, z, w = [float(v) for v in quat[:4]]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _world_xy_to_robot_xy(points_xy: Any, state: Any) -> Any:
    if not numpy_import_is_safe():
        yaw = _yaw_from_quat_xyzw(state.orientation)
        c = math.cos(yaw)
        s = math.sin(yaw)
        sx = float(state.position[0])
        sy = float(state.position[1])
        return [
            _Vector2((point.x - sx) * c + (point.y - sy) * s, -(point.x - sx) * s + (point.y - sy) * c)
            for point in _xy_points(points_xy)
        ]
    pts = np.asarray(points_xy, dtype=np.float64)
    if pts.size == 0:
        return np.zeros((0, 2), dtype=np.float64)
    pts = pts.reshape((-1, 2))
    yaw = _yaw_from_quat_xyzw(state.orientation)
    c = math.cos(yaw)
    s = math.sin(yaw)
    dx = pts[:, 0] - float(state.position[0])
    dy = pts[:, 1] - float(state.position[1])
    # Robot frame: +x forward, +y left.
    return np.stack([dx * c + dy * s, -dx * s + dy * c], axis=1)


def _robot_xy_to_world_xy(points_xy: Any, state: Any) -> Any:
    if not numpy_import_is_safe():
        yaw = _yaw_from_quat_xyzw(state.orientation)
        c = math.cos(yaw)
        s = math.sin(yaw)
        sx = float(state.position[0])
        sy = float(state.position[1])
        return [
            _Vector2(sx + point.x * c - point.y * s, sy + point.x * s + point.y * c)
            for point in _xy_points(points_xy)
        ]
    pts = np.asarray(points_xy, dtype=np.float64)
    if pts.size == 0:
        return np.zeros((0, 2), dtype=np.float64)
    pts = pts.reshape((-1, 2))
    yaw = _yaw_from_quat_xyzw(state.orientation)
    c = math.cos(yaw)
    s = math.sin(yaw)
    x = float(state.position[0]) + pts[:, 0] * c - pts[:, 1] * s
    y = float(state.position[1]) + pts[:, 0] * s + pts[:, 1] * c
    return np.stack([x, y], axis=1)


def _path_is_robot_frame(frame_id: str) -> bool:
    normalized = str(frame_id or "").strip().lstrip("/")
    if not normalized:
        return True
    return normalized not in {"map", "odom", "world"}


def _path_window_by_distance(path: list[list[float]], *, start_idx: int, lookahead_m: float) -> list[list[float]]:
    if not path:
        return []
    idx = max(0, min(int(start_idx), len(path) - 1))
    window = [path[idx]]
    traveled = 0.0
    prev = path[idx]
    for point in path[idx + 1 :]:
        traveled += math.hypot(float(point[0]) - float(prev[0]), float(point[1]) - float(prev[1]))
        window.append(point)
        prev = point
        if traveled >= float(lookahead_m):
            break
    return window


def _wrap_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _select_local_target(
    *,
    state: Any,
    local_path: list[list[float]],
    local_path_frame_id: str,
    fallback_target: list[float],
    lookahead_m: float,
) -> tuple[Any, Any]:
    """Return a local-frame target and the equivalent world-frame target."""

    target_local: Any | None = None
    if len(local_path) >= 2:
        path_xy = _xy_points(local_path)
        local_arr = (
            path_xy
            if _path_is_robot_frame(local_path_frame_id)
            else _xy_points(_world_xy_to_robot_xy(path_xy, state))
        )
        candidates = [point for point in local_arr if math.hypot(point.x, point.y) > 0.08]
        if len(candidates):
            forward = [point for point in candidates if point.x > -0.15]
            candidates = forward if len(forward) else candidates
            distances = [math.hypot(point.x, point.y) for point in candidates]
            threshold = max(float(lookahead_m), 0.05)
            idx = next(
                (candidate_idx for candidate_idx, distance in enumerate(distances) if distance >= threshold),
                max(range(len(distances)), key=lambda candidate_idx: distances[candidate_idx]),
            )
            target_local = candidates[idx]

    if target_local is None:
        target_local = _as_vec2(_world_xy_to_robot_xy([fallback_target[:2]], state)[0])

    target_world = _as_vec2(_robot_xy_to_world_xy([target_local], state)[0])
    return target_local, target_world


def _omni_cart_cmd(
    *,
    state: Any,
    local_path: list[list[float]],
    local_path_frame_id: str,
    fallback_target: list[float],
    lookahead_m: float,
    min_speed: float,
    max_speed: float,
    max_lateral_speed: float,
    yaw_gain: float,
    max_yaw_rate: float,
    yaw_deadband: float,
) -> tuple[Any, dict[str, Any]]:
    target_local, target_world = _select_local_target(
        state=state,
        local_path=local_path,
        local_path_frame_id=local_path_frame_id,
        fallback_target=fallback_target,
        lookahead_m=lookahead_m,
    )
    distance = math.hypot(float(target_local[0]), float(target_local[1]))
    debug = {
        "source": "omni_local_path_tracker",
        "target_distance_m": round(distance, 4),
        "target_local": [round(float(v), 4) for v in target_local[:2]],
        "target_world": [round(float(v), 4) for v in target_world[:2]],
    }
    if distance < 0.05:
        return _VelocityCommand(), debug

    speed = min(float(max_speed), max(float(min_speed), min(float(max_speed), distance * 0.8)))
    vx = _clip(speed * target_local[0] / distance, -float(max_speed), float(max_speed))
    vy = _clip(speed * target_local[1] / distance, -float(max_lateral_speed), float(max_lateral_speed))

    dx = float(target_world[0]) - float(state.position[0])
    dy = float(target_world[1]) - float(state.position[1])
    target_yaw = math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-6 else _yaw_from_quat_xyzw(state.orientation)
    yaw_error = _wrap_angle(target_yaw - _yaw_from_quat_xyzw(state.orientation))
    wz = 0.0 if abs(yaw_error) < float(yaw_deadband) else _clip(float(yaw_gain) * yaw_error, -float(max_yaw_rate), float(max_yaw_rate))
    debug["yaw_error_rad"] = round(float(yaw_error), 4)
    return _VelocityCommand(linear_x=vx, linear_y=vy, angular_z=wz), debug


def _ros_path_to_points(msg: Any) -> list[list[float]]:
    points: list[list[float]] = []
    for pose_stamped in getattr(msg, "poses", []) or []:
        pos = pose_stamped.pose.position
        points.append([float(pos.x), float(pos.y), float(pos.z)])
    return points


def _put_text(frame: np.ndarray, text: str, y: int, color: tuple[int, int, int] = (255, 255, 255)) -> None:
    import cv2

    cv2.putText(
        frame,
        text,
        (18, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        color,
        2,
        cv2.LINE_AA,
    )


def _draw_map_inset(
    frame: np.ndarray,
    *,
    route: PctRoute,
    trail: list[tuple[float, float]],
    target_idx: int,
) -> None:
    import cv2

    h, w = frame.shape[:2]
    x0, y0 = w - 320, 28
    x1, y1 = w - 22, 248
    cv2.rectangle(frame, (x0, y0), (x1, y1), (18, 24, 30), -1)
    cv2.rectangle(frame, (x0, y0), (x1, y1), (210, 220, 230), 1)

    path = [(float(p[0]), float(p[1])) for p in route.path]
    points = [(route.start[0], route.start[1]), (route.goal[0], route.goal[1]), *path, *trail]
    min_x = min(p[0] for p in points) - 0.4
    max_x = max(p[0] for p in points) + 0.4
    min_y = min(p[1] for p in points) - 0.4
    max_y = max(p[1] for p in points) + 0.4
    scale = min((x1 - x0 - 34) / max(max_x - min_x, 0.1), (y1 - y0 - 38) / max(max_y - min_y, 0.1))

    def project(point: tuple[float, float]) -> tuple[int, int]:
        px = x0 + 17 + int((point[0] - min_x) * scale)
        py = y1 - 18 - int((point[1] - min_y) * scale)
        return px, py

    if len(path) >= 2:
        cv2.polylines(frame, [np.asarray([project(p) for p in path], dtype=np.int32)], False, (45, 230, 95), 2)
        for idx, point in enumerate(path):
            radius = 5 if idx == target_idx else 3
            color = (255, 210, 0) if idx == target_idx else (120, 180, 120)
            cv2.circle(frame, project(point), radius, color, -1)
    if len(trail) >= 2:
        cv2.polylines(frame, [np.asarray([project(p) for p in trail], dtype=np.int32)], False, (0, 210, 255), 2)
    cv2.circle(frame, project((route.start[0], route.start[1])), 5, (40, 220, 40), -1)
    cv2.drawMarker(frame, project((route.goal[0], route.goal[1])), (70, 70, 255), cv2.MARKER_STAR, 15, 2)
    if trail:
        cv2.circle(frame, project(trail[-1]), 6, (255, 235, 0), -1)
    cv2.putText(frame, "PCT path + MuJoCo trail", (x0 + 12, y0 + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1)


def _resize_panel(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    import cv2

    if frame.shape[1] == width and frame.shape[0] == height:
        return frame.copy()
    return cv2.resize(frame, (int(width), int(height)), interpolation=cv2.INTER_AREA)


def _draw_panel_title(frame: np.ndarray, title: str, subtitle: str = "") -> None:
    import cv2

    h, w = frame.shape[:2]
    cv2.rectangle(frame, (0, 0), (w, 42 if subtitle else 30), (245, 247, 250), -1)
    cv2.putText(frame, title, (12, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (15, 45, 80), 2, cv2.LINE_AA)
    if subtitle:
        cv2.putText(frame, subtitle, (12, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (55, 78, 95), 1, cv2.LINE_AA)


def _local_projector(
    width: int,
    height: int,
    *,
    x_min: float = -1.0,
    x_max: float = 7.5,
    y_min: float = -3.8,
    y_max: float = 3.8,
) -> Any:
    def project(x: float, y: float) -> tuple[int, int]:
        px = int((float(y) - y_min) / max(y_max - y_min, 1e-6) * width)
        py = int(height - (float(x) - x_min) / max(x_max - x_min, 1e-6) * height)
        return px, py

    return project


def _draw_robot_local_panel(
    *,
    state: Any,
    width: int,
    height: int,
    route: PctRoute,
    local_path: list[list[float]],
    local_path_frame_id: str,
    fallback_path: list[list[float]],
    lidar_points: np.ndarray,
    obstacle_points: list[tuple[float, float, float, float]],
    target_idx: int,
) -> np.ndarray:
    import cv2

    frame = np.full((height, width, 3), (22, 31, 41), dtype=np.uint8)
    project = _local_projector(width, height)

    for gx in np.arange(0.0, 7.6, 1.0):
        cv2.line(frame, project(float(gx), -3.8), project(float(gx), 3.8), (44, 57, 68), 1, cv2.LINE_AA)
    for gy in np.arange(-3.0, 3.1, 1.0):
        cv2.line(frame, project(-1.0, float(gy)), project(7.5, float(gy)), (44, 57, 68), 1, cv2.LINE_AA)
    cv2.line(frame, project(0.0, 0.0), project(7.2, -2.3), (58, 86, 102), 1, cv2.LINE_AA)
    cv2.line(frame, project(0.0, 0.0), project(7.2, 2.3), (58, 86, 102), 1, cv2.LINE_AA)

    obstacle_arr = np.asarray(obstacle_points, dtype=np.float64)
    if obstacle_arr.size:
        local_obs = _world_xy_to_robot_xy(obstacle_arr[:, :2], state)
        mask = (
            (local_obs[:, 0] >= -1.0)
            & (local_obs[:, 0] <= 7.5)
            & (local_obs[:, 1] >= -3.8)
            & (local_obs[:, 1] <= 3.8)
        )
        for x, y in local_obs[mask][:: max(1, int(len(local_obs[mask]) / 450) or 1)]:
            cv2.circle(frame, project(float(x), float(y)), 1, (90, 150, 230), -1)

    lidar_arr = np.asarray(lidar_points, dtype=np.float64)
    if lidar_arr.size:
        local_lidar = _world_xy_to_robot_xy(lidar_arr[:, :2], state)
        mask = (
            (local_lidar[:, 0] >= -1.0)
            & (local_lidar[:, 0] <= 7.5)
            & (local_lidar[:, 1] >= -3.8)
            & (local_lidar[:, 1] <= 3.8)
        )
        for x, y in local_lidar[mask][:: max(1, int(len(local_lidar[mask]) / 300) or 1)]:
            cv2.circle(frame, project(float(x), float(y)), 2, (80, 220, 170), -1)

    if len(fallback_path) >= 2:
        guide = _world_xy_to_robot_xy(np.asarray(fallback_path, dtype=np.float64)[:, :2], state)
        guide_pts: list[tuple[int, int]] = []
        for x, y in guide:
            if -1.0 <= x <= 7.5 and -3.8 <= y <= 3.8:
                guide_pts.append(project(float(x), float(y)))
        if len(guide_pts) >= 2:
            cv2.polylines(frame, [np.asarray(guide_pts, dtype=np.int32)], False, (0, 220, 255), 2, cv2.LINE_AA)

    if len(local_path) >= 2:
        if _path_is_robot_frame(local_path_frame_id):
            local = np.asarray(local_path, dtype=np.float64)[:, :2]
        else:
            local = _world_xy_to_robot_xy(np.asarray(local_path, dtype=np.float64)[:, :2], state)
        pts: list[tuple[int, int]] = []
        for x, y in local:
            if -1.0 <= x <= 7.5 and -3.8 <= y <= 3.8:
                pts.append(project(float(x), float(y)))
        if len(pts) >= 2:
            cv2.polylines(frame, [np.asarray(pts, dtype=np.int32)], False, (255, 230, 0), 3, cv2.LINE_AA)
        for pt in pts[:12]:
            cv2.circle(frame, pt, 3, (255, 245, 80), -1, cv2.LINE_AA)

    body = np.asarray(
        [project(0.42, 0.0), project(-0.28, -0.22), project(-0.18, 0.0), project(-0.28, 0.22)],
        dtype=np.int32,
    )
    cv2.circle(frame, project(0.0, 0.0), 16, (50, 125, 80), 2, cv2.LINE_AA)
    cv2.fillPoly(frame, [body], (42, 185, 105))
    cv2.polylines(frame, [body], True, (220, 255, 230), 1, cv2.LINE_AA)
    _draw_panel_title(
        frame,
        "Robot local planning view",
        f"yellow=native /path, cyan=global lookahead, green=lidar, blue=obstacles; target={target_idx}",
    )
    return frame


def _draw_front_camera_panel(
    *,
    engine: Any,
    state: Any,
    width: int,
    height: int,
    route: PctRoute,
    local_path: list[list[float]],
    local_path_frame_id: str,
    fallback_path: list[list[float]],
    camera_name: str,
) -> np.ndarray:
    import cv2

    cam = engine.get_camera_data(camera_name)
    if cam is None or cam.rgb is None:
        frame = np.full((height, width, 3), (16, 24, 32), dtype=np.uint8)
        _draw_panel_title(frame, "Front camera RGB", f"camera {camera_name!r} unavailable")
        return frame

    frame = _resize_panel(np.asarray(cam.rgb, dtype=np.uint8), width, height)
    scale_x = width / max(float(cam.rgb.shape[1]), 1.0)
    scale_y = height / max(float(cam.rgb.shape[0]), 1.0)
    fx, fy, cx, cy = [float(v) for v in cam.intrinsics]
    fx *= scale_x
    fy *= scale_y
    cx *= scale_x
    cy *= scale_y

    def draw_projected_path(
        points: list[list[float]],
        *,
        robot_frame: bool,
        color: tuple[int, int, int],
        thickness: int,
        max_depth_m: float,
        lift_m: float,
    ) -> None:
        if len(points) < 2:
            return
        pts = np.asarray(points, dtype=np.float64)
        local_xy = pts[:, :2] if robot_frame else _world_xy_to_robot_xy(pts[:, :2], state)
        z_values = pts[:, 2] if pts.shape[1] >= 3 else np.zeros(len(pts), dtype=np.float64)
        cam_x = 0.35
        cam_z = 0.15
        pixels: list[tuple[int, int]] = []
        for idx, (x_forward, y_left) in enumerate(local_xy):
            depth = float(x_forward) - cam_x
            if depth <= 0.20 or depth > max_depth_m:
                continue
            z_rel = float(z_values[idx]) - float(state.position[2]) - cam_z + float(lift_m)
            u = cx - float(y_left) * fx / depth
            v = cy - z_rel * fy / depth
            if -20 <= u <= width + 20 and -20 <= v <= height + 20:
                pixels.append((int(round(u)), int(round(v))))
        if len(pixels) >= 2:
            cv2.polylines(frame, [np.asarray(pixels, dtype=np.int32)], False, color, thickness, cv2.LINE_AA)
            cv2.circle(frame, pixels[min(6, len(pixels) - 1)], 7, color, 2, cv2.LINE_AA)

    draw_projected_path(
        fallback_path,
        robot_frame=False,
        color=(0, 220, 255),
        thickness=3,
        max_depth_m=22.0,
        lift_m=0.28,
    )
    draw_projected_path(
        local_path,
        robot_frame=_path_is_robot_frame(local_path_frame_id),
        color=(255, 230, 0),
        thickness=4,
        max_depth_m=12.0,
        lift_m=0.34,
    )

    goal_local = _world_xy_to_robot_xy(np.asarray([route.goal[:2]], dtype=np.float64), state)[0]
    if goal_local[0] > 0.2:
        u = cx - float(goal_local[1]) * fx / max(float(goal_local[0]), 0.2)
        v = cy
        if -20 <= u <= width + 20:
            cv2.drawMarker(frame, (int(u), int(v)), (255, 80, 255), cv2.MARKER_STAR, 18, 2)
    _draw_panel_title(frame, "Front camera RGB + projected path", "cyan=global lookahead; yellow=native local /path; magenta=goal")
    return frame


def _draw_cloud_map_panel(
    *,
    width: int,
    height: int,
    route: PctRoute,
    trail: list[tuple[float, float]],
    lidar_points: np.ndarray,
    obstacle_points: list[tuple[float, float, float, float]],
    local_path: list[list[float]],
    local_path_frame_id: str,
    state: Any,
    target_idx: int,
) -> np.ndarray:
    import cv2

    frame = np.full((height, width, 3), (14, 23, 33), dtype=np.uint8)
    path = [(float(p[0]), float(p[1])) for p in route.path]
    extra: list[tuple[float, float]] = []
    if local_path:
        lp_arr = np.asarray(local_path, dtype=np.float64)[:, :2]
        if _path_is_robot_frame(local_path_frame_id):
            lp_arr = _robot_xy_to_world_xy(lp_arr, state)
        extra = [(float(p[0]), float(p[1])) for p in lp_arr]
    pts2 = [*path, *trail, *extra, (route.start[0], route.start[1]), (route.goal[0], route.goal[1])]
    if obstacle_points:
        obs_arr = np.asarray(obstacle_points, dtype=np.float64)
        pts2.extend((float(x), float(y)) for x, y in obs_arr[:: max(1, int(len(obs_arr) / 500) or 1), :2])
    if len(pts2) < 2:
        return frame
    min_x = min(p[0] for p in pts2) - 0.8
    max_x = max(p[0] for p in pts2) + 0.8
    min_y = min(p[1] for p in pts2) - 0.8
    max_y = max(p[1] for p in pts2) + 0.8
    scale = min(width / max(max_x - min_x, 0.1), height / max(max_y - min_y, 0.1))

    def project(point: tuple[float, float]) -> tuple[int, int]:
        px = int((point[0] - min_x) * scale)
        py = int(height - (point[1] - min_y) * scale)
        return px, py

    obs_arr = np.asarray(obstacle_points, dtype=np.float64)
    if obs_arr.size:
        for x, y in obs_arr[:: max(1, int(len(obs_arr) / 900) or 1), :2]:
            cv2.circle(frame, project((float(x), float(y))), 1, (90, 150, 230), -1)
    lidar_arr = np.asarray(lidar_points, dtype=np.float64)
    if lidar_arr.size:
        for x, y in lidar_arr[:: max(1, int(len(lidar_arr) / 450) or 1), :2]:
            cv2.circle(frame, project((float(x), float(y))), 2, (80, 220, 170), -1)
    if len(path) >= 2:
        cv2.polylines(frame, [np.asarray([project(p) for p in path], dtype=np.int32)], False, (50, 230, 95), 2)
    if local_path:
        lp_arr = np.asarray(local_path, dtype=np.float64)[:, :2]
        if _path_is_robot_frame(local_path_frame_id):
            lp_arr = _robot_xy_to_world_xy(lp_arr, state)
        lp = [(float(p[0]), float(p[1])) for p in lp_arr]
        cv2.polylines(frame, [np.asarray([project(p) for p in lp], dtype=np.int32)], False, (255, 230, 0), 3)
    if len(trail) >= 2:
        cv2.polylines(frame, [np.asarray([project(p) for p in trail], dtype=np.int32)], False, (255, 220, 0), 2)
    cv2.circle(frame, project((route.start[0], route.start[1])), 5, (40, 220, 40), -1)
    cv2.drawMarker(frame, project((route.goal[0], route.goal[1])), (70, 70, 255), cv2.MARKER_STAR, 16, 2)
    if trail:
        cv2.circle(frame, project(trail[-1]), 6, (255, 235, 0), -1)
    _draw_panel_title(frame, "Live point cloud / map view", f"target={target_idx}; green=global path, yellow=native /path")
    return frame


def _make_observer_renderer(
    *,
    engine: Any,
    route: PctRoute,
    width: int,
    height: int,
) -> tuple[Any, Any]:
    import mujoco

    if engine.model is None or engine.data is None:
        raise RuntimeError("MuJoCo engine is not ready for video rendering")
    engine.model.vis.global_.offwidth = max(int(engine.model.vis.global_.offwidth), int(width))
    engine.model.vis.global_.offheight = max(int(engine.model.vis.global_.offheight), int(height))
    renderer = mujoco.Renderer(engine.model, height=int(height), width=int(width))
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE

    route_points = [(float(p[0]), float(p[1])) for p in route.path] + [
        (float(route.start[0]), float(route.start[1])),
        (float(route.goal[0]), float(route.goal[1])),
    ]
    xs = [p[0] for p in route_points]
    ys = [p[1] for p in route_points]
    center_x = (min(xs) + max(xs)) * 0.5
    center_y = (min(ys) + max(ys)) * 0.5
    extent = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
    camera.lookat[0] = center_x
    camera.lookat[1] = center_y
    camera.lookat[2] = 0.42
    camera.distance = max(4.0, extent * 2.3)
    camera.azimuth = 135.0
    camera.elevation = -55.0

    return renderer, camera


def _make_video_writer(
    *,
    engine: Any,
    route: PctRoute,
    output: Path,
    width: int,
    height: int,
    fps: float,
) -> tuple[Any, Any, Any]:
    import cv2

    renderer, camera = _make_observer_renderer(engine=engine, route=route, width=width, height=height)
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), float(fps), (int(width), int(height)))
    if not writer.isOpened():
        renderer.close()
        raise RuntimeError(f"failed to open video writer: {output}")
    return renderer, camera, writer


def _next_scene_geom(renderer: Any) -> Any | None:
    scene = getattr(renderer, "scene", None)
    if scene is None:
        return None
    if int(scene.ngeom) >= int(scene.maxgeom):
        return None
    geom = scene.geoms[int(scene.ngeom)]
    scene.ngeom += 1
    return geom


def _add_scene_sphere(
    renderer: Any,
    *,
    pos: tuple[float, float, float] | list[float] | np.ndarray,
    radius: float,
    rgba: tuple[float, float, float, float],
) -> None:
    import mujoco

    geom = _next_scene_geom(renderer)
    if geom is None:
        return
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.asarray([radius, radius, radius], dtype=np.float64),
        np.asarray(pos, dtype=np.float64),
        np.eye(3, dtype=np.float64).reshape(-1),
        np.asarray(rgba, dtype=np.float32),
    )


def _add_scene_capsule(
    renderer: Any,
    *,
    start: tuple[float, float, float] | list[float] | np.ndarray,
    end: tuple[float, float, float] | list[float] | np.ndarray,
    radius: float,
    rgba: tuple[float, float, float, float],
) -> None:
    import mujoco

    p0 = np.asarray(start, dtype=np.float64)
    p1 = np.asarray(end, dtype=np.float64)
    if not np.isfinite(p0).all() or not np.isfinite(p1).all():
        return
    if float(np.linalg.norm(p1 - p0)) < 1e-4:
        return
    geom = _next_scene_geom(renderer)
    if geom is None:
        return
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        np.asarray([radius, 0.0, 0.0], dtype=np.float64),
        np.zeros(3, dtype=np.float64),
        np.eye(3, dtype=np.float64).reshape(-1),
        np.asarray(rgba, dtype=np.float32),
    )
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, float(radius), p0, p1)
    geom.rgba[:] = np.asarray(rgba, dtype=np.float32)


def _overlay_scene_polyline(
    renderer: Any,
    points: list[list[float]] | np.ndarray,
    *,
    z_lift: float,
    radius: float,
    rgba: tuple[float, float, float, float],
    max_segments: int = 240,
) -> None:
    pts = np.asarray(points, dtype=np.float64)
    if pts.size == 0:
        return
    pts = pts.reshape((-1, pts.shape[-1]))
    if pts.shape[1] < 2 or len(pts) < 2:
        return
    pts3 = np.zeros((len(pts), 3), dtype=np.float64)
    pts3[:, :2] = pts[:, :2]
    pts3[:, 2] = (pts[:, 2] if pts.shape[1] >= 3 else 0.0) + float(z_lift)
    step = max(1, int(math.ceil((len(pts3) - 1) / max(float(max_segments), 1.0))))
    for idx in range(0, len(pts3) - 1, step):
        _add_scene_capsule(renderer, start=pts3[idx], end=pts3[min(idx + step, len(pts3) - 1)], radius=radius, rgba=rgba)


def _overlay_scene_points(
    renderer: Any,
    points: np.ndarray,
    *,
    point_limit: int,
    radius: float,
    rgba: tuple[float, float, float, float],
) -> None:
    pts = np.asarray(points, dtype=np.float64)
    if pts.size == 0:
        return
    pts = pts.reshape((-1, pts.shape[-1]))
    if pts.shape[1] < 3:
        return
    limit = max(0, int(point_limit))
    if limit <= 0:
        return
    step = max(1, int(math.ceil(len(pts) / float(limit))))
    for point in pts[::step][:limit, :3]:
        if not np.isfinite(point).all():
            continue
        _add_scene_sphere(renderer, pos=point, radius=radius, rgba=rgba)


def _write_video_frame(
    *,
    renderer: Any,
    camera: Any,
    writer: Any,
    engine: Any,
    route: PctRoute,
    trail: list[tuple[float, float]],
    target_idx: int,
    elapsed_s: float,
    distance_m: float,
    latest_cmd: dict[str, float],
    path_count: int,
) -> None:
    renderer.update_scene(engine.data, camera)
    frame = renderer.render().copy()
    _put_text(frame, "LingTu native PCT navigation gate", 34, (255, 255, 255))
    _put_text(frame, "PCT native -> ROS2 localPlanner/pathFollower -> MuJoCo kinematic", 66, (0, 255, 255))
    _put_text(
        frame,
        f"t={elapsed_s:.1f}s  dist={distance_m:.2f}m  target={target_idx}/{len(route.path)-1}  /path={path_count}",
        98,
        (230, 230, 230),
    )
    _put_text(
        frame,
        f"cmd=({latest_cmd['linear_x']:.2f},{latest_cmd['linear_y']:.2f},{latest_cmd['angular_z']:.2f})  hardware=false",
        130,
        (230, 230, 230),
    )
    _draw_map_inset(frame, route=route, trail=trail, target_idx=target_idx)
    writer.write(frame[:, :, ::-1])


def _write_scene_overlay_video_frame(
    *,
    renderer: Any,
    camera: Any,
    writer: Any,
    engine: Any,
    state: Any,
    route: PctRoute,
    trail: list[tuple[float, float]],
    target_idx: int,
    elapsed_s: float,
    distance_m: float,
    latest_cmd: dict[str, float],
    path_count: int,
    latest_local_path: list[list[float]],
    latest_local_path_frame_id: str,
    lidar_points: np.ndarray,
    width: int,
    height: int,
    front_path_lookahead_m: float,
    point_limit: int,
) -> None:
    import cv2

    renderer.update_scene(engine.data, camera)

    full_route = [[float(p[0]), float(p[1]), 0.06] for p in route.path]
    lookahead = _path_window_by_distance(
        route.path,
        start_idx=max(0, target_idx - 1),
        lookahead_m=front_path_lookahead_m,
    )
    lookahead = [[float(p[0]), float(p[1]), 0.13] for p in lookahead]
    if latest_local_path:
        local_arr = np.asarray(latest_local_path, dtype=np.float64)
        if _path_is_robot_frame(latest_local_path_frame_id):
            local_xy = _robot_xy_to_world_xy(local_arr[:, :2], state)
        else:
            local_xy = local_arr[:, :2]
        local_world = np.column_stack(
            [
                local_xy[:, 0],
                local_xy[:, 1],
                np.full(len(local_xy), 0.20, dtype=np.float64),
            ]
        )
    else:
        local_world = np.zeros((0, 3), dtype=np.float64)

    _overlay_scene_polyline(renderer, full_route, z_lift=0.0, radius=0.025, rgba=(0.05, 0.75, 1.0, 0.42), max_segments=260)
    _overlay_scene_polyline(renderer, lookahead, z_lift=0.0, radius=0.050, rgba=(0.0, 0.95, 1.0, 0.86), max_segments=100)
    _overlay_scene_polyline(renderer, local_world, z_lift=0.0, radius=0.060, rgba=(1.0, 0.88, 0.02, 0.96), max_segments=120)
    if len(trail) >= 2:
        trail_world = [[float(x), float(y), 0.24] for x, y in trail]
        _overlay_scene_polyline(renderer, trail_world, z_lift=0.0, radius=0.040, rgba=(0.25, 1.0, 0.35, 0.88), max_segments=260)
    _overlay_scene_points(renderer, lidar_points, point_limit=point_limit, radius=0.030, rgba=(0.15, 1.0, 0.72, 0.74))
    _add_scene_sphere(renderer, pos=(float(route.goal[0]), float(route.goal[1]), 0.36), radius=0.16, rgba=(1.0, 0.10, 1.0, 0.92))
    if 0 <= target_idx < len(route.path):
        target = route.path[target_idx]
        _add_scene_sphere(renderer, pos=(float(target[0]), float(target[1]), 0.30), radius=0.12, rgba=(1.0, 0.78, 0.0, 0.92))

    frame = renderer.render().copy()
    _put_text(frame, "LingTu navigation scene overlay", 36, (255, 255, 255))
    _put_text(frame, "cyan=global/preview  yellow=native local /path  green=actual trail  mint=LiDAR points", 68, (0, 255, 255))
    _put_text(
        frame,
        f"t={elapsed_s:.1f}s  dist={distance_m:.2f}m  target={target_idx}/{len(route.path)-1}  /path={path_count}",
        100,
        (230, 230, 230),
    )
    _put_text(
        frame,
        f"cmd=({latest_cmd['linear_x']:.2f},{latest_cmd['linear_y']:.2f},{latest_cmd['angular_z']:.2f})  hardware=false",
        132,
        (230, 230, 230),
    )
    legend_x = 18
    legend_y = height - 116
    cv2.rectangle(frame, (legend_x - 6, legend_y - 26), (legend_x + 515, legend_y + 86), (12, 18, 24), -1)
    cv2.putText(frame, "single-scene overlay", (legend_x, legend_y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, "global preview + native local path + point cloud are drawn in the MuJoCo scene", (legend_x, legend_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (220, 235, 245), 1, cv2.LINE_AA)
    cv2.putText(frame, f"LiDAR overlay points={min(int(point_limit), int(len(lidar_points)))}", (legend_x, legend_y + 58), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (175, 255, 220), 1, cv2.LINE_AA)
    writer.write(frame[:, :, ::-1])


def _write_evidence_video_frame(
    *,
    renderer: Any,
    camera: Any,
    writer: Any,
    engine: Any,
    state: Any,
    route: PctRoute,
    trail: list[tuple[float, float]],
    target_idx: int,
    elapsed_s: float,
    distance_m: float,
    latest_cmd: dict[str, float],
    path_count: int,
    latest_local_path: list[list[float]],
    latest_local_path_frame_id: str,
    obstacle_points: list[tuple[float, float, float, float]],
    lidar_points: np.ndarray,
    camera_name: str,
    width: int,
    height: int,
    front_path_lookahead_m: float,
) -> None:
    import cv2

    half_w = int(width) // 2
    half_h = int(height) // 2
    fallback_path = _path_window_by_distance(
        route.path,
        start_idx=max(0, target_idx - 1),
        lookahead_m=front_path_lookahead_m,
    )

    front = _draw_front_camera_panel(
        engine=engine,
        state=state,
        width=half_w,
        height=half_h,
        route=route,
        local_path=latest_local_path,
        local_path_frame_id=latest_local_path_frame_id,
        fallback_path=fallback_path,
        camera_name=camera_name,
    )
    local = _draw_robot_local_panel(
        state=state,
        width=half_w,
        height=half_h,
        route=route,
        local_path=latest_local_path,
        local_path_frame_id=latest_local_path_frame_id,
        fallback_path=fallback_path,
        lidar_points=lidar_points,
        obstacle_points=obstacle_points,
        target_idx=target_idx,
    )
    renderer.update_scene(engine.data, camera)
    observer = _resize_panel(renderer.render().copy(), half_w, half_h)
    _put_text(observer, "MuJoCo observer", 28, (255, 255, 255))
    _put_text(observer, f"t={elapsed_s:.1f}s dist={distance_m:.2f}m cmd=({latest_cmd['linear_x']:.2f},{latest_cmd['linear_y']:.2f},{latest_cmd['angular_z']:.2f})", 58, (0, 255, 255))
    _draw_map_inset(observer, route=route, trail=trail, target_idx=target_idx)

    cloud = _draw_cloud_map_panel(
        width=half_w,
        height=half_h,
        route=route,
        trail=trail,
        lidar_points=lidar_points,
        obstacle_points=obstacle_points,
        local_path=latest_local_path,
        local_path_frame_id=latest_local_path_frame_id,
        state=state,
        target_idx=target_idx,
    )
    cv2.putText(
        cloud,
        f"/path frames={path_count} lidar_points={len(lidar_points)} hardware=false",
        (12, half_h - 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (225, 235, 245),
        1,
        cv2.LINE_AA,
    )

    top = np.hstack([front, local])
    bottom = np.hstack([observer, cloud])
    frame = np.vstack([top, bottom])
    writer.write(frame[:, :, ::-1])


def _stamp(node: Any, msg_or_header: Any) -> Any:
    msg_or_header.stamp = node.get_clock().now().to_msg()
    return msg_or_header


def run_gate(args: argparse.Namespace) -> dict[str, Any]:
    if getattr(args, "contract_only", False):
        return _contract_only_report(args)

    if args.video_out and not os.environ.get("MUJOCO_GL"):
        os.environ["MUJOCO_GL"] = "egl"
        os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

    if args.generate_source_report or args.force_generate_source_report:
        _generate_source_report(args)

    planner_name = str(getattr(args, "planner", "pct")).lower().strip()
    route = _load_pct_route(args.source_report, route=args.route, planner=planner_name)
    source_contract = _planner_contract(route, planner_name)
    map_artifacts = _source_map_artifacts(route)

    report: dict[str, Any] = {
        "schema_version": "lingtu.native_pct_mujoco_gate.v1",
        "ok": False,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "driver_used": False,
        "gateway_used": False,
        "physical_gait_verified": False,
        "slam_verified": False,
        "real_lidar_verified": False,
        "backend": {
            "global_planner": "pct_native" if planner_name == "pct" else planner_name,
            "local_planner": "cmu_ros2_native/localPlanner",
            "path_follower": "cmu_ros2_native/pathFollower",
            "sim_driver": "MuJoCoEngine(kinematic)",
        },
        "planning_chain": {
            "global_planner": "source_report/native_pct_tomogram",
            "local_planner": "cmu_ros2_native/localPlanner",
            "path_follower": "cmu_ros2_native/pathFollower",
            "motion_executor": "MuJoCoEngine(kinematic)",
            "fallback_allowed": False,
        },
        "frames": {
            "goal": "map",
            "global_path": "map",
            "local_planner_odometry": "map",
            "local_planner_cloud_map": "map",
            "local_planner_added_obstacles": "map",
            "path_follower_path": "map",
            "cmd_vel": "base_link",
            "sim_odometry": "map",
        },
        "render_backend": os.environ.get("MUJOCO_GL", ""),
        "route": route.route,
        "sim_vehicle": str(args.sim_vehicle),
        "drive_adapter": "omni_local_path_tracker" if str(args.sim_vehicle) == "omni_cart" else "native_pathFollower_cmd_vel",
        "source_report": str(route.source_report),
        "scene_xml": str(route.scene_xml),
        "ros_domain_id": str(args.ros_domain_id),
        "planner": planner_name,
        "primary_planner": source_contract["primary_planner"],
        "selected_planner": source_contract["selected_planner"],
        "fallback_used": source_contract["fallback_used"],
        "global_planner_source": "source_report/native_pct_tomogram",
        "source_planning_contract": source_contract,
        "source_tomogram": source_contract["tomogram"],
        "source_tomogram_sha256": source_contract["tomogram_sha256"],
        "map_artifacts": map_artifacts,
        "deliverable_contract": {
            "checks": {
                "same_source_map_artifact": map_artifacts.get("ok") is True,
            }
        },
        "pct_backend_class": route.plan.get("backend_class"),
        "pct_native_backend_used": bool(route.plan.get("native_backend_used")) if planner_name == "pct" else False,
        "pct_runtime_ok": bool((route.plan.get("native_runtime") or {}).get("ok", True)),
        "pct_path_count": len(route.path),
        "pct_optimizer_enabled": source_contract["pct_optimizer_enabled"],
        "pct_optimizer_attempted": source_contract["pct_optimizer_attempted"],
        "pct_optimizer_accepted": source_contract["pct_optimizer_accepted"],
        "pct_optimizer_reject_reason": source_contract["pct_optimizer_reject_reason"],
        "pct_optimizer_blocked_sample_count": source_contract[
            "pct_optimizer_blocked_sample_count"
        ],
        "pct_planner_path_mode": source_contract["pct_planner_path_mode"],
        "pct_plan_ms": route.plan.get("plan_ms"),
        "pct_path": route.path,
        "start_xy": route.start[:2],
        "requested_goal_xy": route.goal[:2],
        "pct_path_goal_xy": route.path[-1][:2] if route.path else [],
        "tracking_goal_xy": [],
        "final_goal_xy": route.path[-1][:2] if route.path else route.goal[:2],
        "validation_limitations": [
            "MuJoCo kinematic base only; policy gait and contact stability are not verified by this gate.",
            "Fast-LIO2 live localization is not part of this gate; run the Fast-LIO2 MuJoCo/live gate separately.",
            "No real robot driver, Gateway command path, CmdVelMux hardware output, or systemd service is started.",
        ],
    }
    report["backend"]["sim_driver"] = (
        "MuJoCoEngine(kinematic omni_cart)"
        if str(args.sim_vehicle) == "omni_cart"
        else "MuJoCoEngine(kinematic quadruped)"
    )

    artifact_dir = args.json_out.parent if args.json_out else args.artifact_dir
    artifact_dir.mkdir(parents=True, exist_ok=True)
    source_artifact_blockers = _source_map_artifact_blockers(map_artifacts)
    if source_artifact_blockers:
        report["native_gate_skipped"] = True
        report["claim_boundary"] = "pre_native_source_artifact_contract_failed"
        report["blockers"] = source_artifact_blockers
        report["remaining_gaps"] = source_artifact_blockers
        _write_report_if_requested(args, report)
        return report

    path_folder = args.path_folder or _default_local_planner_path_folder()
    obstacle_aware = not args.disable_obstacle_check
    local_planner_cmd, path_follower_cmd = _native_node_commands(
        path_folder=path_folder,
        max_speed=args.max_speed,
        autonomy_speed=args.autonomy_speed,
        goal_clear_range=args.goal_clear_range,
        near_field_stop_distance=args.near_field_stop_distance,
        lookahead=args.lookahead,
        stop_distance=args.stop_distance,
        obstacle_aware=obstacle_aware,
        check_rot_obstacle=args.check_rot_obstacle,
    )
    report["native_node_commands"] = {
        "localPlanner": local_planner_cmd,
        "pathFollower": path_follower_cmd,
    }
    report["command_generation"] = _command_generation_report(
        args,
        route=route,
        path_folder=path_folder,
    )
    obstacle_boxes = _obstacle_boxes(route)
    obstacle_names = [str(box.get("name") or "") for box in obstacle_boxes]
    report["obstacle_aware"] = {
        "enabled": bool(obstacle_aware),
        "check_rot_obstacle": bool(args.check_rot_obstacle),
        "metadata_points": 0,
        "metadata_obstacle_count": len(obstacle_boxes),
        "obstacles": obstacle_names,
        "source": "metadata.added_obstacles" if obstacle_boxes else "",
    }
    report["moving_obstacles"] = {
        "enabled": str(args.moving_obstacle_mode) != "none",
        "mode": str(args.moving_obstacle_mode),
        "route_ratio": float(args.moving_obstacle_route_ratio),
        "count": int(args.moving_obstacle_count),
        "route_ratio_step": float(args.moving_obstacle_route_ratio_step),
        "lateral_phase_step_rad": float(args.moving_obstacle_lateral_phase_step_rad),
        "start_s": float(args.moving_obstacle_start_s),
        "duration_s": float(args.moving_obstacle_duration_s),
        "period_s": float(args.moving_obstacle_period_s),
        "radius_m": float(args.moving_obstacle_radius_m),
        "lateral_amplitude_m": float(args.moving_obstacle_lateral_amplitude_m),
        "along_amplitude_m": float(args.moving_obstacle_along_amplitude_m),
        "published_update_count": 0,
        "published_point_count_max": 0,
    }
    follow_path, waypoint_filter = _build_safe_follow_path(
        route,
        enabled=obstacle_aware and not args.disable_waypoint_safety_filter,
        clearance=args.robot_radius + args.waypoint_safety_margin,
        extra_margin=args.waypoint_detour_margin,
        sample_step_m=args.waypoint_collision_sample_step,
    )
    control_route = replace(route, path=follow_path)
    report["waypoint_safety_filter"] = waypoint_filter
    report["follow_path"] = follow_path
    report["tracking_goal_xy"] = follow_path[-1][:2] if follow_path else []
    report["final_goal_xy"] = report["tracking_goal_xy"]
    if waypoint_filter.get("enabled") is True and waypoint_filter.get("goal_preserved") is not True:
        blocker = "waypoint_safety_filter.goal_preserved is not true"
        report["native_gate_skipped"] = True
        report["claim_boundary"] = "waypoint_safety_filter_truncated_pct_path"
        report["blockers"] = [blocker]
        report["remaining_gaps"] = [blocker]
        _write_report_if_requested(args, report)
        return report

    try:
        mods = _load_ros_modules()
    except Exception as exc:
        blocker = f"ROS2 runtime unavailable for native local planner gate: {exc}"
        report["native_gate_skipped"] = True
        report["claim_boundary"] = "ros2_runtime_unavailable"
        report["blockers"] = [blocker]
        report["remaining_gaps"] = [blocker]
        report["environment"] = {
            "ros_domain_id": str(args.ros_domain_id),
            "ros2_executable": shutil.which("ros2") or "",
            "ros_distro": os.environ.get("ROS_DISTRO", ""),
            "diagnostic_commands": [
                "source /opt/ros/humble/setup.bash",
                "source install/setup.bash",
                "ros2 pkg executables local_planner",
            ],
        }
        _write_report_if_requested(args, report)
        return report

    from sim.engine.core.engine import VelocityCommand
    from sim.scripts.mujoco_fastlio2_live_gate import _build_engine, _resolve_mid360_pattern

    rclpy = mods["rclpy"]
    Odometry = mods["Odometry"]
    PointStamped = mods["PointStamped"]
    PointField = mods["PointField"]
    RosPath = mods["RosPath"]
    TwistStamped = mods["TwistStamped"]
    Float32 = mods["Float32"]
    Header = mods["Header"]
    Int8 = mods["Int8"]
    String = mods["String"]
    point_cloud2 = mods["point_cloud2"]
    obstacle_points, obstacle_names = _obstacle_points_from_metadata(
        route,
        spacing=args.obstacle_point_spacing,
        intensity=args.obstacle_intensity,
    )
    report["obstacle_aware"].update(
        {
            "metadata_points": len(obstacle_points),
            "obstacles": obstacle_names,
            "source": "metadata.added_obstacles" if obstacle_points else "",
        }
    )

    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    env["PYTHONPATH"] = f"{SRC}{os.pathsep}{ROOT}{os.pathsep}" + env.get("PYTHONPATH", "")

    local_planner_log = artifact_dir / "localPlanner.log"
    path_follower_log = artifact_dir / "pathFollower.log"

    procs: list[subprocess.Popen[bytes]] = []
    engine = None
    node = None
    video_renderer = None
    video_writer = None
    video_camera = None
    video_frame_count = 0
    rclpy_started = False
    lp_log_fh = None
    pf_log_fh = None
    try:
        rclpy.init(args=None)
        rclpy_started = True
        node = rclpy.create_node("native_pct_mujoco_gate")

        counts = _cmd_vel_subscriber_counts(rclpy, node, spin_s=args.preflight_spin_s)
        report["preflight_cmd_vel_subscribers"] = counts
        if not args.allow_existing_cmd_vel_subscribers and any(counts.values()):
            raise RuntimeError(
                "refusing to launch pathFollower because this ROS domain already has "
                f"cmd_vel subscribers: {counts}"
            )

        lp_log_fh = local_planner_log.open("wb")
        pf_log_fh = path_follower_log.open("wb")
        procs.append(
            _start_native_node(
                local_planner_cmd,
                env=env,
                log_fh=lp_log_fh,
            )
        )
        procs.append(
            _start_native_node(
                path_follower_cmd,
                env=env,
                log_fh=pf_log_fh,
            )
        )
        time.sleep(args.node_startup_s)
        report["process_returncodes_after_start"] = [proc.poll() for proc in procs]
        if any(proc.poll() is not None for proc in procs):
            raise RuntimeError(f"native local planner process exited early: {report['process_returncodes_after_start']}")

        camera_configs = []
        if args.video_layout == "evidence":
            from sim.engine.core.sensor import CameraConfig

            camera_configs = [
                CameraConfig(
                    name=args.camera_name,
                    width=max(320, int(args.video_width) // 2),
                    height=max(240, int(args.video_height) // 2),
                    fovy=float(args.camera_fovy),
                    render_depth=True,
                    depth_far=30.0,
                )
            ]

        sim_vehicle = str(args.sim_vehicle).strip().lower()
        robot_xml = ROOT / "sim/robots/omni_cart/omni_cart.xml" if sim_vehicle == "omni_cart" else None
        engine_start = list(route.start)
        if sim_vehicle == "omni_cart":
            engine_start = [float(route.start[0]), float(route.start[1]), 0.0]

        engine = _build_engine(
            world=route.scene_xml,
            drive_mode="kinematic",
            n_rays=args.n_rays,
            start=engine_start,
            mujoco_memory=args.mujoco_memory,
            camera_configs=camera_configs,
            robot_xml=robot_xml,
            base_body_name="base_link",
            lidar_body_name="lidar_link",
            leg_joint_names=[] if sim_vehicle == "omni_cart" else None,
            mid360_pattern=None if args.allow_golden_spiral_lidar else args.mid360_pattern,
            mid360_samples_per_frame=args.mid360_samples_per_frame,
        )
        resolved_mid360_pattern = _resolve_mid360_pattern(
            None if args.allow_golden_spiral_lidar else args.mid360_pattern
        )
        state = engine.get_robot_state()
        if args.video_out:
            if args.video_layout == "evidence":
                import cv2

                panel_w = max(320, int(args.video_width) // 2)
                panel_h = max(240, int(args.video_height) // 2)
                video_renderer, video_camera = _make_observer_renderer(
                    engine=engine,
                    route=control_route,
                    width=panel_w,
                    height=panel_h,
                )
                args.video_out.parent.mkdir(parents=True, exist_ok=True)
                video_writer = cv2.VideoWriter(
                    str(args.video_out),
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    float(args.video_fps),
                    (int(args.video_width), int(args.video_height)),
                )
                if not video_writer.isOpened():
                    raise RuntimeError(f"failed to open video writer: {args.video_out}")
            elif args.video_layout == "scene_overlay":
                video_renderer, video_camera, video_writer = _make_video_writer(
                    engine=engine,
                    route=control_route,
                    output=args.video_out,
                    width=args.video_width,
                    height=args.video_height,
                    fps=args.video_fps,
                )
            else:
                video_renderer, video_camera, video_writer = _make_video_writer(
                    engine=engine,
                    route=control_route,
                    output=args.video_out,
                    width=args.video_width,
                    height=args.video_height,
                    fps=args.video_fps,
                )

        odom_pub = node.create_publisher(Odometry, "/Odometry", 10)
        cloud_pub = node.create_publisher(mods["PointCloud2"], "/cloud_map", 5)
        terrain_pub = node.create_publisher(mods["PointCloud2"], "/terrain_map", 5)
        added_obstacles_pub = node.create_publisher(mods["PointCloud2"], "/added_obstacles", 5)
        speed_pub = node.create_publisher(Float32, "/speed", 5)
        goal_pub = node.create_publisher(PointStamped, "/way_point", 5)

        latest_cmd = {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0, "stamp": 0.0}
        path_stats = {"count": 0, "max_poses": 0, "last_poses": 0}
        status_samples: list[str] = []
        cmd_samples: list[dict[str, float]] = []
        applied_cmd_samples: list[dict[str, Any]] = []
        trajectory_samples: list[list[float]] = []
        local_path_samples: list[dict[str, Any]] = []
        local_path_evidence_samples: list[dict[str, Any]] = []
        moving_local_path_evidence_samples: list[dict[str, Any]] = []
        stop_samples: list[int] = []
        slow_down_samples: list[int] = []
        latest_local_path: list[list[float]] = []
        latest_local_path_frame_id = ""
        latest_lidar_points = np.zeros((0, 4), dtype=np.float32)
        video_trail: list[tuple[float, float]] = []
        timed_video_trail: list[tuple[float, float, float]] = []
        current_moving_obstacle_boxes: list[dict[str, Any]] = []
        current_moving_obstacle_points: list[tuple[float, float, float, float]] = []
        moving_obstacle_samples: list[dict[str, Any]] = []

        def on_cmd(msg: Any) -> None:
            latest_cmd["linear_x"] = float(msg.twist.linear.x)
            latest_cmd["linear_y"] = float(msg.twist.linear.y)
            latest_cmd["angular_z"] = float(msg.twist.angular.z)
            latest_cmd["stamp"] = time.time()
            if len(cmd_samples) < args.sample_limit:
                cmd_samples.append(
                    {
                        "linear_x": round(float(latest_cmd["linear_x"]), 6),
                        "linear_y": round(float(latest_cmd["linear_y"]), 6),
                        "angular_z": round(float(latest_cmd["angular_z"]), 6),
                    }
                )

        def on_path(msg: Any) -> None:
            nonlocal latest_local_path, latest_local_path_frame_id
            poses = msg.poses
            path_stats["count"] += 1
            path_stats["last_poses"] = len(poses)
            path_stats["max_poses"] = max(int(path_stats["max_poses"]), len(poses))
            latest_local_path = _ros_path_to_points(msg)
            latest_local_path_frame_id = str(getattr(getattr(msg, "header", None), "frame_id", "") or "")
            if len(local_path_samples) < args.sample_limit:
                stride = max(1, int(math.ceil(max(len(latest_local_path), 1) / 12.0)))
                local_path_samples.append(
                    {
                        "frame_id": latest_local_path_frame_id,
                        "robot_frame": _path_is_robot_frame(latest_local_path_frame_id),
                        "count": len(latest_local_path),
                        "points": [
                            [round(float(p[0]), 4), round(float(p[1]), 4), round(float(p[2]), 4)]
                            for p in latest_local_path[::stride][:12]
                        ],
                    }
                )
            if len(local_path_evidence_samples) < args.sample_limit:
                local_path_evidence_samples.append(
                    _local_path_obstacle_evidence(
                        local_path=latest_local_path,
                        local_path_frame_id=latest_local_path_frame_id,
                        state=state,
                        route=route,
                        robot_radius=float(args.robot_radius),
                        sample_step_m=float(args.waypoint_collision_sample_step),
                        extra_boxes=current_moving_obstacle_boxes,
                    )
                )
            if current_moving_obstacle_boxes and len(moving_local_path_evidence_samples) < args.sample_limit:
                moving_local_path_evidence_samples.append(
                    _local_path_obstacle_evidence(
                        local_path=latest_local_path,
                        local_path_frame_id=latest_local_path_frame_id,
                        state=state,
                        route=route,
                        robot_radius=float(args.robot_radius),
                        sample_step_m=float(args.waypoint_collision_sample_step),
                        extra_boxes=current_moving_obstacle_boxes,
                        include_route_boxes=False,
                    )
                )

        def on_status(msg: Any) -> None:
            if len(status_samples) < args.sample_limit:
                status_samples.append(str(msg.data))

        def on_stop(msg: Any) -> None:
            if len(stop_samples) < args.sample_limit:
                stop_samples.append(int(msg.data))

        def on_slow_down(msg: Any) -> None:
            if len(slow_down_samples) < args.sample_limit:
                slow_down_samples.append(int(msg.data))

        node.create_subscription(TwistStamped, "/cmd_vel", on_cmd, 20)
        node.create_subscription(RosPath, "/path", on_path, 10)
        node.create_subscription(String, "/nav/planner_status", on_status, 10)
        node.create_subscription(Int8, "/stop", on_stop, 10)
        node.create_subscription(Int8, "/slow_down", on_slow_down, 10)

        fields = _make_fields(PointField)

        def publish_odom(st: Any) -> None:
            msg = Odometry()
            msg.header.frame_id = "map"
            msg.child_frame_id = "base_link"
            _stamp(node, msg.header)
            msg.pose.pose.position.x = float(st.position[0])
            msg.pose.pose.position.y = float(st.position[1])
            msg.pose.pose.position.z = 0.0
            q = st.orientation
            msg.pose.pose.orientation.x = float(q[0])
            msg.pose.pose.orientation.y = float(q[1])
            msg.pose.pose.orientation.z = float(q[2])
            msg.pose.pose.orientation.w = float(q[3])
            msg.twist.twist.linear.x = float(st.linear_velocity[0])
            msg.twist.twist.linear.y = float(st.linear_velocity[1])
            msg.twist.twist.angular.z = float(st.angular_velocity[2])
            odom_pub.publish(msg)

        def publish_clouds(st: Any, elapsed_s: float = 0.0) -> None:
            nonlocal latest_lidar_points, current_moving_obstacle_boxes, current_moving_obstacle_points
            x = float(st.position[0])
            y = float(st.position[1])
            latest_lidar_points = _sample_lidar_points(engine, args.lidar_sample_count)
            current_moving_obstacle_boxes = _moving_obstacle_boxes(route, args, elapsed_s)
            current_moving_obstacle_points = _moving_obstacle_points(
                current_moving_obstacle_boxes,
                spacing=args.obstacle_point_spacing,
                intensity=args.moving_obstacle_intensity,
            )
            if obstacle_aware:
                # localPlanner treats every /cloud_map point as an obstacle when
                # useTerrainAnalysis=false. Keep this cloud empty as a trigger
                # and provide obstacle geometry through /added_obstacles.
                points: list[tuple[float, float, float, float]] = []
            else:
                points = _flat_terrain_points(x, y)
                _append_lidar_points(engine, points, args.lidar_sample_count)
            header = Header()
            header.frame_id = "map"
            _stamp(node, header)
            cloud = point_cloud2.create_cloud(header, fields, points)
            cloud_pub.publish(cloud)
            terrain_pub.publish(cloud)
            published_obstacles = [*obstacle_points, *current_moving_obstacle_points]
            if obstacle_aware and published_obstacles:
                obstacle_header = Header()
                obstacle_header.frame_id = "map"
                _stamp(node, obstacle_header)
                added_obstacles_pub.publish(point_cloud2.create_cloud(obstacle_header, fields, published_obstacles))
                if current_moving_obstacle_boxes:
                    report["moving_obstacles"]["published_update_count"] = int(
                        report["moving_obstacles"].get("published_update_count", 0)
                    ) + 1
                    report["moving_obstacles"]["published_point_count_max"] = max(
                        int(report["moving_obstacles"].get("published_point_count_max", 0)),
                        len(current_moving_obstacle_points),
                    )
                    if len(moving_obstacle_samples) < args.sample_limit:
                        box = current_moving_obstacle_boxes[0]
                        moving_obstacle_samples.append(
                            {
                                "elapsed_s": round(float(elapsed_s), 3),
                                "box_count": len(current_moving_obstacle_boxes),
                                "position": [
                                    round(float(box["position"][0]), 4),
                                    round(float(box["position"][1]), 4),
                                    round(float(box["position"][2]), 4),
                                ],
                                "positions": [
                                    [
                                        round(float(sample_box["position"][0]), 4),
                                        round(float(sample_box["position"][1]), 4),
                                        round(float(sample_box["position"][2]), 4),
                                    ]
                                    for sample_box in current_moving_obstacle_boxes[:8]
                                ],
                                "point_count": len(current_moving_obstacle_points),
                            }
                        )

        def publish_speed_goal(st: Any, target: list[float]) -> None:
            speed = Float32()
            speed.data = float(args.autonomy_speed)
            speed_pub.publish(speed)
            goal = PointStamped()
            goal.header.frame_id = "map"
            _stamp(node, goal.header)
            goal.point.x = float(target[0])
            goal.point.y = float(target[1])
            goal.point.z = 0.0
            goal_pub.publish(goal)

        target_idx = 1 if len(control_route.path) > 1 else 0
        waypoint_advance_threshold_m = max(
            float(args.waypoint_threshold_m),
            float(args.stop_distance) + 0.05,
        )
        report["waypoint_advance"] = {
            "configured_threshold_m": float(args.waypoint_threshold_m),
            "effective_threshold_m": round(float(waypoint_advance_threshold_m), 4),
            "path_follower_stop_distance_m": float(args.stop_distance),
            "reason": "must exceed pathFollower stopDisThre so the driver advances to the next waypoint",
        }
        for _ in range(args.warmup_ticks):
            publish_odom(state)
            publish_clouds(state, 0.0)
            publish_speed_goal(state, control_route.path[target_idx])
            rclpy.spin_once(node, timeout_sec=args.spin_timeout_s)
            time.sleep(args.step_s)

        start_xy = np.asarray(state.position[:2], dtype=float)
        final_goal_xy = np.asarray(control_route.path[-1][:2], dtype=float)
        max_abs_linear = 0.0
        max_abs_angular_z = 0.0
        nonzero_cmd_count = 0
        reached_goal = False
        steps = 0
        deadline = time.time() + args.timeout_s
        started_at = time.time()
        next_video_t = 0.0

        while time.time() < deadline:
            current_xy = np.asarray(state.position[:2], dtype=float)
            if target_idx < len(control_route.path) - 1:
                target_xy = np.asarray(control_route.path[target_idx][:2], dtype=float)
                if float(np.linalg.norm(current_xy - target_xy)) < waypoint_advance_threshold_m:
                    target_idx += 1

            final_distance = float(np.linalg.norm(current_xy - final_goal_xy))
            if (
                final_distance <= args.goal_threshold_m
                and path_stats["count"] > 0
                and latest_cmd["stamp"] > 0
            ):
                reached_goal = True
                break

            cmd_age = time.time() - float(latest_cmd["stamp"]) if latest_cmd["stamp"] else 999.0
            if cmd_age <= args.cmd_timeout_s:
                raw_cmd = VelocityCommand(
                    linear_x=max(-args.max_speed, min(args.max_speed, float(latest_cmd["linear_x"]))),
                    linear_y=max(-args.max_lateral_speed, min(args.max_lateral_speed, float(latest_cmd["linear_y"]))),
                    angular_z=max(-args.max_yaw_rate, min(args.max_yaw_rate, float(latest_cmd["angular_z"]))),
                )
            else:
                raw_cmd = VelocityCommand()

            drive_debug: dict[str, Any] = {"source": "native_pathFollower_cmd_vel"}
            if str(args.sim_vehicle) == "omni_cart":
                cmd, drive_debug = _omni_cart_cmd(
                    state=state,
                    local_path=latest_local_path,
                    local_path_frame_id=latest_local_path_frame_id,
                    fallback_target=control_route.path[target_idx],
                    lookahead_m=float(args.omni_lookahead_m),
                    min_speed=float(args.omni_min_speed),
                    max_speed=float(args.max_speed),
                    max_lateral_speed=float(args.max_lateral_speed),
                    yaw_gain=float(args.omni_yaw_gain),
                    max_yaw_rate=min(float(args.max_yaw_rate), float(args.omni_max_yaw_rate)),
                    yaw_deadband=float(args.omni_yaw_deadband),
                )
            else:
                cmd = raw_cmd

            if len(applied_cmd_samples) < args.sample_limit:
                applied_cmd_samples.append(
                    {
                        "linear_x": round(float(cmd.linear_x), 6),
                        "linear_y": round(float(cmd.linear_y), 6),
                        "angular_z": round(float(cmd.angular_z), 6),
                        **drive_debug,
                    }
                )

            max_abs_linear = max(max_abs_linear, abs(cmd.linear_x), abs(cmd.linear_y))
            max_abs_angular_z = max(max_abs_angular_z, abs(cmd.angular_z))
            if abs(cmd.linear_x) + abs(cmd.linear_y) + abs(cmd.angular_z) > 1e-4:
                nonzero_cmd_count += 1

            state = engine.step(cmd)
            elapsed_s = time.time() - started_at
            current_after_xy = (float(state.position[0]), float(state.position[1]))
            if not video_trail or math.hypot(current_after_xy[0] - video_trail[-1][0], current_after_xy[1] - video_trail[-1][1]) > 0.01:
                video_trail.append(current_after_xy)
                timed_video_trail.append((elapsed_s, current_after_xy[0], current_after_xy[1]))
            if steps % args.trajectory_sample_stride == 0:
                trajectory_samples.append(
                    [
                        round(float(state.position[0]), 4),
                        round(float(state.position[1]), 4),
                        int(target_idx),
                    ]
                )
            publish_odom(state)
            publish_clouds(state, elapsed_s)
            publish_speed_goal(state, control_route.path[target_idx])
            rclpy.spin_once(node, timeout_sec=args.spin_timeout_s)
            if video_writer is not None and elapsed_s >= next_video_t:
                distance_now = float(np.linalg.norm(np.asarray(state.position[:2], dtype=float) - final_goal_xy))
                display_cmd = (
                    {"linear_x": float(cmd.linear_x), "linear_y": float(cmd.linear_y), "angular_z": float(cmd.angular_z)}
                    if str(args.sim_vehicle) == "omni_cart"
                    else latest_cmd
                )
                if args.video_layout == "evidence":
                    _write_evidence_video_frame(
                        renderer=video_renderer,
                        camera=video_camera,
                        writer=video_writer,
                        engine=engine,
                        state=state,
                        route=control_route,
                        trail=video_trail,
                        target_idx=target_idx,
                        elapsed_s=elapsed_s,
                        distance_m=distance_now,
                        latest_cmd=display_cmd,
                        path_count=int(path_stats["count"]),
                        latest_local_path=latest_local_path,
                        latest_local_path_frame_id=latest_local_path_frame_id,
                        obstacle_points=[*obstacle_points, *current_moving_obstacle_points],
                        lidar_points=latest_lidar_points,
                        camera_name=args.camera_name,
                        width=int(args.video_width),
                        height=int(args.video_height),
                        front_path_lookahead_m=float(args.front_path_lookahead_m),
                    )
                elif args.video_layout == "scene_overlay":
                    _write_scene_overlay_video_frame(
                        renderer=video_renderer,
                        camera=video_camera,
                        writer=video_writer,
                        engine=engine,
                        state=state,
                        route=control_route,
                        trail=video_trail,
                        target_idx=target_idx,
                        elapsed_s=elapsed_s,
                        distance_m=distance_now,
                        latest_cmd=display_cmd,
                        path_count=int(path_stats["count"]),
                        latest_local_path=latest_local_path,
                        latest_local_path_frame_id=latest_local_path_frame_id,
                        lidar_points=latest_lidar_points,
                        width=int(args.video_width),
                        height=int(args.video_height),
                        front_path_lookahead_m=float(args.front_path_lookahead_m),
                        point_limit=int(args.scene_overlay_point_limit),
                    )
                else:
                    _write_video_frame(
                        renderer=video_renderer,
                        camera=video_camera,
                        writer=video_writer,
                        engine=engine,
                        route=control_route,
                        trail=video_trail,
                        target_idx=target_idx,
                        elapsed_s=elapsed_s,
                        distance_m=distance_now,
                        latest_cmd=display_cmd,
                        path_count=int(path_stats["count"]),
                    )
                video_frame_count += 1
                next_video_t += 1.0 / max(float(args.video_fps), 1.0)
            time.sleep(args.step_s)
            steps += 1

        end_xy = np.asarray(state.position[:2], dtype=float)
        moved_m = float(np.linalg.norm(end_xy - start_xy))
        final_distance_m = float(np.linalg.norm(end_xy - final_goal_xy))
        obstacle_clearance = _trail_obstacle_clearance(
            video_trail,
            route,
            robot_radius=args.robot_radius,
        )
        moving_obstacle_clearance = _moving_obstacle_trail_clearance(
            timed_trail=timed_video_trail,
            route=route,
            args=args,
            robot_radius=args.robot_radius,
        )
        trajectory_quality = _trajectory_correctness(
            trail=video_trail,
            reference_path=control_route.path,
            max_p95_error_m=float(args.max_trajectory_p95_error_m),
            max_progress_regressions=int(args.max_progress_regressions),
            min_route_progress_ratio=float(args.min_route_progress_ratio),
            reached_goal=bool(reached_goal),
        )
        local_path_evidence = _summarize_local_path_obstacle_evidence(
            samples=local_path_evidence_samples,
            path_count=int(path_stats["count"]),
            stop_samples=stop_samples,
            slow_down_samples=slow_down_samples,
            obstacle_aware=bool(obstacle_aware),
        )
        moving_local_path_evidence = _summarize_local_path_obstacle_evidence(
            samples=moving_local_path_evidence_samples,
            path_count=int(path_stats["count"]),
            stop_samples=[],
            slow_down_samples=[],
            obstacle_aware=bool(str(args.moving_obstacle_mode) != "none"),
        )
        moving_obstacle_ok = (
            str(args.moving_obstacle_mode) == "none"
            or (
                int(report["moving_obstacles"].get("published_update_count") or 0) > 0
                and bool(moving_obstacle_clearance.get("checked"))
                and not bool(moving_obstacle_clearance.get("collision"))
                and bool(moving_local_path_evidence.get("ok"))
            )
        )
        ok = (
            reached_goal
            and path_stats["count"] > 0
            and nonzero_cmd_count > 0
            and moved_m >= args.min_motion_m
            and (planner_name != "pct" or bool(route.plan.get("native_backend_used")))
            and (not obstacle_aware or not obstacle_clearance["collision"])
            and bool(trajectory_quality["ok"])
            and bool(local_path_evidence["ok"])
            and bool(moving_obstacle_ok)
        )
        report["moving_obstacles"].update(
            {
                "samples": moving_obstacle_samples,
                "trail_clearance": moving_obstacle_clearance,
                "local_path_obstacle_evidence": moving_local_path_evidence,
                "local_path_obstacle_samples": moving_local_path_evidence_samples,
                "ok": bool(moving_obstacle_ok),
            }
        )
        report.update(
            {
                "ok": bool(ok),
                "reached_goal": bool(reached_goal),
                "target_idx_final": int(target_idx),
                "steps": int(steps),
                "duration_s": round(float(args.timeout_s - max(0.0, deadline - time.time())), 3),
                "start_xy_actual": [round(float(start_xy[0]), 4), round(float(start_xy[1]), 4)],
                "end_xy": [round(float(end_xy[0]), 4), round(float(end_xy[1]), 4)],
                "moved_m": round(moved_m, 4),
                "final_distance_m": round(final_distance_m, 4),
                "path_count": int(path_stats["count"]),
                "max_path_poses": int(path_stats["max_poses"]),
                "last_path_poses": int(path_stats["last_poses"]),
                "local_path_samples": local_path_samples,
                "cmd_count_nonzero": int(nonzero_cmd_count),
                "cmd_samples": cmd_samples,
                "applied_cmd_samples": applied_cmd_samples,
                "max_abs_linear": round(float(max_abs_linear), 5),
                "max_abs_angular_z": round(float(max_abs_angular_z), 5),
                "planner_status_samples": status_samples,
                "obstacle_clearance": obstacle_clearance,
                "local_path_obstacle_evidence": local_path_evidence,
                "local_path_obstacle_samples": local_path_evidence_samples,
                "trajectory_quality": trajectory_quality,
                "trajectory_samples": trajectory_samples[: args.sample_limit],
                "native_nodes": ["local_planner/localPlanner", "local_planner/pathFollower"],
                "video": {
                    "path": str(args.video_out) if args.video_out else "",
                    "layout": str(args.video_layout),
                    "camera_name": str(args.camera_name),
                    "camera_fovy": float(args.camera_fovy),
                    "front_path_lookahead_m": float(args.front_path_lookahead_m),
                    "sim_vehicle": str(args.sim_vehicle),
                    "image_source": (
                        "MuJoCo observer renderer with 3D scene overlays"
                        if str(args.video_layout) == "scene_overlay"
                        else "MuJoCo CameraData RGB from robot MJCF camera"
                    ),
                    "scene_overlay_point_limit": int(args.scene_overlay_point_limit),
                    "lidar_source": {
                        "kind": (
                            "MuJoCo mj_multiRay with official Livox MID-360 scan pattern"
                            if resolved_mid360_pattern is not None
                            else "MuJoCo golden-spiral fallback scan"
                        ),
                        "forced_pattern": resolved_mid360_pattern is not None,
                        "pattern_path": str(resolved_mid360_pattern) if resolved_mid360_pattern is not None else "",
                        "pattern_sha256": _sha256(resolved_mid360_pattern) if resolved_mid360_pattern is not None else "",
                        "samples_per_frame": int(args.mid360_samples_per_frame),
                        "fallback_n_rays": int(args.n_rays),
                        "sample_count": int(args.lidar_sample_count),
                        "body": "lidar_link",
                    },
                    "frames": int(video_frame_count),
                    "fps": float(args.video_fps),
                    "width": int(args.video_width),
                    "height": int(args.video_height),
                    "exists": bool(args.video_out and args.video_out.exists()),
                },
            }
        )
    except Exception as exc:
        report["ok"] = False
        report["error"] = str(exc)
        report["traceback"] = traceback.format_exc(limit=12)
    finally:
        if video_writer is not None:
            video_writer.release()
        if video_renderer is not None:
            video_renderer.close()
        if engine is not None:
            try:
                engine.close()
            except Exception:
                pass
        process_returncodes_before_stop = [proc.poll() for proc in procs]
        for proc in procs:
            _stop_native_node(proc)
        process_returncodes_after_stop = [proc.poll() for proc in procs]
        report["process_returncodes_before_stop"] = process_returncodes_before_stop
        report["process_returncodes_after_stop"] = process_returncodes_after_stop
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy_started:
            try:
                rclpy.shutdown()
            except Exception:
                pass
        if lp_log_fh is not None:
            lp_log_fh.close()
        if pf_log_fh is not None:
            pf_log_fh.close()
        report["localPlanner_log_tail"] = _tail(local_planner_log)
        report["pathFollower_log_tail"] = _tail(path_follower_log)

    _write_report_if_requested(args, report)
    return report


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-report", type=Path, default=ROOT / "artifacts/native_pct_source/report.json")
    parser.add_argument("--generate-source-report", action="store_true")
    parser.add_argument("--force-generate-source-report", action="store_true")
    parser.add_argument("--route", default="same_floor")
    parser.add_argument("--planner", default="pct")
    parser.add_argument(
        "--contract-only",
        action="store_true",
        help=(
            "Validate source-report/PCT/same-source artifact contracts without "
            "launching ROS2 localPlanner/pathFollower or MuJoCo motion."
        ),
    )
    parser.add_argument("--artifact-dir", type=Path, default=ROOT / "artifacts/native_pct_mujoco_gate")
    parser.add_argument("--json-out", type=Path, default=ROOT / "artifacts/native_pct_mujoco_gate/report.json")
    parser.add_argument("--video-out", type=Path, default=None)
    parser.add_argument("--video-layout", choices=["observer", "evidence", "scene_overlay"], default="observer")
    parser.add_argument("--sim-vehicle", choices=["quadruped", "omni_cart"], default="quadruped")
    parser.add_argument("--camera-name", default="front_camera")
    parser.add_argument("--camera-fovy", type=float, default=78.0)
    parser.add_argument("--front-path-lookahead-m", type=float, default=14.0)
    parser.add_argument("--video-fps", type=float, default=20.0)
    parser.add_argument("--video-width", type=int, default=1280)
    parser.add_argument("--video-height", type=int, default=720)
    parser.add_argument("--scene-overlay-point-limit", type=int, default=500)
    parser.add_argument("--path-folder", type=Path, default=None)
    parser.add_argument("--ros-domain-id", default=os.environ.get("ROS_DOMAIN_ID", "93"))
    parser.add_argument("--allow-existing-cmd-vel-subscribers", action="store_true")
    parser.add_argument("--preflight-spin-s", type=float, default=1.0)
    parser.add_argument("--node-startup-s", type=float, default=1.5)
    parser.add_argument("--timeout-s", type=float, default=38.0)
    parser.add_argument("--step-s", type=float, default=0.02)
    parser.add_argument("--spin-timeout-s", type=float, default=0.005)
    parser.add_argument("--warmup-ticks", type=int, default=40)
    parser.add_argument("--max-speed", type=float, default=0.4)
    parser.add_argument("--max-lateral-speed", type=float, default=0.3)
    parser.add_argument("--max-yaw-rate", type=float, default=1.0)
    parser.add_argument("--omni-lookahead-m", type=float, default=1.2)
    parser.add_argument("--omni-min-speed", type=float, default=0.12)
    parser.add_argument("--omni-yaw-gain", type=float, default=0.45)
    parser.add_argument("--omni-max-yaw-rate", type=float, default=0.22)
    parser.add_argument("--omni-yaw-deadband", type=float, default=0.12)
    parser.add_argument("--autonomy-speed", type=float, default=0.35)
    parser.add_argument("--lookahead", type=float, default=0.55)
    parser.add_argument("--stop-distance", type=float, default=0.30)
    parser.add_argument("--goal-clear-range", type=float, default=0.45)
    parser.add_argument("--near-field-stop-distance", type=float, default=0.5)
    parser.add_argument("--disable-obstacle-check", action="store_true")
    parser.add_argument("--check-rot-obstacle", action="store_true")
    parser.add_argument("--obstacle-point-spacing", type=float, default=0.08)
    parser.add_argument("--obstacle-intensity", type=float, default=200.0)
    parser.add_argument("--moving-obstacle-mode", choices=["none", "route_crossing"], default="none")
    parser.add_argument("--moving-obstacle-route-ratio", type=float, default=0.5)
    parser.add_argument("--moving-obstacle-count", type=int, default=1)
    parser.add_argument("--moving-obstacle-route-ratio-step", type=float, default=0.08)
    parser.add_argument("--moving-obstacle-lateral-phase-step-rad", type=float, default=math.pi / 2.0)
    parser.add_argument("--moving-obstacle-start-s", type=float, default=8.0)
    parser.add_argument("--moving-obstacle-duration-s", type=float, default=0.0)
    parser.add_argument("--moving-obstacle-period-s", type=float, default=12.0)
    parser.add_argument("--moving-obstacle-lateral-amplitude-m", type=float, default=0.75)
    parser.add_argument("--moving-obstacle-along-amplitude-m", type=float, default=0.25)
    parser.add_argument("--moving-obstacle-radius-m", type=float, default=0.22)
    parser.add_argument("--moving-obstacle-height-m", type=float, default=0.7)
    parser.add_argument("--moving-obstacle-intensity", type=float, default=220.0)
    parser.add_argument("--robot-radius", type=float, default=0.28)
    parser.add_argument("--disable-waypoint-safety-filter", action="store_true")
    parser.add_argument("--waypoint-safety-margin", type=float, default=0.12)
    parser.add_argument("--waypoint-detour-margin", type=float, default=0.18)
    parser.add_argument("--waypoint-collision-sample-step", type=float, default=0.05)
    parser.add_argument("--waypoint-threshold-m", type=float, default=0.25)
    parser.add_argument("--goal-threshold-m", type=float, default=0.50)
    parser.add_argument("--min-motion-m", type=float, default=1.0)
    parser.add_argument("--max-trajectory-p95-error-m", type=float, default=2.0)
    parser.add_argument("--max-progress-regressions", type=int, default=8)
    parser.add_argument("--min-route-progress-ratio", type=float, default=0.92)
    parser.add_argument("--cmd-timeout-s", type=float, default=0.4)
    parser.add_argument("--n-rays", type=int, default=64)
    parser.add_argument("--lidar-sample-count", type=int, default=64)
    parser.add_argument("--mid360-pattern", type=Path, default=DEFAULT_MID360_PATTERN)
    parser.add_argument("--mid360-samples-per-frame", type=int, default=DEFAULT_MID360_SAMPLES_PER_FRAME)
    parser.add_argument(
        "--allow-golden-spiral-lidar",
        action="store_true",
        help="Use the legacy synthetic ray fan instead of the official MID-360 scan pattern.",
    )
    parser.add_argument("--mujoco-memory", default="512M")
    parser.add_argument("--sample-limit", type=int, default=60)
    parser.add_argument("--trajectory-sample-stride", type=int, default=25)
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    report = run_gate(args)
    text = json.dumps(report, ensure_ascii=False, indent=2)
    print(text)
    if args.strict and not report.get("ok"):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
