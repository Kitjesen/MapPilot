#!/usr/bin/env python3
"""Generate a tiny MuJoCo saved map and validate OctoPlanner3D planning."""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

BUILDING_SCENE_XML = ROOT / "sim" / "worlds" / "mujoco" / "building_scene.xml"
INDUSTRIAL_PARK_SCENE_XML = ROOT / "sim" / "worlds" / "mujoco" / "industrial_park_scene.xml"
CORRIDOR_DEFAULT_START = [0.0, 0.0, 0.0]
CORRIDOR_DEFAULT_GOAL = [2.4, 0.0, 0.0]
BUILDING_DEFAULT_START = [2.0, 3.0, 0.5]
BUILDING_DEFAULT_GOAL = [18.0, 11.0, 4.0]
INDUSTRIAL_PARK_DEFAULT_START = [3.0, 4.0, 0.3]
INDUSTRIAL_PARK_DEFAULT_GOAL = [56.0, 32.0, 0.3]
STAIRS3D_STEP_MAX_M = 0.23
STAIRS3D_MAX_SLOPE = 0.65
STAIRS3D_EDGE_CLEARANCE_M = 0.32
STAIRS3D_BODY_REFERENCE_HEIGHT_M = 0.55
STAIRS3D_BODY_SUPPORT_DEPTH_M = STAIRS3D_BODY_REFERENCE_HEIGHT_M
STAIRS3D_RISE_M = 0.20
STAIRS3D_TREAD_M = 0.42
STAIRS3D_STEP_COUNT = 6
STAIRS3D_UPPER_FLOOR_Z = STAIRS3D_RISE_M * STAIRS3D_STEP_COUNT
STAIRS3D_DEFAULT_START = [0.75, 0.0, 0.55]
STAIRS3D_DEFAULT_GOAL = [7.10, 0.0, STAIRS3D_UPPER_FLOOR_Z + 0.55]
BUILDING_MAP_PREFIXES = ("floor_", "wall_", "rail_", "stair_", "step_", "riser_", "obs_")
MULTIFLOOR_FLOOR_Y_STEP_M = 1.55
MULTILEVEL_SCENE_PRESETS = (
    "stairs3d",
    "stair_easy",
    "stair_limit_23cm",
    "stair_blocked_30cm",
    "multifloor_two_connectors",
    "multifloor_stack_3",
    "rough_terrain_traversability",
)
SCENE_PRESETS = ("corridor", "building", "industrial_park", *MULTILEVEL_SCENE_PRESETS)
STAIR_SCENE_PROFILES: dict[str, dict[str, Any]] = {
    "stairs3d": {
        "label": "legacy_stairs3d_limit",
        "rise_m": STAIRS3D_RISE_M,
        "tread_m": STAIRS3D_TREAD_M,
        "step_count": STAIRS3D_STEP_COUNT,
        "stair_width_m": 1.50,
        "include_down_connector": True,
        "include_second_connector": False,
        "include_rough_patch": False,
        "blocked": False,
    },
    "stair_easy": {
        "label": "easy_15cm_single_connector",
        "rise_m": 0.15,
        "tread_m": 0.45,
        "step_count": 6,
        "stair_width_m": 1.50,
        "include_down_connector": False,
        "include_second_connector": False,
        "include_rough_patch": False,
        "blocked": False,
    },
    "stair_limit_23cm": {
        "label": "limit_23cm_single_connector",
        "rise_m": 0.23,
        "tread_m": 0.45,
        "step_count": 5,
        "stair_width_m": 1.50,
        "include_down_connector": False,
        "include_second_connector": False,
        "include_rough_patch": False,
        "blocked": False,
    },
    "stair_blocked_30cm": {
        "label": "blocked_30cm_negative_case",
        "rise_m": 0.30,
        "tread_m": 0.42,
        "step_count": 4,
        "stair_width_m": 1.50,
        "include_down_connector": False,
        "include_second_connector": False,
        "include_rough_patch": False,
        "blocked": True,
    },
    "multifloor_two_connectors": {
        "label": "two_connectors_one_is_tight",
        "rise_m": 0.20,
        "tread_m": 0.42,
        "step_count": 6,
        "stair_width_m": 1.20,
        "include_down_connector": True,
        "include_second_connector": True,
        "include_rough_patch": False,
        "blocked": False,
    },
    "multifloor_stack_3": {
        "label": "three_level_navigation_building_with_rest_landings",
        "rise_m": 0.1125,
        "tread_m": 0.30,
        "step_count": 16,
        "mid_landing_after_steps": 8,
        "mid_landing_length_m": 0.80,
        "stair_width_m": 1.20,
        "include_down_connector": False,
        "include_second_connector": False,
        "include_rough_patch": False,
        "blocked": False,
        "levels": 3,
        "level_height_m": 1.80,
    },
    "rough_terrain_traversability": {
        "label": "rough_patch_plus_stair_connector",
        "rise_m": 0.18,
        "tread_m": 0.45,
        "step_count": 6,
        "stair_width_m": 1.50,
        "include_down_connector": False,
        "include_second_connector": False,
        "include_rough_patch": True,
        "blocked": False,
    },
}


def _first_executable(env_name: str, candidates: tuple[str, ...]) -> Path:
    override = str(os.environ.get(env_name) or "").strip()
    paths = ((Path(override),) if override else ()) + tuple(ROOT / value for value in candidates)
    for path in paths:
        resolved = path.expanduser().resolve()
        if resolved.is_file():
            return resolved
    raise FileNotFoundError(f"{env_name} executable is unavailable")


def _mapctl_executable() -> Path:
    return _first_executable(
        "LINGTU_MAPCTL_BIN",
        (
            "build/maps-windows/Release/lingtu-mapctl.exe",
            "build/maps/lingtu-mapctl",
        ),
    )


def _planner_executable(override: str = "") -> Path:
    if str(override or "").strip():
        path = Path(override).expanduser().resolve()
        if path.is_file():
            return path
        raise FileNotFoundError(f"OctoPlanner3D executable is unavailable: {path}")
    return _first_executable(
        "LINGTU_OCTOPLANNER3D_BIN",
        (
            "build/nav-cpp/windows-x64-nav-endpoint/Release/octoplanner3d_headless.exe",
            "build/octoplanner3d_headless/octoplanner3d_headless",
        ),
    )


def _run_json_process(
    command: list[str],
    *,
    timeout_s: float,
    payload: dict[str, Any] | None = None,
) -> dict[str, Any]:
    completed = subprocess.run(
        command,
        cwd=ROOT,
        input=(json.dumps(payload, separators=(",", ":")) + "\n" if payload is not None else None),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=max(1.0, float(timeout_s)),
        check=False,
    )
    try:
        result = json.loads(completed.stdout.strip())
    except json.JSONDecodeError:
        result = {
            "success": False,
            "ok": False,
            "reason_code": "invalid_native_json",
            "message": completed.stderr.strip() or completed.stdout.strip(),
        }
    if not isinstance(result, dict):
        result = {"success": False, "ok": False, "reason_code": "invalid_native_json"}
    result.setdefault("native_returncode", completed.returncode)
    if completed.returncode != 0:
        result.setdefault("success", False)
        result.setdefault("ok", False)
        result.setdefault("native_stderr", completed.stderr.strip())
    return result


def _build_octomap_native(
    *,
    map_root: Path,
    map_id: str,
    args: argparse.Namespace,
    scene_preset: str,
    lidar_map_source: bool,
) -> dict[str, Any]:
    command = [
        str(_mapctl_executable()),
        "build",
        map_id,
        "--map-root",
        str(map_root),
        "--build-mode",
        "external_pcl_converter" if args.converter else "native_octomap",
        "--resolution",
        str(float(args.resolution)),
        "--support-dilation-cells",
        str(effective_support_dilation_cells(args, scene_preset)),
        "--free-layers-above",
        str(effective_free_layers_above(args, scene_preset)),
        "--free-dilation-cells",
        str(int(getattr(args, "free_dilation_cells", 1))),
        "--frame",
        "map",
        "--source-profile",
        f"mujoco_{scene_preset}_saved_map_gate",
        "--data-source",
        "mujoco",
        "--slam-source",
        "mujoco_lidar_ground_truth_registered_map" if lidar_map_source else "mujoco_synthetic_map",
        "--localization-source",
        "mujoco_ground_truth_pose" if lidar_map_source else "mujoco_synthetic_scan",
        "--mapping-source",
        (
            f"mujoco_{scene_preset}_lidar_scan_accumulation"
            if lidar_map_source
            else f"mujoco_{scene_preset}_saved_map_plan_gate"
        ),
        "--timeout-s",
        str(float(args.converter_timeout)),
    ]
    if args.converter:
        command.extend(["--converter", str(args.converter)])
    result = _run_json_process(command, timeout_s=float(args.converter_timeout) + 5.0)
    result["ok"] = result.get("success") is True
    return result


def _validate_map_native(map_root: Path, map_id: str) -> dict[str, Any]:
    result = _run_json_process(
        [
            str(_mapctl_executable()),
            "prepare",
            map_id,
            "--map-root",
            str(map_root),
        ],
        timeout_s=10.0,
    )
    accepted = result.get("accepted") is True
    return {
        "ok": accepted,
        "blockers": [] if accepted else [str(result.get("message") or "native_map_validation_failed")],
        "native": result,
    }


def _plan_native(
    *,
    map_path: Path,
    start: list[float],
    goal: list[float],
    constraints: dict[str, Any],
    executable: str,
    timeout_s: float,
) -> dict[str, Any]:
    payload = {
        "planner": "octoplanner3d",
        "protocol_version": 1,
        "map_path": str(map_path.resolve()),
        "map_source": {
            "kind": "octomap_file",
            "path": str(map_path.resolve()),
            "format": map_path.suffix.lower().lstrip("."),
            "frame": "map",
        },
        "map_format": map_path.suffix.lower().lstrip("."),
        "start": [float(value) for value in start],
        "goal": [float(value) for value in goal],
        "options": dict(constraints),
    }
    result = _run_json_process(
        [str(_planner_executable(executable))],
        timeout_s=timeout_s,
        payload=payload,
    )
    path = result.get("path") if isinstance(result.get("path"), list) else []
    return {
        "ok": result.get("ok") is True and bool(path) and bool(result.get("reached_goal", True)),
        "available": True,
        "path_count": len(path),
        "path": path,
        "reached_goal": bool(result.get("reached_goal", False)),
        "error": str(result.get("error") or result.get("message") or ""),
        "diagnostics": result.get("diagnostics") or {},
        "constraint_overrides": constraints,
    }


def _frange(start: float, stop: float, step: float) -> list[float]:
    values: list[float] = []
    n = 0
    while True:
        value = start + n * step
        if value > stop + step * 0.5:
            return values
        values.append(round(value, 6))
        n += 1


def generate_scene_xml(
    path: Path,
    *,
    length: float,
    width: float,
    scene_preset: str = "corridor",
) -> None:
    if scene_preset == "building":
        shutil.copyfile(BUILDING_SCENE_XML, path)
        return
    if scene_preset == "industrial_park":
        shutil.copyfile(INDUSTRIAL_PARK_SCENE_XML, path)
        return
    if scene_preset in MULTILEVEL_SCENE_PRESETS:
        path.write_text(_stairs3d_scene_xml(scene_preset), encoding="utf-8")
        return
    if scene_preset != "corridor":
        raise ValueError(f"unsupported scene preset: {scene_preset}")
    floor_margin = 1.0
    floor_half_length = (float(length) + 2.0 * floor_margin) / 2.0
    path.write_text(
        f"""<mujoco model="lingtu_mujoco_saved_map_gate">
  <worldbody>
    <geom name="floor" type="box" pos="{length / 2:.3f} 0 -0.025" size="{floor_half_length:.3f} {width / 2:.3f} 0.025"/>
    <geom name="left_rail" type="box" pos="{length / 2:.3f} {width / 2:.3f} 0.4" size="{floor_half_length:.3f} 0.05 0.4"/>
    <geom name="right_rail" type="box" pos="{length / 2:.3f} {-width / 2:.3f} 0.4" size="{floor_half_length:.3f} 0.05 0.4"/>
  </worldbody>
</mujoco>
""",
        encoding="utf-8",
    )


def generate_points(
    *,
    length: float,
    width: float,
    spacing: float,
    hits_per_cell: int = 4,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    offsets = _hit_offsets(spacing, max(1, int(hits_per_cell)))
    xs = _frange(-0.2, length + 0.2, spacing)
    ys = _frange(-width / 2.0, width / 2.0, spacing)
    for x in xs:
        for y in ys:
            points.extend((x + dx, y + dy, dz) for dx, dy, dz in offsets)
    for y in (-width / 2.0, width / 2.0):
        for x in xs:
            for z in _frange(0.2, 0.8, spacing):
                points.extend((x + dx, y + dy, z + dz) for dx, dy, dz in offsets)
    return points


def _parse_vec(text: str | None, expected: int) -> list[float]:
    if not text:
        return []
    values = [float(v) for v in text.split()]
    if len(values) != expected or not all(math.isfinite(v) for v in values):
        return []
    return values


def _included_building_geom(name: str) -> bool:
    return name.startswith(BUILDING_MAP_PREFIXES)


def stair_scene_profile(scene_preset: str = "stairs3d") -> dict[str, Any]:
    if scene_preset not in STAIR_SCENE_PROFILES:
        raise ValueError(f"unsupported stair scene preset: {scene_preset}")
    return dict(STAIR_SCENE_PROFILES[scene_preset])


def stair_scene_default_start_goal(scene_preset: str = "stairs3d") -> tuple[list[float], list[float]]:
    if scene_preset == "multifloor_stack_3":
        return [7.00, -0.80, 0.55], [7.40, 2.80, 4.15]
    profile = stair_scene_profile(scene_preset)
    upper_floor_z = float(profile["rise_m"]) * int(profile["step_count"])
    start = [0.75, 0.0, 0.55]
    if bool(profile.get("include_second_connector")):
        goal = [7.45, 1.95, upper_floor_z + 0.55]
    elif bool(profile.get("include_down_connector")):
        goal = [7.10, 0.0, upper_floor_z + 0.55]
    else:
        goal = [5.90, 0.0, upper_floor_z + 0.55]
    return start, goal


def _stairs3d_geoms(scene_preset: str = "stairs3d") -> list[dict[str, Any]]:
    if scene_preset == "multifloor_stack_3":
        return _multifloor_stack_geoms(scene_preset)
    profile = stair_scene_profile(scene_preset)
    lower_z = -0.04
    tread = float(profile["tread_m"])
    rise = float(profile["rise_m"])
    step_count = int(profile["step_count"])
    upper_floor_top = rise * step_count
    stair_width = float(profile["stair_width_m"]) / 2.0
    up_start_x = 3.10
    upper_start_x = up_start_x + step_count * tread
    upper_end_x = upper_start_x + 2.40
    down_start_x = upper_end_x
    lower_b_start_x = down_start_x + step_count * tread
    room_half_y = 2.15 if bool(profile.get("include_second_connector")) else 1.30
    wall_y = room_half_y + 0.06
    geoms: list[dict[str, Any]] = [
        {
            "name": "floor_lower_a",
            "type": "box",
            "pos": [1.45, 0.0, lower_z],
            "size": [1.65, room_half_y, 0.04],
            "rgba": "0.76 0.78 0.74 1",
        },
        {
            "name": "floor_upper",
            "type": "box",
            "pos": [(upper_start_x + upper_end_x) / 2.0, 0.0, upper_floor_top - 0.04],
            "size": [(upper_end_x - upper_start_x) / 2.0, room_half_y, 0.04],
            "rgba": "0.70 0.74 0.80 1",
        },
        {
            "name": "floor_lower_b",
            "type": "box",
            "pos": [lower_b_start_x + 1.45, 0.0, lower_z],
            "size": [1.45, room_half_y, 0.04],
            "rgba": "0.76 0.78 0.74 1",
        },
        {
            "name": "wall_lower_a_north",
            "type": "box",
            "pos": [1.45, wall_y, 0.55],
            "size": [1.65, 0.06, 0.55],
            "rgba": "0.50 0.50 0.54 1",
        },
        {
            "name": "wall_lower_a_south",
            "type": "box",
            "pos": [1.45, -wall_y, 0.55],
            "size": [1.65, 0.06, 0.55],
            "rgba": "0.50 0.50 0.54 1",
        },
        {
            "name": "wall_lower_a_west",
            "type": "box",
            "pos": [-0.25, 0.0, 0.55],
            "size": [0.06, room_half_y, 0.55],
            "rgba": "0.50 0.50 0.54 1",
        },
        {
            "name": "wall_upper_north",
            "type": "box",
            "pos": [(upper_start_x + upper_end_x) / 2.0, wall_y, upper_floor_top + 0.55],
            "size": [(upper_end_x - upper_start_x) / 2.0, 0.06, 0.55],
            "rgba": "0.50 0.50 0.54 1",
        },
        {
            "name": "wall_upper_south",
            "type": "box",
            "pos": [(upper_start_x + upper_end_x) / 2.0, -wall_y, upper_floor_top + 0.55],
            "size": [(upper_end_x - upper_start_x) / 2.0, 0.06, 0.55],
            "rgba": "0.50 0.50 0.54 1",
        },
        {
            "name": "wall_lower_b_north",
            "type": "box",
            "pos": [lower_b_start_x + 1.45, wall_y, 0.55],
            "size": [1.45, 0.06, 0.55],
            "rgba": "0.50 0.50 0.54 1",
        },
        {
            "name": "wall_lower_b_south",
            "type": "box",
            "pos": [lower_b_start_x + 1.45, -wall_y, 0.55],
            "size": [1.45, 0.06, 0.55],
            "rgba": "0.50 0.50 0.54 1",
        },
        {
            "name": "wall_lower_b_east",
            "type": "box",
            "pos": [lower_b_start_x + 2.95, 0.0, 0.55],
            "size": [0.06, room_half_y, 0.55],
            "rgba": "0.50 0.50 0.54 1",
        },
        {
            "name": "obs_lower_crate",
            "type": "box",
            "pos": [1.85, 0.78, 0.28],
            "size": [0.30, 0.30, 0.28],
            "rgba": "0.86 0.64 0.22 1",
        },
        {
            "name": "obs_upper_column",
            "type": "cylinder",
            "pos": [(upper_start_x + upper_end_x) / 2.0, 0.82, upper_floor_top + 0.52],
            "size": [0.18, 0.52],
            "rgba": "0.74 0.42 0.18 1",
        },
        {
            "name": "obs_lower_b_crate",
            "type": "box",
            "pos": [lower_b_start_x + 1.15, -0.78, 0.26],
            "size": [0.30, 0.30, 0.26],
            "rgba": "0.86 0.64 0.22 1",
        },
    ]

    for idx in range(step_count):
        top_z = (idx + 1) * rise
        geoms.append(
            {
                "name": f"step_up_{idx + 1:02d}",
                "type": "box",
                "pos": [up_start_x + idx * tread + tread / 2.0, 0.0, top_z / 2.0],
                "size": [tread / 2.0, stair_width, top_z / 2.0],
                "rgba": "0.78 0.60 0.36 1",
            }
        )
    if bool(profile.get("include_down_connector")):
        for idx in range(step_count):
            top_z = upper_floor_top - idx * rise
            geoms.append(
                {
                    "name": f"step_down_{idx + 1:02d}",
                    "type": "box",
                    "pos": [down_start_x + idx * tread + tread / 2.0, 0.0, top_z / 2.0],
                    "size": [tread / 2.0, stair_width, top_z / 2.0],
                    "rgba": "0.78 0.60 0.36 1",
                }
            )
    if bool(profile.get("include_second_connector")):
        alt_y = 1.45
        for idx in range(step_count):
            top_z = (idx + 1) * rise
            geoms.append(
                {
                    "name": f"step_alt_up_{idx + 1:02d}",
                    "type": "box",
                    "pos": [up_start_x + idx * tread + tread / 2.0, alt_y, top_z / 2.0],
                    "size": [tread / 2.0, stair_width * 0.68, top_z / 2.0],
                    "rgba": "0.74 0.56 0.32 1",
                }
            )
        geoms.append(
            {
                "name": "obs_tight_connector_marker",
                "type": "box",
                "pos": [up_start_x + 1.5 * tread, 0.0, 0.35],
                "size": [0.22, 0.46, 0.35],
                "rgba": "0.88 0.30 0.22 1",
            }
        )
    if bool(profile.get("include_rough_patch")):
        for idx, (x, y, z, sx, sy, sz) in enumerate(
            [
                (1.35, -0.65, 0.06, 0.24, 0.20, 0.06),
                (1.85, -0.22, 0.08, 0.20, 0.24, 0.08),
                (2.30, 0.44, 0.05, 0.26, 0.18, 0.05),
                (2.58, -0.78, 0.10, 0.18, 0.22, 0.10),
            ],
            start=1,
        ):
            geoms.append(
                {
                    "name": f"rough_patch_{idx:02d}",
                    "type": "box",
                    "pos": [x, y, z / 2.0],
                    "size": [sx, sy, z / 2.0],
                    "rgba": "0.44 0.40 0.34 1",
                }
            )
    return geoms


def _multifloor_mid_landing(profile: dict[str, Any]) -> tuple[int, float]:
    steps = int(round(float(profile["level_height_m"]) / float(profile["rise_m"])))
    after_steps = int(profile.get("mid_landing_after_steps", 0) or 0)
    length_m = max(0.0, float(profile.get("mid_landing_length_m", 0.0) or 0.0))
    if after_steps <= 0 or after_steps >= steps or length_m <= 0.0:
        return 0, 0.0
    return after_steps, length_m


def _multifloor_step_progress(
    profile: dict[str, Any],
    index: int,
    *,
    center: bool,
) -> float:
    tread = float(profile["tread_m"])
    after_steps, landing_length = _multifloor_mid_landing(profile)
    progress = (float(index) + (0.5 if center else 0.0)) * tread
    if after_steps and index >= after_steps:
        progress += landing_length
    return progress


def _multifloor_run_length(profile: dict[str, Any]) -> float:
    steps = int(round(float(profile["level_height_m"]) / float(profile["rise_m"])))
    _, landing_length = _multifloor_mid_landing(profile)
    return steps * float(profile["tread_m"]) + landing_length


def _multifloor_stack_geoms(scene_preset: str = "multifloor_stack_3") -> list[dict[str, Any]]:
    profile = stair_scene_profile(scene_preset)
    levels = int(profile.get("levels", 3))
    level_height = float(profile.get("level_height_m", 1.26))
    tread = float(profile["tread_m"])
    rise = float(profile["rise_m"])
    steps = int(round(level_height / rise))
    run_length = _multifloor_run_length(profile)
    mid_landing_after, mid_landing_length = _multifloor_mid_landing(profile)
    slab_half_z = 0.022
    riser_half_x = 0.015
    floor_x = 3.90
    floor_y_step = MULTIFLOOR_FLOOR_Y_STEP_M
    floor_half_x = 4.80
    floor_half_y = 1.75
    stair_start_x = floor_x - 2.15
    stair_half_width = float(profile.get("stair_width_m", 1.20)) / 2.0
    wall_half_z = 0.50
    stair_rail_height = 1.08
    geoms: list[dict[str, Any]] = []

    for level in range(levels):
        z = level * level_height
        level_floor_y = level * floor_y_step
        geoms.extend(
            [
                {
                    "name": f"floor_level_{level + 1}",
                    "type": "box",
                    "pos": [floor_x, level_floor_y, z - 0.04],
                    "size": [floor_half_x, floor_half_y, 0.04],
                    "rgba": "0.74 0.76 0.72 0.66",
                },
                {
                    "name": f"rail_level_{level + 1}_south",
                    "type": "box",
                    "pos": [floor_x, level_floor_y - floor_half_y - 0.06, z + wall_half_z],
                    "size": [floor_half_x, 0.05, wall_half_z],
                    "rgba": "0.50 0.50 0.54 0.72",
                },
                {
                    "name": f"rail_level_{level + 1}_west",
                    "type": "box",
                    "pos": [floor_x - floor_half_x - 0.05, level_floor_y, z + wall_half_z],
                    "size": [0.05, floor_half_y, wall_half_z],
                    "rgba": "0.50 0.50 0.54 0.72",
                },
            ]
        )
        # Regular voxel-floor grid lines make the scene inspectable like planning papers.
        for gx in _frange(floor_x - floor_half_x + 0.42, floor_x + floor_half_x - 0.42, 0.42):
            geoms.append(
                {
                    "name": f"grid_x_l{level + 1}_{len(geoms)}",
                    "type": "box",
                    "pos": [gx, level_floor_y, z + 0.012],
                    "size": [0.012, floor_half_y, 0.012],
                    "rgba": "0.42 0.44 0.44 1",
                }
            )
        for gy in _frange(-floor_half_y + 0.42, floor_half_y - 0.42, 0.42):
            geoms.append(
                {
                    "name": f"grid_y_l{level + 1}_{len(geoms)}",
                    "type": "box",
                    "pos": [floor_x, level_floor_y + gy, z + 0.014],
                    "size": [floor_half_x, 0.012, 0.012],
                    "rgba": "0.42 0.44 0.44 1",
                }
            )
        geoms.append(
            {
                "name": f"obs_level_{level + 1}_block",
                "type": "box",
                "pos": [2.20 + level * 0.80, level_floor_y - 0.82 + 0.42 * (level % 2), z + 0.22],
                "size": [0.32, 0.28, 0.22],
                "rgba": "0.86 0.64 0.22 1",
            }
        )

    for level in range(levels - 1):
        base_z = level * level_height
        lower_floor_y = level * floor_y_step
        upper_floor_y = (level + 1) * floor_y_step
        y = max(lower_floor_y, upper_floor_y) + floor_half_y + stair_half_width + 0.04
        direction_x = 1.0 if level % 2 == 0 else -1.0
        run_start_x = stair_start_x if direction_x > 0.0 else stair_start_x + run_length
        run_end_x = run_start_x + direction_x * run_length
        # The landing must be large enough for the body reference path to leave
        # the last tread onto a real support surface. A too-small landing makes
        # a valid stair exit look like a side-wall climb in the voxel planner.
        landing_half_x = 0.72
        for landing_name, landing_x, landing_z, floor_edge_y in (
            (
                f"floor_stair_bottom_l{level + 1}",
                run_start_x - direction_x * landing_half_x,
                base_z,
                lower_floor_y + floor_half_y,
            ),
            (
                f"floor_stair_top_l{level + 1}",
                run_end_x + direction_x * landing_half_x,
                base_z + level_height,
                upper_floor_y + floor_half_y,
            ),
        ):
            geoms.append(
                {
                    "name": landing_name,
                    "type": "box",
                    "pos": [landing_x, y, landing_z + slab_half_z],
                    "size": [landing_half_x, stair_half_width, slab_half_z],
                    "rgba": "0.62 0.67 0.72 0.78",
                }
            )
            geoms.append(
                {
                    "name": f"floor_stair_bridge_{landing_name.removeprefix('floor_stair_')}",
                    "type": "box",
                    "pos": [landing_x, (floor_edge_y + y) / 2.0, landing_z + slab_half_z],
                    "size": [
                        landing_half_x,
                        (y - floor_edge_y) / 2.0 + STAIRS3D_EDGE_CLEARANCE_M,
                        slab_half_z,
                    ],
                    "rgba": "0.62 0.67 0.72 0.78",
                }
            )
        if level < levels - 2:
            next_upper_floor_y = (level + 2) * floor_y_step
            next_y = max(upper_floor_y, next_upper_floor_y) + floor_half_y + stair_half_width + 0.04
            transfer_half_y = abs(next_y - y) / 2.0 + stair_half_width
            geoms.append(
                {
                    "name": f"floor_stair_transfer_l{level + 1}_to_l{level + 2}",
                    "type": "box",
                    "pos": [
                        run_end_x + direction_x * landing_half_x,
                        (y + next_y) / 2.0,
                        base_z + level_height + slab_half_z,
                    ],
                    "size": [landing_half_x, transfer_half_y, slab_half_z],
                    "rgba": "0.62 0.67 0.72 0.82",
                }
            )
        if mid_landing_after:
            landing_progress = mid_landing_after * tread
            landing_z = base_z + mid_landing_after * rise
            geoms.append(
                {
                    "name": f"floor_stair_mid_l{level + 1}",
                    "type": "box",
                    "pos": [
                        run_start_x + direction_x * (landing_progress + mid_landing_length / 2.0),
                        y,
                        landing_z + slab_half_z,
                    ],
                    "size": [mid_landing_length / 2.0, stair_half_width, slab_half_z],
                    "rgba": "0.62 0.67 0.72 0.88",
                }
            )
        for idx in range(steps):
            top_z = base_z + (idx + 1) * rise
            prev_z = base_z + idx * rise
            center_x = run_start_x + direction_x * _multifloor_step_progress(profile, idx, center=True)
            riser_x = run_start_x + direction_x * _multifloor_step_progress(profile, idx, center=False)
            geoms.append(
                {
                    "name": f"step_stack_l{level + 1}_{idx + 1:02d}",
                    "type": "box",
                    "pos": [center_x, y, top_z - slab_half_z],
                    "size": [tread / 2.0, stair_half_width, slab_half_z],
                    "rgba": "0.56 0.48 0.72 1",
                }
            )
            geoms.append(
                {
                    "name": f"riser_stack_l{level + 1}_{idx + 1:02d}",
                    "type": "box",
                    "pos": [riser_x + direction_x * riser_half_x, y, (prev_z + top_z) / 2.0],
                    "size": [riser_half_x, stair_half_width, (top_z - prev_z) / 2.0],
                    "rgba": "0.42 0.34 0.58 1",
                }
            )
            for side_name, side_sign in (("south", -1.0), ("north", 1.0)):
                geoms.append(
                    {
                        "name": f"rail_stair_l{level + 1}_{idx + 1:02d}_{side_name}_handrail",
                        "type": "box",
                        "pos": [
                            center_x,
                            y + side_sign * (stair_half_width + 0.07),
                            top_z + stair_rail_height,
                        ],
                        "size": [tread / 2.0, 0.025, 0.035],
                        "rgba": "0.36 0.36 0.42 0.82",
                    }
                )
                geoms.append(
                    {
                        "name": f"rail_stair_l{level + 1}_{idx + 1:02d}_{side_name}_post",
                        "type": "box",
                        "pos": [
                            center_x,
                            y + side_sign * (stair_half_width + 0.07),
                            top_z + stair_rail_height / 2.0,
                        ],
                        "size": [0.025, 0.025, stair_rail_height / 2.0],
                        "rgba": "0.36 0.36 0.42 0.82",
                    }
                )
    return geoms


def _interpolate_mapping_waypoints(
    waypoints: list[list[float]],
    *,
    spacing_m: float,
) -> list[list[float]]:
    """Interpolate xyz waypoints and attach a forward yaw to every pose."""

    if not waypoints:
        return []
    spacing = max(0.05, float(spacing_m))
    trajectory: list[list[float]] = [[float(waypoints[0][0]), float(waypoints[0][1]), float(waypoints[0][2]), 0.0]]
    previous_yaw = 0.0
    for start, end in zip(waypoints, waypoints[1:]):
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        dz = float(end[2]) - float(start[2])
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        count = max(1, int(math.ceil(distance / spacing)))
        dxy = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx) if dxy > 1e-9 else previous_yaw
        for index in range(1, count + 1):
            ratio = index / count
            trajectory.append(
                [
                    float(start[0]) + dx * ratio,
                    float(start[1]) + dy * ratio,
                    float(start[2]) + dz * ratio,
                    yaw,
                ]
            )
        previous_yaw = yaw
    if len(trajectory) > 1:
        trajectory[0][3] = trajectory[1][3]
    return trajectory


def multifloor_mapping_trajectory(
    scene_preset: str = "multifloor_stack_3",
    *,
    spacing_m: float = 0.38,
) -> list[list[float]]:
    """Return a physically connected LiDAR mapping route through all floors.

    The route follows both stair centerlines. It is used only to position the
    simulated sensor while building a ground-truth registered LiDAR map; it
    does not inject occupied support points into the resulting PCD.
    """

    if scene_preset != "multifloor_stack_3":
        raise ValueError(f"mapping trajectory is not defined for {scene_preset}")
    profile = stair_scene_profile(scene_preset)
    levels = int(profile.get("levels", 3))
    if levels != 3:
        raise ValueError("multifloor mapping trajectory currently requires three levels")

    level_height = float(profile["level_height_m"])
    tread = float(profile["tread_m"])
    rise = float(profile["rise_m"])
    steps = int(round(level_height / rise))
    run_length = _multifloor_run_length(profile)
    mid_landing_after, mid_landing_length = _multifloor_mid_landing(profile)
    floor_x = 3.90
    floor_y_step = MULTIFLOOR_FLOOR_Y_STEP_M
    floor_half_x = 4.80
    floor_half_y = 1.75
    stair_start_x = floor_x - 2.15
    stair_half_width = float(profile["stair_width_m"]) / 2.0
    body_height = 0.55
    landing_half_x = 0.72

    x_left = floor_x - floor_half_x + 0.70
    x_right = floor_x + floor_half_x - 0.90
    waypoints: list[list[float]] = [
        [0.70, -1.20, body_height],
        [x_right, -1.20, body_height],
        [x_right, 1.20, body_height],
    ]

    for level in range(levels - 1):
        base_z = level * level_height
        lower_floor_y = level * floor_y_step
        upper_floor_y = (level + 1) * floor_y_step
        stair_y = max(lower_floor_y, upper_floor_y) + floor_half_y + stair_half_width + 0.04
        direction_x = 1.0 if level % 2 == 0 else -1.0
        run_start_x = stair_start_x if direction_x > 0.0 else stair_start_x + run_length
        run_end_x = run_start_x + direction_x * run_length
        bottom_landing_x = run_start_x - direction_x * landing_half_x
        top_landing_x = run_end_x + direction_x * landing_half_x
        lower_edge_y = lower_floor_y + floor_half_y
        upper_edge_y = upper_floor_y + floor_half_y

        if level == 0:
            waypoints.extend(
                [
                    [bottom_landing_x, 1.20, base_z + body_height],
                    [bottom_landing_x, lower_edge_y, base_z + body_height],
                    [bottom_landing_x, stair_y, base_z + body_height],
                ]
            )
        else:
            waypoints.append([bottom_landing_x, stair_y, base_z + body_height])

        waypoints.append(
            [
                run_start_x - direction_x * 0.32,
                stair_y,
                base_z + body_height,
            ]
        )
        for index in range(steps):
            if mid_landing_after and index == mid_landing_after:
                waypoints.append(
                    [
                        run_start_x + direction_x * (index * tread + mid_landing_length),
                        stair_y,
                        base_z + index * rise + body_height,
                    ]
                )
            waypoints.append(
                [
                    run_start_x + direction_x * (_multifloor_step_progress(profile, index, center=False) + tread),
                    stair_y,
                    base_z + (index + 1) * rise + body_height,
                ]
            )
        upper_body_z = base_z + level_height + body_height
        waypoints.extend(
            [
                [top_landing_x, stair_y, upper_body_z],
                [top_landing_x, upper_edge_y, upper_body_z],
            ]
        )

        if level < levels - 2:
            deck_y_high = upper_floor_y + floor_half_y - 0.35
            deck_y_low = upper_floor_y - floor_half_y + 0.35
            waypoints.extend(
                [
                    [x_right, deck_y_high, upper_body_z],
                    [x_left, deck_y_high, upper_body_z],
                    [x_left, deck_y_low, upper_body_z],
                    [x_right, deck_y_low, upper_body_z],
                    [top_landing_x, upper_edge_y, upper_body_z],
                ]
            )
            next_level = level + 1
            next_lower_floor_y = next_level * floor_y_step
            next_upper_floor_y = (next_level + 1) * floor_y_step
            next_stair_y = max(next_lower_floor_y, next_upper_floor_y) + floor_half_y + stair_half_width + 0.04
            next_direction_x = 1.0 if next_level % 2 == 0 else -1.0
            next_run_start_x = stair_start_x if next_direction_x > 0.0 else stair_start_x + run_length
            next_bottom_x = next_run_start_x - next_direction_x * landing_half_x
            waypoints.append([next_bottom_x, next_stair_y, upper_body_z])

    top_z = (levels - 1) * level_height + body_height
    top_center_y = (levels - 1) * floor_y_step
    top_y_high = top_center_y + floor_half_y - 0.35
    top_y_low = top_center_y - floor_half_y + 0.35
    waypoints.extend(
        [
            [x_left, top_y_high, top_z],
            [x_left, top_y_low, top_z],
            [x_right, top_y_low, top_z],
            [x_right, top_y_high, top_z],
            [7.40, 2.80, top_z],
        ]
    )
    return _interpolate_mapping_waypoints(waypoints, spacing_m=spacing_m)


def _stairs3d_scene_xml(scene_preset: str = "stairs3d") -> str:
    geom_lines: list[str] = []
    for geom in _stairs3d_geoms(scene_preset):
        rgba = geom.get("rgba", "0.7 0.7 0.7 1")
        size = " ".join(f"{float(v):.4f}".rstrip("0").rstrip(".") for v in geom["size"])
        pos = " ".join(f"{float(v):.4f}".rstrip("0").rstrip(".") for v in geom["pos"])
        geom_lines.append(
            f'    <geom name="{geom["name"]}" type="{geom["type"]}" '
            f'pos="{pos}" size="{size}" rgba="{rgba}" contype="1" conaffinity="1" group="1"/>\n'
        )
    start_values, goal_values = stair_scene_default_start_goal(scene_preset)
    start = " ".join(str(v) for v in start_values)
    goal = " ".join(str(v) for v in goal_values)
    ground_contact = (
        'contype="0" conaffinity="0" group="5"'
        if scene_preset == "multifloor_stack_3"
        else 'contype="1" conaffinity="1" group="1"'
    )
    return f"""<mujoco model="lingtu_{scene_preset}_nav">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <visual>
    <global offwidth="1600" offheight="1000"/>
    <headlight ambient="0.45 0.45 0.45"/>
    <map znear="0.01" zfar="80"/>
  </visual>
  <asset>
    <material name="ground_mat" rgba="0.60 0.63 0.60 1"/>
  </asset>
  <worldbody>
    <body name="robot_placeholder" pos="{start}"/>
    <light pos="5 -7 8" dir="-0.3 0.45 -1" diffuse="0.9 0.88 0.82" castshadow="false"/>
    <geom name="ground_reference" type="plane" size="15 8 0.1" material="ground_mat"
          {ground_contact} condim="3" friction="1 0.5 0.5"/>
{"".join(geom_lines)}    <geom name="goal_marker" type="sphere" size="0.18" pos="{goal}"
          contype="0" conaffinity="0" rgba="0.1 0.32 1 0.75" group="5"/>
  </worldbody>
</mujoco>
"""


def _append_hits(
    points: list[tuple[float, float, float]],
    x: float,
    y: float,
    z: float,
    offsets: list[tuple[float, float, float]],
) -> None:
    points.extend((x + dx, y + dy, z + dz) for dx, dy, dz in offsets)


def _sample_box_surfaces(
    points: list[tuple[float, float, float]],
    *,
    pos: list[float],
    size: list[float],
    spacing: float,
    offsets: list[tuple[float, float, float]],
    yaw: float = 0.0,
) -> None:
    cx, cy, cz = pos
    hx, hy, hz = size
    x0, x1 = -hx, hx
    y0, y1 = -hy, hy
    z0, z1 = cz - hz, cz + hz
    cos_yaw = math.cos(float(yaw))
    sin_yaw = math.sin(float(yaw))

    def append(local_x: float, local_y: float, z: float) -> None:
        x = cx + cos_yaw * local_x - sin_yaw * local_y
        y = cy + sin_yaw * local_x + cos_yaw * local_y
        _append_hits(points, x, y, z, offsets)

    for x in _frange(x0, x1, spacing):
        for y in _frange(y0, y1, spacing):
            append(x, y, z1)
    for z in _frange(z0, z1, spacing):
        for y in _frange(y0, y1, spacing):
            append(x0, y, z)
            append(x1, y, z)
        for x in _frange(x0, x1, spacing):
            append(x, y0, z)
            append(x, y1, z)


def _sample_box_top(
    points: list[tuple[float, float, float]],
    *,
    pos: list[float],
    size: list[float],
    spacing: float,
    offsets: list[tuple[float, float, float]],
    y_half_override: float | None = None,
) -> None:
    cx, cy, cz = pos
    hx, hy, hz = size
    if y_half_override is not None:
        hy = min(hy, max(0.0, y_half_override))
    x0, x1 = cx - hx, cx + hx
    y0, y1 = cy - hy, cy + hy
    z1 = cz + hz
    for x in _frange(x0, x1, spacing):
        for y in _frange(y0, y1, spacing):
            _append_hits(points, x, y, z1, offsets)


def _sample_box_vertical_surfaces(
    points: list[tuple[float, float, float]],
    *,
    pos: list[float],
    size: list[float],
    spacing: float,
    offsets: list[tuple[float, float, float]],
) -> None:
    cx, cy, cz = pos
    hx, hy, hz = size
    x0, x1 = cx - hx, cx + hx
    y0, y1 = cy - hy, cy + hy
    z0, z1 = cz - hz, cz + hz
    for z in _frange(z0, z1, spacing):
        for y in _frange(y0, y1, spacing):
            _append_hits(points, x0, y, z, offsets)
            _append_hits(points, x1, y, z, offsets)
        for x in _frange(x0, x1, spacing):
            _append_hits(points, x, y0, z, offsets)
            _append_hits(points, x, y1, z, offsets)


def _sample_cylinder_surfaces(
    points: list[tuple[float, float, float]],
    *,
    pos: list[float],
    size: list[float],
    spacing: float,
    offsets: list[tuple[float, float, float]],
) -> None:
    if len(size) < 2:
        return
    cx, cy, cz = pos
    radius, half_height = size[:2]
    z0, z1 = cz - half_height, cz + half_height
    angle_step = max(0.12, min(math.pi / 8.0, spacing / max(radius, 1e-6)))
    angles = _frange(0.0, math.tau, angle_step)
    for r in _frange(0.0, radius, max(spacing, radius)):
        for angle in angles:
            x = cx + math.cos(angle) * r
            y = cy + math.sin(angle) * r
            _append_hits(points, x, y, z1, offsets)
    for z in _frange(z0, z1, spacing):
        for angle in angles:
            x = cx + math.cos(angle) * radius
            y = cy + math.sin(angle) * radius
            _append_hits(points, x, y, z, offsets)


def _building_map_geoms() -> list[dict[str, Any]]:
    root = ET.parse(BUILDING_SCENE_XML).getroot()
    geoms: list[dict[str, Any]] = []
    for geom in root.findall(".//geom"):
        name = geom.attrib.get("name", "")
        if not _included_building_geom(name):
            continue
        geom_type = geom.attrib.get("type", "box")
        pos = _parse_vec(geom.attrib.get("pos"), 3)
        size_len = 2 if geom_type == "cylinder" else 3
        size = _parse_vec(geom.attrib.get("size"), size_len)
        if not pos or not size:
            continue
        geoms.append({"name": name, "type": geom_type, "pos": pos, "size": size})
    return geoms


def generate_building_points(
    *,
    spacing: float,
    hits_per_cell: int = 4,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    offsets = _hit_offsets(spacing, max(1, int(hits_per_cell)))
    for geom in _building_map_geoms():
        if geom["type"] == "box":
            _sample_box_surfaces(
                points,
                pos=geom["pos"],
                size=geom["size"],
                spacing=spacing,
                offsets=offsets,
            )
        elif geom["type"] == "cylinder":
            _sample_cylinder_surfaces(
                points,
                pos=geom["pos"],
                size=geom["size"],
                spacing=spacing,
                offsets=offsets,
            )
    return points


def _industrial_park_map_geoms() -> list[dict[str, Any]]:
    root = ET.parse(INDUSTRIAL_PARK_SCENE_XML).getroot()
    geoms: list[dict[str, Any]] = []
    for geom in root.findall("./worldbody/geom"):
        name = geom.attrib.get("name", "")
        if geom.attrib.get("contype", "1").strip() == "0":
            continue
        if name.startswith(("roof_", "beam_", "pipe_")):
            continue
        geom_type = geom.attrib.get("type", "box").strip().lower()
        if geom_type not in {"box", "cylinder"}:
            continue
        pos = _parse_vec(geom.attrib.get("pos"), 3)
        size = _parse_vec(geom.attrib.get("size"), 2 if geom_type == "cylinder" else 3)
        if not pos or not size:
            continue
        euler = _parse_vec(geom.attrib.get("euler"), 3)
        geoms.append(
            {
                "name": name,
                "type": geom_type,
                "pos": pos,
                "size": size,
                "yaw": float(euler[2]) if euler else 0.0,
            }
        )
    return geoms


def generate_industrial_park_points(
    *,
    spacing: float,
    hits_per_cell: int = 4,
) -> list[tuple[float, float, float]]:
    """Sample the product industrial scene into a deterministic diagnostic map."""

    points: list[tuple[float, float, float]] = []
    offsets = _hit_offsets(spacing, max(1, int(hits_per_cell)))
    for geom in _industrial_park_map_geoms():
        if geom["type"] == "box":
            _sample_box_surfaces(
                points,
                pos=geom["pos"],
                size=geom["size"],
                spacing=spacing,
                offsets=offsets,
                yaw=float(geom.get("yaw") or 0.0),
            )
        else:
            _sample_cylinder_surfaces(
                points,
                pos=geom["pos"],
                size=geom["size"],
                spacing=spacing,
                offsets=offsets,
            )
    return points


def generate_stairs3d_points(
    *,
    spacing: float,
    hits_per_cell: int = 4,
    scene_preset: str = "stairs3d",
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    offsets = _hit_offsets(spacing, max(1, int(hits_per_cell)))
    for geom in _stairs3d_geoms(scene_preset):
        name = str(geom.get("name", ""))
        if name.startswith("wall_stair_"):
            continue
        if name.startswith("riser_stack_"):
            continue
        if not _included_building_geom(name):
            continue
        if geom["type"] == "box":
            start_idx = len(points)
            if name.startswith("step_stack_"):
                # Planning support should represent the traversable stair lane,
                # not every visual surface. Keeping the side strip out prevents
                # synthetic OctoMap dilation from inventing edge-walking support.
                safe_half_width = max(
                    0.18,
                    min(float(geom["size"][1]), float(geom["size"][1]) - STAIRS3D_EDGE_CLEARANCE_M),
                )
                _sample_box_top(
                    points,
                    pos=geom["pos"],
                    size=geom["size"],
                    spacing=spacing,
                    offsets=offsets,
                    y_half_override=safe_half_width,
                )
            else:
                _sample_box_surfaces(
                    points,
                    pos=geom["pos"],
                    size=geom["size"],
                    spacing=spacing,
                    offsets=offsets,
                )
            if scene_preset.startswith("multifloor_") and name.startswith(("floor_level_", "grid_")):
                points[start_idx:] = [
                    point for point in points[start_idx:] if not _in_multifloor_stairwell_void(scene_preset, point)
                ]
        elif geom["type"] == "cylinder":
            _sample_cylinder_surfaces(
                points,
                pos=geom["pos"],
                size=geom["size"],
                spacing=spacing,
                offsets=offsets,
            )
    return points


def _hit_offsets(spacing: float, count: int) -> list[tuple[float, float, float]]:
    base = max(0.001, min(0.02, spacing * 0.05))
    pattern = [
        (0.0, 0.0, 0.0),
        (base, 0.0, 0.0),
        (0.0, base, 0.0),
        (base, base, 0.0),
        (-base, 0.0, 0.0),
        (0.0, -base, 0.0),
    ]
    return pattern[:count] if count <= len(pattern) else pattern + [pattern[-1]] * (count - len(pattern))


def write_ascii_pcd(path: Path, points: list[tuple[float, float, float]]) -> None:
    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS x y z",
            "SIZE 4 4 4",
            "TYPE F F F",
            "COUNT 1 1 1",
            f"WIDTH {len(points)}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {len(points)}",
            "DATA ascii",
        ]
    )
    body = "\n".join(f"{x:.6f} {y:.6f} {z:.6f}" for x, y, z in points)
    path.write_text(header + "\n" + body + "\n", encoding="ascii")


def jsonable_path(path: list[Any], *, max_points: int | None = None) -> list[list[float]]:
    """Return planner path points as JSON-safe xyz triples."""

    limited = path if max_points is None else path[: max(0, int(max_points))]
    out: list[list[float]] = []
    for point in limited:
        values = [float(v) for v in point[:3]]
        if len(values) == 3 and all(math.isfinite(v) for v in values):
            out.append(values)
    return out


def _in_multifloor_stairwell_void(scene_preset: str, point: tuple[float, float, float]) -> bool:
    profile = stair_scene_profile(scene_preset)
    levels = int(profile.get("levels", 3))
    if levels < 2:
        return False
    x, y, z = point
    level_height = float(profile.get("level_height_m", 1.26))
    tread = float(profile["tread_m"])
    run_length = _multifloor_run_length(profile)
    floor_x = 3.90
    floor_y_step = MULTIFLOOR_FLOOR_Y_STEP_M
    floor_half_y = 1.75
    stair_start_x = floor_x - 2.15
    stair_half_width = float(profile.get("stair_width_m", 1.20)) / 2.0
    for level in range(levels - 1):
        upper_floor_z = (level + 1) * level_height
        if abs(z - upper_floor_z) > 0.08:
            continue
        lower_floor_y = level * floor_y_step
        upper_floor_y = (level + 1) * floor_y_step
        stair_y = max(lower_floor_y, upper_floor_y) + floor_half_y + stair_half_width + 0.04
        direction_x = 1.0 if level % 2 == 0 else -1.0
        run_start_x = stair_start_x if direction_x > 0.0 else stair_start_x + run_length
        run_end_x = run_start_x + direction_x * run_length
        x_min = min(run_start_x, run_end_x) - tread * 0.65
        x_max = max(run_start_x, run_end_x) + tread * 0.65
        y_min = stair_y - stair_half_width - 0.36
        y_max = stair_y + stair_half_width + 0.36
        if x_min <= x <= x_max and y_min <= y <= y_max:
            return True
    return False


def _multifloor_stair_topology(
    scene_preset: str,
    path: list[list[float]],
) -> dict[str, Any]:
    """Verify that each floor transition follows its physical stair run."""

    if scene_preset != "multifloor_stack_3":
        return {"ok": True, "blockers": [], "stairs": []}

    profile = stair_scene_profile(scene_preset)
    level_height = float(profile["level_height_m"])
    rise = float(profile["rise_m"])
    tread = float(profile["tread_m"])
    steps = int(round(level_height / rise))
    run_length = _multifloor_run_length(profile)
    stair_half_width = float(profile["stair_width_m"]) / 2.0
    floor_y_step = MULTIFLOOR_FLOOR_Y_STEP_M
    floor_half_y = 1.75
    stair_start_x = 3.90 - 2.15
    body_height = STAIRS3D_BODY_REFERENCE_HEIGHT_M
    # A one-cell Z wobble is expected when a continuous floor is represented
    # by an OctoMap whose resolution is close to the physical stair rise. Only
    # classify larger off-stair climbs as topology violations; pure vertical
    # edges are still rejected separately by scene_acceptance().
    major_rise = rise * 0.75
    off_stair_major_rise = max(rise * 1.25, 0.14)

    stairs: list[dict[str, Any]] = []
    for level in range(2):
        lower_floor_y = level * floor_y_step
        upper_floor_y = (level + 1) * floor_y_step
        stair_y = max(lower_floor_y, upper_floor_y) + floor_half_y + stair_half_width + 0.04
        direction = 1.0 if level % 2 == 0 else -1.0
        run_start_x = stair_start_x if direction > 0.0 else stair_start_x + run_length
        run_end_x = run_start_x + direction * run_length
        stairs.append(
            {
                "level": level + 1,
                "stair_y": stair_y,
                "direction": direction,
                "run_start_x": run_start_x,
                "run_end_x": run_end_x,
                "vertical_gain_m": 0.0,
                "directed_run_m": 0.0,
                "climbing_segments": 0,
                "reversed_major_segments": 0,
            }
        )

    blockers: list[str] = []
    major_rises_outside_stairs = 0
    for first, second in zip(path, path[1:]):
        ax, ay, az = (float(v) for v in first[:3])
        bx, by, bz = (float(v) for v in second[:3])
        dz = bz - az
        if dz <= 1e-6:
            continue
        midpoint_x = (ax + bx) * 0.5
        midpoint_y = (ay + by) * 0.5
        midpoint_z = (az + bz) * 0.5
        level_index = int(math.floor((midpoint_z - body_height) / level_height + 1e-6))
        if not 0 <= level_index < len(stairs):
            if dz >= off_stair_major_rise:
                major_rises_outside_stairs += 1
            continue

        stair = stairs[level_index]
        x_low = min(stair["run_start_x"], stair["run_end_x"]) - 0.55
        x_high = max(stair["run_start_x"], stair["run_end_x"]) + 0.55
        in_stair_corridor = (
            x_low <= midpoint_x <= x_high and abs(midpoint_y - stair["stair_y"]) <= stair_half_width + 0.24
        )
        if not in_stair_corridor:
            if dz >= off_stair_major_rise:
                major_rises_outside_stairs += 1
            continue

        directed_dx = (bx - ax) * stair["direction"]
        stair["vertical_gain_m"] += dz
        stair["directed_run_m"] += max(0.0, directed_dx)
        stair["climbing_segments"] += 1
        if dz >= major_rise and directed_dx < -max(0.05, tread * 0.15):
            stair["reversed_major_segments"] += 1

    if major_rises_outside_stairs:
        blockers.append("major_climb_outside_stair_corridor")
    for stair in stairs:
        level = int(stair["level"])
        if stair["vertical_gain_m"] < level_height * 0.72:
            blockers.append(f"stair_{level}_missing_vertical_progress")
        if stair["directed_run_m"] < steps * tread * 0.65:
            blockers.append(f"stair_{level}_missing_horizontal_progress")
        if stair["reversed_major_segments"]:
            blockers.append(f"stair_{level}_climbs_against_expected_direction")
        stair["vertical_gain_m"] = round(float(stair["vertical_gain_m"]), 3)
        stair["directed_run_m"] = round(float(stair["directed_run_m"]), 3)

    return {"ok": not blockers, "blockers": blockers, "stairs": stairs}


def scene_acceptance(scene_preset: str, plan: dict[str, Any]) -> dict[str, Any]:
    if scene_preset not in MULTILEVEL_SCENE_PRESETS:
        return {"ok": True, "blockers": []}
    profile = stair_scene_profile(scene_preset)
    blockers: list[str] = []
    plan_ok = bool(plan.get("ok"))
    reached_goal = bool(plan.get("reached_goal"))
    if bool(profile.get("blocked")) and plan_ok and reached_goal:
        blockers.append("blocked_scene_unexpectedly_reached_goal")
    if not bool(profile.get("blocked")) and not plan_ok:
        blockers.append("traversable_scene_failed_to_plan")
    if not bool(profile.get("blocked")) and plan_ok and not reached_goal:
        blockers.append("path_did_not_reach_goal")
    topology = {"ok": True, "blockers": [], "stairs": []}
    if plan_ok:
        for a, b in zip(plan.get("path", []), plan.get("path", [])[1:]):
            ax, ay, az = (float(v) for v in a[:3])
            bx, by, bz = (float(v) for v in b[:3])
            dz = abs(bz - az)
            if dz <= 1e-6:
                continue
            if dz > STAIRS3D_STEP_MAX_M + 1e-6:
                blockers.append("path_exceeds_step_height")
                break
            dxy = math.hypot(bx - ax, by - ay)
            if dxy <= 1e-6:
                blockers.append("path_has_vertical_floor_transition")
                break
            if (dz / dxy) > (STAIRS3D_MAX_SLOPE + 1e-6):
                blockers.append("path_exceeds_stair_slope")
                break
            midpoint = ((ax + bx) * 0.5, (ay + by) * 0.5, (az + bz) * 0.5)
            if _stair_center_band_violation(scene_preset, midpoint):
                blockers.append("path_leaves_stair_center_band")
                break
        topology = _multifloor_stair_topology(scene_preset, list(plan.get("path", [])))
        blockers.extend(str(item) for item in topology["blockers"] if item not in blockers)
    return {
        "ok": not blockers,
        "blockers": blockers,
        "profile": profile,
        "topology": topology,
        "expected": "reject_goal" if bool(profile.get("blocked")) else "reach_goal",
    }


def _stair_center_band_violation(scene_preset: str, point: tuple[float, float, float]) -> bool:
    """Return true when a climbing segment is on a stair but too close to the side."""

    x, y, z = point
    if scene_preset not in MULTILEVEL_SCENE_PRESETS:
        return False
    if _point_over_stair_landing(scene_preset, point):
        return False
    if _point_over_multifloor_deck(scene_preset, point):
        return False
    nearest: dict[str, Any] | None = None
    nearest_score = float("inf")
    for geom in _stairs3d_geoms(scene_preset):
        name = str(geom.get("name", ""))
        if not name.startswith("step_stack_"):
            continue
        gx, gy, gz = (float(v) for v in geom["pos"])
        sx, sy, sz = (float(v) for v in geom["size"])
        top_z = gz + sz
        if not (gx - sx - 0.20 <= x <= gx + sx + 0.20):
            continue
        if abs(y - gy) > sy + 0.40:
            continue
        z_score = abs(z - (top_z + 0.28))
        if z_score < nearest_score:
            nearest = geom
            nearest_score = z_score
    if nearest is None or nearest_score > 0.45:
        return False
    stair_y = float(nearest["pos"][1])
    safe_half_width = max(
        0.18,
        min(float(nearest["size"][1]), float(nearest["size"][1]) - STAIRS3D_EDGE_CLEARANCE_M),
    )
    return abs(y - stair_y) > safe_half_width + 0.09


def _point_over_multifloor_deck(scene_preset: str, point: tuple[float, float, float]) -> bool:
    """Return true when a body-reference point is above a normal floor deck."""

    x, y, z = point
    for geom in _stairs3d_geoms(scene_preset):
        name = str(geom.get("name", ""))
        if not name.startswith("floor_level_"):
            continue
        if geom["type"] != "box":
            continue
        gx, gy, gz = (float(v) for v in geom["pos"])
        sx, sy, sz = (float(v) for v in geom["size"])
        top_z = gz + sz
        if not (gx - sx - 0.12 <= x <= gx + sx + 0.12):
            continue
        if not (gy - sy - 0.12 <= y <= gy + sy + 0.12):
            continue
        if top_z + 0.05 <= z <= top_z + STAIRS3D_BODY_SUPPORT_DEPTH_M + 0.18:
            return True
    return False


def _point_over_stair_landing(scene_preset: str, point: tuple[float, float, float]) -> bool:
    x, y, z = point
    for geom in _stairs3d_geoms(scene_preset):
        name = str(geom.get("name", ""))
        if not name.startswith("floor_stair_"):
            continue
        if geom["type"] != "box":
            continue
        gx, gy, gz = (float(v) for v in geom["pos"])
        sx, sy, sz = (float(v) for v in geom["size"])
        top_z = gz + sz
        if not (gx - sx - 0.12 <= x <= gx + sx + 0.12):
            continue
        if not (gy - sy - 0.12 <= y <= gy + sy + 0.12):
            continue
        if top_z + 0.05 <= z <= top_z + STAIRS3D_BODY_SUPPORT_DEPTH_M + 0.18:
            return True
    return False


def filter_points_to_scene(
    points: list[tuple[float, float, float]],
    *,
    length: float,
    width: float,
    margin: float = 0.5,
    z_min: float = -0.2,
    z_max: float = 1.2,
    x_min: float | None = None,
    x_max: float | None = None,
    y_min: float | None = None,
    y_max: float | None = None,
) -> list[tuple[float, float, float]]:
    x_min = -margin if x_min is None else x_min
    x_max = length + margin if x_max is None else x_max
    y_min = -(width / 2.0 + margin) if y_min is None else y_min
    y_max = width / 2.0 + margin if y_max is None else y_max
    return [(x, y, z) for x, y, z in points if x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max]


def building_scene_clip(margin: float = 0.5) -> dict[str, float]:
    x_min = y_min = z_min = float("inf")
    x_max = y_max = z_max = float("-inf")
    for geom in _building_map_geoms():
        x, y, z = geom["pos"]
        size = geom["size"]
        if geom["type"] == "cylinder":
            radius, half_height = size[:2]
            bounds = (x - radius, x + radius, y - radius, y + radius, z - half_height, z + half_height)
        else:
            hx, hy, hz = size
            bounds = (x - hx, x + hx, y - hy, y + hy, z - hz, z + hz)
        gx0, gx1, gy0, gy1, gz0, gz1 = bounds
        x_min, x_max = min(x_min, gx0), max(x_max, gx1)
        y_min, y_max = min(y_min, gy0), max(y_max, gy1)
        z_min, z_max = min(z_min, gz0), max(z_max, gz1)
    return {
        "x_min": x_min - margin,
        "x_max": x_max + margin,
        "y_min": y_min - margin,
        "y_max": y_max + margin,
        "z_min": z_min - margin,
        "z_max": z_max + margin,
        "margin": margin,
    }


def collect_mujoco_lidar_points(
    scene_xml: Path,
    *,
    start: list[float] | None = None,
    scans: int,
    duration_s: float,
    timeout_s: float,
    vx: float,
    wz: float,
    publish_hz: float,
    mid360_pattern: Path | None,
    mid360_samples_per_frame: int,
    lidar_backend: str,
    mujoco_lidar_backend: str,
    allow_legacy_lidar_fallback: bool,
    mujoco_memory: str,
    trajectory_support_radius_m: float = 0.0,
    trajectory_support_spacing_m: float = 0.1,
    mapping_trajectory: list[list[float]] | None = None,
) -> tuple[list[tuple[float, float, float]], dict[str, Any]]:
    from sim.engine.core.engine import VelocityCommand

    from drivers.sim.mujoco.runtime import (
        DEFAULT_MID360_PATTERN,
        build_engine,
    )

    engine = None
    clouds: list[Any] = []
    odoms: list[Any] = []
    backend_report: dict[str, Any] = {}
    try:
        engine = build_engine(
            world=scene_xml.resolve(),
            drive_mode="kinematic",
            start=start,
            mujoco_memory=mujoco_memory,
            mid360_pattern=mid360_pattern or DEFAULT_MID360_PATTERN,
            mid360_samples_per_frame=int(mid360_samples_per_frame),
            lidar_backend=lidar_backend,
            mujoco_lidar_backend=mujoco_lidar_backend,
            require_product_lidar_backend=not bool(allow_legacy_lidar_fallback),
        )
        backend_report = engine.get_lidar_backend_report()
    except Exception as exc:
        if engine is not None:
            engine.close()
        return [], {
            "ok": False,
            "scan_count": 0,
            "point_count": 0,
            "error": f"{type(exc).__name__}: {exc}",
            "source": "build_engine.get_lidar_points",
            "requested_backend": str(lidar_backend),
            "mujoco_lidar_backend": str(mujoco_lidar_backend),
            "allow_legacy_lidar_fallback": bool(allow_legacy_lidar_fallback),
        }

    cmd = VelocityCommand(linear_x=float(vx), linear_y=0.0, angular_z=float(wz))
    scan_period_s = 1.0 / max(1e-6, float(publish_hz))
    next_scan_s = 0.0
    scan_attempts = 0
    started = time.time()
    deadline = started + max(0.1, timeout_s)
    trajectory = list(mapping_trajectory or [])
    trajectory_complete = False
    try:
        if trajectory:
            for pose in trajectory:
                if time.time() >= deadline:
                    break
                yaw = float(pose[3])
                engine.set_robot_pose(
                    np.asarray(pose[:3], dtype=np.float64),
                    np.asarray([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)], dtype=np.float64),
                )
                state = engine.get_robot_state()
                odoms.append(state)
                cloud = engine.get_lidar_points()
                scan_attempts += 1
                if cloud is not None and len(cloud) > 0:
                    clouds.append(cloud)
            trajectory_complete = scan_attempts == len(trajectory)
        else:
            while time.time() < deadline and (
                scan_attempts < max(1, scans) or float(engine.sim_time) < max(0.0, duration_s)
            ):
                state = engine.step(cmd)
                odoms.append(state)
                if float(engine.sim_time) + 1e-9 < next_scan_s:
                    continue
                cloud = engine.get_lidar_points()
                scan_attempts += 1
                next_scan_s += scan_period_s
                if cloud is not None and len(cloud) > 0:
                    clouds.append(cloud)
    finally:
        engine.step(VelocityCommand())
        engine.close()

    points: list[tuple[float, float, float]] = []
    for cloud in clouds:
        for point in cloud:
            if len(point) >= 3:
                points.append((float(point[0]), float(point[1]), float(point[2])))
    support_points: list[tuple[float, float, float]] = []
    support_radius = max(0.0, float(trajectory_support_radius_m))
    support_spacing = max(0.02, float(trajectory_support_spacing_m))
    if support_radius > 0.0:
        offsets = _support_offsets(support_radius, support_spacing)
        seen_support: set[tuple[int, int, int]] = set()
        for state in odoms:
            try:
                x = float(state.position[0])
                y = float(state.position[1])
                z = float(state.position[2]) - 0.5
            except Exception:
                continue
            for dx, dy in offsets:
                px = x + dx
                py = y + dy
                pz = z
                key = (
                    int(round(px / support_spacing)),
                    int(round(py / support_spacing)),
                    int(round(pz / support_spacing)),
                )
                if key in seen_support:
                    continue
                seen_support.add(key)
                support_points.append((px, py, pz))
        points.extend(support_points)
    last_odom = odoms[-1] if odoms else None
    return points, {
        "ok": bool(points),
        "scan_count": scan_attempts,
        "nonempty_scan_count": len(clouds),
        "point_count": len(points),
        "lidar_point_count": sum(len(cloud) for cloud in clouds),
        "trajectory_support_point_count": len(support_points),
        "trajectory_support_radius_m": support_radius,
        "trajectory_support_spacing_m": support_spacing,
        "mapping_trajectory_mode": "ground_truth_stair_centerline" if trajectory else "velocity_command",
        "mapping_trajectory_pose_count": len(trajectory),
        "mapping_trajectory_completed": trajectory_complete if trajectory else True,
        "mapping_trajectory_min_z_m": min((float(pose[2]) for pose in trajectory), default=None),
        "mapping_trajectory_max_z_m": max((float(pose[2]) for pose in trajectory), default=None),
        "odom_count": len(odoms),
        "final_pose": (
            {
                "x": float(last_odom.position[0]),
                "y": float(last_odom.position[1]),
                "z": float(last_odom.position[2]),
                "frame_id": "odom",
            }
            if last_odom is not None
            else None
        ),
        "cmd_vx": float(vx),
        "cmd_wz": float(wz),
        "duration_s": float(duration_s),
        "timeout_s": float(timeout_s),
        "publish_hz": float(publish_hz),
        "source": "build_engine.get_lidar_points",
        "lidar_backend": backend_report,
        "requested_backend": str(lidar_backend),
        "mujoco_lidar_backend": str(mujoco_lidar_backend),
        "allow_legacy_lidar_fallback": bool(allow_legacy_lidar_fallback),
        "mid360_pattern": str(mid360_pattern or DEFAULT_MID360_PATTERN),
        "mid360_samples_per_frame": int(mid360_samples_per_frame),
    }


def stairs3d_scene_clip(margin: float = 0.5, scene_preset: str = "stairs3d") -> dict[str, float]:
    x_min = y_min = z_min = float("inf")
    x_max = y_max = z_max = float("-inf")
    for geom in _stairs3d_geoms(scene_preset):
        x, y, z = geom["pos"]
        if geom["type"] == "cylinder":
            radius, half_height = geom["size"][:2]
            bounds = (x - radius, x + radius, y - radius, y + radius, z - half_height, z + half_height)
        else:
            hx, hy, hz = geom["size"]
            bounds = (x - hx, x + hx, y - hy, y + hy, z - hz, z + hz)
        gx0, gx1, gy0, gy1, gz0, gz1 = bounds
        x_min, x_max = min(x_min, gx0), max(x_max, gx1)
        y_min, y_max = min(y_min, gy0), max(y_max, gy1)
        z_min, z_max = min(z_min, gz0), max(z_max, gz1)
    return {
        "x_min": x_min - margin,
        "x_max": x_max + margin,
        "y_min": y_min - margin,
        "y_max": y_max + margin,
        "z_min": z_min - margin,
        "z_max": z_max + margin,
        "margin": margin,
    }


def _support_offsets(radius_m: float, spacing_m: float) -> list[tuple[float, float]]:
    offsets: list[tuple[float, float]] = []
    steps = int(math.ceil(radius_m / spacing_m))
    for ix in range(-steps, steps + 1):
        for iy in range(-steps, steps + 1):
            dx = float(ix) * spacing_m
            dy = float(iy) * spacing_m
            if math.hypot(dx, dy) <= radius_m + 1e-9:
                offsets.append((dx, dy))
    return offsets


def _finite_triplet(values: list[float], name: str) -> list[float]:
    if len(values) != 3 or not all(math.isfinite(float(v)) for v in values):
        raise ValueError(f"{name} must contain exactly three finite numbers")
    return [float(v) for v in values]


def planner_constraint_overrides(args: argparse.Namespace) -> dict[str, Any]:
    overrides: dict[str, Any] = {}
    if args.planner_no_ground_support:
        overrides["require_ground_support"] = False
    if args.planner_clearance_cells >= 0:
        overrides["obstacle_clearance_radius_cells"] = args.planner_clearance_cells
        if args.planner_clearance_cells == 0:
            overrides["obstacle_clearance_weight"] = 0.0
    if args.planner_preblocked_cells >= 0:
        overrides["preblocked_costmap_radius_cells"] = args.planner_preblocked_cells
        overrides["enable_preblocked_costmap"] = args.planner_preblocked_cells > 0
    if args.planner_robot_radius > 0:
        overrides["robot_radius"] = args.planner_robot_radius
    if getattr(args, "planner_goal_tolerance_m", 0.0) > 0:
        overrides["terminal_goal_tolerance_m"] = args.planner_goal_tolerance_m
    if getattr(args, "planner_goal_xy_tolerance_m", 0.0) > 0:
        overrides["terminal_goal_xy_tolerance_m"] = args.planner_goal_xy_tolerance_m
    if getattr(args, "planner_goal_z_tolerance_m", 0.0) > 0:
        overrides["terminal_goal_z_tolerance_m"] = args.planner_goal_z_tolerance_m
    if getattr(args, "planner_max_step_height_m", 0.0) > 0:
        overrides["max_step_height"] = args.planner_max_step_height_m
    elif getattr(args, "scene_preset", "") in MULTILEVEL_SCENE_PRESETS:
        overrides["max_step_height"] = STAIRS3D_STEP_MAX_M
    if getattr(args, "planner_max_slope", 0.0) > 0:
        overrides["max_slope"] = args.planner_max_slope
    elif getattr(args, "scene_preset", "") in MULTILEVEL_SCENE_PRESETS:
        overrides["max_slope"] = STAIRS3D_MAX_SLOPE
    if getattr(args, "scene_preset", "") in MULTILEVEL_SCENE_PRESETS:
        # A route point represents the robot body above a real support surface.
        # Nearby walls and floor edges must never substitute for support in the
        # same XY column.
        overrides["strict_direct_ground_support"] = True
        overrides["ground_support_xy_radius_cells"] = 0
        resolution = float(getattr(args, "resolution", 0.18))
        overrides["ground_support_depth_cells"] = max(
            1,
            int(math.ceil(STAIRS3D_BODY_SUPPORT_DEPTH_M / max(1e-6, resolution))),
        )
        overrides["support_height_m"] = STAIRS3D_BODY_REFERENCE_HEIGHT_M
        # Keep one body-reference layer per physical support surface. The
        # global lowest-only layer is invalid for stacked floors that overlap
        # in XY because it deletes every upper floor in the same column.
        overrides["support_height_tolerance_m"] = max(0.04, resolution * 0.55)
        overrides["support_patch_radius_cells"] = 0
        overrides["support_patch_min_samples"] = 0
        overrides["body_clearance_below_m"] = 0.18
        overrides["body_clearance_above_m"] = 0.30
        overrides["lowest_traversable_only"] = False
    return overrides


def effective_support_dilation_cells(args: argparse.Namespace, scene_preset: str) -> int:
    requested = int(getattr(args, "support_dilation_cells", -1))
    if requested >= 0:
        return requested
    if scene_preset in MULTILEVEL_SCENE_PRESETS:
        return 0
    return 1


def effective_free_layers_above(args: argparse.Namespace, scene_preset: str) -> int:
    requested = int(getattr(args, "free_layers_above", -1))
    if requested >= 0:
        return requested
    if scene_preset in MULTILEVEL_SCENE_PRESETS:
        resolution = max(1e-6, float(getattr(args, "resolution", 0.18)))
        return max(1, int(math.ceil(STAIRS3D_BODY_REFERENCE_HEIGHT_M / resolution)))
    return 3


def _same_triplet(values: list[float], expected: list[float]) -> bool:
    if len(values) != 3:
        return False
    return all(abs(float(a) - float(b)) <= 1e-9 for a, b in zip(values, expected))


def effective_start_goal(args: argparse.Namespace) -> tuple[list[float], list[float]]:
    start = _finite_triplet(list(args.start), "start")
    goal = _finite_triplet(list(args.goal), "goal")
    scene_preset = getattr(args, "scene_preset", "corridor")
    if scene_preset == "building":
        if _same_triplet(start, CORRIDOR_DEFAULT_START):
            start = list(BUILDING_DEFAULT_START)
        if _same_triplet(goal, CORRIDOR_DEFAULT_GOAL):
            goal = list(BUILDING_DEFAULT_GOAL)
    elif scene_preset == "industrial_park":
        if _same_triplet(start, CORRIDOR_DEFAULT_START):
            start = list(INDUSTRIAL_PARK_DEFAULT_START)
        if _same_triplet(goal, CORRIDOR_DEFAULT_GOAL):
            goal = list(INDUSTRIAL_PARK_DEFAULT_GOAL)
    elif scene_preset in MULTILEVEL_SCENE_PRESETS:
        if _same_triplet(start, CORRIDOR_DEFAULT_START):
            start = stair_scene_default_start_goal(scene_preset)[0]
        if _same_triplet(goal, CORRIDOR_DEFAULT_GOAL):
            goal = stair_scene_default_start_goal(scene_preset)[1]
    return start, goal


def run_gate(args: argparse.Namespace) -> dict[str, Any]:
    scene_preset = getattr(args, "scene_preset", "corridor")
    out_dir = Path(args.out_dir or ROOT / "artifacts" / "mujoco_saved_map_plan_gate" / time.strftime("%Y%m%d_%H%M%S"))
    map_dir = out_dir / "same_source_map"
    map_dir.mkdir(parents=True, exist_ok=True)
    scene_xml = out_dir / "scene.xml"
    map_pcd = map_dir / "map.pcd"

    generate_scene_xml(scene_xml, length=args.length, width=args.width, scene_preset=scene_preset)
    scan_report: dict[str, Any] = {"source": args.map_source, "scene_preset": scene_preset}
    if args.map_source == "mujoco_lidar":
        map_start, _ = effective_start_goal(args)
        mapping_trajectory = (
            multifloor_mapping_trajectory(scene_preset) if scene_preset == "multifloor_stack_3" else None
        )
        points, scan_report = collect_mujoco_lidar_points(
            scene_xml,
            start=map_start,
            scans=args.lidar_scans,
            duration_s=args.lidar_duration,
            timeout_s=args.lidar_timeout,
            vx=args.lidar_vx,
            wz=args.lidar_wz,
            publish_hz=getattr(args, "lidar_publish_hz", 10.0),
            mid360_pattern=getattr(args, "mid360_pattern", None),
            mid360_samples_per_frame=getattr(args, "mid360_samples_per_frame", 15000),
            lidar_backend=getattr(args, "lidar_backend", "mujoco_lidar"),
            mujoco_lidar_backend=getattr(args, "mujoco_lidar_backend", "cpu"),
            allow_legacy_lidar_fallback=getattr(args, "allow_legacy_lidar_fallback", False),
            mujoco_memory=getattr(args, "mujoco_memory", "64M"),
            trajectory_support_radius_m=getattr(args, "trajectory_support_radius_m", 0.0),
            trajectory_support_spacing_m=getattr(args, "trajectory_support_spacing_m", 0.1),
            mapping_trajectory=mapping_trajectory,
        )
        raw_point_count = len(points)
        clip: dict[str, Any] = {"length": args.length, "width": args.width}
        if scene_preset == "building":
            clip.update(building_scene_clip())
        elif scene_preset in MULTILEVEL_SCENE_PRESETS:
            clip.update(stairs3d_scene_clip(scene_preset=scene_preset))
        points = filter_points_to_scene(points, **clip)
        scan_report["raw_point_count"] = raw_point_count
        scan_report["clipped_point_count"] = len(points)
        scan_report["scene_preset"] = scene_preset
        scan_report["scene_clip"] = clip
    elif scene_preset == "building":
        points = generate_building_points(
            spacing=args.spacing,
            hits_per_cell=args.hits_per_cell,
        )
    elif scene_preset == "industrial_park":
        points = generate_industrial_park_points(
            spacing=args.spacing,
            hits_per_cell=args.hits_per_cell,
        )
    elif scene_preset in MULTILEVEL_SCENE_PRESETS:
        points = generate_stairs3d_points(
            spacing=args.spacing,
            hits_per_cell=args.hits_per_cell,
            scene_preset=scene_preset,
        )
    else:
        points = generate_points(
            length=args.length,
            width=args.width,
            spacing=args.spacing,
            hits_per_cell=args.hits_per_cell,
        )
    write_ascii_pcd(map_pcd, points)

    lidar_map_source = args.map_source == "mujoco_lidar"
    build = _build_octomap_native(
        map_root=map_dir.parent,
        map_id=map_dir.name,
        args=args,
        scene_preset=scene_preset,
        lidar_map_source=lidar_map_source,
    )
    build["ok"] = build.get("success") is True
    artifact_gate = _validate_map_native(map_dir.parent, map_dir.name)

    plan: dict[str, Any] = {"skipped": bool(args.skip_plan)}
    if artifact_gate.get("ok") is True and not args.skip_plan:
        start, goal = effective_start_goal(args)
        plan = _plan_native(
            map_path=map_dir / "octomap.ot",
            start=start,
            goal=goal,
            constraints=planner_constraint_overrides(args),
            executable=str(args.planner_executable or ""),
            timeout_s=float(args.planner_timeout),
        )

    acceptance = scene_acceptance(scene_preset, plan)
    ok = (
        build.get("ok") is True
        and artifact_gate.get("ok") is True
        and acceptance.get("ok") is True
        and (bool(args.skip_plan) or plan.get("ok") is True)
    )
    return {
        "schema_version": "lingtu.mujoco_saved_map_plan_gate.v1",
        "ok": ok,
        "out_dir": str(out_dir),
        "scene_preset": scene_preset,
        "scene_xml": str(scene_xml),
        "map_dir": str(map_dir),
        "map_pcd": str(map_pcd),
        "point_count": len(points),
        "start": effective_start_goal(args)[0],
        "goal": effective_start_goal(args)[1],
        "scene_profile": stair_scene_profile(scene_preset) if scene_preset in MULTILEVEL_SCENE_PRESETS else None,
        "scan": scan_report,
        "build": build,
        "artifact_gate": artifact_gate,
        "plan": plan,
        "scene_acceptance": acceptance,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
    }


def build_parser() -> argparse.ArgumentParser:
    from drivers.sim.mujoco.runtime import (
        DEFAULT_MID360_PATTERN,
        DEFAULT_MID360_SAMPLES_PER_FRAME,
    )

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", default="")
    parser.add_argument(
        "--scene-preset",
        choices=SCENE_PRESETS,
        default="corridor",
        help=(
            "corridor keeps the tiny gate map; building reuses "
            "sim/worlds/mujoco/building_scene.xml; stair_* and multifloor_* "
            "are named multilevel acceptance scenes for OctoPlanner3D validation"
        ),
    )
    parser.add_argument("--length", type=float, default=3.0)
    parser.add_argument("--width", type=float, default=1.8)
    parser.add_argument("--spacing", type=float, default=0.2)
    parser.add_argument("--hits-per-cell", type=int, default=4)
    parser.add_argument(
        "--map-source",
        choices=("synthetic_hits", "mujoco_lidar"),
        default="synthetic_hits",
    )
    parser.add_argument("--lidar-scans", type=int, default=3)
    parser.add_argument("--lidar-duration", type=float, default=3.0)
    parser.add_argument("--lidar-timeout", type=float, default=8.0)
    parser.add_argument("--lidar-vx", type=float, default=0.2)
    parser.add_argument("--lidar-wz", type=float, default=0.0)
    parser.add_argument("--lidar-publish-hz", type=float, default=10.0)
    parser.add_argument("--trajectory-support-radius-m", type=float, default=0.0)
    parser.add_argument("--trajectory-support-spacing-m", type=float, default=0.1)
    parser.add_argument("--mujoco-memory", default="64M")
    parser.add_argument("--mid360-pattern", type=Path, default=DEFAULT_MID360_PATTERN)
    parser.add_argument("--mid360-samples-per-frame", type=int, default=DEFAULT_MID360_SAMPLES_PER_FRAME)
    parser.add_argument("--lidar-backend", choices=("mujoco_lidar", "ray_caster_lidar"), default="mujoco_lidar")
    parser.add_argument("--mujoco-lidar-backend", choices=("cpu", "taichi", "warp", "jax"), default="cpu")
    parser.add_argument("--allow-legacy-lidar-fallback", action="store_true")
    parser.add_argument("--resolution", type=float, default=0.2)
    parser.add_argument(
        "--support-dilation-cells",
        type=int,
        default=-1,
        help="OctoMap support dilation. Default: 0 for multi-level stair scenes, 1 otherwise.",
    )
    parser.add_argument(
        "--free-layers-above",
        type=int,
        default=-1,
        help="Free-space envelope layers; auto-scales from robot body height when negative.",
    )
    parser.add_argument("--free-dilation-cells", type=int, default=1)
    parser.add_argument("--converter", default="")
    parser.add_argument("--no-env-converter", action="store_true")
    parser.add_argument("--converter-timeout", type=float, default=60.0)
    parser.add_argument("--planner-executable", default="")
    parser.add_argument("--planner-timeout", type=float, default=30.0)
    parser.add_argument("--planner-no-ground-support", action="store_true")
    parser.add_argument("--planner-clearance-cells", type=int, default=-1)
    parser.add_argument("--planner-preblocked-cells", type=int, default=-1)
    parser.add_argument("--planner-robot-radius", type=float, default=0.0)
    parser.add_argument("--planner-goal-tolerance-m", type=float, default=0.0)
    parser.add_argument("--planner-goal-xy-tolerance-m", type=float, default=0.0)
    parser.add_argument("--planner-goal-z-tolerance-m", type=float, default=0.0)
    parser.add_argument(
        "--planner-max-step-height-m",
        type=float,
        default=0.0,
        help="Override OctoPlanner3D max_step_height. Multilevel scenes default to 0.23m.",
    )
    parser.add_argument(
        "--planner-max-slope",
        type=float,
        default=0.0,
        help="Override OctoPlanner3D dz/dxy slope limit. Multilevel scenes default to 0.65.",
    )
    parser.add_argument("--start", nargs=3, type=float, default=CORRIDOR_DEFAULT_START)
    parser.add_argument("--goal", nargs=3, type=float, default=CORRIDOR_DEFAULT_GOAL)
    parser.add_argument("--skip-plan", action="store_true")
    parser.add_argument("--json-out", default="")
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    report = run_gate(args)
    text = json.dumps(report, indent=2, sort_keys=True)
    print(text)
    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n", encoding="utf-8")
    return 0 if report["ok"] or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())
