#!/usr/bin/env python3
"""MuJoCo geometry gate for the native teleop_avoid safety arbiter.

The production arbiter lives in:
  src/nav/cpp/endpoint/teleop_safety.cpp

This script uses MuJoCo for the scene geometry and samples obstacle points from
MuJoCo geoms. The decision logic mirrors the C++ arbiter and reads its default
thresholds from the C++ config contract so the acceptance report can catch
obvious drift.
It is a simulation gate, not a replacement for the C++ endpoint/DDS loopback.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

import mujoco

ROOT = Path(__file__).resolve().parents[3]
TELEOP_SAFETY_CPP = ROOT / "src/nav/cpp/endpoint/teleop_safety.cpp"
ENDPOINT_CONFIG_HPP = ROOT / "src/nav/cpp/endpoint/nav_endpoint_config.hpp"


@dataclass(frozen=True)
class TeleopConfig:
    teleop_max_speed_mps: float
    teleop_max_yaw_rate: float
    teleop_slow_distance_m: float
    teleop_stop_distance_m: float
    teleop_linear_slow_scale: float
    teleop_min_motion_speed_mps: float
    teleop_obstacle_height_min_m: float
    teleop_obstacle_height_max_m: float
    teleop_obstacle_margin_m: float
    teleop_traversability_hard_cost: float
    teleop_traversability_soft_cost: float
    vehicle_length_m: float
    vehicle_width_m: float


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0


@dataclass
class Twist:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


@dataclass
class Decision:
    cmd: Twist
    reason: str
    stopped: bool = False
    slowed: bool = False
    limited: bool = False
    obstacle_distance_m: float = -1.0
    traversability_cost: float = -1.0


@dataclass(frozen=True)
class CaseSpec:
    name: str
    obstacle_front_x: float | None
    obstacle_y: float = 0.0
    obstacle_height: float = 0.50
    obstacle_size_x: float = 0.12
    obstacle_size_y: float = 0.25
    use_traversability: bool = False
    expected_reason: str = "accepted"


def _extract_double(source: str, name: str) -> float:
    pattern = rf"double\s+{re.escape(name)}\s*\{{\s*([0-9.+\-eE]+)\s*\}}"
    match = re.search(pattern, source)
    if not match:
        raise RuntimeError(f"missing default {name} in {ENDPOINT_CONFIG_HPP}")
    return float(match.group(1))


def load_cpp_defaults() -> TeleopConfig:
    source = ENDPOINT_CONFIG_HPP.read_text(encoding="utf-8")
    return TeleopConfig(
        teleop_max_speed_mps=_extract_double(source, "teleop_max_speed_mps"),
        teleop_max_yaw_rate=_extract_double(source, "teleop_max_yaw_rate"),
        teleop_slow_distance_m=_extract_double(source, "teleop_slow_distance_m"),
        teleop_stop_distance_m=_extract_double(source, "teleop_stop_distance_m"),
        teleop_linear_slow_scale=_extract_double(source, "teleop_linear_slow_scale"),
        teleop_min_motion_speed_mps=_extract_double(source, "teleop_min_motion_speed_mps"),
        teleop_obstacle_height_min_m=_extract_double(source, "teleop_obstacle_height_min_m"),
        teleop_obstacle_height_max_m=_extract_double(source, "teleop_obstacle_height_max_m"),
        teleop_obstacle_margin_m=_extract_double(source, "teleop_obstacle_margin_m"),
        teleop_traversability_hard_cost=_extract_double(source, "teleop_traversability_hard_cost"),
        teleop_traversability_soft_cost=_extract_double(source, "teleop_traversability_soft_cost"),
        vehicle_length_m=_extract_double(source, "vehicle_length_m"),
        vehicle_width_m=_extract_double(source, "vehicle_width_m"),
    )


def scene_xml(case: CaseSpec) -> str:
    obstacle = ""
    if case.obstacle_front_x is not None:
        sx = case.obstacle_size_x
        sy = case.obstacle_size_y
        sz = case.obstacle_height * 0.5
        cx = case.obstacle_front_x + sx
        obstacle = (
            f'<geom name="obstacle" type="box" pos="{cx:.3f} {case.obstacle_y:.3f} {sz:.3f}" '
            f'size="{sx:.3f} {sy:.3f} {sz:.3f}" rgba="0.9 0.2 0.1 1"/>'
        )
    hazard = ""
    if case.use_traversability:
        hazard = (
            '<geom name="terrain_hazard" type="box" pos="0.70 0.0 0.006" size="0.55 0.35 0.006" rgba="0.8 0.1 0.8 1"/>'
        )
    return f"""
<mujoco model="lingtu_teleop_avoid_gate">
  <option timestep="0.01" gravity="0 0 -9.81"/>
  <worldbody>
    <light pos="0 0 5"/>
    <geom name="floor" type="plane" size="3 2 0.05" rgba="0.75 0.78 0.72 1"/>
    <body name="robot" pos="0 0 0.25">
      <freejoint name="root"/>
      <geom name="robot_body" type="box" size="0.50 0.30 0.18" rgba="0.1 0.55 0.95 1"/>
    </body>
    {obstacle}
    {hazard}
  </worldbody>
</mujoco>
"""


def load_case_model(case: CaseSpec) -> tuple[mujoco.MjModel, mujoco.MjData, Path]:
    tmp = Path(tempfile.mkdtemp(prefix=f"lingtu_{case.name}_"))
    xml = tmp / "scene.xml"
    xml.write_text(scene_xml(case), encoding="utf-8")
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data, xml


def obstacle_points_from_mujoco(
    model: mujoco.MjModel, data: mujoco.MjData, case: CaseSpec
) -> list[tuple[float, float, float, float]]:
    if case.obstacle_front_x is None:
        return []
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "obstacle")
    if gid < 0:
        return []
    xpos = data.geom_xpos[gid]
    size = model.geom_size[gid]
    front_x = float(xpos[0] - size[0])
    y = float(xpos[1])
    z = float(xpos[2])
    height = float(xpos[2] + size[2])
    return [(front_x, y, z, height)]


def linear_speed(cmd: Twist) -> float:
    return math.hypot(cmd.vx, cmd.vy)


def limited_command(cfg: TeleopConfig, raw: Twist) -> tuple[Twist, bool]:
    out = Twist(raw.vx, raw.vy, raw.wz)
    limited = False
    speed = linear_speed(out)
    if speed > cfg.teleop_max_speed_mps and speed > 1e-6:
        scale = cfg.teleop_max_speed_mps / speed
        out.vx *= scale
        out.vy *= scale
        limited = True
    if abs(out.wz) > cfg.teleop_max_yaw_rate:
        out.wz = math.copysign(cfg.teleop_max_yaw_rate, out.wz)
        limited = True
    return out, limited


def point_to_body(pose: Pose2D, point: tuple[float, float, float, float]) -> tuple[float, float]:
    dx = point[0] - pose.x
    dy = point[1] - pose.y
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    return c * dx + s * dy, -s * dx + c * dy


def terrain_cost_ahead(case: CaseSpec, map_x: float, map_y: float) -> float:
    if not case.use_traversability:
        return -1.0
    if 0.15 <= map_x <= 1.25 and abs(map_y) <= 0.35:
        return 100.0
    return 0.0


def arbitrate(
    cfg: TeleopConfig,
    case: CaseSpec,
    pose: Pose2D,
    raw: Twist,
    obstacles: Iterable[tuple[float, float, float, float]],
) -> Decision:
    cmd, limited = limited_command(cfg, raw)
    speed = linear_speed(cmd)
    if speed < cfg.teleop_min_motion_speed_mps and abs(cmd.wz) < 1e-4:
        return Decision(cmd=cmd, reason="zero_limited" if limited else "zero", limited=limited)

    half_width = max(0.1, cfg.vehicle_width_m * 0.5 + cfg.teleop_obstacle_margin_m)
    linear_motion = speed >= cfg.teleop_min_motion_speed_mps
    dir_x = cmd.vx / speed if linear_motion else 1.0
    dir_y = cmd.vy / speed if linear_motion else 0.0
    min_ahead = math.inf

    for point in obstacles:
        height = point[3]
        if (
            not math.isfinite(height)
            or height < cfg.teleop_obstacle_height_min_m
            or height > cfg.teleop_obstacle_height_max_m
        ):
            continue
        bx, by = point_to_body(pose, point)
        ahead = bx * dir_x + by * dir_y
        lateral = -bx * dir_y + by * dir_x
        if ahead > 0.0 and ahead <= cfg.teleop_slow_distance_m and abs(lateral) <= half_width:
            min_ahead = min(min_ahead, ahead)

    if math.isfinite(min_ahead):
        if min_ahead <= cfg.teleop_stop_distance_m:
            return Decision(
                cmd=Twist(),
                reason="obstacle_stop",
                stopped=True,
                limited=limited,
                obstacle_distance_m=min_ahead,
            )
        cmd.vx *= cfg.teleop_linear_slow_scale
        cmd.vy *= cfg.teleop_linear_slow_scale
        return Decision(
            cmd=cmd,
            reason="obstacle_slow",
            slowed=True,
            limited=limited,
            obstacle_distance_m=min_ahead,
        )

    if case.use_traversability and linear_speed(cmd) >= cfg.teleop_min_motion_speed_mps:
        speed_after = linear_speed(cmd)
        dir_x = cmd.vx / speed_after
        dir_y = cmd.vy / speed_after
        step = 0.20
        horizon = max(cfg.teleop_stop_distance_m, cfg.teleop_slow_distance_m)
        max_cost = -1.0
        for i in range(1, int(horizon / step) + 2):
            distance = i * step
            bx = dir_x * distance
            by = dir_y * distance
            mx = pose.x + bx
            my = pose.y + by
            max_cost = max(max_cost, terrain_cost_ahead(case, mx, my))
        if max_cost >= cfg.teleop_traversability_hard_cost:
            return Decision(
                cmd=Twist(),
                reason="terrain_stop",
                stopped=True,
                limited=limited,
                traversability_cost=max_cost,
            )
        if max_cost >= cfg.teleop_traversability_soft_cost:
            cmd.vx *= cfg.teleop_linear_slow_scale
            cmd.vy *= cfg.teleop_linear_slow_scale
            return Decision(
                cmd=cmd,
                reason="terrain_slow",
                slowed=True,
                limited=limited,
                traversability_cost=max_cost,
            )

    return Decision(cmd=cmd, reason="limited" if limited else "accepted", limited=limited)


def simulate_case(cfg: TeleopConfig, case: CaseSpec, dt: float, steps: int) -> dict:
    model, data, xml = load_case_model(case)
    pose = Pose2D()
    raw = Twist(vx=0.18)
    obstacles = obstacle_points_from_mujoco(model, data, case)
    decisions: list[Decision] = []
    for _ in range(steps):
        decision = arbitrate(cfg, case, pose, raw, obstacles)
        decisions.append(decision)
        pose.x += decision.cmd.vx * dt
        pose.y += decision.cmd.vy * dt
        pose.yaw += decision.cmd.wz * dt
        if model.nq >= 7:
            data.qpos[0] = pose.x
            data.qpos[1] = pose.y
            data.qpos[2] = 0.25
            data.qpos[3] = math.cos(pose.yaw * 0.5)
            data.qpos[4] = 0.0
            data.qpos[5] = 0.0
            data.qpos[6] = math.sin(pose.yaw * 0.5)
        mujoco.mj_forward(model, data)

    first = decisions[0]
    return {
        "case": case.name,
        "scene_xml": str(xml),
        "obstacle_points_xyzh": obstacles,
        "expected_reason": case.expected_reason,
        "first_decision": {
            "reason": first.reason,
            "cmd": asdict(first.cmd),
            "stopped": first.stopped,
            "slowed": first.slowed,
            "limited": first.limited,
            "obstacle_distance_m": first.obstacle_distance_m,
            "traversability_cost": first.traversability_cost,
        },
        "final_pose": asdict(pose),
        "samples": len(decisions),
        "min_vx": min(d.cmd.vx for d in decisions),
        "max_vx": max(d.cmd.vx for d in decisions),
        "stop_samples": sum(1 for d in decisions if d.stopped),
        "slow_samples": sum(1 for d in decisions if d.slowed),
    }


def evaluate_report(results: list[dict]) -> list[str]:
    blockers: list[str] = []
    by_name = {r["case"]: r for r in results}
    for result in results:
        reason = result["first_decision"]["reason"]
        expected = result["expected_reason"]
        if reason != expected:
            blockers.append(f"{result['case']} expected {expected}, got {reason}")

    if by_name["no_obstacle"]["final_pose"]["x"] <= 0.30:
        blockers.append("no_obstacle did not move far enough")
    if by_name["slow_obstacle"]["first_decision"]["cmd"]["vx"] >= 0.10:
        blockers.append("slow_obstacle did not reduce vx")
    if by_name["stop_obstacle"]["final_pose"]["x"] > 0.01:
        blockers.append("stop_obstacle moved despite stop gate")
    if by_name["side_obstacle"]["first_decision"]["reason"] != "accepted":
        blockers.append("side_obstacle should not block forward command")
    if by_name["low_obstacle"]["first_decision"]["reason"] != "accepted":
        blockers.append("low_obstacle should be ignored by height gate")
    if by_name["terrain_hard"]["first_decision"]["reason"] != "terrain_stop":
        blockers.append("terrain_hard should stop on high traversability cost")
    return blockers


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--artifact-dir",
        type=Path,
        default=ROOT / "artifacts/mujoco_teleop_avoid_gate",
    )
    parser.add_argument("--json-out", type=Path, default=None)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--duration-s", type=float, default=2.0)
    parser.add_argument("--strict", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cfg = load_cpp_defaults()
    steps = max(1, int(args.duration_s / args.dt))
    cases = [
        CaseSpec("no_obstacle", None, expected_reason="accepted"),
        CaseSpec("slow_obstacle", 0.90, expected_reason="obstacle_slow"),
        CaseSpec("stop_obstacle", 0.35, expected_reason="obstacle_stop"),
        CaseSpec("side_obstacle", 0.35, obstacle_y=0.65, expected_reason="accepted"),
        CaseSpec("low_obstacle", 0.35, obstacle_height=0.05, expected_reason="accepted"),
        CaseSpec("terrain_hard", None, use_traversability=True, expected_reason="terrain_stop"),
    ]
    results = [simulate_case(cfg, case, args.dt, steps) for case in cases]
    blockers = evaluate_report(results)
    report = {
        "schema_version": "lingtu.mujoco.teleop_avoid_gate.v1",
        "ok": not blockers,
        "blockers": blockers,
        "source_logic": str(TELEOP_SAFETY_CPP),
        "source_config": str(ENDPOINT_CONFIG_HPP),
        "mujoco_version": getattr(mujoco, "__version__", "unknown"),
        "config": asdict(cfg),
        "duration_s": args.duration_s,
        "dt": args.dt,
        "cases": results,
    }
    args.artifact_dir.mkdir(parents=True, exist_ok=True)
    out = args.json_out or args.artifact_dir / "report.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps({"ok": report["ok"], "blockers": blockers, "json": str(out)}, indent=2))
    return 1 if args.strict and blockers else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
