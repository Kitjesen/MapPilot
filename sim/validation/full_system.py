"""Server-side full-system simulation validation.

This module is intentionally simulation-only. It never opens Gateway command
routes, never publishes real robot topics, and never manages systemd services.
It collects reproducible evidence for scene richness, LiDAR simulation
readiness, SLAM/localization metric gates, navigation wiring, local planning,
frontier exploration, and person tracking.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Iterable


PASS = "pass"
FAIL = "fail"
BLOCKED = "blocked"

ENVIRONMENT_DEPENDENCY_MODULES = frozenset(
    {
        "cv2",
        "mujoco",
        "numpy",
        "onnxruntime",
        "scipy",
        "yaml",
    }
)
_NUMPY_IMPORT_SAFE: bool | None = None


@dataclass(frozen=True)
class ValidationCheck:
    name: str
    category: str
    status: str
    summary: str
    evidence: dict[str, Any] = field(default_factory=dict)
    duration_s: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ValidationReport:
    generated_at: float
    repo_root: str
    passed: bool
    checks: tuple[ValidationCheck, ...]

    @property
    def summary(self) -> dict[str, int]:
        counts = {PASS: 0, FAIL: 0, BLOCKED: 0}
        for check in self.checks:
            counts[check.status] = counts.get(check.status, 0) + 1
        return counts

    def to_dict(self) -> dict[str, Any]:
        return {
            "generated_at": self.generated_at,
            "repo_root": self.repo_root,
            "passed": self.passed,
            "summary": self.summary,
            "checks": [check.to_dict() for check in self.checks],
        }


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _missing_module_root(exc: ModuleNotFoundError) -> str:
    return str(getattr(exc, "name", "") or "").split(".", 1)[0]


def _module_not_found_check(
    name: str,
    category: str,
    exc: ModuleNotFoundError,
) -> ValidationCheck:
    missing = _missing_module_root(exc)
    status = BLOCKED if missing in ENVIRONMENT_DEPENDENCY_MODULES else FAIL
    summary = (
        f"missing Python dependency: {missing}"
        if status == BLOCKED and missing
        else f"{type(exc).__name__}: {exc}"
    )
    return ValidationCheck(
        name=name,
        category=category,
        status=status,
        summary=summary,
        evidence={
            "missing_module": missing,
            "exception": f"{type(exc).__name__}: {exc}",
        },
    )


def _timed(name: str, category: str, fn) -> ValidationCheck:
    started = time.perf_counter()
    try:
        check = fn()
    except ModuleNotFoundError as exc:
        check = _module_not_found_check(name, category, exc)
    except Exception as exc:
        check = ValidationCheck(
            name=name,
            category=category,
            status=FAIL,
            summary=f"{type(exc).__name__}: {exc}",
            evidence={},
        )
    elapsed = time.perf_counter() - started
    return ValidationCheck(
        name=check.name,
        category=check.category,
        status=check.status,
        summary=check.summary,
        evidence=check.evidence,
        duration_s=round(elapsed, 3),
    )


def _pos_z(geom: ET.Element) -> float:
    raw = geom.attrib.get("pos", "0 0 0").split()
    if len(raw) < 3:
        return 0.0
    try:
        return float(raw[2])
    except ValueError:
        return 0.0


def _geom_names(root: ET.Element) -> set[str]:
    return {geom.attrib.get("name", "") for geom in root.findall(".//geom")}


def _numpy():
    if not _numpy_import_is_safe():
        raise ModuleNotFoundError(
            "No module named 'numpy'",
            name="numpy",
        )
    import numpy as np

    return np


def _numpy_import_is_safe() -> bool:
    global _NUMPY_IMPORT_SAFE
    if _NUMPY_IMPORT_SAFE is None:
        try:
            probe = subprocess.run(
                [sys.executable, "-c", "import numpy"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=10,
            )
        except (OSError, subprocess.TimeoutExpired):
            _NUMPY_IMPORT_SAFE = False
        else:
            _NUMPY_IMPORT_SAFE = probe.returncode == 0
    return _NUMPY_IMPORT_SAFE


def validate_scene_catalog(repo_root: Path) -> list[ValidationCheck]:
    worlds = {
        "open_field": repo_root / "sim/worlds/mujoco/open_field.xml",
        "building_scene": repo_root / "sim/worlds/mujoco/building_scene.xml",
        "factory_scene": repo_root / "sim/worlds/mujoco/factory_scene.xml",
        "spiral_terrain": repo_root / "sim/worlds/mujoco/spiral_terrain.xml",
    }
    checks: list[ValidationCheck] = []
    missing = [name for name, path in worlds.items() if not path.exists()]
    checks.append(
        ValidationCheck(
            name="required_worlds_exist",
            category="scene",
            status=FAIL if missing else PASS,
            summary="required simulation worlds are present" if not missing else "missing worlds",
            evidence={
                "worlds": {name: str(path.relative_to(repo_root)) for name, path in worlds.items()},
                "missing": missing,
            },
        )
    )

    building = worlds["building_scene"]
    if not building.exists():
        checks.append(
            ValidationCheck(
                name="building_scene_multifloor_contract",
                category="scene",
                status=FAIL,
                summary="building_scene.xml is missing",
            )
        )
        return checks

    root = ET.parse(building).getroot()
    names = _geom_names(root)
    z_values = [_pos_z(geom) for geom in root.findall(".//geom")]
    step_count = sum(1 for name in names if name.startswith("step_"))
    second_floor_count = sum(1 for name in names if name.startswith("floor_2"))
    has_goal_2f = "goal_disk" in names and max(z_values or [0.0]) >= 3.5
    passed = step_count >= 10 and second_floor_count >= 2 and has_goal_2f
    checks.append(
        ValidationCheck(
            name="building_scene_multifloor_contract",
            category="scene",
            status=PASS if passed else FAIL,
            summary=(
                "building_scene has stairs, second-floor geometry, and elevated goal"
                if passed
                else "building_scene is not rich enough for multi-floor validation"
            ),
            evidence={
                "step_count": step_count,
                "second_floor_geom_count": second_floor_count,
                "max_geom_z": max(z_values or [0.0]),
                "has_goal_marker": "goal_disk" in names,
            },
        )
    )
    return checks


def validate_slam_localization_contract(repo_root: Path) -> ValidationCheck:
    from sim.evaluation.slam import TumPose, evaluate_trajectory, load_case

    case_path = repo_root / "sim/evaluation/slam/configs/nova_dog_fastlio2.json"
    case = load_case(case_path)
    errors = case.validate(repo_root=repo_root)

    reference = [
        TumPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        TumPose(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        TumPose(2.0, 2.0, 0.2, 0.0, 0.0, 0.0, 0.0, 1.0),
        TumPose(3.0, 3.0, 0.4, 0.0, 0.0, 0.0, 0.0, 1.0),
    ]
    estimate = [
        TumPose(0.005, 0.01, 0.00, 0.0, 0.0, 0.0, 0.0, 1.0),
        TumPose(1.005, 1.02, 0.01, 0.0, 0.0, 0.0, 0.0, 1.0),
        TumPose(2.005, 2.02, 0.18, 0.0, 0.0, 0.0, 0.0, 1.0),
        TumPose(3.005, 3.03, 0.38, 0.0, 0.0, 0.0, 0.0, 1.0),
    ]
    metrics = evaluate_trajectory(reference, estimate, max_dt=case.max_association_dt_s)
    metric_pass = metrics.coverage_ratio >= 0.99 and metrics.rmse_translation_error_m < 0.10
    passed = not errors and case.backend.name == "fastlio2" and metric_pass
    return ValidationCheck(
        name="slam_localization_metric_gate",
        category="slam_localization",
        status=PASS if passed else FAIL,
        summary=(
            "Fast-LIO2 simulation manifest and localization metric gate are valid"
            if passed
            else "SLAM/localization contract is not valid"
        ),
        evidence={
            "manifest": str(case_path.relative_to(repo_root)),
            "manifest_errors": errors,
            "backend": case.backend.name,
            "world": case.world,
            "metrics": metrics.to_dict(),
        },
    )


def validate_navigation_blueprint(repo_root: Path) -> ValidationCheck:
    from core.blueprints.profile_graph import graph_for_profile

    graph_kwargs = {
        "slam_profile": "none",
        "enable_semantic": False,
        "enable_gateway": False,
        "python_autonomy_backend": "simple",
        "python_path_follower_backend": "pid",
        "run_startup_checks": False,
    }
    graph = graph_for_profile(
        "sim",
        **graph_kwargs,
    )
    connections = {
        (
            wire.out_module,
            wire.out_port,
            wire.in_module,
            wire.in_port,
        )
        for wire in graph.explicit_wires
    }
    required = {
        ("MujocoDriverModule", "odometry", "NavigationModule", "odometry"),
        ("MujocoDriverModule", "map_cloud", "OccupancyGridModule", "map_cloud"),
        ("OccupancyGridModule", "costmap", "TraversabilityCostModule", "costmap"),
        ("TraversabilityCostModule", "fused_cost", "NavigationModule", "costmap"),
        ("NavigationModule", "waypoint", "LocalPlannerModule", "waypoint"),
        ("LocalPlannerModule", "local_path", "PathFollowerModule", "local_path"),
        ("PathFollowerModule", "cmd_vel", "CmdVelMux", "path_follower_cmd_vel"),
        ("CmdVelMux", "driver_cmd_vel", "MujocoDriverModule", "cmd_vel"),
    }
    missing = sorted(required - connections)
    blockers = list(missing)
    runtime_parity: dict[str, Any] = {
        "checked": False,
        "status": BLOCKED,
        "reason": "numpy import unavailable or unsafe",
    }
    if _numpy_import_is_safe():
        try:
            runtime_graph = graph_for_profile("sim", mode="runtime", **graph_kwargs)
            static_snapshot = graph.as_snapshot()
            runtime_snapshot = runtime_graph.as_snapshot()
            missing_runtime_modules = sorted(
                set(static_snapshot["modules"]) - set(runtime_snapshot["modules"])
            )
            extra_runtime_modules = sorted(
                set(runtime_snapshot["modules"]) - set(static_snapshot["modules"])
            )
            missing_runtime_wires = sorted(
                set(static_snapshot["explicit_wires"])
                - set(runtime_snapshot["explicit_wires"])
            )
            extra_runtime_wires = sorted(
                set(runtime_snapshot["explicit_wires"])
                - set(static_snapshot["explicit_wires"])
            )
            runtime_parity = {
                "checked": True,
                "status": PASS,
                "missing_runtime_modules": missing_runtime_modules,
                "extra_runtime_modules": extra_runtime_modules,
                "missing_runtime_wires": missing_runtime_wires,
                "extra_runtime_wires": extra_runtime_wires,
            }
            if (
                missing_runtime_modules
                or extra_runtime_modules
                or missing_runtime_wires
                or extra_runtime_wires
            ):
                runtime_parity["status"] = FAIL
                blockers.append("runtime profile graph differs from static profile graph")
        except ModuleNotFoundError as exc:
            missing_module = _missing_module_root(exc)
            parity_status = (
                BLOCKED if missing_module in ENVIRONMENT_DEPENDENCY_MODULES else FAIL
            )
            runtime_parity = {
                "checked": False,
                "status": parity_status,
                "reason": (
                    f"missing Python dependency: {missing_module}"
                    if parity_status == BLOCKED
                    else "runtime profile graph construction failed"
                ),
                "missing_module": missing_module,
                "exception": f"{type(exc).__name__}: {exc}",
            }
            if parity_status == FAIL:
                blockers.append("runtime profile graph construction failed")
        except Exception as exc:
            runtime_parity = {
                "checked": False,
                "status": FAIL,
                "reason": "runtime profile graph construction failed",
                "exception": f"{type(exc).__name__}: {exc}",
            }
            blockers.append("runtime profile graph construction failed")
    status = (
        FAIL
        if blockers
        else PASS
        if runtime_parity.get("checked") and runtime_parity.get("status") == PASS
        else BLOCKED
    )
    if blockers:
        summary = "simulation navigation wiring is incomplete"
    elif status == PASS:
        summary = "global planning, local planning, tracking, mux, and simulated driver are wired"
    else:
        summary = "simulation navigation static wiring is valid; runtime parity is blocked"
    return ValidationCheck(
        name="sim_nav_planning_wiring",
        category="navigation",
        status=status,
        summary=summary,
        evidence={
            "missing_connections": missing,
            "module_count": len(graph.modules),
            "connection_count": len(graph.explicit_wires),
            "repo_root": str(repo_root),
            "profile_graph_mode": "static",
            "runtime_parity": runtime_parity,
        },
    )


def validate_frontier_exploration_runtime() -> ValidationCheck:
    np = _numpy()

    from core.blueprints.profile_builder import build_system_for_profile
    from core.msgs.geometry import Pose
    from core.msgs.nav import Odometry

    system = build_system_for_profile("sim_nav", dict(
        robot="stub",
        slam_profile="none",
        planner_backend="astar",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=False,
        enable_frontier=True,
        python_autonomy_backend="simple",
        python_path_follower_backend="pid",
        frontier_min_size=1,
        frontier_safe_distance=0.0,
        frontier_goal_timeout=0.5,
        frontier_rate=10.0,
        run_startup_checks=False,
    ))
    explorer = system.get_module("WavefrontFrontierExplorer")
    nav = system.get_module("NavigationModule")

    seen = {"exploration_goals": 0, "global_paths": 0, "waypoints": 0}
    explorer.exploration_goal._add_callback(
        lambda _: seen.__setitem__("exploration_goals", seen["exploration_goals"] + 1)
    )
    nav.global_path._add_callback(
        lambda _: seen.__setitem__("global_paths", seen["global_paths"] + 1)
    )
    nav.waypoint._add_callback(lambda _: seen.__setitem__("waypoints", seen["waypoints"] + 1))

    grid = np.full((50, 50), -1, dtype=np.int16)
    grid[18:33, 18:33] = 0
    costmap = {
        "grid": grid,
        "resolution": 0.2,
        "origin": np.array([-5.0, -5.0], dtype=np.float32),
        "origin_x": -5.0,
        "origin_y": -5.0,
        "width": 50,
        "height": 50,
    }
    odom = Odometry(pose=Pose(0.0, 0.0, 0.0), frame_id="map")

    system.start()
    try:
        explorer.odometry._deliver(odom)
        nav.odometry._deliver(odom)
        explorer.costmap._deliver(costmap)
        nav.costmap._deliver(costmap)
        started = explorer.begin_exploration()
        deadline = time.time() + 3.0
        while time.time() < deadline and (seen["exploration_goals"] == 0 or seen["waypoints"] == 0):
            time.sleep(0.05)
    finally:
        explorer.end_exploration()
        system.stop()

    passed = started == "started" and all(value > 0 for value in seen.values())
    return ValidationCheck(
        name="frontier_exploration_to_navigation",
        category="exploration",
        status=PASS if passed else FAIL,
        summary=(
            "frontier exploration emits goals that reach NavigationModule"
            if passed
            else "frontier exploration did not close the goal-to-plan loop"
        ),
        evidence={"started": started, "seen": seen},
    )


def validate_person_tracking_runtime() -> ValidationCheck:
    np = _numpy()

    from sim.following.behavior import BehaviorConfig, BehaviorState, FollowingBehavior
    from sim.following.controller.pure_pursuit import PurePursuitFollower
    from sim.following.interfaces import PerceivedTarget
    from sim.following.task import TaskParser

    parser = TaskParser()
    task = parser.parse("follow the person in red")
    controller = PurePursuitFollower(target_distance=1.0, lookahead_distance=1.0)
    behavior = FollowingBehavior(
        BehaviorConfig(search_timeout_s=0.2, explore_timeout_s=0.5, recover_timeout_s=1.0),
        controller=controller,
    )
    behavior.set_task(task)

    robot_pos = np.array([0.0, 0.0, 0.0], dtype=float)
    target = PerceivedTarget(
        position_world=np.array([2.0, 0.0, 0.0], dtype=float),
        velocity_world=np.array([0.2, 0.0, 0.0], dtype=float),
        confidence=1.0,
    )
    follow_cmds = []
    for _ in range(5):
        cmd = behavior.update(robot_pos, 0.0, target, None, 0.1)
        follow_cmds.append(cmd)

    lost_states = []
    for _ in range(8):
        behavior.update(robot_pos, 0.0, None, None, 0.1)
        lost_states.append(behavior.state)

    passed = (
        task.type == "follow"
        and any(cmd.vx > 0.0 for cmd in follow_cmds)
        and BehaviorState.SEARCH in lost_states
        and BehaviorState.EXPLORE in lost_states
    )
    return ValidationCheck(
        name="person_tracking_behavior_loop",
        category="tracking",
        status=PASS if passed else FAIL,
        summary=(
            "person-following parser, controller, and loss escalation are wired"
            if passed
            else "person-following tracking loop did not exercise expected states"
        ),
        evidence={
            "task": asdict(task),
            "max_follow_vx": max((cmd.vx for cmd in follow_cmds), default=0.0),
            "lost_states": [state.value for state in lost_states],
        },
    )


def validate_mujoco_lidar_runtime(repo_root: Path, worlds: Iterable[str]) -> list[ValidationCheck]:
    if importlib.util.find_spec("mujoco") is None:
        return [
            ValidationCheck(
                name="mujoco_lidar_runtime",
                category="lidar",
                status=BLOCKED,
                summary="mujoco is not installed in this environment",
                evidence={"worlds": list(worlds)},
            )
        ]

    from drivers.sim.mujoco_driver_module import MujocoDriverModule

    checks: list[ValidationCheck] = []
    for world in worlds:
        def _run(world_name=world):
            driver = MujocoDriverModule(
                world=world_name,
                render=False,
                enable_camera=False,
                drive_mode="kinematic",
            )
            driver.setup()
            try:
                if driver._engine is None:
                    raise RuntimeError("MujocoDriverModule has no engine after setup")
                points = driver._engine.get_lidar_points()
                count = 0 if points is None else int(len(points))
                return ValidationCheck(
                    name=f"mujoco_lidar_{world_name}",
                    category="lidar",
                    status=PASS if count > 0 else FAIL,
                    summary=(
                        "MuJoCo LiDAR produced point cloud"
                        if count > 0
                        else "MuJoCo LiDAR returned an empty point cloud"
                    ),
                    evidence={
                        "world": world_name,
                        "point_count": count,
                        "scene": str((repo_root / "sim/worlds" / f"{world_name}.xml").relative_to(repo_root)),
                    },
                )
            finally:
                if driver._engine is not None:
                    driver._engine.close()
                    driver._engine = None

        checks.append(_timed(f"mujoco_lidar_{world}", "lidar", _run))
    return checks


def validate_mujoco_kinematic_nav_runtime(
    *,
    duration_s: float,
    goal_distance_m: float,
    min_motion_m: float,
) -> ValidationCheck:
    if importlib.util.find_spec("mujoco") is None:
        return ValidationCheck(
            name="mujoco_kinematic_nav_runtime",
            category="local_planning",
            status=BLOCKED,
            summary="mujoco is not installed in this environment",
        )

    from core.blueprints.profile_builder import build_system_for_profile
    from core.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3

    system = build_system_for_profile("sim", dict(
        robot="sim_mujoco",
        world="open_field",
        slam_profile="none",
        detector="sim_scene",
        llm="mock",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        render=False,
        python_autonomy_backend="simple",
        python_path_follower_backend="pid",
        drive_mode="kinematic",
        waypoint_threshold=0.35,
        downsample_dist=0.5,
        run_startup_checks=False,
    ))

    driver = system.get_module("MujocoDriverModule")
    ogm = system.get_module("OccupancyGridModule")
    nav = system.get_module("NavigationModule")
    local_planner = system.get_module("LocalPlannerModule")
    path_follower = system.get_module("PathFollowerModule")
    mux = system.get_module("CmdVelMux")

    seen = {
        "costmap": 0,
        "waypoints": 0,
        "global_path": 0,
        "local_path": 0,
        "path_follower_cmd": 0,
        "mux_cmd": 0,
        "direct_fallback": 0,
    }
    odom: list[tuple[float, float, float]] = []
    ogm.costmap._add_callback(lambda _: seen.__setitem__("costmap", seen["costmap"] + 1))
    nav.global_path._add_callback(lambda _: seen.__setitem__("global_path", seen["global_path"] + 1))
    nav.waypoint._add_callback(lambda _: seen.__setitem__("waypoints", seen["waypoints"] + 1))
    nav.adapter_status._add_callback(
        lambda e: seen.__setitem__(
            "direct_fallback",
            seen["direct_fallback"] + (1 if e.get("event") == "direct_goal_fallback" else 0),
        )
    )
    local_planner.local_path._add_callback(
        lambda _: seen.__setitem__("local_path", seen["local_path"] + 1)
    )
    path_follower.cmd_vel._add_callback(
        lambda _: seen.__setitem__("path_follower_cmd", seen["path_follower_cmd"] + 1)
    )
    mux.driver_cmd_vel._add_callback(
        lambda _: seen.__setitem__("mux_cmd", seen["mux_cmd"] + 1)
    )
    driver.odometry._add_callback(
        lambda m: odom.append(
            (float(m.pose.position.x), float(m.pose.position.y), float(m.pose.position.z))
        )
    )

    system.start()
    try:
        warm_deadline = time.time() + min(6.0, max(2.0, duration_s * 0.3))
        while time.time() < warm_deadline and (seen["costmap"] == 0 or not odom):
            time.sleep(0.1)
        if not odom:
            return ValidationCheck(
                name="mujoco_kinematic_nav_runtime",
                category="local_planning",
                status=FAIL,
                summary="sim_mujoco produced no odometry",
                evidence={"seen": seen},
            )

        start = odom[-1]
        goal_x = start[0] + goal_distance_m
        goal_y = start[1]
        nav.goal_pose._deliver(
            PoseStamped(
                pose=Pose(
                    position=Vector3(goal_x, goal_y, 0.0),
                    orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                ),
                frame_id="map",
                ts=time.time(),
            )
        )

        deadline = time.time() + duration_s
        moved = 0.0
        dist_to_goal = math.hypot(goal_x - start[0], goal_y - start[1])
        finite = True
        while time.time() < deadline:
            time.sleep(0.1)
            if odom:
                x, y, z = odom[-1]
                finite = finite and all(math.isfinite(v) for v in odom[-1])
                moved = math.hypot(x - start[0], y - start[1])
                dist_to_goal = math.hypot(goal_x - x, goal_y - y)
            if (
                seen["costmap"] > 0
                and seen["global_path"] > 0
                and seen["waypoints"] > 0
                and seen["local_path"] > 0
                and seen["path_follower_cmd"] > 3
                and seen["mux_cmd"] > 3
                and moved >= min_motion_m
            ):
                break
    finally:
        system.stop()

    passed = (
        finite
        and seen["costmap"] > 0
        and seen["global_path"] > 0
        and seen["waypoints"] > 0
        and seen["local_path"] > 0
        and seen["path_follower_cmd"] > 3
        and seen["mux_cmd"] > 3
        and seen["direct_fallback"] == 0
        and moved >= min_motion_m
        and dist_to_goal < goal_distance_m
    )
    return ValidationCheck(
        name="mujoco_kinematic_nav_runtime",
        category="local_planning",
        status=PASS if passed else FAIL,
        summary=(
            "kinematic MuJoCo full-stack navigation produced motion through local planning"
            if passed
            else "kinematic MuJoCo navigation did not close the motion loop"
        ),
        evidence={
            "world": "open_field",
            "drive_mode": "kinematic",
            "duration_s": duration_s,
            "goal_distance_m": goal_distance_m,
            "min_motion_m": min_motion_m,
            "seen": seen,
            "finite": finite,
            "moved_m": moved,
            "dist_to_goal_m": dist_to_goal,
            "start": [float(v) for v in start],
            "end": [float(v) for v in odom[-1]] if odom else None,
        },
    )


def validate_policy_smoke_runtime(
    *,
    world: str,
    direct_duration_s: float,
    nav_duration_s: float,
    goal_distance_m: float,
    policy_path: str = "",
) -> ValidationCheck:
    if importlib.util.find_spec("mujoco") is None:
        return ValidationCheck(
            name="mujoco_policy_smoke",
            category="policy",
            status=BLOCKED,
            summary="mujoco is not installed in this environment",
        )
    if importlib.util.find_spec("onnxruntime") is None:
        return ValidationCheck(
            name="mujoco_policy_smoke",
            category="policy",
            status=BLOCKED,
            summary="onnxruntime is not installed in this environment",
        )

    from sim.scripts import policy_nav_smoke

    direct = policy_nav_smoke.run_direct_policy(
        world=world,
        duration=direct_duration_s,
        linear_x=0.2,
        angular_z=0.0,
        policy_path=policy_path,
    )
    direct["passed"] = policy_nav_smoke._passes_direct(direct, min_motion=0.20)
    direct["policy"] = policy_nav_smoke._load_policy_metadata(
        str(direct.get("policy_path", ""))
    )

    nav = policy_nav_smoke.run_full_stack_nav(
        world=world,
        duration=nav_duration_s,
        goal_distance=goal_distance_m,
        policy_path=policy_path,
    )
    nav["passed"] = policy_nav_smoke._passes_nav(nav, min_motion=0.20)
    nav["policy"] = policy_nav_smoke._load_policy_metadata(str(nav.get("policy_path", "")))

    policy_loaded = bool(direct.get("policy_loaded")) and bool(nav.get("policy_loaded"))
    policy_exists = bool(direct.get("policy", {}).get("exists")) or bool(
        nav.get("policy", {}).get("exists")
    )
    passed = bool(direct.get("passed")) and bool(nav.get("passed"))
    if passed:
        status = PASS
        summary = "policy-mode direct gait and full-stack navigation smoke passed"
    elif not policy_loaded and not policy_exists:
        status = BLOCKED
        summary = "no compatible policy checkpoint was found"
    elif not policy_loaded:
        status = BLOCKED
        summary = "policy checkpoint exists but was not loaded by the simulator"
    else:
        status = FAIL
        summary = "policy-mode smoke ran but did not meet motion/stability gates"

    return ValidationCheck(
        name="mujoco_policy_smoke",
        category="policy",
        status=status,
        summary=summary,
        evidence={
            "world": world,
            "policy_path_arg": policy_path,
            "direct": direct,
            "nav": nav,
        },
    )


def run_validation(
    *,
    repo_root: str | Path | None = None,
    run_mujoco: bool = False,
    mujoco_worlds: Iterable[str] = ("open_field", "building_scene"),
    nav_duration_s: float = 10.0,
    nav_goal_distance_m: float = 1.0,
    min_nav_motion_m: float = 0.20,
    run_policy: bool = False,
    policy_path: str = "",
    policy_direct_duration_s: float = 6.0,
    policy_nav_duration_s: float = 18.0,
    policy_goal_distance_m: float = 1.0,
    require_all: bool = False,
) -> ValidationReport:
    root = Path(repo_root).resolve() if repo_root is not None else _repo_root()
    checks: list[ValidationCheck] = []
    checks.extend(validate_scene_catalog(root))
    checks.append(
        _timed(
            "slam_localization_metric_gate",
            "slam_localization",
            lambda: validate_slam_localization_contract(root),
        )
    )
    checks.append(
        _timed("sim_nav_planning_wiring", "navigation", lambda: validate_navigation_blueprint(root))
    )
    checks.append(
        _timed(
            "frontier_exploration_to_navigation",
            "exploration",
            validate_frontier_exploration_runtime,
        )
    )
    checks.append(
        _timed("person_tracking_behavior_loop", "tracking", validate_person_tracking_runtime)
    )
    if run_mujoco:
        checks.extend(validate_mujoco_lidar_runtime(root, mujoco_worlds))
        checks.append(
            _timed(
                "mujoco_kinematic_nav_runtime",
                "local_planning",
                lambda: validate_mujoco_kinematic_nav_runtime(
                    duration_s=nav_duration_s,
                    goal_distance_m=nav_goal_distance_m,
                    min_motion_m=min_nav_motion_m,
                ),
            )
        )
    else:
        checks.append(
            ValidationCheck(
                name="mujoco_lidar_runtime",
                category="lidar",
                status=BLOCKED,
                summary="run with --run-mujoco to collect live LiDAR point-cloud evidence",
                evidence={"worlds": list(mujoco_worlds)},
            )
        )
    if run_policy:
        checks.append(
            _timed(
                "mujoco_policy_smoke",
                "policy",
                lambda: validate_policy_smoke_runtime(
                    world="open_field",
                    direct_duration_s=policy_direct_duration_s,
                    nav_duration_s=policy_nav_duration_s,
                    goal_distance_m=policy_goal_distance_m,
                    policy_path=policy_path,
                ),
            )
        )

    fail_count = sum(1 for check in checks if check.status == FAIL)
    blocked_count = sum(1 for check in checks if check.status == BLOCKED)
    required_blocked_count = sum(
        1
        for check in checks
        if check.name == "sim_nav_planning_wiring" and check.status == BLOCKED
    )
    passed = (
        fail_count == 0
        and required_blocked_count == 0
        and (blocked_count == 0 if require_all else True)
    )
    return ValidationReport(
        generated_at=time.time(),
        repo_root=str(root),
        passed=passed,
        checks=tuple(checks),
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", default="")
    parser.add_argument("--run-mujoco", action="store_true")
    parser.add_argument(
        "--mujoco-world",
        action="append",
        dest="mujoco_worlds",
        default=[],
        help="MuJoCo world name to validate. Repeatable.",
    )
    parser.add_argument("--nav-duration", type=float, default=10.0)
    parser.add_argument("--nav-goal-distance", type=float, default=1.0)
    parser.add_argument("--min-nav-motion", type=float, default=0.20)
    parser.add_argument("--run-policy", action="store_true")
    parser.add_argument("--policy-path", default="")
    parser.add_argument("--policy-direct-duration", type=float, default=6.0)
    parser.add_argument("--policy-nav-duration", type=float, default=18.0)
    parser.add_argument("--policy-goal-distance", type=float, default=1.0)
    parser.add_argument("--require-all", action="store_true")
    parser.add_argument("--json-out", default="")
    args = parser.parse_args(argv)

    report = run_validation(
        repo_root=args.repo_root or None,
        run_mujoco=args.run_mujoco,
        mujoco_worlds=args.mujoco_worlds or ("open_field", "building_scene"),
        nav_duration_s=args.nav_duration,
        nav_goal_distance_m=args.nav_goal_distance,
        min_nav_motion_m=args.min_nav_motion,
        run_policy=args.run_policy,
        policy_path=args.policy_path,
        policy_direct_duration_s=args.policy_direct_duration,
        policy_nav_duration_s=args.policy_nav_duration,
        policy_goal_distance_m=args.policy_goal_distance,
        require_all=args.require_all,
    )
    output = json.dumps(report.to_dict(), indent=2, sort_keys=True)
    print(output)
    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(output + "\n", encoding="utf-8")
    return 0 if report.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
