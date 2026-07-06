#!/usr/bin/env python3
"""Run the MuJoCo LiDAR/IMU localization gate without ROS.

This gate is simulation-only. It does not connect to robot services and does
not publish hardware commands. The old portable LIO path has been removed;
live localization must come from a real endpoint adapter before this gate is
treated as a closed-loop localization benchmark.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from runtime.msgs.numpy_compat import np
from runtime.runtime_interface import (
    FRAME_LINKS,
    TOPICS,
    adapter_source_for_target,
    lidar_extrinsic,
    simulator_world_frame_id,
    topic_default_frame_id,
)
from drivers.sim.mujoco.runtime import (
    DEFAULT_MID360_PATTERN,
    DEFAULT_MID360_SAMPLES_PER_FRAME,
    build_engine as _build_engine,
    parse_start as _parse_start,
    resolve_mid360_pattern as _resolve_mid360_pattern,
    resolve_world as _resolve_world,
    scene_start as _scene_start,
)
from runtime.same_source_map_artifacts import (
    add_points_to_voxel_store as _add_points_to_voxel_store,
    write_same_source_map_artifacts as _write_same_source_map_artifacts,
)
from sim.scripts.mujoco_live.diag import (
    _aligned_motion_window,
    _degeneracy_detail_sample,
    _dynamic_obstacle_sweep_quality,
    _fastlio2_log_diagnostics,
    _fastlio_large_loop_diagnostic_report,
    _navigation_diagnostic_sample,
    _nearest_sim_pose_sample,
    _path_summary,
    _round_float,
    _summarize_degeneracy_detail_samples,
    _update_runtime_fault_streak,
)
from sim.scripts.mujoco_live.motion import (
    _area_growth,
    _coerce_xyzi_cloud,
    _core_cloud_xy_stats,
    _coverage_growth,
    _exploration_area_sample,
    _goal_xy,
    _goal_xy_matches,
    _limit_command_delta,
    _live_moving_obstacle_boxes_from_pose,
    _live_moving_obstacle_points,
    _live_moving_obstacle_speed_bounds,
    _live_moving_obstacle_trail_clearance,
    _map_frame_origin_world_xy_from_tomogram,
    _motion_consistency_report,
    _nav_planner_has_live_map,
    _normalize_localization_backend,
    _parse_inspection_goals,
    _physical_rolling_scan_from_samples,
    _relative_times_for_scan,
    _select_nav_cmd_for_step,
    _state_in_map_frame,
)
from sim.scripts.mujoco_live.report import (
    _exception_partial_report_path,
    _gate_exception_report,
    _inspection_gate_evidence_complete,
    _load_exception_partial_report,
    _merge_exception_partial_report,
    _mujoco_data_flow_evidence,
    _mujoco_fastlio_contract_definition,
    _mujoco_frame_evidence,
    _mujoco_hardware_safety,
    _mujoco_runtime_contract,
    _runtime_evidence_report,
    _video_sample_elapsed_s,
    _wall_timeout_status,
    _write_json_atomic,
)
from sim.scripts.mujoco_live.sensors import (
    _angle_delta_rad,
    _specific_force_body,
    _world_xyzi_to_sensor_xyzi,
    _yaw_from_quat_xyzw,
)
from sim.scripts.mujoco_live.video import (
    _render_mujoco_overview,
    _write_stage_video,
)

SIM_WORLD_FRAME_ID = simulator_world_frame_id()
SIM_MAP_FRAME_ID = FRAME_LINKS["map_to_odom"].parent
SIM_ODOM_FRAME_ID = FRAME_LINKS["map_to_odom"].child
SIM_BODY_FRAME_ID = FRAME_LINKS["odom_to_body"].child
SIM_LIDAR_FRAME_ID = FRAME_LINKS["body_to_lidar"].child
SIM_CAMERA_FRAME_ID = FRAME_LINKS["body_to_camera"].child
SIM_NAV_ODOMETRY_FRAME_ID = topic_default_frame_id(TOPICS.odometry)
SIM_NAV_REGISTERED_CLOUD_FRAME_ID = topic_default_frame_id(TOPICS.registered_cloud)
# Fast-LIO live map clouds remain in local odom until a localizer owns map->odom.
SIM_FASTLIO_LIVE_MAP_FRAME_ID = SIM_NAV_ODOMETRY_FRAME_ID
FASTLIO_REGISTERED_CLOUD_TOPIC = adapter_source_for_target(
    "fastlio2",
    TOPICS.registered_cloud,
)
FASTLIO_MAP_CLOUD_TOPIC = adapter_source_for_target("fastlio2", TOPICS.map_cloud)
FASTLIO_ODOMETRY_TOPIC = adapter_source_for_target("fastlio2", TOPICS.odometry)
PCT_OPTIMIZE_TRAJECTORY_ENV = "LINGTU_PCT_OPTIMIZE_TRAJECTORY"
_FALSE_ENV_VALUES = {"0", "false", "no", "off"}


def _pct_optimizer_enabled_from_env(env: dict[str, str] | None = None) -> bool:
    values = os.environ if env is None else env
    raw = values.get(PCT_OPTIMIZE_TRAJECTORY_ENV)
    if raw is None or str(raw).strip() == "":
        return True
    return str(raw).strip().lower() not in _FALSE_ENV_VALUES




































































































































def _run_no_ros_portable_lio_gate(**cfg: Any) -> dict[str, Any]:
    """Removed portable LIO gate."""

    del cfg
    raise RuntimeError(
        "portable_lio MuJoCo gate was removed; use a real localization endpoint "
        "or reintroduce this only after extracting the actual LIO algorithm"
    )


def _wrap_angle_rad(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def _clamp_float(value: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(value)))


def _run_mujoco_ground_truth_exploration_gate(**cfg: Any) -> dict[str, Any]:
    """Run the live exploration gate from MuJoCo truth pose and LiDAR scans.

    This is a simulation gate only. It proves the frontier -> cmd_vel -> MuJoCo
    motion -> map artifact loop, without pretending to be a SLAM backend.
    """

    from sim.engine.core.engine import VelocityCommand
    from runtime.msgs.geometry import Pose, Quaternion, Twist, Vector3
    from runtime.msgs.nav import Odometry
    from runtime.msgs.sensor import PointCloud2
    from nav.exploration.frontier_explorer_module import WavefrontFrontierExplorer
    from nav.services.map_layers.occupancy_grid_module import OccupancyGridModule

    world = Path(cfg["world"])
    work_dir = Path(cfg["work_dir"])
    work_dir.mkdir(parents=True, exist_ok=True)

    duration = max(0.0, float(cfg["duration"]))
    duration_clock = str(cfg.get("duration_clock") or "wall")
    max_wall_time_s = max(0.0, float(cfg.get("max_wall_time_s") or 0.0))
    run_frontier = bool(cfg.get("run_lingtu_frontier"))
    run_tare = bool(cfg.get("run_lingtu_tare"))
    run_inspection = bool(cfg.get("run_lingtu_inspection"))
    require_goal_arrival = bool(cfg.get("require_goal_arrival"))
    goal_arrival_threshold = max(0.01, float(cfg.get("goal_arrival_threshold") or 0.30))
    remaining_gaps: list[str] = []
    if run_tare:
        remaining_gaps.append("tare_native_exploration_not_connected_in_ground_truth_gate")
    if run_inspection:
        remaining_gaps.append("inspection_not_connected_in_ground_truth_gate")

    try:
        engine = _build_engine(
            world=world,
            drive_mode=str(cfg["drive_mode"]),
            n_rays=int(cfg["n_rays"]),
            start=cfg.get("start"),
            mujoco_memory=str(cfg.get("mujoco_memory") or ""),
            mid360_pattern=cfg.get("mid360_pattern"),
            mid360_samples_per_frame=int(cfg["mid360_samples_per_frame"]),
            lidar_backend=str(cfg["lidar_backend"]),
            mujoco_lidar_backend=str(cfg["mujoco_lidar_backend"]),
            require_product_lidar_backend=True,
            allow_legacy_lidar_fallback=bool(cfg["allow_legacy_lidar_fallback"]),
        )
    except Exception as exc:
        report = {
            "schema_version": "lingtu.mujoco_ground_truth_live_gate.v1",
            "ok": False,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "no_python_slam": True,
            "nav_data_source": "mujoco_ground_truth",
            "localization_backend": "mujoco_ground_truth",
            "world": str(world),
            "work_dir": str(work_dir),
            "lidar_backend": {
                "requested_backend": str(cfg["lidar_backend"]),
                "mujoco_lidar_backend": str(cfg["mujoco_lidar_backend"]),
                "require_product_backend": True,
                "allow_legacy_fallback": bool(cfg["allow_legacy_lidar_fallback"]),
                "product_lidar_backend_verified": False,
                "error": f"{type(exc).__name__}: {exc}",
            },
            "map_artifacts": {"ok": False, "blockers": ["engine_init_failed"]},
            "deliverable_contract": {
                "product_mujoco_lidar_backend": False,
                "raw_mujoco_lidar": False,
                "raw_mujoco_imu": False,
                "same_source_map_artifact": False,
                "frontier_or_exploration": False,
                "no_ros_default_runtime": True,
                "no_ros_message_shim": True,
                "localization_odometry_and_map": False,
                "nav_cmd_vel_nonzero": False,
            },
            "remaining_gaps": sorted(set(remaining_gaps + ["mujoco_engine_init_failed"])),
        }
        if cfg.get("partial_json_out"):
            _write_json_atomic(Path(cfg["partial_json_out"]), report)
        return report

    occupancy = OccupancyGridModule(
        resolution=max(0.10, float(cfg.get("tomogram_resolution") or 0.20)),
        map_radius=18.0,
        z_min=0.15,
        z_max=2.20,
        inflation_radius=0.25,
        robot_clear_radius=0.45,
        publish_hz=20.0,
        frame_id=SIM_NAV_ODOMETRY_FRAME_ID,
        raycast_free_space=True,
        raycast_max_rays=max(600, min(5000, int(cfg["n_rays"]))),
        raycast_free_inflation_radius=0.05,
    )
    frontier = WavefrontFrontierExplorer(
        min_frontier_size=3,
        safe_distance=0.45,
        lookahead_distance=6.0,
        max_explored_distance=18.0,
        info_gain_threshold=0.0,
        goal_timeout=float(cfg.get("frontier_goal_timeout") or 30.0),
        costmap_max_age=120.0,
        reachable_goal_radius=0.7,
        approach_max_target_distance_m=20.0 if require_goal_arrival else 4.0,
        approach_goal_max_distance_m=(
            max(goal_arrival_threshold + 0.20, goal_arrival_threshold * 1.5)
            if require_goal_arrival
            else 8.0
        ),
    )
    occupancy.costmap.subscribe(frontier._on_costmap)
    occupancy.exploration_grid.subscribe(frontier._on_exploration_grid)

    voxel_store: dict[tuple[int, int, int], tuple[float, float, float]] = {}
    path_xy: list[tuple[float, float]] = []
    goal_trace: list[dict[str, Any]] = []
    latest_frontiers: list[dict[str, Any]] = []
    explored_area_samples: list[float] = []
    active_goal_xy: tuple[float, float] | None = None
    goal_reached = False
    distance_to_goal_m: float | None = None
    goal_reached_at_sim_s: float | None = None
    goal_reached_at_wall_s: float | None = None
    active_goal_type: str | None = None
    active_frontier_target_xy: tuple[float, float] | None = None
    last_goal_xy: tuple[float, float] | None = None
    last_cmd = (0.0, 0.0, 0.0)
    scan_count = 0
    imu_sample_count = 0
    nonzero_cmd_count = 0
    timeout = False
    start_wall = time.monotonic()
    start_sim = float(getattr(engine, "sim_time", 0.0))

    if float(cfg.get("startup_sleep") or 0.0) > 0.0:
        time.sleep(float(cfg["startup_sleep"]))

    try:
        for _ in range(max(0, int(float(cfg.get("settle_sleep") or 0.0) / max(engine.control_dt, 1e-3)))):
            engine.step(VelocityCommand())

        while True:
            now_wall = time.monotonic()
            elapsed_wall = now_wall - start_wall
            elapsed_sim = float(getattr(engine, "sim_time", 0.0)) - start_sim
            elapsed = elapsed_sim if duration_clock == "sim" else elapsed_wall
            if elapsed >= duration:
                break
            if max_wall_time_s and elapsed_wall >= max_wall_time_s:
                timeout = True
                remaining_gaps.append("max_wall_time_reached")
                break

            state = engine.get_robot_state()
            pos = np.asarray(state.position, dtype=np.float64)
            quat = np.asarray(state.orientation, dtype=np.float64)
            yaw = _yaw_from_quat_xyzw(quat)
            odom = Odometry(
                pose=Pose(Vector3(pos[:3]), Quaternion(quat[:4])),
                twist=Twist(
                    Vector3(np.asarray(state.linear_velocity, dtype=np.float64)[:3]),
                    Vector3(np.asarray(state.angular_velocity, dtype=np.float64)[:3]),
                ),
                ts=time.time(),
                frame_id=SIM_NAV_ODOMETRY_FRAME_ID,
                child_frame_id=SIM_BODY_FRAME_ID,
            )
            path_xy.append((float(pos[0]), float(pos[1])))
            frontier._on_odometry(odom)
            occupancy._on_odom(odom)

            raw_points = np.asarray(engine.get_lidar_points(), dtype=np.float32)
            points = _coerce_xyzi_cloud(raw_points)
            if points.shape[0] > 0:
                scan_count += 1
                _add_points_to_voxel_store(
                    voxel_store,
                    points[:, :3],
                    voxel_size=float(cfg["map_artifact_voxel_size"]),
                    max_points=int(cfg["map_artifact_max_points"]),
                )
                occupancy._on_cloud(
                    PointCloud2.from_numpy(
                        points[:, :3],
                        frame_id=SIM_NAV_ODOMETRY_FRAME_ID,
                        ts=time.time(),
                    )
                )
                latest_grid = getattr(frontier, "_exploration_grid_data", None)
                if isinstance(latest_grid, dict) and "grid" in latest_grid:
                    grid = np.asarray(latest_grid["grid"])
                    explored_area_samples.append(
                        float((grid == 0).sum())
                        * float(latest_grid.get("resolution", 0.0)) ** 2
                    )

            gyro = np.asarray(getattr(state, "imu_gyro", []), dtype=np.float64)
            acc = np.asarray(getattr(state, "imu_linear_acceleration", []), dtype=np.float64)
            if gyro.size >= 3 and acc.size >= 3 and np.isfinite(gyro).all() and np.isfinite(acc).all():
                imu_sample_count += 1

            cmd = VelocityCommand(
                linear_x=float(cfg["drive_vx"]),
                linear_y=float(cfg["drive_vy"]),
                angular_z=float(cfg["drive_wz"]),
            )
            if run_frontier and elapsed >= float(cfg.get("frontier_start_delay") or 0.0):
                latest_frontiers = frontier._compute_frontier_clusters()
                if latest_frontiers and (active_goal_xy is None or not require_goal_arrival):
                    if require_goal_arrival:
                        target = min(
                            latest_frontiers,
                            key=lambda item: (
                                math.hypot(float(item["cx"]) - float(pos[0]), float(item["cy"]) - float(pos[1]))
                                + 0.4
                                * abs(
                                    _wrap_angle_rad(
                                        math.atan2(
                                            float(item["cy"]) - float(pos[1]),
                                            float(item["cx"]) - float(pos[0]),
                                        )
                                        - yaw
                                    )
                                )
                            ),
                        )
                    else:
                        target = latest_frontiers[0]
                    goal_xy = (float(target["cx"]), float(target["cy"]))
                    goal_type = (
                        "frontier_approach_goal"
                        if bool(target.get("approach_goal"))
                        else "frontier_goal"
                    )
                    frontier_target_raw = target.get("frontier_target")
                    frontier_target_xy: tuple[float, float] | None = None
                    if (
                        isinstance(frontier_target_raw, (list, tuple))
                        and len(frontier_target_raw) >= 2
                    ):
                        frontier_target_xy = (
                            float(frontier_target_raw[0]),
                            float(frontier_target_raw[1]),
                        )
                    if require_goal_arrival:
                        dx_goal = goal_xy[0] - float(pos[0])
                        dy_goal = goal_xy[1] - float(pos[1])
                        goal_dist = math.hypot(dx_goal, dy_goal)
                        max_goal_dist = goal_arrival_threshold + 0.12
                        if goal_dist > max_goal_dist > 0.0:
                            scale = max_goal_dist / goal_dist
                            goal_xy = (
                                float(pos[0]) + dx_goal * scale,
                                float(pos[1]) + dy_goal * scale,
                            )
                            goal_type = f"bounded_{goal_type}"
                    active_goal_xy = goal_xy
                    active_goal_type = goal_type
                    active_frontier_target_xy = frontier_target_xy
                    if (
                        last_goal_xy is None
                        or math.hypot(goal_xy[0] - last_goal_xy[0], goal_xy[1] - last_goal_xy[1]) > 0.45
                    ):
                        trace_item: dict[str, Any] = {
                            "x": round(goal_xy[0], 3),
                            "y": round(goal_xy[1], 3),
                            "goal_type": goal_type,
                            "score": round(float(target.get("score", 0.0)), 4),
                            "size": int(target.get("size", 0)),
                            "elapsed_s": round(float(elapsed), 3),
                        }
                        if frontier_target_xy is not None:
                            trace_item["frontier_target"] = [
                                round(float(frontier_target_xy[0]), 3),
                                round(float(frontier_target_xy[1]), 3),
                            ]
                        goal_trace.append(trace_item)
                        last_goal_xy = goal_xy
                if active_goal_xy is not None:
                    dx = active_goal_xy[0] - float(pos[0])
                    dy = active_goal_xy[1] - float(pos[1])
                    dist = math.hypot(dx, dy)
                    distance_to_goal_m = float(dist)
                    if dist <= goal_arrival_threshold:
                        goal_reached = True
                        goal_reached_at_sim_s = float(elapsed_sim)
                        goal_reached_at_wall_s = float(elapsed_wall)
                        if require_goal_arrival:
                            break
                    yaw_err = _wrap_angle_rad(math.atan2(dy, dx) - yaw)
                    linear_limit = min(
                        float(cfg["cmd_vel_linear_limit"]),
                        float(cfg["nav_max_linear_speed"]),
                    )
                    angular_limit = min(
                        float(cfg["cmd_vel_angular_limit"]),
                        float(cfg["nav_max_angular_z"]),
                    )
                    linear = _clamp_float(dist * 0.45, 0.0, linear_limit)
                    if abs(yaw_err) > 0.9:
                        linear = min(linear, 0.04)
                    cmd = VelocityCommand(
                        linear_x=linear,
                        linear_y=0.0,
                        angular_z=_clamp_float(yaw_err * 1.4, -angular_limit, angular_limit),
                    )

            limited = _limit_command_delta(
                previous=last_cmd,
                target=(float(cmd.linear_x), float(cmd.linear_y), float(cmd.angular_z)),
                dt_s=max(float(engine.control_dt), 1.0e-3),
                linear_accel_limit=float(cfg["cmd_vel_linear_accel_limit"]),
                angular_accel_limit=float(cfg["cmd_vel_angular_accel_limit"]),
            )
            last_cmd = limited
            cmd = VelocityCommand(*limited)
            if abs(cmd.linear_x) + abs(cmd.linear_y) + abs(cmd.angular_z) > 1.0e-4:
                nonzero_cmd_count += 1
            engine.step(cmd)
    finally:
        try:
            engine.close()
        except Exception:
            pass

    point_sample = np.asarray(list(voxel_store.values()), dtype=np.float32)
    map_artifacts: dict[str, Any] = {"ok": False, "blockers": ["map_artifacts_disabled"]}
    if bool(cfg.get("save_map_artifacts", True)):
        map_artifacts = _write_same_source_map_artifacts(
            artifact_dir=work_dir / "same_source_map",
            points=point_sample,
            frame_id=SIM_NAV_ODOMETRY_FRAME_ID,
            world=world,
            source_topics=(TOPICS.map_cloud,),
            mapping_input_path=TOPICS.map_cloud,
            build_tomogram=bool(cfg.get("build_tomogram")),
            tomogram_resolution=float(cfg["tomogram_resolution"]),
            tomogram_slice_dh=float(cfg["tomogram_slice_dh"]),
            tomogram_ground_h=float(cfg["tomogram_ground_h"]),
            map_artifact_max_span_m=float(cfg["map_artifact_max_span_m"]),
            tomogram_max_cells=int(cfg["tomogram_max_cells"]),
            source="mujoco_ground_truth_live_gate",
            extra_metadata={
                "source_profile": "mujoco_ground_truth_live_gate",
                "data_source": "mujoco_ground_truth",
                "slam_source": "mujoco_ground_truth",
                "localization_source": "mujoco_ground_truth",
                "mapping_source": TOPICS.map_cloud,
                "no_python_slam": True,
            },
        )

    sim_wall_time_s = time.monotonic() - start_wall
    elapsed_sim_s = float(getattr(engine, "sim_time", start_sim)) - start_sim
    sim_path_length_m = 0.0
    for prev, cur in zip(path_xy, path_xy[1:]):
        sim_path_length_m += math.hypot(cur[0] - prev[0], cur[1] - prev[1])
    odom_path_length_m = sim_path_length_m
    applied_cmd_stats = {
        "nonzero_count": int(nonzero_cmd_count),
        "last": [round(float(v), 4) for v in last_cmd],
    }
    explored_area_growth_m2 = (
        max(0.0, explored_area_samples[-1] - explored_area_samples[0])
        if len(explored_area_samples) >= 2
        else 0.0
    )
    coverage_growth_ratio = (
        explored_area_growth_m2 / max(explored_area_samples[-1], 1.0)
        if explored_area_samples
        else 0.0
    )
    lidar_backend_report = (
        engine.get_lidar_backend_report()
        if hasattr(engine, "get_lidar_backend_report")
        else {"backend": str(cfg["lidar_backend"]), "product_backend": False}
    )
    product_lidar_backend_verified = bool(
        lidar_backend_report.get("product_lidar_backend_verified")
    )
    if not product_lidar_backend_verified:
        remaining_gaps.append("product_mujoco_lidar_backend_not_verified")
    if scan_count <= 0:
        remaining_gaps.append("no_lidar_scans")
    if run_frontier and len(goal_trace) < int(cfg["frontier_min_goals"]):
        remaining_gaps.append("frontier_goal_count_below_threshold")
    if require_goal_arrival and not goal_reached:
        remaining_gaps.append("goal_arrival_not_reached")
    if bool(cfg.get("save_map_artifacts", True)) and not bool(map_artifacts.get("ok")):
        remaining_gaps.append("same_source_map_artifact_failed")

    ok = (
        not timeout
        and scan_count > 0
        and (not bool(cfg.get("save_map_artifacts", True)) or bool(map_artifacts.get("ok")))
        and (not run_frontier or len(goal_trace) >= int(cfg["frontier_min_goals"]))
        and (not require_goal_arrival or goal_reached)
        and product_lidar_backend_verified
        and not run_tare
        and not run_inspection
    )
    report: dict[str, Any] = {
        "schema_version": "lingtu.mujoco_ground_truth_live_gate.v1",
        "ok": bool(ok),
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "no_python_slam": True,
        "nav_data_source": "mujoco_ground_truth",
        "localization_backend": "mujoco_ground_truth",
        "localization_runtime": {
            "transport": "in_process_core_msgs",
            "uses_ros_message_shim": False,
            "outputs": ["lio_odometry", "lio_cloud_registered", "lio_cloud_map"],
        },
        "world": str(world),
        "work_dir": str(work_dir),
        "duration_clock": duration_clock,
        "timing": {
            "elapsed_wall_s": round(float(sim_wall_time_s), 3),
            "elapsed_sim_s": round(float(elapsed_sim_s), 3),
            "sim_realtime_factor": (
                round(float(elapsed_sim_s / sim_wall_time_s), 3)
                if sim_wall_time_s > 0
                else 0.0
            ),
            "timeout": bool(timeout),
        },
        "lidar_backend": lidar_backend_report,
        "product_lidar_backend_verified": product_lidar_backend_verified,
        "sensor_flow": {
            "lidar_scan_count": int(scan_count),
            "imu_sample_count": int(imu_sample_count),
            "accumulated_map_points": int(point_sample.shape[0]),
        },
        "motion": {
            "sim_path_length_m": round(float(sim_path_length_m), 4),
            "localization_path_length_m": round(float(odom_path_length_m), 4),
            "odom_path_length_m": round(float(odom_path_length_m), 4),
            "nonzero_cmd_count": int(nonzero_cmd_count),
            "applied_command": dict(applied_cmd_stats),
            "final_pose": (
                [round(float(path_xy[-1][0]), 4), round(float(path_xy[-1][1]), 4)]
                if path_xy
                else None
            ),
        },
        "exploration": {
            "mode": "frontier" if run_frontier else "fixed_drive",
            "frontier_goal_count": int(len(goal_trace)),
            "frontier_goals": goal_trace[:20],
            "latest_frontier_count": int(len(latest_frontiers)),
            "frontier_debug": dict(getattr(frontier, "_frontier_debug", {})),
            "explored_area_growth_m2": round(float(explored_area_growth_m2), 4),
            "coverage_growth_ratio": round(float(coverage_growth_ratio), 6),
            "min_goal_threshold": int(cfg["frontier_min_goals"]) if run_frontier else 0,
        },
        "goal_arrival": {
            "required": bool(require_goal_arrival),
            "goal_reached": bool(goal_reached),
            "threshold_m": round(float(goal_arrival_threshold), 4),
            "goal_type": active_goal_type,
            "goal": (
                [round(float(active_goal_xy[0]), 4), round(float(active_goal_xy[1]), 4)]
                if active_goal_xy is not None
                else None
            ),
            "frontier_target": (
                [
                    round(float(active_frontier_target_xy[0]), 4),
                    round(float(active_frontier_target_xy[1]), 4),
                ]
                if active_frontier_target_xy is not None
                else None
            ),
            "distance_to_goal_m": (
                round(float(distance_to_goal_m), 4)
                if distance_to_goal_m is not None
                else None
            ),
            "distance_to_frontier_target_m": (
                round(
                    math.hypot(
                        float(path_xy[-1][0]) - float(active_frontier_target_xy[0]),
                        float(path_xy[-1][1]) - float(active_frontier_target_xy[1]),
                    ),
                    4,
                )
                if active_frontier_target_xy is not None and path_xy
                else None
            ),
            "path_length_m": round(float(sim_path_length_m), 4),
            "elapsed_sim_s": (
                round(float(goal_reached_at_sim_s), 3)
                if goal_reached_at_sim_s is not None
                else round(float(elapsed_sim_s), 3)
            ),
            "elapsed_wall_s": (
                round(float(goal_reached_at_wall_s), 3)
                if goal_reached_at_wall_s is not None
                else round(float(sim_wall_time_s), 3)
            ),
        },
        "map_artifacts": map_artifacts,
        "deliverable_contract": {
            "product_mujoco_lidar_backend": product_lidar_backend_verified,
            "raw_mujoco_lidar": scan_count > 0,
            "raw_mujoco_imu": imu_sample_count > 0,
            "same_source_map_artifact": bool(map_artifacts.get("ok")),
            "frontier_or_exploration": bool(goal_trace) if run_frontier else True,
            "goal_arrival": bool(goal_reached),
            "tare_native_exploration": False if run_tare else None,
            "no_ros_default_runtime": True,
            "no_ros_message_shim": True,
            "localization_odometry_and_map": True,
            "nav_cmd_vel_nonzero": nonzero_cmd_count > 0,
        },
        "remaining_gaps": sorted(set(remaining_gaps)),
    }
    if cfg.get("partial_json_out"):
        _write_json_atomic(Path(cfg["partial_json_out"]), report)
    return report


def run_gate(
    *,
    world: Path,
    duration: float,
    drive_vx: float,
    drive_vy: float,
    drive_wz: float,
    n_rays: int,
    start: list[float] | None,
    mujoco_memory: str,
    mid360_pattern: Path | str | None,
    mid360_samples_per_frame: int,
    lidar_backend: str,
    mujoco_lidar_backend: str,
    allow_legacy_lidar_fallback: bool,
    startup_sleep: float,
    settle_sleep: float,
    work_dir: Path,
    backend_profile: str,
    drive_mode: str,
    duration_clock: str = "wall",
    max_wall_time_s: float = 0.0,
    drive_source: str = "fixed",
    cmd_vel_timeout: float = 0.75,
    cmd_vel_linear_limit: float = 0.25,
    cmd_vel_angular_limit: float = 0.15,
    cmd_vel_sim_linear_scale: float = 1.0,
    cmd_vel_sim_angular_scale: float = 1.0,
    cmd_vel_linear_accel_limit: float = 0.5,
    cmd_vel_angular_accel_limit: float = 1.0,
    nav_max_linear_speed: float = 0.25,
    nav_max_angular_z: float = 0.15,
    nav_turn_speed_yaw_rate_start: float = 0.0,
    nav_turn_speed_min_scale: float = 1.0,
    cmd_vel_mux_source_timeout: float = 0.5,
    run_lingtu_frontier: bool = False,
    run_lingtu_tare: bool = False,
    run_lingtu_inspection: bool = False,
    require_goal_arrival: bool = False,
    goal_arrival_threshold: float = 0.30,
    tare_min_goals: int = 2,
    tare_start_delay: float = 0.0,
    tare_goal_timeout: float = 180.0,
    tare_scenario: str = "indoor",
    frontier_min_goals: int = 3,
    frontier_start_delay: float = 0.0,
    frontier_goal_timeout: float = 30.0,
    inspection_goals: str = "0.8,0.0;1.6,0.2;2.4,0.4",
    inspection_min_checkpoints: int = 3,
    inspection_start_delay: float = 0.0,
    inspection_goal_timeout: float = 90.0,
    inspection_downsample_dist: float = 0.35,
    inspection_planner: str = "astar",
    inspection_tomogram: Path | str | None = None,
    inspection_waypoint_threshold: float = 0.50,
    inspection_final_waypoint_threshold: float = 0.50,
    inspection_complete_path_on_goal_proximity: bool = False,
    inspection_goal_proximity_completion_threshold: float | None = None,
    inspection_path_goal_tolerance: float = 0.12,
    inspection_path_lookahead: float = 1.5,
    inspection_path_min_speed: float = 0.15,
    inspection_path_yaw_rate_gain: float = 7.5,
    inspection_path_stop_yaw_rate_gain: float = 7.5,
    inspection_path_dir_diff_thre: float = 0.1,
    min_map_area_growth_m2: float = 0.25,
    min_explored_area_growth_m2: float = 0.25,
    min_exploration_coverage_growth_ratio: float = 0.001,
    max_fastlio_z_drift_m: float = 1.0,
    max_fastlio_yaw_drift_rad: float = 0.5,
    runtime_fault_confirm_samples: int = 2,
    runtime_motion_fault_min_sim_m: float = 0.25,
    localization_backend: str = "",
    fastlio_lidar_input: str = "livox_custom_msg",
    fastlio_lidar_filter_num: int = 4,
    fastlio_scan_resolution: float = 0.15,
    fastlio_map_resolution: float = 0.3,
    fastlio_near_search_num: int = 5,
    fastlio_ieskf_max_iter: int = 5,
    fastlio_lidar_cov_inv: float = 1000.0,
    fastlio_time_diff_lidar_to_imu: float = 0.0,
    fastlio_vertical_velocity_constraint: str = "off",
    scan_time_profile: str = "physical_rolling",
    imu_acc_mode: str = "finite_difference",
    nav_data_source: str = "fastlio2",
    show_mujoco_window: bool = False,
    mujoco_window_fps: float = 10.0,
    video_out: Path | str | None = None,
    video_fps: float = 8.0,
    moving_obstacle_mode: str = "none",
    moving_obstacle_count: int = 1,
    moving_obstacle_start_s: float = 8.0,
    moving_obstacle_duration_s: float = 0.0,
    moving_obstacle_period_s: float = 10.0,
    moving_obstacle_forward_m: float = 2.0,
    moving_obstacle_forward_step_m: float = 0.8,
    moving_obstacle_lateral_phase_step_rad: float = math.pi / 2.0,
    moving_obstacle_lateral_amplitude_m: float = 0.75,
    moving_obstacle_along_amplitude_m: float = 0.25,
    moving_obstacle_radius_m: float = 0.16,
    moving_obstacle_height_m: float = 0.60,
    moving_obstacle_point_spacing: float = 0.10,
    moving_obstacle_intensity: float = 220.0,
    moving_obstacle_robot_radius_m: float = 0.28,
    max_yaw_per_meter: float = 1.2,
    max_angular_saturation_ratio: float = 0.35,
    save_map_artifacts: bool = True,
    build_tomogram: bool = False,
    map_artifact_voxel_size: float = 0.10,
    map_artifact_max_points: int = 250_000,
    map_artifact_max_span_m: float = 120.0,
    tomogram_resolution: float = 0.20,
    tomogram_slice_dh: float = 0.25,
    tomogram_ground_h: float = 0.0,
    tomogram_max_cells: int = 50_000_000,
    partial_json_out: Path | str | None = None,
) -> dict[str, Any]:
    selected_lingtu_modes = [
        name
        for name, enabled in (
            ("frontier", run_lingtu_frontier),
            ("tare", run_lingtu_tare),
            ("inspection", run_lingtu_inspection),
        )
        if enabled
    ]
    if len(selected_lingtu_modes) > 1:
        raise ValueError(
            "--run-lingtu-frontier, --run-lingtu-tare, and "
            "--run-lingtu-inspection are mutually exclusive"
        )

    if nav_data_source == "mujoco_ground_truth" and not str(localization_backend or "").strip():
        localization_backend = "mujoco_ground_truth"
    else:
        localization_backend = _normalize_localization_backend(localization_backend)

    inspection_planner = str(inspection_planner or "astar").strip().lower()
    pct_optimizer_defaulted = False
    if run_lingtu_inspection and inspection_planner == "pct":
        pct_optimizer_defaulted = PCT_OPTIMIZE_TRAJECTORY_ENV not in os.environ
        os.environ.setdefault(PCT_OPTIMIZE_TRAJECTORY_ENV, "0")
    pct_optimizer_enabled = (
        _pct_optimizer_enabled_from_env()
        if run_lingtu_inspection and inspection_planner == "pct"
        else None
    )

    inspection_goal_list = _parse_inspection_goals(inspection_goals)
    if run_lingtu_inspection and not inspection_goal_list:
        raise ValueError("--run-lingtu-inspection requires at least one inspection goal")

    max_wall_time_s = float(max_wall_time_s or 0.0)
    if max_wall_time_s < 0.0:
        raise ValueError("--max-wall-time-s must be non-negative")

    cmd_vel_mux_source_timeout = max(0.02, float(cmd_vel_mux_source_timeout))
    runtime_motion_fault_min_sim_m = max(
        0.0,
        float(runtime_motion_fault_min_sim_m),
    )
    if nav_data_source == "mujoco_ground_truth":
        return _run_mujoco_ground_truth_exploration_gate(**locals())
    return _run_no_ros_portable_lio_gate(**locals())

def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="building_scene")
    parser.add_argument("--start", default="", help="Optional start pose x,y,z; defaults to scene marker")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument(
        "--max-wall-time-s",
        type=float,
        default=0.0,
        help=(
            "Abort the live gate after this many wall-clock seconds while still "
            "writing a red diagnostic report. 0 disables the guard."
        ),
    )
    parser.add_argument(
        "--duration-clock",
        choices=["wall", "sim"],
        default="wall",
        help=(
            "Interpret --duration as wall-clock seconds or MuJoCo simulation "
            "seconds. Use sim for exploration validation when the server runs "
            "below realtime."
        ),
    )
    parser.add_argument("--drive-vx", type=float, default=0.25)
    parser.add_argument("--drive-vy", type=float, default=0.0)
    parser.add_argument("--drive-wz", type=float, default=0.06)
    parser.add_argument("--drive-mode", choices=["kinematic", "policy"], default="kinematic")
    parser.add_argument(
        "--drive-source",
        choices=["fixed", "nav_cmd_vel"],
        default="fixed",
        help=f"Use a fixed simulation command or feed MuJoCo from LingTu {TOPICS.cmd_vel}.",
    )
    parser.add_argument("--cmd-vel-timeout", type=float, default=0.75)
    parser.add_argument("--cmd-vel-linear-limit", type=float, default=0.25)
    parser.add_argument("--cmd-vel-angular-limit", type=float, default=0.45)
    parser.add_argument("--cmd-vel-linear-accel-limit", type=float, default=0.5)
    parser.add_argument("--cmd-vel-angular-accel-limit", type=float, default=1.0)
    parser.add_argument(
        "--cmd-vel-sim-linear-scale",
        type=float,
        default=1.0,
        help=(
            f"Scale LingTu {TOPICS.cmd_vel} linear velocity only when applying it "
            f"to the kinematic MuJoCo demo model. The published {TOPICS.cmd_vel} "
            "message remains unmodified."
        ),
    )
    parser.add_argument(
        "--cmd-vel-sim-angular-scale",
        type=float,
        default=1.0,
        help=(
            f"Scale LingTu {TOPICS.cmd_vel} angular velocity only when applying it "
            "to the kinematic MuJoCo demo model."
        ),
    )
    parser.add_argument(
        "--nav-max-linear-speed",
        type=float,
        default=0.25,
        help="PathFollower max linear speed for LingTu-driven live exploration.",
    )
    parser.add_argument(
        "--nav-max-angular-z",
        type=float,
        default=0.45,
        help="PathFollower max yaw rate for LingTu-driven live exploration.",
    )
    parser.add_argument(
        "--nav-turn-speed-yaw-rate-start",
        type=float,
        default=0.0,
        help="Yaw-rate threshold in rad/s where PathFollower starts reducing linear speed; <=0 disables it.",
    )
    parser.add_argument(
        "--nav-turn-speed-min-scale",
        type=float,
        default=1.0,
        help="Minimum PathFollower linear speed scale at max yaw rate.",
    )
    parser.add_argument(
        "--cmd-vel-mux-source-timeout",
        type=float,
        default=0.5,
        help=(
            "VelocityMux source timeout for LingTu-driven live simulation. "
            "Keep production defaults unchanged; launcher raises this only "
            "for slow sim-clock MuJoCo validation runs."
        ),
    )
    parser.add_argument("--run-lingtu-frontier", action="store_true")
    parser.add_argument("--run-lingtu-tare", action="store_true")
    parser.add_argument("--run-lingtu-inspection", action="store_true")
    parser.add_argument(
        "--require-goal-arrival",
        action="store_true",
        help="Fail the frontier gate unless the active frontier approach goal is reached.",
    )
    parser.add_argument("--goal-arrival-threshold", type=float, default=0.30)
    parser.add_argument("--tare-min-goals", type=int, default=2)
    parser.add_argument("--tare-start-delay", type=float, default=0.0)
    parser.add_argument("--tare-goal-timeout", type=float, default=180.0)
    parser.add_argument("--tare-scenario", default="indoor")
    parser.add_argument("--frontier-min-goals", type=int, default=3)
    parser.add_argument("--frontier-start-delay", type=float, default=0.0)
    parser.add_argument("--frontier-goal-timeout", type=float, default=30.0)
    parser.add_argument(
        "--inspection-goals",
        default="0.8,0.0;1.6,0.2;2.4,0.4",
        help="Inspection patrol checkpoints as 'x,y[,z];...' or JSON list.",
    )
    parser.add_argument("--inspection-min-checkpoints", type=int, default=3)
    parser.add_argument("--inspection-start-delay", type=float, default=0.0)
    parser.add_argument("--inspection-goal-timeout", type=float, default=90.0)
    parser.add_argument(
        "--inspection-downsample-dist",
        type=float,
        default=0.35,
        help=(
            "Global path waypoint spacing used by Navigation for "
            "simulation-only inspection runs."
        ),
    )
    parser.add_argument(
        "--inspection-planner",
        choices=["astar", "pct"],
        default="astar",
        help=(
            "Global planner for LingTu inspection mode. Use pct with "
            "--inspection-tomogram to validate static tomogram global routing "
            "while Fast-LIO feeds live localization/local maps."
        ),
    )
    parser.add_argument(
        "--inspection-tomogram",
        type=Path,
        default=None,
        help="Tomogram path used when --inspection-planner=pct.",
    )
    parser.add_argument(
        "--inspection-waypoint-threshold",
        type=float,
        default=0.50,
        help=(
            "Navigation intermediate patrol waypoint radius for "
            "simulation-only inspection runs."
        ),
    )
    parser.add_argument(
        "--inspection-final-waypoint-threshold",
        type=float,
        default=0.50,
        help=(
            "Navigation final patrol waypoint radius for simulation-only "
            "inspection runs."
        ),
    )
    parser.add_argument(
        "--inspection-complete-path-on-goal-proximity",
        action="store_true",
        help=(
            "Simulation-only inspection guard: allow Navigation to mark a "
            "path complete when the robot is within the patrol goal radius even "
            "if an intermediate waypoint was skipped. Off by default."
        ),
    )
    parser.add_argument(
        "--inspection-goal-proximity-completion-threshold",
        type=float,
        default=None,
        help=(
            "Goal-proximity completion radius used only with "
            "--inspection-complete-path-on-goal-proximity. Defaults to the "
            "Navigation final waypoint threshold."
        ),
    )
    parser.add_argument(
        "--inspection-path-goal-tolerance",
        type=float,
        default=0.12,
        help="PathFollower local-path stop radius for simulation-only inspection runs.",
    )
    parser.add_argument(
        "--inspection-path-lookahead",
        type=float,
        default=1.5,
        help="PathFollower lookahead for simulation-only inspection runs.",
    )
    parser.add_argument(
        "--inspection-path-min-speed",
        type=float,
        default=0.15,
        help="PathFollower minimum speed for simulation-only inspection runs.",
    )
    parser.add_argument(
        "--inspection-path-yaw-rate-gain",
        type=float,
        default=7.5,
        help="PathFollower yaw-rate gain for simulation-only inspection runs.",
    )
    parser.add_argument(
        "--inspection-path-stop-yaw-rate-gain",
        type=float,
        default=7.5,
        help=(
            "PathFollower yaw-rate gain used when nearly stopped for "
            "simulation-only inspection runs."
        ),
    )
    parser.add_argument(
        "--inspection-path-dir-diff-thre",
        type=float,
        default=0.1,
        help=(
            "PathFollower heading-error threshold for allowing acceleration "
            "during simulation-only inspection runs."
        ),
    )
    parser.add_argument("--min-map-area-growth-m2", type=float, default=0.25)
    parser.add_argument("--min-explored-area-growth-m2", type=float, default=0.25)
    parser.add_argument("--min-exploration-coverage-growth-ratio", type=float, default=0.001)
    parser.add_argument(
        "--max-fastlio-z-drift-m",
        type=float,
        default=1.0,
        help="Maximum allowed Fast-LIO Z drift relative to MuJoCo motion in one gate.",
    )
    parser.add_argument(
        "--max-fastlio-yaw-drift-rad",
        type=float,
        default=0.5,
        help="Maximum allowed Fast-LIO yaw delta drift relative to MuJoCo motion.",
    )
    parser.add_argument(
        "--runtime-fault-confirm-samples",
        type=int,
        default=2,
        help="Consecutive runtime guard samples required before aborting a live run.",
    )
    parser.add_argument(
        "--runtime-motion-fault-min-sim-m",
        type=float,
        default=0.25,
        help=(
            "Minimum MuJoCo displacement before a Fast-LIO XY motion mismatch "
            "can terminate the run. The final motion_consistency check remains "
            "strict and always reports the actual error."
        ),
    )
    parser.add_argument("--n-rays", type=int, default=6400)
    parser.add_argument("--mid360-pattern", type=Path, default=DEFAULT_MID360_PATTERN)
    parser.add_argument("--mid360-samples-per-frame", type=int, default=DEFAULT_MID360_SAMPLES_PER_FRAME)
    parser.add_argument(
        "--lidar-backend",
        choices=["mujoco_lidar", "ray_caster_lidar"],
        default="mujoco_lidar",
        help=(
            "Strict MuJoCo LiDAR backend for this portable/ROS LIO hard gate. "
            "mujoco_lidar uses the discoverse-dev/MuJoCo-LiDAR package; "
            "ray_caster_lidar requires a loaded mujoco.sensor.ray_caster_lidar plugin."
        ),
    )
    parser.add_argument(
        "--mujoco-lidar-backend",
        choices=["cpu", "taichi", "warp", "jax"],
        default="cpu",
        help="MuJoCo-LiDAR implementation backend; cpu is the portable validation default.",
    )
    parser.add_argument(
        "--allow-legacy-lidar-fallback",
        action="store_true",
        help=(
            "Diagnostic-only escape hatch: let the engine initialize legacy mj_multiRay "
            "if the product backend is unavailable. The hard gate still reports a blocker."
        ),
    )
    parser.add_argument(
        "--localization-backend",
        default="",
        help=(
            "Localization/mapping backend for the live gate. The old portable_lio "
            "backend was removed because it was not real Fast-LIO2."
        ),
    )
    parser.add_argument(
        "--fastlio-lidar-input",
        choices=["livox_custom_msg", "timed_pointcloud2"],
        default="livox_custom_msg",
        help="Raw LiDAR message shape sent to Fast-LIO2.",
    )
    parser.add_argument("--fastlio-lidar-filter-num", type=int, default=4)
    parser.add_argument("--fastlio-scan-resolution", type=float, default=0.15)
    parser.add_argument("--fastlio-map-resolution", type=float, default=0.3)
    parser.add_argument("--fastlio-near-search-num", type=int, default=5)
    parser.add_argument("--fastlio-ieskf-max-iter", type=int, default=5)
    parser.add_argument("--fastlio-lidar-cov-inv", type=float, default=1000.0)
    parser.add_argument(
        "--fastlio-time-diff-lidar-to-imu",
        type=float,
        default=0.0,
        help=(
            "Fast-LIO time_diff_lidar_to_imu control in seconds. It is applied "
            "inside Fast-LIO to align IMU timestamps against LiDAR timestamps."
        ),
    )
    parser.add_argument(
        "--fastlio-vertical-velocity-constraint",
        choices=["auto", "on", "off"],
        default="off",
        help=(
            "Optional world-frame v_z=0 pseudo-observation for planar kinematic "
            "MuJoCo ablations. It is off by default because it can hide or move "
            "SLAM drift; real robot Fast-LIO configs also keep this disabled."
        ),
    )
    parser.add_argument(
        "--scan-time-profile",
        choices=["instantaneous", "synthetic_rolling", "physical_rolling"],
        default="physical_rolling",
        help=(
            "Per-point time model for the simulated scan. physical_rolling "
            "accumulates subscans captured across the LiDAR scan window and is "
            "the strict validation default. synthetic_rolling keeps the older "
            "single-snapshot rays with MID-360-style offsets for ablation; "
            "instantaneous uses zero offsets."
        ),
    )
    parser.add_argument(
        "--imu-acc-mode",
        choices=["gravity_only", "finite_difference"],
        default="finite_difference",
        help=(
            "IMU linear acceleration model. finite_difference is the MuJoCo "
            "live validation default because Fast-LIO needs translational "
            "acceleration during turns; moving live gates still disable ZUPT "
            "so constant-velocity motion is not misclassified as static."
        ),
    )
    parser.add_argument(
        "--nav-data-source",
        choices=["fastlio2", "mujoco_ground_truth"],
        default="fastlio2",
        help=(
            f"Source for canonical {TOPICS.odometry}, {TOPICS.registered_cloud}, "
            f"and {TOPICS.map_cloud}. Use fastlio2 for strict SLAM validation; "
            "use mujoco_ground_truth for stable visible demos while keeping "
            "Fast-LIO diagnostics on their raw topics."
        ),
    )
    parser.add_argument(
        "--allow-golden-spiral-lidar",
        action="store_true",
        help="Use the legacy synthetic ray fan instead of the official MID-360 scan pattern.",
    )
    parser.add_argument("--mujoco-memory", default="64M")
    parser.add_argument("--startup-sleep", type=float, default=2.0)
    parser.add_argument("--settle-sleep", type=float, default=1.0)
    parser.add_argument("--backend-profile", default="fastlio2")
    parser.add_argument("--work-dir", default="artifacts/mujoco_fastlio2_live")
    parser.add_argument("--json-out", default="")
    parser.add_argument(
        "--partial-json-out",
        default="",
        help=(
            "Optional diagnostic JSON path updated while the live gate is still "
            "running. This is always red/partial evidence and is not an "
            "acceptance report."
        ),
    )
    parser.add_argument(
        "--video-out",
        default="",
        help="Optional MP4 path rendering raw MID-360 scans, Fast-LIO map cloud, and odometry evidence.",
    )
    parser.add_argument("--video-fps", type=float, default=8.0)
    parser.add_argument(
        "--moving-obstacle-mode",
        choices=["none", "robot_crossing"],
        default="none",
        help="Inject simulation-only moving obstacle returns into the raw MID-360 input.",
    )
    parser.add_argument("--moving-obstacle-count", type=int, default=1)
    parser.add_argument("--moving-obstacle-start-s", type=float, default=8.0)
    parser.add_argument("--moving-obstacle-duration-s", type=float, default=0.0)
    parser.add_argument("--moving-obstacle-period-s", type=float, default=10.0)
    parser.add_argument("--moving-obstacle-forward-m", type=float, default=2.0)
    parser.add_argument("--moving-obstacle-forward-step-m", type=float, default=0.8)
    parser.add_argument("--moving-obstacle-lateral-phase-step-rad", type=float, default=math.pi / 2.0)
    parser.add_argument("--moving-obstacle-lateral-amplitude-m", type=float, default=0.75)
    parser.add_argument("--moving-obstacle-along-amplitude-m", type=float, default=0.25)
    parser.add_argument("--moving-obstacle-radius-m", type=float, default=0.16)
    parser.add_argument("--moving-obstacle-height-m", type=float, default=0.60)
    parser.add_argument("--moving-obstacle-point-spacing", type=float, default=0.10)
    parser.add_argument("--moving-obstacle-intensity", type=float, default=220.0)
    parser.add_argument("--moving-obstacle-robot-radius-m", type=float, default=0.28)
    parser.add_argument("--max-yaw-per-meter", type=float, default=1.2)
    parser.add_argument("--max-angular-saturation-ratio", type=float, default=0.35)
    parser.add_argument(
        "--no-save-map-artifacts",
        action="store_true",
        help=f"Do not persist same-source map.pcd/metadata artifacts from {TOPICS.map_cloud}.",
    )
    parser.add_argument(
        "--build-tomogram",
        action="store_true",
        help="Build a same-source PCT tomogram.pickle from the saved map.pcd artifact.",
    )
    parser.add_argument("--map-artifact-voxel-size", type=float, default=0.10)
    parser.add_argument("--map-artifact-max-points", type=int, default=250_000)
    parser.add_argument("--map-artifact-max-span-m", type=float, default=120.0)
    parser.add_argument("--tomogram-resolution", type=float, default=0.20)
    parser.add_argument("--tomogram-slice-dh", type=float, default=0.25)
    parser.add_argument("--tomogram-ground-h", type=float, default=0.0)
    parser.add_argument("--tomogram-max-cells", type=int, default=50_000_000)
    parser.add_argument(
        "--show-mujoco-window",
        action="store_true",
        help="Open a live MuJoCo chase-camera window for visible desktop demos.",
    )
    parser.add_argument("--mujoco-window-fps", type=float, default=10.0)
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    try:
        report = run_gate(
            world=_resolve_world(args.world),
            duration=args.duration,
            drive_vx=args.drive_vx,
            drive_vy=args.drive_vy,
            drive_wz=args.drive_wz,
            n_rays=args.n_rays,
            start=_parse_start(args.start),
            mujoco_memory=args.mujoco_memory,
            mid360_pattern=None if args.allow_golden_spiral_lidar else args.mid360_pattern,
            mid360_samples_per_frame=args.mid360_samples_per_frame,
            lidar_backend=args.lidar_backend,
            mujoco_lidar_backend=args.mujoco_lidar_backend,
            allow_legacy_lidar_fallback=args.allow_legacy_lidar_fallback,
            startup_sleep=args.startup_sleep,
            settle_sleep=args.settle_sleep,
            work_dir=Path(args.work_dir),
            backend_profile=args.backend_profile,
            drive_mode=args.drive_mode,
            duration_clock=args.duration_clock,
            max_wall_time_s=args.max_wall_time_s,
            drive_source=args.drive_source,
            cmd_vel_timeout=args.cmd_vel_timeout,
            cmd_vel_linear_limit=args.cmd_vel_linear_limit,
            cmd_vel_angular_limit=args.cmd_vel_angular_limit,
            cmd_vel_sim_linear_scale=args.cmd_vel_sim_linear_scale,
            cmd_vel_sim_angular_scale=args.cmd_vel_sim_angular_scale,
            cmd_vel_linear_accel_limit=args.cmd_vel_linear_accel_limit,
            cmd_vel_angular_accel_limit=args.cmd_vel_angular_accel_limit,
            nav_max_linear_speed=args.nav_max_linear_speed,
            nav_max_angular_z=args.nav_max_angular_z,
            nav_turn_speed_yaw_rate_start=args.nav_turn_speed_yaw_rate_start,
            nav_turn_speed_min_scale=args.nav_turn_speed_min_scale,
            cmd_vel_mux_source_timeout=args.cmd_vel_mux_source_timeout,
            run_lingtu_frontier=args.run_lingtu_frontier,
            run_lingtu_tare=args.run_lingtu_tare,
            run_lingtu_inspection=args.run_lingtu_inspection,
            require_goal_arrival=args.require_goal_arrival,
            goal_arrival_threshold=args.goal_arrival_threshold,
            tare_min_goals=args.tare_min_goals,
            tare_start_delay=args.tare_start_delay,
            tare_goal_timeout=args.tare_goal_timeout,
            tare_scenario=args.tare_scenario,
            frontier_min_goals=args.frontier_min_goals,
            frontier_start_delay=args.frontier_start_delay,
            frontier_goal_timeout=args.frontier_goal_timeout,
            inspection_goals=args.inspection_goals,
            inspection_min_checkpoints=args.inspection_min_checkpoints,
            inspection_start_delay=args.inspection_start_delay,
            inspection_goal_timeout=args.inspection_goal_timeout,
            inspection_downsample_dist=args.inspection_downsample_dist,
            inspection_planner=args.inspection_planner,
            inspection_tomogram=args.inspection_tomogram,
            inspection_waypoint_threshold=args.inspection_waypoint_threshold,
            inspection_final_waypoint_threshold=args.inspection_final_waypoint_threshold,
            inspection_complete_path_on_goal_proximity=(
                args.inspection_complete_path_on_goal_proximity
            ),
            inspection_goal_proximity_completion_threshold=(
                args.inspection_goal_proximity_completion_threshold
            ),
            inspection_path_goal_tolerance=args.inspection_path_goal_tolerance,
            inspection_path_lookahead=args.inspection_path_lookahead,
            inspection_path_min_speed=args.inspection_path_min_speed,
            inspection_path_yaw_rate_gain=args.inspection_path_yaw_rate_gain,
            inspection_path_stop_yaw_rate_gain=args.inspection_path_stop_yaw_rate_gain,
            inspection_path_dir_diff_thre=args.inspection_path_dir_diff_thre,
            min_map_area_growth_m2=args.min_map_area_growth_m2,
            min_explored_area_growth_m2=args.min_explored_area_growth_m2,
            min_exploration_coverage_growth_ratio=args.min_exploration_coverage_growth_ratio,
            max_fastlio_z_drift_m=args.max_fastlio_z_drift_m,
            max_fastlio_yaw_drift_rad=args.max_fastlio_yaw_drift_rad,
            runtime_fault_confirm_samples=args.runtime_fault_confirm_samples,
            runtime_motion_fault_min_sim_m=args.runtime_motion_fault_min_sim_m,
            localization_backend=args.localization_backend,
            fastlio_lidar_input=args.fastlio_lidar_input,
            fastlio_lidar_filter_num=args.fastlio_lidar_filter_num,
            fastlio_scan_resolution=args.fastlio_scan_resolution,
            fastlio_map_resolution=args.fastlio_map_resolution,
            fastlio_near_search_num=args.fastlio_near_search_num,
            fastlio_ieskf_max_iter=args.fastlio_ieskf_max_iter,
            fastlio_lidar_cov_inv=args.fastlio_lidar_cov_inv,
            fastlio_time_diff_lidar_to_imu=args.fastlio_time_diff_lidar_to_imu,
            fastlio_vertical_velocity_constraint=args.fastlio_vertical_velocity_constraint,
            scan_time_profile=args.scan_time_profile,
            imu_acc_mode=args.imu_acc_mode,
            nav_data_source=args.nav_data_source,
            show_mujoco_window=args.show_mujoco_window,
            mujoco_window_fps=args.mujoco_window_fps,
            video_out=args.video_out or None,
            video_fps=args.video_fps,
            moving_obstacle_mode=args.moving_obstacle_mode,
            moving_obstacle_count=args.moving_obstacle_count,
            moving_obstacle_start_s=args.moving_obstacle_start_s,
            moving_obstacle_duration_s=args.moving_obstacle_duration_s,
            moving_obstacle_period_s=args.moving_obstacle_period_s,
            moving_obstacle_forward_m=args.moving_obstacle_forward_m,
            moving_obstacle_forward_step_m=args.moving_obstacle_forward_step_m,
            moving_obstacle_lateral_phase_step_rad=args.moving_obstacle_lateral_phase_step_rad,
            moving_obstacle_lateral_amplitude_m=args.moving_obstacle_lateral_amplitude_m,
            moving_obstacle_along_amplitude_m=args.moving_obstacle_along_amplitude_m,
            moving_obstacle_radius_m=args.moving_obstacle_radius_m,
            moving_obstacle_height_m=args.moving_obstacle_height_m,
            moving_obstacle_point_spacing=args.moving_obstacle_point_spacing,
            moving_obstacle_intensity=args.moving_obstacle_intensity,
            moving_obstacle_robot_radius_m=args.moving_obstacle_robot_radius_m,
            max_yaw_per_meter=args.max_yaw_per_meter,
            max_angular_saturation_ratio=args.max_angular_saturation_ratio,
            save_map_artifacts=not args.no_save_map_artifacts,
            build_tomogram=args.build_tomogram,
            map_artifact_voxel_size=args.map_artifact_voxel_size,
            map_artifact_max_points=args.map_artifact_max_points,
            map_artifact_max_span_m=args.map_artifact_max_span_m,
            tomogram_resolution=args.tomogram_resolution,
            tomogram_slice_dh=args.tomogram_slice_dh,
            tomogram_ground_h=args.tomogram_ground_h,
            tomogram_max_cells=args.tomogram_max_cells,
            partial_json_out=(
                Path(args.partial_json_out)
                if args.partial_json_out
                else Path(args.json_out).with_suffix(".partial.json")
                if args.json_out
                else None
            ),
        )
    except Exception as exc:
        partial_report, partial_report_path = _load_exception_partial_report(args)
        report = _gate_exception_report(
            args,
            exc,
            partial_report=partial_report,
            partial_report_path=partial_report_path,
        )
    text = json.dumps(report, indent=2, sort_keys=True, default=str)
    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n", encoding="utf-8")
    try:
        print(text)
    except BrokenPipeError:
        pass
    return 0 if report.get("ok") or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())
