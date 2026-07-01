"""Pure-Python CMU local planner scorer."""

from __future__ import annotations

import math

from nav.services.plan.local_planner.models import (
    CmuPyLocalPlannerDecision,
    CmuPyLocalPlannerRequest,
    DIR_THRE,
    DIR_WEIGHT,
    GROUND_HEIGHT_THRE,
    GROUP_NUM,
    OBSTACLE_HEIGHT_THRE,
    PATH_NUM,
    PATH_RANGE,
    POINT_PER_PATH_THRE,
)
from runtime.msgs.numpy_compat import np

ROT_SIN = tuple(math.sin((10.0 * i - 180.0) * math.pi / 180.0) for i in range(36))
ROT_COS = tuple(math.cos((10.0 * i - 180.0) * math.pi / 180.0) for i in range(36))


def score_cmu_py_paths(
    obstacle_pts: np.ndarray,
    rel_goal_x: float,
    rel_goal_y: float,
    rel_goal_dis: float,
    joy_dir_deg: float,
    correspondences: dict[int, list[int]],
    group_of_path: np.ndarray,
    grid_voxel_num_x: int,
    grid_voxel_num_y: int,
    *,
    grid_voxel_size: float = 0.02,
    grid_voxel_offset_x: float = 3.2,
    grid_voxel_offset_y: float = 5.25,
    search_radius: float = 0.45,
    obstacle_height_thre: float = OBSTACLE_HEIGHT_THRE,
    ground_height_thre: float = GROUND_HEIGHT_THRE,
    point_per_path_thre: int = POINT_PER_PATH_THRE,
    dir_weight: float = DIR_WEIGHT,
    dir_thre: float = DIR_THRE,
    path_range: float = PATH_RANGE,
    slope_weight: float = 0.0,
    use_cost: bool = False,
) -> np.ndarray:
    """Score CMU local planner path groups for the pure-Python runtime."""
    inv_gs = 1.0 / grid_voxel_size
    offset_x_half = grid_voxel_offset_x + grid_voxel_size * 0.5
    offset_y_half = grid_voxel_offset_y + grid_voxel_size * 0.5
    scale_y_coef_a = search_radius / grid_voxel_offset_y
    scale_y_coef_b = 1.0 / grid_voxel_offset_x
    path_range_sq = path_range * path_range

    clear_path_list = np.zeros(PATH_NUM * 36, dtype=np.int32)
    path_penalty_list = np.zeros(PATH_NUM * 36, dtype=np.float32)
    clear_per_group_score = np.zeros(36 * GROUP_NUM, dtype=np.float64)

    ang_diff_list = np.array(
        [abs(joy_dir_deg - (10.0 * d - 180.0)) % 360.0 for d in range(36)],
        dtype=np.float32,
    )
    ang_diff_list = np.where(
        ang_diff_list > 180.0,
        360.0 - ang_diff_list,
        ang_diff_list,
    )
    rot_dir_valid = ang_diff_list <= dir_thre

    if obstacle_pts.shape[0] > 0:
        xs = obstacle_pts[:, 0].astype(np.float64)
        ys = obstacle_pts[:, 1].astype(np.float64)
        if obstacle_pts.shape[1] >= 4:
            hs = obstacle_pts[:, 3].astype(np.float64)
        else:
            hs = obstacle_pts[:, 2].astype(np.float64)

        dist_sq = xs * xs + ys * ys
        in_range = dist_sq < path_range_sq

        for rot_dir in range(36):
            if not rot_dir_valid[rot_dir]:
                continue
            rc = ROT_COS[rot_dir]
            rs = ROT_SIN[rot_dir]
            x2 = rc * xs + rs * ys
            y2 = -rs * xs + rc * ys

            scale_y = (
                x2 * scale_y_coef_b
                + scale_y_coef_a * (grid_voxel_offset_x - x2) * scale_y_coef_b
            )
            valid_scale = np.abs(scale_y) > 1e-6
            ind_x = ((offset_x_half - x2) * inv_gs).astype(np.int32)
            safe_scale_y = np.where(valid_scale, scale_y, 1.0)
            ind_y = ((offset_y_half - y2 / safe_scale_y) * inv_gs).astype(np.int32)

            grid_valid = (
                valid_scale
                & in_range
                & (ind_x >= 0)
                & (ind_x < grid_voxel_num_x)
                & (ind_y >= 0)
                & (ind_y < grid_voxel_num_y)
            )

            for vi in np.where(grid_valid)[0]:
                voxel_id = int(ind_x[vi]) * grid_voxel_num_y + int(ind_y[vi])
                if voxel_id not in correspondences:
                    continue
                h = float(hs[vi])
                for path_id in correspondences[voxel_id]:
                    idx = PATH_NUM * rot_dir + path_id
                    if h > obstacle_height_thre:
                        clear_path_list[idx] += 1
                    elif use_cost and h > ground_height_thre:
                        if path_penalty_list[idx] < h:
                            path_penalty_list[idx] = h

    omni_thre = 5.0

    for rot_dir in range(36):
        if not rot_dir_valid[rot_dir]:
            continue
        ang_diff = float(ang_diff_list[rot_dir])
        dw = abs(dir_weight * ang_diff)
        sqrt_sqrt_dw = math.sqrt(math.sqrt(dw))
        if rot_dir < 18:
            rot_dir_w = abs(abs(rot_dir - 9.0) + 1.0)
        else:
            rot_dir_w = abs(abs(rot_dir - 27.0) + 1.0)
        rot_dir_w2 = rot_dir_w * rot_dir_w
        rot_dir_w4 = rot_dir_w2 * rot_dir_w2

        base = PATH_NUM * rot_dir
        for path_id in range(PATH_NUM):
            if clear_path_list[base + path_id] >= point_per_path_thre:
                continue
            group_id = int(group_of_path[path_id])
            group_dir_w = 4.0 - abs(group_id - 3.0)
            terrain_penalty = float(path_penalty_list[base + path_id])
            terrain_factor = (
                max(0.0, 1.0 - slope_weight * terrain_penalty)
                if slope_weight > 0.0
                else 1.0
            )
            if rel_goal_dis < omni_thre:
                score = (
                    (1.0 - sqrt_sqrt_dw)
                    * group_dir_w
                    * group_dir_w
                    * terrain_factor
                )
            else:
                score = (1.0 - sqrt_sqrt_dw) * rot_dir_w4 * terrain_factor
            clear_per_group_score[GROUP_NUM * rot_dir + group_id] += score

    return clear_per_group_score


def _cmu_py_rot_obstacle_window(
    obstacle_pts: np.ndarray,
    *,
    vehicle_length: float,
    vehicle_width: float,
    obstacle_height_thre: float,
) -> tuple[float, float]:
    min_obs_ang_cw = -180.0
    min_obs_ang_ccw = 180.0
    if obstacle_pts.shape[0] == 0:
        return min_obs_ang_cw, min_obs_ang_ccw

    half_len = vehicle_length * 0.5
    half_wid = vehicle_width * 0.5
    diameter_sq = half_len * half_len + half_wid * half_wid
    ang_offset = math.atan2(vehicle_width, vehicle_length) * 180.0 / math.pi
    for pt in obstacle_pts:
        x = float(pt[0])
        y = float(pt[1])
        h = float(pt[3]) if len(pt) >= 4 else float(pt[2])
        dis_sq = x * x + y * y
        if (
            dis_sq < diameter_sq
            and (abs(x) > half_len or abs(y) > half_wid)
            and h > obstacle_height_thre
        ):
            ang_obs = math.atan2(y, x) * 180.0 / math.pi
            if ang_obs > 0.0:
                min_obs_ang_ccw = min(min_obs_ang_ccw, ang_obs - ang_offset)
                min_obs_ang_cw = max(min_obs_ang_cw, ang_obs + ang_offset - 180.0)
            else:
                min_obs_ang_cw = max(min_obs_ang_cw, ang_obs + ang_offset)
                min_obs_ang_ccw = min(min_obs_ang_ccw, 180.0 + ang_obs - ang_offset)

    if min_obs_ang_cw > 0.0:
        min_obs_ang_cw = 0.0
    if min_obs_ang_ccw < 0.0:
        min_obs_ang_ccw = 0.0
    return min_obs_ang_cw, min_obs_ang_ccw


def _select_cmu_py_group(
    group_scores: np.ndarray,
    *,
    check_rot_obstacle: bool,
    two_way_drive: bool,
    min_obs_ang_cw: float,
    min_obs_ang_ccw: float,
) -> tuple[int, float]:
    selected_group_id = -1
    max_score = 0.0
    for i in range(36 * GROUP_NUM):
        rot_dir = i // GROUP_NUM
        rot_ang_deg = 10.0 * rot_dir - 180.0
        rot_deg = 10.0 * rot_dir
        if rot_deg > 180.0:
            rot_deg -= 360.0
        rot_ok = (
            (rot_ang_deg > min_obs_ang_cw and rot_ang_deg < min_obs_ang_ccw)
            or (
                two_way_drive
                and rot_deg > min_obs_ang_cw
                and rot_deg < min_obs_ang_ccw
            )
            or not check_rot_obstacle
        )
        if rot_ok and group_scores[i] > max_score:
            max_score = float(group_scores[i])
            selected_group_id = i
    return selected_group_id, max_score


def apply_traversability_cost_to_group_scores(
    group_scores: np.ndarray,
    start_paths: dict[int, np.ndarray] | list[np.ndarray] | tuple[np.ndarray, ...],
    *,
    cos_yaw: float,
    sin_yaw: float,
    planner_pos: np.ndarray,
    rel_goal_dis: float,
    path_range: float,
    grid: np.ndarray | None,
    resolution: float,
    origin: np.ndarray | None,
    hard_cost: float = 90.0,
    soft_cost: float = 40.0,
    weight: float = 0.01,
) -> np.ndarray:
    if grid is None or origin is None or resolution <= 0.0:
        return group_scores
    risk_grid = np.asarray(grid, dtype=np.float32)
    grid_origin = np.asarray(origin, dtype=np.float32)
    if risk_grid.ndim != 2 or grid_origin.shape[0] < 2:
        return group_scores

    out = group_scores.copy()
    max_dis = min(float(path_range), float(rel_goal_dis)) if rel_goal_dis > 0 else float(path_range)
    max_dis_sq = max_dis * max_dis
    rows, cols = risk_grid.shape
    for rot_dir in range(36):
        rc = ROT_COS[rot_dir]
        rs = ROT_SIN[rot_dir]
        for group_id in range(GROUP_NUM):
            seg = start_paths[group_id]
            max_risk = 0.0
            for pt in seg:
                px = float(pt[0])
                py = float(pt[1])
                if px * px + py * py > max_dis_sq:
                    break
                bx = rc * px - rs * py
                by = rs * px + rc * py
                wx = bx * cos_yaw - by * sin_yaw + planner_pos[0]
                wy = bx * sin_yaw + by * cos_yaw + planner_pos[1]
                col = int((wx - grid_origin[0]) / resolution)
                row = int((wy - grid_origin[1]) / resolution)
                if row < 0 or row >= rows or col < 0 or col >= cols:
                    continue
                risk = float(risk_grid[row, col])
                if math.isfinite(risk) and risk > max_risk:
                    max_risk = risk
            idx = rot_dir * GROUP_NUM + group_id
            if max_risk >= hard_cost:
                out[idx] = 0.0
            elif max_risk > soft_cost:
                factor = max(0.0, min(1.0, 1.0 - weight * (max_risk - soft_cost)))
                out[idx] *= factor
    return out


def plan_cmu_py_local_path(
    request: CmuPyLocalPlannerRequest,
) -> CmuPyLocalPlannerDecision | None:
    """Run the pure-Python CMU local planner and return a publishable decision."""
    path_data = request.path_data
    correspondences = path_data["correspondences"]
    group_of_path = path_data["group_of_path"]
    start_paths = path_data["start_paths"]
    grid_voxel_num_x = path_data["grid_voxel_num_x"]
    grid_voxel_num_y = path_data["grid_voxel_num_y"]

    robot_pos = np.asarray(request.robot_pos, dtype=float)
    goal = np.asarray(request.goal, dtype=float)
    cos_yaw = math.cos(float(request.robot_yaw))
    sin_yaw = math.sin(float(request.robot_yaw))
    planner_pos = robot_pos.astype(float, copy=True)
    planner_pos[0] = (
        robot_pos[0]
        - cos_yaw * float(request.sensor_offset_x)
        + sin_yaw * float(request.sensor_offset_y)
    )
    planner_pos[1] = (
        robot_pos[1]
        - sin_yaw * float(request.sensor_offset_x)
        - cos_yaw * float(request.sensor_offset_y)
    )

    gx = float(goal[0] - planner_pos[0])
    gy = float(goal[1] - planner_pos[1])
    rel_goal_x = gx * cos_yaw + gy * sin_yaw
    rel_goal_y = -gx * sin_yaw + gy * cos_yaw
    rel_goal_dis = math.sqrt(rel_goal_x * rel_goal_x + rel_goal_y * rel_goal_y)
    joy_dir_deg = math.atan2(rel_goal_y, rel_goal_x) * 180.0 / math.pi
    joy_dir_deg = max(-95.0, min(95.0, joy_dir_deg))

    merged = np.asarray(request.obstacle_points_world, dtype=np.float32)
    if merged.shape[0] > 0:
        dx = merged[:, 0] - planner_pos[0]
        dy = merged[:, 1] - planner_pos[1]
        bx = dx * cos_yaw + dy * sin_yaw
        by = -dx * sin_yaw + dy * cos_yaw
        bz = (
            merged[:, 2]
            if merged.shape[1] >= 3
            else np.zeros(len(dx), dtype=np.float32)
        )
        intensity = merged[:, 3]
        obs_pts = np.stack([bx, by, bz, intensity], axis=1).astype(np.float32)
    else:
        obs_pts = np.zeros((0, 3), dtype=np.float32)

    group_scores = score_cmu_py_paths(
        obs_pts,
        rel_goal_x,
        rel_goal_y,
        rel_goal_dis,
        joy_dir_deg,
        correspondences,
        group_of_path,
        grid_voxel_num_x,
        grid_voxel_num_y,
        grid_voxel_size=request.grid_voxel_size,
        grid_voxel_offset_x=request.grid_voxel_offset_x,
        grid_voxel_offset_y=request.grid_voxel_offset_y,
        search_radius=request.grid_search_radius,
        obstacle_height_thre=request.obstacle_height_thre,
        ground_height_thre=request.ground_height_thre,
        point_per_path_thre=request.point_per_path_thre,
        dir_weight=request.dir_weight,
        dir_thre=request.dir_thre,
        path_range=request.path_range,
        slope_weight=request.slope_weight,
        use_cost=request.use_cost,
    )
    if request.use_traversability_cost:
        group_scores = apply_traversability_cost_to_group_scores(
            group_scores,
            start_paths,
            cos_yaw=cos_yaw,
            sin_yaw=sin_yaw,
            planner_pos=planner_pos,
            rel_goal_dis=rel_goal_dis,
            path_range=request.path_range,
            grid=request.traversability_grid,
            resolution=request.traversability_resolution,
            origin=request.traversability_origin,
            hard_cost=request.traversability_hard_cost,
            soft_cost=request.traversability_soft_cost,
            weight=request.traversability_weight,
        )

    min_obs_ang_cw, min_obs_ang_ccw = _cmu_py_rot_obstacle_window(
        obs_pts,
        vehicle_length=request.vehicle_length,
        vehicle_width=request.vehicle_width,
        obstacle_height_thre=request.obstacle_height_thre,
    )
    selected_group_id, max_score = _select_cmu_py_group(
        group_scores,
        check_rot_obstacle=request.check_rot_obstacle,
        two_way_drive=request.two_way_drive,
        min_obs_ang_cw=min_obs_ang_cw,
        min_obs_ang_ccw=min_obs_ang_ccw,
    )

    if selected_group_id < 0 or max_score <= 0.0:
        return CmuPyLocalPlannerDecision(
            world_path=[],
            path_found=False,
            safety_stop=True,
            reason="no_feasible_local_path",
            score=max_score,
            obstacle_point_count=int(merged.shape[0]),
        )

    rot_dir = selected_group_id // GROUP_NUM
    group_id = selected_group_id % GROUP_NUM
    seg = start_paths[group_id]
    if seg.shape[0] == 0:
        return None

    rc = ROT_COS[rot_dir]
    rs = ROT_SIN[rot_dir]
    world_path: list[tuple[float, float, float]] = []
    for pt in seg:
        x_body = rc * float(pt[0]) - rs * float(pt[1])
        y_body = rs * float(pt[0]) + rc * float(pt[1])
        z_body = float(pt[2])
        x_world = (x_body * cos_yaw - y_body * sin_yaw) + planner_pos[0]
        y_world = (x_body * sin_yaw + y_body * cos_yaw) + planner_pos[1]
        z_world = z_body + planner_pos[2]
        world_path.append((float(x_world), float(y_world), float(z_world)))

    return CmuPyLocalPlannerDecision(
        world_path=world_path,
        path_found=True,
        safety_stop=False,
        reason="cmu_py_path",
        selected_group_id=selected_group_id,
        selected_rot_dir=rot_dir,
        selected_path_group=group_id,
        score=max_score,
        obstacle_point_count=int(merged.shape[0]),
    )
