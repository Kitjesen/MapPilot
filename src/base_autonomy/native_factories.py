"""NativeModule factories for the base autonomy C++ stack.

Each function builds a NativeModule that runs a compiled ROS2 node as a
managed subprocess.  Parameters come entirely from config/robot_config.yaml
via the typed RobotConfig; no values are hardcoded here.

Usage::

    from base_autonomy.native_factories import terrain_analysis, local_planner, path_follower
    from core.config import get_config

    cfg = get_config()
    bp.add(terrain_analysis(cfg), alias="terrain")
    bp.add(local_planner(cfg),    alias="local_planner")
    bp.add(path_follower(cfg),    alias="path_follower")
"""

from __future__ import annotations


from core.config import RobotConfig, get_config
from core.native_install import DDS_ENV, exe
from core.native_module import NativeModule, NativeModuleConfig
from core.runtime_interface import adapter_remappings


def terrain_analysis(cfg: RobotConfig | None = None) -> NativeModule:
    """Ground estimation + obstacle detection (terrainAnalysis C++ node)."""
    cfg = cfg or get_config()
    ta = cfg.raw.get("terrain", {})
    return NativeModule(NativeModuleConfig(
        executable=exe(cfg, "terrain_analysis", "terrainAnalysis"),
        name="terrain_analysis",
        parameters={
            "vehicleHeight":       cfg.geometry.vehicle_height,
            "obstacleHeightThre":  cfg.safety.obstacle_height_thre,
            "groundHeightThre":    cfg.safety.ground_height_thre,
            "terrainVoxelSize":    ta.get("scan_voxel_size",          0.05),
            "useSorting":          ta.get("use_sorting",               True),
            "quantileZ":           ta.get("quantile_z",                0.25),
            "considerDrop":        ta.get("consider_drop",             False),
            "limitGroundLift":     ta.get("limit_ground_lift",         False),
            "maxGroundLift":       ta.get("max_ground_lift",           0.15),
            "clearDyObs":          ta.get("clear_dy_obs",              False),
            "minDyObsDis":         ta.get("min_dy_obs_dis",            0.3),
            "absDyObsRelZThre":    ta.get("abs_dy_obs_rel_z_thre",     0.2),
            "minDyObsVFOV":        ta.get("min_dy_obs_vfov",          -16.0),
            "maxDyObsVFOV":        ta.get("max_dy_obs_vfov",           16.0),
            "minDyObsPointNum":    ta.get("min_dy_obs_point_num",      1),
            "noDataObstacle":      ta.get("no_data_obstacle",          False),
            "noDataBlockSkipNum":  ta.get("no_data_block_skip_num",    0),
            "minBlockPointNum":    ta.get("min_block_point_num",       10),
            "voxelPointUpdateThre":ta.get("voxel_point_update_thre",   100),
            "voxelTimeUpdateThre": ta.get("voxel_time_update_thre",    2.0),
            "minRelZ":             ta.get("min_rel_z",                -1.5),
            "maxRelZ":             ta.get("max_rel_z",                 0.2),
            "disRatioZ":           ta.get("dis_ratio_z",               0.2),
            "checkCollision":      ta.get("check_collision",           True),
        },
        remappings=adapter_remappings("terrain_analysis"),
        env=DDS_ENV,
    ))


def terrain_analysis_ext(cfg: RobotConfig | None = None) -> NativeModule:
    """Terrain connectivity + 2.5D height map (terrainAnalysisExt C++ node)."""
    cfg = cfg or get_config()
    ta = cfg.raw.get("terrain", {})
    return NativeModule(NativeModuleConfig(
        executable=exe(cfg, "terrain_analysis_ext", "terrainAnalysisExt"),
        name="terrain_analysis_ext",
        parameters={
            "vehicleHeight":        cfg.geometry.vehicle_height,
            "scanVoxelSize":        ta.get("scan_voxel_size",          0.05),
            "useSorting":           ta.get("use_sorting",               True),
            "quantileZ":            ta.get("quantile_z",                0.25),
            "voxelPointUpdateThre": ta.get("voxel_point_update_thre",   100),
            "voxelTimeUpdateThre":  ta.get("voxel_time_update_thre",    2.0),
            "lowerBoundZ":          ta.get("min_rel_z",                -1.5),
            "upperBoundZ":          ta.get("max_rel_z",                 0.2),
            "disRatioZ":            ta.get("dis_ratio_z",               0.2),
            "terrainUnderVehicle":  ta.get("terrain_under_vehicle",    -0.2),
            "terrainConnThre":      ta.get("terrain_conn_thre",         0.5),
            "ceilingFilteringThre": ta.get("ceiling_filtering_thre",    2.0),
            "checkTerrainConn":     ta.get("check_collision",           True),
        },
        remappings=adapter_remappings("terrain_analysis_ext"),
        env=DDS_ENV,
    ))


def local_planner(cfg: RobotConfig | None = None) -> NativeModule:
    """Obstacle avoidance + path scoring (localPlanner C++ node)."""
    cfg = cfg or get_config()
    lp  = cfg.raw.get("local_planner", {})
    aut = cfg.raw.get("autonomy", {})
    ta  = cfg.raw.get("terrain", {})
    return NativeModule(NativeModuleConfig(
        executable=exe(cfg, "local_planner", "localPlanner"),
        name="local_planner",
        parameters={
            "vehicleLength":    cfg.geometry.vehicle_length,
            "vehicleWidth":     cfg.geometry.vehicle_width,
            "sensorOffsetX":    cfg.geometry.sensor_offset_x,
            "sensorOffsetY":    cfg.geometry.sensor_offset_y,
            "maxSpeed":         cfg.speed.max_speed,
            "autonomySpeed":    cfg.speed.autonomy_speed,
            "obstacleHeightThre":  cfg.safety.obstacle_height_thre,
            "groundHeightThre":    cfg.safety.ground_height_thre,
            "twoWayDrive":         lp.get("two_way_drive",           True),
            "laserVoxelSize":      lp.get("laser_voxel_size",        0.05),
            "terrainVoxelSize":    ta.get("scan_voxel_size",         0.2),
            "useTerrainAnalysis":  True,
            "checkObstacle":       lp.get("check_obstacle",          True),
            "checkRotObstacle":    lp.get("check_rot_obstacle",      False),
            "adjacentRange":       lp.get("adjacent_range",          3.5),
            "costHeightThre1":     lp.get("cost_height_thre_1",      0.15),
            "costHeightThre2":     lp.get("cost_height_thre_2",      0.1),
            "useCost":             lp.get("use_cost",                False),
            "slopeWeight":         lp.get("slope_weight",            0.0),
            "slowPathNumThre":     lp.get("slow_path_num_thre",      5),
            "slowGroupNumThre":    lp.get("slow_group_num_thre",     1),
            "pointPerPathThre":    lp.get("point_per_path_thre",     2),
            "minRelZ":             lp.get("min_rel_z", ta.get("min_rel_z", -1.5)),
            "maxRelZ":             lp.get("max_rel_z", ta.get("max_rel_z", 0.2)),
            "dirWeight":           lp.get("dir_weight",              0.02),
            "dirThre":             lp.get("dir_thre",                90.0),
            "dirToVehicle":        lp.get("dir_to_vehicle",          False),
            "pathScale":           lp.get("path_scale",              1.0),
            "minPathScale":        lp.get("min_path_scale",          0.75),
            "pathScaleStep":       lp.get("path_scale_step",         0.25),
            "pathScaleBySpeed":    lp.get("path_scale_by_speed",     True),
            "minPathRange":        lp.get("min_path_range",          1.0),
            "pathRangeStep":       lp.get("path_range_step",         0.5),
            "pathRangeBySpeed":    lp.get("path_range_by_speed",     True),
            "pathCropByGoal":      lp.get("path_crop_by_goal",       True),
            "freezeAng":           lp.get("freeze_ang",              90.0),
            "freezeTime":          lp.get("freeze_time",             2.0),
            "omniDirGoalThre":     lp.get("omni_dir_goal_thre",      1.0),
            "goalClearRange":      lp.get("goal_clear_range",        0.5),
            "goalBehindRange":     lp.get("goal_behind_range",       0.8),
            "autonomyMode":              True,
            "joyToSpeedDelay":           aut.get("joy_to_speed_delay",          2.0),
            "joyToCheckObstacleDelay":   aut.get("joy_to_check_obstacle_delay", 5.0),
        },
        remappings=adapter_remappings("local_planner"),
        env=DDS_ENV,
    ))


def path_follower(cfg: RobotConfig | None = None) -> NativeModule:
    """Pure Pursuit waypoint tracking + cmd_vel output (pathFollower C++ node)."""
    cfg = cfg or get_config()
    pf  = cfg.raw.get("path_follower", {})
    ctl = cfg.raw.get("control", {})
    aut = cfg.raw.get("autonomy", {})
    return NativeModule(NativeModuleConfig(
        executable=exe(cfg, "local_planner", "pathFollower"),
        name="path_follower",
        parameters={
            "maxSpeed":           cfg.speed.max_speed,
            "autonomySpeed":      cfg.speed.autonomy_speed,
            "yawRateGain":        ctl.get("yaw_rate_gain",       7.5),
            "stopYawRateGain":    ctl.get("stop_yaw_rate_gain",  7.5),
            "maxYawRate":         ctl.get("max_yaw_rate",        45.0),
            "maxAccel":           ctl.get("max_accel",           1.0),
            "switchTimeThre":     pf.get("switch_time_thre",      1.0),
            "dirDiffThre":        pf.get("dir_diff_thre",         0.1),
            "pubSkipNum":         pf.get("pub_skip_num",           1),
            "useInclRateToSlow":  pf.get("use_incl_rate_to_slow", False),
            "inclRateThre":       pf.get("incl_rate_thre",        120.0),
            "slowRate1":          pf.get("slow_rate_1",           0.25),
            "slowRate2":          pf.get("slow_rate_2",           0.5),
            "slowRate3":          pf.get("slow_rate_3",           0.75),
            "slowTime1":          pf.get("slow_time_1",           2.0),
            "slowTime2":          pf.get("slow_time_2",           2.0),
            "useInclToStop":      pf.get("use_incl_to_stop",      False),
            "inclThre":           pf.get("incl_thre",             45.0),
            "stopTime":           pf.get("stop_time",             5.0),
            "noRotAtStop":        pf.get("no_rot_at_stop",        False),
            "noRotAtGoal":        pf.get("no_rot_at_goal",        True),
            "autonomyMode":       True,
            "joyToSpeedDelay":    aut.get("joy_to_speed_delay",   2.0),
        },
        remappings=adapter_remappings("path_follower"),
        env=DDS_ENV,
    ))
