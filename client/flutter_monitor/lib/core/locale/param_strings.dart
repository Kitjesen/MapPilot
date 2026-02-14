/// Runtime‑config 参数的英文翻译表
///
/// key = ParamConstraint.fieldName / BoolParam.fieldName / ParamGroup.key
/// 中文名保留在 runtime_config.dart 的 displayName 字段里，
/// 英文名查本文件的 map，找不到时 fallback 回 displayName。
library;

// ═══════════════════════════════════════════════════════════════
//  Group label
// ═══════════════════════════════════════════════════════════════

const Map<String, String> groupLabelEn = {
  'common': 'Common',
  'safety': 'Safety',
  'navigation': 'Navigation',
  'path_follow': 'Path Following',
  'terrain': 'Terrain Analysis',
  'local_planner': 'Local Planner',
  'perception': 'Perception',
  'system': 'Hardware & System',
};

// ═══════════════════════════════════════════════════════════════
//  Numeric param display name
// ═══════════════════════════════════════════════════════════════

const Map<String, String> paramNameEn = {
  // ── Common (motion) ──
  'max_speed': 'Max Speed',
  'max_angular': 'Max Angular Speed',
  'autonomy_speed': 'Autonomy Speed',
  'cruise_speed': 'Cruise Speed',

  // ── Safety ──
  'stop_distance': 'E‑Stop Distance',
  'slow_distance': 'Slow Distance',
  'obstacle_height_thre': 'Obstacle Height Threshold',
  'ground_height_thre': 'Ground Height Threshold',
  'tilt_limit_deg': 'Tilt Limit',
  'deadman_timeout_ms': 'Deadman Timeout',
  'safety_margin': 'Safety Margin',
  'vehicle_width_margin': 'Width Margin',
  // geofence
  'geofence_warn_margin': 'Geofence Warn Distance',
  'geofence_stop_margin': 'Geofence Stop Distance',
  'geofence_check_hz': 'Geofence Check Rate',

  // ── Navigation ──
  'arrival_radius': 'Arrival Radius',
  'waypoint_radius': 'Waypoint Switch Radius',
  'path_tolerance': 'Path Tolerance',
  'waypoint_distance': 'Waypoint Spacing',
  'waypoint_timeout_sec': 'Waypoint Timeout',
  'patrol_wait_time': 'Patrol Wait Time',
  // global planner
  'arrival_threshold': 'Arrival Threshold',
  'lookahead_dist': 'Lookahead Distance',
  'tomogram_resolution': 'Tomogram Resolution',
  'tomogram_slice_dh': 'Tomogram Slice dH',
  'tomogram_ground_h': 'Ground Height',
  'max_heading_rate': 'Max Heading Rate',
  'min_plan_interval': 'Min Plan Interval',
  'default_goal_height': 'Default Goal Height',

  // ── Path Following ──
  'yaw_rate_gain': 'Yaw Rate Gain',
  'stop_yaw_rate_gain': 'Stop Yaw Rate Gain',
  'max_yaw_rate': 'Max Yaw Rate',
  'max_accel': 'Max Acceleration',
  'look_ahead_dis': 'Look Ahead Distance',
  'stop_dis_thre': 'Stop Distance Threshold',
  'slow_dwn_dis_thre': 'Slow Down Threshold',
  'base_look_ahead_dis': 'Base Look Ahead',
  'min_look_ahead_dis': 'Min Look Ahead',
  'max_look_ahead_dis': 'Max Look Ahead',
  'look_ahead_ratio': 'Look Ahead Ratio',
  'switch_time_thre': 'Direction Switch Delay',
  'dir_diff_thre': 'Direction Diff Threshold',
  'incl_rate_thre': 'Incline Rate Threshold',
  'slow_rate_1': 'Slow Rate 1',
  'slow_rate_2': 'Slow Rate 2',
  'slow_rate_3': 'Slow Rate 3',
  'slow_time_1': 'Slow Time 1',
  'slow_time_2': 'Slow Time 2',
  'incl_thre': 'Incline Stop Threshold',
  'stop_time': 'Stop Wait Time',
  'pub_skip_num': 'Publish Skip Count',

  // ── Terrain ──
  'terrain_voxel_size': 'Terrain Voxel Size',
  'scan_voxel_size': 'Scan Voxel Size',
  'decay_time': 'Decay Time',
  'no_decay_dis': 'No Decay Distance',
  'clearing_dis': 'Clearing Distance',
  'local_terrain_map_radius': 'Local Terrain Radius',
  'quantile_z': 'Height Quantile Z',
  'max_ground_lift': 'Max Ground Lift',
  'min_dy_obs_dis': 'Min Dynamic Obs Distance',
  'abs_dy_obs_rel_z_thre': 'Dynamic Obs Z Threshold',
  'min_dy_obs_vfov': 'Min Dynamic Obs VFOV',
  'max_dy_obs_vfov': 'Max Dynamic Obs VFOV',
  'min_dy_obs_point_num': 'Min Dynamic Obs Points',
  'no_data_block_skip_num': 'No Data Skip Count',
  'min_block_point_num': 'Min Block Points',
  'voxel_point_update_thre': 'Voxel Point Update Threshold',
  'voxel_time_update_thre': 'Voxel Time Update Threshold',
  'min_rel_z': 'Min Relative Z',
  'max_rel_z': 'Max Relative Z',
  'dis_ratio_z': 'Z Distance Ratio',
  'terrain_under_vehicle': 'Terrain Under Vehicle',
  'terrain_conn_thre': 'Terrain Connectivity Threshold',
  'ceiling_filtering_thre': 'Ceiling Filter Threshold',

  // ── Local Planner ──
  'path_scale': 'Path Scale',
  'min_path_scale': 'Min Path Scale',
  'path_scale_step': 'Scale Step',
  'adjacent_range': 'Adjacent Range',
  'cost_height_thre_1': 'Cost Height Threshold 1',
  'cost_height_thre_2': 'Cost Height Threshold 2',
  'slow_path_num_thre': 'Slow Path Count Threshold',
  'slow_group_num_thre': 'Slow Group Count Threshold',
  'point_per_path_thre': 'Min Points Per Path',
  'dir_weight': 'Direction Weight',
  'dir_thre': 'Direction Threshold',
  'min_path_range': 'Min Path Range',
  'path_range_step': 'Path Range Step',
  'joy_to_speed_delay': 'Joy‑to‑Speed Delay',
  'joy_to_check_obstacle_delay': 'Joy‑to‑Obstacle Delay',
  'freeze_ang': 'Freeze Angle',
  'freeze_time': 'Freeze Time',
  'goal_clear_range': 'Goal Clear Range',
  'goal_behind_range': 'Goal Behind Range',
  'laser_voxel_size': 'Local Laser Voxel',

  // ── Perception (SLAM + Localizer) ──
  'lidar_min_range': 'Lidar Min Range',
  'lidar_max_range': 'Lidar Max Range',
  'scan_resolution': 'Scan Resolution',
  'map_resolution': 'Map Resolution',
  'det_range': 'Detection Range',
  'move_thresh': 'Move Threshold',
  'stationary_thresh': 'Stationary Threshold',
  'localizer_update_hz': 'Localizer Update Rate',
  'icp_excellent_threshold': 'ICP Excellent Threshold',
  'icp_poor_threshold': 'ICP Poor Threshold',
  'tf_max_age_ms': 'TF Max Age',

  // ── Hardware & System ──
  'vehicle_height': 'Vehicle Height',
  'vehicle_width': 'Vehicle Width',
  'vehicle_length': 'Vehicle Length',
  'sensor_offset_x': 'Sensor Offset X',
  'sensor_offset_y': 'Sensor Offset Y',
  'cmd_vel_timeout_ms': 'Cmd Vel Timeout',
  'control_rate': 'Control Rate',
  'reconnect_interval': 'Reconnect Interval',
  'fast_state_hz': 'Fast State Rate',
  'slow_state_hz': 'Slow State Rate',
  'lidar_publish_freq': 'Lidar Publish Rate',
  'health_eval_hz': 'Health Eval Rate',
  'health_rate_window_sec': 'Rate Window',
};

// ═══════════════════════════════════════════════════════════════
//  Bool param display name
// ═══════════════════════════════════════════════════════════════

const Map<String, String> toggleNameEn = {
  // common / features
  'obstacle_avoidance_enabled': 'Obstacle Avoidance',
  'auto_speed_scale_enabled': 'Auto Speed Scale',
  'autonomy_mode': 'Autonomy Mode',
  // safety
  'geofence_enabled': 'Enable Geofence',
  // navigation
  'patrol_loop': 'Patrol Loop',
  'use_quintic': 'Quintic Spline',
  // path following
  'no_rot_at_goal': 'No Rotation at Goal',
  'no_rot_at_stop': 'No Rotation at Stop',
  'use_incl_rate_to_slow': 'Slow on Incline Rate',
  'use_incl_to_stop': 'Stop on Incline',
  // terrain
  'check_collision': 'Collision Detection',
  'use_sorting': 'Voxel Sorting',
  'consider_drop': 'Consider Drop',
  'limit_ground_lift': 'Limit Ground Lift',
  'clear_dy_obs': 'Clear Dynamic Obstacles',
  'no_data_obstacle': 'No Data as Obstacle',
  // local planner
  'check_obstacle': 'Obstacle Detection',
  'two_way_drive': 'Two‑Way Drive',
  'path_scale_by_speed': 'Scale Path by Speed',
  'check_rot_obstacle': 'Rotation Obstacle Check',
  'use_cost': 'Use Cost Function',
  'dir_to_vehicle': 'Direction to Vehicle',
  'path_range_by_speed': 'Path Range by Speed',
  'path_crop_by_goal': 'Crop Path by Goal',
  // system
  'auto_enable': 'Auto Enable',
  'auto_standup': 'Auto Standup',
};

// ═══════════════════════════════════════════════════════════════
//  Bool param subtitle
// ═══════════════════════════════════════════════════════════════

const Map<String, String> toggleSubtitleEn = {
  'obstacle_avoidance_enabled': 'Enable obstacle detection and avoidance',
  'auto_speed_scale_enabled': 'Auto slow down on poor localization',
  'autonomy_mode': 'Enable autonomous navigation mode',
  'geofence_enabled': 'Restrict robot to designated area',
  'patrol_loop': 'Loop back to start after last waypoint',
  'use_quintic': 'Smooth global path with quintic polynomial',
  'no_rot_at_goal': "Don't adjust heading at goal",
  'no_rot_at_stop': "Don't allow in‑place rotation when stopped",
  'use_incl_rate_to_slow': 'Auto slow on rapid slope change',
  'use_incl_to_stop': 'Auto stop when slope exceeds limit',
  'check_collision': 'Enable path collision detection',
  'use_sorting': 'Sort voxels by height for ground classification',
  'consider_drop': 'Detect sudden ground drop areas',
  'limit_ground_lift': 'Limit max ground elevation estimate',
  'clear_dy_obs': 'Enable dynamic obstacle detection and clearing',
  'no_data_obstacle': 'Mark lidar blind zones as impassable',
  'check_obstacle': 'Enable obstacle detection in local planner',
  'two_way_drive': 'Allow robot to drive in reverse',
  'path_scale_by_speed': 'Auto scale avoidance paths at high speed',
  'check_rot_obstacle': 'Detect obstacles during rotation',
  'use_cost': 'Select path based on cost map',
  'dir_to_vehicle': 'Compute direction in vehicle frame',
  'path_range_by_speed': 'Expand path search range at high speed',
  'path_crop_by_goal': 'Crop candidate paths by goal distance',
  'auto_enable': 'Auto enable chassis on connect',
  'auto_standup': 'Auto standup after enabling',
};

// ═══════════════════════════════════════════════════════════════
//  Helpers
// ═══════════════════════════════════════════════════════════════

/// 根据 isEn 返回参数组标签
String groupLabel(String key, String zhLabel, {required bool isEn}) =>
    isEn ? (groupLabelEn[key] ?? zhLabel) : zhLabel;

/// 根据 isEn 返回参数名
String paramName(String fieldName, String zhName, {required bool isEn}) =>
    isEn ? (paramNameEn[fieldName] ?? zhName) : zhName;

/// 根据 isEn 返回开关名
String toggleName(String fieldName, String zhName, {required bool isEn}) =>
    isEn ? (toggleNameEn[fieldName] ?? zhName) : zhName;

/// 根据 isEn 返回开关副标题
String? toggleSubtitle(String fieldName, String? zhSub, {required bool isEn}) =>
    isEn ? (toggleSubtitleEn[fieldName] ?? zhSub) : zhSub;
