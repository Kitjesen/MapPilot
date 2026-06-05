/**
 * nanobind bindings for nav_core
 *
 * Migrated from pybind11 → nanobind for:
 *   - 2-4x faster compilation
 *   - 3-5x smaller .so
 *   - Better ndarray zero-copy with numpy
 *   - Free-threaded Python support (future)
 *
 * Build: cmake -B build && cmake --build build
 * Usage: import _nav_core
 */
#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>

#include "nav_core/types.hpp"
#include "nav_core/path_follower_core.hpp"
#include "nav_core/pct_adapter_core.hpp"
#include "nav_core/local_planner_core.hpp"
#include "nav_core/local_planner_full.hpp"
#include "nav_core/terrain_core.hpp"

namespace nb = nanobind;
using namespace nav_core;

NB_MODULE(_nav_core, m) {
  m.doc() = "nav_core -- ROS2-free navigation core algorithms (C++ to Python, nanobind)";

  // ── Core types ──
  nb::class_<Vec3>(m, "Vec3")
    .def(nb::init<>())
    .def("__init__", [](Vec3* v, double x, double y, double z) {
      new (v) Vec3{x, y, z};
    }, nb::arg("x") = 0.0, nb::arg("y") = 0.0, nb::arg("z") = 0.0)
    .def_rw("x", &Vec3::x)
    .def_rw("y", &Vec3::y)
    .def_rw("z", &Vec3::z);

  nb::class_<Pose>(m, "Pose")
    .def(nb::init<>())
    .def_rw("position", &Pose::position)
    .def_rw("yaw", &Pose::yaw);

  nb::class_<Twist>(m, "Twist")
    .def(nb::init<>())
    .def_rw("vx", &Twist::vx)
    .def_rw("vy", &Twist::vy)
    .def_rw("wz", &Twist::wz);

  m.def("normalize_angle", &normalizeAngle);
  m.def("distance_2d", &distance2D);
  m.def("distance_3d", &distance3D);

  // ── PathFollower ──
  nb::class_<PathFollowerParams>(m, "PathFollowerParams")
    .def(nb::init<>())
    .def_rw("sensor_offset_x", &PathFollowerParams::sensorOffsetX)
    .def_rw("sensor_offset_y", &PathFollowerParams::sensorOffsetY)
    .def_rw("base_look_ahead_dis", &PathFollowerParams::baseLookAheadDis)
    .def_rw("look_ahead_ratio", &PathFollowerParams::lookAheadRatio)
    .def_rw("min_look_ahead_dis", &PathFollowerParams::minLookAheadDis)
    .def_rw("max_look_ahead_dis", &PathFollowerParams::maxLookAheadDis)
    .def_rw("yaw_rate_gain", &PathFollowerParams::yawRateGain)
    .def_rw("stop_yaw_rate_gain", &PathFollowerParams::stopYawRateGain)
    .def_rw("max_yaw_rate", &PathFollowerParams::maxYawRate)
    .def_rw("max_speed", &PathFollowerParams::maxSpeed)
    .def_rw("max_accel", &PathFollowerParams::maxAccel)
    .def_rw("turn_speed_yaw_rate_start", &PathFollowerParams::turnSpeedYawRateStart)
    .def_rw("turn_speed_min_scale", &PathFollowerParams::turnSpeedMinScale)
    .def_rw("switch_time_thre", &PathFollowerParams::switchTimeThre)
    .def_rw("dir_diff_thre", &PathFollowerParams::dirDiffThre)
    .def_rw("omni_dir_goal_thre", &PathFollowerParams::omniDirGoalThre)
    .def_rw("omni_dir_diff_thre", &PathFollowerParams::omniDirDiffThre)
    .def_rw("stop_dis_thre", &PathFollowerParams::stopDisThre)
    .def_rw("slow_dwn_dis_thre", &PathFollowerParams::slowDwnDisThre)
    .def_rw("two_way_drive", &PathFollowerParams::twoWayDrive)
    .def_rw("no_rot_at_goal", &PathFollowerParams::noRotAtGoal);

  nb::class_<PathFollowerState>(m, "PathFollowerState")
    .def(nb::init<>())
    .def_rw("vehicle_speed", &PathFollowerState::vehicleSpeed)
    .def_rw("path_point_id", &PathFollowerState::pathPointID)
    .def_rw("nav_fwd", &PathFollowerState::navFwd);

  nb::class_<PathFollowerOutput>(m, "PathFollowerOutput")
    .def(nb::init<>())
    .def_rw("cmd", &PathFollowerOutput::cmd)
    .def_rw("dir_diff", &PathFollowerOutput::dirDiff)
    .def_rw("end_dis", &PathFollowerOutput::endDis)
    .def_rw("turn_speed_scale", &PathFollowerOutput::turnSpeedScale)
    .def_rw("can_accel", &PathFollowerOutput::canAccel);

  m.def("adaptive_look_ahead", &adaptiveLookAhead);
  m.def("compute_control", &computeControl,
    nb::arg("vehicle_rel"), nb::arg("vehicle_yaw_diff"),
    nb::arg("path_points"), nb::arg("joy_speed"),
    nb::arg("current_time"), nb::arg("slow_factor"),
    nb::arg("safety_stop"), nb::arg("params"), nb::arg("state"));

  // ── PCT Adapter ──
  m.def("downsample_path", &downsamplePath);

  nb::class_<WaypointTrackerParams>(m, "WaypointTrackerParams")
    .def(nb::init<>())
    .def_rw("waypoint_distance", &WaypointTrackerParams::waypointDistance)
    .def_rw("arrival_threshold", &WaypointTrackerParams::arrivalThreshold)
    .def_rw("stuck_timeout_sec", &WaypointTrackerParams::stuckTimeoutSec)
    .def_rw("max_replan_count", &WaypointTrackerParams::maxReplanCount)
    .def_rw("replan_cooldown_sec", &WaypointTrackerParams::replanCooldownSec)
    .def_rw("search_window", &WaypointTrackerParams::searchWindow);

  nb::enum_<WaypointEvent>(m, "WaypointEvent")
    .value("NONE", WaypointEvent::kNone)
    .value("WAYPOINT_REACHED", WaypointEvent::kWaypointReached)
    .value("GOAL_REACHED", WaypointEvent::kGoalReached)
    .value("PATH_RECEIVED", WaypointEvent::kPathReceived)
    .value("REPLANNING", WaypointEvent::kReplanning)
    .value("STUCK_FINAL", WaypointEvent::kStuckFinal);

  nb::class_<WaypointResult>(m, "WaypointResult")
    .def(nb::init<>())
    .def_rw("event", &WaypointResult::event)
    .def_rw("current_index", &WaypointResult::currentIndex)
    .def_rw("total_waypoints", &WaypointResult::totalWaypoints)
    .def_rw("target_point", &WaypointResult::targetPoint)
    .def_rw("has_target", &WaypointResult::hasTarget);

  nb::class_<WaypointTracker>(m, "WaypointTracker")
    .def(nb::init<const WaypointTrackerParams&>(),
         nb::arg("params") = WaypointTrackerParams())
    .def("set_path", &WaypointTracker::setPath)
    .def("update", &WaypointTracker::update)
    .def("path", &WaypointTracker::path)
    .def("current_index", &WaypointTracker::currentIndex)
    .def("goal_reached", &WaypointTracker::goalReached)
    .def("replan_count", &WaypointTracker::replanCount)
    .def("goal_pose", &WaypointTracker::goalPose);

  // ── Local Planner ──
  nb::class_<VoxelGridParams>(m, "VoxelGridParams")
    .def(nb::init<>())
    .def_rw("grid_voxel_size", &VoxelGridParams::gridVoxelSize)
    .def_rw("grid_voxel_offset_x", &VoxelGridParams::gridVoxelOffsetX)
    .def_rw("grid_voxel_offset_y", &VoxelGridParams::gridVoxelOffsetY)
    .def_rw("search_radius", &VoxelGridParams::searchRadius)
    .def_rw("grid_voxel_num_x", &VoxelGridParams::gridVoxelNumX)
    .def_rw("grid_voxel_num_y", &VoxelGridParams::gridVoxelNumY);

  m.def("world_to_voxel", [](double x2, double y2, const VoxelGridParams& g) {
    int indX, indY;
    bool ok = worldToVoxel(x2, y2, g, indX, indY);
    return nb::make_tuple(ok, indX, indY);
  });

  nb::class_<PathScoreParams>(m, "PathScoreParams")
    .def(nb::init<>())
    .def_rw("dir_weight", &PathScoreParams::dirWeight)
    .def_rw("slope_weight", &PathScoreParams::slopeWeight)
    .def_rw("omni_dir_goal_thre", &PathScoreParams::omniDirGoalThre);

  m.def("score_path", &scorePath);
  m.def("ang_diff_deg", &angDiffDeg);
  m.def("compute_rot_dir_w", &computeRotDirW);
  m.def("compute_group_dir_w", &computeGroupDirW);

  // ── LocalPlannerCore (full scoring loop, zero ROS2) ──
  nb::class_<LocalPlannerParams>(m, "LocalPlannerParams")
    .def(nb::init<>())
    .def_rw("vehicle_length", &LocalPlannerParams::vehicleLength)
    .def_rw("vehicle_width", &LocalPlannerParams::vehicleWidth)
    .def_rw("sensor_offset_x", &LocalPlannerParams::sensorOffsetX)
    .def_rw("sensor_offset_y", &LocalPlannerParams::sensorOffsetY)
    .def_rw("two_way_drive", &LocalPlannerParams::twoWayDrive)
    .def_rw("adjacent_range", &LocalPlannerParams::adjacentRange)
    .def_rw("obstacle_height_thre", &LocalPlannerParams::obstacleHeightThre)
    .def_rw("ground_height_thre", &LocalPlannerParams::groundHeightThre)
    .def_rw("cost_height_thre_1", &LocalPlannerParams::costHeightThre1)
    .def_rw("cost_height_thre_2", &LocalPlannerParams::costHeightThre2)
    .def_rw("check_obstacle", &LocalPlannerParams::checkObstacle)
    .def_rw("check_rot_obstacle", &LocalPlannerParams::checkRotObstacle)
    .def_rw("use_terrain_analysis", &LocalPlannerParams::useTerrainAnalysis)
    .def_rw("point_per_path_thre", &LocalPlannerParams::pointPerPathThre)
    .def_rw("min_rel_z", &LocalPlannerParams::minRelZ)
    .def_rw("max_rel_z", &LocalPlannerParams::maxRelZ)
    .def_rw("dir_weight", &LocalPlannerParams::dirWeight)
    .def_rw("dir_thre", &LocalPlannerParams::dirThre)
    .def_rw("dir_to_vehicle", &LocalPlannerParams::dirToVehicle)
    .def_rw("path_scale", &LocalPlannerParams::pathScale)
    .def_rw("min_path_scale", &LocalPlannerParams::minPathScale)
    .def_rw("path_scale_step", &LocalPlannerParams::pathScaleStep)
    .def_rw("path_scale_by_speed", &LocalPlannerParams::pathScaleBySpeed)
    .def_rw("min_path_range", &LocalPlannerParams::minPathRange)
    .def_rw("path_range_step", &LocalPlannerParams::pathRangeStep)
    .def_rw("path_range_by_speed", &LocalPlannerParams::pathRangeBySpeed)
    .def_rw("path_crop_by_goal", &LocalPlannerParams::pathCropByGoal)
    .def_rw("max_speed", &LocalPlannerParams::maxSpeed)
    .def_rw("autonomy_speed", &LocalPlannerParams::autonomySpeed)
    .def_rw("slope_weight", &LocalPlannerParams::slopeWeight)
    .def_rw("goal_clear_range", &LocalPlannerParams::goalClearRange)
    .def_rw("goal_behind_range", &LocalPlannerParams::goalBehindRange)
    .def_rw("near_field_stop_dis", &LocalPlannerParams::nearFieldStopDis)
    .def_rw("freeze_ang", &LocalPlannerParams::freezeAng)
    .def_rw("freeze_time", &LocalPlannerParams::freezeTime)
    .def_rw("omni_dir_goal_thre", &LocalPlannerParams::omniDirGoalThre)
    .def_rw("slow_path_num_thre", &LocalPlannerParams::slowPathNumThre)
    .def_rw("slow_group_num_thre", &LocalPlannerParams::slowGroupNumThre)
    .def_rw("recovery_blocked_thre", &LocalPlannerParams::recoveryBlockedThre)
    .def_rw("recovery_rotate_time", &LocalPlannerParams::recoveryRotateTime)
    .def_rw("recovery_backup_time", &LocalPlannerParams::recoveryBackupTime)
    .def_rw("recovery_max_cycles", &LocalPlannerParams::recoveryMaxCycles);

  nb::class_<LocalPlanResult>(m, "LocalPlanResult")
    .def(nb::init<>())
    .def_rw("path", &LocalPlanResult::path)
    .def_rw("slow_down", &LocalPlanResult::slowDown)
    .def_rw("path_found", &LocalPlanResult::pathFound)
    .def_rw("near_field_stop", &LocalPlanResult::nearFieldStop)
    .def_rw("recovery_state", &LocalPlanResult::recoveryState);

  nb::class_<LocalPlannerCore>(m, "LocalPlannerCore")
    .def(nb::init<const LocalPlannerParams&>(),
         nb::arg("params") = LocalPlannerParams())
    .def("load_paths", &LocalPlannerCore::loadPaths, nb::arg("paths_dir"))
    .def("paths_loaded", &LocalPlannerCore::pathsLoaded)
    .def("set_vehicle", &LocalPlannerCore::setVehicle,
         nb::arg("x"), nb::arg("y"), nb::arg("z"), nb::arg("yaw"))
    .def("set_goal", &LocalPlannerCore::setGoal,
         nb::arg("gx"), nb::arg("gy"))
    .def("plan", [](LocalPlannerCore& self,
                     const std::vector<float>& obstacle_flat,
                     double timestamp) {
      int n = (int)obstacle_flat.size() / 4;
      return self.plan(obstacle_flat.data(), n, timestamp);
    }, nb::arg("obstacle_xyzi"), nb::arg("timestamp"));

  // ── Terrain Analysis ──
  nb::class_<TerrainParams>(m, "TerrainParams")
    .def(nb::init<>())
    .def_rw("scan_voxel_size", &TerrainParams::scanVoxelSize)
    .def_rw("terrain_voxel_size", &TerrainParams::terrainVoxelSize)
    .def_rw("terrain_voxel_half_width", &TerrainParams::terrainVoxelHalfWidth)
    .def_rw("decay_time", &TerrainParams::decayTime)
    .def_rw("no_decay_dis", &TerrainParams::noDecayDis)
    .def_rw("clearing_dis", &TerrainParams::clearingDis)
    .def_rw("use_sorting", &TerrainParams::useSorting)
    .def_rw("quantile_z", &TerrainParams::quantileZ)
    .def_rw("consider_drop", &TerrainParams::considerDrop)
    .def_rw("limit_ground_lift", &TerrainParams::limitGroundLift)
    .def_rw("max_ground_lift", &TerrainParams::maxGroundLift)
    .def_rw("obstacle_height_thre", &TerrainParams::obstacleHeightThre)
    .def_rw("no_data_obstacle", &TerrainParams::noDataObstacle)
    .def_rw("min_block_point_num", &TerrainParams::minBlockPointNum)
    .def_rw("vehicle_height", &TerrainParams::vehicleHeight)
    .def_rw("min_rel_z", &TerrainParams::minRelZ)
    .def_rw("max_rel_z", &TerrainParams::maxRelZ)
    .def_rw("dis_ratio_z", &TerrainParams::disRatioZ)
    .def_rw("planar_voxel_size", &TerrainParams::planarVoxelSize)
    .def_rw("planar_voxel_half_width", &TerrainParams::planarVoxelHalfWidth);

  nb::class_<TerrainResult>(m, "TerrainResult")
    .def(nb::init<>())
    .def_rw("terrain_points", &TerrainResult::terrain_points)
    .def_rw("n_points", &TerrainResult::n_points)
    .def_rw("elevation_map", &TerrainResult::elevation_map)
    .def_rw("map_width", &TerrainResult::map_width)
    .def_rw("map_origin_x", &TerrainResult::map_origin_x)
    .def_rw("map_origin_y", &TerrainResult::map_origin_y)
    .def_rw("map_resolution", &TerrainResult::map_resolution);

  nb::class_<TerrainAnalysisCore>(m, "TerrainAnalysisCore")
    .def(nb::init<const TerrainParams&>(),
         nb::arg("params") = TerrainParams())
    .def("update_vehicle", &TerrainAnalysisCore::updateVehicle,
         nb::arg("x"), nb::arg("y"), nb::arg("z"),
         nb::arg("roll"), nb::arg("pitch"), nb::arg("yaw"))
    .def("process", [](TerrainAnalysisCore& self,
                        const std::vector<float>& cloud_flat,
                        double timestamp) {
      // cloud_flat: [x0,y0,z0,i0, x1,y1,z1,i1, ...] — Nx4 flat
      int n = (int)cloud_flat.size() / 4;
      return self.process(cloud_flat.data(), n, timestamp);
    }, nb::arg("cloud_xyzi"), nb::arg("timestamp"))
    .def("clear", &TerrainAnalysisCore::clear);
}
