#include "bindings.hpp"

#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "local_planner.hpp"
#include "local_planner_scoring.hpp"

namespace nb = nanobind;
using namespace nav_kernel;

namespace lingtu_nav_kernel_bindings {

void bind_local_planner(nb::module_& m) {
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
    .def_rw("use_cost", &LocalPlannerParams::useCost)
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
    .def_rw("footprint_padding", &LocalPlannerParams::footprintPadding)
    .def_rw("freeze_ang", &LocalPlannerParams::freezeAng)
    .def_rw("freeze_time", &LocalPlannerParams::freezeTime)
    .def_rw("omni_dir_goal_thre", &LocalPlannerParams::omniDirGoalThre)
    .def_rw("slow_path_num_thre", &LocalPlannerParams::slowPathNumThre)
    .def_rw("slow_group_num_thre", &LocalPlannerParams::slowGroupNumThre)
    .def_rw("use_traversability_cost", &LocalPlannerParams::useTraversabilityCost)
    .def_rw("traversability_near_field_stop", &LocalPlannerParams::traversabilityNearFieldStop)
    .def_rw("traversability_hard_cost", &LocalPlannerParams::traversabilityHardCost)
    .def_rw("traversability_soft_cost", &LocalPlannerParams::traversabilitySoftCost)
    .def_rw("traversability_weight", &LocalPlannerParams::traversabilityWeight)
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
    .def_rw("recovery_state", &LocalPlanResult::recoveryState)
    .def_rw("recovery_exhausted", &LocalPlanResult::recoveryExhausted)
    .def_rw("recovery_active", &LocalPlanResult::recoveryActive)
    .def_rw("recovery_verified", &LocalPlanResult::recoveryVerified)
    .def_rw(
        "recovery_observation_refresh_required",
        &LocalPlanResult::recoveryObservationRefreshRequired)
    .def_rw("recovery_direct_command", &LocalPlanResult::recoveryDirectCommand)
    .def_prop_ro("recovery_action", [](const LocalPlanResult& result) {
      return static_cast<int>(result.recoveryAction);
    })
    .def_rw("recovery_reason", &LocalPlanResult::recoveryReason)
    .def_rw("recovery_progress", &LocalPlanResult::recoveryProgress)
    .def_rw("recovery_attempt", &LocalPlanResult::recoveryAttempt)
    .def_rw("recovery_candidate_count", &LocalPlanResult::recoveryCandidateCount)
    .def_rw("recovery_rotation_direction", &LocalPlanResult::recoveryRotationDirection);

  nb::class_<LocalPlannerCore>(m, "LocalPlanner")
    .def(nb::init<const LocalPlannerParams&>(),
         nb::arg("params") = LocalPlannerParams())
    .def("load_paths", &LocalPlannerCore::loadPaths, nb::arg("paths_dir"))
    .def("paths_loaded", &LocalPlannerCore::pathsLoaded)
    .def("set_vehicle", &LocalPlannerCore::setVehicle,
         nb::arg("x"), nb::arg("y"), nb::arg("z"), nb::arg("yaw"))
    .def("set_goal", &LocalPlannerCore::setGoal,
         nb::arg("gx"), nb::arg("gy"))
    .def("set_traversability_grid", [](LocalPlannerCore& self,
                                        nb::ndarray<const float,
                                                    nb::ndim<2>,
                                                    nb::c_contig,
                                                    nb::device::cpu> grid,
                                        double resolution,
                                        double origin_x,
                                        double origin_y) {
      self.setTraversabilityGrid(
          grid.size() == 0 ? nullptr : grid.data(),
          static_cast<int>(grid.shape(0)),
          static_cast<int>(grid.shape(1)),
          resolution,
          origin_x,
          origin_y);
    }, nb::arg("grid"), nb::arg("resolution"), nb::arg("origin_x"), nb::arg("origin_y"))
    .def("clear_traversability_grid", &LocalPlannerCore::clearTraversabilityGrid)
    .def("plan", [](LocalPlannerCore& self,
                     nb::ndarray<const float,
                                 nb::ndim<1>,
                                 nb::c_contig,
                                 nb::device::cpu> obstacle_flat,
                     double timestamp) {
      int n = static_cast<int>(obstacle_flat.size()) / 4;
      return self.plan(obstacle_flat.data(), n, timestamp);
    }, nb::arg("obstacle_xyzi"), nb::arg("timestamp"))
    .def("plan_frame", [](LocalPlannerCore& self,
                           double x,
                           double y,
                           double z,
                           double yaw,
                           double gx,
                           double gy,
                           nb::ndarray<const float,
                                       nb::ndim<2>,
                                       nb::c_contig,
                                       nb::device::cpu> grid,
                           double resolution,
                           double origin_x,
                           double origin_y,
                           nb::ndarray<const float,
                                       nb::ndim<1>,
                                       nb::c_contig,
                                       nb::device::cpu> obstacle_flat,
                           double timestamp) {
      int n = static_cast<int>(obstacle_flat.size()) / 4;
      return self.planFrame(
          x,
          y,
          z,
          yaw,
          gx,
          gy,
          grid.size() == 0 ? nullptr : grid.data(),
          static_cast<int>(grid.shape(0)),
          static_cast<int>(grid.shape(1)),
          resolution,
          origin_x,
          origin_y,
          obstacle_flat.data(),
          n,
          timestamp);
    }, nb::arg("x"), nb::arg("y"), nb::arg("z"), nb::arg("yaw"),
       nb::arg("gx"), nb::arg("gy"), nb::arg("traversability_grid"),
       nb::arg("resolution"), nb::arg("origin_x"), nb::arg("origin_y"),
       nb::arg("obstacle_xyzi"), nb::arg("timestamp"))
    .def("plan_frame_without_grid", [](LocalPlannerCore& self,
                                        double x,
                                        double y,
                                        double z,
                                        double yaw,
                                        double gx,
                                        double gy,
                                        nb::ndarray<const float,
                                                    nb::ndim<1>,
                                                    nb::c_contig,
                                                    nb::device::cpu> obstacle_flat,
                                        double timestamp) {
      int n = static_cast<int>(obstacle_flat.size()) / 4;
      return self.planFrame(
          x,
          y,
          z,
          yaw,
          gx,
          gy,
          nullptr,
          0,
          0,
          0.0,
          0.0,
          0.0,
          obstacle_flat.data(),
          n,
          timestamp);
    }, nb::arg("x"), nb::arg("y"), nb::arg("z"), nb::arg("yaw"),
       nb::arg("gx"), nb::arg("gy"), nb::arg("obstacle_xyzi"),
       nb::arg("timestamp"));
}

}  // namespace lingtu_nav_kernel_bindings
