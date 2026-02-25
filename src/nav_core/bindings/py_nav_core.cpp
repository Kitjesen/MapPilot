/**
 * pybind11 bindings for nav_core
 *
 * 编译: cmake -B build && cmake --build build
 * 使用: import _nav_core
 */
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "nav_core/types.hpp"
#include "nav_core/path_follower_core.hpp"
#include "nav_core/pct_adapter_core.hpp"
#include "nav_core/local_planner_core.hpp"

namespace py = pybind11;
using namespace nav_core;

PYBIND11_MODULE(_nav_core, m) {
  m.doc() = "nav_core — 零 ROS2 依赖的导航核心算法 (C++ → Python)";

  // ── 基础类型 ──
  py::class_<Vec3>(m, "Vec3")
    .def(py::init<>())
    .def(py::init([](double x, double y, double z) {
      return Vec3{x, y, z};
    }), py::arg("x") = 0.0, py::arg("y") = 0.0, py::arg("z") = 0.0)
    .def_readwrite("x", &Vec3::x)
    .def_readwrite("y", &Vec3::y)
    .def_readwrite("z", &Vec3::z);

  py::class_<Pose>(m, "Pose")
    .def(py::init<>())
    .def_readwrite("position", &Pose::position)
    .def_readwrite("yaw", &Pose::yaw);

  py::class_<Twist>(m, "Twist")
    .def(py::init<>())
    .def_readwrite("vx", &Twist::vx)
    .def_readwrite("vy", &Twist::vy)
    .def_readwrite("wz", &Twist::wz);

  m.def("normalize_angle", &normalizeAngle);
  m.def("distance_2d", &distance2D);
  m.def("distance_3d", &distance3D);

  // ── PathFollower ──
  py::class_<PathFollowerParams>(m, "PathFollowerParams")
    .def(py::init<>())
    .def_readwrite("sensor_offset_x", &PathFollowerParams::sensorOffsetX)
    .def_readwrite("sensor_offset_y", &PathFollowerParams::sensorOffsetY)
    .def_readwrite("base_look_ahead_dis", &PathFollowerParams::baseLookAheadDis)
    .def_readwrite("look_ahead_ratio", &PathFollowerParams::lookAheadRatio)
    .def_readwrite("min_look_ahead_dis", &PathFollowerParams::minLookAheadDis)
    .def_readwrite("max_look_ahead_dis", &PathFollowerParams::maxLookAheadDis)
    .def_readwrite("yaw_rate_gain", &PathFollowerParams::yawRateGain)
    .def_readwrite("stop_yaw_rate_gain", &PathFollowerParams::stopYawRateGain)
    .def_readwrite("max_yaw_rate", &PathFollowerParams::maxYawRate)
    .def_readwrite("max_speed", &PathFollowerParams::maxSpeed)
    .def_readwrite("max_accel", &PathFollowerParams::maxAccel)
    .def_readwrite("switch_time_thre", &PathFollowerParams::switchTimeThre)
    .def_readwrite("dir_diff_thre", &PathFollowerParams::dirDiffThre)
    .def_readwrite("omni_dir_goal_thre", &PathFollowerParams::omniDirGoalThre)
    .def_readwrite("omni_dir_diff_thre", &PathFollowerParams::omniDirDiffThre)
    .def_readwrite("stop_dis_thre", &PathFollowerParams::stopDisThre)
    .def_readwrite("slow_dwn_dis_thre", &PathFollowerParams::slowDwnDisThre)
    .def_readwrite("two_way_drive", &PathFollowerParams::twoWayDrive)
    .def_readwrite("no_rot_at_goal", &PathFollowerParams::noRotAtGoal);

  py::class_<PathFollowerState>(m, "PathFollowerState")
    .def(py::init<>())
    .def_readwrite("vehicle_speed", &PathFollowerState::vehicleSpeed)
    .def_readwrite("path_point_id", &PathFollowerState::pathPointID)
    .def_readwrite("nav_fwd", &PathFollowerState::navFwd);

  py::class_<PathFollowerOutput>(m, "PathFollowerOutput")
    .def(py::init<>())
    .def_readwrite("cmd", &PathFollowerOutput::cmd)
    .def_readwrite("dir_diff", &PathFollowerOutput::dirDiff)
    .def_readwrite("end_dis", &PathFollowerOutput::endDis)
    .def_readwrite("can_accel", &PathFollowerOutput::canAccel);

  m.def("adaptive_look_ahead", &adaptiveLookAhead);
  m.def("compute_control", &computeControl,
    py::arg("vehicle_rel"), py::arg("vehicle_yaw_diff"),
    py::arg("path_points"), py::arg("joy_speed"),
    py::arg("current_time"), py::arg("slow_factor"),
    py::arg("safety_stop"), py::arg("params"), py::arg("state"));

  // ── PCT Adapter ──
  m.def("downsample_path", &downsamplePath);

  py::class_<WaypointTrackerParams>(m, "WaypointTrackerParams")
    .def(py::init<>())
    .def_readwrite("waypoint_distance", &WaypointTrackerParams::waypointDistance)
    .def_readwrite("arrival_threshold", &WaypointTrackerParams::arrivalThreshold)
    .def_readwrite("stuck_timeout_sec", &WaypointTrackerParams::stuckTimeoutSec)
    .def_readwrite("max_replan_count", &WaypointTrackerParams::maxReplanCount)
    .def_readwrite("replan_cooldown_sec", &WaypointTrackerParams::replanCooldownSec)
    .def_readwrite("search_window", &WaypointTrackerParams::searchWindow);

  py::enum_<WaypointEvent>(m, "WaypointEvent")
    .value("NONE", WaypointEvent::kNone)
    .value("WAYPOINT_REACHED", WaypointEvent::kWaypointReached)
    .value("GOAL_REACHED", WaypointEvent::kGoalReached)
    .value("PATH_RECEIVED", WaypointEvent::kPathReceived)
    .value("REPLANNING", WaypointEvent::kReplanning)
    .value("STUCK_FINAL", WaypointEvent::kStuckFinal);

  py::class_<WaypointResult>(m, "WaypointResult")
    .def(py::init<>())
    .def_readwrite("event", &WaypointResult::event)
    .def_readwrite("current_index", &WaypointResult::currentIndex)
    .def_readwrite("total_waypoints", &WaypointResult::totalWaypoints)
    .def_readwrite("target_point", &WaypointResult::targetPoint)
    .def_readwrite("has_target", &WaypointResult::hasTarget);

  py::class_<WaypointTracker>(m, "WaypointTracker")
    .def(py::init<const WaypointTrackerParams&>(),
         py::arg("params") = WaypointTrackerParams())
    .def("set_path", &WaypointTracker::setPath)
    .def("update", &WaypointTracker::update)
    .def("path", &WaypointTracker::path)
    .def("current_index", &WaypointTracker::currentIndex)
    .def("goal_reached", &WaypointTracker::goalReached)
    .def("replan_count", &WaypointTracker::replanCount)
    .def("goal_pose", &WaypointTracker::goalPose);

  // ── Local Planner ──
  py::class_<VoxelGridParams>(m, "VoxelGridParams")
    .def(py::init<>())
    .def_readwrite("grid_voxel_size", &VoxelGridParams::gridVoxelSize)
    .def_readwrite("grid_voxel_offset_x", &VoxelGridParams::gridVoxelOffsetX)
    .def_readwrite("grid_voxel_offset_y", &VoxelGridParams::gridVoxelOffsetY)
    .def_readwrite("search_radius", &VoxelGridParams::searchRadius)
    .def_readwrite("grid_voxel_num_x", &VoxelGridParams::gridVoxelNumX)
    .def_readwrite("grid_voxel_num_y", &VoxelGridParams::gridVoxelNumY);

  m.def("world_to_voxel", [](double x2, double y2, const VoxelGridParams& g) {
    int indX, indY;
    bool ok = worldToVoxel(x2, y2, g, indX, indY);
    return py::make_tuple(ok, indX, indY);
  });

  py::class_<PathScoreParams>(m, "PathScoreParams")
    .def(py::init<>())
    .def_readwrite("dir_weight", &PathScoreParams::dirWeight)
    .def_readwrite("slope_weight", &PathScoreParams::slopeWeight)
    .def_readwrite("omni_dir_goal_thre", &PathScoreParams::omniDirGoalThre);

  m.def("score_path", &scorePath);
  m.def("ang_diff_deg", &angDiffDeg);
  m.def("compute_rot_dir_w", &computeRotDirW);
  m.def("compute_group_dir_w", &computeGroupDirW);
}
