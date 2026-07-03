#include "bindings.hpp"

#include <nanobind/stl/vector.h>

#include "nav_kernel/path_follower_core.hpp"

namespace nb = nanobind;
using namespace nav_kernel;

namespace lingtu_nav_kernel_bindings {

void bind_path_follower(nb::module_& m) {
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
    .def_rw("last_path_point_id", &PathFollowerState::lastPathPointID)
    .def_rw("last_path_size", &PathFollowerState::lastPathSize)
    .def_rw("nav_fwd", &PathFollowerState::navFwd)
    .def_rw("switch_time", &PathFollowerState::switchTime);

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
}

}  // namespace lingtu_nav_kernel_bindings
