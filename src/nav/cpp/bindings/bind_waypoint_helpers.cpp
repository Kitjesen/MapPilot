#include "bindings.hpp"

#include <nanobind/stl/vector.h>

#include "nav_kernel/waypoint_helpers_core.hpp"

namespace nb = nanobind;
using namespace nav_kernel;

namespace lingtu_nav_kernel_bindings {

void bind_waypoint_helpers(nb::module_& m) {
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
}

}  // namespace lingtu_nav_kernel_bindings
