#include "bindings.hpp"

#include <new>

#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "explore_contract.hpp"

namespace nb = nanobind;
using namespace lingtu::explore;

namespace lingtu_explore_kernel_bindings {

void bind_types(nb::module_& m) {
  m.attr("FREE") = static_cast<int>(kFree);
  m.attr("OCCUPIED") = static_cast<int>(kOccupied);
  m.attr("UNKNOWN") = static_cast<int>(kUnknown);

  nb::class_<Grid2D>(m, "Grid2D")
    .def(nb::init<>())
    .def_rw("width", &Grid2D::width)
    .def_rw("height", &Grid2D::height)
    .def_rw("resolution", &Grid2D::resolution)
    .def_rw("origin_x", &Grid2D::origin_x)
    .def_rw("origin_y", &Grid2D::origin_y)
    .def_rw("cells", &Grid2D::cells)
    .def("valid", &Grid2D::valid);

  nb::class_<Pose2D>(m, "Pose2D")
    .def(nb::init<>())
    .def("__init__", [](Pose2D* p, double x, double y, double yaw) {
      new (p) Pose2D{x, y, yaw};
    }, nb::arg("x") = 0.0, nb::arg("y") = 0.0, nb::arg("yaw") = 0.0)
    .def_rw("x", &Pose2D::x)
    .def_rw("y", &Pose2D::y)
    .def_rw("yaw", &Pose2D::yaw);

  nb::class_<ExploreMapIdentity>(m, "ExploreMapIdentity")
    .def(nb::init<>())
    .def_rw("frame_id", &ExploreMapIdentity::frame_id)
    .def_rw("session_id", &ExploreMapIdentity::session_id)
    .def_rw("map_id", &ExploreMapIdentity::map_id)
    .def_rw("map_version", &ExploreMapIdentity::map_version)
    .def_rw("artifact_hash", &ExploreMapIdentity::artifact_hash)
    .def_rw("reset_epoch", &ExploreMapIdentity::reset_epoch)
    .def_rw("generation", &ExploreMapIdentity::generation)
    .def_rw("live", &ExploreMapIdentity::live)
    .def("valid", &ExploreMapIdentity::valid);

  nb::class_<ExploreInput>(m, "ExploreInput")
    .def(nb::init<>())
    .def_rw("exploration_grid", &ExploreInput::exploration_grid)
    .def_rw("robot_pose", &ExploreInput::robot_pose)
    .def_rw("visited_goals", &ExploreInput::visited_goals)
    .def_rw("stamp_s", &ExploreInput::stamp_s)
    .def_rw("map_frame", &ExploreInput::map_frame)
    .def_rw("map", &ExploreInput::map);

  nb::class_<ExploreCandidate>(m, "ExploreCandidate")
    .def(nb::init<>())
    .def_rw("x", &ExploreCandidate::x)
    .def_rw("y", &ExploreCandidate::y)
    .def_rw("z", &ExploreCandidate::z)
    .def_rw("score", &ExploreCandidate::score)
    .def_rw("distance_m", &ExploreCandidate::distance_m)
    .def_rw("frontier_size", &ExploreCandidate::frontier_size)
    .def_rw("covered_frontier_cells", &ExploreCandidate::covered_frontier_cells)
    .def_rw("route_cost_m", &ExploreCandidate::route_cost_m)
    .def_rw("revisit_penalty", &ExploreCandidate::revisit_penalty)
    .def_rw("cluster_id", &ExploreCandidate::cluster_id);

  nb::class_<ExploreDiagnostics>(m, "ExploreDiagnostics")
    .def(nb::init<>())
    .def_rw("phase", &ExploreDiagnostics::phase)
    .def_rw("frame_id", &ExploreDiagnostics::frame_id)
    .def_rw("session_id", &ExploreDiagnostics::session_id)
    .def_rw("map_id", &ExploreDiagnostics::map_id)
    .def_rw("map_version", &ExploreDiagnostics::map_version)
    .def_rw("reset_epoch", &ExploreDiagnostics::reset_epoch)
    .def_rw("generation", &ExploreDiagnostics::generation)
    .def_rw("accepted_generation", &ExploreDiagnostics::accepted_generation)
    .def_rw("reachable_free_cells", &ExploreDiagnostics::reachable_free_cells)
    .def_rw("frontier_cells", &ExploreDiagnostics::frontier_cells)
    .def_rw("frontier_clusters", &ExploreDiagnostics::frontier_clusters)
    .def_rw("keypose_nodes", &ExploreDiagnostics::keypose_nodes)
    .def_rw("keypose_edges", &ExploreDiagnostics::keypose_edges)
    .def_rw("covered_cells", &ExploreDiagnostics::covered_cells)
    .def_rw("route_targets", &ExploreDiagnostics::route_targets)
    .def_rw("reset_count", &ExploreDiagnostics::reset_count)
    .def_rw("route_length_m", &ExploreDiagnostics::route_length_m)
    .def_rw("planning_time_ms", &ExploreDiagnostics::planning_time_ms)
    .def_rw("state_committed", &ExploreDiagnostics::state_committed);

  nb::class_<ExploreDecision>(m, "ExploreDecision")
    .def(nb::init<>())
    .def_rw("has_goal", &ExploreDecision::has_goal)
    .def_rw("done", &ExploreDecision::done)
    .def_rw("goal_x", &ExploreDecision::goal_x)
    .def_rw("goal_y", &ExploreDecision::goal_y)
    .def_rw("goal_z", &ExploreDecision::goal_z)
    .def_rw("reason", &ExploreDecision::reason)
    .def_rw("candidates", &ExploreDecision::candidates)
    .def_rw("route", &ExploreDecision::route)
    .def_rw("diagnostics", &ExploreDecision::diagnostics);
}

}  // namespace lingtu_explore_kernel_bindings
