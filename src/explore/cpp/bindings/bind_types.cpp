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

  nb::class_<ExploreInput>(m, "ExploreInput")
    .def(nb::init<>())
    .def_rw("exploration_grid", &ExploreInput::exploration_grid)
    .def_rw("robot_pose", &ExploreInput::robot_pose)
    .def_rw("visited_goals", &ExploreInput::visited_goals)
    .def_rw("stamp_s", &ExploreInput::stamp_s)
    .def_rw("map_frame", &ExploreInput::map_frame);

  nb::class_<ExploreCandidate>(m, "ExploreCandidate")
    .def(nb::init<>())
    .def_rw("x", &ExploreCandidate::x)
    .def_rw("y", &ExploreCandidate::y)
    .def_rw("z", &ExploreCandidate::z)
    .def_rw("score", &ExploreCandidate::score)
    .def_rw("distance_m", &ExploreCandidate::distance_m)
    .def_rw("frontier_size", &ExploreCandidate::frontier_size)
    .def_rw("covered_frontier_cells", &ExploreCandidate::covered_frontier_cells);

  nb::class_<ExploreDecision>(m, "ExploreDecision")
    .def(nb::init<>())
    .def_rw("has_goal", &ExploreDecision::has_goal)
    .def_rw("done", &ExploreDecision::done)
    .def_rw("goal_x", &ExploreDecision::goal_x)
    .def_rw("goal_y", &ExploreDecision::goal_y)
    .def_rw("goal_z", &ExploreDecision::goal_z)
    .def_rw("reason", &ExploreDecision::reason)
    .def_rw("candidates", &ExploreDecision::candidates);
}

}  // namespace lingtu_explore_kernel_bindings
