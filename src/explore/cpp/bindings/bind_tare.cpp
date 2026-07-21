#include "bindings.hpp"

#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "explore_contract.hpp"
#include "tare_policy.hpp"

#ifdef LINGTU_EXPLORE_HAS_DDS
#include "tare_dds.hpp"
#endif

namespace nb = nanobind;
using namespace lingtu::explore;

namespace lingtu_explore_kernel_bindings {

void bind_tare(nb::module_& m) {
  nb::class_<TarePolicyConfig>(m, "TarePolicyConfig")
    .def(nb::init<>())
    .def_rw("min_frontier_size", &TarePolicyConfig::min_frontier_size)
    .def_rw("sensor_range_m", &TarePolicyConfig::sensor_range_m)
    .def_rw("candidate_radius_m", &TarePolicyConfig::candidate_radius_m)
    .def_rw("min_goal_distance_m", &TarePolicyConfig::min_goal_distance_m)
    .def_rw("novelty_radius_m", &TarePolicyConfig::novelty_radius_m)
    .def_rw("max_candidates", &TarePolicyConfig::max_candidates)
    .def_rw("local_route_radius_m", &TarePolicyConfig::local_route_radius_m)
    .def_rw("coverage_resolution_m", &TarePolicyConfig::coverage_resolution_m)
    .def_rw("return_home_distance_m", &TarePolicyConfig::return_home_distance_m)
    .def_rw("keypose_min_distance_m", &TarePolicyConfig::keypose_min_distance_m)
    .def_rw("keypose_connect_distance_m", &TarePolicyConfig::keypose_connect_distance_m)
    .def_rw("gain_weight", &TarePolicyConfig::gain_weight)
    .def_rw("travel_weight", &TarePolicyConfig::travel_weight)
    .def_rw("momentum_weight", &TarePolicyConfig::momentum_weight)
    .def_rw("revisit_weight", &TarePolicyConfig::revisit_weight)
    .def_rw("max_plan_time_ms", &TarePolicyConfig::max_plan_time_ms)
    .def_rw("route_2opt_iterations", &TarePolicyConfig::route_2opt_iterations)
    .def_rw("max_grid_cells", &TarePolicyConfig::max_grid_cells)
    .def_rw("max_frontier_cells", &TarePolicyConfig::max_frontier_cells)
    .def_rw("max_frontier_clusters", &TarePolicyConfig::max_frontier_clusters)
    .def_rw("max_coverage_cells", &TarePolicyConfig::max_coverage_cells)
    .def_rw("max_keyposes", &TarePolicyConfig::max_keyposes)
    .def_rw("max_keypose_edges", &TarePolicyConfig::max_keypose_edges)
    .def_rw("max_keypose_neighbor_links", &TarePolicyConfig::max_keypose_neighbor_links)
    .def_rw("max_route_targets", &TarePolicyConfig::max_route_targets)
    .def_rw("return_home_when_done", &TarePolicyConfig::return_home_when_done);

  nb::class_<TarePolicy>(m, "TarePolicy")
    .def(nb::init<TarePolicyConfig>(), nb::arg("config") = TarePolicyConfig())
    .def("name", &TarePolicy::name)
    .def("plan",
         static_cast<ExploreDecision (TarePolicy::*)(const ExploreInput&)>(&TarePolicy::plan),
         nb::arg("input"))
    .def("select", &TarePolicy::select,
         nb::arg("grid"), nb::arg("robot"),
         nb::arg("visited_goals") = std::vector<Pose2D>())
    .def("reset", &TarePolicy::reset)
    .def("diagnostics", &TarePolicy::diagnostics);

  // ── DDS transport ─────────────────────────────────────────────────────
#ifdef LINGTU_EXPLORE_HAS_DDS
  // Expose the DDS value types so Python can inspect received data.
  nb::class_<DdsWayPoint>(m, "DdsWayPoint")
    .def(nb::init<>())
    .def_ro("x", &DdsWayPoint::x)
    .def_ro("y", &DdsWayPoint::y)
    .def_ro("z", &DdsWayPoint::z)
    .def_ro("frame_id", &DdsWayPoint::frame_id)
    .def_ro("valid", &DdsWayPoint::valid);

  nb::class_<DdsPathPoint>(m, "DdsPathPoint")
    .def(nb::init<>())
    .def_ro("x", &DdsPathPoint::x)
    .def_ro("y", &DdsPathPoint::y)
    .def_ro("z", &DdsPathPoint::z);

  nb::class_<DdsPath>(m, "DdsPath")
    .def(nb::init<>())
    .def_ro("frame_id", &DdsPath::frame_id)
    .def_ro("poses", &DdsPath::poses)
    .def_ro("valid", &DdsPath::valid);

  nb::class_<DdsRuntime>(m, "DdsRuntime")
    .def(nb::init<>())
    .def_ro("data", &DdsRuntime::data)
    .def_ro("valid", &DdsRuntime::valid);

  nb::class_<DdsFinish>(m, "DdsFinish")
    .def(nb::init<>())
    .def_ro("data", &DdsFinish::data)
    .def_ro("valid", &DdsFinish::valid);

  nb::class_<TareDdsTransport>(m, "TareDdsTransport")
    .def(nb::init<int>(), nb::arg("domain_id"))
    .def("spin_once", &TareDdsTransport::spin_once,
         "Non-blocking take from all DDS readers.")
    .def("publish_start", &TareDdsTransport::publish_start,
         nb::arg("start"),
         "Publish a start/stop signal to the TARE planner.")
    .def("cleanup", &TareDdsTransport::cleanup,
         "Release all DDS entities.")
    .def_prop_ro("last_way_point", &TareDdsTransport::last_way_point)
    .def_prop_ro("last_path", &TareDdsTransport::last_path)
    .def_prop_ro("last_runtime", &TareDdsTransport::last_runtime)
    .def_prop_ro("last_finish", &TareDdsTransport::last_finish)
    .def_prop_ro("domain_id", &TareDdsTransport::domain_id);

  // Module-level constant so Python can check availability.
  m.attr("HAS_DDS") = true;
#else
  // DDS not compiled: provide a stub class that always raises.
  m.attr("HAS_DDS") = false;

  m.def("TareDdsTransport", [](int /*domain_id*/) -> nb::object {
    throw std::runtime_error(
        "DDS not available: lingtu_explore_kernel was compiled without "
        "LINGTU_EXPLORE_CPP_WITH_DDS. Install CycloneDDS and rebuild.");
  }, nb::arg("domain_id"),
     "Stub: raises RuntimeError (DDS transport not compiled).");
#endif
}

}  // namespace lingtu_explore_kernel_bindings
