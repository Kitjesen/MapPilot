#include "bindings.hpp"

#include <nanobind/stl/vector.h>

#include "lingtu/maps/layers/grid.hpp"

namespace nb = nanobind;
using namespace lingtu::maps::layers;

namespace lingtu_nav_kernel_bindings {

void bind_map_layers(nb::module_& m) {
  nb::class_<Grid2D>(m, "Grid2D")
    .def(nb::init<>())
    .def_rw("rows", &Grid2D::rows)
    .def_rw("cols", &Grid2D::cols)
    .def_rw("resolution", &Grid2D::resolution)
    .def_rw("origin_x", &Grid2D::originX)
    .def_rw("origin_y", &Grid2D::originY)
    .def_rw("data", &Grid2D::data)
    .def("empty", &Grid2D::empty);

  nb::class_<ElevationMapResult>(m, "ElevationMapResult")
    .def(nb::init<>())
    .def_rw("min_z", &ElevationMapResult::minZ)
    .def_rw("max_z", &ElevationMapResult::maxZ)
    .def_rw("clearance", &ElevationMapResult::clearance)
    .def_rw("valid", &ElevationMapResult::valid);

  nb::class_<EsdfResult>(m, "EsdfResult")
    .def(nb::init<>())
    .def_rw("distance", &EsdfResult::distance)
    .def_rw("grad_x", &EsdfResult::gradX)
    .def_rw("grad_y", &EsdfResult::gradY);

  nb::class_<TerrainRiskParams>(m, "TerrainRiskParams")
    .def(nb::init<>())
    .def_rw("max_slope_deg", &TerrainRiskParams::maxSlopeDeg)
    .def_rw("soft_slope_start_deg", &TerrainRiskParams::softSlopeStartDeg)
    .def_rw("critical_step_m", &TerrainRiskParams::criticalStepM)
    .def_rw("roughness_critical_m", &TerrainRiskParams::roughnessCriticalM);

  nb::class_<TerrainRiskResult>(m, "TerrainRiskResult")
    .def(nb::init<>())
    .def_rw("risk", &TerrainRiskResult::risk)
    .def_rw("slope_deg", &TerrainRiskResult::slopeDeg)
    .def_rw("step_height", &TerrainRiskResult::stepHeight)
    .def_rw("roughness", &TerrainRiskResult::roughness);

  nb::class_<TraversabilityParams>(m, "TraversabilityParams")
    .def(nb::init<>())
    .def_rw("lethal", &TraversabilityParams::lethal)
    .def_rw("inscribed", &TraversabilityParams::inscribed)
    .def_rw("max_slope_deg", &TraversabilityParams::maxSlopeDeg)
    .def_rw("soft_slope_start_deg", &TraversabilityParams::softSlopeStartDeg)
    .def_rw("safe_distance", &TraversabilityParams::safeDistance)
    .def_rw("proximity_cap", &TraversabilityParams::proximityCap);

  nb::class_<CmdVelCollisionParams>(m, "CmdVelCollisionParams")
    .def(nb::init<>())
    .def_rw("horizon_s", &CmdVelCollisionParams::horizonS)
    .def_rw("step_s", &CmdVelCollisionParams::stepS)
    .def_rw("stop_cost", &CmdVelCollisionParams::stopCost)
    .def_rw("slow_cost", &CmdVelCollisionParams::slowCost);

  nb::class_<CmdVelCollisionResult>(m, "CmdVelCollisionResult")
    .def(nb::init<>())
    .def_rw("action", &CmdVelCollisionResult::action)
    .def_rw("reason", &CmdVelCollisionResult::reason)
    .def_rw("max_cost", &CmdVelCollisionResult::maxCost);

  m.def("make_grid_2d", &makeGrid2D,
    nb::arg("rows"), nb::arg("cols"), nb::arg("resolution"),
    nb::arg("origin_x"), nb::arg("origin_y"), nb::arg("fill") = 0.0f);
  m.def("build_elevation_map", &buildElevationMap,
    nb::arg("xyz_flat"), nb::arg("robot_x"), nb::arg("robot_y"),
    nb::arg("resolution"), nb::arg("radius"), nb::arg("z_floor"), nb::arg("z_ceil"));
  m.def("compute_esdf", &computeEsdf,
    nb::arg("occupancy"), nb::arg("obstacle_threshold") = 50.0f);
  m.def("compute_terrain_risk", &computeTerrainRisk,
    nb::arg("elevation"), nb::arg("params") = TerrainRiskParams());
  m.def("fuse_traversability_cost", &fuseTraversabilityCost,
    nb::arg("costmap"), nb::arg("slope_deg"), nb::arg("esdf_distance"),
    nb::arg("terrain_risk"), nb::arg("params") = TraversabilityParams());
  m.def("project_cmd_vel_collision", &projectCmdVelCollision,
    nb::arg("costmap"), nb::arg("x"), nb::arg("y"), nb::arg("yaw"),
    nb::arg("vx"), nb::arg("vy"), nb::arg("wz"),
    nb::arg("params") = CmdVelCollisionParams());
}

}  // namespace lingtu_nav_kernel_bindings
