#include "bindings.hpp"

#include <nanobind/stl/vector.h>

#include "nav_kernel/terrain_core.hpp"

namespace nb = nanobind;
using namespace nav_kernel;

namespace lingtu_nav_kernel_bindings {

void bind_terrain(nb::module_& m) {
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
      int n = static_cast<int>(cloud_flat.size()) / 4;
      return self.process(cloud_flat.data(), n, timestamp);
    }, nb::arg("cloud_xyzi"), nb::arg("timestamp"))
    .def("clear", &TerrainAnalysisCore::clear);
}

}  // namespace lingtu_nav_kernel_bindings
