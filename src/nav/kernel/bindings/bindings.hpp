#pragma once

#include <nanobind/nanobind.h>

namespace lingtu_nav_kernel_bindings {

void bind_types(nanobind::module_& m);
void bind_map_layers(nanobind::module_& m);
void bind_path_follower(nanobind::module_& m);
void bind_waypoint_helpers(nanobind::module_& m);
void bind_local_planner(nanobind::module_& m);
void bind_terrain(nanobind::module_& m);

}  // namespace lingtu_nav_kernel_bindings
