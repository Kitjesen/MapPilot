#include <nanobind/nanobind.h>

#include "bindings.hpp"

namespace nb = nanobind;

NB_MODULE(lingtu_nav_kernel, m) {
  m.doc() = "LingTu native navigation algorithms exposed through nanobind.";

  lingtu_nav_kernel_bindings::bind_types(m);
  lingtu_nav_kernel_bindings::bind_map_layers(m);
  lingtu_nav_kernel_bindings::bind_path_follower(m);
  lingtu_nav_kernel_bindings::bind_waypoint_helpers(m);
  lingtu_nav_kernel_bindings::bind_local_planner(m);
  lingtu_nav_kernel_bindings::bind_terrain(m);
}
