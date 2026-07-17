#include <nanobind/nanobind.h>

#include "bindings.hpp"

namespace nb = nanobind;

NB_MODULE(lingtu_explore_kernel, m) {
  m.doc() = "LingTu native exploration algorithms exposed through nanobind.";

  lingtu_explore_kernel_bindings::bind_types(m);
  lingtu_explore_kernel_bindings::bind_tare(m);
}
