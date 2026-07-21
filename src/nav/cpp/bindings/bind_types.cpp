#include "bindings.hpp"

#include <new>

#include "nav_kernel/types.hpp"

namespace nb = nanobind;
using namespace nav_kernel;

namespace lingtu_nav_kernel_bindings {

void bind_types(nb::module_& m) {
  nb::class_<Vec3>(m, "Vec3")
    .def(nb::init<>())
    .def("__init__", [](Vec3* v, double x, double y, double z) {
      new (v) Vec3{x, y, z};
    }, nb::arg("x") = 0.0, nb::arg("y") = 0.0, nb::arg("z") = 0.0)
    .def_rw("x", &Vec3::x)
    .def_rw("y", &Vec3::y)
    .def_rw("z", &Vec3::z);

  nb::class_<Pose>(m, "Pose")
    .def(nb::init<>())
    .def_rw("position", &Pose::position)
    .def_rw("yaw", &Pose::yaw);

  nb::class_<Twist>(m, "Twist")
    .def(nb::init<>())
    .def_rw("vx", &Twist::vx)
    .def_rw("vy", &Twist::vy)
    .def_rw("wz", &Twist::wz);

  m.def("normalize_angle", &normalizeAngle);
  m.def("distance_2d", &distance2D);
  m.def("distance_3d", &distance3D);
}

}  // namespace lingtu_nav_kernel_bindings
