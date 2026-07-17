// CustomPoint.h shim — stub for standalone build
#ifndef LIVOX_CUSTOM_POINT_H
#define LIVOX_CUSTOM_POINT_H
#include <cstdint>

struct CustomPoint {
  float x = 0, y = 0, z = 0;
  uint8_t reflectivity = 0;
  uint8_t tag = 0;
  uint8_t line = 0;
  double offset_time = 0;
};

namespace livox_ros_driver {
  using ::CustomPoint;
}
#endif
