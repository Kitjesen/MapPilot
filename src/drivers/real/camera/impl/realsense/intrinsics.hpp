#pragma once
#include <librealsense2/h/rs_types.h>

namespace lingtu::drivers::realsense {
inline bool recordCompatibleIntrinsics(const rs2_intrinsics& intr) {
  if (intr.model == RS2_DISTORTION_NONE || intr.model == RS2_DISTORTION_BROWN_CONRADY)
    return true;
  if (intr.model != RS2_DISTORTION_INVERSE_BROWN_CONRADY) return false;
  // D435i RGB can advertise inverse Brown with an identity distortion map.
  // Nonzero inverse coefficients cannot be relabeled as forward Brown.
  for (float coefficient : intr.coeffs)
    if (coefficient != 0.0f) return false;
  return true;
}
}
