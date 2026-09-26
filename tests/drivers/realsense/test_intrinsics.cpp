#include "intrinsics.hpp"
#include <stdexcept>
int main() {
  using lingtu::drivers::realsense::recordCompatibleIntrinsics;
  rs2_intrinsics intr{};
  intr.model = RS2_DISTORTION_INVERSE_BROWN_CONRADY;
  if (!recordCompatibleIntrinsics(intr)) throw std::runtime_error("zero inverse distortion rejected");
  intr.coeffs[2] = 0.01f;
  if (recordCompatibleIntrinsics(intr)) throw std::runtime_error("nonzero inverse distortion mislabeled");
  intr.model = RS2_DISTORTION_BROWN_CONRADY;
  if (!recordCompatibleIntrinsics(intr)) throw std::runtime_error("forward Brown rejected");
  intr = {};
  intr.model = RS2_DISTORTION_FTHETA;
  if (recordCompatibleIntrinsics(intr)) throw std::runtime_error("unsupported projection accepted");
}
