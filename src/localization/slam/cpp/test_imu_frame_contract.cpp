#include "imu_frame_contract.hpp"

#include <cassert>

int main() {
  using lingtu::slam::ImuFrameValidation;
  using lingtu::slam::imuFrameRejectionReason;
  using lingtu::slam::validateImuFrame;

  assert(validateImuFrame("imu_link") == ImuFrameValidation::kAccepted);
  assert(validateImuFrame(nullptr) == ImuFrameValidation::kMissing);
  assert(validateImuFrame("") == ImuFrameValidation::kMissing);
  assert(validateImuFrame("lidar_link") == ImuFrameValidation::kMismatch);
  assert(validateImuFrame("/imu_link") == ImuFrameValidation::kMismatch);
  assert(std::string_view(imuFrameRejectionReason(ImuFrameValidation::kMissing)) ==
         "imu_frame_missing");
  assert(std::string_view(imuFrameRejectionReason(ImuFrameValidation::kMismatch)) ==
         "imu_frame_mismatch");
  assert(std::string_view(imuFrameRejectionReason(ImuFrameValidation::kAccepted)).empty());
  return 0;
}
