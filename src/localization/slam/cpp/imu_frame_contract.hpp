#pragma once

#include <string_view>

namespace lingtu::slam {

inline constexpr std::string_view kImuFrameId{"imu_link"};

enum class ImuFrameValidation {
  kAccepted,
  kMissing,
  kMismatch,
};

inline ImuFrameValidation validateImuFrame(const char* frame_id) noexcept {
  if (frame_id == nullptr || frame_id[0] == '\0') {
    return ImuFrameValidation::kMissing;
  }
  return std::string_view(frame_id) == kImuFrameId
      ? ImuFrameValidation::kAccepted
      : ImuFrameValidation::kMismatch;
}

inline const char* imuFrameRejectionReason(ImuFrameValidation validation) noexcept {
  switch (validation) {
    case ImuFrameValidation::kMissing:
      return "imu_frame_missing";
    case ImuFrameValidation::kMismatch:
      return "imu_frame_mismatch";
    case ImuFrameValidation::kAccepted:
      return "";
  }
  return "imu_frame_mismatch";
}

}  // namespace lingtu::slam
