#pragma once

#include <cstdint>

namespace lingtu::drivers::realsense {

class FrameSequence {
 public:
  bool isNew(std::uint64_t frame_number) noexcept {
    if (seen_ && frame_number == last_) return false;
    last_ = frame_number;
    seen_ = true;
    return true;
  }

 private:
  std::uint64_t last_{0};
  bool seen_{false};
};

}  // namespace lingtu::drivers::realsense
